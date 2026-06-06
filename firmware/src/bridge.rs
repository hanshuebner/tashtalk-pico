//! LToUDP bridge — bidirectional LocalTalk ↔ UDP tunnel.
//!
//! Wire → UDP (egress): good-CRC frames from the wire are stripped of FCS,
//! filtered (no RTS/CTS), prepended with a sender ID, and sent as UDP
//! multicast to 239.192.76.84:1954.
//!
//! UDP → Wire (ingress): received UDP packets are stripped of sender ID,
//! checked for echo, CRC-appended, and submitted to the TX path for
//! CSMA transmission on the physical bus.
//!
//! See `docs/ltoudp-spec.md` for the protocol specification.

use portable_atomic::{AtomicBool, AtomicU32, Ordering};

use defmt::{info, warn};
use embassy_net::udp::{PacketMetadata, UdpSocket};
use embassy_net::{IpAddress, Ipv4Address};

use tashtalk_proto::codec::MAX_FRAME_LEN;
use tashtalk_proto::crc::lt_crc;

use crate::core1::SharedState;

// ── Constants ──────────────────────────────────────────────────────

const LTOUDP_PORT: u16 = 1954;
const LTOUDP_GROUP: Ipv4Address = Ipv4Address::new(239, 192, 76, 84);
/// Opaque sender ID embedded in every outgoing packet.
/// 0 is fine for a single bare-metal speaker (per spec).
const SENDER_ID: u32 = 0;

// ── Counters ───────────────────────────────────────────────────────

static EGRESS_COUNT: AtomicU32 = AtomicU32::new(0);
static INGRESS_COUNT: AtomicU32 = AtomicU32::new(0);
static DROP_COUNT: AtomicU32 = AtomicU32::new(0);
static ENABLED: AtomicBool = AtomicBool::new(true);

pub fn egress_count() -> u32 {
    EGRESS_COUNT.load(Ordering::Relaxed)
}
pub fn ingress_count() -> u32 {
    INGRESS_COUNT.load(Ordering::Relaxed)
}
pub fn drop_count() -> u32 {
    DROP_COUNT.load(Ordering::Relaxed)
}
pub fn is_enabled() -> bool {
    ENABLED.load(Ordering::Relaxed)
}
pub fn set_enabled(en: bool) {
    ENABLED.store(en, Ordering::Relaxed);
}
pub fn clear_counters() {
    EGRESS_COUNT.store(0, Ordering::Relaxed);
    INGRESS_COUNT.store(0, Ordering::Relaxed);
    DROP_COUNT.store(0, Ordering::Relaxed);
}

// ── Egress channel (wire → UDP) ────────────────────────────────────

/// Maximum LLAP packet size without FCS: dest(1) + src(1) + type(1) + payload(≤600) = 603.
const MAX_PACKET_LEN: usize = MAX_FRAME_LEN - 2;

struct EgressFrame {
    buf: [u8; MAX_PACKET_LEN],
    len: u16,
}

static EGRESS_CHANNEL: embassy_sync::channel::Channel<
    embassy_sync::blocking_mutex::raw::ThreadModeRawMutex,
    EgressFrame,
    4,
> = embassy_sync::channel::Channel::new();

/// Called from rx_bridge_task for every good-CRC frame.
/// Filters and enqueues for UDP transmission.
pub fn try_forward_to_udp(frame: &[u8], crc_ok: bool) {
    if !ENABLED.load(Ordering::Relaxed) || !crc_ok {
        return;
    }
    // Need at least dest + src + type + 2 CRC bytes.
    if frame.len() < 5 {
        return;
    }
    let frame_type = frame[2];
    // RTS (0x84) and CTS (0x85) must never cross the bridge.
    if frame_type == 0x84 || frame_type == 0x85 {
        return;
    }
    // Strip the trailing 2-byte FCS.
    let packet = &frame[..frame.len() - 2];
    if packet.len() > MAX_PACKET_LEN {
        return;
    }
    let mut ef = EgressFrame {
        buf: [0u8; MAX_PACKET_LEN],
        len: packet.len() as u16,
    };
    ef.buf[..packet.len()].copy_from_slice(packet);
    if EGRESS_CHANNEL.try_send(ef).is_err() {
        DROP_COUNT.fetch_add(1, Ordering::Relaxed);
    }
}

// ── Bridge task ────────────────────────────────────────────────────

#[embassy_executor::task]
pub async fn bridge_task(shared: &'static SharedState) {
    // Wait for network stack to be available.
    loop {
        if let Some(stack) = crate::wifi::stack() {
            // Wait for an IP address (DHCP complete).
            loop {
                if stack.config_v4().is_some() {
                    break;
                }
                embassy_time::Timer::after_millis(500).await;
            }

            // Join multicast group.
            if let Err(_e) = stack.join_multicast_group(IpAddress::Ipv4(LTOUDP_GROUP)) {
                warn!("bridge: failed to join multicast group");
            }
            info!("bridge: started");

            // Create UDP socket.
            let mut rx_meta = [PacketMetadata::EMPTY; 4];
            let mut rx_buf = [0u8; 700];
            let mut tx_meta = [PacketMetadata::EMPTY; 4];
            let mut tx_buf = [0u8; 700];
            let mut socket = UdpSocket::new(
                *stack,
                &mut rx_meta,
                &mut rx_buf,
                &mut tx_meta,
                &mut tx_buf,
            );
            socket.bind(LTOUDP_PORT).unwrap();

            let endpoint = (IpAddress::Ipv4(LTOUDP_GROUP), LTOUDP_PORT);

            // Main bridge loop: select between egress and ingress.
            loop {
                use embassy_futures::select::{select, Either};

                match select(
                    EGRESS_CHANNEL.receive(),
                    recv_ingress(&mut socket, shared),
                )
                .await
                {
                    Either::First(ef) => {
                        // Wire → UDP: prepend sender_id + send.
                        let pkt_len = ef.len as usize;
                        let mut udp_buf = [0u8; 4 + MAX_PACKET_LEN];
                        udp_buf[..4].copy_from_slice(&SENDER_ID.to_le_bytes());
                        udp_buf[4..4 + pkt_len].copy_from_slice(&ef.buf[..pkt_len]);
                        if socket.send_to(&udp_buf[..4 + pkt_len], endpoint).await.is_ok() {
                            EGRESS_COUNT.fetch_add(1, Ordering::Relaxed);
                        }
                    }
                    Either::Second(()) => {
                        // Ingress handled inside recv_ingress.
                    }
                }
            }
        }
        embassy_time::Timer::after_secs(1).await;
    }
}

/// Receive one UDP packet and inject onto the wire.
async fn recv_ingress(socket: &mut UdpSocket<'_>, shared: &'static SharedState) {
    let mut pkt_buf = [0u8; 4 + MAX_FRAME_LEN];

    socket
        .recv_from_with(|data, _meta| {
            if data.len() < 7 {
                // Too short: need 4 sender_id + at least dest+src+type.
                return;
            }
            let sender_id = u32::from_le_bytes([data[0], data[1], data[2], data[3]]);
            if sender_id == SENDER_ID {
                // Echo of our own packet — drop.
                return;
            }
            let packet = &data[4..];
            if packet.len() > MAX_PACKET_LEN {
                return;
            }
            // Build wire frame: packet + CRC.
            let crc = lt_crc(packet);
            let frame_len = packet.len() + 2;
            pkt_buf[..packet.len()].copy_from_slice(packet);
            pkt_buf[packet.len()] = crc[0];
            pkt_buf[packet.len() + 1] = crc[1];

            // Submit to TX (blocking try — cooperative executor serializes).
            // We can't await inside recv_from_with, so use a sync path.
            // Store frame for async submission after callback returns.
            unsafe {
                INGRESS_FRAME_LEN = frame_len as u16;
                INGRESS_FRAME_BUF[..frame_len].copy_from_slice(&pkt_buf[..frame_len]);
            }
        })
        .await;

    // Submit the frame outside the callback (can await here).
    let len = unsafe { INGRESS_FRAME_LEN } as usize;
    if len > 0 {
        unsafe { INGRESS_FRAME_LEN = 0 };
        let frame = unsafe { &INGRESS_FRAME_BUF[..len] };
        crate::host::submit_tx_from_debug(shared, frame).await;
        INGRESS_COUNT.fetch_add(1, Ordering::Relaxed);
    }
}

static mut INGRESS_FRAME_BUF: [u8; MAX_FRAME_LEN] = [0u8; MAX_FRAME_LEN];
static mut INGRESS_FRAME_LEN: u16 = 0;
