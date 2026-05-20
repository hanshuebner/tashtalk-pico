//! Data CDC task — TT_DATA.
//!
//! Wires the host byte stream into `tashtalk-proto`'s `CommandParser`,
//! and emits periodic *synthetic* LocalTalk frames up to the host so the
//! device→host escape encoder gets exercised even before the PIO RX path
//! exists. Bring-up step 5 in PICO_PORT_PLAN.md.
//!
//! Every 2 seconds while the host is connected, the firmware crafts a
//! known LLAP data frame containing a literal `0x00` byte (to force the
//! `0x00 0xFF` escape), computes the CRC, and pushes it through TT_DATA's
//! IN endpoint terminated by `0x00 0xFD`. A host-side decoder (e.g.
//! TailTalk's `TashTalkCodec`, or the small Python helper at
//! `tools/synth_rx.py`) recovers the original bytes and verifies the CRC.

use crate::csma::defer_and_backoff;
use crate::tt_log;
use crate::usb_dev::Cdc;
use crate::{MaybeLtTx, SharedBackoff, SharedDe, SharedNodes};
use defmt::{info, warn};
use embassy_futures::select::{select, Either};
use embassy_rp::gpio::Level;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Ticker, Timer};
use embassy_usb::driver::EndpointError;
use tashtalk_proto::codec::{
    encode_frame, CommandParser, FrameStatus, Sink, MAX_FRAME_LEN,
};
use tashtalk_proto::crc::lt_crc;
use tashtalk_proto::nodemap::NodeBitmap;

/// One-shot SetNodeIds events: parser pushes a new bitmap onto this
/// channel; the task that owns the shared NodeBitmap drains it. We use
/// a channel rather than directly locking the SharedNodes from inside
/// the parser sink because `Sink` callbacks are synchronous and
/// `Mutex::lock` is async.
pub static SET_NODES: Channel<ThreadModeRawMutex, [u8; 32], 4> = Channel::new();

/// Outbound LocalTalk frames queued by the host command parser.
/// `(byte_count, buffer_with_byte_count_valid_bytes)`. The run() task
/// drains this and invokes transmit_on_phy.
pub static OUTBOUND_FRAMES: Channel<ThreadModeRawMutex, ([u8; 610], u16), 4> =
    Channel::new();

/// Period between synthetic device→host frames.
const SYNTHETIC_PERIOD: Duration = Duration::from_secs(2);

/// Compose the synthetic test frame.
///
/// LLAP layout (data frame, type byte's high bit clear):
///   `[dest=0x42, src=0x01, type=0x01, len_hi=0x00, len_lo=0x05,`
///   ` body=[0xDE, 0x00, 0xAD], crc1, crc2]`
///
/// length = 5 means "5 bytes including the 2-byte length field but
/// excluding the 3-byte LLAP header and the 2 CRC bytes". So the body
/// after the length field is 5-2 = 3 bytes, total wire payload 10 bytes.
fn synthetic_frame_bytes() -> [u8; 10] {
    let mut f = [0u8; 10];
    f[0] = 0x42; // dest — should match a node ID in the bitmap to be "received"
    f[1] = 0x01; // src
    f[2] = 0x01; // LLAP type (data, low bit set, high bit clear)
    f[3] = 0x00; // length high
    f[4] = 0x05; // length low (body = 3, length field counts itself)
    f[5] = 0xDE;
    f[6] = 0x00; // ← literal 0x00 forces an escape during encoding
    f[7] = 0xAD;
    let crc = lt_crc(&f[..8]);
    f[8] = crc[0];
    f[9] = crc[1];
    f
}

#[embassy_executor::task]
pub async fn run(
    mut cdc: Cdc,
    lt_tx: &'static MaybeLtTx,
    lt_de: &'static SharedDe,
    nodes: &'static SharedNodes,
    backoff: &'static SharedBackoff,
) -> ! {
    let mut buf = [0u8; 64];
    let mut parser = CommandParser::new();
    let mut sink = LogSink::default();

    loop {
        cdc.wait_connection().await;
        info!("TT_DATA: host connected");
        tt_log!("TT_DATA: host connected");
        parser.reset();

        let mut tick = Ticker::every(SYNTHETIC_PERIOD);
        let mut frame_no: u32 = 0;

        loop {
            match select(cdc.read_packet(&mut buf), tick.next()).await {
                Either::First(Ok(n)) => {
                    tt_log!("TT_DATA: rx {} bytes", n);
                    for &byte in &buf[..n] {
                        parser.push(byte, &mut sink);
                    }
                    // Drain any SetNodeIds events the parser pushed.
                    while let Ok(new_bitmap_bytes) = SET_NODES.try_receive() {
                        let mut g = nodes.lock().await;
                        *g = NodeBitmap::from_bytes(new_bitmap_bytes);
                    }
                    // Drain any outbound frames queued by the parser
                    // and transmit them over the LocalTalk PHY.
                    while let Ok((frame_buf, frame_len)) = OUTBOUND_FRAMES.try_receive() {
                        let frame = &frame_buf[..frame_len as usize];
                        tt_log!("HOST: PIO TX {} bytes", frame.len());
                        transmit_on_phy(lt_tx, lt_de, backoff, frame).await;
                    }
                }
                Either::First(Err(e)) => {
                    warn!("TT_DATA read failed: {}", classify(e));
                    tt_log!("TT_DATA: read err");
                    break;
                }
                Either::Second(()) => {
                    frame_no = frame_no.wrapping_add(1);
                    let raw = synthetic_frame_bytes();

                    // Ticker PIO TX is OFF for now while debugging the ENQ→ACK
                    // autoresponder — it floods the RX with data frames every
                    // 2s that obscure the targeted ENQ→ACK traces.
                    let _ = (lt_tx, lt_de, backoff);
                    if false {
                        transmit_on_phy(lt_tx, lt_de, backoff, &raw).await;
                    }

                    // Also stream the frame up TT_DATA if a host
                    // application has the port open (DTR asserted).
                    if cdc.dtr() {
                        let encoded = encode_frame(&raw, FrameStatus::Good);
                        if let Err(e) = write_chunked(&mut cdc, &encoded).await {
                            warn!("TT_DATA write failed: {}", classify(e));
                            tt_log!("TT_DATA: synthetic tx failed");
                            break;
                        }
                        tt_log!(
                            "tx frame #{}: {} raw -> {} on-wire (CDC + PIO)",
                            frame_no,
                            raw.len(),
                            encoded.len()
                        );
                    }
                }
            }
        }
    }
}

/// Drive one frame out the LocalTalk PHY with CSMA/CA defer-and-backoff.
///
/// LLAP's per-frame RTS/CTS pairing for unicast data frames is NOT
/// performed here. The PIC reference firmware did it inside the IOC
/// ISR with sub-microsecond response time; with our DMA-buffered RX
/// path, the round-trip latency between "an RTS hits the wire" and
/// "we have a fresh CTS on the wire" is ~2-3 ms — far past LLAP's
/// 200 µs CTS-timeout. Until the architecture can guarantee that
/// timing (e.g. via smaller DMA buffers or interrupt-driven decode),
/// host-side software that needs the dance must issue the RTS and
/// data frame as separate `0x01` commands.
pub async fn transmit_on_phy(
    lt_tx: &'static MaybeLtTx,
    lt_de: &'static SharedDe,
    backoff: &'static SharedBackoff,
    frame: &[u8],
) {
    defer_and_backoff(backoff).await;
    do_inner_tx(lt_tx, lt_de, frame).await;
}

/// The inner "raise DE, drive PIO, drop DE" sequence — no CSMA logic.
/// Used by both the control-frame fast path and each step of the data
/// frame's RTS/CTS dance.
pub async fn do_inner_tx(
    lt_tx: &'static MaybeLtTx,
    lt_de: &'static SharedDe,
    frame: &[u8],
) {
    let mut tx_guard = lt_tx.lock().await;
    let Some(tx) = tx_guard.as_mut() else {
        tt_log!("PHY tx skipped — LtTx not initialised");
        return;
    };
    let mut de = lt_de.lock().await;

    de.set_level(Level::High);
    Timer::after_micros(5).await;
    tx.transmit_frame(frame).await;
    Timer::after_micros(5).await;
    de.set_level(Level::Low);
}

/// Transmit an autonomous reply (ACK or CTS) bypassing the CSMA defer
/// and backoff. The PIC firmware responds to ENQ / RTS immediately
/// from inside the IOC ISR, with no random delay — we mirror that
/// here. Bypassing defer is crucial: the wire is just-busy from the
/// incoming frame, so a full IDG + random backoff would push the
/// reply far past the originator's CTS timeout, causing cascading
/// retries.
pub async fn transmit_autoreply(
    lt_tx: &'static MaybeLtTx,
    lt_de: &'static SharedDe,
    frame: &[u8],
) {
    do_inner_tx(lt_tx, lt_de, frame).await;
}

/// Write a byte slice over the CDC, splitting on the 64-byte max packet
/// size. For payloads that are an exact multiple of 64 bytes, embassy-usb
/// handles the zero-length-packet flush automatically when we drop below
/// the boundary in the next call.
async fn write_chunked(cdc: &mut Cdc, data: &[u8]) -> Result<(), EndpointError> {
    for chunk in data.chunks(64) {
        cdc.write_packet(chunk).await?;
    }
    Ok(())
}

struct LogSink {
    noop_run: u16,
    frame_bytes_seen: u16,
    current_frame: [u8; 610],
    current_frame_len: u16,
}

impl Default for LogSink {
    fn default() -> Self {
        Self {
            noop_run: 0,
            frame_bytes_seen: 0,
            current_frame: [0u8; 610],
            current_frame_len: 0,
        }
    }
}

impl LogSink {
    fn flush_noops(&mut self) {
        if self.noop_run > 0 {
            tt_log!("HOST: {} noop bytes", self.noop_run);
            self.noop_run = 0;
        }
    }
}

impl Sink for LogSink {
    fn on_noop(&mut self) {
        self.noop_run = self.noop_run.saturating_add(1);
    }

    fn on_frame_header(&mut self, dest: u8, src: u8, llap_type: u8) {
        self.flush_noops();
        self.frame_bytes_seen = 0;
        self.current_frame_len = 0;
        let kind = if llap_type & 0x80 != 0 { "ctrl" } else { "data" };
        tt_log!(
            "HOST: tx frame [{}] dest={:02x} src={:02x} type={:02x}",
            kind,
            dest,
            src,
            llap_type
        );
    }

    fn on_frame_byte(&mut self, byte: u8) {
        self.frame_bytes_seen = self.frame_bytes_seen.saturating_add(1);
        if (self.current_frame_len as usize) < self.current_frame.len() {
            self.current_frame[self.current_frame_len as usize] = byte;
            self.current_frame_len += 1;
        }
    }

    fn on_frame_end(&mut self) {
        tt_log!(
            "HOST: tx frame end ({} bytes total)",
            self.frame_bytes_seen
        );
        let _ = OUTBOUND_FRAMES.try_send((self.current_frame, self.current_frame_len));
        self.current_frame_len = 0;
    }

    fn on_set_node_ids(&mut self, bitmap: &[u8; 32]) {
        self.flush_noops();
        let count: u32 = bitmap.iter().map(|b| b.count_ones()).sum();
        let first = (0u16..=255)
            .find(|n| bitmap[(n >> 3) as usize] & (1 << (n & 7)) != 0)
            .map(|n| n as i32)
            .unwrap_or(-1);
        tt_log!("HOST: SetNodeIds ({} ids, first={})", count, first);
        // Push the bitmap onto the SET_NODES channel; the run() task
        // drains it and updates the shared NodeBitmap (async-locked).
        let _ = SET_NODES.try_send(*bitmap);
    }

    fn on_set_features(&mut self, features: u8) {
        self.flush_noops();
        let calc = if features & 0x80 != 0 { "Y" } else { "n" };
        let check = if features & 0x40 != 0 { "Y" } else { "n" };
        tt_log!(
            "HOST: SetFeatures crc_calc={} crc_check={} (raw {:02x})",
            calc,
            check,
            features
        );
    }
}

// Touch MAX_FRAME_LEN so the import is "used" — keeps the encoder dependency
// visible and clippy/dead-code happy if encode_frame's type bound changes.
#[allow(dead_code)]
const _: usize = MAX_FRAME_LEN;

fn classify(e: EndpointError) -> defmt::Str {
    match e {
        EndpointError::BufferOverflow => defmt::intern!("buffer overflow"),
        EndpointError::Disabled => defmt::intern!("endpoint disabled"),
    }
}
