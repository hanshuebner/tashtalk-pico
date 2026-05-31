//! Data CDC task — TT_DATA.
//!
//! Wires the host byte stream into `tashtalk-proto`'s `CommandParser`
//! (host → LocalTalk) and forwards every LocalTalk frame heard on the
//! wire back up to the host (LocalTalk → host), escape-encoded exactly
//! as the original TashTalk firmware does (`tashtalk/documentation/
//! protocol.md`). A host AppleTalk stack such as TailTalk's `tashtalk`
//! crate opens this port in place of `/dev/ttyUSB0`.
//!
//! Receive direction (the datapath this port previously lacked): the
//! continuous `lt_rx_task` decodes frames off the wire and hands the
//! complete frames to us over [`INBOUND_FRAMES`]; we escape-encode each
//! one and stream it up the IN endpoint, gated on DTR so we never block
//! on an endpoint with no reader.

use crate::csma::defer_and_backoff;
use crate::tt_log;
use crate::usb_dev::Cdc;
use crate::{MaybeLtDialogTx, MaybeLtTx, SharedBackoff, SharedDe, SharedNodes};
use core::cell::RefCell;
use core::sync::atomic::{AtomicU8, Ordering};
use defmt::warn;
use embassy_futures::select::{select, Either};
use embassy_rp::gpio::Level;
use embassy_rp::pac;
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, ThreadModeRawMutex};
use embassy_sync::blocking_mutex::Mutex as BlockingMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Instant, Timer};
use embassy_usb::driver::EndpointError;
use tashtalk_proto::codec::{
    encode_frame, CommandParser, FrameStatus, Sink, MAX_FRAME_LEN,
};
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

/// A complete LocalTalk frame received off the wire, on its way up to
/// the host. `lt_rx_task` produces these; `run()` escape-encodes and
/// streams them out TT_DATA.
pub struct InboundFrame {
    pub buf: [u8; MAX_FRAME_LEN],
    pub len: u16,
    pub status: FrameStatus,
}

/// LocalTalk → host frame queue. Depth 8 so a short burst (e.g. an
/// NBP lookup plus a couple of replies) doesn't drop frames if the USB
/// IN endpoint is momentarily behind.
pub static INBOUND_FRAMES: Channel<ThreadModeRawMutex, InboundFrame, 8> = Channel::new();


/// Active feature bits, as set by the host's `0x03 SetFeatures` command.
/// Bit 7 = CRC calculation (we ignore it — well-behaved hosts, TailTalk
/// included, compute the CRC themselves and hand us correct bytes).
/// Bit 6 = CRC checking (we honour it: when set, completed frames are
/// reported up with `0x00 0xFD` for a good CRC and `0x00 0xFC` for a bad
/// one; when clear, every structurally-complete frame is reported `0xFD`
/// and the host validates the CRC itself).
pub static FEATURES: AtomicU8 = AtomicU8::new(0);

/// True if the host enabled the CRC-checking feature (bit 6).
pub fn crc_check_enabled() -> bool {
    FEATURES.load(Ordering::Relaxed) & 0x40 != 0
}

// ── Self-transmit echo suppression ───────────────────────────────────────────
//
// With a half-duplex RS-422/485 transceiver (or the bring-up GP2↔GP4
// loopback) the receiver hears everything this device transmits. The
// original TashTalk forwards every bus frame up to the host *except the
// ones it transmits itself*; we reproduce that by recording a short
// signature of each transmitted frame and dropping the matching frame
// when the continuous receiver decodes it back. Auto-replies (ACK/CTS)
// go through the same TX choke point, so they are suppressed too.

/// How long a recorded TX signature stays eligible to match an echo.
/// One DMA buffer is ~1.1 ms; 250 ms is comfortably longer than the
/// wire-to-decode latency while short enough that a genuine, identical
/// frame from a peer much later is not falsely suppressed.
const ECHO_TTL: Duration = Duration::from_millis(250);
const ECHO_SLOTS: usize = 8;

struct EchoRing {
    /// `(signature, recorded_at)` for each live slot.
    slots: [Option<(u32, Instant)>; ECHO_SLOTS],
    next: usize,
}

static ECHO_RING: BlockingMutex<CriticalSectionRawMutex, RefCell<EchoRing>> =
    BlockingMutex::new(RefCell::new(EchoRing {
        slots: [None; ECHO_SLOTS],
        next: 0,
    }));

/// Signature of a frame: its length plus its trailing two bytes (the
/// CRC for a well-formed frame). Two distinct frames sharing this are
/// almost certainly byte-identical, so suppressing on a match is safe.
fn frame_sig(frame: &[u8]) -> u32 {
    let len = frame.len();
    let lo = if len >= 1 { frame[len - 1] as u32 } else { 0 };
    let hi = if len >= 2 { frame[len - 2] as u32 } else { 0 };
    ((len as u32 & 0xFFFF) << 16) | (hi << 8) | lo
}

/// Record that we just put `frame` on the wire, so its echo is dropped.
pub fn record_tx(frame: &[u8]) {
    let sig = frame_sig(frame);
    let now = Instant::now();
    ECHO_RING.lock(|r| {
        let mut r = r.borrow_mut();
        let slot = r.next;
        r.slots[slot] = Some((sig, now));
        r.next = (slot + 1) % ECHO_SLOTS;
    });
}

/// If `frame` matches a still-live recorded transmission, consume that
/// record and return true (caller should drop the frame as a self-echo).
pub fn take_self_echo(frame: &[u8]) -> bool {
    let sig = frame_sig(frame);
    ECHO_RING.lock(|r| {
        let mut r = r.borrow_mut();
        for slot in r.slots.iter_mut() {
            if let Some((s, when)) = *slot {
                if when.elapsed() > ECHO_TTL {
                    *slot = None;
                } else if s == sig {
                    *slot = None;
                    return true;
                }
            }
        }
        false
    })
}

/// Queue a received LocalTalk frame for delivery up TT_DATA. Non-blocking;
/// if the host is slow and the queue is full the frame is dropped (the
/// AppleTalk layer above tolerates loss — LocalTalk itself is lossy).
pub fn try_forward_inbound(frame: &[u8], status: FrameStatus) {
    let n = frame.len().min(MAX_FRAME_LEN);
    let mut f = InboundFrame {
        buf: [0u8; MAX_FRAME_LEN],
        len: n as u16,
        status,
    };
    f.buf[..n].copy_from_slice(&frame[..n]);
    if INBOUND_FRAMES.try_send(f).is_err() {
        tt_log!("LT_RX: host queue full, dropped frame");
    }
}

#[embassy_executor::task]
pub async fn run(
    mut cdc: Cdc,
    lt_tx: &'static MaybeLtTx,
    lt_dialog_tx: &'static MaybeLtDialogTx,
    lt_de: &'static SharedDe,
    nodes: &'static SharedNodes,
    backoff: &'static SharedBackoff,
) -> ! {
    let mut buf = [0u8; 64];
    let mut parser = CommandParser::new();
    let mut sink = LogSink::default();

    loop {
        cdc.wait_connection().await;
        info_connected();
        parser.reset();

        loop {
            match select(cdc.read_packet(&mut buf), INBOUND_FRAMES.receive()).await {
                // ── Host → LocalTalk: command bytes from the host. ──
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
                        transmit_on_phy(lt_tx, lt_dialog_tx, lt_de, backoff, frame).await;
                    }
                }
                Either::First(Err(e)) => {
                    warn!("TT_DATA read failed: {}", classify(e));
                    tt_log!("TT_DATA: read err");
                    break;
                }
                // ── LocalTalk → host: a frame heard on the wire. ──
                Either::Second(inbound) => {
                    // Only stream up if a host application has the port
                    // open (DTR asserted); writing to an IN endpoint with
                    // no reader would hang this task. If no reader, drop.
                    if cdc.dtr() {
                        let encoded =
                            encode_frame(&inbound.buf[..inbound.len as usize], inbound.status);
                        if let Err(e) = write_chunked(&mut cdc, &encoded).await {
                            warn!("TT_DATA write failed: {}", classify(e));
                            tt_log!("TT_DATA: inbound tx failed");
                            break;
                        }
                    }
                }
            }
        }
    }
}

fn info_connected() {
    defmt::info!("TT_DATA: host connected");
    tt_log!("TT_DATA: host connected");
}

/// Drive one host-originated frame out the LocalTalk PHY after CSMA.
///
/// Directed data frames use the hardware-driven RTS/CTS dialog
/// (RTS → CTS → DATA). Broadcast and control frames are sent
/// directly (no RTS). On CTS timeout, falls back to direct TX.
pub async fn transmit_on_phy(
    lt_tx: &'static MaybeLtTx,
    lt_dialog_tx: &'static MaybeLtDialogTx,
    lt_de: &'static SharedDe,
    backoff: &'static SharedBackoff,
    frame: &[u8],
) {
    // Dialog disabled: the Mac does not respond to our RTS with CTS
    // (unknown reason — see HANDOFF.md). Sending RTS without completing
    // the CTS handshake is worse than no RTS: the Mac enters a "dialog
    // pending" state and discards our fallback direct TX. SM2 infra is
    // in place and ready; flip this once the CTS issue is resolved.
    let is_directed_data = false;

    if is_directed_data {
        if transmit_directed_dialog(lt_tx, lt_dialog_tx, lt_de, backoff, frame).await {
            return; // dialog succeeded
        }
        // Fall back to direct TX (RTS-less) — slow but works.
        tt_log!("HOST: dialog failed, fallback to direct TX");
    }
    defer_and_backoff(backoff).await;
    do_inner_tx(lt_tx, lt_de, frame).await;
}

/// Hardware-driven RTS/CTS/DATA originator dialog for directed data.
/// Uses SM2 (via `lt_dialog_tx`) for the gated data TX, keeping SM0
/// clean for regular `transmit_frame` calls.
async fn transmit_directed_dialog(
    lt_tx: &'static MaybeLtTx,
    lt_dialog_tx: &'static MaybeLtDialogTx,
    lt_de: &'static SharedDe,
    backoff: &'static SharedBackoff,
    frame: &[u8],
) -> bool {
    use crate::bitstream::BitStream;
    use crate::csma::{set_pending_rts, clear_pending_rts, CTS_RECEIVED};
    use crate::dialog_tx;
    use tashtalk_proto::llap::compose_rts;

    let dest = frame[0];
    let src = frame[1];

    // Pre-encode the DATA frame into the static DMA buffer.
    let stream = BitStream::build_frame(frame);
    dialog_tx::load_data(&stream);

    let rts = compose_rts(dest, src);

    // CSMA defer before RTS.
    defer_and_backoff(backoff).await;

    // Send RTS via SM0 (normal TX path). SM0 ends disabled.
    do_inner_tx(lt_tx, lt_de, &rts).await;

    // Record DATA frame for echo suppression.
    record_tx(frame);

    // Clear any stale PIO IRQ 4 from a previous dialog.
    pac::PIO0.irq().write(|w| w.set_irq(1 << 4));

    // Arm SM2 for dialog TX (gated on PIO IRQ 4).
    {
        let mut dtx_guard = lt_dialog_tx.lock().await;
        if let Some(dtx) = dtx_guard.as_mut() {
            dtx.arm_dialog();
        }
    }

    // Arm DMA → SM2's TX FIFO. SM2 is gated on IRQ 4.
    dialog_tx::arm_dma();

    // Set the pending-RTS match for the RX task's CTS detector.
    set_pending_rts(src, dest);

    // Wait for CTS (2 ms timeout).
    match embassy_time::with_timeout(
        Duration::from_millis(2),
        CTS_RECEIVED.wait(),
    )
    .await
    {
        Ok(()) => {
            // CTS received → trigger() asserted DE and fired IRQ 4 →
            // SM2 is encoding the DMA data right now.
            {
                let mut dtx_guard = lt_dialog_tx.lock().await;
                if let Some(dtx) = dtx_guard.as_mut() {
                    let mut iters = 0u32;
                    while dialog_tx::is_dma_busy() {
                        Timer::after_micros(50).await;
                        iters += 1;
                        if iters > 1000 { break; }
                    }
                    dtx.wait_tx_drain().await;
                }
            }
            dialog_tx::deassert_de();
            tt_log!("HOST: dialog TX done");
            true
        }
        Err(_timeout) => {
            clear_pending_rts();
            dialog_tx::abort();
            {
                let mut dtx_guard = lt_dialog_tx.lock().await;
                if let Some(dtx) = dtx_guard.as_mut() {
                    dtx.disarm_dialog();
                }
            }
            dialog_tx::deassert_de();
            false
        }
    }
}

/// The inner "raise DE, drive PIO, drop DE" sequence — no CSMA logic.
/// Used by both the host-frame path and the autonomous reply path. Every
/// transmitted frame is recorded for self-echo suppression so the
/// continuous receiver doesn't forward our own transmission back to the
/// host.
pub async fn do_inner_tx(
    lt_tx: &'static MaybeLtTx,
    lt_de: &'static SharedDe,
    frame: &[u8],
) {
    record_tx(frame);
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
        FEATURES.store(features, Ordering::Relaxed);
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

fn classify(e: EndpointError) -> defmt::Str {
    match e {
        EndpointError::BufferOverflow => defmt::intern!("buffer overflow"),
        EndpointError::Disabled => defmt::intern!("endpoint disabled"),
    }
}
