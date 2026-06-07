//! Data CDC task — USB host.
//!
//! Wires the host byte stream into `tashtalk-proto`'s `CommandParser`
//! (host → LocalTalk) and forwards every LocalTalk frame heard on the
//! wire back up to the host (LocalTalk → host), escape-encoded exactly
//! as the original TashTalk firmware does.
//!
//! TX frames are pre-encoded (HDLC + FM-0 + DMA word packing) on core 0
//! then submitted to core 1 via the shared TxSlot + TX_SUBMIT flag.

use crate::core1::{self, SharedState};
use crate::tt_log;
use crate::usb_dev::Cdc;
use core::sync::atomic::{compiler_fence, Ordering};
use defmt::warn;
use embassy_futures::select::{select, Either};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::Timer;
use embassy_usb::driver::EndpointError;
use portable_atomic::AtomicBool;
use tashtalk_proto::codec::{
    encode_frame, CommandParser, FrameStatus, Sink, MAX_FRAME_LEN,
};
use tashtalk_proto::hdlc::{build_tx_buffer_into, Fm0Buffer, HdlcBits};
use tashtalk_proto::llap::compose_rts;

/// A complete LocalTalk frame received off the wire.
pub struct InboundFrame {
    pub buf: [u8; MAX_FRAME_LEN],
    pub len: u16,
    pub status: FrameStatus,
}

/// LocalTalk → host frame queue.
pub static INBOUND_FRAMES: Channel<ThreadModeRawMutex, InboundFrame, 8> = Channel::new();

/// Whether a USB host is connected to USB host.
static HOST_CONNECTED: AtomicBool = AtomicBool::new(false);

/// Queue a received LocalTalk frame for delivery up USB host.
pub fn try_forward_inbound(frame: &[u8], status: FrameStatus) {
    if !HOST_CONNECTED.load(Ordering::Relaxed) {
        // No host connected — silently drop, don't queue.
        return;
    }
    let n = frame.len().min(MAX_FRAME_LEN);
    let mut f = InboundFrame {
        buf: [0u8; MAX_FRAME_LEN],
        len: n as u16,
        status,
    };
    f.buf[..n].copy_from_slice(&frame[..n]);
    if INBOUND_FRAMES.try_send(f).is_err() {
        tt_log!("USB host: dropped frame (queue full)");
    }
}

#[embassy_executor::task]
pub async fn run(
    mut cdc: Cdc,
    shared: &'static SharedState,
) -> ! {
    let mut buf = [0u8; 64];
    let mut parser = CommandParser::new();
    let mut sink = HostSink { shared };

    loop {
        // Wait for a stable connection (see debug.rs for rationale).
        loop {
            cdc.wait_connection().await;
            embassy_time::Timer::after_millis(200).await;
            if cdc.dtr() {
                break;
            }
        }
        defmt::info!("USB host: host connected");
        tt_log!("USB host: connected");
        HOST_CONNECTED.store(true, Ordering::Relaxed);
        parser.reset();

        loop {
            // Drain any pending outbound frames from the sink.
            sink.drain_outbound().await;

            match select(cdc.read_packet(&mut buf), INBOUND_FRAMES.receive()).await {
                Either::First(Ok(n)) => {
                    for &byte in &buf[..n] {
                        parser.push(byte, &mut sink);
                    }
                    sink.drain_outbound().await;
                }
                Either::First(Err(e)) => {
                    warn!("USB host read failed: {}", classify(e));
                    break;
                }
                Either::Second(inbound) => {
                    if cdc.dtr() {
                        let encoded =
                            encode_frame(&inbound.buf[..inbound.len as usize], inbound.status);
                        if let Err(e) = write_chunked(&mut cdc, &encoded).await {
                            warn!("USB host write failed: {}", classify(e));
                            break;
                        }
                    } else {
                        // Host dropped DTR — disconnect.
                        break;
                    }
                }
            }
        }
        HOST_CONNECTED.store(false, Ordering::Relaxed);
        defmt::info!("USB host: host disconnected");
        tt_log!("USB host: disconnected");
    }
}

/// Pre-encode a frame and submit to core 1 for CSMA TX (for debug task).
pub async fn submit_tx_from_debug(shared: &'static SharedState, frame: &[u8]) {
    submit_tx(shared, frame).await;
}

/// Pre-encode a frame and submit to core 1 for CSMA TX.
/// Blocks (async) until core 1 signals TX_DONE.
async fn submit_tx(shared: &SharedState, frame: &[u8]) {
    if frame.len() < 5 || frame.len() > MAX_FRAME_LEN {
        warn!("TX: invalid frame len={}", frame.len());
        return;
    }

    // Build FM-0 encoded buffer using static scratch buffers (saves 24KB stack).
    // SAFETY: submit_tx is only called from core 0, never re-entrant.
    let fm0_len = unsafe {
        match build_tx_buffer_into(frame, &mut TX_HDLC_BUF, &mut TX_FM0_BUF) {
            Some(len) => len,
            None => {
                warn!("TX: build_tx_buffer failed");
                return;
            }
        }
    };

    // Pack into DMA words and write to tx_slot.
    // SAFETY: TX_SUBMIT is clear (we check below), so core 1 is not reading.
    // Wait for any previous TX to complete.
    while shared.flags.load(Ordering::Relaxed) & core1::TX_SUBMIT != 0 {
        Timer::after_micros(50).await;
    }
    while shared.flags.load(Ordering::Relaxed) & core1::TX_DONE != 0 {
        // Clear stale TX_DONE from previous round.
        shared.flags.fetch_and(!(core1::TX_DONE | core1::TX_RESULT_FAIL), Ordering::Release);
    }

    let slot = unsafe { &mut *shared.tx_slot.get() };
    let fm0 = unsafe { &TX_FM0_BUF };
    let word_count = pack_fm0_into_dma_buf(fm0, &mut slot.dma_buf);
    slot.word_count = word_count as u16;
    slot.fm0_len = fm0_len as u16;
    slot.frame_dest = if frame.len() >= 1 { frame[0] } else { 0 };
    slot.frame_src = if frame.len() >= 2 { frame[1] } else { 0 };
    let is_directed = frame.len() >= 3 && frame[2] < 0x80 && frame[0] != 0xFF;
    slot.is_directed = is_directed;

    // For directed frames, pre-encode the RTS frame too.
    if is_directed {
        let rts = compose_rts(slot.frame_dest, slot.frame_src);
        // Reuse the static scratch buffers (safe: still in submit_tx).
        let rts_ok = unsafe {
            build_tx_buffer_into(&rts, &mut TX_HDLC_BUF, &mut TX_FM0_BUF)
        };
        if let Some(rts_len) = rts_ok {
            let rts_fm0 = unsafe { &TX_FM0_BUF };
            let rts_words = pack_fm0_into_rts_buf(rts_fm0, &mut slot.rts_dma_buf);
            slot.rts_word_count = rts_words as u16;
            slot.rts_fm0_len = rts_len as u16;
        }
    } else {
        slot.rts_word_count = 0;
        slot.rts_fm0_len = 0;
    }

    defmt::debug!("TX {} bytes d={:02x} s={:02x} t={:02x} dir={}",
            frame.len(), slot.frame_dest, slot.frame_src,
            if frame.len() >= 3 { frame[2] } else { 0xFF },
            is_directed);

    compiler_fence(Ordering::SeqCst);
    shared.flags.fetch_or(core1::TX_SUBMIT, Ordering::Release);

    // Wait for core 1 to complete the TX.
    let deadline = embassy_time::Instant::now() + embassy_time::Duration::from_secs(2);
    loop {
        let flags = shared.flags.load(Ordering::Relaxed);
        if flags & core1::TX_DONE != 0 {
            shared.flags.fetch_and(!(core1::TX_DONE | core1::TX_RESULT_FAIL), Ordering::Release);
            break;
        }
        if embassy_time::Instant::now() >= deadline {
            warn!("TX: timeout waiting for core 1");
            break;
        }
        Timer::after_micros(50).await;
    }
}

/// Pack FM-0 symbols into 32-bit DMA words. Returns word count.
fn pack_fm0_into_dma_buf(fm0: &Fm0Buffer, dma_buf: &mut [u32; core1::TX_DMA_BUF_WORDS]) -> usize {
    let mut count = 0;
    for chunk in fm0.chunks(32) {
        let mut word = 0u32;
        for (i, &bit) in chunk.iter().enumerate() {
            word |= (bit as u32) << i;
        }
        dma_buf[count] = word;
        count += 1;
    }
    count
}

/// Pack FM-0 symbols into the RTS DMA buffer (16 words max). Returns word count.
fn pack_fm0_into_rts_buf(fm0: &Fm0Buffer, rts_buf: &mut [u32; 16]) -> usize {
    let mut count = 0;
    for chunk in fm0.chunks(32) {
        if count >= 16 { break; }
        let mut word = 0u32;
        for (i, &bit) in chunk.iter().enumerate() {
            word |= (bit as u32) << i;
        }
        rts_buf[count] = word;
        count += 1;
    }
    count
}

/// Update the node bitmap in shared state for core 1.
pub fn update_nodes(shared: &SharedState, bitmap: &[u8; 32]) {
    // Wait for core 1 to consume previous update.
    while shared.flags.load(Ordering::Relaxed) & core1::NODES_UPDATED != 0 {
        cortex_m::asm::nop();
    }
    // SAFETY: NODES_UPDATED is clear → core 1 is not reading.
    let bm = unsafe { &mut *shared.node_bitmap.get() };
    *bm = *bitmap;
    compiler_fence(Ordering::SeqCst);
    shared.flags.fetch_or(core1::NODES_UPDATED, Ordering::Release);
}

async fn write_chunked(cdc: &mut Cdc, data: &[u8]) -> Result<(), EndpointError> {
    for chunk in data.chunks(64) {
        match embassy_futures::select::select(
            cdc.write_packet(chunk),
            Timer::after_millis(100),
        )
        .await
        {
            embassy_futures::select::Either::First(r) => r?,
            embassy_futures::select::Either::Second(()) => {
                return Err(EndpointError::Disabled);
            }
        }
    }
    Ok(())
}

/// Sink for the CommandParser — collects frames and node updates.
struct HostSink {
    shared: &'static SharedState,
}

/// Pending outbound frame from the parser.
static mut PENDING_FRAME: Option<([u8; 610], u16)> = None;
/// Set by on_frame_end when the frame is fully received.
static mut PENDING_FRAME_READY: bool = false;
/// Pending node bitmap update.
static mut PENDING_NODES: Option<[u8; 32]> = None;

/// Static scratch buffers for TX encoding — avoids 24KB on the stack.
/// Only accessed from submit_tx (single-threaded on core 0).
static mut TX_HDLC_BUF: HdlcBits = HdlcBits::new();
static mut TX_FM0_BUF: Fm0Buffer = Fm0Buffer::new();

impl HostSink {
    /// Drain pending operations (must be called from async context).
    async fn drain_outbound(&mut self) {
        // Process node updates (synchronous, no await needed).
        if let Some(bitmap) = unsafe { PENDING_NODES.take() } {
            update_nodes(self.shared, &bitmap);
        }
        // Process outbound frames (only when fully received).
        if unsafe { PENDING_FRAME_READY } {
            if let Some((frame_buf, frame_len)) = unsafe { PENDING_FRAME.take() } {
                unsafe { PENDING_FRAME_READY = false; }
                submit_tx(self.shared, &frame_buf[..frame_len as usize]).await;
            }
        }
    }
}

impl Sink for HostSink {
    fn on_noop(&mut self) {}

    fn on_frame_header(&mut self, _dest: u8, _src: u8, _llap_type: u8) {
        // Reset pending frame.
        unsafe {
            PENDING_FRAME = Some(([0u8; 610], 0));
            PENDING_FRAME_READY = false;
        }
    }

    fn on_frame_byte(&mut self, byte: u8) {
        if let Some((ref mut buf, ref mut len)) = unsafe { &mut PENDING_FRAME } {
            if (*len as usize) < buf.len() {
                buf[*len as usize] = byte;
                *len += 1;
            }
        }
    }

    fn on_frame_end(&mut self) {
        // Signal drain_outbound that the frame is complete.
        unsafe { PENDING_FRAME_READY = true; }
    }

    fn on_set_node_ids(&mut self, bitmap: &[u8; 32]) {
        defmt::info!("HOST: SetNodeIds");
        unsafe { PENDING_NODES = Some(*bitmap); }
    }

    fn on_set_features(&mut self, features: u8) {
        use core::sync::atomic::AtomicU8;
        static FEATURES: AtomicU8 = AtomicU8::new(0);
        FEATURES.store(features, Ordering::Relaxed);
        defmt::info!("HOST: SetFeatures {:02x}", features);
    }
}

fn classify(e: EndpointError) -> defmt::Str {
    match e {
        EndpointError::BufferOverflow => defmt::intern!("buffer overflow"),
        EndpointError::Disabled => defmt::intern!("endpoint disabled"),
    }
}
