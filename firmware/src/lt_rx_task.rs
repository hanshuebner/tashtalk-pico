//! Continuous LocalTalk receiver — DMA-driven.
//!
//! Owns PIO0 SM1 (LtRx) for the lifetime of the program. Uses ping-pong
//! DMA to pull samples from the PIO RX FIFO into RAM with zero per-word
//! CPU cost — eliminating the FIFO-overflow risk that polling-based
//! capture has under bursty CPU load.
//!
//! Each iteration of the main loop:
//!   1. Starts a DMA transfer into the "next" buffer (background).
//!   2. Synchronously walks the "current" buffer through the FM-0
//!      decoder and the SDLC frame assembler.
//!   3. Awaits the DMA completion. While we were processing, DMA was
//!      filling next.
//!   4. Swaps buffers and repeats.
//!
//! If processing is faster than DMA time, the await is brief and we
//! capture continuously with no FIFO buffering needed.
//!
//! On a complete frame: validates CRC, runs the autoresponder, and —
//! if needed — locks the TX path briefly to emit the reply. The
//! firmware-side analogue of `DealWithFrame` in `one-chip.asm:704`.

use crate::csma::{mark_wire_active, maybe_signal_cts};
use crate::host::{crc_check_enabled, take_self_echo, transmit_autoreply, try_forward_inbound};
use crate::{MaybeLtRx, MaybeLtTx, SharedDe, SharedNodes};
use defmt::warn;
use embassy_rp::peripherals::DMA_CH0;
use embassy_rp::PeripheralRef;
use embassy_time::Timer;
use tashtalk_proto::codec::FrameStatus;
use tashtalk_proto::fm0::{Fm0Decoder, Fm0Event};
use tashtalk_proto::llap::{auto_reply, frame_crc_ok, AsmEvent, FrameAssembler};

/// Words per DMA buffer. 64 words = 2048 samples ≈ 1.1 ms of wire at
/// 1.8432 MHz oversampling. Long enough that DMA setup overhead is
/// negligible and the receiver never overruns the PIO FIFO while the
/// CPU is busy decoding or driving an autonomous reply.
///
/// This is the known-good size: an AFP session mounts and transfers
/// files over it (slowly — the wire→CTS latency is ~1 ms, well past
/// LLAP's ~200 µs window, so the Mac retransmits its way through). A
/// smaller buffer cuts that latency but overran the FIFO during our own
/// reply TX and corrupted the peer's following data frame, breaking the
/// session entirely. Reducing latency safely needs continuous (chained
/// or ring) DMA so the capture never stalls during TX — future work.
const BUF_WORDS: usize = 64;

#[embassy_executor::task]
pub async fn run(
    lt_rx: &'static MaybeLtRx,
    lt_tx: &'static MaybeLtTx,
    lt_de: &'static SharedDe,
    nodes: &'static SharedNodes,
    mut dma_ch: PeripheralRef<'static, DMA_CH0>,
) -> ! {
    crate::tt_log!("LT_RX: task starting (DMA)");

    let mut rx_guard = lt_rx.lock().await;
    let Some(rx) = rx_guard.as_mut() else {
        warn!("LT_RX: LtRx is None — sleeping forever");
        loop {
            Timer::after_secs(3600).await;
        }
    };
    rx.drain();
    rx.enable();
    crate::tt_log!("LT_RX: SM1 enabled, starting DMA pump");

    let mut decoder = Fm0Decoder::new();
    let mut asm = FrameAssembler::new();
    let mut frames_seen: u32 = 0;

    // Two ping-pong buffers. While DMA fills one, we decode the other.
    let mut buf_a = [0u32; BUF_WORDS];
    let mut buf_b = [0u32; BUF_WORDS];

    // Prime the pipeline: fill buf_a once, then enter the ping-pong loop.
    rx.dma_pull(dma_ch.reborrow(), &mut buf_a).await;

    loop {
        // Phase 1: DMA into buf_b while we process buf_a.
        {
            let xfer = rx.dma_pull(dma_ch.reborrow(), &mut buf_b);
            decode_buffer(&buf_a, &mut decoder, &mut asm, &mut frames_seen, nodes, lt_tx, lt_de)
                .await;
            xfer.await;
        }

        // Phase 2: DMA into buf_a while we process buf_b.
        {
            let xfer = rx.dma_pull(dma_ch.reborrow(), &mut buf_a);
            decode_buffer(&buf_b, &mut decoder, &mut asm, &mut frames_seen, nodes, lt_tx, lt_de)
                .await;
            xfer.await;
        }
    }
}

/// Walk one DMA buffer through the decoder + assembler + autoresponder.
#[allow(clippy::too_many_arguments)]
async fn decode_buffer(
    buf: &[u32],
    decoder: &mut Fm0Decoder,
    asm: &mut FrameAssembler,
    frames_seen: &mut u32,
    nodes: &'static SharedNodes,
    lt_tx: &'static MaybeLtTx,
    lt_de: &'static SharedDe,
) {
    let mut pending_reply: Option<[u8; 5]> = None;

    for word in buf.iter() {
        for bit_idx in 0..32 {
            let sample = ((word >> bit_idx) & 1) as u8;
            let Some(event) = decoder.feed_sample(sample) else { continue };
            // The wire produced an event — record activity for CSMA.
            mark_wire_active();
            let asm_event = match event {
                Fm0Event::Byte(b) => asm.feed_byte(b),
                Fm0Event::Flag => asm.feed_flag(),
                Fm0Event::Abort => asm.feed_abort(),
            };
            if let Some(AsmEvent::FrameEnd) = asm_event {
                *frames_seen = frames_seen.wrapping_add(1);

                // Work directly off the assembler's buffer (full frame,
                // up to 605 bytes — not truncated to 64) until we've
                // decided what to do with it, then release it.
                let frame = asm.current();
                let len = frame.len();
                let crc_ok = frame_crc_ok(frame);
                let dest = if len >= 1 { frame[0] } else { 0xFF };
                let src = if len >= 2 { frame[1] } else { 0xFF };
                let t = if len >= 3 { frame[2] } else { 0xFF };
                let c1 = if len >= 4 { frame[len - 2] } else { 0xFF };
                let c2 = if len >= 5 { frame[len - 1] } else { 0xFF };

                // If this CTS matches an RTS we just sent, wake up the
                // TX task. Independent of the autoresponder below.
                maybe_signal_cts(frame);

                // Autonomous reply? (ENQ/RTS addressed to one of our
                // nodes.) Queue it for the dedicated TX task and keep
                // draining — never transmit from this capture loop.
                let nodes_snapshot = *nodes.lock().await;
                let reply = auto_reply(frame, &nodes_snapshot).into_frame();

                let disposition = if let Some(reply_frame) = reply {
                    // The firmware answers this autonomously. Do NOT
                    // forward it up: a host that also responds to ENQ/RTS
                    // (e.g. TailTalk) would emit a duplicate reply.
                    pending_reply = Some(reply_frame);
                    "auto"
                } else if take_self_echo(frame) {
                    // Our own transmission, looped back through the
                    // transceiver. Suppress — the host already has it.
                    "self"
                } else {
                    // A genuine bus frame destined for the host.
                    let status = if !crc_ok && crc_check_enabled() {
                        FrameStatus::BadCrc
                    } else {
                        FrameStatus::Good
                    };
                    try_forward_inbound(frame, status);
                    "host"
                };

                asm.ack_frame_end();

                crate::tt_log!(
                    "LT_RX: frame #{} len={} d={:02x} s={:02x} t={:02x} crc=[{:02x} {:02x}] crc_ok={} -> {}",
                    *frames_seen,
                    len,
                    dest,
                    src,
                    t,
                    c1,
                    c2,
                    crc_ok,
                    disposition
                );
            }
        }
    }

    // Transmit any autonomous reply immediately, inline on this task —
    // not via a separate task. The capture loop runs at high frequency
    // (one small buffer ≈139 µs), so this keeps the wire→CTS latency
    // inside LLAP's ~200 µs window; handing off to another task adds
    // ~1 ms of cooperative-scheduler delay. The brief PIO-FIFO overrun
    // while we drive the ~300 µs reply is harmless: the transceiver mutes
    // its receiver during our TX anyway, so the only samples lost are our
    // own reply, and the decoder re-syncs on the peer's next frame.
    if let Some(reply) = pending_reply {
        transmit_autoreply(lt_tx, lt_de, &reply).await;
    }
}
