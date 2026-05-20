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
use crate::host::transmit_autoreply;
use crate::{MaybeLtRx, MaybeLtTx, SharedBackoff, SharedDe, SharedNodes};
use defmt::warn;
use embassy_rp::peripherals::DMA_CH0;
use embassy_rp::PeripheralRef;
use embassy_time::Timer;
use tashtalk_proto::fm0::{Fm0Decoder, Fm0Event};
use tashtalk_proto::llap::{auto_reply, AsmEvent, AutoReply, FrameAssembler};

/// Words per DMA buffer. 64 words = 2048 samples = ~1.1 ms of wire at
/// 1.8432 MHz oversampling. Long enough that DMA setup overhead is
/// negligible and short enough that worst-case latency from wire to
/// autoresponse is ~2 ms.
const BUF_WORDS: usize = 64;

#[embassy_executor::task]
pub async fn run(
    lt_rx: &'static MaybeLtRx,
    lt_tx: &'static MaybeLtTx,
    lt_de: &'static SharedDe,
    nodes: &'static SharedNodes,
    backoff: &'static SharedBackoff,
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
            decode_buffer(
                &buf_a,
                &mut decoder,
                &mut asm,
                &mut frames_seen,
                nodes,
                lt_tx,
                lt_de,
                backoff,
            )
            .await;
            xfer.await;
        }

        // Phase 2: DMA into buf_a while we process buf_b.
        {
            let xfer = rx.dma_pull(dma_ch.reborrow(), &mut buf_a);
            decode_buffer(
                &buf_b,
                &mut decoder,
                &mut asm,
                &mut frames_seen,
                nodes,
                lt_tx,
                lt_de,
                backoff,
            )
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
    backoff: &'static SharedBackoff,
) {
    let mut pending_reply: Option<[u8; 5]> = None;
    let mut frame_for_log: Option<([u8; 64], usize)> = None;

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
                let frame_len = asm.current().len();
                let mut frame_buf = [0u8; 64];
                let take = frame_len.min(frame_buf.len());
                frame_buf[..take].copy_from_slice(&asm.current()[..take]);
                asm.ack_frame_end();
                frame_for_log = Some((frame_buf, take));

                // If this CTS matches an RTS we just sent, wake up
                // the TX task. Independent of the autoresponder below.
                maybe_signal_cts(&frame_buf[..take]);

                // Decide reply synchronously (no await — we're inside
                // a hot decoding loop; defer the actual TX to after the
                // loop to keep this iteration short).
                let nodes_snapshot = *nodes.lock().await;
                if let Some(reply_frame) = auto_reply(&frame_buf[..take], &nodes_snapshot)
                    .into_frame()
                {
                    pending_reply = Some(reply_frame);
                }
            }
        }
    }

    // After the buffer is fully decoded, emit logs + any pending reply.
    if let Some((fbuf, len)) = frame_for_log {
        let dest = if len >= 1 { fbuf[0] } else { 0xFF };
        let src = if len >= 2 { fbuf[1] } else { 0xFF };
        let t = if len >= 3 { fbuf[2] } else { 0xFF };
        let c1 = if len >= 4 { fbuf[len - 2] } else { 0xFF };
        let c2 = if len >= 5 { fbuf[len - 1] } else { 0xFF };
        let crc_ok = tashtalk_proto::llap::frame_crc_ok(&fbuf[..len]);
        crate::tt_log!(
            "LT_RX: frame #{} len={} d={:02x} s={:02x} t={:02x} crc=[{:02x} {:02x}] crc_ok={}",
            *frames_seen,
            len,
            dest,
            src,
            t,
            c1,
            c2,
            crc_ok
        );
    }
    if let Some(reply) = pending_reply {
        crate::tt_log!(
            "LT_RX: -> reply [{:02x} {:02x} {:02x}]",
            reply[0],
            reply[1],
            reply[2]
        );
        transmit_autoreply(lt_tx, lt_de, &reply).await;
        let _ = backoff; // unused by autoreplies; mirrors PIC ISR behaviour
        // NOTE: do NOT reset the decoder/assembler here. With DMA-based
        // capture, sampling is uninterrupted during our TX, so the
        // decoder is in sync with the live wire — including the
        // self-loopback of the CTS we just emitted, which the
        // host-side CTS-watcher relies on to advance the RTS/CTS
        // dance. The abort-resync in the decoder already handles the
        // re-sync if anything does drift.
    }
    let _ = AutoReply::None;
}
