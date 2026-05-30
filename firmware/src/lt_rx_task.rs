//! Continuous LocalTalk receiver — ring-DMA, low-latency.
//!
//! Owns PIO0 SM1 (LtRx) for the lifetime of the program. A single DMA
//! channel runs **continuously** into a power-of-two ring buffer using
//! the RP2040 write-address wrap (`RING_SEL`/`RING_SIZE`): the channel
//! never needs the CPU to restart it. The task polls the DMA's write
//! pointer and decodes whatever samples have arrived since last time, in
//! near-real-time.
//!
//! Why a ring instead of the old ping-pong: with CPU-restarted ping-pong
//! the receiver couldn't restart its capture DMA while the CPU was busy
//! driving a ~300 µs autonomous reply, so a small (low-latency) buffer
//! overran the PIO FIFO mid-reply and corrupted the peer's following data
//! frame. A continuous ring keeps capturing through our own TX (the DMA
//! is autonomous), so we can compose **and immediately transmit** an
//! ACK/CTS inline the instant a frame's closing flag is decoded — driving
//! the wire→CTS turnaround down from ~1.1 ms (one big DMA buffer) to tens
//! of microseconds, inside LLAP's ~200 µs response window.
//!
//! On a complete frame: validate CRC, forward to the host (unless it is
//! our own echo or an ENQ/RTS we answer), and fire any autonomous reply.

use crate::csma::{mark_wire_active, maybe_signal_cts};
use crate::host::{crc_check_enabled, take_self_echo, transmit_autoreply, try_forward_inbound};
use crate::{MaybeLtRx, MaybeLtTx, SharedDe, SharedNodes};
use core::sync::atomic::{compiler_fence, Ordering};
use defmt::warn;
use embassy_rp::dma::Channel;
use embassy_rp::pac;
use embassy_rp::pac::dma::vals::{DataSize, TreqSel};
use embassy_rp::peripherals::DMA_CH0;
use embassy_rp::PeripheralRef;
use embassy_time::Timer;
use static_cell::StaticCell;
use tashtalk_proto::codec::FrameStatus;
use tashtalk_proto::fm0::{Fm0Decoder, Fm0Event};
use tashtalk_proto::llap::{auto_reply, frame_crc_ok, AsmEvent, FrameAssembler};

/// Ring size in 32-bit words. Must be a power of two so the byte-granular
/// write-ring wrap (`1 << RING_SIZE_LOG2` bytes) lines up. 512 words =
/// 2048 bytes ≈ 8.9 ms of wire at 1.8432 MHz — far more than the CPU can
/// fall behind, even while it drives a ~300 µs reply (during which the
/// DMA keeps filling the ring).
const RING_WORDS: usize = 512;
const RING_SIZE_LOG2: u8 = 11; // log2(RING_WORDS * 4)

/// DREQ for the PIO0 SM1 RX FIFO: `pio_no*8 + sm + 4 = 0*8 + 1 + 4`.
const PIO0_RX1_TREQ: u8 = 5;

/// Ring buffer, aligned so the DMA write-address wrap is valid.
#[repr(C, align(2048))]
struct Ring([u32; RING_WORDS]);

#[embassy_executor::task]
pub async fn run(
    lt_rx: &'static MaybeLtRx,
    lt_tx: &'static MaybeLtTx,
    lt_de: &'static SharedDe,
    nodes: &'static SharedNodes,
    dma_ch: PeripheralRef<'static, DMA_CH0>,
) -> ! {
    crate::tt_log!("LT_RX: task starting (ring DMA)");

    let mut rx_guard = lt_rx.lock().await;
    let Some(rx) = rx_guard.as_mut() else {
        warn!("LT_RX: LtRx is None — sleeping forever");
        loop {
            Timer::after_secs(3600).await;
        }
    };
    rx.drain();
    rx.enable();

    // The DMA ring lives in a 2048-byte-aligned static. We only touch it
    // through a raw pointer after this point — the DMA writes it from
    // hardware, the CPU reads behind the write pointer.
    static RING: StaticCell<Ring> = StaticCell::new();
    let ring_ptr: *const u32 = RING.init(Ring([0u32; RING_WORDS])).0.as_ptr();
    let ring_base = ring_ptr as u32;

    let ch_num = dma_ch.number();
    start_ring_dma(ch_num, ring_base);
    crate::tt_log!("LT_RX: ring DMA armed (ch {})", ch_num);

    let mut decoder = Fm0Decoder::new();
    let mut asm = FrameAssembler::new();
    let mut frames_seen: u32 = 0;
    let mut read_idx: usize = 0;

    loop {
        // Where has the DMA written to? write_addr wraps inside the ring.
        let write_addr = pac::DMA.ch(ch_num as usize).write_addr().read();
        let write_idx = (write_addr.wrapping_sub(ring_base) as usize / 4) % RING_WORDS;
        let avail = (write_idx + RING_WORDS - read_idx) % RING_WORDS;

        if avail == 0 {
            // Re-arm if the channel ever stopped (trans_count exhausted).
            if !pac::DMA.ch(ch_num as usize).ctrl_trig().read().busy() {
                rx.drain();
                start_ring_dma(ch_num, ring_base);
                read_idx = 0;
            }
            Timer::after_micros(20).await;
            continue;
        }

        for _ in 0..avail {
            // SAFETY: the DMA wrote this word before advancing write_addr
            // past it; we only read indices strictly behind write_idx.
            let word = unsafe { core::ptr::read_volatile(ring_ptr.add(read_idx)) };
            read_idx = (read_idx + 1) % RING_WORDS;

            for bit_idx in 0..32 {
                let sample = ((word >> bit_idx) & 1) as u8;
                let Some(event) = decoder.feed_sample(sample) else { continue };
                mark_wire_active();
                let asm_event = match event {
                    Fm0Event::Byte(b) => asm.feed_byte(b),
                    Fm0Event::Flag => asm.feed_flag(),
                    Fm0Event::Abort => asm.feed_abort(),
                };
                if let Some(AsmEvent::FrameEnd) = asm_event {
                    frames_seen = frames_seen.wrapping_add(1);

                    let frame = asm.current();
                    let len = frame.len();
                    let crc_ok = frame_crc_ok(frame);
                    let dest = if len >= 1 { frame[0] } else { 0xFF };
                    let src = if len >= 2 { frame[1] } else { 0xFF };
                    let t = if len >= 3 { frame[2] } else { 0xFF };
                    let c1 = if len >= 4 { frame[len - 2] } else { 0xFF };
                    let c2 = if len >= 5 { frame[len - 1] } else { 0xFF };

                    // Log any CTS frame seen, for diagnostics.
                    if t == 0x85 {
                        crate::tt_log!("LT_RX: CTS! d={:02x} s={:02x} crc_ok={}", dest, src, crc_ok);
                    }

                    // If this CTS matches an RTS we just sent, wake the TX
                    // task (host-driven dance). Independent of the below.
                    maybe_signal_cts(frame);

                    let nodes_snapshot = *nodes.lock().await;
                    let reply = auto_reply(frame, &nodes_snapshot).into_frame();

                    let disposition = if reply.is_some() {
                        // We answer ENQ/RTS autonomously; don't forward it
                        // up or a host that also replies would duplicate.
                        "auto"
                    } else if take_self_echo(frame) {
                        "self"
                    } else {
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
                        frames_seen,
                        len,
                        dest,
                        src,
                        t,
                        c1,
                        c2,
                        crc_ok,
                        disposition
                    );

                    // Transmit the reply inline, right now. The ring DMA
                    // keeps capturing while we drive DE, so nothing is
                    // lost (and our own echo is muted/suppressed anyway).
                    if let Some(reply_frame) = reply {
                        transmit_autoreply(lt_tx, lt_de, &reply_frame).await;
                    }
                }
            }
        }

        // Let the USB / host tasks run between chunks so a long inbound
        // frame doesn't starve them, without adding meaningful latency.
        embassy_futures::yield_now().await;
    }
}

/// Arm `ch` for continuous capture from the PIO0 SM1 RX FIFO into a
/// write-address-wrapped ring at `ring_base`. Self-chained (no chaining)
/// with a maximal transfer count; the caller re-arms if it ever expires.
fn start_ring_dma(ch_num: u8, ring_base: u32) {
    let ch = pac::DMA.ch(ch_num as usize);
    let fifo = pac::PIO0.rxf(1).as_ptr() as u32;
    ch.read_addr().write_value(fifo);
    ch.write_addr().write_value(ring_base);
    ch.trans_count().write_value(0xFFFF_FFFF);
    compiler_fence(Ordering::SeqCst);
    ch.ctrl_trig().write(|w| {
        w.set_treq_sel(TreqSel(PIO0_RX1_TREQ));
        w.set_data_size(DataSize::SIZE_WORD);
        w.set_incr_read(false); // FIFO is a fixed address
        w.set_incr_write(true); // walk the ring
        w.set_ring_sel(true); // wrap the WRITE address
        w.set_ring_size(RING_SIZE_LOG2);
        w.set_chain_to(ch_num); // chain to self = chaining disabled
        w.set_en(true);
    });
    compiler_fence(Ordering::SeqCst);
}
