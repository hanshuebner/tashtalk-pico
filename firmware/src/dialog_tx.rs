//! Hardware-driven originator RTS/CTS/DATA dialog.
//!
//! Pre-encodes the data frame into a static DMA buffer, arms DMA CH1
//! targeting PIO0 TXF0, and exposes a trigger function that the RX task
//! calls inline (no await) when it detects a matching CTS.
//!
//! The trigger path is pure hardware:
//!   1. RX task asserts DE via PAC GPIO write
//!   2. Brief inline spin (~5 µs settle)
//!   3. Forces PIO IRQ 4 → SM0 unblocks from `wait 1 irq 4`
//!   4. SM0 pulls from TX FIFO → DMA fires on DREQ and feeds words
//!
//! Total CTS-detection → first-bit-on-wire latency: ~10 µs.

use crate::bitstream::BitStream;
use core::sync::atomic::{AtomicBool, Ordering};
use embassy_rp::pac;
use embassy_rp::pac::dma::vals::{DataSize, TreqSel};

/// DMA channel used for the dialog data frame.
const DMA_CH: usize = 1;

/// DREQ for PIO0 SM0 TX FIFO: `pio_no*8 + sm = 0`.
const PIO0_TX0_DREQ: u8 = 0;

/// Maximum word count for a pre-encoded frame.
/// MAX_FRAME_LEN=605, bit-stuffed ≈ 605*10+32 = 6082 bits, /32 = 191 words.
/// Round up generously.
const MAX_WORDS: usize = 256;

/// Static DMA-source buffer. Must be accessible to DMA (any SRAM is fine
/// on RP2040). Aligned to 4 bytes for word-sized DMA.
#[repr(align(4))]
struct DmaBuf([u32; MAX_WORDS]);

static mut DMA_BUF: DmaBuf = DmaBuf([0u32; MAX_WORDS]);

/// Number of valid words in the buffer.
static mut DMA_WORD_COUNT: u32 = 0;

/// Set to `true` by the RX task's inline trigger. Polled by the host
/// task to know when the hardware-triggered TX has fired.
pub static DIALOG_TX_FIRED: AtomicBool = AtomicBool::new(false);

/// Set to `true` when the dialog DMA + SM0 are armed and ready for
/// a CTS trigger. The RX task checks this before triggering.
pub static DIALOG_ARMED: AtomicBool = AtomicBool::new(false);

/// Copy a pre-encoded bitstream into the static DMA buffer.
///
/// # Safety
/// Must be called while no DMA transfer is in progress on CH1.
pub fn load_data(stream: &BitStream) {
    let bytes = stream.as_bytes();
    let total_bits = stream.bit_len();
    let word_count = (total_bits + 31) / 32;

    // SAFETY: single-threaded executor; DMA is not active.
    unsafe {
        let mut byte_idx = 0usize;
        for w in 0..word_count as usize {
            let mut word = 0u32;
            for shift in 0..4 {
                if byte_idx < bytes.len() {
                    word |= (bytes[byte_idx] as u32) << (shift * 8);
                    byte_idx += 1;
                }
            }
            DMA_BUF.0[w] = word;
        }
        DMA_WORD_COUNT = word_count;
    }
}

/// Arm DMA CH1 to feed the pre-encoded data frame to PIO0 SM0 TX FIFO.
/// The DMA is enabled but won't transfer until SM0 starts pulling
/// (which happens when PIO IRQ 4 fires and SM0 unblocks).
pub fn arm_dma() {
    // SAFETY: single-threaded executor; we own this channel.
    let word_count = unsafe { DMA_WORD_COUNT };
    let read_addr = unsafe { DMA_BUF.0.as_ptr() as u32 };

    let ch = pac::DMA.ch(DMA_CH);
    // Abort any previous transfer
    ch.ctrl_trig().write(|w| w.set_en(false));

    ch.read_addr().write_value(read_addr);
    ch.write_addr().write_value(pac::PIO0.txf(0).as_ptr() as u32);
    ch.trans_count().write_value(word_count);
    core::sync::atomic::compiler_fence(Ordering::SeqCst);
    ch.ctrl_trig().write(|w| {
        w.set_treq_sel(TreqSel(PIO0_TX0_DREQ));
        w.set_data_size(DataSize::SIZE_WORD);
        w.set_incr_read(true);
        w.set_incr_write(false);
        w.set_chain_to(DMA_CH as u8); // self = no chaining
        w.set_en(true);
    });

    DIALOG_TX_FIRED.store(false, Ordering::Release);
    DIALOG_ARMED.store(true, Ordering::Release);
}

/// Abort an armed or in-progress dialog DMA transfer.
pub fn abort() {
    DIALOG_ARMED.store(false, Ordering::Release);
    let ch = pac::DMA.ch(DMA_CH);
    ch.ctrl_trig().write(|w| w.set_en(false));
}

/// `true` if DMA CH1 is still busy transferring.
pub fn is_dma_busy() -> bool {
    pac::DMA.ch(DMA_CH).ctrl_trig().read().busy()
}

/// Called inline by the RX task when a matching CTS is decoded.
/// Asserts DE, spins briefly for driver settle, then fires PIO IRQ 4
/// to unblock SM0. No async, no mutex, no scheduler involvement.
///
/// Returns `true` if the trigger fired, `false` if the dialog was
/// not armed (caller should not signal CTS_RECEIVED in that case).
///
/// # Safety
/// Writes directly to GPIO and PIO registers. Safe to call from the
/// RX task because:
/// - GPIO set/clear are separate registers (no read-modify-write race)
/// - The host task knows DE may be asserted and will clear it after DMA
/// - PIO IRQ 4 is reserved for this purpose
#[inline(never)]
pub fn trigger() -> bool {
    if !DIALOG_ARMED.load(Ordering::Acquire) {
        return false;
    }
    DIALOG_ARMED.store(false, Ordering::Release);

    // 1. Assert DE (GP3) via PAC — no mutex needed
    pac::SIO.gpio_out(0).value_set().write_value(1 << 3);

    // 2. Settle time (~5 µs at 125 MHz ≈ 625 cycles)
    cortex_m::asm::delay(625);

    // 3. Fire PIO IRQ 4 → SM0 unblocks from `wait 1 irq 4`
    pac::PIO0
        .irq_force()
        .write(|w| w.set_irq_force(1 << 4));

    // 4. Signal the host task
    DIALOG_TX_FIRED.store(true, Ordering::Release);
    true
}

/// De-assert DE (GP3) after dialog TX completes.
pub fn deassert_de() {
    pac::SIO.gpio_out(0).value_clr().write_value(1 << 3);
}
