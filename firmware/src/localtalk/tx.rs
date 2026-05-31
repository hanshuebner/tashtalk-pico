//! LocalTalk transmitter — CSMA/CA and immediate modes.
//!
//! Two transmit modes per the design plan:
//!
//! 1. **CSMA/CA** (for RTS, ENQ, broadcast): waits for idle bus, then
//!    starts SM0+SM1 atomically. SM1 handles sync pulse, collision
//!    check, and frame gating in PIO hardware. CPU only handles
//!    carrier-sense polling and backoff.
//!
//! 2. **Immediate** (for CTS, ACK, data-after-CTS): bypasses SM1,
//!    drives TX_EN from software, and shifts out via SM0 only. Used
//!    for responses within an established dialog (IFG-bound).

use core::ptr::addr_of;
use core::sync::atomic::{compiler_fence, Ordering};

use defmt::warn;
use embassy_rp::pac;
use embassy_rp::pac::dma::vals::{DataSize, TreqSel};
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio::StateMachine;
use embassy_time::{Duration, Instant, Timer};
use tashtalk_proto::backoff::BackoffState;
use tashtalk_proto::hdlc::{build_immediate_tx_buffer, build_tx_buffer, Fm0Buffer};

use super::phy::{LtTxData, LtTxEnable};

/// DMA channel used for TX FIFO loading.
const TX_DMA_CH: usize = 1;

/// DREQ number for PIO0 SM0 TX.
const DREQ_PIO0_TX0: u8 = 0;

/// Maximum number of 32-bit words in a TX DMA buffer.
/// 16384 FM0 symbols / 32 bits per word = 512 words.
const TX_DMA_BUF_WORDS: usize = 512;

/// Static DMA source buffer. Must be word-aligned (u32 naturally is).
static mut TX_DMA_BUF: [u32; TX_DMA_BUF_WORDS] = [0u32; TX_DMA_BUF_WORDS];

/// Carrier sense GPIO pin number (GP5, driven by SM2).
const CARRIER_SENSE_PIN: u32 = 5;

/// TX enable GPIO pin number (GP3).
const TX_EN_PIN: u32 = 3;

/// Read the carrier sense GPIO pad directly.
#[inline]
fn carrier_sense_active() -> bool {
    pac::SIO.gpio_in(0).read() & (1 << CARRIER_SENSE_PIN) != 0
}

/// Maximum retries before giving up on a CSMA/CA transmission.
const MAX_CSMA_RETRIES: u32 = 32;

/// Transmit a frame using CSMA/CA (sync pulse + collision detection).
///
/// Returns `true` on success, `false` if all retries exhausted (collision).
pub async fn transmit_csma(
    tx_data: &mut LtTxData<'_>,
    tx_enable: &mut LtTxEnable<'_>,
    backoff: &mut BackoffState,
    frame: &[u8],
) -> bool {
    let Some((fm0_buf, frame_halfcells)) = build_tx_buffer(frame) else {
        warn!("TX: invalid frame");
        return false;
    };

    backoff.prep_for_next_frame();

    // Pack FM0 bits into DMA buffer (done once, reused across retries).
    let word_count = pack_into_dma_buf(&fm0_buf);

    for _ in 0..MAX_CSMA_RETRIES {
        // Step 1: Wait for carrier sense LOW (bus idle).
        wait_bus_idle().await;

        // Step 2: Wait IDG + random backoff.
        let entropy = (Instant::now().as_micros() as u8)
            ^ ((Instant::now().as_ticks() >> 8) as u8);
        let delay_units = backoff.next_backoff_units(entropy);
        let delay_us = delay_units as u64 * 100;

        if !wait_idle_for(delay_us).await {
            backoff.record_deferral();
            continue;
        }

        // Step 3: Push SM1 half-cell count and start DMA + SMs.
        tx_enable.sm.tx().wait_push(frame_halfcells).await;

        // Start DMA feeding SM0 TX FIFO *before* enabling SMs so the
        // FIFO is already being filled when the state machines start.
        start_tx_dma(word_count);

        // Start SM0 and SM1 simultaneously.
        start_sms_atomic(&mut tx_data.sm, &mut tx_enable.sm);

        // Step 4: Wait for PIO result.
        // SM1 sets IRQ 1 (success) or IRQ 2 (collision).
        let result = wait_tx_irq().await;

        // Stop both SMs.
        tx_data.sm.set_enable(false);
        tx_enable.sm.set_enable(false);

        // Abort any in-flight DMA and wait for it to finish.
        abort_tx_dma();

        // Clear IRQ flags.
        pac::PIO0.irq().write(|w| w.set_irq(0x06)); // clear bits 1 and 2

        // Restart both SMs to reset PC and FIFOs.
        tx_data.sm.restart();
        tx_enable.sm.restart();

        match result {
            TxResult::Success => return true,
            TxResult::Collision => {
                backoff.record_collision();
                continue;
            }
        }
    }

    false
}

/// Transmit a frame immediately (no CSMA/CA, no sync pulse).
///
/// Used for CTS, ACK, and data-after-CTS responses that must arrive
/// within the 200 µs IFG. TX_EN is controlled directly from software.
pub async fn transmit_immediate(
    tx_data: &mut LtTxData<'_>,
    frame: &[u8],
) {
    let Some(fm0_buf) = build_immediate_tx_buffer(frame) else {
        warn!("TX immediate: invalid frame");
        return;
    };

    // Pack FM0 bits into DMA buffer.
    let word_count = pack_into_dma_buf(&fm0_buf);

    // Enable TX driver directly via GPIO.
    pac::SIO.gpio_out(0).value_set().write_value(1 << TX_EN_PIN);

    // Small settle time for RS-422 driver.
    cortex_m::asm::delay(625); // ~5 µs at 125 MHz

    // Start DMA feeding SM0 TX FIFO, then enable SM0.
    start_tx_dma(word_count);
    tx_data.sm.set_enable(true);

    // Wait for DMA to complete (all words pushed to FIFO).
    wait_tx_dma().await;

    // Wait for FIFO to drain (SM shifts out remaining words).
    wait_fifo_drain(&mut tx_data.sm).await;

    // Wait for the last word to shift out.
    let last_bits = fm0_buf.len() % 32;
    let last_bits = if last_bits == 0 { 32 } else { last_bits };
    // Each bit is one PIO cycle at 460.8 kHz = ~2.17 µs per half-cell.
    Timer::after_micros((last_bits as u64 + 2) * 2).await;

    tx_data.sm.set_enable(false);
    tx_data.sm.restart();

    // Disable TX driver.
    pac::SIO.gpio_out(0).value_clr().write_value(1 << TX_EN_PIN);
}

// ── Helpers ─────────────────────────────────────────────────────────

/// Wait until the carrier sense GPIO is LOW.
async fn wait_bus_idle() {
    while carrier_sense_active() {
        Timer::after_micros(10).await;
    }
}

/// Wait for the bus to remain idle for `us` microseconds.
/// Returns `false` if interrupted by bus activity.
async fn wait_idle_for(us: u64) -> bool {
    let deadline = Instant::now() + Duration::from_micros(us);
    while Instant::now() < deadline {
        if carrier_sense_active() {
            return false;
        }
        Timer::after_micros(10).await;
    }
    !carrier_sense_active()
}

/// Pack FM-0 symbols into the static DMA buffer as 32-bit words.
///
/// Returns the number of words written.
fn pack_into_dma_buf(fm0: &Fm0Buffer) -> usize {
    let mut idx = 0usize;
    for chunk in fm0.chunks(32) {
        let mut word = 0u32;
        for (i, &bit) in chunk.iter().enumerate() {
            word |= (bit as u32) << i;
        }
        // SAFETY: single-threaded access; only called from the TX task.
        unsafe { TX_DMA_BUF[idx] = word };
        idx += 1;
    }
    // Ensure all stores to the buffer are visible before DMA reads them.
    compiler_fence(Ordering::Release);
    idx
}

/// Start a DMA transfer from `TX_DMA_BUF` to PIO0 SM0 TX FIFO.
fn start_tx_dma(word_count: usize) {
    let ch = pac::DMA.ch(TX_DMA_CH);

    // SAFETY: addr_of! only takes the address without creating a reference.
    let src = addr_of!(TX_DMA_BUF) as u32;
    let dst = pac::PIO0.txf(0).as_ptr() as u32;

    ch.read_addr().write_value(src);
    ch.write_addr().write_value(dst);
    ch.trans_count().write_value(word_count as u32);
    ch.ctrl_trig().write(|w| {
        w.set_en(true);
        w.set_data_size(DataSize::SIZE_WORD);
        w.set_incr_read(true);
        w.set_incr_write(false);
        w.set_treq_sel(TreqSel(DREQ_PIO0_TX0));
        // Chain to self (disable chaining).
        w.set_chain_to(TX_DMA_CH as u8);
    });
}

/// Wait for the TX DMA transfer to complete.
async fn wait_tx_dma() {
    let ch = pac::DMA.ch(TX_DMA_CH);
    let deadline = Instant::now() + Duration::from_millis(500);
    while ch.ctrl_trig().read().busy() {
        if Instant::now() >= deadline {
            warn!("TX DMA: timeout waiting for completion");
            abort_tx_dma();
            return;
        }
        embassy_futures::yield_now().await;
    }
}

/// Abort any in-flight TX DMA transfer and wait for it to stop.
fn abort_tx_dma() {
    let ch_bit = 1u16 << TX_DMA_CH;
    pac::DMA.chan_abort().write(|w| w.set_chan_abort(ch_bit));
    // Poll until the abort register clears (typically < 1 µs).
    while pac::DMA.chan_abort().read().chan_abort() & ch_bit != 0 {
        cortex_m::asm::nop();
    }
}

/// Start SM0 and SM1 simultaneously using the PIO CTRL register.
fn start_sms_atomic(
    _sm0: &mut StateMachine<'_, PIO0, 0>,
    _sm1: &mut StateMachine<'_, PIO0, 1>,
) {
    // Set SM_ENABLE bits for SM0 (bit 0) and SM1 (bit 1) atomically.
    pac::PIO0.ctrl().modify(|w| {
        w.set_sm_enable(w.sm_enable() | 0x03);
    });
}

/// Wait for SM1 to signal success (IRQ 1) or collision (IRQ 2).
///
/// Times out after 500 ms to avoid hanging forever if the PIO program
/// gets stuck.
async fn wait_tx_irq() -> TxResult {
    let deadline = Instant::now() + Duration::from_millis(500);
    loop {
        let irq = pac::PIO0.irq().read().irq();
        if irq & (1 << 1) != 0 {
            return TxResult::Success;
        }
        if irq & (1 << 2) != 0 {
            return TxResult::Collision;
        }
        if Instant::now() >= deadline {
            warn!("TX: wait_tx_irq timeout (500 ms)");
            return TxResult::Collision;
        }
        // Yield briefly to avoid busy-spinning.
        embassy_futures::yield_now().await;
    }
}

/// Wait for the TX FIFO to empty.
async fn wait_fifo_drain(sm: &mut StateMachine<'_, PIO0, 0>) {
    let mut iters = 0u32;
    while !sm.tx().empty() {
        Timer::after_micros(50).await;
        iters += 1;
        if iters > 2000 {
            break;
        }
    }
}

enum TxResult {
    Success,
    Collision,
}
