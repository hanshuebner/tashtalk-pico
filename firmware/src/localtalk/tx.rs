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

use defmt::warn;
use embassy_rp::pac;
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio::StateMachine;
use embassy_time::{Duration, Instant, Timer};
use tashtalk_proto::backoff::BackoffState;
use tashtalk_proto::hdlc::{build_immediate_tx_buffer, build_tx_buffer, Fm0Buffer};

use super::phy::{LtTxData, LtTxEnable};

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

        // Step 3: Load SM0 + SM1 FIFOs and start atomically.
        load_tx_fifo(&mut tx_data.sm, &fm0_buf);
        tx_enable.sm.tx().wait_push(frame_halfcells).await;

        // Start SM0 and SM1 simultaneously.
        start_sms_atomic(&mut tx_data.sm, &mut tx_enable.sm);

        // Step 4: Wait for PIO result.
        // SM1 sets IRQ 1 (success) or IRQ 2 (collision).
        let result = wait_tx_irq().await;

        // Stop both SMs.
        tx_data.sm.set_enable(false);
        tx_enable.sm.set_enable(false);

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

    // Enable TX driver directly via GPIO.
    pac::SIO.gpio_out(0).value_set().write_value(1 << TX_EN_PIN);

    // Small settle time for RS-422 driver.
    cortex_m::asm::delay(625); // ~5 µs at 125 MHz

    // Load SM0 FIFO and start.
    load_tx_fifo(&mut tx_data.sm, &fm0_buf);
    tx_data.sm.set_enable(true);

    // Wait for FIFO to drain.
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

/// Load FM-0 symbols into the TX FIFO, packed as 32-bit words.
fn load_tx_fifo(sm: &mut StateMachine<'_, PIO0, 0>, fm0: &Fm0Buffer) {
    for chunk in fm0.chunks(32) {
        let mut word = 0u32;
        for (i, &bit) in chunk.iter().enumerate() {
            word |= (bit as u32) << i;
        }
        // Non-blocking push — FIFO depth is 8 words with TxOnly join.
        // For longer frames, the DMA should be used. For now, blocking
        // push for simplicity.
        while sm.tx().full() {
            cortex_m::asm::nop();
        }
        sm.tx().push(word);
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
async fn wait_tx_irq() -> TxResult {
    loop {
        let irq = pac::PIO0.irq().read().irq();
        if irq & (1 << 1) != 0 {
            return TxResult::Success;
        }
        if irq & (1 << 2) != 0 {
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
