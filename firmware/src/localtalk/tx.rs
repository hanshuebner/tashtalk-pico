//! LocalTalk transmitter.
//!
//! DE (driver enable) is driven directly from software. SM0 shifts
//! pre-encoded FM-0 data via DMA. Both CSMA/CA and immediate modes
//! use the same TX path — only the preamble and CSMA wait differ.

use core::ptr::addr_of;
use core::sync::atomic::{compiler_fence, Ordering};

use defmt::warn;
use embassy_rp::gpio::Level;
use embassy_rp::pac;
use embassy_rp::pac::dma::vals::{DataSize, TreqSel};
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio::StateMachine;
use embassy_time::{Duration, Instant, Timer};
use tashtalk_proto::backoff::BackoffState;
use tashtalk_proto::codec::MAX_FRAME_LEN;
use tashtalk_proto::hdlc::{build_immediate_tx_buffer, build_tx_buffer, Fm0Buffer};

use super::phy::{LtTxData, LtTxEnable};

const CARRIER_SENSE_PIN: u32 = 5;

/// DMA channel for TX data.
const TX_DMA_CH: usize = 1;
/// DREQ for PIO0 SM0 TX FIFO.
const DREQ_PIO0_TX0: u8 = 0;
/// Max DMA buffer words.
const TX_DMA_BUF_WORDS: usize = 512;

static mut TX_DMA_BUF: [u32; TX_DMA_BUF_WORDS] = [0u32; TX_DMA_BUF_WORDS];

#[inline]
fn carrier_sense_active() -> bool {
    pac::SIO.gpio_in(0).read() & (1 << CARRIER_SENSE_PIN) != 0
}

fn pack_into_dma_buf(fm0: &Fm0Buffer) -> usize {
    let mut count = 0;
    for chunk in fm0.chunks(32) {
        let mut word = 0u32;
        for (i, &bit) in chunk.iter().enumerate() {
            word |= (bit as u32) << i;
        }
        unsafe { TX_DMA_BUF[count] = word; }
        count += 1;
    }
    count
}

fn start_tx_dma(word_count: usize) {
    let ch = pac::DMA.ch(TX_DMA_CH);
    ch.ctrl_trig().write(|w| w.set_en(false));
    let read_addr = addr_of!(TX_DMA_BUF) as u32;
    ch.read_addr().write_value(read_addr);
    ch.write_addr().write_value(pac::PIO0.txf(0).as_ptr() as u32);
    ch.trans_count().write_value(word_count as u32);
    compiler_fence(Ordering::SeqCst);
    ch.ctrl_trig().write(|w| {
        w.set_treq_sel(TreqSel(DREQ_PIO0_TX0));
        w.set_data_size(DataSize::SIZE_WORD);
        w.set_incr_read(true);
        w.set_incr_write(false);
        w.set_chain_to(TX_DMA_CH as u8);
        w.set_en(true);
    });
    compiler_fence(Ordering::SeqCst);
}

async fn wait_tx_dma() {
    let ch = pac::DMA.ch(TX_DMA_CH);
    let deadline = Instant::now() + Duration::from_millis(500);
    while ch.ctrl_trig().read().busy() {
        if Instant::now() >= deadline {
            warn!("TX DMA: timeout");
            ch.ctrl_trig().write(|w| w.set_en(false));
            return;
        }
        Timer::after_micros(10).await;
    }
}

fn abort_tx_dma() {
    pac::DMA.ch(TX_DMA_CH).ctrl_trig().write(|w| w.set_en(false));
}

async fn wait_fifo_drain(sm: &mut StateMachine<'_, PIO0, 0>) {
    let deadline = Instant::now() + Duration::from_millis(500);
    while !sm.tx().empty() {
        if Instant::now() >= deadline { break; }
        Timer::after_micros(10).await;
    }
}

/// Inner TX: assert DE, shift out FM-0 data via DMA + SM0, deassert DE.
async fn do_tx(
    tx_data: &mut LtTxData<'_>,
    tx_enable: &mut LtTxEnable<'_>,
    fm0: &Fm0Buffer,
) {
    let word_count = pack_into_dma_buf(fm0);

    // Assert DE.
    tx_enable.de.set_level(Level::High);
    cortex_m::asm::delay(625); // ~5 µs settle

    // Start DMA + SM0.
    start_tx_dma(word_count);
    tx_data.sm.set_enable(true);

    // Wait for completion.
    wait_tx_dma().await;
    wait_fifo_drain(&mut tx_data.sm).await;

    // Wait for last bits to shift out of OSR.
    let last_bits = fm0.len() % 32;
    let last_bits = if last_bits == 0 { 32 } else { last_bits };
    Timer::after_micros((last_bits as u64 + 2) * 2).await;

    // Stop SM0.
    tx_data.sm.set_enable(false);
    abort_tx_dma();
    tx_data.sm.restart();

    // Deassert DE.
    tx_enable.de.set_level(Level::Low);
}

/// Wait for bus idle + CSMA/CA backoff. Returns `false` if the frame
/// is invalid. Call this BEFORE acquiring the TX lock so that
/// auto-reply CTS/ACK frames from the RX task aren't blocked during
/// the (potentially long) CSMA wait.
pub async fn csma_wait(
    backoff: &mut BackoffState,
    frame: &[u8],
) -> bool {
    if frame.len() < 5 || frame.len() > MAX_FRAME_LEN {
        warn!("TX: invalid frame");
        return false;
    }

    backoff.prep_for_next_frame();

    // Wait for bus idle + CSMA backoff.
    wait_bus_idle().await;
    let entropy = (Instant::now().as_micros() as u8)
        ^ ((Instant::now().as_ticks() >> 8) as u8);
    let delay_units = backoff.next_backoff_units(entropy);
    let delay_us = delay_units as u64 * 100;
    wait_idle_for(delay_us).await;

    true
}

/// Build FM-0 and transmit. Called after csma_wait, with TX locks held.
pub async fn transmit_frame(
    tx_data: &mut LtTxData<'_>,
    tx_enable: &mut LtTxEnable<'_>,
    frame: &[u8],
) {
    let Some(fm0_buf) = build_tx_buffer(frame) else {
        warn!("TX: build failed");
        return;
    };
    do_tx(tx_data, tx_enable, &fm0_buf).await;
}

/// Transmit a frame immediately (no CSMA/CA).
pub async fn transmit_immediate(
    tx_data: &mut LtTxData<'_>,
    tx_enable: &mut LtTxEnable<'_>,
    frame: &[u8],
) {
    let Some(fm0_buf) = build_immediate_tx_buffer(frame) else {
        warn!("TX immediate: invalid frame");
        return;
    };

    do_tx(tx_data, tx_enable, &fm0_buf).await;
}

// ── Helpers ─────────────────────────────────────────────────────────

async fn wait_bus_idle() {
    while carrier_sense_active() {
        Timer::after_micros(10).await;
    }
}

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
