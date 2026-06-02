//! Core 1 — bare-metal wire engine.
//!
//! Runs on the RP2040's second core with NO Embassy executor, NO async,
//! NO scheduler. Owns all LocalTalk wire interaction: PIO0 SM0 (TX),
//! PIO1 SM0 (RX), DMA CH1 (TX), DE pin (GP3), carrier sense (GP5).
//!
//! Communication with core 0 is via shared memory with atomic flag bits.

#![allow(dead_code)]

use core::cell::UnsafeCell;
use core::sync::atomic::{compiler_fence, Ordering};

use embassy_rp::pac;
use embassy_rp::pac::dma::vals::{DataSize, TreqSel};
use portable_atomic::AtomicU32;
use tashtalk_proto::backoff::BackoffState;
use tashtalk_proto::bitstuff::{BitUnstuffer, UnstuffEvent};
use tashtalk_proto::codec::MAX_FRAME_LEN;
use tashtalk_proto::bitstuff::FLAG;
use tashtalk_proto::crc::CrcCalculator;
use tashtalk_proto::fm0::Fm0Encoder;
use tashtalk_proto::llap::{auto_reply, cts_matches_pending, frame_crc_ok, AsmEvent, FrameAssembler};
use tashtalk_proto::nodemap::NodeBitmap;

// ── Atomic flag bits ───────────────────────────────────────────────
pub const TX_SUBMIT: u32     = 1 << 0;
pub const TX_DONE: u32       = 1 << 1;
pub const TX_RESULT_FAIL: u32 = 1 << 2;
pub const RX_READY: u32      = 1 << 3;
pub const NODES_UPDATED: u32 = 1 << 4;
pub const CORE1_ALIVE: u32   = 1 << 5;

// ── Hardware constants ─────────────────────────────────────────────
const DE_PIN: u32 = 3;
const CARRIER_SENSE_PIN: u32 = 5;
const DBG0_PIN: u32 = 6;
const DBG1_PIN: u32 = 7;
const TX_DMA_CH: usize = 1;
const RX_DMA_CH: usize = 0;
const DREQ_PIO0_TX0: u8 = 0;
const DREQ_PIO1_RX0: u8 = 12;

// ── RX DMA ring buffer ───────────────────────────────────────────
const RX_RING_SIZE: usize = 4096;
const RX_RING_MASK: usize = RX_RING_SIZE - 1;
#[repr(C, align(16384))] // 4096 * 4 = 16KB
pub struct RxRingBuf(pub [u32; RX_RING_SIZE]);
pub static mut RX_RING: RxRingBuf = RxRingBuf([0u32; RX_RING_SIZE]);
static mut RX_READ_IDX: usize = 0;

// ── Debug: last CRC-failed frame info ─────────────────────────────
pub static mut LAST_BAD_FRAME: [u8; 8] = [0u8; 8];
pub static mut LAST_BAD_LEN: u16 = 0;
pub static mut BAD_CRC_COUNT: u32 = 0;
pub static mut GOOD_CRC_COUNT: u32 = 0;
pub static mut AUTOREPLY_COUNT: u32 = 0;
pub static mut LAST_GOOD_FRAME: [u8; 8] = [0u8; 8];
pub static mut LAST_GOOD_LEN: u16 = 0;
/// Count of CRC-failed frames with len > 5 (likely real data, not garbage).
pub static mut BAD_REAL_COUNT: u32 = 0;
pub static mut RING_OVERRUN_COUNT: u32 = 0;
pub static mut DIRECTED_TX_ATTEMPTS: u32 = 0;

/// Zero all debug counters. Called from core 0's debug shell.
pub fn clear_counters() {
    unsafe {
        GOOD_CRC_COUNT = 0;
        BAD_CRC_COUNT = 0;
        AUTOREPLY_COUNT = 0;
        LAST_GOOD_LEN = 0;
        LAST_GOOD_FRAME = [0u8; 8];
        LAST_BAD_LEN = 0;
        LAST_BAD_FRAME = [0u8; 8];
        BAD_REAL_COUNT = 0;
        RING_OVERRUN_COUNT = 0;
        DIRECTED_TX_ATTEMPTS = 0;
        DIRECTED_TX_CTS_OK = 0;
        DIRECTED_TX_TIMEOUT = 0;
        DIRECTED_RX_COUNT = 0;
        ECHO_SKIP_COUNT = 0;
    }
}
pub static mut DIRECTED_TX_CTS_OK: u32 = 0;
pub static mut DIRECTED_TX_TIMEOUT: u32 = 0;
/// Frames received where dest is in our node bitmap (Mac → us).
pub static mut DIRECTED_RX_COUNT: u32 = 0;
/// Frames skipped by echo filter.
pub static mut ECHO_SKIP_COUNT: u32 = 0;

// ── DMA buffers (core 1 exclusive) ────────────────────────────────
pub const TX_DMA_BUF_WORDS: usize = 512;
static mut AUTOREPLY_DMA_BUF: [u32; 16] = [0u32; 16];


// ── RX constants ──────────────────────────────────────────────────
const INTERVAL_THRESHOLD: u32 = 16;

#[derive(Clone, Copy, PartialEq, Eq)]
enum Fm0State {
    Boundary,
    MidCell,
}

// ── Shared state ───────────────────────────────────────────────────

/// Pre-encoded frame for CSMA TX, written by core 0, read by core 1.
#[repr(C)]
pub struct TxSlot {
    /// Pre-packed DMA words for the DATA frame.
    pub dma_buf: [u32; TX_DMA_BUF_WORDS],
    /// Number of valid DMA words.
    pub word_count: u16,
    /// Total FM-0 symbol count (for last-bits timing).
    pub fm0_len: u16,
    /// Pre-packed DMA words for the RTS frame (directed TX only).
    pub rts_dma_buf: [u32; 16],
    /// RTS DMA word count.
    pub rts_word_count: u16,
    /// RTS FM-0 symbol count.
    pub rts_fm0_len: u16,
    /// LLAP destination (for CTS matching).
    pub frame_dest: u8,
    /// LLAP source (for CTS matching).
    pub frame_src: u8,
    /// True if this is a directed data frame needing RTS/CTS dialog.
    pub is_directed: bool,
}

/// Received frame, written by core 1, read by core 0.
#[repr(C)]
pub struct RxSlot {
    pub buf: [u8; MAX_FRAME_LEN],
    pub len: u16,
    /// 0=good, 1=bad_crc
    pub status: u8,
}

/// Shared state between core 0 and core 1.
///
/// Fields wrapped in UnsafeCell have a single writer at any time,
/// enforced by the atomic flag protocol.
#[repr(C, align(32))]
pub struct SharedState {
    // Core 0 writes, core 1 reads (synchronized by TX_SUBMIT)
    pub tx_slot: UnsafeCell<TxSlot>,
    // Core 0 writes, core 1 reads (synchronized by NODES_UPDATED)
    pub node_bitmap: UnsafeCell<[u8; 32]>,

    // Core 1 writes, core 0 reads (synchronized by RX_READY)
    pub rx_slot: UnsafeCell<RxSlot>,

    // Both cores (atomic)
    pub flags: AtomicU32,
}

unsafe impl Sync for SharedState {}

impl SharedState {
    pub const fn new() -> Self {
        Self {
            tx_slot: UnsafeCell::new(TxSlot {
                dma_buf: [0u32; TX_DMA_BUF_WORDS],
                word_count: 0,
                fm0_len: 0,
                rts_dma_buf: [0u32; 16],
                rts_word_count: 0,
                rts_fm0_len: 0,
                frame_dest: 0,
                frame_src: 0,
                is_directed: false,
            }),
            node_bitmap: UnsafeCell::new([0u8; 32]),
            rx_slot: UnsafeCell::new(RxSlot {
                buf: [0u8; MAX_FRAME_LEN],
                len: 0,
                status: 0,
            }),
            flags: AtomicU32::new(0),
        }
    }
}

// ── RX via DMA ring buffer ────────────────────────────────────────
//
// DMA CH0 continuously drains PIO1 SM0 RX FIFO into a ring buffer.
// This runs even when core 1 is busy transmitting, preventing overflow.

fn init_rx_dma() {
    let ch = pac::DMA.ch(RX_DMA_CH);
    ch.ctrl_trig().write(|w| w.set_en(false));
    // Read from PIO1 SM0 RX FIFO (fixed address).
    ch.read_addr().write_value(pac::PIO1.rxf(0).as_ptr() as u32);
    // Write into ring buffer (incrementing, wraps via ring mode).
    ch.write_addr().write_value(unsafe { RX_RING.0.as_ptr() } as u32);
    ch.trans_count().write_value(0xFFFF_FFFF);
    compiler_fence(Ordering::SeqCst);
    ch.ctrl_trig().write(|w| {
        w.set_treq_sel(TreqSel(DREQ_PIO1_RX0));
        w.set_data_size(DataSize::SIZE_WORD);
        w.set_incr_read(false);
        w.set_incr_write(true);
        w.set_ring_sel(true); // ring on write address
        w.set_ring_size(14);  // 2^14 = 16384 bytes = 4096 u32 entries
        w.set_chain_to(RX_DMA_CH as u8);
        w.set_en(true);
    });
    compiler_fence(Ordering::SeqCst);
}

#[inline(always)]
fn try_read_rx() -> Option<u32> {
    // DMA write_addr points to the NEXT location to write.
    // Compare to our read pointer to see if data is available.
    let write_addr = pac::DMA.ch(RX_DMA_CH).write_addr().read() as usize;
    let base = unsafe { RX_RING.0.as_ptr() } as usize;
    let write_idx = (write_addr.wrapping_sub(base)) / 4;

    let read_idx = unsafe { RX_READ_IDX };
    if read_idx == write_idx {
        return None;
    }

    // Detect overrun: if distance > half the ring, DMA has lapped us.
    let distance = (write_idx.wrapping_sub(read_idx)) & RX_RING_MASK;
    if distance > RX_RING_SIZE / 2 {
        unsafe {
            RING_OVERRUN_COUNT += 1;
            RX_READ_IDX = write_idx;
        }
        return None;
    }

    let val = unsafe { core::ptr::read_volatile(&RX_RING.0[read_idx]) };
    unsafe { RX_READ_IDX = (read_idx + 1) & RX_RING_MASK; }
    Some(val)
}

/// Diagnostic: return (ring_base, write_addr, read_idx, dma_enabled, trans_remaining).
pub fn rx_dma_diag() -> (u32, u32, u32, bool, u32) {
    let ch = pac::DMA.ch(RX_DMA_CH);
    let base = unsafe { RX_RING.0.as_ptr() } as u32;
    let waddr = ch.write_addr().read();
    let ridx = unsafe { RX_READ_IDX } as u32;
    let en = ch.ctrl_trig().read().en();
    let remain = ch.trans_count().read();
    (base, waddr, ridx, en, remain)
}

// ── PAC: TX (PIO0 SM0 + DMA + DE) ────────────────────────────────

#[inline(always)]
fn set_de(high: bool) {
    if high {
        pac::SIO.gpio_out(0).value_set().write_value(1 << DE_PIN);
    } else {
        pac::SIO.gpio_out(0).value_clr().write_value(1 << DE_PIN);
    }
}

#[inline(always)]
fn carrier_sense_active() -> bool {
    pac::SIO.gpio_in(0).read() & (1 << CARRIER_SENSE_PIN) != 0
}

fn enable_pio0_sm0() {
    pac::PIO0.ctrl().modify(|w| {
        w.set_sm_enable(w.sm_enable() | 1); // set bit 0 = SM0
    });
}

fn disable_pio0_sm0() {
    pac::PIO0.ctrl().modify(|w| {
        w.set_sm_enable(w.sm_enable() & !1); // clear bit 0
    });
}

fn restart_pio0_sm0() {
    pac::PIO0.ctrl().modify(|w| {
        w.set_sm_restart(1); // bit 0 = SM0
    });
}

fn pio0_tx_fifo_empty() -> bool {
    pac::PIO0.fstat().read().txempty() & 1 != 0
}

fn start_tx_dma(read_addr: *const u32, word_count: usize) {
    let ch = pac::DMA.ch(TX_DMA_CH);
    ch.ctrl_trig().write(|w| w.set_en(false));
    ch.read_addr().write_value(read_addr as u32);
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

fn wait_tx_dma() {
    let ch = pac::DMA.ch(TX_DMA_CH);
    let deadline = read_timer_us() + 500_000; // 500ms safety
    while ch.ctrl_trig().read().busy() {
        if read_timer_us() >= deadline {
            ch.ctrl_trig().write(|w| w.set_en(false));
            return;
        }
    }
}

fn abort_tx_dma() {
    pac::DMA.ch(TX_DMA_CH).ctrl_trig().write(|w| w.set_en(false));
}

/// Shift out pre-packed DMA words via PIO0 SM0 with DE control.
///
/// RX data arriving during TX is captured by the RX DMA ring buffer
/// (DMA CH0 runs autonomously). No FIFO overflow, no data loss.
fn transmit_dma(read_addr: *const u32, word_count: usize, fm0_len: usize) {
    set_de(true);
    cortex_m::asm::delay(625); // ~5 µs settle

    start_tx_dma(read_addr, word_count);
    enable_pio0_sm0();

    wait_tx_dma();

    // Wait for TX FIFO to drain.
    let drain_deadline = read_timer_us() + 500_000;
    while !pio0_tx_fifo_empty() {
        if read_timer_us() >= drain_deadline { break; }
    }

    // Wait for last bits to shift out of OSR.
    let last_bits = fm0_len % 32;
    let last_bits = if last_bits == 0 { 32 } else { last_bits };
    busy_wait_us((last_bits as u64 + 2) * 3);

    disable_pio0_sm0();
    abort_tx_dma();
    restart_pio0_sm0();

    set_de(false);
    // With a 16KB ring buffer (4096 entries, ~27ms at full edge rate),
    // no echo can overflow it. The decoder processes the echo as
    // normal frames/garbage, re-syncs on the idle gap timeout, then
    // cleanly decodes whatever follows (e.g. Mac's DATA after our CTS).
}

/// Build HDLC + FM-0 for a small control frame (5 bytes with CRC)
/// directly into AUTOREPLY_DMA_BUF. No large stack allocations.
/// Returns (word_count, fm0_symbol_count).
fn build_small_frame_dma(frame: &[u8; 5]) -> (usize, usize) {
    // Build HDLC bitstream into a small stack buffer.
    // 5-byte frame: 2 flags (16 bits) + ~42 stuffed bits + flag (8) + abort (16) < 128 bits
    let mut bits = [0u8; 256];
    let mut bpos = 0;

    // Helper: push one bit
    macro_rules! push_bit {
        ($b:expr) => { bits[bpos] = $b; bpos += 1; }
    }

    // Two opening flags (0x7E = 01111110)
    for _ in 0..2 {
        for i in 0..8 {
            push_bit!((FLAG >> i) & 1);
        }
    }

    // Bit-stuff payload + CRC
    let mut crc = CrcCalculator::new();
    // Frame already includes CRC bytes (5 bytes: 3 payload + 2 CRC)
    let mut ones_count = 0u8;
    for &byte in frame.iter() {
        crc.feed_byte(byte); // CRC runs over everything including CRC bytes
        for i in 0..8 {
            let bit = (byte >> i) & 1;
            push_bit!(bit);
            if bit == 1 {
                ones_count += 1;
                if ones_count == 5 {
                    push_bit!(0); // stuff zero
                    ones_count = 0;
                }
            } else {
                ones_count = 0;
            }
        }
    }

    // Closing flag
    for i in 0..8 {
        push_bit!((FLAG >> i) & 1);
    }

    // Abort (16 ones)
    for _ in 0..16 {
        push_bit!(1);
    }

    // FM-0 encode into DMA words
    let mut enc = Fm0Encoder::new();
    let mut fm0_count = 0usize;
    let mut word_count = 0usize;
    let mut current_word = 0u32;
    let mut bit_in_word = 0;

    for &bit in &bits[..bpos] {
        let (boundary, mid) = enc.encode_bit(bit);
        for &sym in &[boundary, mid] {
            current_word |= (sym as u32) << bit_in_word;
            bit_in_word += 1;
            fm0_count += 1;
            if bit_in_word == 32 {
                unsafe { AUTOREPLY_DMA_BUF[word_count] = current_word; }
                word_count += 1;
                current_word = 0;
                bit_in_word = 0;
            }
        }
    }
    // Flush partial word
    if bit_in_word > 0 {
        unsafe { AUTOREPLY_DMA_BUF[word_count] = current_word; }
        word_count += 1;
    }

    (word_count, fm0_count)
}

/// Build and transmit a small control frame (auto-reply).
fn transmit_small_frame(frame: &[u8; 5]) {
    let (word_count, fm0_len) = build_small_frame_dma(frame);
    transmit_dma(
        unsafe { AUTOREPLY_DMA_BUF.as_ptr() },
        word_count,
        fm0_len,
    );
}

// ── Debug GPIO ────────────────────────────────────────────────────

fn init_debug_gpio() {
    let pads = pac::PADS_BANK0;
    let io = pac::IO_BANK0;
    for pin in [DBG0_PIN, DBG1_PIN] {
        pads.gpio(pin as usize).modify(|w| { w.set_ie(true); w.set_od(false); });
        io.gpio(pin as usize).ctrl().write(|w| w.set_funcsel(5));
        pac::SIO.gpio_oe(0).value_set().write_value(1 << pin);
        pac::SIO.gpio_out(0).value_clr().write_value(1 << pin);
    }
}

#[inline(always)]
fn dbg1_high() {
    pac::SIO.gpio_out(0).value_set().write_value(1 << DBG1_PIN);
}

#[inline(always)]
fn dbg1_low() {
    pac::SIO.gpio_out(0).value_clr().write_value(1 << DBG1_PIN);
}

#[inline(always)]
fn dbg0_high() {
    pac::SIO.gpio_out(0).value_set().write_value(1 << DBG0_PIN);
}

#[inline(always)]
fn dbg0_low() {
    pac::SIO.gpio_out(0).value_clr().write_value(1 << DBG0_PIN);
}

// ── Timer ─────────────────────────────────────────────────────────

#[inline(always)]
fn read_timer_us() -> u64 {
    let timer = pac::TIMER;
    let lo = timer.timelr().read() as u64;
    let hi = timer.timehr().read() as u64;
    (hi << 32) | lo
}

fn busy_wait_us(us: u64) {
    let start = read_timer_us();
    while read_timer_us() - start < us {
        cortex_m::asm::nop();
    }
}

// ── CSMA/CA helpers ───────────────────────────────────────────────

/// Wait until carrier sense is inactive (bus idle). Blocks.
fn wait_bus_idle() {
    while carrier_sense_active() {
        cortex_m::asm::nop();
    }
}

/// Wait for the bus to stay idle for `us` microseconds.
/// Returns false if carrier sense fires during the wait (deferral).
fn wait_idle_for(us: u64) -> bool {
    let deadline = read_timer_us() + us;
    while read_timer_us() < deadline {
        if carrier_sense_active() {
            return false;
        }
    }
    !carrier_sense_active()
}

/// CSMA/CA wait + transmit. Dispatches to directed (RTS/CTS/DATA)
/// or broadcast path based on tx_slot.is_directed.
fn csma_transmit(shared: &SharedState, backoff: &mut BackoffState) -> bool {
    // SAFETY: TX_SUBMIT was set, so core 0 is not writing tx_slot.
    let slot = unsafe { &*shared.tx_slot.get() };

    if slot.is_directed {
        return directed_tx(shared, slot, backoff);
    }

    // Broadcast / control frame — simple CSMA then TX.
    backoff.prep_for_next_frame();

    loop {
        if backoff.exhausted() {
            return false;
        }

        wait_bus_idle();

        let entropy = (read_timer_us() as u8)
            ^ ((read_timer_us() >> 8) as u8);
        let delay_units = backoff.next_backoff_units(entropy);
        let delay_us = delay_units as u64 * 100;
        if !wait_idle_for(delay_us) {
            backoff.record_deferral();
            backoff.consume_attempt();
            continue;
        }

        transmit_dma(
            slot.dma_buf.as_ptr(),
            slot.word_count as usize,
            slot.fm0_len as usize,
        );
        return true;
    }
}

/// Directed data TX: CSMA → RTS → wait CTS → DATA.
///
/// The RTS and DATA frames are both pre-encoded in tx_slot by core 0.
/// This function runs the full dialog in a tight loop with no scheduler
/// involvement — the CTS-to-DATA transition is sub-microsecond.
fn directed_tx(
    shared: &SharedState,
    slot: &TxSlot,
    backoff: &mut BackoffState,
) -> bool {
    let our_src = slot.frame_src;
    let their_dest = slot.frame_dest;

    unsafe { DIRECTED_TX_ATTEMPTS += 1; }
    backoff.prep_for_next_frame();

    loop {
        if backoff.exhausted() {
            return false;
        }

        // ── CSMA wait ─────────────────────────────────────────────
        wait_bus_idle();

        let entropy = (read_timer_us() as u8)
            ^ ((read_timer_us() >> 8) as u8);
        let delay_units = backoff.next_backoff_units(entropy);
        let delay_us = delay_units as u64 * 100;
        if !wait_idle_for(delay_us) {
            backoff.record_deferral();
            backoff.consume_attempt();
            continue;
        }

        // ── Send RTS ──────────────────────────────────────────────
        transmit_dma(
            slot.rts_dma_buf.as_ptr(),
            slot.rts_word_count as usize,
            slot.rts_fm0_len as usize,
        );

        // ── Wait for CTS (tight polling loop) ─────────────────────
        // 400 µs generous window (PIC uses 200 µs).
        let cts_deadline = read_timer_us() + 400;
        let mut cts_fm0 = Fm0State::Boundary;
        let mut cts_unstuffer = BitUnstuffer::new();
        let mut cts_assembler = FrameAssembler::new();
        let mut got_cts = false;

        loop {
            if read_timer_us() > cts_deadline {
                break; // Timeout — no CTS.
            }

            if let Some(w) = try_read_rx() {
                let remaining = w & 0x1F;
                if remaining == 0 {
                    cts_fm0 = Fm0State::Boundary;
                    continue;
                }

                let is_short = remaining > INTERVAL_THRESHOLD;
                let bit = match cts_fm0 {
                    Fm0State::Boundary => {
                        if is_short {
                            cts_fm0 = Fm0State::MidCell;
                            Some(0u8)
                        } else {
                            Some(1u8)
                        }
                    }
                    Fm0State::MidCell => {
                        cts_fm0 = Fm0State::Boundary;
                        if !is_short {
                            cts_unstuffer.reset();
                            cts_assembler.feed_abort();
                        }
                        None
                    }
                };

                if let Some(bit) = bit {
                    match cts_unstuffer.feed_bit(bit) {
                        UnstuffEvent::Byte(b) => {
                            cts_assembler.feed_byte(b);
                        }
                        UnstuffEvent::Flag => {
                            if let Some(AsmEvent::FrameEnd) = cts_assembler.feed_flag() {
                                let frame = cts_assembler.current();
                                if frame_crc_ok(frame)
                                    && cts_matches_pending(frame, our_src, their_dest)
                                {
                                    // Forward CTS to host if rx_slot free.
                                    if shared.flags.load(Ordering::Relaxed) & RX_READY == 0 {
                                        let n = frame.len().min(MAX_FRAME_LEN);
                                        let rx = unsafe { &mut *shared.rx_slot.get() };
                                        rx.buf[..n].copy_from_slice(&frame[..n]);
                                        rx.len = n as u16;
                                        rx.status = 0;
                                        compiler_fence(Ordering::SeqCst);
                                        shared.flags.fetch_or(RX_READY, Ordering::Release);
                                    }
                                    got_cts = true;
                                    unsafe { DIRECTED_TX_CTS_OK += 1; }
                                }
                                cts_assembler.ack_frame_end();
                            }
                        }
                        UnstuffEvent::Abort => {
                            cts_assembler.feed_abort();
                            cts_fm0 = Fm0State::Boundary;
                            cts_unstuffer.reset();
                        }
                        UnstuffEvent::None => {}
                    }
                }
            }

            if got_cts {
                break;
            }
        }

        if got_cts {
            // ── Send DATA immediately ─────────────────────────────
            // CTS-to-DATA: only register writes = sub-microsecond.
            transmit_dma(
                slot.dma_buf.as_ptr(),
                slot.word_count as usize,
                slot.fm0_len as usize,
            );
            return true;
        }

        // No CTS — collision. Retry.
        unsafe { DIRECTED_TX_TIMEOUT += 1; }
        backoff.record_collision();
        backoff.consume_attempt();
    }
}

// ── Core 1 entry point ─────────────────────────────────────────────

pub fn core1_main(shared: &'static SharedState) -> ! {
    init_debug_gpio();
    init_rx_dma();

    shared.flags.fetch_or(CORE1_ALIVE, Ordering::Release);
    compiler_fence(Ordering::SeqCst);

    // Local state — entirely owned by core 1.
    let mut fm0_state = Fm0State::Boundary;
    let mut unstuffer = BitUnstuffer::new();
    let mut assembler = FrameAssembler::new();
    let mut backoff = BackoffState::new();
    let mut nodes = NodeBitmap::empty();
    let mut frames_seen: u32 = 0;

    loop {
        // ── Check node bitmap update (rare) ───────────────────────
        if shared.flags.load(Ordering::Relaxed) & NODES_UPDATED != 0 {
            compiler_fence(Ordering::SeqCst);
            // SAFETY: NODES_UPDATED set → core 0 finished writing.
            let bm = unsafe { &*shared.node_bitmap.get() };
            nodes = NodeBitmap::from_bytes(*bm);
            shared.flags.fetch_and(!NODES_UPDATED, Ordering::Release);
        }

        // ── Poll PIO1 RX FIFO (hot path) ─────────────────────────
        match try_read_rx() {
            Some(w) => {
                let remaining = w & 0x1F;

                if remaining == 0 {
                    fm0_state = Fm0State::Boundary;
                    continue;
                }

                let is_short = remaining > INTERVAL_THRESHOLD;

                let bit = match fm0_state {
                    Fm0State::Boundary => {
                        if is_short {
                            fm0_state = Fm0State::MidCell;
                            Some(0u8)
                        } else {
                            Some(1u8)
                        }
                    }
                    Fm0State::MidCell => {
                        fm0_state = Fm0State::Boundary;
                        if !is_short {
                            unstuffer.reset();
                            assembler.feed_abort();
                        }
                        None
                    }
                };

                if let Some(bit) = bit {
                    match unstuffer.feed_bit(bit) {
                        UnstuffEvent::Byte(b) => {
                            assembler.feed_byte(b);
                        }
                        UnstuffEvent::Flag => {
                            if let Some(AsmEvent::FrameEnd) = assembler.feed_flag() {
                                frames_seen = frames_seen.wrapping_add(1);
                                let frame = assembler.current();
                                let crc_ok = frame_crc_ok(frame);

                                // Echo suppression: skip frames where src
                                // is one of our nodes (self-echo via RS-422).
                                let is_echo = frame.len() >= 2
                                    && nodes.contains(frame[1]);

                                // Record stats (skip echoes).
                                if is_echo {
                                    unsafe { ECHO_SKIP_COUNT += 1; }
                                    assembler.ack_frame_end();
                                    continue;
                                }

                                // Count directed frames (Mac → us).
                                if frame.len() >= 1 && nodes.contains(frame[0]) {
                                    unsafe { DIRECTED_RX_COUNT += 1; }
                                }

                                if crc_ok {
                                    unsafe {
                                        GOOD_CRC_COUNT += 1;
                                        LAST_GOOD_LEN = frame.len() as u16;
                                        let n = frame.len().min(8);
                                        LAST_GOOD_FRAME = [0u8; 8];
                                        LAST_GOOD_FRAME[..n].copy_from_slice(&frame[..n]);
                                    }
                                } else {
                                    unsafe {
                                        BAD_CRC_COUNT += 1;
                                        if frame.len() > 5 { BAD_REAL_COUNT += 1; }
                                        LAST_BAD_LEN = frame.len() as u16;
                                        let n = frame.len().min(8);
                                        LAST_BAD_FRAME = [0u8; 8];
                                        LAST_BAD_FRAME[..n].copy_from_slice(&frame[..n]);
                                    }
                                }

                                // Check auto-reply BEFORE forwarding.
                                let reply = if crc_ok {
                                    auto_reply(frame, &nodes).into_frame()
                                } else {
                                    None
                                };

                                // Don't forward RTS/CTS control frames to the
                                // host — the original TashTalk handles them
                                // internally. TailTalk doesn't expect them.
                                // Forward to core 0 via rx_slot.
                                // Only forward frames with good CRC — bad CRC
                                // frames are useless to TailTalk and produce
                                // noise errors that obscure real issues.
                                if crc_ok
                                    && shared.flags.load(Ordering::Relaxed) & RX_READY == 0
                                {
                                    let n = frame.len().min(MAX_FRAME_LEN);
                                    let slot = unsafe { &mut *shared.rx_slot.get() };
                                    slot.buf[..n].copy_from_slice(&frame[..n]);
                                    slot.len = n as u16;
                                    slot.status = 0; // good CRC
                                    compiler_fence(Ordering::SeqCst);
                                    shared.flags.fetch_or(RX_READY, Ordering::Release);
                                }

                                assembler.ack_frame_end();

                                // Auto-reply TX (immediate, no CSMA).
                                if let Some(reply_frame) = reply {
                                    unsafe { AUTOREPLY_COUNT += 1; }
                                    dbg0_high();
                                    transmit_small_frame(&reply_frame);
                                    dbg0_low();
                                }
                            }
                        }
                        UnstuffEvent::Abort => {
                            assembler.feed_abort();
                            fm0_state = Fm0State::Boundary;
                            unstuffer.reset();
                        }
                        UnstuffEvent::None => {}
                    }
                }
            }
            None => {
                // ── FIFO empty: check for host TX request ─────────
                let flags = shared.flags.load(Ordering::Relaxed);
                if flags & TX_SUBMIT != 0 {
                    shared.flags.fetch_and(!TX_SUBMIT, Ordering::Acquire);
                    compiler_fence(Ordering::SeqCst);

                    let ok = csma_transmit(shared, &mut backoff);

                    // Signal completion to core 0.
                    let result_bits = TX_DONE | if ok { 0 } else { TX_RESULT_FAIL };
                    shared.flags.fetch_and(!(TX_DONE | TX_RESULT_FAIL), Ordering::Relaxed);
                    shared.flags.fetch_or(result_bits, Ordering::Release);
                }
            }
        }
    }
}
