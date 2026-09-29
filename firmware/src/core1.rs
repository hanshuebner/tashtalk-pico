//! Core 1 — bare-metal wire engine.
//!
//! Runs on the RP2040's second core with NO Embassy executor, NO async,
//! NO scheduler. Owns all LocalTalk wire interaction: PIO0 SM0 (TX),
//! PIO1 SM0 (RX FM-0 decoder), DMA CH0 (RX ring), DMA CH1 (TX), DE (GP3).
//!
//! RX: PIO1 SM0 decodes FM-0 in hardware at 16× oversampling (3.6864 MHz).
//! Each autopush word contains 8 decoded bits in [31:24]. Frame-end is
//! signaled by a 0xFFFFFFFF marker word. Core 1 feeds decoded bits
//! through BitUnstuffer → FrameAssembler in a tight poll loop.
//!
//! Communication with core 0 is via shared memory, with atomic flag bits
//! for TX and the node bitmap and a small ring for received frames.

#![allow(dead_code)]

use core::cell::UnsafeCell;
use core::sync::atomic::{compiler_fence, Ordering};

use embassy_rp::pac;
use embassy_rp::pac::dma::vals::{DataSize, TreqSel};
use portable_atomic::AtomicU32;
use tashtalk_proto::backoff::BackoffState;
use tashtalk_proto::bitstuff::{BitUnstuffer, UnstuffEvent, FLAG};
use tashtalk_proto::codec::MAX_FRAME_LEN;
use tashtalk_proto::crc::CrcCalculator;
use tashtalk_proto::fm0::Fm0Encoder;
use tashtalk_proto::llap::{auto_reply, cts_matches_pending, frame_crc_ok, AsmEvent, FrameAssembler};
use tashtalk_proto::nodemap::NodeBitmap;

// ── Atomic flag bits ───────────────────────────────────────────────
pub const TX_SUBMIT: u32     = 1 << 0;
pub const TX_DONE: u32       = 1 << 1;
pub const TX_RESULT_FAIL: u32 = 1 << 2;
// Bit 3 used to be RX_READY, the handshake for the old single RX slot.
// Received frames go through the rx_slots ring now.
pub const NODES_UPDATED: u32 = 1 << 4;
pub const CORE1_ALIVE: u32   = 1 << 5;

// ── Hardware constants ─────────────────────────────────────────────
const DE_PIN: u32 = 3;
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
/// Core 1 main loop iteration counter (heartbeat).
pub static mut LOOP_COUNT: u32 = 0;
pub static mut RING_OVERRUN_COUNT: u32 = 0;
pub static mut DIRECTED_TX_ATTEMPTS: u32 = 0;
/// Frames we dropped because core 0 hadn't emptied the RX slot ring yet.
pub static mut RX_DROP_COUNT: u32 = 0;
/// Auto-replies we dropped because the line didn't go quiet in time.
pub static mut AUTOREPLY_LATE: u32 = 0;
/// The longest we've waited between a closing flag and the line going
/// quiet before sending a reply.
pub static mut AUTOREPLY_WAIT_MAX_US: u32 = 0;
/// How many times wait_bus_idle gave up on a carrier that never dropped.
pub static mut BUS_IDLE_TIMEOUTS: u32 = 0;

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
        RX_DROP_COUNT = 0;
        AUTOREPLY_LATE = 0;
        AUTOREPLY_WAIT_MAX_US = 0;
        BUS_IDLE_TIMEOUTS = 0;
    }
}
pub static mut DIRECTED_TX_CTS_OK: u32 = 0;
pub static mut DIRECTED_TX_TIMEOUT: u32 = 0;
/// Timestamp of last CTS wait start (µs since boot).
pub static mut LAST_CTS_WAIT_START: u64 = 0;
/// Timestamp when CTS was received or deadline expired.
pub static mut LAST_CTS_WAIT_END: u64 = 0;
/// Number of ring entries processed during last CTS wait.
pub static mut LAST_CTS_ENTRIES: u32 = 0;
/// Frames received where dest is in our node bitmap (Mac → us).
pub static mut DIRECTED_RX_COUNT: u32 = 0;
/// Frames skipped by echo filter.
pub static mut ECHO_SKIP_COUNT: u32 = 0;

// ── DMA buffers (core 1 exclusive) ────────────────────────────────
pub const TX_DMA_BUF_WORDS: usize = 512;
static mut AUTOREPLY_DMA_BUF: [u32; 16] = [0u32; 16];


// ── RX constants ──────────────────────────────────────────────────
/// Frame-end marker pushed by PIO FM-0 decoder when idle timeout fires.
const FRAME_END_MARKER: u32 = 0xFFFF_FFFF;

/// Microsecond timestamp of last wire activity (decoded bits from PIO).
/// Used for software carrier sense in CSMA/CA.
static mut LAST_ACTIVITY_US: u64 = 0;
/// Bus is considered idle if no activity for this many microseconds.
/// One bit cell is ~4.34µs; a full flag byte is ~35µs. 50µs of silence
/// means at least one full byte time with no transitions.
const CARRIER_IDLE_US: u64 = 50;

/// How long after a frame's closing flag we can still send its auto-reply.
///
/// The sender keeps driving the line for its abort sequence (12-18 one bits,
/// up to ~78µs) after the closing flag, so we hold the reply until the line
/// goes quiet. Inside AppleTalk gives the responder 200µs. If the line is
/// still busy after that it's probably someone else's frame, and the sender
/// will retry anyway.
const REPLY_WINDOW_US: u64 = 200;

/// How long to wait for a peer's CTS to start arriving after our RTS.
///
/// Inside AppleTalk gives the responder 200µs to answer an RTS. We only see
/// a word once 8 bits of it have been decoded, so the extra margin covers
/// that latency on our side and the peer's decode latency on theirs. Once we
/// see any activity the deadline gets extended so the whole frame has time
/// to arrive.
const CTS_START_WINDOW_US: u64 = 400;

/// The longest wait_bus_idle will wait for the carrier to drop before giving
/// up. Without this a transceiver fault or a jabbering node could keep core 1
/// stuck in that loop forever, so we let the backoff deal with it instead.
const MAX_BUS_IDLE_WAIT_US: u64 = 500_000;

/// How many received frames core 1 can hold for core 0. We forward every good
/// frame, RTS and CTS included, and a dialog's data frame follows its RTS
/// within 200µs. With a single slot that data frame gets dropped if core 0
/// hasn't collected the RTS yet.
pub const RX_SLOT_COUNT: usize = 4;

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

    // Core 1 writes, core 0 reads. It's a ring indexed by free-running
    // counters: core 1 moves rx_head along and core 0 moves rx_tail.
    pub rx_slots: [UnsafeCell<RxSlot>; RX_SLOT_COUNT],
    pub rx_head: AtomicU32,
    pub rx_tail: AtomicU32,

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
            rx_slots: [
                UnsafeCell::new(RxSlot::empty()),
                UnsafeCell::new(RxSlot::empty()),
                UnsafeCell::new(RxSlot::empty()),
                UnsafeCell::new(RxSlot::empty()),
            ],
            rx_head: AtomicU32::new(0),
            rx_tail: AtomicU32::new(0),
            flags: AtomicU32::new(0),
        }
    }

    /// Core 0 side: pass the oldest received frame to `f`, then free up its
    /// slot. Returns `None` if nothing is waiting.
    pub fn with_rx<R>(&self, f: impl FnOnce(&RxSlot) -> R) -> Option<R> {
        let tail = self.rx_tail.load(Ordering::Relaxed);
        let head = self.rx_head.load(Ordering::Acquire);
        if tail == head {
            return None;
        }
        compiler_fence(Ordering::SeqCst);
        // SAFETY: tail != head means core 1 published this slot and will
        // not rewrite it until rx_tail advances past it.
        let slot = unsafe { &*self.rx_slots[tail as usize % RX_SLOT_COUNT].get() };
        let r = f(slot);
        compiler_fence(Ordering::SeqCst);
        self.rx_tail.store(tail.wrapping_add(1), Ordering::Release);
        Some(r)
    }
}

impl RxSlot {
    pub const fn empty() -> Self {
        Self {
            buf: [0u8; MAX_FRAME_LEN],
            len: 0,
            status: 0,
        }
    }
}

/// Core 1 side: push a received frame into the RX ring, dropping when full.
fn push_rx(shared: &SharedState, frame: &[u8], crc_ok: bool) {
    let head = shared.rx_head.load(Ordering::Relaxed);
    let tail = shared.rx_tail.load(Ordering::Acquire);
    if head.wrapping_sub(tail) >= RX_SLOT_COUNT as u32 {
        unsafe { RX_DROP_COUNT += 1; }
        return;
    }
    let n = frame.len().min(MAX_FRAME_LEN);
    // SAFETY: the slot at head is not visible to core 0 until rx_head is
    // advanced below.
    let slot = unsafe { &mut *shared.rx_slots[head as usize % RX_SLOT_COUNT].get() };
    slot.buf[..n].copy_from_slice(&frame[..n]);
    slot.len = n as u16;
    slot.status = if crc_ok { 0 } else { 1 };
    compiler_fence(Ordering::SeqCst);
    shared.rx_head.store(head.wrapping_add(1), Ordering::Release);
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
    let now = read_timer_us();
    let last = unsafe { LAST_ACTIVITY_US };
    now.wrapping_sub(last) < CARRIER_IDLE_US
}

/// Mark the bus as active (called from the RX decode loop on every
/// FM-0 event — byte, flag, or abort).
#[inline(always)]
fn mark_activity() {
    unsafe { LAST_ACTIVITY_US = read_timer_us(); }
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

// ── Frame completion (shared by main loop and CTS wait) ──────────

// ── Receive engine ────────────────────────────────────────────────

/// An auto-reply that's been encoded into AUTOREPLY_DMA_BUF and is waiting
/// for the line to go quiet.
struct PendingReply {
    word_count: usize,
    fm0_len: usize,
    /// When we decoded the closing flag of the frame this is answering.
    since: u64,
}

/// Decodes the RX ring into frames and answers ENQs and RTSes. The main loop
/// owns it and hands it to the CSMA waits as well, so we keep decoding and
/// answering frames even while we're waiting to transmit.
struct RxEngine {
    unstuffer: BitUnstuffer,
    assembler: FrameAssembler,
    nodes: NodeBitmap,
    frames_seen: u32,
    pending: Option<PendingReply>,
}

impl RxEngine {
    fn new() -> Self {
        Self {
            unstuffer: BitUnstuffer::new(),
            assembler: FrameAssembler::new(),
            nodes: NodeBitmap::empty(),
            frames_seen: 0,
            pending: None,
        }
    }

    /// Decode everything waiting in the RX ring.
    fn service(&mut self, shared: &SharedState) {
        while let Some(word) = try_read_rx() {
            self.feed(word, shared);
        }
        self.expire_pending();
    }

    fn feed(&mut self, word: u32, shared: &SharedState) {
        if word == FRAME_END_MARKER {
            // PIO detected idle after a frame. Force-complete any
            // in-progress frame and reset for the next one.
            if let Some(AsmEvent::FrameEnd) = self.assembler.feed_flag() {
                self.frame_end(shared);
                self.assembler.ack_frame_end();
            }
            self.unstuffer.reset();
            self.assembler.feed_abort();
            // The PIO pushes this ~1.75 cells after the last transition, so by
            // now the sender has finished and let go of the line.
            self.send_pending();
            return;
        }

        mark_activity();
        // Each word has 8 decoded FM-0 bits in [31:24].
        let byte = ((word >> 24) & 0xFF) as u8;
        for bit_idx in 0..8u32 {
            let bit = (byte >> bit_idx) & 1;
            match self.unstuffer.feed_bit(bit) {
                UnstuffEvent::Byte(b) => {
                    self.assembler.feed_byte(b);
                }
                UnstuffEvent::Flag => {
                    if let Some(AsmEvent::FrameEnd) = self.assembler.feed_flag() {
                        self.frame_end(shared);
                        self.assembler.ack_frame_end();
                    }
                }
                UnstuffEvent::Abort => {
                    self.assembler.feed_abort();
                    self.unstuffer.reset();
                }
                UnstuffEvent::None => {}
            }
        }
    }

    /// Process a completed frame: stats, echo suppression, auto-reply and
    /// forwarding to the host.
    fn frame_end(&mut self, shared: &SharedState) {
        self.frames_seen = self.frames_seen.wrapping_add(1);
        let frame = self.assembler.current();
        let crc_ok = frame_crc_ok(frame);

        let analyzer = crate::debug::ANALYZER_MODE.load(Ordering::Relaxed);

        // Echo suppression: skip frames where src is one of our nodes.
        // In analyzer mode, show everything including echoes.
        let is_echo = frame.len() >= 2 && self.nodes.contains(frame[1]);
        if is_echo && !analyzer {
            unsafe { ECHO_SKIP_COUNT += 1; }
            return;
        }

        // Count directed frames (Mac → us).
        if frame.len() >= 1 && self.nodes.contains(frame[0]) {
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

        let reply = if crc_ok {
            auto_reply(frame, &self.nodes).into_frame()
        } else {
            None
        };

        // Forward frames to core 0. In normal mode: good-CRC only. In
        // analyzer mode: all frames.
        if crc_ok || analyzer {
            push_rx(shared, frame, crc_ok);
        }

        // Encode it now while the sender is still on its abort sequence, so
        // all we have left to do once the line goes quiet is start the DMA.
        if let Some(reply_frame) = reply {
            let (word_count, fm0_len) = build_small_frame_dma(&reply_frame);
            self.pending = Some(PendingReply {
                word_count,
                fm0_len,
                since: read_timer_us(),
            });
        }
    }

    fn send_pending(&mut self) {
        let Some(p) = self.pending.take() else { return };
        let waited = read_timer_us().wrapping_sub(p.since);
        if waited > REPLY_WINDOW_US {
            unsafe { AUTOREPLY_LATE += 1; }
            return;
        }
        transmit_dma(unsafe { AUTOREPLY_DMA_BUF.as_ptr() }, p.word_count, p.fm0_len);
        // Count our own reply as bus activity, so the silence a CSMA wait
        // needs is measured from the end of it.
        mark_activity();
        unsafe {
            AUTOREPLY_COUNT += 1;
            AUTOREPLY_WAIT_MAX_US = AUTOREPLY_WAIT_MAX_US.max(waited as u32);
        }
    }

    /// Drop the reply if its window closed before the line went quiet.
    fn expire_pending(&mut self) {
        let expired = match &self.pending {
            Some(p) => read_timer_us().wrapping_sub(p.since) > REPLY_WINDOW_US,
            None => false,
        };
        if expired {
            self.pending = None;
            unsafe { AUTOREPLY_LATE += 1; }
        }
    }
}

// ── CSMA/CA helpers ───────────────────────────────────────────────

/// Wait until carrier sense is inactive (bus idle), decoding and answering
/// anything that arrives in the meantime. We give up after
/// MAX_BUS_IDLE_WAIT_US and let the backoff sort it out, instead of leaving
/// core 1 stuck here.
fn wait_bus_idle(rx: &mut RxEngine, shared: &SharedState) {
    let deadline = read_timer_us() + MAX_BUS_IDLE_WAIT_US;
    while carrier_sense_active() {
        rx.service(shared);
        if read_timer_us() >= deadline {
            unsafe { BUS_IDLE_TIMEOUTS += 1; }
            return;
        }
    }
}

/// Wait for the bus to stay idle for `us` microseconds, decoding and
/// answering anything that arrives in the meantime. Returns false if carrier
/// sense fires or we end up owing a reply during the wait (deferral).
fn wait_idle_for(us: u64, rx: &mut RxEngine, shared: &SharedState) -> bool {
    let deadline = read_timer_us() + us;
    while read_timer_us() < deadline {
        rx.service(shared);
        if carrier_sense_active() || rx.pending.is_some() {
            return false;
        }
    }
    rx.service(shared);
    !carrier_sense_active() && rx.pending.is_none()
}

/// CSMA/CA wait + transmit. Dispatches to directed (RTS/CTS/DATA)
/// or broadcast path based on tx_slot.is_directed.
fn csma_transmit(shared: &SharedState, backoff: &mut BackoffState, rx: &mut RxEngine) -> bool {
    // SAFETY: TX_SUBMIT was set, so core 0 is not writing tx_slot.
    let slot = unsafe { &*shared.tx_slot.get() };

    if slot.is_directed {
        return directed_tx(shared, slot, backoff, rx);
    }

    // Broadcast / control frame — simple CSMA then TX.
    backoff.prep_for_next_frame();

    loop {
        if backoff.exhausted() {
            return false;
        }

        wait_bus_idle(rx, shared);

        let entropy = (read_timer_us() as u8)
            ^ ((read_timer_us() >> 8) as u8);
        let delay_units = backoff.next_backoff_units(entropy);
        let delay_us = delay_units as u64 * 100;
        if !wait_idle_for(delay_us, rx, shared) {
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
    rx: &mut RxEngine,
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
        wait_bus_idle(rx, shared);

        let entropy = (read_timer_us() as u8)
            ^ ((read_timer_us() >> 8) as u8);
        let delay_units = backoff.next_backoff_units(entropy);
        let delay_us = delay_units as u64 * 100;
        if !wait_idle_for(delay_us, rx, shared) {
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
        // Start listening as soon as the RTS is out. We don't wait for our
        // own RTS echo first, because whether we hear the echo at all depends
        // on how the transceiver is wired: with TX and RX tied to one pair we
        // always hear it, but with them on separate pairs we never do, and
        // waiting for it just runs out the clock while the peer's CTS comes
        // and goes. An echoed RTS has dst = their_dest and src = our_src, so
        // cts_matches_pending throws it away anyway.
        //
        // The CTS has CTS_START_WINDOW_US to START arriving. Once we see
        // decoded data we extend the deadline so the whole frame can arrive
        // and be decoded.
        let cts_wait_start = read_timer_us();
        let mut cts_deadline = cts_wait_start + CTS_START_WINDOW_US;
        let mut saw_activity = false;
        let mut cts_unstuffer = BitUnstuffer::new();
        let mut cts_assembler = FrameAssembler::new();
        let mut got_cts = false;
        let mut entries_processed: u32 = 0;

        unsafe { LAST_CTS_WAIT_START = cts_wait_start; }

        loop {
            if read_timer_us() > cts_deadline {
                break;
            }

            match try_read_rx() {
                Some(FRAME_END_MARKER) => {
                    // Frame ended — if we were assembling, check it.
                    if let Some(AsmEvent::FrameEnd) = cts_assembler.feed_flag() {
                        let frame = cts_assembler.current();
                        if frame_crc_ok(frame)
                            && cts_matches_pending(frame, our_src, their_dest)
                        {
                            push_rx(shared, frame, true);
                            got_cts = true;
                            unsafe { DIRECTED_TX_CTS_OK += 1; }
                        }
                        cts_assembler.ack_frame_end();
                    }
                    cts_unstuffer.reset();
                }
                Some(word) => {
                    entries_processed += 1;
                    if !saw_activity {
                        saw_activity = true;
                        cts_deadline = read_timer_us() + 600;
                    }
                    let byte = ((word >> 24) & 0xFF) as u8;
                    for bit_idx in 0..8u32 {
                        let bit = (byte >> bit_idx) & 1;
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
                                        push_rx(shared, frame, true);
                                        got_cts = true;
                                        unsafe { DIRECTED_TX_CTS_OK += 1; }
                                    }
                                    cts_assembler.ack_frame_end();
                                }
                            }
                            UnstuffEvent::Abort => {
                                cts_assembler.feed_abort();
                                cts_unstuffer.reset();
                            }
                            UnstuffEvent::None => {}
                        }
                    }
                }
                None => {}
            }

            if got_cts {
                break;
            }
        }

        unsafe {
            LAST_CTS_WAIT_END = read_timer_us();
            LAST_CTS_ENTRIES = entries_processed;
        }

        if got_cts {
            if slot.fm0_len > 1000 {
                dbg0_high();
            }
            transmit_dma(
                slot.dma_buf.as_ptr(),
                slot.word_count as usize,
                slot.fm0_len as usize,
            );
            dbg0_low();
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
    let mut rx = RxEngine::new();
    let mut backoff = BackoffState::new();

    loop {
        unsafe { LOOP_COUNT = LOOP_COUNT.wrapping_add(1); }

        // ── Check node bitmap update (rare) ───────────────────────
        if shared.flags.load(Ordering::Relaxed) & NODES_UPDATED != 0 {
            compiler_fence(Ordering::SeqCst);
            // SAFETY: NODES_UPDATED set → core 0 finished writing.
            let bm = unsafe { &*shared.node_bitmap.get() };
            rx.nodes = NodeBitmap::from_bytes(*bm);
            shared.flags.fetch_and(!NODES_UPDATED, Ordering::Release);
        }

        // ── Poll PIO1 RX FIFO (hot path) ─────────────────────────
        match try_read_rx() {
            Some(word) => rx.feed(word, shared),
            None => {
                rx.expire_pending();
                // ── FIFO empty: check for host TX request ─────────
                // Hold off while a reply is waiting for the line to go quiet.
                // It goes out within REPLY_WINDOW_US at most, and a CSMA wait
                // would just get deferred by it anyway.
                let flags = shared.flags.load(Ordering::Relaxed);
                if flags & TX_SUBMIT != 0 && rx.pending.is_none() {
                    shared.flags.fetch_and(!TX_SUBMIT, Ordering::Acquire);
                    compiler_fence(Ordering::SeqCst);

                    let ok = csma_transmit(shared, &mut backoff, &mut rx);

                    // Signal completion to core 0.
                    let result_bits = TX_DONE | if ok { 0 } else { TX_RESULT_FAIL };
                    shared.flags.fetch_and(!(TX_DONE | TX_RESULT_FAIL), Ordering::Relaxed);
                    shared.flags.fetch_or(result_bits, Ordering::Release);
                }
            }
        }
    }
}
