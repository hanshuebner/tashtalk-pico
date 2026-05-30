//! LocalTalk PHY — FM-0 TX (PIO0 SM0) and RX sampler (PIO0 SM1).
//!
//! ## Wire format
//!
//! Differential Manchester (FM-0) at 230.4 kbit/s. One bit cell = 4.34 µs.
//! Every cell starts with a mandatory clock transition; a *data* `0` adds
//! a second transition at mid-cell, a data `1` does not.
//!
//! ## PIO programs
//!
//! - **TX (SM0)**: 8 PIO cycles per bit cell. PIO clocked at 1.8432 MHz
//!   (125 MHz / 67.8164). Pulls bits LSB-first from TX FIFO, toggles the
//!   phase register and writes it to the OUT pin to produce FM-0 edges.
//! - **RX (SM1)**: same 1.8432 MHz clock (8× oversample of the bit
//!   rate). Single-instruction sampler — `in pins, 1` — pushes raw
//!   samples into the RX FIFO, autopushing one u32 per 32 samples
//!   (4 bit cells). Software (`fm0_decode`) handles bit alignment and
//!   FM-0 decode.
//!
//! ## Pin assignment (default)
//!
//! - **GP2** — `LT_DATA` TX (PIO out)
//! - **GP3** — `LT_DE` (driver enable; M0+ GPIO)
//! - **GP4** — `LT_DATA_RX` (PIO in). For self-loopback validation,
//!   jumper GP2 ↔ GP4 with a wire.

use crate::bitstream::BitStream;
use embassy_rp::pac;
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio::{
    Config, Direction, FifoJoin, InterruptHandler, Pio, PioPin, ShiftConfig,
    ShiftDirection, StateMachine,
};
use embassy_rp::Peripheral;
use fixed::types::U24F8;
use tashtalk_proto::bitstuff::{BitUnstuffer, UnstuffEvent};

embassy_rp::bind_interrupts!(pub(crate) struct PioIrqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

const BIT_RATE_HZ: u32 = 230_400;
const CYCLES_PER_BIT: u32 = 8;
const SYS_CLK_HZ: u32 = 125_000_000;

/// Initialise the PHY: create both LtTx (TX SM0) and LtRx (RX SM1)
/// sharing PIO0 common. Returns the two halves which can then be put in
/// separate Mutexes.
pub fn init_phy<'d>(
    pio: impl Peripheral<P = PIO0> + 'd,
    _irqs: PioIrqs,
    tx_pin: impl Peripheral<P = impl PioPin> + 'd,
    rx_pin: impl Peripheral<P = impl PioPin> + 'd,
) -> (LtTx<'d>, LtRx<'d>) {
    let Pio {
        mut common,
        mut sm0,
        mut sm1,
        ..
    } = Pio::new(pio, PioIrqs {});

    // ── TX program (SM0) ─────────────────────────────────────────────
    // FM-0 encoder: 8 PIO cycles per bit cell.
    //
    // Per bit:
    //   c0  out x, 1            — pull data bit
    //   c1  mov y, !y           — toggle phase register
    //   c2  mov pins, y         — write phase to OUT pin (clock edge)
    //   c3  jmp !x, mid_zero    — branch if bit is 0
    //   c4..c7  (bit=1) nops    — wait the rest of the cell
    // OR
    //   c4  nop                 — pad to mid-cell
    //   c5  mov y, !y           — toggle phase (data 0 mid-cell edge)
    //   c6  mov pins, y         — write
    //   c7  jmp start           — explicit branch back to wrap_target.
    //
    // The trailing `jmp start` matters: without it, PC walks past the
    // end of this program into the NEXT program (or NOP memory), and
    // SM0 emits garbage on the OUT pin until PC wraps back at 32.
    let tx_prog = pio_proc::pio_asm!(
        ".wrap_target",
        "start:",
        "    out x, 1",            // c0
        "    mov y, !y",           // c1
        "    mov pins, y",         // c2
        "    jmp !x, mid_zero",    // c3
        "    nop",                 // c4
        "    nop",                 // c5
        "    nop",                 // c6
        "    nop",                 // c7
        ".wrap",
        "mid_zero:",
        "    nop",                 // c4
        "    mov y, !y",           // c5
        "    mov pins, y",         // c6
        "    jmp start",           // c7 — explicit return to wrap_target
    );
    let tx_loaded = common.load_program(&tx_prog.program);
    let tx_pin_obj = common.make_pio_pin(tx_pin);
    let mut tx_cfg = Config::default();
    tx_cfg.use_program(&tx_loaded, &[]);
    tx_cfg.set_out_pins(&[&tx_pin_obj]);
    tx_cfg.set_set_pins(&[&tx_pin_obj]);
    tx_cfg.shift_out = ShiftConfig {
        threshold: 32,
        direction: ShiftDirection::Right,
        auto_fill: true,
    };
    tx_cfg.fifo_join = FifoJoin::TxOnly;
    tx_cfg.clock_divider = clock_divider();
    sm0.set_config(&tx_cfg);
    sm0.set_pin_dirs(Direction::Out, &[&tx_pin_obj]);

    // ── RX program (SM1) ─────────────────────────────────────────────
    let rx_prog = pio_proc::pio_asm!(
        ".wrap_target",
        "    in pins, 1",
        ".wrap",
    );
    let rx_loaded = common.load_program(&rx_prog.program);
    let rx_pin_obj = common.make_pio_pin(rx_pin);
    let mut rx_cfg = Config::default();
    rx_cfg.use_program(&rx_loaded, &[]);
    rx_cfg.set_in_pins(&[&rx_pin_obj]);
    rx_cfg.shift_in = ShiftConfig {
        threshold: 32,
        direction: ShiftDirection::Right,
        auto_fill: true,
    };
    rx_cfg.fifo_join = FifoJoin::RxOnly;
    rx_cfg.clock_divider = clock_divider();
    sm1.set_config(&rx_cfg);
    sm1.set_pin_dirs(Direction::In, &[&rx_pin_obj]);

    // CRITICAL: restart the SM after set_config so its PC jumps to the
    // RX program's wrap_target instead of staying at PC=0 (which is the
    // middle of the TX program — shared instruction memory!).
    sm1.restart();
    // Same precaution for SM0 — its wrap_target IS 0 so this is mostly
    // a no-op for TX, but it's harmless and makes the intent explicit.
    sm0.restart();

    (LtTx { sm: sm0 }, LtRx { sm: sm1 })
}

// ── TX ──────────────────────────────────────────────────────────────

pub struct LtTx<'d> {
    sm: StateMachine<'d, PIO0, 0>,
}

impl<'d> LtTx<'d> {
    pub async fn transmit_frame(&mut self, frame: &[u8]) {
        let stream = BitStream::build_frame(frame);
        let total_bits = stream.bit_len();
        let bytes = stream.as_bytes();

        self.sm.set_enable(true);

        let mut bits_remaining = total_bits;
        let mut byte_idx = 0usize;
        while bits_remaining > 0 {
            let mut word = 0u32;
            for shift in 0..4 {
                if byte_idx < bytes.len() {
                    word |= (bytes[byte_idx] as u32) << (shift * 8);
                    byte_idx += 1;
                }
            }
            self.sm.tx().wait_push(word).await;
            bits_remaining = bits_remaining.saturating_sub(32);
        }

        let mut drain_iters = 0u32;
        while !self.sm.tx().empty() {
            embassy_time::Timer::after_micros(50).await;
            drain_iters += 1;
            if drain_iters > 2000 {
                break;
            }
        }
        // After the TX FIFO empties, the last word has just been auto-
        // pulled into the OSR and is being shifted out. We must wait for
        // the VALID bits (frame data including the closing SDLC flag) to
        // finish, but NOT the zero-padding — those are inert and keeping
        // DE high for them delays our ability to hear a CTS response
        // (since RE is tied to DE).
        let bit_us = 1_000_000 / BIT_RATE_HZ as u64;
        let last_word_valid = if total_bits % 32 == 0 { 32 } else { total_bits % 32 };
        // Add 2 bit-times of margin for clock accuracy + line settling.
        embassy_time::Timer::after_micros((last_word_valid as u64 + 2) * bit_us).await;

        self.sm.set_enable(false);
    }

    /// Arm SM0 for hardware-triggered dialog TX.
    ///
    /// SM0 must already be disabled (from the preceding `transmit_frame`
    /// that sent the RTS). The TX FIFO should be empty; the caller arms
    /// DMA (which fills the FIFO) after this call.
    ///
    /// Sequence: enable SM0 → discard residual OSR bits → gate on PIO
    /// IRQ 4. When the RX task fires IRQ 4 (CTS detected), SM0
    /// unblocks, auto-pulls the first DMA word from the FIFO, and
    /// starts FM-0 encoding with zero wasted bits.
    ///
    /// **Why no `restart()`:** restart clears both FIFOs (losing any
    /// DMA-prefilled data) and sets OSR to all-ones (injecting 32
    /// garbage bits before real data). We avoid both by leaving the
    /// FIFO untouched and draining OSR with `out null, 32`.
    pub fn arm_dialog(&mut self) {
        // SM0 is disabled at PC = `out x, 1` (where transmit_frame
        // left it stalled on the empty FIFO). Enable it — the first
        // PIO cycle stalls again on the empty FIFO, giving us time
        // to exec the setup instructions below.
        self.sm.set_enable(true);
        // Discard residual OSR bits from the previous transmit_frame
        // so the first auto-pull after IRQ 4 gets real FIFO data.
        // PIO `out null, 32` = 0x6060.
        unsafe { self.sm.exec_instr(0x6060) };
        // Gate SM0 on PIO IRQ 4. SM stalls until trigger() fires
        // the IRQ, then resumes at PC (`out x, 1`), auto-pulls the
        // first DMA word, and starts encoding.
        // PIO `wait 1 irq 4` = 0x20C4.
        unsafe { self.sm.exec_instr(0x20c4) };
    }

    /// Disarm a previously armed dialog TX (e.g. on CTS timeout).
    /// Disables SM0, clears the FIFO (DMA may have pre-filled it),
    /// and clears the IRQ flag. Uses `restart()` to flush the FIFO;
    /// this also resets OSR/shift counters but preserves X/Y, which
    /// is fine — the next `transmit_frame` handles any OSR state.
    pub fn disarm_dialog(&mut self) {
        self.sm.set_enable(false);
        self.sm.restart();
        pac::PIO0.irq().write(|w| w.set_irq(1 << 4));
    }

    /// Wait for SM0's TX FIFO to drain (the DMA-fed data has been
    /// consumed by the PIO encoder). Call after dialog TX fires.
    pub async fn wait_tx_drain(&mut self) {
        let mut iters = 0u32;
        while !self.sm.tx().empty() {
            embassy_time::Timer::after_micros(50).await;
            iters += 1;
            if iters > 2000 {
                break;
            }
        }
        // Wait for the last bits to shift out of the OSR onto the wire.
        let bit_us = 1_000_000 / BIT_RATE_HZ as u64;
        embassy_time::Timer::after_micros((bit_us * 4).max(20)).await;
        self.sm.set_enable(false);
    }
}

// ── RX ──────────────────────────────────────────────────────────────

pub struct LtRx<'d> {
    sm: StateMachine<'d, PIO0, 1>,
}

impl<'d> LtRx<'d> {
    pub fn drain(&mut self) {
        while !self.sm.rx().empty() {
            let _ = self.sm.rx().try_pull();
        }
    }
    pub fn enable(&mut self) {
        self.sm.set_enable(true);
    }
    pub fn disable(&mut self) {
        self.sm.set_enable(false);
    }

    /// DMA-driven capture. Returns a future that resolves when `out`
    /// has been filled from the PIO RX FIFO. While the DMA is in
    /// flight, the CPU is free.
    pub fn dma_pull<'a, C: embassy_rp::dma::Channel>(
        &'a mut self,
        ch: embassy_rp::PeripheralRef<'a, C>,
        out: &'a mut [u32],
    ) -> embassy_rp::dma::Transfer<'a, C> {
        self.sm.rx().dma_pull(ch, out)
    }

    /// Polled fallback capture for code that doesn't have a DMA
    /// channel handy (debug commands).
    pub async fn capture(&mut self, out: &mut [u32]) {
        let mut filled = 0;
        let start = embassy_time::Instant::now();
        while filled < out.len() {
            match self.sm.rx().try_pull() {
                Some(w) => {
                    out[filled] = w;
                    filled += 1;
                }
                None => {
                    if start.elapsed() > embassy_time::Duration::from_millis(50) {
                        return;
                    }
                    embassy_time::Timer::after_micros(20).await;
                }
            }
        }
    }
}

// ── FM-0 decoder ────────────────────────────────────────────────────

/// Decoded event from the FM-0 + SDLC layer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DecodedEvent {
    Byte(u8),
    Flag,
    Abort,
}

/// Decode a buffer of 8×-oversampled FM-0 samples into a sequence of
/// SDLC events (bytes, flag boundaries, aborts).
///
/// `samples` is the contiguous bit stream from the RX FIFO. Each word
/// holds 32 samples, LSB = first sample in time.
///
/// `out` receives the decoded events. Returns the number of events
/// written.
pub fn fm0_decode<const N: usize>(
    samples: &[u32],
    out: &mut heapless::Vec<DecodedEvent, N>,
) -> usize {
    // Flatten samples to a single iterator of bits.
    // Each u32 contains 32 samples, sample 0 in the LSB.
    let total_samples = samples.len() * 32;
    if total_samples < 8 {
        return 0;
    }

    // Find the first edge — that anchors our bit-cell phase. A clean
    // PIO TX always starts the wire with at least one transition (the
    // first clock edge of the first preamble bit cell).
    let mut first_edge_idx: Option<usize> = None;
    let mut prev_bit = sample_at(samples, 0);
    for i in 1..total_samples {
        let b = sample_at(samples, i);
        if b != prev_bit {
            first_edge_idx = Some(i);
            break;
        }
        prev_bit = b;
    }
    let Some(start) = first_edge_idx else {
        return 0; // line never moved — nothing to decode
    };

    // Walk the bit stream 8 samples at a time, starting at `start`.
    // sample[0] is right after the clock edge.
    // bit = (sample[0] == sample[7]) ? 1 : 0.
    let mut unstuffer = BitUnstuffer::new();
    let mut idx = start;
    let events_before = out.len();

    while idx + 8 <= total_samples {
        let s0 = sample_at(samples, idx);
        let s7 = sample_at(samples, idx + 7);
        let bit: u8 = if s0 == s7 { 1 } else { 0 };
        match unstuffer.feed_bit(bit) {
            UnstuffEvent::Byte(b) => {
                if out.push(DecodedEvent::Byte(b)).is_err() {
                    break;
                }
            }
            UnstuffEvent::Flag => {
                if out.push(DecodedEvent::Flag).is_err() {
                    break;
                }
            }
            UnstuffEvent::Abort => {
                if out.push(DecodedEvent::Abort).is_err() {
                    break;
                }
            }
            UnstuffEvent::None => {}
        }
        idx += 8;
    }

    out.len() - events_before
}

#[inline]
fn sample_at(samples: &[u32], i: usize) -> u8 {
    let word = samples[i / 32];
    ((word >> (i % 32)) & 1) as u8
}

const fn clock_divider() -> U24F8 {
    let target = BIT_RATE_HZ * CYCLES_PER_BIT;
    let div_x256 = (SYS_CLK_HZ as u64 * 256 + target as u64 / 2) / target as u64;
    U24F8::from_bits(div_x256 as u32)
}
