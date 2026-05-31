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
    Config, Direction, FifoJoin, InterruptHandler, Pin, Pio, PioPin, ShiftConfig,
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
) -> (LtTx<'d>, LtRx<'d>, LtDialogTx<'d>) {
    let Pio {
        mut common,
        mut sm0,
        mut sm1,
        mut sm2,
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

    // ── Dialog TX (SM2) ───────────────────────────────────────────────
    // SM2 runs the same FM-0 TX program as SM0. It is used exclusively
    // for the hardware-triggered RTS/CTS dialog, gated on PIO IRQ 4.
    // SM0 is never touched by exec'd instructions.
    //
    // CRITICAL: SM2 is configured WITHOUT output pins at init. On the
    // RP2040, PIO output values are ORed across ALL state machines —
    // if SM2 had GP2 as an output pin, its latched output value would
    // corrupt SM0's FM-0 signal. The output pin is only configured
    // in arm_dialog() and removed in disarm_dialog().
    let mut sm2_cfg = Config::default();
    sm2_cfg.use_program(&tx_loaded, &[]);
    // No output pins — arm_dialog will configure PINCTRL via PAC.
    sm2_cfg.shift_out = ShiftConfig {
        threshold: 32,
        direction: ShiftDirection::Right,
        auto_fill: true,
    };
    sm2_cfg.fifo_join = FifoJoin::TxOnly;
    sm2_cfg.clock_divider = clock_divider();
    sm2.set_config(&sm2_cfg);

    // CRITICAL: restart all SMs after set_config so PCs jump to each
    // program's wrap_target instead of staying at PC=0.
    sm1.restart();
    sm0.restart();
    sm2.restart();

    // SM2 has no output pins configured (OUT_COUNT=0, SET_COUNT=0),
    // so it cannot contribute to the PIO output OR for GP2.

    (LtTx { sm: sm0 }, LtRx { sm: sm1 }, LtDialogTx { sm: sm2, pin: tx_pin_obj })
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
        // pulled into the OSR and is being shifted out. Wait for the
        // VALID bits (frame data + closing flag + postamble) to finish.
        let bit_us = 1_000_000 / BIT_RATE_HZ as u64;
        let last_word_valid = if total_bits % 32 == 0 { 32 } else { total_bits % 32 };
        embassy_time::Timer::after_micros((last_word_valid as u64 + 2) * bit_us).await;

        self.sm.set_enable(false);
    }
}

// ── Dialog TX (SM2) ────────────────────────────────────────────────
//
// Dedicated state machine for the hardware-triggered RTS/CTS dialog.
// SM2 runs the same FM-0 TX program as SM0 on the same output pin
// (GP2), but is only enabled during the dialog window. This isolates
// SM0 from exec'd instructions (`wait 1 irq 4`) that would otherwise
// persist across enable/disable cycles and cause hangs.

pub struct LtDialogTx<'d> {
    sm: StateMachine<'d, PIO0, 2>,
    pin: Pin<'d, PIO0>,
}

impl<'d> LtDialogTx<'d> {
    /// Configure SM2's PINCTRL to drive GP2 (OUT and SET groups).
    fn claim_pin(&mut self) {
        pac::PIO0.sm(2).pinctrl().write(|w| {
            w.set_out_base(2); // GP2
            w.set_out_count(1);
            w.set_set_base(2);
            w.set_set_count(1);
        });
        self.sm.set_pin_dirs(Direction::Out, &[&self.pin]);
    }

    /// Remove SM2 from GP2: clear output values and PINCTRL.
    fn release_pin(&mut self) {
        // Clear both OUT and SET output values for GP2 so the PIO
        // output OR doesn't corrupt SM0's signal.
        unsafe { self.sm.exec_instr(0xE000) }; // set pins, 0
        unsafe { self.sm.exec_instr(0xA003) }; // mov pins, null
        self.sm.set_pin_dirs(Direction::In, &[&self.pin]);
        // Remove GP2 from SM2's PINCTRL entirely.
        pac::PIO0.sm(2).pinctrl().write(|w| {
            w.set_out_base(0);
            w.set_out_count(0);
            w.set_set_base(0);
            w.set_set_count(0);
        });
    }

    /// Arm SM2 for hardware-triggered dialog TX.
    pub fn arm_dialog(&mut self) {
        self.claim_pin();
        self.sm.set_enable(true);
        // Drain residual OSR so first auto-pull gets real FIFO data.
        unsafe { self.sm.exec_instr(0x6060) }; // out null, 32
        // Gate on PIO IRQ 4 (CTS trigger).
        unsafe { self.sm.exec_instr(0x20c4) }; // wait 1 irq 4
    }

    /// Disarm SM2 (e.g. on CTS timeout). SM0 is unaffected.
    pub fn disarm_dialog(&mut self) {
        self.sm.set_enable(false);
        self.sm.restart();
        self.release_pin();
        pac::PIO0.irq().write(|w| w.set_irq(1 << 4));
    }

    /// Wait for SM2's DMA-fed TX to complete, then release GP2.
    pub async fn wait_tx_drain(&mut self) {
        let mut iters = 0u32;
        while !self.sm.tx().empty() {
            embassy_time::Timer::after_micros(50).await;
            iters += 1;
            if iters > 2000 { break; }
        }
        let bit_us = 1_000_000 / BIT_RATE_HZ as u64;
        embassy_time::Timer::after_micros(bit_us * 34).await;
        self.sm.set_enable(false);
        self.release_pin();
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
