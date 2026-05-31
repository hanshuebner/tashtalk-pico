//! PIO state machine programs for LocalTalk PHY.
//!
//! Three programs share one or two PIO blocks:
//!
//! - **SM0 — TX Data Shift-out**: Shifts pre-encoded FM-0 half-cell
//!   symbols from the TX FIFO to the data output pin at 460.8 kHz.
//!
//! - **SM1 — TX Enable + Collision Gate**: Controls the RS-422 driver
//!   enable pin. Implements the sync pulse → silence → collision check
//!   → frame transmit sequence entirely in PIO hardware.
//!
//! - **SM2 — RX Edge Timer + Carrier Sense**: Measures inter-edge
//!   timing on the RX input, pushes interval counts to the RX FIFO,
//!   and drives a carrier-sense output pin for SM1's collision gate.
//!
//! ## Pin assignments
//!
//! | Pin function       | PIO config            | SM  |
//! |--------------------|-----------------------|-----|
//! | TX data            | OUT base, 1 pin       | SM0 |
//! | TX enable          | SET base, 1 pin       | SM1 |
//! | Carrier sense in   | JMP PIN               | SM1 |
//! | RX data            | JMP PIN               | SM2 |
//! | Carrier sense out  | SET base, 1 pin       | SM2 |

use pio::{
    Assembler, InSource, JmpCondition, OutDestination, SetDestination, RP2040_MAX_PROGRAM_SIZE,
};

/// Assembled PIO program ready for loading.
#[derive(Debug, Clone)]
pub struct PioProgram {
    /// Assembled instruction words.
    pub instructions: [u16; RP2040_MAX_PROGRAM_SIZE],
    /// Number of valid instructions.
    pub len: usize,
    /// Wrap target offset.
    pub wrap_target: u8,
    /// Wrap source offset (last instruction in wrap range).
    pub wrap: u8,
}

// ── SM0: TX Data Shift-out ────────────────────────────────────────────

/// Assemble the SM0 program: TX data shift-out.
///
/// Single instruction: `out pins, 1` in a tight wrap loop.
/// Autopull at 32 bits. Clock = 460.8 kHz.
pub fn assemble_tx_data() -> PioProgram {
    let mut a = Assembler::<RP2040_MAX_PROGRAM_SIZE>::new();
    let mut wrap_target = a.label();
    let mut wrap_source = a.label();

    a.bind(&mut wrap_target);
    a.out(OutDestination::PINS, 1);
    a.bind(&mut wrap_source);

    let prog = a.assemble_with_wrap(wrap_source, wrap_target);
    to_pio_program(&prog)
}

// ── SM1: TX Enable + Collision Gate ───────────────────────────────────

/// Assemble the SM1 program: TX enable + collision gate.
///
/// Phases:
/// 1. Sync pulse: set pins 1, hold for 2 half-cells
/// 2. Silence: set pins 0, wait 4 half-cells checking carrier sense
/// 3. Frame: set pins 1, pull frame duration, count down
/// 4. Done: set pins 0, IRQ success or collision
///
/// JMP PIN must be wired to the carrier-sense GPIO driven by SM2.
/// SET pin controls TX_EN.
/// Clock = 460.8 kHz (same as SM0).
pub fn assemble_tx_enable() -> PioProgram {
    let mut a = Assembler::<RP2040_MAX_PROGRAM_SIZE>::new();

    let mut silence_check = a.label();
    let mut frame_hold = a.label();
    let mut collision = a.label();
    let mut halt_ok = a.label();
    let mut halt_col = a.label();

    // Phase 1: Sync pulse (2 half-cells = 1 bit time)
    a.set(SetDestination::PINS, 1); // enable TX
    a.set(SetDestination::X, 0); // X=0: loop body runs once (set+jmp = 2 cycles)
    // jmp x-- is 1 cycle, + set above = 2 half-cells total for the enable
    a.jmp(JmpCondition::XDecNonZero, &mut silence_check); // X was 0, falls through immediately

    // Phase 2: Silence (4 half-cells = 2 bit times), check collision
    a.set(SetDestination::PINS, 0); // disable TX
    a.set(SetDestination::X, 3); // 4 iterations
    a.bind(&mut silence_check);
    a.jmp(JmpCondition::PinHigh, &mut collision); // carrier sense HIGH → collision
    a.jmp(JmpCondition::XDecNonZero, &mut silence_check); // count down

    // Phase 3: Line clear — enable TX for frame
    a.set(SetDestination::PINS, 1); // enable TX
    a.pull(false, true); // pull block: get frame duration
    a.mov(
        pio::MovDestination::X,
        pio::MovOperation::None,
        pio::MovSource::OSR,
    ); // X = frame_halfcells
    a.bind(&mut frame_hold);
    a.jmp(JmpCondition::XDecNonZero, &mut frame_hold); // count down

    // Phase 4: Done — disable TX, signal success
    a.set(SetDestination::PINS, 0);
    a.irq(false, false, 1, false); // irq 1 (success)
    a.bind(&mut halt_ok);
    a.jmp(JmpCondition::Always, &mut halt_ok); // halt

    // Collision path
    a.bind(&mut collision);
    a.set(SetDestination::PINS, 0); // ensure TX off
    a.irq(false, false, 2, false); // irq 2 (collision)
    a.bind(&mut halt_col);
    a.jmp(JmpCondition::Always, &mut halt_col); // halt

    let prog = a.assemble_program();
    to_pio_program(&prog)
}

// ── SM2: RX Edge Timer + Carrier Sense ────────────────────────────────

/// Assemble the SM2 program: RX edge timer + carrier sense.
///
/// Measures inter-edge intervals on the RX input (JMP PIN).
/// Pushes 5-bit remaining-count values via IN (autopush at 30 or 32 bits).
/// Drives a carrier-sense SET pin (HIGH=busy, LOW=idle) for SM1.
///
/// Clock = 9.216 MHz (40× half-bit-rate). Each count loop = 2 cycles.
/// T ≈ 20 iterations, T/2 ≈ 10. Max counter = 31 ≈ 1.5T.
pub fn assemble_rx_edge_timer() -> PioProgram {
    let mut a = Assembler::<RP2040_MAX_PROGRAM_SIZE>::new();

    let mut start = a.label();
    let mut high_state = a.label();
    let mut low_state = a.label();
    let mut low_count = a.label();
    let mut low_edge = a.label();
    let mut high_count = a.label();
    let mut still_high = a.label();
    let mut timeout = a.label();
    let mut high_edge = a.label();

    // start:
    a.bind(&mut start);
    a.set(SetDestination::PINS, 0); // carrier sense = idle
    a.jmp(JmpCondition::PinHigh, &mut high_state); // check initial level

    // low_state: pin is LOW, wait for rising edge
    a.bind(&mut low_state);
    a.set(SetDestination::X, 31); // max counter
    a.bind(&mut low_count);
    a.jmp(JmpCondition::PinHigh, &mut low_edge); // pin HIGH → edge
    a.jmp(JmpCondition::XDecNonZero, &mut low_count); // count down
    a.jmp(JmpCondition::Always, &mut timeout); // counter exhausted

    // low_edge: rising edge detected
    a.bind(&mut low_edge);
    a.set(SetDestination::PINS, 1); // carrier sense = busy
    a.r#in(InSource::X, 5); // push remaining count

    // high_state: pin is HIGH, wait for falling edge
    a.bind(&mut high_state);
    a.set(SetDestination::X, 31); // max counter
    a.bind(&mut high_count);
    a.jmp(JmpCondition::PinHigh, &mut still_high); // still HIGH
    // pin went LOW → edge
    a.jmp(JmpCondition::Always, &mut high_edge);

    a.bind(&mut still_high);
    a.jmp(JmpCondition::XDecNonZero, &mut high_count); // count down
    // timeout from high:
    // fall through to timeout

    // timeout: no edge detected within counter range
    a.bind(&mut timeout);
    a.set(SetDestination::PINS, 0); // carrier sense = idle
    a.r#in(InSource::NULL, 5); // push 0 (timeout marker)
    a.jmp(JmpCondition::Always, &mut start); // restart

    // high_edge: falling edge detected
    a.bind(&mut high_edge);
    a.set(SetDestination::PINS, 1); // carrier sense = busy
    a.r#in(InSource::X, 5); // push remaining count
    a.jmp(JmpCondition::Always, &mut low_state); // now pin is LOW

    let prog = a.assemble_program();
    to_pio_program(&prog)
}

// ── Helper ────────────────────────────────────────────────────────────

fn to_pio_program(prog: &pio::Program<RP2040_MAX_PROGRAM_SIZE>) -> PioProgram {
    let mut instructions = [0u16; RP2040_MAX_PROGRAM_SIZE];
    let len = prog.code.len();
    for (i, &instr) in prog.code.iter().enumerate() {
        instructions[i] = instr;
    }
    PioProgram {
        instructions,
        len,
        wrap_target: prog.wrap.target,
        wrap: prog.wrap.source,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tx_data_assembles() {
        let prog = assemble_tx_data();
        assert_eq!(prog.len, 1);
        // out pins, 1 = 0110_0000_0000_0001 = 0x6001
        assert_eq!(prog.instructions[0], 0x6001);
    }

    #[test]
    fn tx_enable_assembles() {
        let prog = assemble_tx_enable();
        assert!(prog.len <= 32, "SM1 has {} instructions", prog.len);
        // First instruction should be `set pins, 1` = 0xE001
        assert_eq!(
            prog.instructions[0], 0xE001,
            "first instr should be set pins 1"
        );
    }

    #[test]
    fn rx_edge_timer_assembles() {
        let prog = assemble_rx_edge_timer();
        assert!(prog.len <= 32, "SM2 has {} instructions", prog.len);
        // First instruction should be `set pins, 0` = 0xE000
        assert_eq!(
            prog.instructions[0], 0xE000,
            "first instr should be set pins 0"
        );
    }

    #[test]
    fn programs_fit_in_pio_memory() {
        let sm0 = assemble_tx_data();
        let sm1 = assemble_tx_enable();
        let sm2 = assemble_rx_edge_timer();

        assert!(sm0.len <= 32);
        assert!(sm1.len <= 32);
        assert!(sm2.len <= 32);

        // SM0 can go on a separate PIO block, so SM1+SM2 must fit in 32.
        // If not, instruction sharing or PIO1 is needed.
        let combined = sm1.len + sm2.len;
        eprintln!(
            "SM0={}, SM1={}, SM2={}, SM1+SM2={}",
            sm0.len, sm1.len, sm2.len, combined
        );
        if combined > 32 {
            eprintln!(
                "WARNING: SM1+SM2 = {} > 32; need instruction sharing or split across PIO0/PIO1",
                combined
            );
        }
    }

    #[test]
    fn tx_data_wraps_on_itself() {
        let prog = assemble_tx_data();
        assert_eq!(prog.wrap_target, 0);
        assert_eq!(prog.wrap, 0);
    }
}
