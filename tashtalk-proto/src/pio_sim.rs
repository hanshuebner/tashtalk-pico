//! Minimal PIO state machine simulator for host-side testing.
//!
//! Supports the instruction subset used by our LocalTalk PIO programs:
//! `OUT`, `SET`, `JMP`, `IN`, `PULL`, `MOV`, `IRQ`, and `NOP`.
//!
//! The simulator executes one instruction per call to [`step`](PioSim::step),
//! applying the instruction's effect to simulated registers, pins, and FIFOs.
//! Delay/side-set cycles are not simulated (all instructions take 1 cycle).

use pio::RP2040_MAX_PROGRAM_SIZE;

/// Simulated PIO state machine.
#[derive(Debug, Clone)]
pub struct PioSim {
    /// Program instructions.
    pub program: [u16; RP2040_MAX_PROGRAM_SIZE],
    /// Number of valid instructions.
    pub program_len: usize,
    /// Program counter.
    pub pc: usize,
    /// Scratch register X.
    pub x: u32,
    /// Scratch register Y.
    pub y: u32,
    /// Output shift register.
    pub osr: u32,
    /// Number of bits shifted out of OSR.
    pub osr_shift_count: u32,
    /// Input shift register.
    pub isr: u32,
    /// Number of bits shifted into ISR.
    pub isr_shift_count: u32,
    /// Output pins (directly driven by `out pins` / `set pins`).
    pub out_pins: u32,
    /// SET pins (driven by `set pins`).
    pub set_pins: u32,
    /// JMP PIN input (caller sets this before each step).
    pub jmp_pin: bool,
    /// TX FIFO (data from CPU → PIO).
    pub tx_fifo: heapless::Deque<u32, 8>,
    /// RX FIFO (data from PIO → CPU).
    pub rx_fifo: heapless::Vec<u32, 256>,
    /// Autopull threshold (bits). 0 = disabled.
    pub autopull_threshold: u32,
    /// Autopush threshold (bits). 0 = disabled.
    pub autopush_threshold: u32,
    /// IRQ flags (bits 0-7).
    pub irq_flags: u8,
    /// `true` if the SM is stalled (e.g., pull block with empty FIFO).
    pub stalled: bool,
    /// Wrap target (PC jumps here after wrap source).
    pub wrap_target: u8,
    /// Wrap source (last instruction before wrap).
    pub wrap_source: u8,
    /// Total cycles executed.
    pub cycle_count: u64,
}

impl PioSim {
    /// Create a new simulator from a program.
    pub fn new(
        program: &[u16],
        wrap_target: u8,
        wrap_source: u8,
    ) -> Self {
        let mut prog = [0u16; RP2040_MAX_PROGRAM_SIZE];
        let len = program.len().min(RP2040_MAX_PROGRAM_SIZE);
        prog[..len].copy_from_slice(&program[..len]);

        Self {
            program: prog,
            program_len: len,
            pc: 0,
            x: 0,
            y: 0,
            osr: 0,
            osr_shift_count: 32, // OSR starts empty
            isr: 0,
            isr_shift_count: 0,
            out_pins: 0,
            set_pins: 0,
            jmp_pin: false,
            tx_fifo: heapless::Deque::new(),
            rx_fifo: heapless::Vec::new(),
            autopull_threshold: 0,
            autopush_threshold: 0,
            irq_flags: 0,
            stalled: false,
            wrap_target,
            wrap_source,
            cycle_count: 0,
        }
    }

    /// Load from a [`PioProgram`](crate::pio_programs::PioProgram).
    pub fn from_program(prog: &crate::pio_programs::PioProgram) -> Self {
        Self::new(
            &prog.instructions[..prog.len],
            prog.wrap_target,
            prog.wrap,
        )
    }

    /// Execute one instruction. Returns `true` if the instruction executed
    /// (not stalled).
    pub fn step(&mut self) -> bool {
        if self.pc >= self.program_len {
            return false;
        }

        // Autopull: if OSR is empty and TX FIFO has data, refill OSR.
        if self.autopull_threshold > 0 && self.osr_shift_count >= self.autopull_threshold {
            if let Some(val) = self.tx_fifo.pop_front() {
                self.osr = val;
                self.osr_shift_count = 0;
            }
        }

        let instr = self.program[self.pc];
        let op = (instr >> 13) & 0x7;
        self.stalled = false;

        match op {
            0b000 => self.exec_jmp(instr),
            0b001 => self.exec_wait(instr),
            0b010 => self.exec_in(instr),
            0b011 => self.exec_out(instr),
            0b100 => self.exec_push_pull(instr),
            0b101 => self.exec_mov(instr),
            0b110 => self.exec_irq(instr),
            0b111 => self.exec_set(instr),
            _ => unreachable!(),
        }

        self.cycle_count += 1;

        if !self.stalled {
            // Check autopush after IN instructions.
            if self.autopush_threshold > 0 && self.isr_shift_count >= self.autopush_threshold {
                let _ = self.rx_fifo.push(self.isr);
                self.isr = 0;
                self.isr_shift_count = 0;
            }
        }

        !self.stalled
    }

    /// Run for up to `max_cycles` steps. Returns number of cycles executed.
    pub fn run(&mut self, max_cycles: usize) -> usize {
        let mut executed = 0;
        for _ in 0..max_cycles {
            self.step();
            executed += 1;
            if self.stalled {
                break;
            }
        }
        executed
    }

    // ── Instruction implementations ───────────────────────────────────

    fn exec_jmp(&mut self, instr: u16) {
        let condition = (instr >> 5) & 0x7;
        let address = (instr & 0x1F) as usize;

        let take = match condition {
            0b000 => true,                           // always
            0b001 => self.x == 0,                    // !X
            0b010 => {                               // X--
                let nz = self.x != 0;
                self.x = self.x.wrapping_sub(1);
                nz
            }
            0b011 => self.y == 0,                    // !Y
            0b100 => {                               // Y--
                let nz = self.y != 0;
                self.y = self.y.wrapping_sub(1);
                nz
            }
            0b101 => self.x != self.y,               // X!=Y
            0b110 => self.jmp_pin,                   // PIN
            0b111 => self.osr_shift_count < 32,      // !OSRE
            _ => false,
        };

        if take {
            self.pc = address;
        } else {
            self.advance_pc();
        }
    }

    fn exec_wait(&mut self, _instr: u16) {
        // Minimal wait support: just advance PC.
        // Full WAIT implementation would block until condition met.
        self.advance_pc();
    }

    fn exec_in(&mut self, instr: u16) {
        let source = (instr >> 5) & 0x7;
        let bit_count = (instr & 0x1F) as u32;
        let bit_count = if bit_count == 0 { 32 } else { bit_count };

        let value = match source {
            0b000 => 0,             // PINS (not simulated, return 0)
            0b001 => self.x,        // X
            0b010 => self.y,        // Y
            0b011 => 0,             // NULL
            0b110 => self.isr,      // ISR
            0b111 => self.osr,      // OSR
            _ => 0,
        };

        // Shift ISR left and OR in the low bits of value.
        let mask = if bit_count >= 32 { u32::MAX } else { (1u32 << bit_count) - 1 };
        self.isr = (self.isr << bit_count) | (value & mask);
        self.isr_shift_count += bit_count;

        self.advance_pc();
    }

    fn exec_out(&mut self, instr: u16) {
        let destination = (instr >> 5) & 0x7;
        let bit_count = (instr & 0x1F) as u32;
        let bit_count = if bit_count == 0 { 32 } else { bit_count };

        // Extract low `bit_count` bits from OSR.
        let mask = if bit_count >= 32 { u32::MAX } else { (1u32 << bit_count) - 1 };
        let value = self.osr & mask;
        self.osr >>= bit_count;
        self.osr_shift_count += bit_count;

        match destination {
            0b000 => self.out_pins = value, // PINS
            0b001 => self.x = value,        // X
            0b010 => self.y = value,        // Y
            0b011 => {}                     // NULL (discard)
            0b100 => {}                     // PINDIRS (ignore)
            0b101 => {}                     // PC (not supported)
            0b110 => {}                     // ISR (not supported)
            0b111 => {}                     // EXEC (not supported)
            _ => {}
        }

        self.advance_pc();
    }

    fn exec_push_pull(&mut self, instr: u16) {
        let is_pull = (instr >> 5) & 1 == 1;
        let block = (instr >> 6) & 1 == 1;

        if is_pull {
            if let Some(val) = self.tx_fifo.pop_front() {
                self.osr = val;
                self.osr_shift_count = 0;
                self.advance_pc();
            } else if block {
                self.stalled = true;
                // Don't advance PC — retry next cycle.
            } else {
                // Non-blocking pull with empty FIFO: copy X to OSR.
                self.osr = self.x;
                self.osr_shift_count = 0;
                self.advance_pc();
            }
        } else {
            // PUSH
            let _ = self.rx_fifo.push(self.isr);
            self.isr = 0;
            self.isr_shift_count = 0;
            self.advance_pc();
        }
    }

    fn exec_mov(&mut self, instr: u16) {
        let destination = (instr >> 5) & 0x7;
        let operation = (instr >> 3) & 0x3;
        let source = instr & 0x7;

        let val = match source {
            0b000 => 0,             // PINS
            0b001 => self.x,        // X
            0b010 => self.y,        // Y
            0b011 => 0,             // NULL
            // 0b100 => reserved
            0b101 => 0,             // STATUS
            0b110 => self.isr,      // ISR
            0b111 => self.osr,      // OSR
            _ => 0,
        };

        let val = match operation {
            0b00 => val,            // None
            0b01 => !val,           // Invert
            0b10 => val.reverse_bits(), // Bit-reverse
            _ => val,
        };

        match destination {
            0b000 => self.out_pins = val, // PINS
            0b001 => self.x = val,        // X
            0b010 => self.y = val,        // Y
            // 0b011 => reserved
            0b100 => self.pc = val as usize, // EXEC (set PC)
            0b101 => {},                  // PC
            0b110 => {                    // ISR
                self.isr = val;
                self.isr_shift_count = 0;
            }
            0b111 => {                    // OSR
                self.osr = val;
                self.osr_shift_count = 0;
            }
            _ => {}
        }

        if destination != 0b100 && destination != 0b101 {
            self.advance_pc();
        }
    }

    fn exec_irq(&mut self, instr: u16) {
        let clear = (instr >> 6) & 1 == 1;
        let wait = (instr >> 5) & 1 == 1;
        let index = (instr & 0x1F) as u8;
        let flag_bit = 1u8 << (index & 0x7);

        if clear {
            self.irq_flags &= !flag_bit;
        } else {
            self.irq_flags |= flag_bit;
            if wait {
                // Wait for flag to be cleared (by CPU or another SM).
                // In simulation, just stall.
                self.stalled = true;
                return;
            }
        }

        self.advance_pc();
    }

    fn exec_set(&mut self, instr: u16) {
        let destination = (instr >> 5) & 0x7;
        let data = (instr & 0x1F) as u32;

        match destination {
            0b000 => self.set_pins = data,   // PINS
            0b001 => self.x = data,          // X
            0b010 => self.y = data,          // Y
            0b100 => {}                      // PINDIRS (ignore)
            _ => {}
        }

        self.advance_pc();
    }

    fn advance_pc(&mut self) {
        if self.pc as u8 == self.wrap_source {
            self.pc = self.wrap_target as usize;
        } else {
            self.pc += 1;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pio_programs::*;

    #[test]
    fn tx_data_shifts_out_bits() {
        let prog = assemble_tx_data();
        let mut sim = PioSim::from_program(&prog);
        sim.autopull_threshold = 32;

        // Push a known pattern: 0b1010_0101 in the low 8 bits.
        sim.tx_fifo.push_back(0xA5).unwrap();

        let mut output_bits = Vec::new();
        for _ in 0..32 {
            sim.step();
            output_bits.push(sim.out_pins as u8);
        }

        // Should shift out LSB first: 0xA5 = 1010_0101
        // Bits: 1,0,1,0,0,1,0,1, then 0s for remaining 24 bits
        assert_eq!(output_bits[0], 1); // bit 0
        assert_eq!(output_bits[1], 0); // bit 1
        assert_eq!(output_bits[2], 1); // bit 2
        assert_eq!(output_bits[3], 0); // bit 3
        assert_eq!(output_bits[4], 0); // bit 4
        assert_eq!(output_bits[5], 1); // bit 5
        assert_eq!(output_bits[6], 0); // bit 6
        assert_eq!(output_bits[7], 1); // bit 7
    }

    #[test]
    fn tx_data_autopulls_continuously() {
        let prog = assemble_tx_data();
        let mut sim = PioSim::from_program(&prog);
        sim.autopull_threshold = 32;

        sim.tx_fifo.push_back(0xFF).unwrap();
        sim.tx_fifo.push_back(0x00).unwrap();

        // First 32 cycles: all 1s (0xFF = low 8 bits all 1)
        for _ in 0..32 {
            sim.step();
        }
        // After 32 bits, autopull loads 0x00.
        sim.step();
        assert_eq!(sim.out_pins, 0);
    }

    #[test]
    fn rx_edge_timer_measures_intervals() {
        let prog = assemble_rx_edge_timer();
        let mut sim = PioSim::from_program(&prog);
        sim.autopush_threshold = 5; // push after every 5-bit IN

        // Simulate a rising edge after some LOW time, then a falling edge.
        // Start with pin LOW.
        sim.jmp_pin = false;

        // Run until we're in the low_count loop (should take a few cycles).
        for _ in 0..10 {
            sim.step();
        }

        // Now simulate a rising edge: set pin HIGH.
        sim.jmp_pin = true;

        // Run until edge is detected and count is pushed.
        let rx_count_before = sim.rx_fifo.len();
        for _ in 0..5 {
            sim.step();
        }

        // Carrier sense should be HIGH (busy).
        assert_eq!(sim.set_pins, 1, "carrier sense should be busy after edge");

        // Should have pushed at least one interval count.
        assert!(
            sim.rx_fifo.len() > rx_count_before,
            "should have pushed interval count"
        );
    }

    #[test]
    fn rx_edge_timer_timeout_sets_idle() {
        let prog = assemble_rx_edge_timer();
        let mut sim = PioSim::from_program(&prog);
        sim.autopush_threshold = 5;

        // Keep pin LOW for longer than the counter range (31 iterations × 2 cycles).
        sim.jmp_pin = false;

        // Run long enough for timeout.
        for _ in 0..80 {
            sim.step();
        }

        // After timeout, carrier sense should be LOW (idle).
        assert_eq!(sim.set_pins, 0, "carrier sense should be idle after timeout");
    }

    #[test]
    fn tx_enable_sync_pulse_timing() {
        let prog = assemble_tx_enable();
        let mut sim = PioSim::from_program(&prog);

        // No collision: carrier sense stays LOW.
        sim.jmp_pin = false;

        // Push frame duration.
        sim.tx_fifo.push_back(10).unwrap();

        let mut pin_trace: Vec<u32> = Vec::new();

        // Run until completion (IRQ set) or max cycles.
        for _ in 0..100 {
            sim.step();
            pin_trace.push(sim.set_pins);
            if sim.irq_flags != 0 || sim.stalled {
                break;
            }
        }

        // Verify sync pulse pattern:
        // - Starts HIGH (sync pulse ON)
        // - Goes LOW (silence)
        // - Goes HIGH again (frame)
        // - Ends LOW (done)
        assert_eq!(pin_trace[0], 1, "sync: TX should start enabled");

        // Find transitions.
        let transitions: Vec<(usize, u32)> = pin_trace
            .windows(2)
            .enumerate()
            .filter(|(_, w)| w[0] != w[1])
            .map(|(i, w)| (i + 1, w[1]))
            .collect();

        assert!(
            transitions.len() >= 3,
            "expected ≥3 transitions, got {:?}",
            transitions
        );
        // First transition: HIGH→LOW (end of sync pulse)
        assert_eq!(transitions[0].1, 0, "first transition should be to LOW");
        // Second transition: LOW→HIGH (start of frame)
        assert_eq!(transitions[1].1, 1, "second transition should be to HIGH");
        // Third transition: HIGH→LOW (end of frame)
        assert_eq!(transitions[2].1, 0, "third transition should be to LOW");

        // Should have set IRQ 1 (success).
        assert!(
            sim.irq_flags & (1 << 1) != 0,
            "IRQ 1 (success) should be set"
        );
    }

    #[test]
    fn tx_enable_collision_detected() {
        let prog = assemble_tx_enable();
        let mut sim = PioSim::from_program(&prog);

        // Simulate collision: carrier sense goes HIGH during silence phase.
        sim.jmp_pin = false;

        let mut pin_trace: Vec<u32> = Vec::new();

        for cycle in 0..100 {
            // After the sync pulse (first couple cycles), set carrier sense HIGH.
            if cycle == 4 {
                sim.jmp_pin = true; // collision!
            }

            sim.step();
            pin_trace.push(sim.set_pins);
            if sim.irq_flags != 0 || sim.stalled {
                break;
            }
        }

        // Should have set IRQ 2 (collision), not IRQ 1 (success).
        assert!(
            sim.irq_flags & (1 << 2) != 0,
            "IRQ 2 (collision) should be set, flags=0b{:08b}",
            sim.irq_flags
        );
        assert!(
            sim.irq_flags & (1 << 1) == 0,
            "IRQ 1 (success) should NOT be set"
        );

        // TX should be disabled at end.
        assert_eq!(*pin_trace.last().unwrap(), 0, "TX should be disabled");
    }
}
