//! PIO initialization and low-level PHY access.

use core::mem;

use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{PIO0, PIO1};
use embassy_rp::pio::{
    Config, Direction, FifoJoin, InterruptHandler, Pio, PioPin, ShiftConfig, ShiftDirection,
};
use embassy_rp::Peripheral;
use fixed::types::U24F8;

embassy_rp::bind_interrupts!(pub struct Pio0Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

embassy_rp::bind_interrupts!(pub struct Pio1Irqs {
    PIO1_IRQ_0 => InterruptHandler<PIO1>;
});

const SYS_CLK_HZ: u32 = 125_000_000;
const TX_CLOCK_HZ: u32 = 460_800;
/// RX clock: 20× half-cell rate. Half-cell = 20 PIO cycles.
/// Edge wait loop = 2 cycles/iteration → 10 iterations per half-cell.
const RX_CLOCK_HZ: u32 = 9_216_000;

const fn clock_div(target_hz: u32) -> U24F8 {
    let div_x256 = (SYS_CLK_HZ as u64 * 256 + target_hz as u64 / 2) / target_hz as u64;
    U24F8::from_bits(div_x256 as u32)
}

pub fn init_phy(
    pio0: impl Peripheral<P = PIO0> + 'static,
    _pio0_irqs: Pio0Irqs,
    pio1: impl Peripheral<P = PIO1> + 'static,
    _pio1_irqs: Pio1Irqs,
    tx_data_pin: impl Peripheral<P = impl PioPin> + 'static,
    tx_en_pin: impl Peripheral<P = impl embassy_rp::gpio::Pin> + 'static,
    rx_data_pin: impl Peripheral<P = impl PioPin> + 'static,
    carrier_sense_pin: impl Peripheral<P = impl PioPin> + 'static,
) {
    // ── PIO0: SM0 (TX Data Shift-out) ──────────────────────────────

    let Pio {
        common: mut common0,
        sm0: mut tx_sm,
        ..
    } = Pio::new(pio0, Pio0Irqs {});

    let tx_data_prog = pio_proc::pio_asm!(
        ".wrap_target",
        "    out pins, 1",
        ".wrap",
    );
    let tx_data_loaded = common0.load_program(&tx_data_prog.program);
    let tx_data_pin_obj = common0.make_pio_pin(tx_data_pin);

    let mut tx_data_cfg = Config::default();
    tx_data_cfg.use_program(&tx_data_loaded, &[]);
    tx_data_cfg.set_out_pins(&[&tx_data_pin_obj]);
    tx_data_cfg.shift_out = ShiftConfig {
        threshold: 32,
        direction: ShiftDirection::Right,
        auto_fill: true,
    };
    tx_data_cfg.fifo_join = FifoJoin::TxOnly;
    tx_data_cfg.clock_divider = clock_div(TX_CLOCK_HZ);
    tx_sm.set_config(&tx_data_cfg);
    tx_sm.set_pin_dirs(Direction::Out, &[&tx_data_pin_obj]);

    // ── GP3: TX Driver Enable (GPIO, not PIO) ──────────────────────
    let de = Output::new(tx_en_pin, Level::Low);
    mem::forget(de);

    // ── PIO1: SM0 (RX FM-0 Interval Decoder) ──────────────────────
    //
    // Two-phase interval measurement with FM-0 state machine in PIO.
    // Y register: 0 = Boundary, non-zero = MidCell.
    //
    // Phase 1: count down from 7 (= half-cell threshold at 8 iterations).
    //   Edge before counter expires → short interval (half-cell).
    // Phase 2: count down from 15 (timeout margin).
    //   Edge in phase 2 → long interval (full cell).
    //   Counter expires → idle timeout.
    //
    // Carrier sense on SET pin. Autopush decoded bits at 8 (bytes).

    let Pio {
        common: mut common1,
        sm0: mut rx_sm,
        ..
    } = Pio::new(pio1, Pio1Irqs {});

    let rx_fm0_prog = pio_proc::pio_asm!(
        "idle:",
        "    set pins, 0",           // carrier sense LOW
        "    set y, 0",              // state = Boundary
        "    wait 0 pin 0",          // wait for pin LOW (first falling edge)

        "got_edge:",
        "    set pins, 1",            // carrier sense HIGH
        "    set x, 12",             // phase 1: 13 iter. Half-cell=10 → x≈2. Full-cell=20 → expires at 13.
        "    jmp pin, p1_hi",        // branch on current level

        // Phase 1 LOW: wait for rising edge (short interval check)
        "p1_lo:",
        "    jmp pin, short_int",    // pin HIGH → short interval
        "    jmp x--, p1_lo",
        // Counter expired → phase 2 (long interval range)
        "    set x, 15",
        "p2_lo:",
        "    jmp pin, long_int",     // pin HIGH → long interval
        "    jmp x--, p2_lo",
        "    jmp idle",              // timeout

        // Phase 1 HIGH: wait for falling edge
        "p1_hi:",
        "    jmp pin, p1_hi_c",      // still HIGH → continue
        "    jmp short_int",         // pin LOW → short interval
        "p1_hi_c:",
        "    jmp x--, p1_hi",
        // Counter expired → phase 2
        "    set x, 15",
        "p2_hi:",
        "    jmp pin, p2_hi_c",      // still HIGH
        "    jmp long_int",          // pin LOW → long interval
        "p2_hi_c:",
        "    jmp x--, p2_hi",
        "    jmp idle",              // timeout

        // === Short interval (half-cell) ===
        "short_int:",
        "    jmp !y, short_bnd",     // Y=0 → Boundary
        // MidCell + short → complete '0', back to Boundary
        "    set y, 0",
        "    jmp got_edge",

        "short_bnd:",                // Boundary + short → '0' bit
        "    in null, 1",            // push 0
        "    set y, 1",              // enter MidCell
        "    jmp got_edge",

        // === Long interval (full cell) ===
        "long_int:",
        "    jmp !y, long_bnd",      // Y=0 → Boundary
        // MidCell + long → error, reset
        "    set y, 0",
        "    jmp got_edge",

        "long_bnd:",                 // Boundary + long → '1' bit
        "    set x, 1",
        "    in x, 1",               // push 1
        "    jmp got_edge",
    );

    let rx_loaded = common1.load_program(&rx_fm0_prog.program);
    let rx_data_pin_obj = common1.make_pio_pin(rx_data_pin);
    let carrier_sense_pin_obj = common1.make_pio_pin(carrier_sense_pin);

    let mut rx_cfg = Config::default();
    rx_cfg.use_program(&rx_loaded, &[]);
    rx_cfg.set_set_pins(&[&carrier_sense_pin_obj]);
    rx_cfg.set_jmp_pin(&rx_data_pin_obj);
    rx_cfg.set_in_pins(&[&rx_data_pin_obj]); // for wait instruction
    rx_cfg.shift_in = ShiftConfig {
        threshold: 8,                        // autopush every 8 decoded bits
        direction: ShiftDirection::Right,    // LSB-first (HDLC bit order)
        auto_fill: true,
    };
    rx_cfg.fifo_join = FifoJoin::RxOnly;
    rx_cfg.clock_divider = clock_div(RX_CLOCK_HZ);
    rx_sm.set_config(&rx_cfg);
    rx_sm.set_pin_dirs(Direction::In, &[&rx_data_pin_obj]);
    rx_sm.set_pin_dirs(Direction::Out, &[&carrier_sense_pin_obj]);

    rx_sm.set_enable(true);

    mem::forget(tx_sm);
    mem::forget(common0);
    mem::forget(rx_sm);
    mem::forget(common1);
}
