//! PIO initialization and low-level PHY access.
//!
//! Configures PIO0 SM0 (TX shift-out) and PIO1 SM0 (RX edge timer).
//! After init, core 1 accesses these via PAC — the Embassy HAL handles
//! are leaked (mem::forget) to prevent Drop from disabling the SMs
//! or resetting pin function selects.

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
const RX_CLOCK_HZ: u32 = 9_216_000;

const fn clock_div(target_hz: u32) -> U24F8 {
    let div_x256 = (SYS_CLK_HZ as u64 * 256 + target_hz as u64 / 2) / target_hz as u64;
    U24F8::from_bits(div_x256 as u32)
}

/// Initialize PIO state machines and pin configuration.
///
/// After this call:
/// - PIO0 SM0 is configured for TX but NOT enabled (core 1 enables per-TX).
/// - PIO1 SM0 is configured and ENABLED for continuous RX.
/// - GP3 (DE) is configured as SIO output, driven low.
///
/// All Embassy handles are leaked via `mem::forget` to prevent their
/// Drop impls from disabling state machines or resetting pin function
/// selects. Core 1 accesses hardware via PAC registers.
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
    // SM0 left disabled — core 1 enables it per-TX.

    // ── GP3: TX Driver Enable (GPIO, not PIO) ──────────────────────
    let de = Output::new(tx_en_pin, Level::Low);
    mem::forget(de); // Keep pin configured as SIO output.

    // ── PIO1: SM0 (RX Edge Timer + Carrier Sense) ──────────────────

    let Pio {
        common: mut common1,
        sm0: mut rx_sm,
        ..
    } = Pio::new(pio1, Pio1Irqs {});

    let rx_edge_prog = pio_proc::pio_asm!(
        "start:",
        "    set pins, 0",
        "    jmp pin, high_state",

        "low_state:",
        "    set x, 31",
        "low_count:",
        "    jmp pin, low_edge",
        "    jmp x--, low_count",
        "    jmp timeout",

        "low_edge:",
        "    set pins, 1",
        "    in x, 5",

        "high_state:",
        "    set x, 31",
        "high_count:",
        "    jmp pin, still_high",
        "    jmp high_edge",

        "still_high:",
        "    jmp x--, high_count",

        "timeout:",
        "    set pins, 0",
        "    in null, 5",
        "    jmp start",

        "high_edge:",
        "    set pins, 1",
        "    in x, 5",
        "    jmp low_state",
    );
    let rx_loaded = common1.load_program(&rx_edge_prog.program);
    let rx_data_pin_obj = common1.make_pio_pin(rx_data_pin);
    let carrier_sense_pin_obj = common1.make_pio_pin(carrier_sense_pin);

    let mut rx_cfg = Config::default();
    rx_cfg.use_program(&rx_loaded, &[]);
    rx_cfg.set_set_pins(&[&carrier_sense_pin_obj]);
    rx_cfg.set_jmp_pin(&rx_data_pin_obj);
    rx_cfg.shift_in = ShiftConfig {
        threshold: 5,
        direction: ShiftDirection::Left,
        auto_fill: true,
    };
    rx_cfg.fifo_join = FifoJoin::RxOnly;
    rx_cfg.clock_divider = clock_div(RX_CLOCK_HZ);
    rx_sm.set_config(&rx_cfg);
    rx_sm.set_pin_dirs(Direction::In, &[&rx_data_pin_obj]);
    rx_sm.set_pin_dirs(Direction::Out, &[&carrier_sense_pin_obj]);

    rx_sm.set_enable(true);

    // Leak all handles to prevent Drop from disabling SMs or resetting
    // pin function selects. Core 1 owns the hardware via PAC.
    mem::forget(tx_sm);
    mem::forget(common0);
    mem::forget(rx_sm);
    mem::forget(common1);
}
