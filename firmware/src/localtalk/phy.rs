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
/// RX clock: 12× the FM-0 half-cell rate. At 230.4 kbps, half-cell =
/// 460.8 kHz, so 12× = 5,529,600 Hz. One PIO cycle ≈ 0.181 µs.
/// Half-cell = 12 PIO cycles. Mid-cell sample at cycle ~6.
/// RX clock: 16× half-cell rate for wider sampling window.
/// Half-cell = 16 PIO cycles. Mid-cell at cycle 8.
const RX_CLOCK_HZ: u32 = 7_372_800;

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

    // FM-0 bit decoder. Detects boundary edges, delays to mid-cell,
    // samples to determine bit value. Pushes decoded bits via autopush.
    // Carrier sense on SET pin (HIGH during frame, LOW on idle).
    //
    // Clock: 12× half-cell → half-cell = 12 PIO cycles.
    // Mid-cell sample at ~6 cycles after boundary edge.
    // Idle timeout: ~2 cell periods (24 cycles) without edge.
    // TEMPORARY: raw pin sampler. Samples JMP PIN state at PIO clock rate.
    // With threshold=32 and left shift, each FIFO word = 32 consecutive
    // pin samples. Should show alternating patterns during FM-0 activity.
    // TEMPORARY: simplest possible sampler using `in pins, 1`.
    // Needs set_in_pins configured to GP4.
    let _rx_raw_sampler = pio_proc::pio_asm!(
        ".wrap_target",
        "    in pins, 1",
        ".wrap",
    );

    let rx_fm0_prog = pio_proc::pio_asm!(
        // === Idle: wait for first edge ===
        "idle:",
        "    set pins, 0",           // carrier sense LOW
        "    jmp pin, wait_fall",    // pin HIGH → wait for falling edge

        "wait_rise:",
        "    jmp pin, got_rise",     // pin HIGH → rising boundary
        "    jmp wait_rise",

        "wait_fall:",
        "    jmp pin, wf_cont",     // still HIGH → keep waiting
        "    jmp got_fall",          // pin LOW → falling boundary
        "wf_cont:",
        "    jmp wait_fall",

        // === Rising boundary (pin just went HIGH) ===
        "got_rise:",
        "    set pins, 1  [12]",     // carrier sense HIGH + delay 5 cycles
        "    jmp pin, rise_one",    // mid-cell: still HIGH → '1'
        // pin LOW → mid-cell transition → '0'
        "    in null, 1",           // shift in 0
        "    jmp wr_next",          // next boundary: wait for rising (pin LOW)

        "rise_one:",
        "    set x, 1",
        "    in x, 1",              // shift in 1
        "    jmp wf_next",          // next boundary: wait for falling (pin HIGH)

        // === Falling boundary (pin just went LOW) ===
        "got_fall:",
        "    set pins, 1  [12]",     // carrier sense HIGH + delay 5 cycles
        "    jmp pin, fall_zero",   // mid-cell: pin HIGH → transition → '0'
        // pin still LOW → no transition → '1'
        "    set x, 1",
        "    in x, 1",              // shift in 1
        "    jmp wr_next",          // next boundary: wait for rising (pin LOW)

        "fall_zero:",
        "    in null, 1",           // shift in 0
        "    jmp wf_next",          // next boundary: wait for falling (pin HIGH)

        // === Wait for next boundary with idle timeout ===
        "wr_next:",                 // wait for rising edge
        "    set x, 24",            // timeout counter
        "wr_loop:",
        "    jmp pin, got_rise",    // pin HIGH → rising boundary
        "    jmp x--, wr_loop",
        "    jmp idle",             // timeout → idle

        "wf_next:",                 // wait for falling edge
        "    set x, 24",            // timeout counter
        "wf_loop:",
        "    jmp pin, wf_high",     // pin HIGH → keep waiting
        "    jmp got_fall",         // pin LOW → falling boundary
        "wf_high:",
        "    jmp x--, wf_loop",
        "    jmp idle",             // timeout → idle
    );
    let rx_loaded = common1.load_program(&rx_fm0_prog.program);
    let rx_data_pin_obj = common1.make_pio_pin(rx_data_pin);
    let carrier_sense_pin_obj = common1.make_pio_pin(carrier_sense_pin);

    let mut rx_cfg = Config::default();
    rx_cfg.use_program(&rx_loaded, &[]);
    rx_cfg.set_set_pins(&[&carrier_sense_pin_obj]);
    rx_cfg.set_jmp_pin(&rx_data_pin_obj);
    rx_cfg.set_in_pins(&[&rx_data_pin_obj]);   // TEMP: for raw sampler
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

    // Leak all handles to prevent Drop from disabling SMs or resetting
    // pin function selects. Core 1 owns the hardware via PAC.
    mem::forget(tx_sm);
    mem::forget(common0);
    mem::forget(rx_sm);
    mem::forget(common1);
}
