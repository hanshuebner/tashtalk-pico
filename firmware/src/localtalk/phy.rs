//! PIO initialization and low-level PHY access.
//!
//! Configures PIO0 SM0 (TX shift-out) and PIO1 SM0 (RX FM-0 decoder).
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
/// 16× oversampling of the 230.4 kbps FM-0 bit rate.
/// Half bit cell = 8 PIO cycles, full cell = 16 cycles.
const RX_CLOCK_HZ: u32 = 3_686_400;

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

    // ── PIO1 SM0: Hardware FM-0 decoder ───────────────────────────────
    //
    // Decodes FM-0 (differential Manchester) directly in PIO hardware.
    // At 16× oversampling (3.6864 MHz), one bit cell = 16 PIO cycles,
    // half cell = 8 cycles.
    //
    // Algorithm: after each boundary transition (mandatory in FM-0),
    // delay to mid-cell and sample. If the pin level changed → mid-cell
    // transition → bit 0. If unchanged → bit 1. The `wait` instructions
    // serve double duty: edge detection during idle AND mid-cell delay
    // provider when jumped to from the boundary polling loop.
    //
    // Frame end: when no boundary arrives within the timeout (~1.75
    // cells), pushes 0xFFFFFFFF as frame-end marker and returns to idle.
    //
    // Output: autopush every 8 decoded bits. Each FIFO word has one
    // byte of decoded FM-0 data in bits [31:24]. Frame-end marker
    // 0xFFFFFFFF is distinguishable (normal words have [23:0] = 0).
    let rx_fm0_prog = pio_proc::pio_asm!(
        // One-time init: y = 0xFFFFFFFF. Bit 0 = 1 for `in y, 1`.
        "    mov y, !null",

        ".wrap_target",
        // ── Idle: clear ISR, detect first transition ──
        "    mov isr, null",
        "    jmp pin, falling",        // pin HIGH → wait for falling edge

        // ── Rising edge entry (idle LOW or mid-frame boundary) ──
        "rising:",
        "    wait 1 pin 0 [7]",        // wait for/confirm HIGH + 8-cycle delay
        "    jmp pin, high_b1",        // sample: still HIGH → bit 1
        "    in null, 1",              // went LOW at mid-cell → bit 0
        "    set x, 7",
        "high_b0_wait:",
        "    jmp pin, rising",         // rising edge → re-enter
        "    jmp x--, high_b0_wait",
        "    jmp frame_end",           // timeout

        "high_b1:",
        "    in y, 1",                 // bit 1
        "    set x, 7",
        "high_b1_wait:",
        "    jmp pin, high_b1_stay",   // still HIGH → keep waiting
        "    jmp falling",             // went LOW → falling edge path
        "high_b1_stay:",
        "    jmp x--, high_b1_wait",
        "    jmp frame_end",           // timeout

        // ── Falling edge entry (idle HIGH or mid-frame boundary) ──
        "falling:",
        "    wait 0 pin 0 [7]",        // wait for/confirm LOW + 8-cycle delay
        "    jmp pin, low_b0",         // sample: went HIGH → bit 0
        "    in y, 1",                 // still LOW → bit 1
        "    set x, 7",
        "low_b1_wait:",
        "    jmp pin, rising",         // rising edge → rising path
        "    jmp x--, low_b1_wait",
        "    jmp frame_end",           // timeout

        "low_b0:",
        "    in null, 1",              // bit 0
        "    set x, 7",
        "low_b0_wait:",
        "    jmp pin, low_b0_stay",    // still HIGH → keep waiting
        "    jmp falling",             // went LOW → falling edge path
        "low_b0_stay:",
        "    jmp x--, low_b0_wait",
        // timeout falls through ↓

        "frame_end:",
        "    mov isr, !null",          // ISR = 0xFFFFFFFF
        "    push noblock",            // push frame-end marker
        ".wrap",
    );
    let rx_loaded = common1.load_program(&rx_fm0_prog.program);
    let rx_data_pin_obj = common1.make_pio_pin(rx_data_pin);
    // GP5 not used by PIO — consume pin to prevent accidental reuse.
    let _carrier_sense_pin_obj = common1.make_pio_pin(carrier_sense_pin);

    let mut rx_cfg = Config::default();
    rx_cfg.use_program(&rx_loaded, &[]);
    rx_cfg.set_jmp_pin(&rx_data_pin_obj);
    rx_cfg.set_in_pins(&[&rx_data_pin_obj]);
    rx_cfg.shift_in = ShiftConfig {
        threshold: 8,
        direction: ShiftDirection::Right,
        auto_fill: true,
    };
    rx_cfg.fifo_join = FifoJoin::RxOnly;
    rx_cfg.clock_divider = clock_div(RX_CLOCK_HZ);
    rx_sm.set_config(&rx_cfg);
    rx_sm.set_pin_dirs(Direction::In, &[&rx_data_pin_obj]);

    rx_sm.set_enable(true);

    // Leak all handles to prevent Drop from disabling SMs or resetting
    // pin function selects. Core 1 owns the hardware via PAC.
    mem::forget(tx_sm);
    mem::forget(common0);
    mem::forget(rx_sm);
    mem::forget(common1);
}
