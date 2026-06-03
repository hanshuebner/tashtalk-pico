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
const RX_CLOCK_HZ: u32 = 9_216_000; // 20× half-cell

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

    let de = Output::new(tx_en_pin, Level::Low);
    mem::forget(de);

    // ── PIO1: SM0 (RX FM-0 Interval Decoder) ──────────────────────
    //
    // Measures inter-edge intervals, classifies short/long, decodes
    // FM-0 bits. Pushes decoded bits via autopush at 8 (bytes).
    // On timeout (frame end), flushes partial bits via `push noblock`.
    //
    // When GP4 idles LOW (no bus bias or Mac connection), the `wait 0`
    // passes immediately and the PIO cycles through got_edge→timeout.
    // Each cycle pushes 0x00000000 via noblock (dropped if FIFO full).
    // This is harmless — software ignores zero words.
    //
    // 32 instructions exactly (wrap replaces final jmp got_edge).

    let Pio {
        common: mut common1,
        sm0: mut rx_sm,
        ..
    } = Pio::new(pio1, Pio1Irqs {});

    let rx_fm0_prog = pio_proc::pio_asm!(
        "idle:",
        "    in null, 8",            //  0: flush partial bits via autopush
        "    set pins, 0",           //  1: carrier sense LOW
        "    set y, 0",              //  2: Boundary state
        "    wait 1 pin 0",          //  3: wait for IN pin 0 HIGH (rising edge)

        ".wrap_target",
        "got_edge:",
        "    set pins, 1",           //  4: carrier sense HIGH
        "    set x, 2",              //  5: phase 1 (3 iterations) — force most to phase 2
        "    jmp pin, p1_hi",        //  6

        "p1_lo:",
        "    jmp pin, short_int",    //  7
        "    jmp x--, p1_lo",        //  8
        "    set x, 15",             //  9: phase 2
        "p2_lo:",
        "    jmp pin, long_int",     // 10
        "    jmp x--, p2_lo",        // 11
        "    jmp idle",              // 12: timeout

        "p1_hi:",
        "    jmp pin, p1_hi_c",      // 13
        "    jmp short_int",         // 14
        "p1_hi_c:",
        "    jmp x--, p1_hi",        // 15
        "    set x, 15",             // 16: phase 2
        "p2_hi:",
        "    jmp pin, p2_hi_c",      // 17
        "    jmp long_int",          // 18
        "p2_hi_c:",
        "    jmp x--, p2_hi",        // 19
        "    jmp idle",              // 20: timeout

        "short_int:",
        "    jmp !y, short_bnd",     // 21
        "reset_y:",
        "    set y, 0",              // 22
        "    jmp got_edge",          // 23

        "short_bnd:",
        "    in null, 1",            // 24: push '0'
        "    set y, 1",              // 25: → MidCell
        "    jmp got_edge",          // 26

        "long_int:",
        "    jmp !y, long_bnd",      // 27
        "    jmp reset_y",           // 28: error → Boundary

        "long_bnd:",
        "    set x, 1",              // 29
        "    in x, 1",               // 30: push '1'
        ".wrap",                     // 31 → got_edge (4)
    );

    let rx_loaded = common1.load_program(&rx_fm0_prog.program);
    let rx_data_pin_obj = common1.make_pio_pin(rx_data_pin);
    let carrier_sense_pin_obj = common1.make_pio_pin(carrier_sense_pin);

    let mut rx_cfg = Config::default();
    rx_cfg.use_program(&rx_loaded, &[]);
    rx_cfg.set_set_pins(&[&carrier_sense_pin_obj]);
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
    rx_sm.set_pin_dirs(Direction::Out, &[&carrier_sense_pin_obj]);

    rx_sm.set_enable(true);

    mem::forget(tx_sm);
    mem::forget(common0);
    mem::forget(rx_sm);
    mem::forget(common1);
}
