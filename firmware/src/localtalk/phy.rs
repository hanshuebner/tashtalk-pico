//! PIO initialization and low-level PHY access.
//!
//! Loads the three PIO programs, configures clock dividers and pin
//! mappings, and provides the TX/RX handles used by the async tasks.

use embassy_rp::peripherals::{PIO0, PIO1};
use embassy_rp::pio::{
    Config, Direction, FifoJoin, InterruptHandler, Pio, PioPin, ShiftConfig, ShiftDirection,
    StateMachine,
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

pub struct LtTxData<'d> {
    pub sm: StateMachine<'d, PIO0, 0>,
}

pub struct LtTxEnable<'d> {
    pub sm: StateMachine<'d, PIO0, 1>,
}

pub struct LtRx<'d> {
    pub sm: StateMachine<'d, PIO1, 0>,
}

pub fn init_phy<'d>(
    pio0: impl Peripheral<P = PIO0> + 'd,
    _pio0_irqs: Pio0Irqs,
    pio1: impl Peripheral<P = PIO1> + 'd,
    _pio1_irqs: Pio1Irqs,
    tx_data_pin: impl Peripheral<P = impl PioPin> + 'd,
    tx_en_pin: impl Peripheral<P = impl PioPin> + 'd,
    rx_data_pin: impl Peripheral<P = impl PioPin> + 'd,
    carrier_sense_pin: impl Peripheral<P = impl PioPin> + 'd,
) -> (LtTxData<'d>, LtTxEnable<'d>, LtRx<'d>) {
    // ── PIO0: SM0 (TX Data) + SM1 (TX Enable) ──────────────────────

    let Pio {
        mut common,
        mut sm0,
        mut sm1,
        ..
    } = Pio::new(pio0, Pio0Irqs {});

    // SM0: TX Data Shift-out
    let tx_data_prog = pio_proc::pio_asm!(
        ".wrap_target",
        "    out pins, 1",
        ".wrap",
    );
    let tx_data_loaded = common.load_program(&tx_data_prog.program);
    let tx_data_pin_obj = common.make_pio_pin(tx_data_pin);

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
    sm0.set_config(&tx_data_cfg);
    sm0.set_pin_dirs(Direction::Out, &[&tx_data_pin_obj]);

    // SM1: TX Enable + Collision Gate
    let tx_en_prog = pio_proc::pio_asm!(
        // Phase 1: Sync pulse (2 half-cells = 1 bit time)
        "    set pins, 1",
        "    set x, 0",
        "    jmp x--, silence",

        // Phase 2: Silence (4 half-cells, check carrier sense)
        "silence:",
        "    set pins, 0",
        "    set x, 3",
        "silence_check:",
        "    jmp pin, collision",
        "    jmp x--, silence_check",

        // Phase 3: Frame
        "    set pins, 1",
        "    pull block",
        "    mov x, osr",
        "frame_hold:",
        "    jmp x--, frame_hold",

        // Phase 4: Done
        "    set pins, 0",
        "    irq set 1",
        "halt_ok:",
        "    jmp halt_ok",

        // Collision path
        "collision:",
        "    set pins, 0",
        "    irq set 2",
        "halt_col:",
        "    jmp halt_col",
    );
    let tx_en_loaded = common.load_program(&tx_en_prog.program);
    let tx_en_pin_obj = common.make_pio_pin(tx_en_pin);

    let mut tx_en_cfg = Config::default();
    tx_en_cfg.use_program(&tx_en_loaded, &[]);
    tx_en_cfg.set_set_pins(&[&tx_en_pin_obj]);
    tx_en_cfg.clock_divider = clock_div(TX_CLOCK_HZ);
    sm1.set_config(&tx_en_cfg);
    sm1.set_pin_dirs(Direction::Out, &[&tx_en_pin_obj]);

    // SM1 JMP PIN reads GP5 (carrier sense from SM2).
    embassy_rp::pac::PIO0.sm(1).execctrl().modify(|w| {
        w.set_jmp_pin(5);
    });

    // ── PIO1: SM0 (RX Edge Timer + Carrier Sense) ──────────────────

    let Pio {
        mut common,
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
    let rx_loaded = common.load_program(&rx_edge_prog.program);
    let rx_data_pin_obj = common.make_pio_pin(rx_data_pin);
    let carrier_sense_pin_obj = common.make_pio_pin(carrier_sense_pin);

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

    (
        LtTxData { sm: sm0 },
        LtTxEnable { sm: sm1 },
        LtRx { sm: rx_sm },
    )
}
