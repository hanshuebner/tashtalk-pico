//! CYW43 WiFi initialization and embassy-net stack for Pico W.
//!
//! Three tasks run on core 0 alongside USB and debug:
//! - `cyw43_task`: drives the CYW43 chip SPI bus
//! - `net_task`: drives the smoltcp network stack
//! - `wifi_task`: joins WiFi and handles reconnection

use core::sync::atomic::{AtomicU8, Ordering};

use cyw43::JoinOptions;
use cyw43_pio::PioSpi;
use defmt::{info, warn};
use embassy_net::StackResources;
use embassy_rp::dma;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{DMA_CH2, PIN_23, PIN_24, PIN_25, PIN_29, PIO0};
use embassy_rp::Peri;
use static_cell::StaticCell;

use crate::localtalk::phy::Pio0Remainder;

// ── WiFi state tracking ────────────────────────────────────────────

const STATE_INIT: u8 = 0;
const STATE_FW_LOADING: u8 = 1;
const STATE_FW_LOADED: u8 = 2;
const STATE_JOINING: u8 = 3;
const STATE_CONNECTED: u8 = 4;
const STATE_DISCONNECTED: u8 = 5;

static WIFI_STATE: AtomicU8 = AtomicU8::new(STATE_INIT);

pub fn state_str() -> &'static str {
    match WIFI_STATE.load(Ordering::Relaxed) {
        STATE_INIT => "init",
        STATE_FW_LOADING => "fw-loading",
        STATE_FW_LOADED => "ready",
        STATE_JOINING => "joining",
        STATE_CONNECTED => "connected",
        STATE_DISCONNECTED => "disconnected",
        _ => "unknown",
    }
}

// ── Firmware blobs (included at compile time) ──────────────────────

static FW: &cyw43::Aligned<cyw43::A4, [u8]> = cyw43::aligned_bytes!("../cyw43-fw/43439A0.bin");
static NVRAM: &cyw43::Aligned<cyw43::A4, [u8]> = cyw43::aligned_bytes!("../cyw43-fw/nvram_rp2040.bin");
static CLM: &[u8] = include_bytes!("../cyw43-fw/43439A0_clm.bin");

// ── WiFi credential storage ────────────────────────────────────────

/// Maximum SSID length (IEEE 802.11).
const MAX_SSID_LEN: usize = 32;
/// Maximum passphrase length (WPA2).
const MAX_PASS_LEN: usize = 63;

/// WiFi credentials stored in RAM (set via debug shell).
struct WifiConfig {
    ssid: heapless::String<MAX_SSID_LEN>,
    pass: heapless::String<MAX_PASS_LEN>,
}

static mut WIFI_CONFIG: WifiConfig = WifiConfig {
    ssid: heapless::String::new(),
    pass: heapless::String::new(),
};

/// Set WiFi credentials (called from debug shell).
pub fn set_credentials(ssid: &str, pass: &str) {
    unsafe {
        WIFI_CONFIG.ssid.clear();
        let _ = WIFI_CONFIG.ssid.push_str(ssid);
        WIFI_CONFIG.pass.clear();
        let _ = WIFI_CONFIG.pass.push_str(pass);
    }
}

/// Get current SSID (for status display).
pub fn ssid() -> &'static str {
    unsafe { WIFI_CONFIG.ssid.as_str() }
}

fn credentials() -> (&'static str, &'static str) {
    unsafe { (WIFI_CONFIG.ssid.as_str(), WIFI_CONFIG.pass.as_str()) }
}

// ── DMA interrupt binding ──────────────────────────────────────────

embassy_rp::bind_interrupts!(struct DmaIrqs {
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH2>;
});

// ── Network stack ──────────────────────────────────────────────────

pub type NetStack = embassy_net::Stack<'static>;

static mut STACK_REF: Option<&'static NetStack> = None;

/// Get a reference to the network stack (available after init).
pub fn stack() -> Option<&'static NetStack> {
    unsafe { STACK_REF }
}

// ── Task type aliases ──────────────────────────────────────────────

type Cyw43Spi = PioSpi<'static, PIO0, 0>;
type Cyw43Bus = cyw43::SpiBus<Output<'static>, Cyw43Spi>;
type Cyw43Runner = cyw43::Runner<'static, Cyw43Bus>;
type NetRunner = embassy_net::Runner<'static, cyw43::NetDriver<'static>>;

// ── Tasks ──────────────────────────────────────────────────────────

#[embassy_executor::task]
async fn cyw43_task(runner: Cyw43Runner) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn net_task(mut runner: NetRunner) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn wifi_task(mut control: cyw43::Control<'static>) -> ! {
    let mut join_failures: u32 = 0;
    loop {
        let (ssid, pass) = credentials();
        if ssid.is_empty() {
            join_failures = 0;
            embassy_time::Timer::after_secs(5).await;
            continue;
        }

        WIFI_STATE.store(STATE_JOINING, Ordering::Relaxed);
        if join_failures == 0 {
            tt_log!("WiFi: joining");
        }
        let opts = JoinOptions::new(pass.as_bytes());
        match control.join(ssid, opts).await {
            Ok(()) => {
                WIFI_STATE.store(STATE_CONNECTED, Ordering::Relaxed);
                tt_log!("WiFi: connected");
                join_failures = 0;
                loop {
                    embassy_time::Timer::after_secs(30).await;
                }
            }
            Err(_e) => {
                WIFI_STATE.store(STATE_DISCONNECTED, Ordering::Relaxed);
                if join_failures == 0 {
                    tt_log!("WiFi: join failed, retrying");
                }
                join_failures += 1;
                embassy_time::Timer::after_secs(5).await;
            }
        }
    }
}

// ── Initialization ─────────────────────────────────────────────────

static CYW43_STATE: StaticCell<cyw43::State> = StaticCell::new();
static NET_RESOURCES: StaticCell<StackResources<2>> = StaticCell::new();
static NET_STACK: StaticCell<NetStack> = StaticCell::new();

/// Initialize CYW43 WiFi and embassy-net stack, spawn tasks.
///
/// Must be called from within the executor closure (needs spawner + await).
pub async fn init(
    spawner: embassy_executor::Spawner,
    pio0_rem: Pio0Remainder,
    pin_23: Peri<'static, PIN_23>,
    pin_25: Peri<'static, PIN_25>,
    pin_24: Peri<'static, PIN_24>,
    pin_29: Peri<'static, PIN_29>,
    dma_ch: Peri<'static, DMA_CH2>,
) {
    info!("WiFi: initializing CYW43");
    WIFI_STATE.store(STATE_FW_LOADING, Ordering::Relaxed);
    tt_log!("WiFi: loading firmware...");

    // Power pin (active high) — directly consumed by cyw43::new().
    let pwr = Output::new(pin_23, Level::Low);

    // Chip select for PIO SPI (active low, idle high).
    let cs = Output::new(pin_25, Level::High);

    // PIO SPI transport — uses PIO0 SM1 (SM0 is LocalTalk TX).
    let Pio0Remainder { mut common, sm, irq0 } = pio0_rem;
    let dma_chan = dma::Channel::new(dma_ch, DmaIrqs);
    tt_log!("WiFi: PIO SPI setup...");
    let spi = PioSpi::new(
        &mut common,
        sm,
        cyw43_pio::DEFAULT_CLOCK_DIVIDER,
        irq0,
        cs,
        pin_24, // DIO
        pin_29, // CLK
        dma_chan,
    );
    tt_log!("WiFi: PIO SPI ready, loading fw...");

    // Initialize CYW43 driver.
    let state = CYW43_STATE.init(cyw43::State::new());
    tt_log!("WiFi: calling cyw43::new...");
    let (net_device, mut control, runner) = cyw43::new(state, pwr, spi, FW, NVRAM).await;
    tt_log!("WiFi: cyw43::new returned!");

    // Spawn the cyw43 runner BEFORE calling control.init() — the runner
    // must be processing SPI commands for control ioctls to complete.
    spawner.spawn(cyw43_task(runner).unwrap());

    control.init(CLM).await;
    control
        .set_power_management(cyw43::PowerManagementMode::PowerSave)
        .await;

    WIFI_STATE.store(STATE_FW_LOADED, Ordering::Relaxed);
    info!("WiFi: CYW43 initialized");
    tt_log!("WiFi: CYW43 ready");

    // Embassy-net stack with DHCP.
    let net_config = embassy_net::Config::dhcpv4(Default::default());

    let seed = 0x4c6f63616c54616c; // "LocalTal" as a fixed seed
    let resources = NET_RESOURCES.init(StackResources::new());
    let (stack, net_runner) = embassy_net::new(net_device, net_config, resources, seed);
    let stack_ref = NET_STACK.init(stack);
    unsafe { STACK_REF = Some(stack_ref) };

    info!("WiFi: network stack ready");

    // Spawn remaining background tasks.
    spawner.spawn(net_task(net_runner).unwrap());
    spawner.spawn(wifi_task(control).unwrap());
}
