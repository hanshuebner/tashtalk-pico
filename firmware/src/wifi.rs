//! CYW43 WiFi initialization and embassy-net stack for Pico W.
//!
//! Three tasks run on core 0 alongside USB and debug:
//! - `cyw43_task`: drives the CYW43 chip SPI bus
//! - `net_task`: drives the smoltcp network stack
//! - `wifi_task`: joins WiFi and handles reconnection

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

// ── Firmware blobs (included at compile time) ──────────────────────

static FW: &cyw43::Aligned<cyw43::A4, [u8]> =
    &cyw43::Aligned(*include_bytes!("../cyw43-fw/43439A0.bin"));
static NVRAM: &cyw43::Aligned<cyw43::A4, [u8]> =
    &cyw43::Aligned(*include_bytes!("../cyw43-fw/nvram_rp2040.bin"));
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

type Cyw43Spi = PioSpi<'static, PIO0, 1>;
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
    loop {
        let (ssid, pass) = credentials();
        if ssid.is_empty() {
            info!("WiFi: no SSID configured, waiting...");
            embassy_time::Timer::after_secs(5).await;
            continue;
        }

        info!("WiFi: joining '{}'", ssid);
        let opts = JoinOptions::new(pass.as_bytes());
        match control.join(ssid, opts).await {
            Ok(()) => {
                info!("WiFi: joined '{}'", ssid);
                // Stay connected — if the connection drops, cyw43 will
                // report it and we'll loop back to rejoin.
                // For now, just wait and let the net stack handle DHCP.
                loop {
                    embassy_time::Timer::after_secs(30).await;
                    // TODO: check link state and break if disconnected
                }
            }
            Err(e) => {
                warn!("WiFi: join failed: {}", e);
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
    dma_ch2: Peri<'static, DMA_CH2>,
) {
    info!("WiFi: initializing CYW43");

    // Power pin (active high) — directly consumed by cyw43::new().
    let pwr = Output::new(pin_23, Level::Low);

    // Chip select for PIO SPI (active low, idle high).
    let cs = Output::new(pin_25, Level::High);

    // PIO SPI transport — uses PIO0 SM1 (SM0 is LocalTalk TX).
    let Pio0Remainder { mut common, sm1, irq0 } = pio0_rem;
    let dma_ch = dma::Channel::new(dma_ch2, DmaIrqs);
    let spi = PioSpi::new(
        &mut common,
        sm1,
        cyw43_pio::DEFAULT_CLOCK_DIVIDER,
        irq0,
        cs,
        pin_24, // DIO
        pin_29, // CLK
        dma_ch,
    );

    // Initialize CYW43 driver.
    let state = CYW43_STATE.init(cyw43::State::new());
    let (net_device, mut control, runner) = cyw43::new(state, pwr, spi, FW, NVRAM).await;

    control.init(CLM).await;
    control
        .set_power_management(cyw43::PowerManagementMode::PowerSave)
        .await;

    info!("WiFi: CYW43 initialized");

    // Embassy-net stack with DHCP.
    let net_config = embassy_net::Config::dhcpv4(Default::default());

    let seed = 0x4c6f63616c54616c; // "LocalTal" as a fixed seed
    let resources = NET_RESOURCES.init(StackResources::new());
    let (stack, net_runner) = embassy_net::new(net_device, net_config, resources, seed);
    let stack_ref = NET_STACK.init(stack);
    unsafe { STACK_REF = Some(stack_ref) };

    info!("WiFi: network stack ready");

    // Spawn background tasks.
    spawner.spawn(cyw43_task(runner).unwrap());
    spawner.spawn(net_task(net_runner).unwrap());
    spawner.spawn(wifi_task(control).unwrap());
}
