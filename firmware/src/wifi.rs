//! CYW43 WiFi initialization and embassy-net stack for Pico W.
//!
//! Three tasks run on core 0 alongside USB and debug:
//! - `cyw43_task`: drives the CYW43 chip SPI bus
//! - `net_task`: drives the smoltcp network stack
//! - `wifi_task`: scans, joins strongest known network, handles reconnection

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
const STATE_SCANNING: u8 = 3;
const STATE_JOINING: u8 = 4;
const STATE_CONNECTED: u8 = 5;
const STATE_DISCONNECTED: u8 = 6;

static WIFI_STATE: AtomicU8 = AtomicU8::new(STATE_INIT);

pub fn state_str() -> &'static str {
    match WIFI_STATE.load(Ordering::Relaxed) {
        STATE_INIT => "init",
        STATE_FW_LOADING => "fw-loading",
        STATE_FW_LOADED => "ready",
        STATE_SCANNING => "scanning",
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

const MAX_SSID_LEN: usize = 32;
const MAX_PASS_LEN: usize = 63;
const MAX_NETWORKS: usize = 4;

struct NetworkEntry {
    ssid: heapless::String<MAX_SSID_LEN>,
    pass: heapless::String<MAX_PASS_LEN>,
}

struct WifiConfigList {
    entries: [NetworkEntry; MAX_NETWORKS],
    count: usize,
}

impl WifiConfigList {
    const fn new() -> Self {
        const EMPTY: NetworkEntry = NetworkEntry {
            ssid: heapless::String::new(),
            pass: heapless::String::new(),
        };
        Self {
            entries: [EMPTY; MAX_NETWORKS],
            count: 0,
        }
    }

    fn find(&self, ssid: &str) -> Option<usize> {
        for i in 0..self.count {
            if self.entries[i].ssid.as_str() == ssid {
                return Some(i);
            }
        }
        None
    }

    fn add(&mut self, ssid: &str, pass: &str) -> bool {
        // Update existing entry if SSID matches.
        if let Some(i) = self.find(ssid) {
            self.entries[i].pass.clear();
            let _ = self.entries[i].pass.push_str(pass);
            return true;
        }
        if self.count >= MAX_NETWORKS {
            return false;
        }
        self.entries[self.count].ssid.clear();
        let _ = self.entries[self.count].ssid.push_str(ssid);
        self.entries[self.count].pass.clear();
        let _ = self.entries[self.count].pass.push_str(pass);
        self.count += 1;
        true
    }

    fn remove(&mut self, ssid: &str) -> bool {
        if let Some(i) = self.find(ssid) {
            // Shift remaining entries down using a temp buffer.
            for j in i..self.count - 1 {
                let mut tmp_ssid = heapless::String::<MAX_SSID_LEN>::new();
                let mut tmp_pass = heapless::String::<MAX_PASS_LEN>::new();
                let _ = tmp_ssid.push_str(self.entries[j + 1].ssid.as_str());
                let _ = tmp_pass.push_str(self.entries[j + 1].pass.as_str());
                self.entries[j].ssid.clear();
                let _ = self.entries[j].ssid.push_str(tmp_ssid.as_str());
                self.entries[j].pass.clear();
                let _ = self.entries[j].pass.push_str(tmp_pass.as_str());
            }
            self.count -= 1;
            self.entries[self.count].ssid.clear();
            self.entries[self.count].pass.clear();
            true
        } else {
            false
        }
    }

    fn passphrase_for(&self, ssid: &str) -> Option<&str> {
        self.find(ssid).map(|i| self.entries[i].pass.as_str())
    }
}

static mut WIFI_CONFIGS: WifiConfigList = WifiConfigList::new();

/// Currently connected SSID (set by wifi_task on successful join).
static mut CONNECTED_SSID: heapless::String<MAX_SSID_LEN> = heapless::String::new();

/// Add a WiFi network (called from debug shell).
pub fn add_network(ssid: &str, pass: &str) -> bool {
    unsafe { WIFI_CONFIGS.add(ssid, pass) }
}

/// Remove a WiFi network (called from debug shell).
pub fn remove_network(ssid: &str) -> bool {
    unsafe { WIFI_CONFIGS.remove(ssid) }
}

/// Number of configured networks.
pub fn network_count() -> usize {
    unsafe { WIFI_CONFIGS.count }
}

/// Get configured network SSID by index.
pub fn network_ssid(idx: usize) -> &'static str {
    unsafe {
        if idx < WIFI_CONFIGS.count {
            WIFI_CONFIGS.entries[idx].ssid.as_str()
        } else {
            ""
        }
    }
}

/// Get the currently connected SSID.
pub fn connected_ssid() -> &'static str {
    unsafe { CONNECTED_SSID.as_str() }
}

// ── DMA interrupt binding ──────────────────────────────────────────

embassy_rp::bind_interrupts!(struct DmaIrqs {
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH2>;
});

// ── Network stack ──────────────────────────────────────────────────

pub type NetStack = embassy_net::Stack<'static>;

static mut STACK_REF: Option<&'static NetStack> = None;

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
        if unsafe { WIFI_CONFIGS.count } == 0 {
            embassy_time::Timer::after_secs(5).await;
            continue;
        }

        // Scan for networks and pick the strongest known SSID.
        WIFI_STATE.store(STATE_SCANNING, Ordering::Relaxed);
        let mut best_ssid: heapless::String<MAX_SSID_LEN> = heapless::String::new();
        let mut best_rssi: i16 = i16::MIN;

        {
            let mut scanner = control.scan(Default::default()).await;
            while let Some(bss) = scanner.next().await {
                let ssid_len = (bss.ssid_len as usize).min(32);
                if let Ok(ssid) = core::str::from_utf8(&bss.ssid[..ssid_len]) {
                    if unsafe { WIFI_CONFIGS.find(ssid).is_some() } && bss.rssi > best_rssi {
                        best_rssi = bss.rssi;
                        best_ssid.clear();
                        let _ = best_ssid.push_str(ssid);
                    }
                }
            }
        } // scanner dropped here, releasing &mut control

        if best_ssid.is_empty() {
            WIFI_STATE.store(STATE_DISCONNECTED, Ordering::Relaxed);
            info!("WiFi: no known network in scan");
            embassy_time::Timer::after_secs(10).await;
            continue;
        }

        // Join the best network.
        let pass = unsafe { WIFI_CONFIGS.passphrase_for(best_ssid.as_str()) }
            .unwrap_or("");
        WIFI_STATE.store(STATE_JOINING, Ordering::Relaxed);
        info!("WiFi: joining '{}'", best_ssid.as_str());
        let opts = JoinOptions::new(pass.as_bytes());
        match control.join(best_ssid.as_str(), opts).await {
            Ok(()) => {
                WIFI_STATE.store(STATE_CONNECTED, Ordering::Relaxed);
                unsafe {
                    CONNECTED_SSID.clear();
                    let _ = CONNECTED_SSID.push_str(best_ssid.as_str());
                }
                info!("WiFi: connected to '{}'", best_ssid.as_str());
                // Stay connected. Re-scan if we lose the connection.
                loop {
                    embassy_time::Timer::after_secs(30).await;
                    // TODO: detect disconnection and break
                }
            }
            Err(_e) => {
                WIFI_STATE.store(STATE_DISCONNECTED, Ordering::Relaxed);
                info!("WiFi: join failed");
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
    tt_log!("WiFi: loading firmware");

    let pwr = Output::new(pin_23, Level::Low);
    let cs = Output::new(pin_25, Level::High);

    let Pio0Remainder { mut common, sm, irq0 } = pio0_rem;
    let dma_chan = dma::Channel::new(dma_ch, DmaIrqs);
    let spi = PioSpi::new(
        &mut common,
        sm,
        cyw43_pio::DEFAULT_CLOCK_DIVIDER,
        irq0,
        cs,
        pin_24,
        pin_29,
        dma_chan,
    );

    let state = CYW43_STATE.init(cyw43::State::new());
    let (net_device, mut control, runner) = cyw43::new(state, pwr, spi, FW, NVRAM).await;

    // Spawn runner BEFORE control.init — runner processes ioctl commands.
    spawner.spawn(cyw43_task(runner).unwrap());

    control.init(CLM).await;
    control
        .set_power_management(cyw43::PowerManagementMode::None)
        .await;

    WIFI_STATE.store(STATE_FW_LOADED, Ordering::Relaxed);
    info!("WiFi: CYW43 initialized");
    tt_log!("WiFi: ready");

    let net_config = embassy_net::Config::dhcpv4(Default::default());
    let seed = 0x4c6f63616c54616c;
    let resources = NET_RESOURCES.init(StackResources::new());
    let (stack, net_runner) = embassy_net::new(net_device, net_config, resources, seed);
    let stack_ref = NET_STACK.init(stack);
    unsafe { STACK_REF = Some(stack_ref) };

    spawner.spawn(net_task(net_runner).unwrap());
    spawner.spawn(wifi_task(control).unwrap());
}
