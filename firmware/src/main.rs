//! TashTalk-Pico firmware entry point.
//!
//! Dual-core architecture:
//! - **Core 0** (Embassy async): USB CDC, host protocol, debug shell.
//! - **Core 1** (bare-metal loop): PIO, DMA, all LocalTalk wire interaction.
//!
//! ## Pin assignments
//!
//! - **GP2** — TX data (PIO0 SM0 OUT pin → MAX-485 DI)
//! - **GP3** — TX enable / driver enable (software GPIO → MAX-485 DE)
//! - **GP4** — RX data (PIO1 SM0 JMP PIN ← MAX-485 RO)
//! - **GP5** — Carrier sense (PIO1 SM0 SET pin, internal)
//! - **GP6** — Debug output 0 (core 1 timing, AD2 probe)

#![no_std]
#![no_main]
#![allow(static_mut_refs)] // embassy multicore API requires &mut Stack

use core::sync::atomic::{compiler_fence, Ordering};

use defmt::info;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::multicore::Stack;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver, InterruptHandler};
use embassy_time::Timer;
use defmt_rtt as _;
use static_cell::StaticCell;

use tashtalk_proto::backoff::BackoffState;
use tashtalk_proto::codec::FrameStatus;
use tashtalk_proto::nodemap::NodeBitmap;

use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;

mod core1;
mod localtalk;
mod host;
mod debug;
#[macro_use]
mod tt_log;
mod usb_dev;
mod wifi;
mod bridge;
mod config;

pub type SharedNodes = Mutex<ThreadModeRawMutex, NodeBitmap>;
pub type SharedBackoff = Mutex<ThreadModeRawMutex, BackoffState>;

/// No-init RAM buffer for panic messages (survives soft reset).
#[link_section = ".uninit.PANIC_BUF"]
static mut PANIC_BUF: [u8; 256] = [0u8; 256];

struct SliceWriter<'a> {
    buf: &'a mut [u8],
    pos: usize,
}

impl core::fmt::Write for SliceWriter<'_> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        for b in s.bytes() {
            if self.pos < self.buf.len() - 1 {
                self.buf[self.pos] = b;
                self.pos += 1;
            }
        }
        Ok(())
    }
}

#[panic_handler]
fn panic_handler(info: &core::panic::PanicInfo) -> ! {
    use core::fmt::Write;
    unsafe {
        let buf = &mut *core::ptr::addr_of_mut!(PANIC_BUF);
        let mut w = SliceWriter { buf, pos: 0 };
        let _ = write!(w, "{}", info);
    }
    embassy_rp::rom_data::reset_to_usb_boot(0, 0);
    loop {}
}

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

// ── Core 1 stack ────────────────────────────────────────────────────
const CORE1_STACK_SIZE: usize = 8192;
static mut CORE1_STACK: Stack<CORE1_STACK_SIZE> = Stack::new();

/// Sentinel word used to paint the core-1 stack for high-water measurement.
const CORE1_STACK_PAINT: u32 = 0xC1AC_57AC;

fn paint_core1_stack() {
    let words = CORE1_STACK_SIZE / 4;
    let base = core::ptr::addr_of_mut!(CORE1_STACK) as *mut u32;
    for i in 0..words {
        unsafe { base.add(i).write_volatile(CORE1_STACK_PAINT) };
    }
}

pub fn core1_stack_stats() -> (usize, usize) {
    let words = CORE1_STACK_SIZE / 4;
    let base = core::ptr::addr_of!(CORE1_STACK) as *const u32;
    let mut unused = 0;
    while unused < words && unsafe { base.add(unused).read_volatile() } == CORE1_STACK_PAINT {
        unused += 1;
    }
    ((words - unused) * 4, words * 4)
}

// ── Shared state (inter-core) ───────────────────────────────────────
static SHARED: core1::SharedState = core1::SharedState::new();

// ── Main ────────────────────────────────────────────────────────────

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    info!("tashtalk-pico booting (dual-core)");

    // ── LocalTalk PHY (configures PIO + pins) ────────────────────────
    info!("PHY: init_phy starting");
    let pio0_remainder = localtalk::phy::init_phy(
        p.PIO0,
        localtalk::phy::Pio0Irqs {},
        p.PIO1,
        localtalk::phy::Pio1Irqs {},
        p.PIN_2,
        p.PIN_3,
        p.PIN_4,
        p.PIN_5,
    );
    info!("PHY: init_phy done");

    // ── Spawn core 1 (before any async work) ────────────────────────
    paint_core1_stack();
    embassy_rp::multicore::spawn_core1(
        p.CORE1,
        unsafe { &mut CORE1_STACK },
        move || core1::core1_main(&SHARED),
    );
    info!("Core 1 spawned");

    // ── Flash config ──────────────────────────────────────────────────
    {
        use embassy_rp::flash::{Blocking, Flash};
        let mut fl = Flash::<_, Blocking, { 2 * 1024 * 1024 }>::new_blocking(
            unsafe { p.FLASH.clone_unchecked() }
        );
        let mut n = 0u32;
        config::load(&mut fl, |ssid, pass| {
            wifi::add_network(ssid, pass);
            n += 1;
        });
        if n > 0 {
            info!("loaded {} WiFi network(s) from flash", n);
        }
        config::init(fl);
    }

    // ── USB ──────────────────────────────────────────────────────────
    let driver = Driver::new(p.USB, Irqs);
    let (usb_run, data_cdc, debug_cdc) = usb_dev::build(driver, p.FLASH);

    // ── Spawn non-WiFi tasks ────────────────────────────────────────
    spawner.spawn(usb_run_task(usb_run).unwrap());
    spawner.spawn(debug::run(debug_cdc, &SHARED).unwrap());
    spawner.spawn(host::run(data_cdc, &SHARED).unwrap());
    spawner.spawn(rx_bridge_task(&SHARED).unwrap());
    spawner.spawn(bridge::bridge_task(&SHARED).unwrap());

    // ── WiFi init (async — runs on this task) ───────────────────────
    wifi::init(
        spawner,
        pio0_remainder,
        p.PIN_23,
        p.PIN_25,
        p.PIN_24,
        p.PIN_29,
        p.DMA_CH2,
    )
    .await;
}

#[embassy_executor::task]
async fn usb_run_task(mut usb: usb_dev::UsbDevice) {
    usb.run().await;
}

/// Bridge: polls core 1's rx_slot and forwards frames to the host task.
#[embassy_executor::task]
async fn rx_bridge_task(shared: &'static core1::SharedState) {
    loop {
        let flags = shared.flags.load(Ordering::Relaxed);
        if flags & core1::RX_READY != 0 {
            compiler_fence(Ordering::SeqCst);
            // SAFETY: RX_READY set → core 1 is not writing.
            let slot = unsafe { &*shared.rx_slot.get() };
            let len = slot.len as usize;
            let status = match slot.status {
                1 => FrameStatus::BadCrc,
                _ => FrameStatus::Good,
            };
            host::try_forward_inbound(&slot.buf[..len], status);
            let crc_ok = matches!(status, FrameStatus::Good);
            debug::log_rx_frame(&slot.buf[..len], crc_ok);
            bridge::try_forward_to_udp(&slot.buf[..len], crc_ok);
            shared.flags.fetch_and(!core1::RX_READY, Ordering::Release);
        } else {
            Timer::after_micros(50).await;
        }
    }
}
