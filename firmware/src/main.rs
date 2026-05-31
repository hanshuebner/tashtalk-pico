//! TashTalk-Pico firmware entry point.
//!
//! This binary brings up an RP2040 as a USB-CDC LocalTalk interface. It
//! enumerates as a **composite USB device** with two CDC-ACM interfaces:
//!
//! 1. **TT_DATA** — carries the TashTalk byte protocol (commands and
//!    escape-encoded received frames). Existing host drivers — TailTalk's
//!    `tashtalk` crate, `tashtalkd`, TashRouter, MultiTalk — open this port
//!    as if it were a TashTalk on a CP210x UART.
//! 2. **TT_DEBUG** — carries a human-readable log and an interactive debug
//!    shell. Used during bring-up because we have no logic analyzer in
//!    the loop and no existing TashTalk board to compare against.
//!
//! On Linux the two ports appear as `/dev/ttyACM0` and `/dev/ttyACM1`
//! (numbering depends on enumeration order — order is documented in the
//! USB descriptors so a host-side udev rule can disambiguate by interface
//! string).
//!
//! ## Pin assignments
//!
//! - **GP2** — TX data (PIO0 SM0 OUT pin → MAX-485 DI)
//! - **GP3** — TX enable / driver enable (PIO0 SM1 SET pin → MAX-485 DE)
//! - **GP4** — RX data (PIO1 SM0 JMP PIN ← MAX-485 RO)
//! - **GP5** — Carrier sense (PIO1 SM0 SET pin → PIO0 SM1 JMP PIN, internal)

#![no_std]
#![no_main]

use defmt::{info, unwrap};
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver, InterruptHandler};
use defmt_rtt as _;


/// No-init RAM buffer that the panic handler writes to before rebooting
/// into BOOTSEL. Survives the reset (RP2040 SRAM is volatile only to
/// power cycles, not soft resets). After a panic, dump it with
/// `picotool save -r <addr> 0x200 panic.bin`.
#[link_section = ".uninit.PANIC_BUF"]
static mut PANIC_BUF: core::mem::MaybeUninit<[u8; 512]> =
    core::mem::MaybeUninit::uninit();

const PANIC_MARKER: &[u8] = b"\xDEPANIC_BUF\xDE ";

/// Custom panic handler — writes a recognizable marker + file:line:col
/// + (truncated) panic message into PANIC_BUF, then reboots to BOOTSEL.
#[panic_handler]
fn panic_handler(info: &core::panic::PanicInfo) -> ! {
    use core::fmt::Write;
    // SAFETY: panics are single-threaded (other tasks frozen); we own
    // this buffer exclusively for the remainder of this diverging function.
    let buf = unsafe { &mut *core::ptr::addr_of_mut!(PANIC_BUF).cast::<[u8; 512]>() };
    for b in buf.iter_mut() {
        *b = 0;
    }
    buf[..PANIC_MARKER.len()].copy_from_slice(PANIC_MARKER);
    let mut w = SliceWriter {
        buf,
        pos: PANIC_MARKER.len(),
    };
    if let Some(loc) = info.location() {
        let _ = write!(&mut w, "{}:{}:{} ", loc.file(), loc.line(), loc.column());
    } else {
        let _ = write!(&mut w, "<no location> ");
    }
    let _ = write!(&mut w, "msg=");
    let _ = write!(&mut w, "{}", info);

    embassy_rp::rom_data::reset_to_usb_boot(0, 0);
    loop {
        cortex_m::asm::nop();
    }
}

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

mod localtalk;
mod host;
mod debug;
#[macro_use]
mod tt_log;
mod usb_dev;

use tashtalk_proto::backoff::BackoffState;
use tashtalk_proto::nodemap::NodeBitmap;

use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use static_cell::StaticCell;

pub type SharedNodes = Mutex<ThreadModeRawMutex, NodeBitmap>;
pub type SharedBackoff = Mutex<ThreadModeRawMutex, BackoffState>;
pub type SharedTxData = Mutex<ThreadModeRawMutex, localtalk::phy::LtTxData<'static>>;
pub type SharedTxEnable = Mutex<ThreadModeRawMutex, localtalk::phy::LtTxEnable<'static>>;
pub type SharedRx = Mutex<ThreadModeRawMutex, localtalk::phy::LtRx<'static>>;

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    info!("tashtalk-pico booting");

    // ── LocalTalk PHY ────────────────────────────────────────────────
    // GP2 = TX data, GP3 = TX enable (DE), GP4 = RX data, GP5 = carrier sense
    info!("PHY: init_phy starting");
    let (tx_data, tx_enable, rx) = localtalk::phy::init_phy(
        p.PIO0,
        localtalk::phy::Pio0Irqs {},
        p.PIO1,
        localtalk::phy::Pio1Irqs {},
        p.PIN_2,  // TX data
        p.PIN_3,  // TX enable / DE
        p.PIN_4,  // RX data
        p.PIN_5,  // carrier sense (internal)
    );

    info!("PHY: init_phy done");

    static TX_DATA: StaticCell<SharedTxData> = StaticCell::new();
    let tx_data: &'static SharedTxData = TX_DATA.init(Mutex::new(tx_data));

    static TX_ENABLE: StaticCell<SharedTxEnable> = StaticCell::new();
    let tx_enable: &'static SharedTxEnable = TX_ENABLE.init(Mutex::new(tx_enable));

    static RX: StaticCell<SharedRx> = StaticCell::new();
    let rx: &'static SharedRx = RX.init(Mutex::new(rx));

    // Shared node-ID bitmap. Empty at boot.
    static NODES: StaticCell<SharedNodes> = StaticCell::new();
    let nodes: &'static SharedNodes = NODES.init(Mutex::new(NodeBitmap::empty()));

    // CSMA/CA backoff state.
    static BACKOFF: StaticCell<SharedBackoff> = StaticCell::new();
    let backoff: &'static SharedBackoff =
        BACKOFF.init(Mutex::new(BackoffState::new()));

    // ── USB ──────────────────────────────────────────────────────────
    let driver = Driver::new(p.USB, Irqs);
    let (usb_run, data_cdc, debug_cdc) = usb_dev::build(driver, p.FLASH);

    unwrap!(spawner.spawn(usb_run_task(usb_run)));

    // Debug CDC: banner, echo, log lines, debug shell.
    unwrap!(spawner.spawn(debug::run(debug_cdc, tx_data, tx_enable, backoff)));

    // Data CDC: host command parser + LocalTalk→host frame forwarding.
    unwrap!(spawner.spawn(host::run(data_cdc, tx_data, tx_enable, nodes, backoff)));

    // Continuous LocalTalk receiver + autoresponder.
    unwrap!(spawner.spawn(localtalk::rx::run(rx, tx_data, nodes)));
}

#[embassy_executor::task]
async fn usb_run_task(mut usb: usb_dev::UsbDevice) {
    usb.run().await;
}
