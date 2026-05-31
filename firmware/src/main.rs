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
/// The next firmware boot can read PANIC_BUF and report the last panic
/// (or a host can dump SRAM with picotool and grep for `PANIC_MARKER`).
#[panic_handler]
fn panic_handler(info: &core::panic::PanicInfo) -> ! {
    use core::fmt::Write;
    // SAFETY: panics are single-threaded (other tasks frozen on panic in
    // cooperative async); we own this buffer for the duration.
    let buf = unsafe {
        let ptr = PANIC_BUF.as_mut_ptr() as *mut [u8; 512];
        &mut *ptr
    };
    // Clear, then write the marker + location.
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
    // PanicInfo::message() is unstable, but we can format with %s of info
    // itself which yields "panicked at ..." text.
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

mod bitstream;
mod csma;
mod debug;
mod dialog_tx;
mod host;
mod lt;
mod lt_phy;
mod lt_rx_task;
mod pio_rx;
mod pio_tx;
#[macro_use]
mod tt_log;
mod usb_dev;

use tashtalk_proto::backoff::BackoffState;
use tashtalk_proto::nodemap::NodeBitmap;

pub use csma::SharedBackoff;
pub type SharedNodes = Mutex<ThreadModeRawMutex, NodeBitmap>;

use embassy_rp::gpio::{Level, Output};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use static_cell::StaticCell;

pub type SharedLtTx = Mutex<ThreadModeRawMutex, lt_phy::LtTx<'static>>;
pub type MaybeLtTx = Mutex<ThreadModeRawMutex, Option<lt_phy::LtTx<'static>>>;
pub type MaybeLtRx = Mutex<ThreadModeRawMutex, Option<lt_phy::LtRx<'static>>>;
pub type MaybeLtDialogTx = Mutex<ThreadModeRawMutex, Option<lt_phy::LtDialogTx<'static>>>;
pub type SharedDe = Mutex<ThreadModeRawMutex, Output<'static>>;

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    info!("tashtalk-pico booting");

    // ── LocalTalk PHY (PIO TX + RX) ──────────────────────────────────
    // GP2 = LT_DATA TX (PIO0 SM0). GP3 = LT_DE (M0+ GPIO).
    // GP4 = LT_DATA RX (PIO0 SM1) — for self-loopback, jumper GP2 to GP4.
    let (lt_tx, lt_rx, lt_dialog_tx) = lt_phy::init_phy(p.PIO0, lt_phy::PioIrqs {}, p.PIN_2, p.PIN_4);
    let de = Output::new(p.PIN_3, Level::Low);

    static LT_DE: StaticCell<SharedDe> = StaticCell::new();
    let lt_de: &'static SharedDe = LT_DE.init(Mutex::new(de));
    static LT_TX: StaticCell<MaybeLtTx> = StaticCell::new();
    let lt_tx: &'static MaybeLtTx = LT_TX.init(Mutex::new(Some(lt_tx)));
    static LT_RX: StaticCell<MaybeLtRx> = StaticCell::new();
    let lt_rx: &'static MaybeLtRx = LT_RX.init(Mutex::new(Some(lt_rx)));
    static LT_DIALOG_TX: StaticCell<MaybeLtDialogTx> = StaticCell::new();
    let lt_dialog_tx: &'static MaybeLtDialogTx = LT_DIALOG_TX.init(Mutex::new(Some(lt_dialog_tx)));

    // Shared node-ID bitmap. Empty at boot: the autoresponder defends
    // nothing until a host claims node IDs via the 0x02 SetNodeIds
    // opcode. (A non-empty default would make the Pico squat on a node
    // and answer ENQs for it, corrupting a host stack's own LocalTalk
    // address acquisition.)
    let initial_nodes = NodeBitmap::empty();
    static NODES: StaticCell<SharedNodes> = StaticCell::new();
    let nodes: &'static SharedNodes = NODES.init(Mutex::new(initial_nodes));

    // CSMA/CA backoff state, used by transmit_on_phy before each frame.
    static BACKOFF: StaticCell<SharedBackoff> = StaticCell::new();
    let backoff: &'static SharedBackoff =
        BACKOFF.init(Mutex::new(BackoffState::new()));

    // ── USB ───────────────────────────────────────────────────────────
    let driver = Driver::new(p.USB, Irqs);
    let (usb_run, data_cdc, debug_cdc) = usb_dev::build(driver);

    // USB event pump — never returns.
    unwrap!(spawner.spawn(usb_run_task(usb_run)));

    // Debug CDC: banner, echo, log lines, command shell.
    unwrap!(spawner.spawn(debug::run(debug_cdc, lt_tx, lt_dialog_tx, lt_rx, lt_de, backoff)));

    // Data CDC: host command parser + LocalTalk→host frame forwarding.
    unwrap!(spawner.spawn(host::run(data_cdc, lt_tx, lt_dialog_tx, lt_de, nodes, backoff)));

    // Continuous LocalTalk receiver + autoresponder. Small ping-pong DMA
    // buffers keep wire→CTS latency inside LLAP's response window; the
    // autonomous ACK/CTS is transmitted inline on this task.
    unwrap!(spawner.spawn(lt_rx_task::run(
        lt_rx,
        lt_tx,
        lt_de,
        nodes,
        embassy_rp::Peripheral::into_ref(p.DMA_CH0),
    )));
}

#[embassy_executor::task]
async fn usb_run_task(mut usb: usb_dev::UsbDevice) {
    usb.run().await;
}
