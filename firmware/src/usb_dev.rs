//! Composite USB device — two CDC-ACM interfaces on one RP2040 USB peripheral.
//!
//! The interface order is fixed:
//!
//!   - **Interface 0/1 (CDC ACM #1)** — TT_DATA, exposes the TashTalk byte
//!     protocol. Configured first so it usually shows up as the
//!     lower-numbered `/dev/ttyACM*`.
//!   - **Interface 2/3 (CDC ACM #2)** — TT_DEBUG, exposes human-readable
//!     logs and the debug shell.
//!
//! On hosts where the order is ambiguous, the iProduct/iInterface strings
//! disambiguate.

use embassy_rp::flash::{Blocking, Flash};
use embassy_rp::peripherals::{FLASH, USB};
use embassy_rp::usb::Driver;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::{Builder, Config};
use static_cell::StaticCell;

/// Type alias for the concrete CDC-ACM class we hand to consumer tasks.
pub type Cdc = CdcAcmClass<'static, Driver<'static, USB>>;

/// Type alias for the `embassy-usb` device future driven by `usb_run_task`.
pub type UsbDevice = embassy_usb::UsbDevice<'static, Driver<'static, USB>>;

/// Build the composite USB device. Returns the device future plus the two
/// CDC class handles (TT_DATA first, TT_DEBUG second).
///
/// All long-lived buffers are kept in `'static` storage via `StaticCell`,
/// which is the conventional embassy-usb pattern for keeping the borrow
/// checker happy across task boundaries.
pub fn build(driver: Driver<'static, USB>, flash: FLASH) -> (UsbDevice, Cdc, Cdc) {
    // Read the flash chip's unique ID to use as USB serial number.
    // This gives each Pico a distinct identity for udev symlinks.
    static SERIAL_BUF: StaticCell<[u8; 17]> = StaticCell::new();
    let serial: &'static str = {
        let buf = SERIAL_BUF.init([0u8; 17]);
        let mut uid = [0u8; 8];
        let mut fl = Flash::<_, Blocking, { 2 * 1024 * 1024 }>::new_blocking(flash);
        let _ = fl.blocking_unique_id(&mut uid);
        // Format as 16-char hex string.
        const HEX: &[u8; 16] = b"0123456789ABCDEF";
        for (i, &b) in uid.iter().enumerate() {
            buf[i * 2] = HEX[(b >> 4) as usize];
            buf[i * 2 + 1] = HEX[(b & 0xF) as usize];
        }
        core::str::from_utf8(&buf[..16]).unwrap()
    };

    // USB descriptor strings. The VID/PID pair below is pid.codes test
    // space — replace with a real assignment before shipping. The
    // interface-association descriptor (IAD) class triplet
    // (0xEF, 0x02, 0x01) is required for composite devices that contain
    // multiple CDC interfaces on Windows.
    let mut config = Config::new(0x16c0, 0x27dd);
    config.manufacturer = Some("FeralFirmware");
    config.product = Some("TashTalk-Pico");
    config.serial_number = Some(serial);
    config.max_power = 100;
    config.max_packet_size_0 = 64;
    config.device_class = 0xEF;
    config.device_sub_class = 0x02;
    config.device_protocol = 0x01;
    config.composite_with_iads = true;

    static CONFIG_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static BOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static MSOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();
    static STATE_DATA: StaticCell<State> = StaticCell::new();
    static STATE_DEBUG: StaticCell<State> = StaticCell::new();

    let mut builder = Builder::new(
        driver,
        config,
        CONFIG_DESC.init([0; 256]),
        BOS_DESC.init([0; 256]),
        MSOS_DESC.init([0; 256]),
        CONTROL_BUF.init([0; 64]),
    );

    let class_data = CdcAcmClass::new(&mut builder, STATE_DATA.init(State::new()), 64);
    let class_debug = CdcAcmClass::new(&mut builder, STATE_DEBUG.init(State::new()), 64);

    let usb = builder.build();
    (usb, class_data, class_debug)
}
