//! Composite USB device — two CDC-ACM interfaces on one RP2040 USB peripheral.

use embassy_rp::flash::{Blocking, Flash};
use embassy_rp::peripherals::{FLASH, USB};
use embassy_rp::usb::Driver;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::{Builder, Config};
use static_cell::StaticCell;

pub type Cdc = CdcAcmClass<'static, Driver<'static, USB>>;
pub type UsbDevice = embassy_usb::UsbDevice<'static, Driver<'static, USB>>;

static mut SERIAL_PTR: Option<&'static str> = None;

pub fn serial() -> &'static str {
    unsafe { SERIAL_PTR.unwrap_or("") }
}

/// Read the flash unique ID and format as a serial number string.
pub fn read_serial(flash: &mut Flash<'_, FLASH, Blocking, { 2 * 1024 * 1024 }>) -> &'static str {
    static SERIAL_BUF: StaticCell<[u8; 17]> = StaticCell::new();
    let buf = SERIAL_BUF.init([0u8; 17]);
    let mut uid = [0u8; 8];
    let _ = flash.blocking_unique_id(&mut uid);
    const HEX: &[u8; 16] = b"0123456789ABCDEF";
    for (i, &b) in uid.iter().enumerate() {
        buf[i * 2] = HEX[(b >> 4) as usize];
        buf[i * 2 + 1] = HEX[(b & 0xF) as usize];
    }
    let s = core::str::from_utf8(&buf[..16]).unwrap();
    unsafe { SERIAL_PTR = Some(s); }
    s
}

/// Build USB device with a pre-read serial number.
pub fn build_with_serial(driver: Driver<'static, USB>, serial: &'static str) -> (UsbDevice, Cdc, Cdc) {
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
