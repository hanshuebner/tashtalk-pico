//! Flash-backed configuration storage.
//!
//! Stores WiFi credentials in the last 4KB sector of flash (offset 0x1FF000).
//! Format: [magic:u32][version:u8][count:u8][padding:u16][entries...][crc:u32]
//! Each entry: [ssid_len:u8][ssid:32 bytes][pass_len:u8][pass:63 bytes] = 96 bytes
//! Total: 8 + 4*96 + 4 = 396 bytes (fits in one 256-byte page? No — needs 2 pages)

use core::cell::RefCell;

use embassy_rp::flash::{Blocking, Flash, ERASE_SIZE};
use embassy_rp::peripherals::FLASH;
use static_cell::StaticCell;

type FlashType = Flash<'static, FLASH, Blocking, FLASH_SIZE>;

static FLASH_CELL: StaticCell<RefCell<FlashType>> = StaticCell::new();
static mut FLASH_REF: Option<&'static RefCell<FlashType>> = None;

/// Store the flash handle for later use by save().
pub fn init(flash: FlashType) {
    let cell = FLASH_CELL.init(RefCell::new(flash));
    unsafe { FLASH_REF = Some(cell) };
}

/// Save WiFi config using the stored flash handle.
pub fn save_config(ssids: &[(&str, &str)]) {
    if let Some(cell) = unsafe { FLASH_REF } {
        let mut fl = cell.borrow_mut();
        save(ssids, &mut fl);
    }
}

const FLASH_SIZE: usize = 2 * 1024 * 1024;
/// Config lives in the last 4KB sector.
const CONFIG_OFFSET: u32 = (FLASH_SIZE - ERASE_SIZE) as u32;
const MAGIC: u32 = 0x5441_5348; // "TASH"
const VERSION: u8 = 1;

const MAX_SSID: usize = 32;
const MAX_PASS: usize = 63;
const MAX_ENTRIES: usize = 4;
/// One entry: 1 (ssid_len) + 32 (ssid) + 1 (pass_len) + 63 (pass) = 97 bytes.
/// Round to 100 for alignment.
const ENTRY_SIZE: usize = 100;
/// Header: magic(4) + version(1) + count(1) + pad(2) = 8
const HEADER_SIZE: usize = 8;
/// Total: header + entries + crc
const CONFIG_SIZE: usize = HEADER_SIZE + MAX_ENTRIES * ENTRY_SIZE + 4;

fn crc32(data: &[u8]) -> u32 {
    let mut crc: u32 = 0xFFFF_FFFF;
    for &b in data {
        crc ^= b as u32;
        for _ in 0..8 {
            if crc & 1 != 0 {
                crc = (crc >> 1) ^ 0xEDB8_8320;
            } else {
                crc >>= 1;
            }
        }
    }
    !crc
}

/// Read WiFi config from flash. Calls `callback` for each (ssid, pass) pair.
pub fn load(flash: &mut Flash<'_, FLASH, Blocking, FLASH_SIZE>, mut callback: impl FnMut(&str, &str)) {
    let mut buf = [0u8; CONFIG_SIZE];

    if flash.blocking_read(CONFIG_OFFSET, &mut buf).is_err() {
        return;
    }

    // Check magic.
    let magic = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
    if magic != MAGIC {
        return;
    }
    if buf[4] != VERSION {
        return;
    }
    let count = buf[5] as usize;
    if count > MAX_ENTRIES {
        return;
    }

    // Verify CRC over header + entries.
    let data_len = HEADER_SIZE + MAX_ENTRIES * ENTRY_SIZE;
    let stored_crc = u32::from_le_bytes([
        buf[data_len],
        buf[data_len + 1],
        buf[data_len + 2],
        buf[data_len + 3],
    ]);
    if crc32(&buf[..data_len]) != stored_crc {
        return;
    }

    // Parse entries.
    for i in 0..count {
        let base = HEADER_SIZE + i * ENTRY_SIZE;
        let ssid_len = (buf[base] as usize).min(MAX_SSID);
        let ssid = &buf[base + 1..base + 1 + ssid_len];
        let pass_len = (buf[base + 1 + MAX_SSID] as usize).min(MAX_PASS);
        let pass = &buf[base + 1 + MAX_SSID + 1..base + 1 + MAX_SSID + 1 + pass_len];

        if let (Ok(s), Ok(p)) = (core::str::from_utf8(ssid), core::str::from_utf8(pass)) {
            callback(s, p);
        }
    }
}

/// Save WiFi config to flash. Erases the config sector and writes.
/// The flash erase (~100ms) briefly pauses XIP; USB may see a glitch.
pub fn save(ssids: &[(&str, &str)], flash: &mut Flash<'_, FLASH, Blocking, FLASH_SIZE>) {
    let count = ssids.len().min(MAX_ENTRIES);

    // Buffer rounded up to page boundary (256 bytes).
    const BUF_SIZE: usize = (CONFIG_SIZE + 255) & !255;
    let mut buf = [0xFFu8; BUF_SIZE];

    // Header.
    buf[0..4].copy_from_slice(&MAGIC.to_le_bytes());
    buf[4] = VERSION;
    buf[5] = count as u8;
    buf[6] = 0; // padding
    buf[7] = 0;

    // Entries.
    for (i, (ssid, pass)) in ssids.iter().take(MAX_ENTRIES).enumerate() {
        let base = HEADER_SIZE + i * ENTRY_SIZE;
        let ssid_bytes = ssid.as_bytes();
        let pass_bytes = pass.as_bytes();
        buf[base] = ssid_bytes.len().min(MAX_SSID) as u8;
        buf[base + 1..base + 1 + ssid_bytes.len().min(MAX_SSID)]
            .copy_from_slice(&ssid_bytes[..ssid_bytes.len().min(MAX_SSID)]);
        buf[base + 1 + MAX_SSID] = pass_bytes.len().min(MAX_PASS) as u8;
        buf[base + 1 + MAX_SSID + 1..base + 1 + MAX_SSID + 1 + pass_bytes.len().min(MAX_PASS)]
            .copy_from_slice(&pass_bytes[..pass_bytes.len().min(MAX_PASS)]);
    }

    // CRC over header + entries.
    let data_len = HEADER_SIZE + MAX_ENTRIES * ENTRY_SIZE;
    let crc = crc32(&buf[..data_len]);
    buf[data_len..data_len + 4].copy_from_slice(&crc.to_le_bytes());

    let _ = flash.blocking_erase(CONFIG_OFFSET, CONFIG_OFFSET + ERASE_SIZE as u32);
    let _ = flash.blocking_write(CONFIG_OFFSET, &buf);
}
