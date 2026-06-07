//! Flash-backed configuration storage.
//!
//! Stores WiFi credentials in the last 4KB sector of flash (offset 0x1FF000).
//! Reads via XIP (memory-mapped flash). Writes via embassy-rp Flash HAL.

use embassy_rp::flash::{Blocking, Flash, ERASE_SIZE};
use embassy_rp::peripherals::FLASH;

const FLASH_SIZE: usize = 2 * 1024 * 1024;
/// Config lives in the last 4KB sector.
const CONFIG_OFFSET: u32 = (FLASH_SIZE - ERASE_SIZE) as u32;
/// XIP base address for reading flash as memory.
const XIP_BASE: usize = 0x10000000;
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
/// Buffer rounded up to flash page size (256 bytes).
const BUF_SIZE: usize = (CONFIG_SIZE + 255) & !255;

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

/// Read WiFi config from flash via XIP. No Flash handle needed.
pub fn load(mut callback: impl FnMut(&str, &str)) {
    // Read directly from memory-mapped flash.
    let ptr = (XIP_BASE + CONFIG_OFFSET as usize) as *const u8;
    let flash_data = unsafe { core::slice::from_raw_parts(ptr, CONFIG_SIZE) };

    // Copy to a stack buffer to avoid alignment issues.
    let mut buf = [0u8; CONFIG_SIZE];
    buf.copy_from_slice(flash_data);

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

/// Save WiFi config to flash using a Flash handle.
pub fn save(ssids: &[(&str, &str)], flash: &mut Flash<'_, FLASH, Blocking, FLASH_SIZE>) {
    let count = ssids.len().min(MAX_ENTRIES);

    let mut buf = [0xFFu8; BUF_SIZE];

    // Header.
    buf[0..4].copy_from_slice(&MAGIC.to_le_bytes());
    buf[4] = VERSION;
    buf[5] = count as u8;

    // Entries.
    for (i, (ssid, pass)) in ssids.iter().take(MAX_ENTRIES).enumerate() {
        let base = HEADER_SIZE + i * ENTRY_SIZE;
        let ssid_bytes = ssid.as_bytes();
        let pass_bytes = pass.as_bytes();
        let slen = ssid_bytes.len().min(MAX_SSID);
        let plen = pass_bytes.len().min(MAX_PASS);
        buf[base] = slen as u8;
        buf[base + 1..base + 1 + slen].copy_from_slice(&ssid_bytes[..slen]);
        buf[base + 1 + MAX_SSID] = plen as u8;
        buf[base + 1 + MAX_SSID + 1..base + 1 + MAX_SSID + 1 + plen]
            .copy_from_slice(&pass_bytes[..plen]);
    }

    // CRC over header + entries.
    let data_len = HEADER_SIZE + MAX_ENTRIES * ENTRY_SIZE;
    let crc = crc32(&buf[..data_len]);
    buf[data_len..data_len + 4].copy_from_slice(&crc.to_le_bytes());

    if let Err(_e) = flash.blocking_erase(CONFIG_OFFSET, CONFIG_OFFSET + ERASE_SIZE as u32) {
        defmt::warn!("config: flash erase failed");
        return;
    }
    if let Err(_e) = flash.blocking_write(CONFIG_OFFSET, &buf) {
        defmt::warn!("config: flash write failed");
    }
}
