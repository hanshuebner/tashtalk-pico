//! TashTalk protocol primitives — `no_std`, host-testable.
//!
//! This crate contains every piece of TashTalk's byte-level logic that does
//! not need hardware to validate:
//!
//! - [`crc`] — the LocalTalk CRC-16 (SDLC/CCITT) used at the frame level.
//! - [`codec`] — the host ↔ device byte protocol described in
//!   `tashtalk/documentation/protocol.md` (commands `0x00`–`0x03`, escape
//!   sequences `0x00 0xFD/FC/FE/FA/FF`).
//! - [`nodemap`] — the 256-bit node-ID bitmap that the `0x02 SetNodeIds`
//!   command writes.
//! - [`bitstuff`] — SDLC bit-stuffing (insert a `0` after five consecutive
//!   `1`s on transmit; drop the matching `0` on receive) plus flag and
//!   abort detection.
//!
//! Everything here runs on the host as well as on the Pico, and has unit
//! tests that do not require any embedded hardware.

#![cfg_attr(not(test), no_std)]
#![deny(missing_debug_implementations)]

pub mod backoff;
pub mod bitstuff;
pub mod codec;
pub mod crc;
pub mod fm0;
pub mod llap;
pub mod nodemap;
