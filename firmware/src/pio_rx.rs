//! PIO-based FM-0 receiver for LocalTalk — **stub**.
//!
//! Replaces the PIC firmware's interrupt-driven receiver (`LtReceiver` and
//! the three FSA tables `OutFSA` / `EmpFSA` / `InFSA` starting at
//! `org 0xA00` in `one-chip.asm`). The PIC's design polls
//! `IOCAF5` across the bit cell to recover clock; PIO will instead
//! oversample at 8× the bit rate and detect transitions with `jmp pin`.
//!
//! ## PIO program intent
//!
//! Free-runs at 8× the 230.4 kHz bit clock. Maintains a `Y` countdown for
//! "cells since last transition"; transitions reset `Y`. A transition at
//! mid-cell decodes as a `0`, a transition only at cell boundary decodes
//! as a `1`. Once 8 payload bits have accumulated (or a 6-ones run is
//! detected), the program pushes a 16-bit word to the RX FIFO whose layout
//! is:
//!
//!   bits 0-7  : payload byte (or 0 for events)
//!   bits 8-15 : status — 0 = data byte, 1 = flag, 2 = abort, 3 = lost clock
//!
//! Bit-unstuffing happens in the PIO loop's "5 ones then a zero → drop the
//! zero" branch; flag and abort detection happen at "6 ones then 0 or 1".
//! These three behaviours mirror what `tashtalk_proto::bitstuff::BitUnstuffer`
//! does on the host for unit tests, so the PIO program can be cross-checked
//! against that pure logic during bring-up.
//!
//! ## Bring-up gate
//!
//! Loopback through PIO TX, validated by round-tripping arbitrary 605-byte
//! payloads through the encoder → wire → decoder → host comparison. The
//! TT_DEBUG CDC will be used for the comparison output.

pub struct PioRx {
    _private: (),
}

impl PioRx {
    pub const fn new() -> Self {
        Self { _private: () }
    }
}

impl Default for PioRx {
    fn default() -> Self {
        Self::new()
    }
}
