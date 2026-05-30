//! Build the wire-level bit stream for a LocalTalk frame.
//!
//! The output is what eventually shows up on the FM-0–encoded GPIO, in
//! transmission order, packed LSB-first into bytes (and later into u32
//! words for the PIO TX FIFO).
//!
//! Frame layout, in order on the wire:
//!
//! 1. `0x00` sync pulse (one bit time of low — see `SendSyncPreamble`
//!    in `one-chip.asm`). Currently omitted in this firmware iteration;
//!    will be added when bus contention / CSMA-CA lands.
//! 2. **Three flag bytes** `0x7E 0x7E 0x7E`. Not bit-stuffed (flags are
//!    framing, not payload). Three matches what the PIC firmware does
//!    (`other_info.md` notes this is a IIgs-compat requirement).
//! 3. **Payload** — LLAP header + body + 2 CRC bytes. Bit-stuffed.
//! 4. **One flag byte** `0x7E` postamble.
//! 5. **Final '0' bit + idle** — see `SendPoZ` in `one-chip.asm`. Drives
//!    the line low before tristating so receivers don't see a `1→Z→1`
//!    glitch. Implemented in firmware/src/lt_phy.rs (after the bit stream
//!    is exhausted, the PIO program leaves the line low and the driver
//!    enable is dropped).

use heapless::Vec;
use tashtalk_proto::codec::MAX_FRAME_LEN;

/// Maximum bit-stuffed bit count for one frame:
/// - 3 leading flags × 8 = 24
/// - payload: MAX_FRAME_LEN × 8 with up to one stuffed zero per 5 ones
///   ≈ 1.2× expansion in worst case → round up generously
/// - 1 trailing flag × 8 = 8
const MAX_BITS: usize = 24 + MAX_FRAME_LEN * 10 + 8;

/// Bit-stream byte length (LSB-first packed). Round up.
const MAX_BYTES: usize = (MAX_BITS + 7) / 8;

pub struct BitStream {
    /// Packed bits, LSB of byte 0 = first bit on the wire.
    bytes: Vec<u8, MAX_BYTES>,
    /// Number of valid bits (≤ bytes.len() * 8).
    bit_count: u32,
    /// Tracks consecutive `1` bits — only meaningful while in the
    /// payload region (flag bytes are emitted via [`push_byte_raw`]
    /// which bypasses stuffing and resets the counter).
    ones_run: u8,
    cur_byte: u8,
    cur_bit_pos: u8,
}

impl BitStream {
    pub fn new() -> Self {
        Self {
            bytes: Vec::new(),
            bit_count: 0,
            ones_run: 0,
            cur_byte: 0,
            cur_bit_pos: 0,
        }
    }

    pub fn bit_len(&self) -> u32 {
        self.bit_count
    }

    /// Get a slice of the assembled bit stream as packed bytes (LSB-first).
    pub fn as_bytes(&self) -> &[u8] {
        &self.bytes
    }

    /// Append one bit, no stuffing.
    fn push_bit_raw(&mut self, bit: u8) {
        debug_assert!(bit <= 1);
        self.cur_byte |= bit << self.cur_bit_pos;
        self.cur_bit_pos += 1;
        self.bit_count += 1;
        if self.cur_bit_pos == 8 {
            // SAFETY of unwrap: capacity was sized for the worst case.
            self.bytes.push(self.cur_byte).unwrap();
            self.cur_byte = 0;
            self.cur_bit_pos = 0;
        }
    }

    /// Append a whole byte without bit-stuffing or ones-counter updates.
    /// Used for flag bytes (`0x7E`), which intentionally contain six
    /// consecutive 1 bits — they must reach the wire unmodified.
    pub fn push_byte_raw(&mut self, byte: u8) {
        for i in 0..8 {
            self.push_bit_raw((byte >> i) & 1);
        }
        // Flag bytes reset the ones counter at their trailing 0.
        self.ones_run = 0;
    }

    /// Append one payload bit, with SDLC bit-stuffing applied.
    fn push_bit_stuffed(&mut self, bit: u8) {
        self.push_bit_raw(bit);
        if bit == 1 {
            self.ones_run += 1;
            if self.ones_run == 5 {
                self.push_bit_raw(0);
                self.ones_run = 0;
            }
        } else {
            self.ones_run = 0;
        }
    }

    /// Append a whole payload byte, bit-stuffed.
    pub fn push_byte_stuffed(&mut self, byte: u8) {
        for i in 0..8 {
            self.push_bit_stuffed((byte >> i) & 1);
        }
    }

    /// Assemble a full LocalTalk frame as a bit stream:
    /// 3 flags + stuffed payload + 1 flag + postamble.
    ///
    /// The postamble matches the PIC reference firmware (`SendPostamble`
    /// in `one-chip.asm`): 13 raw `1` bits (SDLC abort marker) followed
    /// by a `0` bit (`SendPoZ`) so the line is driven low before the
    /// RS-485 driver is tristated. Without the abort marker, the padding
    /// zeros that would fill the last word look like the start of a new
    /// frame to the SCC, causing a spurious framing error that can
    /// prevent the peer from processing the frame just received.
    pub fn build_frame(payload: &[u8]) -> Self {
        let mut s = Self::new();
        for _ in 0..3 {
            s.push_byte_raw(0x7E);
        }
        for &b in payload {
            s.push_byte_stuffed(b);
        }
        s.push_byte_raw(0x7E);
        // Postamble: 13 ones (abort) + 1 zero (SendPoZ).
        for _ in 0..13 {
            s.push_bit_raw(1);
        }
        s.push_bit_raw(0);
        // Flush any partial byte to bytes[] so as_bytes() returns the
        // complete stream. The last byte may have unused high bits.
        if s.cur_bit_pos > 0 {
            s.bytes.push(s.cur_byte).unwrap();
            s.cur_byte = 0;
            s.cur_bit_pos = 0;
        }
        s
    }
}

impl Default for BitStream {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    // Note: this file is only compiled in the firmware crate (not the
    // proto crate), so tests don't run on host by default. Logic is
    // mirrored from `tashtalk_proto::bitstuff` whose tests do run.
}
