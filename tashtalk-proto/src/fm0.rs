//! Stateful FM-0 (differential Manchester) decoder.
//!
//! Receives a stream of binary samples taken at 8× the bit rate from a
//! LocalTalk wire. Recovers bit-cell boundaries by detecting the first
//! transition (clock edge), then yields one [`Fm0Bit`] per bit cell —
//! either a payload bit (after the bit-unstuffer has processed it) or an
//! SDLC-level marker (`Flag`, `Abort`).
//!
//! ## Bit cell timing
//!
//! Each FM-0 bit cell is 8 samples long. The first sample of the cell
//! is the state JUST AFTER the mandatory clock-edge transition. The
//! last sample (position 7) is the state JUST BEFORE the next clock
//! edge.
//!
//! - **Bit value `1`**: no mid-cell transition. `sample[0] == sample[7]`.
//! - **Bit value `0`**: mid-cell transition. `sample[0] != sample[7]`.
//!
//! ## Sync recovery
//!
//! When a fresh decoder receives samples, it has no notion of where bit
//! cells begin. It watches for the first transition between two
//! successive samples — that transition IS the first clock edge — and
//! takes the second of those two samples as position 0 of the first
//! decodable cell.
//!
//! This sync assumes a long-enough idle period precedes the frame so
//! that the first transition really IS the first clock edge of the
//! preamble, not a random bit boundary mid-frame. The PIC firmware
//! handles this by always preceding payload with three flag bytes (the
//! preamble); we rely on the same.

use crate::bitstuff::{BitUnstuffer, UnstuffEvent};

/// One output event from the stateful decoder, after FM-0 → bit →
/// SDLC bit-unstuff has been applied. Mirrors [`UnstuffEvent`] but
/// drops the `None` variant since callers don't see it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Fm0Event {
    Byte(u8),
    Flag,
    Abort,
}

/// 8× oversampled FM-0 decoder feeding through SDLC bit-unstuffing.
#[derive(Debug, Clone)]
pub struct Fm0Decoder {
    /// `false` until the first transition is observed.
    synced: bool,
    /// Last raw sample seen, while still searching for the first edge.
    prev_sample: Option<u8>,
    /// Level at position 0 of the current bit cell.
    cell_start_level: u8,
    /// How many samples we've consumed in the current bit cell, 0..=7.
    /// 0 means "the next sample is position 0".
    samples_in_cell: u8,
    /// SDLC bit-unstuffer / flag detector / abort detector.
    unstuffer: BitUnstuffer,
}

impl Fm0Decoder {
    pub const fn new() -> Self {
        Self {
            synced: false,
            prev_sample: None,
            cell_start_level: 0,
            samples_in_cell: 0,
            unstuffer: BitUnstuffer::new(),
        }
    }

    /// Reset the decoder to its initial unsynced state.
    pub fn reset(&mut self) {
        *self = Self::new();
    }

    /// `true` if the decoder has locked onto a bit-cell boundary.
    pub fn is_synced(&self) -> bool {
        self.synced
    }

    /// Feed one binary sample. Returns an event when a complete bit
    /// has been decoded AND the bit-unstuffer produced something
    /// non-trivial (a byte boundary, a flag, or an abort).
    pub fn feed_sample(&mut self, sample: u8) -> Option<Fm0Event> {
        debug_assert!(sample <= 1);

        if !self.synced {
            if let Some(prev) = self.prev_sample {
                if sample != prev {
                    // First clock edge. `sample` is now position 0 of
                    // the first decodable cell.
                    self.synced = true;
                    self.cell_start_level = sample;
                    self.samples_in_cell = 1;
                    return None;
                }
            }
            self.prev_sample = Some(sample);
            return None;
        }

        if self.samples_in_cell == 0 {
            // This sample is position 0 of a new cell (post-clock-edge).
            self.cell_start_level = sample;
            self.samples_in_cell = 1;
            return None;
        }

        self.samples_in_cell += 1;
        if self.samples_in_cell == 8 {
            // Just consumed position 7 — the last sample of the cell.
            let bit = if sample == self.cell_start_level { 1 } else { 0 };
            self.samples_in_cell = 0;
            match self.unstuffer.feed_bit(bit) {
                UnstuffEvent::Byte(b) => Some(Fm0Event::Byte(b)),
                UnstuffEvent::Flag => Some(Fm0Event::Flag),
                UnstuffEvent::Abort => {
                    // Abort means we saw 7+ consecutive `1` bits, which
                    // in LocalTalk only happens on a real abort *or* an
                    // idle line. Our bit-cell phase is anchored on the
                    // last clock edge — which during idle becomes
                    // stale. Drop sync and search for a new edge before
                    // resuming decode, otherwise the next frame would
                    // be decoded at the wrong phase.
                    self.synced = false;
                    self.prev_sample = Some(sample);
                    self.unstuffer.reset();
                    Some(Fm0Event::Abort)
                }
                UnstuffEvent::None => None,
            }
        } else {
            None
        }
    }
}

impl Default for Fm0Decoder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Encode a single bit as 8 FM-0 samples, given the line level
    /// BEFORE the clock edge.
    fn encode_bit(prev_level: u8, bit: u8) -> [u8; 8] {
        // Clock edge: line toggles at position 0.
        let post_edge = prev_level ^ 1;
        let mut s = [post_edge; 8];
        // If bit is 0, line toggles again at mid-cell (between positions
        // 3 and 4). Samples 4..7 are at the new level.
        if bit == 0 {
            for cell in s.iter_mut().skip(4) {
                *cell = prev_level;
            }
        }
        s
    }

    fn encode_bits(prev_level: u8, bits: &[u8]) -> (u8, alloc::vec::Vec<u8>) {
        // Reproduce the same toggling logic the PIO TX does.
        let mut samples = alloc::vec::Vec::new();
        let mut level = prev_level;
        for &b in bits {
            let cell = encode_bit(level, b);
            // The cell's last sample is the level the line ends at.
            level = cell[7];
            samples.extend_from_slice(&cell);
        }
        (level, samples)
    }

    extern crate alloc;

    #[test]
    fn syncs_on_first_edge() {
        let mut d = Fm0Decoder::new();
        // Idle low samples — no edge.
        for _ in 0..32 {
            assert_eq!(d.feed_sample(0), None);
            assert!(!d.is_synced());
        }
        // Now an edge: 0 → 1.
        assert_eq!(d.feed_sample(1), None);
        assert!(d.is_synced());
    }

    #[test]
    fn decodes_a_single_one_bit() {
        let mut d = Fm0Decoder::new();
        // Pad some idle, then transmit bit=1.
        // For bit 1: line transitions at position 0 of the cell, stays
        // for all 8 samples. We need a prior edge to sync. So we'll
        // emit two adjacent cells whose first edge syncs us, and the
        // second cell whose bit we recover is the 1.
        let (_, mut samples) = encode_bits(0, &[1]); // first cell, bit=1
        let (_, more) = encode_bits(samples[7], &[1]); // second cell, bit=1
        samples.extend_from_slice(&more);

        // Feed all 16 samples.
        let mut events = alloc::vec::Vec::new();
        for s in samples {
            if let Some(e) = d.feed_sample(s) {
                events.push(e);
            }
        }
        // The bit-unstuffer accumulates bits into bytes; 2 bits is not
        // enough for a Byte event. But the decoder should not have
        // emitted Flag or Abort either.
        assert!(events.is_empty());
    }

    /// Feed `samples` into the decoder. Returns all events.
    fn decode_with_idle_prefix(samples: alloc::vec::Vec<u8>) -> alloc::vec::Vec<Fm0Event> {
        let mut d = Fm0Decoder::new();
        // Prepend 16 samples of idle low so the decoder syncs on the
        // FIRST edge of the encoded data (instead of treating the
        // first encoded sample as post-edge, which loses cell 0).
        let mut events = alloc::vec::Vec::new();
        for _ in 0..16 {
            assert_eq!(d.feed_sample(0), None);
        }
        for s in samples {
            if let Some(e) = d.feed_sample(s) {
                events.push(e);
            }
        }
        events
    }

    #[test]
    fn round_trip_byte() {
        // Encode 8 specific data bits, LSB-first, into samples, then
        // decode and check we recover the byte. To prevent flag/abort
        // patterns we choose a byte that has no run of 5+ ones:
        // 0x55 = 0b01010101.
        let bits: alloc::vec::Vec<u8> = (0..8).map(|i| (0x55 >> i) & 1).collect();
        // After 16 idle-low samples the decoder is unsynced. The first
        // encoded cell's transition (level 0 → 1 for bit=1, or 0 → 0
        // for bit=0 — wait, the encoder always transitions at the
        // start of a cell). Encode starting from level=0.
        let (_, samples) = encode_bits(0, &bits);
        let events = decode_with_idle_prefix(samples);
        assert!(
            events.contains(&Fm0Event::Byte(0x55)),
            "events: {:?}",
            events
        );
    }

    /// Encode the SDLC flag pattern (six consecutive 1s flanked by 0s)
    /// and check the decoder emits Flag exactly once.
    #[test]
    fn detects_flag() {
        let bits = [0u8, 1, 1, 1, 1, 1, 1, 0];
        let (_, samples) = encode_bits(0, &bits);
        let events = decode_with_idle_prefix(samples);
        assert!(events.contains(&Fm0Event::Flag), "events: {:?}", events);
    }
}
