//! Host ↔ TashTalk byte protocol.
//!
//! Spec: `tashtalk/documentation/protocol.md`.
//!
//! ## Host → device (parser)
//!
//! The host sends a stream of command bytes:
//!
//! | Opcode | Meaning |
//! | ------ | ------- |
//! | `0x00` | No-op. |
//! | `0x01` | Transmit Frame, followed by frame bytes (no escapes). Length is inferred from the LLAP header: control frames are 5 bytes total; data frames carry their length in the 4th and 5th bytes. |
//! | `0x02` | Set Node-ID bitmap, followed by 32 bytes. |
//! | `0x03` | Set Features, followed by 1 byte. |
//!
//! All other bytes seen at command position are treated as no-ops, matching
//! the PIC firmware's behaviour at `AwaitCommand`.
//!
//! ## Device → host (encoder)
//!
//! The device emits a stream of decoded frame bytes. `0x00` in the data
//! stream is escaped as `0x00 0xFF`. End-of-frame markers (also escape
//! sequences) carry the frame's status:
//!
//! | Sequence | Meaning |
//! | -------- | ------- |
//! | `0x00 0xFD` | Frame done (good CRC, or CRC checking disabled). |
//! | `0x00 0xFC` | Frame done, CRC failed (only when `FEATURES.CHKCRC` is set). |
//! | `0x00 0xFE` | Framing error. |
//! | `0x00 0xFA` | Frame aborted. |
//!
//! Both halves of this protocol are pure byte-level state machines with no
//! hardware dependencies, so all of it lives here in `tashtalk-proto` and
//! has unit tests.

use heapless::Vec;

/// Maximum LocalTalk frame payload (per LLAP). Used to size internal buffers.
/// The PIC firmware does not enforce this limit either.
pub const MAX_FRAME_LEN: usize = 605;

/// Escape character used by both directions of the protocol.
pub const ESC: u8 = 0x00;

/// Device-to-host end-of-frame markers (the byte that follows `ESC`).
pub mod marker {
    /// Literal `0x00` in the frame body (`0x00 0xFF`).
    pub const LITERAL_ZERO: u8 = 0xFF;
    /// Frame done, good CRC (`0x00 0xFD`).
    pub const FRAME_GOOD: u8 = 0xFD;
    /// Frame done, bad CRC — only emitted when `FEATURES.CHKCRC` is set (`0x00 0xFC`).
    pub const FRAME_BAD_CRC: u8 = 0xFC;
    /// Framing error mid-frame (`0x00 0xFE`).
    pub const FRAME_FRAMING_ERROR: u8 = 0xFE;
    /// Frame aborted (`0x00 0xFA`).
    pub const FRAME_ABORTED: u8 = 0xFA;
}

/// End-of-frame outcome reported by the LocalTalk receiver to the host.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameStatus {
    Good,
    BadCrc,
    FramingError,
    Aborted,
}

impl FrameStatus {
    pub const fn marker_byte(self) -> u8 {
        match self {
            FrameStatus::Good => marker::FRAME_GOOD,
            FrameStatus::BadCrc => marker::FRAME_BAD_CRC,
            FrameStatus::FramingError => marker::FRAME_FRAMING_ERROR,
            FrameStatus::Aborted => marker::FRAME_ABORTED,
        }
    }
}

// ── Device → host: escape encoder ────────────────────────────────────────────

/// Write one decoded frame byte to `out`, escaping `0x00` as `0x00 0xFF`.
///
/// Returns the number of bytes appended (1 or 2). The caller is responsible
/// for ensuring `out` has space; the function panics if it does not.
pub fn write_byte_escaped<const N: usize>(out: &mut Vec<u8, N>, byte: u8) {
    if byte == ESC {
        out.push(ESC).unwrap();
        out.push(marker::LITERAL_ZERO).unwrap();
    } else {
        out.push(byte).unwrap();
    }
}

/// Write an end-of-frame status marker (`0x00 <marker>`).
pub fn write_status<const N: usize>(out: &mut Vec<u8, N>, status: FrameStatus) {
    out.push(ESC).unwrap();
    out.push(status.marker_byte()).unwrap();
}

/// Encode a complete decoded frame for transmission to the host.
///
/// Convenience for tests and bring-up code — the firmware will normally
/// stream byte-by-byte through the USB CDC class rather than buffer a whole
/// frame.
pub fn encode_frame(payload: &[u8], status: FrameStatus) -> Vec<u8, { MAX_FRAME_LEN * 2 + 2 }> {
    let mut out = Vec::new();
    for &b in payload {
        write_byte_escaped(&mut out, b);
    }
    write_status(&mut out, status);
    out
}

// ── Host → device: command parser ────────────────────────────────────────────

/// A complete command parsed from the host byte stream.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum HostCommand<'a> {
    /// 0x00 — no-op. Mostly used to flush the device back to its idle
    /// command-awaiting state.
    Noop,
    /// 0x01 — transmit this LocalTalk frame. Includes the trailing two CRC
    /// bytes; whether the device honours them depends on `FEATURES.CALCCRC`.
    TransmitFrame(&'a [u8]),
    /// 0x02 — set the 256-bit node-ID bitmap (32 bytes; bit 0 of byte 0 = ID 0).
    SetNodeIds(&'a [u8; 32]),
    /// 0x03 — set the feature byte (bit 7: CRC calc; bit 6: CRC check).
    SetFeatures(u8),
}

/// Internal state of the host-to-device command parser.
///
/// The parser is byte-driven (one input byte → at most one output command)
/// because that matches how bytes arrive from USB and was the model used in
/// the PIC firmware's `AwaitCommand` / `AwaitDest`/... chain.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum State {
    AwaitCommand,
    AwaitFrameByte(FrameProgress),
    AwaitNodeIds { received: usize },
    AwaitFeatures,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum FrameProgress {
    /// 0: dest, 1: src, 2: type, 3: 4th, 4: 5th.
    Header(u8),
    /// Data frame body. `remaining` counts bytes still to read (including
    /// the trailing two CRC bytes).
    DataBody { remaining: u16 },
}

/// Streaming command parser for the host byte stream.
///
/// Calls to [`push`](Self::push) feed one byte at a time and may produce
/// callbacks via the user-supplied [`Sink`] trait. We use a sink rather
/// than returning frames directly so frame bodies can stream out
/// byte-by-byte (the firmware will push each byte into the PIO TX FIFO as
/// it arrives, rather than buffering a whole 605-byte frame in RAM first).
pub struct CommandParser {
    state: State,
    /// 5-byte header buffer for the in-progress TransmitFrame.
    hdr: [u8; 5],
    /// 32-byte node-ID buffer for the in-progress SetNodeIds.
    nodes: [u8; 32],
}

impl Default for CommandParser {
    fn default() -> Self {
        Self::new()
    }
}

impl core::fmt::Debug for CommandParser {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("CommandParser").field("state", &self.state).finish()
    }
}

/// Receives parser callbacks. The firmware implements this against the
/// LocalTalk TX path; tests implement it against a `Vec<Event>`.
pub trait Sink {
    /// `0x00` no-op received at command boundary.
    fn on_noop(&mut self) {}
    /// The 3-byte LLAP header has been seen for a frame about to be
    /// transmitted. Useful for waking up the LocalTalk TX state machine.
    fn on_frame_header(&mut self, dest: u8, src: u8, llap_type: u8);
    /// A subsequent byte of an in-flight TransmitFrame. Called in order
    /// for every byte after the 3-byte LLAP header up to and including the
    /// two trailing CRC bytes.
    fn on_frame_byte(&mut self, byte: u8);
    /// The current TransmitFrame has finished (all bytes delivered).
    fn on_frame_end(&mut self);
    /// `0x02` SetNodeIds, with the full 32-byte bitmap.
    fn on_set_node_ids(&mut self, bitmap: &[u8; 32]);
    /// `0x03` SetFeatures.
    fn on_set_features(&mut self, features: u8);
}

impl CommandParser {
    pub const fn new() -> Self {
        Self {
            state: State::AwaitCommand,
            hdr: [0; 5],
            nodes: [0; 32],
        }
    }

    /// Reset to the idle "awaiting command" state. The PIC firmware
    /// guarantees this happens after 1024 `0x00` no-op bytes; we make it
    /// available as an explicit call too.
    pub fn reset(&mut self) {
        self.state = State::AwaitCommand;
    }

    /// Feed one byte from the host. Drives the parser forward and emits
    /// callbacks via `sink`.
    pub fn push<S: Sink>(&mut self, byte: u8, sink: &mut S) {
        self.state = match self.state {
            State::AwaitCommand => match byte {
                0x00 => {
                    sink.on_noop();
                    State::AwaitCommand
                }
                0x01 => State::AwaitFrameByte(FrameProgress::Header(0)),
                0x02 => State::AwaitNodeIds { received: 0 },
                0x03 => State::AwaitFeatures,
                _ => State::AwaitCommand, // unknown opcodes are no-ops
            },

            State::AwaitFrameByte(FrameProgress::Header(i)) => {
                self.hdr[i as usize] = byte;
                let next_i = i + 1;
                if next_i == 3 {
                    // LLAP header just completed — wake up TX.
                    sink.on_frame_header(self.hdr[0], self.hdr[1], self.hdr[2]);
                    sink.on_frame_byte(self.hdr[0]);
                    sink.on_frame_byte(self.hdr[1]);
                    sink.on_frame_byte(self.hdr[2]);
                }
                if next_i < 5 {
                    State::AwaitFrameByte(FrameProgress::Header(next_i))
                } else {
                    // Just stored the 5th byte (index 4). Decide control vs data.
                    sink.on_frame_byte(self.hdr[3]);
                    sink.on_frame_byte(self.hdr[4]);
                    let llap_type = self.hdr[2];
                    if llap_type & 0x80 != 0 {
                        // Control frame: 5 bytes total, header already
                        // includes the 2 trailing CRC bytes. Done.
                        sink.on_frame_end();
                        State::AwaitCommand
                    } else {
                        // Data frame: extract length from bytes 3 and 4.
                        // length = ((hdr[3] & 0x03) << 8) | hdr[4]
                        // This length does NOT count the 3 LLAP header bytes
                        // and DOES count the length field itself but not the
                        // two CRC bytes. From the spec: total remaining =
                        // length - 2 (bytes 3..4 already consumed) + 2 CRC.
                        let length =
                            (((self.hdr[3] & 0x03) as u16) << 8) | self.hdr[4] as u16;
                        // bytes 3 and 4 are accounted for in `length` already;
                        // remaining body bytes = length - 2, then +2 for CRC.
                        let remaining = length.saturating_sub(2).saturating_add(2);
                        if remaining == 0 {
                            sink.on_frame_end();
                            State::AwaitCommand
                        } else {
                            State::AwaitFrameByte(FrameProgress::DataBody { remaining })
                        }
                    }
                }
            }

            State::AwaitFrameByte(FrameProgress::DataBody { remaining }) => {
                sink.on_frame_byte(byte);
                let next = remaining - 1;
                if next == 0 {
                    sink.on_frame_end();
                    State::AwaitCommand
                } else {
                    State::AwaitFrameByte(FrameProgress::DataBody { remaining: next })
                }
            }

            State::AwaitNodeIds { received } => {
                self.nodes[received] = byte;
                let next = received + 1;
                if next == 32 {
                    sink.on_set_node_ids(&self.nodes);
                    State::AwaitCommand
                } else {
                    State::AwaitNodeIds { received: next }
                }
            }

            State::AwaitFeatures => {
                sink.on_set_features(byte);
                State::AwaitCommand
            }
        };
    }

    /// Feed a whole slice.
    pub fn push_slice<S: Sink>(&mut self, data: &[u8], sink: &mut S) {
        for &b in data {
            self.push(b, sink);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    extern crate std;
    use std::vec::Vec as StdVec;

    /// Test sink: records every callback as an Event for assertion.
    #[derive(Debug, PartialEq, Eq)]
    enum Event {
        Noop,
        FrameHeader(u8, u8, u8),
        FrameByte(u8),
        FrameEnd,
        SetNodeIds([u8; 32]),
        SetFeatures(u8),
    }

    #[derive(Default)]
    struct TestSink {
        events: StdVec<Event>,
    }

    impl Sink for TestSink {
        fn on_noop(&mut self) {
            self.events.push(Event::Noop);
        }
        fn on_frame_header(&mut self, d: u8, s: u8, t: u8) {
            self.events.push(Event::FrameHeader(d, s, t));
        }
        fn on_frame_byte(&mut self, b: u8) {
            self.events.push(Event::FrameByte(b));
        }
        fn on_frame_end(&mut self) {
            self.events.push(Event::FrameEnd);
        }
        fn on_set_node_ids(&mut self, b: &[u8; 32]) {
            self.events.push(Event::SetNodeIds(*b));
        }
        fn on_set_features(&mut self, f: u8) {
            self.events.push(Event::SetFeatures(f));
        }
    }

    // ── Encoder ──

    #[test]
    fn escape_literal_zero() {
        let mut out: heapless::Vec<u8, 16> = heapless::Vec::new();
        write_byte_escaped(&mut out, 0xAA);
        write_byte_escaped(&mut out, 0x00);
        write_byte_escaped(&mut out, 0xBB);
        assert_eq!(&out[..], &[0xAA, 0x00, 0xFF, 0xBB]);
    }

    #[test]
    fn encode_status_markers() {
        for (status, expected) in [
            (FrameStatus::Good, 0xFD),
            (FrameStatus::BadCrc, 0xFC),
            (FrameStatus::FramingError, 0xFE),
            (FrameStatus::Aborted, 0xFA),
        ] {
            let mut out: heapless::Vec<u8, 4> = heapless::Vec::new();
            write_status(&mut out, status);
            assert_eq!(&out[..], &[ESC, expected]);
        }
    }

    #[test]
    fn encode_frame_with_zero() {
        let encoded = encode_frame(&[0x12, 0x00, 0x34], FrameStatus::Good);
        assert_eq!(&encoded[..], &[0x12, 0x00, 0xFF, 0x34, 0x00, 0xFD]);
    }

    // ── Parser: commands ──

    #[test]
    fn parse_noop() {
        let mut p = CommandParser::new();
        let mut sink = TestSink::default();
        p.push(0x00, &mut sink);
        assert_eq!(sink.events, [Event::Noop]);
    }

    #[test]
    fn parse_set_node_ids() {
        let mut p = CommandParser::new();
        let mut sink = TestSink::default();
        p.push(0x02, &mut sink);
        let mut bitmap = [0u8; 32];
        for (i, b) in bitmap.iter_mut().enumerate() {
            *b = i as u8;
        }
        p.push_slice(&bitmap, &mut sink);
        assert_eq!(sink.events, [Event::SetNodeIds(bitmap)]);
    }

    #[test]
    fn parse_set_features() {
        let mut p = CommandParser::new();
        let mut sink = TestSink::default();
        p.push(0x03, &mut sink);
        p.push(0xC0, &mut sink);
        assert_eq!(sink.events, [Event::SetFeatures(0xC0)]);
    }

    #[test]
    fn parse_control_frame_is_5_bytes() {
        // type byte 0x82 = ACK (high bit set → control frame).
        let mut p = CommandParser::new();
        let mut sink = TestSink::default();
        let bytes = [0x01, 0x12, 0x34, 0x82, 0xAA, 0xBB];
        p.push_slice(&bytes, &mut sink);
        assert_eq!(
            sink.events,
            [
                Event::FrameHeader(0x12, 0x34, 0x82),
                Event::FrameByte(0x12),
                Event::FrameByte(0x34),
                Event::FrameByte(0x82),
                Event::FrameByte(0xAA),
                Event::FrameByte(0xBB),
                Event::FrameEnd,
            ]
        );
    }

    #[test]
    fn parse_data_frame_with_length() {
        // Data frame: type byte 0x01 (high bit clear). Length field carried
        // in bytes 4-5 of the header (after dest/src/type). We pick
        // length = 4, which the spec defines as "includes the length field
        // itself, but not the 3 LLAP header bytes or the two CRC bytes".
        // So after we send the 5 header bytes, expect 4-2 = 2 more body
        // bytes plus 2 CRC bytes = 4 trailing bytes.
        let mut p = CommandParser::new();
        let mut sink = TestSink::default();
        let header = [0x01, 0x12, 0x34, 0x01, 0x00, 0x04]; // 0x01 cmd + 5 hdr
        let body = [0xAA, 0xBB, 0xCC, 0xDD]; // 2 data + 2 CRC
        p.push_slice(&header, &mut sink);
        p.push_slice(&body, &mut sink);

        let expected = [
            Event::FrameHeader(0x12, 0x34, 0x01),
            Event::FrameByte(0x12),
            Event::FrameByte(0x34),
            Event::FrameByte(0x01),
            Event::FrameByte(0x00),
            Event::FrameByte(0x04),
            Event::FrameByte(0xAA),
            Event::FrameByte(0xBB),
            Event::FrameByte(0xCC),
            Event::FrameByte(0xDD),
            Event::FrameEnd,
        ];
        assert_eq!(sink.events, expected);
    }

    /// Unknown opcodes (anything that isn't 0x00..=0x03) must behave like
    /// no-ops, matching `AwaitCommand` in `one-chip.asm`.
    #[test]
    fn unknown_opcodes_are_noops() {
        let mut p = CommandParser::new();
        let mut sink = TestSink::default();
        p.push(0x77, &mut sink);
        p.push(0xFF, &mut sink);
        // After two unknowns, a valid command still parses:
        p.push(0x03, &mut sink);
        p.push(0x80, &mut sink);
        assert_eq!(sink.events, [Event::SetFeatures(0x80)]);
    }
}
