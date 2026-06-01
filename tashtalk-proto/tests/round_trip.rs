//! Full round-trip integration tests for the LocalTalk TX/RX pipeline.
//!
//! These tests exercise the complete path:
//!   LLAP frame → HDLC framing → FM-0 encode → (simulated wire) →
//!   FM-0 decode → HDLC deframe → verify original LLAP frame
//!
//! The "simulated wire" stretches each FM-0 half-cell symbol into N
//! samples (mimicking the 8× oversampling that the PIO RX edge timer
//! + software decoder would produce).

use tashtalk_proto::crc::lt_crc;
use tashtalk_proto::fm0::{Fm0Decoder, Fm0Encoder, Fm0Event};
use tashtalk_proto::hdlc::{build_hdlc_bitstream, build_immediate_tx_buffer, build_tx_buffer};
use tashtalk_proto::llap::{
    compose_ack, compose_cts, compose_rts, frame_crc_ok, FrameAssembler, AsmEvent,
};

/// Simulate the wire: stretch FM-0 half-cell symbols to `samples_per_half`
/// samples each, prepend idle, append trailing edge + idle.
fn simulate_wire(fm0_symbols: &[u8], samples_per_half: usize) -> Vec<u8> {
    let mut wire = Vec::new();

    // Leading idle (LOW) so the decoder sees a clean first edge.
    wire.extend(std::iter::repeat(0u8).take(samples_per_half * 4));

    // Stretch each half-cell symbol to `samples_per_half` samples.
    for &sym in fm0_symbols {
        for _ in 0..samples_per_half {
            wire.push(sym);
        }
    }

    // Trailing edge + idle so the last bit's closing edge is visible.
    let last = fm0_symbols.last().copied().unwrap_or(0);
    wire.extend(std::iter::repeat(last ^ 1).take(samples_per_half * 2));
    wire.extend(std::iter::repeat(last ^ 1).take(samples_per_half * 4));

    wire
}

/// Decode a wire sample stream through FM-0 decoder + frame assembler.
/// Returns all complete frames (as byte vectors).
fn decode_wire(wire: &[u8]) -> Vec<Vec<u8>> {
    let mut decoder = Fm0Decoder::new();
    let mut assembler = FrameAssembler::new();
    let mut frames = Vec::new();

    for &sample in wire {
        if let Some(event) = decoder.feed_sample(sample) {
            match event {
                Fm0Event::Byte(b) => {
                    assembler.feed_byte(b);
                }
                Fm0Event::Flag => {
                    if let Some(AsmEvent::FrameEnd) = assembler.feed_flag() {
                        frames.push(assembler.current().to_vec());
                        assembler.ack_frame_end();
                    }
                }
                Fm0Event::Abort => {
                    assembler.feed_abort();
                }
            }
        }
    }

    frames
}

/// Full round-trip helper: build HDLC bitstream, FM-0 encode, simulate
/// wire, decode, return recovered frames.
fn round_trip(frame: &[u8], samples_per_half: usize) -> Vec<Vec<u8>> {
    let hdlc_bits = build_hdlc_bitstream(frame).expect("valid frame");

    // FM-0 encode the HDLC bitstream.
    let mut enc = Fm0Encoder::new();
    let mut fm0_symbols = Vec::new();
    for &bit in hdlc_bits.iter() {
        let (a, b) = enc.encode_bit(bit);
        fm0_symbols.push(a);
        fm0_symbols.push(b);
    }

    let wire = simulate_wire(&fm0_symbols, samples_per_half);
    decode_wire(&wire)
}

// ── Round-trip tests ──────────────────────────────────────────────────

#[test]
fn round_trip_enq_frame() {
    let frame = [0x42u8, 0x01, 0x81]; // ENQ
    let crc = lt_crc(&frame);
    let full_frame: Vec<u8> = frame.iter().chain(crc.iter()).copied().collect();

    let recovered = round_trip(&frame, 4);
    assert_eq!(recovered.len(), 1, "expected 1 frame, got {:?}", recovered);
    assert_eq!(recovered[0], full_frame);
    assert!(frame_crc_ok(&recovered[0]));
}

#[test]
fn round_trip_ack_frame() {
    let ack = compose_ack(0x42);
    // compose_ack includes CRC, so pass the header-only part to build_hdlc.
    let frame = &ack[..3];
    let recovered = round_trip(frame, 4);
    assert_eq!(recovered.len(), 1);
    assert_eq!(recovered[0], ack.to_vec());
    assert!(frame_crc_ok(&recovered[0]));
}

#[test]
fn round_trip_rts_frame() {
    let rts = compose_rts(0x10, 0x42);
    let recovered = round_trip(&rts[..3], 4);
    assert_eq!(recovered.len(), 1);
    assert_eq!(recovered[0], rts.to_vec());
    assert!(frame_crc_ok(&recovered[0]));
}

#[test]
fn round_trip_cts_frame() {
    let cts = compose_cts(0x42, 0x10);
    let recovered = round_trip(&cts[..3], 4);
    assert_eq!(recovered.len(), 1);
    assert_eq!(recovered[0], cts.to_vec());
    assert!(frame_crc_ok(&recovered[0]));
}

#[test]
fn round_trip_data_frame() {
    let mut frame = vec![0u8; 20];
    frame[0] = 0xFF; // broadcast
    frame[1] = 0x42;
    frame[2] = 0x01; // DDP short
    for i in 3..20 {
        frame[i] = i as u8;
    }

    let crc = lt_crc(&frame);
    let mut expected = frame.clone();
    expected.extend_from_slice(&crc);

    let recovered = round_trip(&frame, 4);
    assert_eq!(recovered.len(), 1);
    assert_eq!(recovered[0], expected);
    assert!(frame_crc_ok(&recovered[0]));
}

#[test]
fn round_trip_max_control_types() {
    // Test all four control frame types.
    for (dest, src, ltype, label) in [
        (0x42, 0x01, 0x81u8, "ENQ"),
        (0x42, 0x42, 0x82, "ACK"),
        (0x10, 0x42, 0x84, "RTS"),
        (0x42, 0x10, 0x85, "CTS"),
    ] {
        let frame = [dest, src, ltype];
        let crc = lt_crc(&frame);
        let expected: Vec<u8> = frame.iter().chain(crc.iter()).copied().collect();

        let recovered = round_trip(&frame, 4);
        assert_eq!(recovered.len(), 1, "{label}: wrong frame count");
        assert_eq!(recovered[0], expected, "{label}: content mismatch");
        assert!(frame_crc_ok(&recovered[0]), "{label}: CRC failed");
    }
}

#[test]
fn round_trip_all_ff_data() {
    // Worst-case bit stuffing: payload of all 0xFF.
    let mut frame = vec![0xFFu8; 50];
    frame[0] = 0xFF; // broadcast
    frame[1] = 0x42;
    frame[2] = 0x02;

    let crc = lt_crc(&frame);
    let mut expected = frame.clone();
    expected.extend_from_slice(&crc);

    let recovered = round_trip(&frame, 4);
    assert_eq!(recovered.len(), 1);
    assert_eq!(recovered[0], expected);
    assert!(frame_crc_ok(&recovered[0]));
}

#[test]
fn round_trip_all_zero_data() {
    let mut frame = vec![0x00u8; 30];
    frame[0] = 0xFF;
    frame[1] = 0x42;
    frame[2] = 0x02;

    let crc = lt_crc(&frame);
    let mut expected = frame.clone();
    expected.extend_from_slice(&crc);

    let recovered = round_trip(&frame, 4);
    assert_eq!(recovered.len(), 1);
    assert_eq!(recovered[0], expected);
    assert!(frame_crc_ok(&recovered[0]));
}

// ── Clock tolerance tests ─────────────────────────────────────────────

#[test]
fn round_trip_slow_clock() {
    // Sender's clock is slower: 5 samples per half-cell instead of 4.
    let frame = [0x42u8, 0x01, 0x81];
    let crc = lt_crc(&frame);
    let expected: Vec<u8> = frame.iter().chain(crc.iter()).copied().collect();

    let recovered = round_trip(&frame, 5);
    assert_eq!(recovered.len(), 1);
    assert_eq!(recovered[0], expected);
}

#[test]
fn round_trip_fast_clock() {
    // Sender's clock is faster: 3 samples per half-cell.
    let frame = [0x42u8, 0x01, 0x81];
    let crc = lt_crc(&frame);
    let expected: Vec<u8> = frame.iter().chain(crc.iter()).copied().collect();

    let recovered = round_trip(&frame, 3);
    assert_eq!(recovered.len(), 1);
    assert_eq!(recovered[0], expected);
}

// ── TX pipeline tests ─────────────────────────────────────────────────

#[test]
fn tx_buffer_round_trip() {
    // Test build_tx_buffer produces a buffer that decodes correctly.
    let frame = [0x42u8, 0x01, 0x81];
    let (buf, frame_hc) = build_tx_buffer(&frame).unwrap();

    // The sync prefix (6 half-cells) is don't-care on the wire when
    // TX_EN is gated. Skip it and decode just the frame portion.
    use tashtalk_proto::hdlc::SM1_PRE_FRAME_CYCLES;
    let frame_symbols = &buf[SM1_PRE_FRAME_CYCLES..];
    assert_eq!(frame_symbols.len(), frame_hc as usize);

    let wire = simulate_wire(frame_symbols, 4);
    let recovered = decode_wire(&wire);
    assert_eq!(recovered.len(), 1);
    assert!(frame_crc_ok(&recovered[0]));
    assert_eq!(&recovered[0][..3], &frame);
}

#[test]
fn immediate_tx_buffer_round_trip() {
    // Test build_immediate_tx_buffer (no sync pulse).
    let cts = compose_cts(0x42, 0x10);
    let buf = build_immediate_tx_buffer(&cts[..3]).unwrap();

    let wire = simulate_wire(&buf, 4);
    let recovered = decode_wire(&wire);
    assert_eq!(recovered.len(), 1);
    assert!(frame_crc_ok(&recovered[0]));
    assert_eq!(recovered[0], cts.to_vec());
}

// ── Dialog simulation tests ───────────────────────────────────────────

#[test]
fn rts_cts_data_dialog() {
    // Simulate a complete directed dialog:
    // Node A (0x42) sends RTS to node B (0x10)
    // Node B responds with CTS
    // Node A sends data frame

    let rts = compose_rts(0x10, 0x42);
    let cts = compose_cts(0x42, 0x10);
    let mut data_frame = vec![0u8; 15];
    data_frame[0] = 0x10;
    data_frame[1] = 0x42;
    data_frame[2] = 0x01;
    for i in 3..15 {
        data_frame[i] = (i * 7) as u8;
    }

    // Verify each frame round-trips independently.
    let rts_recovered = round_trip(&rts[..3], 4);
    assert_eq!(rts_recovered.len(), 1);
    assert!(frame_crc_ok(&rts_recovered[0]));
    assert_eq!(rts_recovered[0][2], 0x84); // RTS type

    let cts_recovered = round_trip(&cts[..3], 4);
    assert_eq!(cts_recovered.len(), 1);
    assert!(frame_crc_ok(&cts_recovered[0]));
    assert_eq!(cts_recovered[0][2], 0x85); // CTS type

    let data_recovered = round_trip(&data_frame, 4);
    assert_eq!(data_recovered.len(), 1);
    assert!(frame_crc_ok(&data_recovered[0]));
    assert_eq!(&data_recovered[0][..data_frame.len()], &data_frame[..]);
}

#[test]
fn enq_ack_dialog() {
    // Node probing: ENQ from 0x01 to 0x42, ACK back.
    let enq_frame = [0x42u8, 0x01, 0x81];
    let ack = compose_ack(0x42);

    let enq_recovered = round_trip(&enq_frame, 4);
    assert_eq!(enq_recovered.len(), 1);
    assert!(frame_crc_ok(&enq_recovered[0]));

    let ack_recovered = round_trip(&ack[..3], 4);
    assert_eq!(ack_recovered.len(), 1);
    assert!(frame_crc_ok(&ack_recovered[0]));
    assert_eq!(ack_recovered[0], ack.to_vec());
}

// ── Larger frame stress test ──────────────────────────────────────────

#[test]
fn round_trip_large_data_frame() {
    // Near-maximum payload (not quite 600 to keep test fast).
    let mut frame = vec![0u8; 200];
    frame[0] = 0x10;
    frame[1] = 0x42;
    frame[2] = 0x02;
    // Fill with varied data to exercise bit-stuffing thoroughly.
    for i in 3..200 {
        frame[i] = ((i * 37 + 13) & 0xFF) as u8;
    }

    let crc = lt_crc(&frame);
    let mut expected = frame.clone();
    expected.extend_from_slice(&crc);

    let recovered = round_trip(&frame, 4);
    assert_eq!(recovered.len(), 1);
    assert_eq!(recovered[0], expected);
    assert!(frame_crc_ok(&recovered[0]));
}
