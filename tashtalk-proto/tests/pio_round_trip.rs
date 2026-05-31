//! PIO-level round-trip tests.
//!
//! These tests exercise the complete hardware path using the PIO simulator:
//!
//! 1. Build LLAP frame → HDLC bitstream → FM-0 encode
//! 2. Feed FM-0 symbols through SM0 (TX shift-out) simulator
//! 3. Feed SM0's output through SM2 (RX edge timer) simulator
//! 4. Decode SM2's FIFO output (interval counts) → FM-0 decode → HDLC
//! 5. Verify the original frame is recovered
//!
//! This validates that the actual PIO programs, running on the simulated
//! instruction set, produce correct results end-to-end.

use tashtalk_proto::crc::lt_crc;
use tashtalk_proto::fm0::Fm0Encoder;
use tashtalk_proto::hdlc::build_hdlc_bitstream;
use tashtalk_proto::llap::{compose_ack, compose_cts, compose_rts, frame_crc_ok};
use tashtalk_proto::llap::{AsmEvent, FrameAssembler};
use tashtalk_proto::pio_programs::{assemble_rx_edge_timer, assemble_tx_data};
use tashtalk_proto::pio_sim::PioSim;

/// Number of SM2 clock cycles per SM0 clock cycle.
///
/// SM0 runs at 460.8 kHz, SM2 at 9.216 MHz → ratio = 20.
/// Each SM0 half-cell = 20 SM2 cycles.
const RX_CYCLES_PER_HALFCELL: usize = 20;

/// Feed an LLAP frame through the full PIO pipeline and recover it.
///
/// Steps:
/// 1. HDLC-frame the input (flags + bit-stuff + CRC + flag + abort)
/// 2. FM-0 encode to half-cell symbols
/// 3. Feed through SM0 (TX shift-out) to get output pin trace
/// 4. Stretch SM0 output to SM2's clock rate
/// 5. Feed through SM2 (RX edge timer) to get interval counts
/// 6. Decode intervals → FM-0 bits → HDLC bytes → frame
fn pio_round_trip(frame: &[u8]) -> Vec<Vec<u8>> {
    // Step 1-2: HDLC frame + FM-0 encode
    let hdlc_bits = build_hdlc_bitstream(frame).expect("valid frame");
    let mut enc = Fm0Encoder::new();
    let mut fm0_symbols: Vec<u8> = Vec::new();
    for &bit in hdlc_bits.iter() {
        let (a, b) = enc.encode_bit(bit);
        fm0_symbols.push(a);
        fm0_symbols.push(b);
    }

    // Step 3: Feed through SM0 to verify shift-out matches
    let tx_prog = assemble_tx_data();
    let mut tx_sim = PioSim::from_program(&tx_prog);
    tx_sim.autopull_threshold = 32;

    // Pack FM-0 symbols into 32-bit words for the TX FIFO
    let mut tx_words: Vec<u32> = Vec::new();
    for chunk in fm0_symbols.chunks(32) {
        let mut word = 0u32;
        for (i, &bit) in chunk.iter().enumerate() {
            word |= (bit as u32) << i;
        }
        tx_words.push(word);
    }

    // Feed words into TX FIFO and shift out
    let mut sm0_output: Vec<u8> = Vec::new();
    let mut word_idx = 0;

    // Pre-fill FIFO
    while word_idx < tx_words.len() && tx_sim.tx_fifo.len() < 4 {
        tx_sim.tx_fifo.push_back(tx_words[word_idx]).unwrap();
        word_idx += 1;
    }

    for _ in 0..fm0_symbols.len() {
        tx_sim.step();
        sm0_output.push(tx_sim.out_pins as u8);

        // Refill FIFO
        if tx_sim.tx_fifo.len() < 2 && word_idx < tx_words.len() {
            tx_sim.tx_fifo.push_back(tx_words[word_idx]).unwrap();
            word_idx += 1;
        }
    }

    // Verify SM0 output matches the FM-0 symbols
    assert_eq!(
        sm0_output, fm0_symbols,
        "SM0 output should match FM-0 symbols"
    );

    // Step 4: Stretch SM0 output to SM2's clock rate
    let mut rx_pin_trace: Vec<bool> = Vec::new();
    // Leading idle (LOW) for SM2 to initialize
    for _ in 0..(RX_CYCLES_PER_HALFCELL * 4) {
        rx_pin_trace.push(false);
    }
    // Stretched FM-0 signal
    for &sym in &sm0_output {
        for _ in 0..RX_CYCLES_PER_HALFCELL {
            rx_pin_trace.push(sym != 0);
        }
    }
    // Trailing idle
    for _ in 0..(RX_CYCLES_PER_HALFCELL * 8) {
        rx_pin_trace.push(false);
    }

    // Step 5: Feed through SM2 (RX edge timer)
    let rx_prog = assemble_rx_edge_timer();
    let mut rx_sim = PioSim::from_program(&rx_prog);
    rx_sim.autopush_threshold = 5; // push after each 5-bit IN

    for &pin_state in &rx_pin_trace {
        rx_sim.jmp_pin = pin_state;
        rx_sim.step();
    }

    // Step 6: Decode SM2 FIFO output → FM-0 bits → HDLC bytes
    let intervals = decode_fifo_intervals(&rx_sim.rx_fifo);
    let decoded_bits = intervals_to_fm0_bits(&intervals);
    bits_to_frames(&decoded_bits)
}

/// Extract 5-bit interval values from packed FIFO words.
fn decode_fifo_intervals(fifo: &[u32]) -> Vec<u32> {
    let mut intervals = Vec::new();
    for &word in fifo {
        // Each word may contain multiple 5-bit values packed by autopush.
        // With autopush_threshold=5, each word contains exactly one 5-bit value
        // in the low 5 bits (ISR shifts left, so the value ends up shifted).
        // Actually, with autopush at 5 bits, each push contains the 5 bits
        // that were shifted in. Since `in x, 5` shifts X's low 5 bits into ISR
        // from the right (ISR <<= 5, ISR |= (X & 0x1F)), and autopush fires
        // after 5 bits, each FIFO word has the interval in bits [4:0].
        let val = word & 0x1F;
        intervals.push(val);
    }
    intervals
}

/// Convert edge intervals (remaining counter values) to FM-0 decoded bits.
///
/// Each interval is the remaining counter value when the edge was detected.
/// Elapsed = 31 - remaining. At 20 SM2 cycles per half-cell:
/// - Half-cell (T/2): elapsed ≈ 10 → remaining ≈ 21
/// - Full cell (T): elapsed ≈ 20 → remaining ≈ 11
/// - Timeout: remaining = 0 (from `in null, 5`)
///
/// Threshold: remaining > 16 → short (T/2), remaining ≤ 16 → long (T).
fn intervals_to_fm0_bits(intervals: &[u32]) -> Vec<u8> {
    let mut bits = Vec::new();
    let mut state = FmState::Boundary;

    for &remaining in intervals {
        if remaining == 0 {
            // Timeout — idle gap. Could signal end of frame.
            continue;
        }

        let elapsed = 31_u32.saturating_sub(remaining);
        // With 20 cycles per half-cell, short ≈ 10, long ≈ 20.
        // But the counting loop is 2 cycles per iteration, so
        // actual elapsed iterations ≈ 10 for T/2 and 20 for T.
        let is_short = elapsed < 15;

        match state {
            FmState::Boundary => {
                if is_short {
                    // Short interval: boundary to mid-cell of a '0' bit
                    bits.push(0);
                    state = FmState::MidCell;
                } else {
                    // Long interval: full cell → '1' bit
                    bits.push(1);
                    state = FmState::Boundary;
                }
            }
            FmState::MidCell => {
                // Should be short (mid-cell to next boundary)
                state = FmState::Boundary;
                // Don't emit a bit — the '0' was already emitted
            }
        }
    }

    bits
}

#[derive(Debug, Clone, Copy)]
enum FmState {
    Boundary,
    MidCell,
}

/// Convert a raw bit stream through the HDLC unstuffer to extract frames.
fn bits_to_frames(bits: &[u8]) -> Vec<Vec<u8>> {
    use tashtalk_proto::bitstuff::{BitUnstuffer, UnstuffEvent};

    let mut unstuffer = BitUnstuffer::new();
    let mut assembler = FrameAssembler::new();
    let mut frames = Vec::new();

    for &bit in bits {
        match unstuffer.feed_bit(bit) {
            UnstuffEvent::Byte(b) => {
                assembler.feed_byte(b);
            }
            UnstuffEvent::Flag => {
                if let Some(AsmEvent::FrameEnd) = assembler.feed_flag() {
                    frames.push(assembler.current().to_vec());
                    assembler.ack_frame_end();
                }
            }
            UnstuffEvent::Abort => {
                assembler.feed_abort();
            }
            UnstuffEvent::None => {}
        }
    }

    frames
}

// ── Tests ─────────────────────────────────────────────────────────────

#[test]
fn pio_round_trip_enq() {
    let frame = [0x42u8, 0x01, 0x81]; // ENQ
    let crc = lt_crc(&frame);
    let expected: Vec<u8> = frame.iter().chain(crc.iter()).copied().collect();

    let recovered = pio_round_trip(&frame);
    assert_eq!(recovered.len(), 1, "expected 1 frame, got {}", recovered.len());
    assert_eq!(recovered[0], expected);
    assert!(frame_crc_ok(&recovered[0]));
}

#[test]
fn pio_round_trip_ack() {
    let ack = compose_ack(0x42);
    let recovered = pio_round_trip(&ack[..3]);
    assert_eq!(recovered.len(), 1);
    assert_eq!(recovered[0], ack.to_vec());
    assert!(frame_crc_ok(&recovered[0]));
}

#[test]
fn pio_round_trip_rts_cts() {
    let rts = compose_rts(0x10, 0x42);
    let recovered_rts = pio_round_trip(&rts[..3]);
    assert_eq!(recovered_rts.len(), 1);
    assert_eq!(recovered_rts[0], rts.to_vec());

    let cts = compose_cts(0x42, 0x10);
    let recovered_cts = pio_round_trip(&cts[..3]);
    assert_eq!(recovered_cts.len(), 1);
    assert_eq!(recovered_cts[0], cts.to_vec());
}

#[test]
fn pio_round_trip_data_frame() {
    let mut frame = vec![0u8; 15];
    frame[0] = 0xFF;
    frame[1] = 0x42;
    frame[2] = 0x02;
    for i in 3..15 {
        frame[i] = (i * 13) as u8;
    }

    let crc = lt_crc(&frame);
    let mut expected = frame.clone();
    expected.extend_from_slice(&crc);

    let recovered = pio_round_trip(&frame);
    assert_eq!(recovered.len(), 1);
    assert_eq!(recovered[0], expected);
    assert!(frame_crc_ok(&recovered[0]));
}

#[test]
fn pio_round_trip_all_ff() {
    let frame = [0xFF, 0x42, 0x02, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF];
    let crc = lt_crc(&frame);
    let mut expected: Vec<u8> = frame.to_vec();
    expected.extend_from_slice(&crc);

    let recovered = pio_round_trip(&frame);
    assert_eq!(recovered.len(), 1);
    assert_eq!(recovered[0], expected);
    assert!(frame_crc_ok(&recovered[0]));
}

#[test]
fn sm0_output_matches_fm0_encoder() {
    // Verify SM0 produces bit-identical output to the software FM-0 encoder.
    let frame = [0x42u8, 0x01, 0x81];
    let hdlc_bits = build_hdlc_bitstream(&frame).unwrap();
    let mut enc = Fm0Encoder::new();
    let mut expected_symbols: Vec<u8> = Vec::new();
    for &bit in hdlc_bits.iter() {
        let (a, b) = enc.encode_bit(bit);
        expected_symbols.push(a);
        expected_symbols.push(b);
    }

    // Feed through SM0
    let tx_prog = assemble_tx_data();
    let mut tx_sim = PioSim::from_program(&tx_prog);
    tx_sim.autopull_threshold = 32;

    // Pack symbols into words
    for chunk in expected_symbols.chunks(32) {
        let mut word = 0u32;
        for (i, &bit) in chunk.iter().enumerate() {
            word |= (bit as u32) << i;
        }
        tx_sim.tx_fifo.push_back(word).unwrap();
    }

    let mut sm0_output: Vec<u8> = Vec::new();
    for _ in 0..expected_symbols.len() {
        tx_sim.step();
        sm0_output.push(tx_sim.out_pins as u8);
    }

    assert_eq!(sm0_output, expected_symbols);
}

#[test]
fn sm2_carrier_sense_tracks_activity() {
    // Verify SM2 sets carrier sense HIGH when edges are detected
    // and LOW during idle periods.
    let rx_prog = assemble_rx_edge_timer();
    let mut sim = PioSim::from_program(&rx_prog);
    sim.autopush_threshold = 5;

    // Start with idle (LOW) — carrier sense should go LOW after timeout.
    sim.jmp_pin = false;
    for _ in 0..100 {
        sim.step();
    }
    assert_eq!(sim.set_pins, 0, "idle: carrier sense should be LOW");

    // Generate an edge (LOW→HIGH).
    sim.jmp_pin = true;
    for _ in 0..5 {
        sim.step();
    }
    assert_eq!(sim.set_pins, 1, "after edge: carrier sense should be HIGH");

    // Go idle again.
    sim.jmp_pin = false;
    for _ in 0..100 {
        sim.step();
    }
    assert_eq!(sim.set_pins, 0, "idle again: carrier sense should be LOW");
}
