//! PIO-level round-trip tests.
//!
//! These tests exercise the complete hardware path using the PIO simulator:
//!
//! 1. Build LLAP frame (with CRC) → HDLC bitstream → FM-0 encode
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

/// Build a frame with CRC from a header slice.
fn with_crc(header: &[u8]) -> Vec<u8> {
    let crc = lt_crc(header);
    let mut frame = header.to_vec();
    frame.extend_from_slice(&crc);
    frame
}

/// Feed an LLAP frame (WITH CRC) through the full PIO pipeline and recover it.
fn pio_round_trip(frame_with_crc: &[u8]) -> Vec<Vec<u8>> {
    // Step 1-2: HDLC frame + FM-0 encode
    let hdlc_bits = build_hdlc_bitstream(frame_with_crc).expect("valid frame");
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
        let val = word & 0x1F;
        intervals.push(val);
    }
    intervals
}

/// Convert edge intervals (remaining counter values) to FM-0 decoded bits.
fn intervals_to_fm0_bits(intervals: &[u32]) -> Vec<u8> {
    let mut bits = Vec::new();
    let mut state = FmState::Boundary;

    for &remaining in intervals {
        if remaining == 0 {
            continue;
        }

        let elapsed = 31_u32.saturating_sub(remaining);
        let is_short = elapsed < 15;

        match state {
            FmState::Boundary => {
                if is_short {
                    bits.push(0);
                    state = FmState::MidCell;
                } else {
                    bits.push(1);
                    state = FmState::Boundary;
                }
            }
            FmState::MidCell => {
                state = FmState::Boundary;
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
    let frame = with_crc(&[0x42u8, 0x01, 0x81]);

    let recovered = pio_round_trip(&frame);
    assert_eq!(recovered.len(), 1, "expected 1 frame, got {}", recovered.len());
    assert_eq!(recovered[0], frame);
    assert!(frame_crc_ok(&recovered[0]));
}

#[test]
fn pio_round_trip_ack() {
    let ack = compose_ack(0x42); // already includes CRC
    let recovered = pio_round_trip(&ack);
    assert_eq!(recovered.len(), 1);
    assert_eq!(recovered[0], ack.to_vec());
    assert!(frame_crc_ok(&recovered[0]));
}

#[test]
fn pio_round_trip_rts_cts() {
    let rts = compose_rts(0x10, 0x42); // includes CRC
    let recovered_rts = pio_round_trip(&rts);
    assert_eq!(recovered_rts.len(), 1);
    assert_eq!(recovered_rts[0], rts.to_vec());

    let cts = compose_cts(0x42, 0x10); // includes CRC
    let recovered_cts = pio_round_trip(&cts);
    assert_eq!(recovered_cts.len(), 1);
    assert_eq!(recovered_cts[0], cts.to_vec());
}

#[test]
fn pio_round_trip_data_frame() {
    let mut header = vec![0u8; 15];
    header[0] = 0xFF;
    header[1] = 0x42;
    header[2] = 0x02;
    for i in 3..15 {
        header[i] = (i * 13) as u8;
    }
    let frame = with_crc(&header);

    let recovered = pio_round_trip(&frame);
    assert_eq!(recovered.len(), 1);
    assert_eq!(recovered[0], frame);
    assert!(frame_crc_ok(&recovered[0]));
}

#[test]
fn pio_round_trip_all_ff() {
    let frame = with_crc(&[0xFF, 0x42, 0x02, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]);

    let recovered = pio_round_trip(&frame);
    assert_eq!(recovered.len(), 1);
    assert_eq!(recovered[0], frame);
    assert!(frame_crc_ok(&recovered[0]));
}

#[test]
fn sm0_output_matches_fm0_encoder() {
    let frame = with_crc(&[0x42u8, 0x01, 0x81]);
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
