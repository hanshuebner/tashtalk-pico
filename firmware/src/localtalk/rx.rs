//! LocalTalk receiver — continuous edge-timer decoding.
//!
//! Owns PIO1 SM0 (RX edge timer). Continuously reads interval counts
//! from the RX FIFO, decodes FM-0 from edge timings, assembles HDLC
//! frames, and dispatches auto-replies (CTS/ACK) and inbound frames.

use crate::host::try_forward_inbound;
use crate::{SharedNodes, SharedTxData};
use embassy_time::Timer;
use tashtalk_proto::bitstuff::{BitUnstuffer, UnstuffEvent};
use tashtalk_proto::codec::FrameStatus;
use tashtalk_proto::llap::{auto_reply, frame_crc_ok, AsmEvent, FrameAssembler};

use super::tx;

/// Threshold for classifying edge intervals.
/// Remaining > THRESHOLD → short (T/2), ≤ THRESHOLD → long (T).
const INTERVAL_THRESHOLD: u32 = 16;

/// FM-0 decoding state.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Fm0State {
    Boundary,
    MidCell,
}

#[embassy_executor::task]
pub async fn run(
    rx: &'static crate::SharedRx,
    tx_data: &'static SharedTxData,
    nodes: &'static SharedNodes,
) -> ! {
    crate::tt_log!("LT_RX: task starting (edge timer)");

    let mut fm0_state = Fm0State::Boundary;
    let mut unstuffer = BitUnstuffer::new();
    let mut assembler = FrameAssembler::new();
    let mut frames_seen: u32 = 0;

    loop {
        let word = {
            let mut rx_guard = rx.lock().await;
            rx_guard.sm.rx().try_pull()
        };

        match word {
            Some(w) => {
                let remaining = w & 0x1F;

                if remaining == 0 {
                    // Timeout — idle gap. Reset FM-0 state.
                    fm0_state = Fm0State::Boundary;
                    continue;
                }

                let is_short = remaining > INTERVAL_THRESHOLD;

                let bit = match fm0_state {
                    Fm0State::Boundary => {
                        if is_short {
                            // Short: boundary→mid of a '0' bit.
                            fm0_state = Fm0State::MidCell;
                            Some(0u8)
                        } else {
                            // Long: full cell → '1' bit.
                            Some(1u8)
                        }
                    }
                    Fm0State::MidCell => {
                        // Should be short (mid→boundary).
                        fm0_state = Fm0State::Boundary;
                        if !is_short {
                            // Framing error — reset.
                            unstuffer.reset();
                            assembler.feed_abort();
                        }
                        None // no new bit, the '0' was already emitted
                    }
                };

                if let Some(bit) = bit {
                    match unstuffer.feed_bit(bit) {
                        UnstuffEvent::Byte(b) => {
                            assembler.feed_byte(b);
                        }
                        UnstuffEvent::Flag => {
                            if let Some(AsmEvent::FrameEnd) = assembler.feed_flag() {
                                frames_seen = frames_seen.wrapping_add(1);
                                let frame = assembler.current();
                                let crc_ok = frame_crc_ok(frame);

                                let nodes_snapshot = *nodes.lock().await;

                                // Check auto-reply before forwarding.
                                let reply = auto_reply(frame, &nodes_snapshot).into_frame();

                                // Forward to host.
                                let status = if crc_ok {
                                    FrameStatus::Good
                                } else {
                                    FrameStatus::BadCrc
                                };
                                try_forward_inbound(frame, status);

                                let t = if frame.len() >= 3 { frame[2] } else { 0xFF };
                                if frames_seen % 50 == 0 || t >= 0x80 {
                                    crate::tt_log!(
                                        "LT_RX: #{} t={:02x} crc={} len={}",
                                        frames_seen, t, crc_ok, frame.len()
                                    );
                                }

                                assembler.ack_frame_end();

                                // Transmit auto-reply immediately if needed.
                                if let Some(rf) = reply {
                                    let mut txd = tx_data.lock().await;
                                    tx::transmit_immediate(&mut txd, &rf).await;
                                }
                            }
                        }
                        UnstuffEvent::Abort => {
                            assembler.feed_abort();
                            fm0_state = Fm0State::Boundary;
                            unstuffer.reset();
                        }
                        UnstuffEvent::None => {}
                    }
                }
            }
            None => {
                // FIFO empty — yield.
                Timer::after_micros(5).await;
            }
        }
    }
}
