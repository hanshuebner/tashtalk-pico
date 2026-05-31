//! LocalTalk PHY — reimplemented from the Zilog AN006401 app note.
//!
//! ## Architecture
//!
//! Three PIO state machines handle the timing-critical operations:
//!
//! - **SM0 (TX Data)**: shifts pre-encoded FM-0 half-cell symbols from
//!   FIFO to the TX data pin at 460.8 kHz (2× bit rate).
//! - **SM1 (TX Enable)**: controls the RS-422 driver enable. Implements
//!   sync pulse → silence → collision check → frame transmit, all in
//!   PIO hardware.
//! - **SM2 (RX Edge Timer)**: measures inter-edge timing on the RX input,
//!   pushes interval counts to FIFO, drives carrier-sense output.
//!
//! SM0 + SM1 share PIO0 (18 instructions). SM2 runs on PIO1 (18 instructions).
//!
//! ## Pin assignments
//!
//! - **GP2** — TX data (PIO0 SM0 OUT pin)
//! - **GP3** — TX enable / driver enable (PIO0 SM1 SET pin)
//! - **GP4** — RX data (PIO1 SM2 JMP PIN)
//! - **GP5** — Carrier sense (PIO1 SM2 SET pin, PIO0 SM1 JMP PIN)

pub mod phy;
pub mod rx;
pub mod tx;
