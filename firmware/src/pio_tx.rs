//! PIO-based FM-0 transmitter for LocalTalk — **stub**.
//!
//! Replaces the PIC firmware's `SendByte` family (lines 1041-1419 of
//! `one-chip.asm`). The PIC issues one FM-0 mid-cell clock edge per
//! instruction-cycle group and squeezes UART servicing into the spare
//! cycles; the RP2040 will instead delegate the entire bit-cell timing to
//! a PIO state machine, freeing the M0+ to handle USB.
//!
//! ## PIO program intent
//!
//! Side-set pin: `LT_DE` (driver enable on the transceiver).
//! `OUT` pin: `LT_DATA`.
//! Clock divider: configured so 1 PIO instruction = ⅛ of a 230.4 kHz bit
//! cell.
//!
//! ```text
//! .program tashtalk_tx
//! .side_set 1
//! .wrap_target
//!     out  x, 1     side 1            ; next bit, DE high
//!     set  pins, 1  side 1            ; first half-cell — always inverts prev
//!     nop           side 1 [3]
//!     jmp  !x, bit0 side 1
//!     nop           side 1 [3]
//!     jmp  next     side 1
//! bit0:
//!     set  pins, 0  side 1 [3]        ; mid-cell toggle for a 0
//! next:
//! .wrap
//! ```
//!
//! Bit-stuffing runs on the M0+ via `tashtalk_proto::bitstuff::stuff()`
//! (which is host-tested). The TX FIFO is fed with the already-stuffed
//! bit stream packed LSB-first into bytes, so the PIO program never needs
//! to count consecutive ones — it just shifts.
//!
//! ## Bring-up gate
//!
//! Validated against PIO RX (see [`pio_rx`](crate::pio_rx)) on the same
//! Pico, with `LT_DATA_TX` wired to `LT_DATA_RX` through a 1 kΩ resistor.
//! See PICO_PORT_PLAN.md §"Bring-up order (no-board)" step 5.

// The implementation is intentionally absent at this point in the bring-up.
// Adding the empty struct here keeps the rest of the firmware compiling
// once it starts referring to `pio_tx::PioTx`.

pub struct PioTx {
    _private: (),
}

impl PioTx {
    pub const fn new() -> Self {
        Self { _private: () }
    }
}

impl Default for PioTx {
    fn default() -> Self {
        Self::new()
    }
}
