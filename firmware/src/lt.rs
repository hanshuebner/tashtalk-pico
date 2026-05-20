//! LocalTalk PHY abstraction.
//!
//! The eventual TX/RX implementations live in `pio_tx` and `pio_rx`. This
//! module defines the trait surface the rest of the firmware programs
//! against so the autonomous control-frame handler (ENQ→ACK, RTS→CTS,
//! CTS-data pairing) and the CSMA/CA state machine can be written without
//! waiting for the PIO programs to be finished.

/// Status of a frame received from the LocalTalk bus.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RxStatus {
    Good,
    BadCrc,
    FramingError,
    Aborted,
}

/// One end-to-end frame received from the bus.
///
/// Stored as a fixed-capacity slice; the producer is responsible for
/// observing LocalTalk's 605-byte maximum.
#[derive(Debug)]
pub struct RxFrame<'a> {
    pub bytes: &'a [u8],
    pub status: RxStatus,
}
