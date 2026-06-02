//! LocalTalk PHY — dual-core architecture.
//!
//! PIO state machines and pins are configured in `phy.rs`. All runtime
//! wire interaction (RX decode, TX, CSMA, auto-reply) runs on core 1
//! via `core1.rs`. This module only contains the initialization code.

pub mod phy;
