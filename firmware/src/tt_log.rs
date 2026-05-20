//! Cross-task log channel that ends up on TT_DEBUG.
//!
//! `defmt-rtt` requires an SWD probe to see anything, which is unhelpful
//! during the no-board bring-up. This module provides a simple
//! `tt_log!(fmt, args...)` macro that pushes a formatted line onto an
//! `embassy_sync::Channel`. The `debug` task drains the channel and
//! writes each message out to the TT_DEBUG CDC interface.
//!
//! On overflow the message is dropped — we never block the producer.
//! This is exactly the behaviour you want for diagnostic logging: a slow
//! host terminal must not be able to stall LocalTalk operations.

use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Channel;
use heapless::String;

/// Maximum log line length (bytes).
pub const LINE_LEN: usize = 128;

/// Capacity of the channel — how many lines can be queued before the next
/// `tt_log!()` starts dropping messages.
pub const CHANNEL_DEPTH: usize = 32;

pub type LogLine = String<LINE_LEN>;

pub static CHANNEL: Channel<ThreadModeRawMutex, LogLine, CHANNEL_DEPTH> = Channel::new();

/// Format and enqueue a log message. Drops the message if the channel
/// is full. Never blocks.
#[macro_export]
macro_rules! tt_log {
    ($($t:tt)*) => {{
        let mut s: $crate::tt_log::LogLine = ::heapless::String::new();
        // Truncate silently if the formatted text overflows — the user can
        // still read the prefix, and that's more useful than a panic.
        let _ = ::core::fmt::Write::write_fmt(&mut s, ::core::format_args!($($t)*));
        let _ = $crate::tt_log::CHANNEL.try_send(s);
    }};
}
