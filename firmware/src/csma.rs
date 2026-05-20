//! CSMA/CA defer-and-backoff for outbound transmissions.
//!
//! Glue between the pure backoff state machine in
//! `tashtalk_proto::backoff` and the firmware's TX path:
//!
//! - The [`WIRE_LAST_ACTIVITY_US`] atomic is updated by `lt_rx_task`
//!   every time the FM-0 decoder emits an event (byte, flag, or
//!   abort). The atomic holds a microsecond timestamp from
//!   `embassy_time::Instant`.
//! - [`defer_and_backoff`] is called by `transmit_on_phy` *before*
//!   driving the wire. It:
//!     1. Advances the backoff state machine for the new frame.
//!     2. Computes a per-attempt delay = IDG + (entropy & local_mask).
//!     3. Waits until `now - last_activity ≥ delay`.
//!
//! Per the PIC reference: `SetTimer2`/`PrepForNextFrame` /
//! `WaitForSilence` at `one-chip.asm:439, 649, 679`.

// thumbv6m-none-eabi (Cortex-M0+) doesn't have native 64-bit atomics.
// We use AtomicU32 microseconds, which wraps every 4.29 billion µs
// (~71 minutes). For "wire was busy recently" checks this is more
// than enough — anything beyond a few hundred microseconds is treated
// as "idle". The wrap is handled with wrapping_sub which is correct
// as long as the gap is less than ~36 minutes (half the wrap period).
use core::sync::atomic::{AtomicU32, AtomicU8, Ordering};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Timer};
use tashtalk_proto::backoff::BackoffState;

/// Shared backoff state, locked during each TX.
pub type SharedBackoff = Mutex<ThreadModeRawMutex, BackoffState>;

/// Microsecond timestamp of the most recent decoder event. Updated by
/// `lt_rx_task` on every Byte / Flag / Abort. Read by
/// [`defer_and_backoff`].
pub static WIRE_LAST_ACTIVITY_US: AtomicU32 = AtomicU32::new(0);

/// While the host transmitter is waiting for a CTS in response to its
/// own RTS, these atomics describe the pairing it expects: the CTS we
/// want carries `dest=PENDING_RTS_OUR_SRC, src=PENDING_RTS_THEIR_DEST`.
///
/// Loaded by the RX task on every decoded CTS. Stored by the TX path
/// just before sending the RTS. A sentinel value of `0xFF` for both
/// (an illegal LocalTalk node ID) means "no RTS pending".
pub static PENDING_RTS_OUR_SRC: AtomicU8 = AtomicU8::new(0xFF);
pub static PENDING_RTS_THEIR_DEST: AtomicU8 = AtomicU8::new(0xFF);

/// Fires when the RX task decodes a CTS frame that matches the
/// pending-RTS pairing above.
pub static CTS_RECEIVED: Signal<ThreadModeRawMutex, ()> = Signal::new();

/// Clear the pending RTS state.
#[inline(always)]
pub fn clear_pending_rts() {
    PENDING_RTS_OUR_SRC.store(0xFF, Ordering::Relaxed);
    PENDING_RTS_THEIR_DEST.store(0xFF, Ordering::Relaxed);
}

/// Arm the pending-RTS pairing. Call this *before* transmitting the
/// RTS so that a very fast responder can't beat us to the signal.
#[inline]
pub fn set_pending_rts(our_src: u8, their_dest: u8) {
    CTS_RECEIVED.reset();
    PENDING_RTS_OUR_SRC.store(our_src, Ordering::Relaxed);
    PENDING_RTS_THEIR_DEST.store(their_dest, Ordering::Relaxed);
}

/// If a CTS we just decoded matches the pending RTS pairing, signal
/// the TX task and clear the pending state. Called by `lt_rx_task`.
pub fn maybe_signal_cts(frame: &[u8]) {
    use tashtalk_proto::llap::cts_matches_pending;
    let our_src = PENDING_RTS_OUR_SRC.load(Ordering::Relaxed);
    let their_dest = PENDING_RTS_THEIR_DEST.load(Ordering::Relaxed);
    if our_src == 0xFF {
        return;
    }
    if cts_matches_pending(frame, our_src, their_dest) {
        clear_pending_rts();
        CTS_RECEIVED.signal(());
    }
}

/// Mark the wire as active *now*. Called by the RX task whenever the
/// decoder emits any event.
#[inline(always)]
pub fn mark_wire_active() {
    WIRE_LAST_ACTIVITY_US.store(Instant::now().as_micros() as u32, Ordering::Relaxed);
}

/// Wait for the wire to be quiet for IDG + random-backoff before
/// allowing the caller to drive a frame onto it.
///
/// Updates the shared [`BackoffState`] for a new frame attempt and
/// uses its computed delay. Returns once the wait is complete.
///
/// This is the firmware-side analogue of `WaitForSilence`
/// (`one-chip.asm:649`) for broadcast / control frames. Per-frame
/// collision detection (RTS-without-CTS) is not yet implemented.
pub async fn defer_and_backoff(backoff: &'static SharedBackoff) {
    // Step 1: advance the backoff state machine.
    let delay_units = {
        let mut g = backoff.lock().await;
        g.prep_for_next_frame();
        let entropy = (Instant::now().as_micros() as u8) ^ ((Instant::now().as_ticks() >> 8) as u8);
        g.next_backoff_units(entropy)
    };

    // delay_units are in 100 µs steps (matching the PIC's Timer2 ÷ 100
    // µs prescale). So target delay in µs:
    let required_idle_us: u32 = (delay_units as u32) * 100;

    // Step 2: keep checking elapsed-since-last-activity until we hit
    // the required idle period. Loop because new activity may extend
    // the wait — the wire really must be quiet for the FULL required
    // window, not just any consecutive window.
    loop {
        let now = Instant::now().as_micros() as u32;
        let last = WIRE_LAST_ACTIVITY_US.load(Ordering::Relaxed);
        // wrapping_sub correctly handles the AtomicU32 wrap for any
        // gap < 2^31 µs ≈ 35 minutes.
        let idle_for = now.wrapping_sub(last);
        if idle_for >= required_idle_us {
            return;
        }
        let remaining = required_idle_us - idle_for;
        Timer::after(Duration::from_micros(remaining as u64)).await;
    }
}
