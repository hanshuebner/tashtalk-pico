//! TashTalk-Pico firmware entry point.
//!
//! Dual-core architecture:
//! - **Core 0** (Embassy async): USB CDC, host protocol, debug shell.
//! - **Core 1** (bare-metal loop): PIO, DMA, all LocalTalk wire interaction.
//!
//! ## Pin assignments
//!
//! - **GP2** — TX data (PIO0 SM0 OUT pin → MAX-485 DI)
//! - **GP3** — TX enable / driver enable (software GPIO → MAX-485 DE)
//! - **GP4** — RX data (PIO1 SM0 JMP PIN ← MAX-485 RO)
//! - **GP5** — Carrier sense (PIO1 SM0 SET pin, internal)
//! - **GP6** — Debug output 0 (core 1 timing, AD2 probe)

#![no_std]
#![no_main]
#![allow(static_mut_refs)] // embassy multicore API requires &mut Stack

use core::sync::atomic::{compiler_fence, Ordering};

use defmt::info;
use embassy_executor::Executor;
use embassy_rp::bind_interrupts;
use embassy_rp::multicore::Stack;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver, InterruptHandler};
use embassy_time::Timer;
use defmt_rtt as _;
use static_cell::StaticCell;

use tashtalk_proto::backoff::BackoffState;
use tashtalk_proto::codec::FrameStatus;
use tashtalk_proto::nodemap::NodeBitmap;

use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;

mod core1;
mod localtalk;
mod host;
mod debug;
#[macro_use]
mod tt_log;
mod usb_dev;

pub type SharedNodes = Mutex<ThreadModeRawMutex, NodeBitmap>;
pub type SharedBackoff = Mutex<ThreadModeRawMutex, BackoffState>;

/// No-init RAM buffer for panic messages (survives soft reset).
#[link_section = ".uninit.PANIC_BUF"]
static mut PANIC_BUF: core::mem::MaybeUninit<[u8; 512]> =
    core::mem::MaybeUninit::uninit();

const PANIC_MARKER: &[u8] = b"\xDEPANIC_BUF\xDE ";

#[panic_handler]
fn panic_handler(info: &core::panic::PanicInfo) -> ! {
    use core::fmt::Write;
    let buf = unsafe { &mut *core::ptr::addr_of_mut!(PANIC_BUF).cast::<[u8; 512]>() };
    for b in buf.iter_mut() { *b = 0; }
    buf[..PANIC_MARKER.len()].copy_from_slice(PANIC_MARKER);
    let mut w = SliceWriter { buf, pos: PANIC_MARKER.len() };
    if let Some(loc) = info.location() {
        let _ = write!(&mut w, "{}:{}:{} ", loc.file(), loc.line(), loc.column());
    } else {
        let _ = write!(&mut w, "<no location> ");
    }
    let _ = write!(&mut w, "msg=");
    let _ = write!(&mut w, "{}", info);
    embassy_rp::rom_data::reset_to_usb_boot(0, 0);
    loop { cortex_m::asm::nop(); }
}

struct SliceWriter<'a> { buf: &'a mut [u8], pos: usize }

impl core::fmt::Write for SliceWriter<'_> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        for b in s.bytes() {
            if self.pos < self.buf.len() - 1 {
                self.buf[self.pos] = b;
                self.pos += 1;
            }
        }
        Ok(())
    }
}

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

// ── Core 1 stack ────────────────────────────────────────────────────
// Currently 64KB. It is almost certainly oversized (all large core-1 buffers
// are static), but it must be measured before shrinking — an undersized stack
// faults silently. `paint_core1_stack` fills it with a sentinel at boot and
// the `stack` debug command reports the high-water mark. See
// `docs/debug-cli-plan.md` and the RAM notes in `docs/ltoudp-design.md`.
const CORE1_STACK_SIZE: usize = 8192;
static mut CORE1_STACK: Stack<CORE1_STACK_SIZE> = Stack::new();

/// Sentinel word used to paint the core-1 stack for high-water measurement.
const CORE1_STACK_PAINT: u32 = 0xC1AC_57AC;

/// Fill the core-1 stack with the sentinel word. MUST be called before
/// `spawn_core1`, while core 1 is not yet touching the stack.
fn paint_core1_stack() {
    let words = CORE1_STACK_SIZE / 4;
    let base = core::ptr::addr_of_mut!(CORE1_STACK) as *mut u32;
    for i in 0..words {
        unsafe { base.add(i).write_volatile(CORE1_STACK_PAINT) };
    }
}

/// `(used_bytes, total_bytes)` for the core-1 stack. The stack grows downward
/// from the top, so untouched sentinels accumulate at the low (deep) end;
/// counting them gives the headroom. Reads run concurrently with core 1 — a
/// benign race that is fine for a diagnostic.
pub fn core1_stack_stats() -> (usize, usize) {
    let words = CORE1_STACK_SIZE / 4;
    let base = core::ptr::addr_of!(CORE1_STACK) as *const u32;
    let mut unused = 0;
    while unused < words && unsafe { base.add(unused).read_volatile() } == CORE1_STACK_PAINT {
        unused += 1;
    }
    ((words - unused) * 4, words * 4)
}

// ── Shared state (inter-core) ───────────────────────────────────────
static SHARED: core1::SharedState = core1::SharedState::new();

// ── Executor ────────────────────────────────────────────────────────
static EXECUTOR0: StaticCell<Executor> = StaticCell::new();

#[cortex_m_rt::entry]
fn main() -> ! {
    let p = embassy_rp::init(Default::default());
    info!("tashtalk-pico booting (dual-core)");

    // ── LocalTalk PHY (configures PIO + pins, drops handles) ────────
    info!("PHY: init_phy starting");
    localtalk::phy::init_phy(
        p.PIO0,
        localtalk::phy::Pio0Irqs {},
        p.PIO1,
        localtalk::phy::Pio1Irqs {},
        p.PIN_2,
        p.PIN_3,
        p.PIN_4,
        p.PIN_5,
    );
    info!("PHY: init_phy done");

    // ── Spawn core 1 (before starting executor) ─────────────────────
    paint_core1_stack();
    embassy_rp::multicore::spawn_core1(
        p.CORE1,
        unsafe { &mut CORE1_STACK },
        move || core1::core1_main(&SHARED),
    );
    info!("Core 1 spawned");

    // ── USB ──────────────────────────────────────────────────────────
    let driver = Driver::new(p.USB, Irqs);
    let (usb_run, data_cdc, debug_cdc) = usb_dev::build(driver, p.FLASH);

    // ── Start Embassy executor on core 0 ────────────────────────────
    let executor0 = EXECUTOR0.init(Executor::new());
    executor0.run(|spawner| {
        spawner.spawn(usb_run_task(usb_run).unwrap());
        spawner.spawn(debug::run(debug_cdc, &SHARED).unwrap());
        spawner.spawn(host::run(data_cdc, &SHARED).unwrap());
        spawner.spawn(rx_bridge_task(&SHARED).unwrap());
    });
}

#[embassy_executor::task]
async fn usb_run_task(mut usb: usb_dev::UsbDevice) {
    usb.run().await;
}

/// Bridge: polls core 1's rx_slot and forwards frames to the host task.
#[embassy_executor::task]
async fn rx_bridge_task(shared: &'static core1::SharedState) {
    loop {
        let flags = shared.flags.load(Ordering::Relaxed);
        if flags & core1::RX_READY != 0 {
            compiler_fence(Ordering::SeqCst);
            // SAFETY: RX_READY set → core 1 is not writing.
            let slot = unsafe { &*shared.rx_slot.get() };
            let len = slot.len as usize;
            let status = match slot.status {
                1 => FrameStatus::BadCrc,
                _ => FrameStatus::Good,
            };
            host::try_forward_inbound(&slot.buf[..len], status);
            let crc_ok = matches!(status, FrameStatus::Good);
            debug::log_rx_frame(&slot.buf[..len], crc_ok);
            shared.flags.fetch_and(!core1::RX_READY, Ordering::Release);
        } else {
            Timer::after_micros(50).await;
        }
    }
}
