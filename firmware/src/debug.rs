//! Debug CDC task — TT_DEBUG.
//!
//! Drains the `tt_log` channel and emits each message to the host as a
//! line. Single-letter debug commands are processed inline. Commands
//! that take arguments (t, n, N, d) accumulate a line buffer until CR.

use core::sync::atomic::{AtomicBool, Ordering};

use crate::core1::SharedState;
use crate::tt_log::{LogLine, CHANNEL as LOG_CHANNEL};
use crate::usb_dev::Cdc;
use defmt::{info, warn};
use embassy_futures::select::{select, Either};
use embassy_usb::driver::EndpointError;
use tashtalk_proto::crc::lt_crc;
use tashtalk_proto::nodemap::NodeBitmap;

// ── RX monitor and frame log ──────────────────────────────────────

/// When true, every good-CRC received frame is printed as `RX:hex\r\n`.
pub static MONITOR_ENABLED: AtomicBool = AtomicBool::new(false);

const RX_LOG_SLOTS: usize = 16;
const RX_LOG_BYTES: usize = 64; // first N bytes of each frame stored
static mut RX_LOG: [[u8; RX_LOG_BYTES]; RX_LOG_SLOTS] = [[0u8; RX_LOG_BYTES]; RX_LOG_SLOTS];
static mut RX_LOG_LENS: [u16; RX_LOG_SLOTS] = [0u16; RX_LOG_SLOTS];
static mut RX_LOG_WRITE: usize = 0;
static mut RX_LOG_COUNT: usize = 0;

/// Called from rx_bridge_task for every good-CRC frame.
pub fn log_rx_frame(frame: &[u8]) {
    let n = frame.len().min(RX_LOG_BYTES);
    unsafe {
        let idx = RX_LOG_WRITE;
        RX_LOG[idx][..n].copy_from_slice(&frame[..n]);
        RX_LOG_LENS[idx] = frame.len() as u16;
        RX_LOG_WRITE = (idx + 1) % RX_LOG_SLOTS;
        if RX_LOG_COUNT < RX_LOG_SLOTS {
            RX_LOG_COUNT += 1;
        }
    }

    if MONITOR_ENABLED.load(Ordering::Relaxed) {
        let mut s: LogLine = heapless::String::new();
        let ms = embassy_time::Instant::now().as_millis();
        let _ = core::fmt::Write::write_fmt(
            &mut s, core::format_args!("[{:06}] RX:", ms));
        for &b in &frame[..frame.len().min(30)] {
            let _ = core::fmt::Write::write_fmt(
                &mut s, core::format_args!("{:02X}", b));
        }
        if frame.len() > 30 {
            let _ = core::fmt::Write::write_str(&mut s, "..");
        }
        let _ = crate::tt_log::CHANNEL.try_send(s);
    }
}

// ── Command state machine ─────────────────────────────────────────

const CMD_BUF_SIZE: usize = 1280;

enum CmdState {
    Idle,
    /// Accumulating arguments for a command. `cmd` is the command letter.
    Accumulating { cmd: u8, pos: usize },
}

static mut CMD_BUF: [u8; CMD_BUF_SIZE] = [0u8; CMD_BUF_SIZE];
static mut CMD_STATE: CmdState = CmdState::Idle;

// ── Node bitmap (local copy on core 0 for debug commands) ─────────

static mut LOCAL_NODES: NodeBitmap = NodeBitmap::empty();

// ── Constants ─────────────────────────────────────────────────────

const BUILD_ID: &str = env!("BUILD_ID");

const HELP: &[u8] = b"\r\n\
commands:\r\n\
  t AABB..  TX raw hex frame (incl CRC) via CSMA\r\n\
  d XX HEX  directed TX to node XX with payload\r\n\
  n XX      set node ID XX in bitmap\r\n\
  N XX      clear node ID XX from bitmap\r\n\
  m         toggle RX monitor (prints RX:hex lines)\r\n\
  c         clear all counters\r\n\
  f         dump buffered RX frames\r\n\
  i         print USB serial number\r\n\
  s         show status/counters\r\n\
  p         TX known test frame via CSMA\r\n\
  b         reboot to BOOTSEL\r\n\
  ?         this help\r\n\
\r\n";

fn format_banner() -> heapless::String<256> {
    use core::fmt::Write;
    let mut s = heapless::String::<256>::new();
    let serial = crate::usb_dev::serial();
    let ms = embassy_time::Instant::now().as_millis();
    let _ = write!(s, "\r\nTASHTALK-PICO {} serial={} uptime={}ms\r\n", BUILD_ID, serial, ms);
    let _ = write!(s, "commands: t n N d m c f i s p b ?\r\n");
    s
}

// ── Task entry ────────────────────────────────────────────────────

#[embassy_executor::task]
pub async fn run(
    mut cdc: Cdc,
    shared: &'static SharedState,
) -> ! {
    // Wait until core 1 is alive before accepting any commands.
    while shared.flags.load(Ordering::Relaxed) & crate::core1::CORE1_ALIVE == 0 {
        embassy_time::Timer::after_millis(1).await;
    }

    let mut buf = [0u8; 64];
    loop {
        cdc.wait_connection().await;
        info!("TT_DEBUG: host connected");
        unsafe { CMD_STATE = CmdState::Idle; }

        let banner = format_banner();
        let _ = write_chunked(&mut cdc, banner.as_bytes()).await;
        let _ = write_chunked(&mut cdc, b"READY\r\n").await;

        loop {
            match select(cdc.read_packet(&mut buf), LOG_CHANNEL.receive()).await {
                Either::First(Ok(n)) => {
                    if let Err(e) = handle_input(&mut cdc, &buf[..n], shared).await {
                        warn!("TT_DEBUG write failed: {}", classify(e));
                        break;
                    }
                }
                Either::First(Err(e)) => {
                    warn!("TT_DEBUG read failed: {}", classify(e));
                    break;
                }
                Either::Second(line) => {
                    if let Err(e) = write_log_line(&mut cdc, &line).await {
                        warn!("TT_DEBUG log write failed: {}", classify(e));
                        break;
                    }
                }
            }
        }
        info!("TT_DEBUG: host disconnected");
    }
}

// ── Input handling ────────────────────────────────────────────────

async fn handle_input(
    cdc: &mut Cdc,
    data: &[u8],
    shared: &'static SharedState,
) -> Result<(), EndpointError> {
    for &b in data {
        let state = unsafe { &mut CMD_STATE };
        match state {
            CmdState::Idle => {
                match b {
                    // Commands that take arguments — start accumulating
                    b't' | b'T' | b'n' | b'd' => {
                        unsafe {
                            CMD_STATE = CmdState::Accumulating { cmd: b, pos: 0 };
                        }
                        write_chunked(cdc, core::slice::from_ref(&b)).await?;
                    }
                    b'N' => {
                        unsafe {
                            CMD_STATE = CmdState::Accumulating { cmd: b'N', pos: 0 };
                        }
                        write_chunked(cdc, b"N").await?;
                    }
                    // Immediate commands
                    b'p' | b'P' => {
                        write_chunked(cdc, b"\r\nOK TX\r\n").await?;
                        let frame = debug_test_frame();
                        crate::host::submit_tx_from_debug(shared, &frame).await;
                        write_chunked(cdc, b"OK\r\n").await?;
                    }
                    b's' | b'S' => {
                        write_chunked(cdc, b"\r\n").await?;
                        let status = pio_status(shared);
                        write_chunked(cdc, status.as_bytes()).await?;
                        write_chunked(cdc, b"---\r\n").await?;
                    }
                    b'c' | b'C' => {
                        crate::core1::clear_counters();
                        write_chunked(cdc, b"\r\nCOUNTERS CLEARED\r\n").await?;
                    }
                    b'm' | b'M' => {
                        let prev = MONITOR_ENABLED.load(Ordering::Relaxed);
                        MONITOR_ENABLED.store(!prev, Ordering::Relaxed);
                        if prev {
                            write_chunked(cdc, b"\r\nMONITOR OFF\r\n").await?;
                        } else {
                            write_chunked(cdc, b"\r\nMONITOR ON\r\n").await?;
                        }
                    }
                    b'f' | b'F' => {
                        write_chunked(cdc, b"\r\n").await?;
                        cmd_dump_rx(cdc).await?;
                    }
                    b'i' | b'I' => {
                        let serial = crate::usb_dev::serial();
                        write_chunked(cdc, b"\r\nID:").await?;
                        write_chunked(cdc, serial.as_bytes()).await?;
                        write_chunked(cdc, b"\r\n").await?;
                    }
                    b'b' | b'B' => {
                        write_chunked(cdc, b"\r\nBOOTSEL\r\n").await?;
                        embassy_time::Timer::after_millis(50).await;
                        embassy_rp::rom_data::reset_to_usb_boot(0, 0);
                    }
                    b'?' => {
                        write_chunked(cdc, HELP).await?;
                    }
                    b'\r' | b'\n' => {
                        write_chunked(cdc, b"\r\n").await?;
                    }
                    _ => {
                        write_chunked(cdc, core::slice::from_ref(&b)).await?;
                    }
                }
            }
            CmdState::Accumulating { cmd, pos } => {
                if b == b'\r' || b == b'\n' {
                    write_chunked(cdc, b"\r\n").await?;
                    let cmd_byte = *cmd;
                    let len = *pos;
                    unsafe { CMD_STATE = CmdState::Idle; }
                    let line = unsafe { &CMD_BUF[..len] };
                    dispatch_cmd(cdc, shared, cmd_byte, line).await?;
                } else if *pos < CMD_BUF_SIZE {
                    unsafe { CMD_BUF[*pos] = b; }
                    *pos += 1;
                    write_chunked(cdc, core::slice::from_ref(&b)).await?;
                }
                // else: overflow, silently drop extra bytes
            }
        }
    }
    Ok(())
}

/// Dispatch an accumulated command.
async fn dispatch_cmd(
    cdc: &mut Cdc,
    shared: &'static SharedState,
    cmd: u8,
    args: &[u8],
) -> Result<(), EndpointError> {
    // Strip leading spaces
    let args = strip_leading(args, b' ');

    match cmd {
        b't' | b'T' => cmd_tx_raw(cdc, shared, args).await,
        b'n' => cmd_set_node(cdc, shared, args, true).await,
        b'N' => cmd_set_node(cdc, shared, args, false).await,
        b'd' => cmd_directed(cdc, shared, args).await,
        _ => {
            write_chunked(cdc, b"ERR:unknown\r\n").await
        }
    }
}

// ── Command implementations ───────────────────────────────────────

/// `t AABB..` — TX raw hex frame via CSMA
async fn cmd_tx_raw(
    cdc: &mut Cdc,
    shared: &'static SharedState,
    hex: &[u8],
) -> Result<(), EndpointError> {
    let Some(()) = parse_hex_to_buf(hex) else {
        return write_chunked(cdc, b"ERR:bad hex\r\n").await;
    };
    let len = frame_len();
    if len < 5 || len > 605 {
        return write_chunked(cdc, b"ERR:bad len\r\n").await;
    }
    let frame_slice = frame_slice_ref();
    crate::host::submit_tx_from_debug(shared, frame_slice).await;
    write_chunked(cdc, b"OK\r\n").await
}

/// `n XX` / `N XX` — set or clear node ID in bitmap
async fn cmd_set_node(
    cdc: &mut Cdc,
    shared: &'static SharedState,
    hex: &[u8],
    set: bool,
) -> Result<(), EndpointError> {
    let Some(node_id) = parse_hex_byte(hex) else {
        return write_chunked(cdc, b"ERR:bad hex\r\n").await;
    };
    unsafe {
        if set {
            LOCAL_NODES.set(node_id);
        } else {
            LOCAL_NODES.clear(node_id);
        }
        crate::host::update_nodes(shared, LOCAL_NODES.as_bytes());
    }
    // Wait for core 1 to acknowledge the bitmap update.
    // NODES_UPDATED flag is cleared by core 1 after it copies the bitmap.
    let deadline = embassy_time::Instant::now() + embassy_time::Duration::from_millis(100);
    while shared.flags.load(Ordering::Relaxed) & crate::core1::NODES_UPDATED != 0 {
        if embassy_time::Instant::now() >= deadline { break; }
        embassy_time::Timer::after_micros(10).await;
    }
    if set {
        write_chunked(cdc, b"NODE SET\r\n").await
    } else {
        write_chunked(cdc, b"NODE CLR\r\n").await
    }
}

/// `d XX AABB..` — directed data TX (builds frame with CRC, uses RTS/CTS)
async fn cmd_directed(
    cdc: &mut Cdc,
    shared: &'static SharedState,
    args: &[u8],
) -> Result<(), EndpointError> {
    // Parse: first 2 hex chars = dest node, then space, then payload hex
    if args.len() < 3 {
        return write_chunked(cdc, b"ERR:need dest+payload\r\n").await;
    }
    let Some(dest) = parse_hex_byte(&args[..2]) else {
        return write_chunked(cdc, b"ERR:bad dest hex\r\n").await;
    };
    let payload_hex = strip_leading(&args[2..], b' ');
    let Some(_) = parse_hex_to_buf(payload_hex) else {
        return write_chunked(cdc, b"ERR:bad payload hex\r\n").await;
    };
    let payload_len = frame_len();

    // Find our source node from bitmap
    let src = unsafe { LOCAL_NODES.iter().next().unwrap_or(1) };

    // Build LLAP data frame: [dest, src, type=0x01, length_hi, length_lo, payload..., CRC]
    // DDP Short header: type=0x01, then 2 bytes length (of DDP data including header)
    let mut frame_buf = [0u8; 610];
    frame_buf[0] = dest;
    frame_buf[1] = src;
    frame_buf[2] = 0x01; // DDP short
    // Copy payload after the 3-byte LLAP header
    let payload_slice = frame_slice_ref();
    let total_data = 3 + payload_len; // LLAP header + payload
    frame_buf[3..3 + payload_len].copy_from_slice(&payload_slice[..payload_len]);
    // Compute and append CRC
    let crc = lt_crc(&frame_buf[..total_data]);
    frame_buf[total_data] = crc[0];
    frame_buf[total_data + 1] = crc[1];
    let total = total_data + 2;

    crate::host::submit_tx_from_debug(shared, &frame_buf[..total]).await;

    // Check result
    let flags = shared.flags.load(Ordering::Relaxed);
    if flags & crate::core1::TX_RESULT_FAIL != 0 {
        write_chunked(cdc, b"DIR TIMEOUT\r\n").await
    } else {
        write_chunked(cdc, b"DIR OK\r\n").await
    }
}

/// `f` — dump buffered RX frames
async fn cmd_dump_rx(cdc: &mut Cdc) -> Result<(), EndpointError> {
    let count = unsafe { RX_LOG_COUNT };
    let write_idx = unsafe { RX_LOG_WRITE };

    if count == 0 {
        return write_chunked(cdc, b"END\r\n").await;
    }

    let start = if count < RX_LOG_SLOTS {
        0
    } else {
        write_idx // oldest is at write_idx (wrapped)
    };

    let mut hex_buf = [0u8; 132]; // "RX:" + 64 bytes * 2 + "\r\n"
    for i in 0..count {
        let idx = (start + i) % RX_LOG_SLOTS;
        let len = unsafe { RX_LOG_LENS[idx] } as usize;
        let stored = len.min(RX_LOG_BYTES);
        let frame = unsafe { &RX_LOG[idx][..stored] };

        let mut pos = 0;
        hex_buf[pos] = b'R'; pos += 1;
        hex_buf[pos] = b'X'; pos += 1;
        hex_buf[pos] = b':'; pos += 1;
        for &b in frame {
            hex_buf[pos] = hex_char(b >> 4); pos += 1;
            hex_buf[pos] = hex_char(b & 0xF); pos += 1;
        }
        hex_buf[pos] = b'\r'; pos += 1;
        hex_buf[pos] = b'\n'; pos += 1;
        write_chunked(cdc, &hex_buf[..pos]).await?;
    }

    // Clear buffer
    unsafe {
        RX_LOG_COUNT = 0;
        RX_LOG_WRITE = 0;
    }

    write_chunked(cdc, b"END\r\n").await
}

// ── Helpers ───────────────────────────────────────────────────────

fn debug_test_frame() -> [u8; 10] {
    let mut f = [0u8; 10];
    f[0] = 0x42; f[1] = 0x01; f[2] = 0x01;
    f[3] = 0x00; f[4] = 0x05; f[5] = 0xDE;
    f[6] = 0x00; f[7] = 0xAD;
    let crc = lt_crc(&f[..8]);
    f[8] = crc[0]; f[9] = crc[1];
    f
}

fn hex_char(nibble: u8) -> u8 {
    if nibble < 10 { b'0' + nibble } else { b'A' + nibble - 10 }
}

fn hex_val(c: u8) -> Option<u8> {
    match c {
        b'0'..=b'9' => Some(c - b'0'),
        b'a'..=b'f' => Some(c - b'a' + 10),
        b'A'..=b'F' => Some(c - b'A' + 10),
        _ => None,
    }
}

/// Parse hex string into the shared FRAME_BUF. Returns Some(()) on success.
/// Use frame_len() and frame_slice_ref() to access the result.
static mut FRAME_BUF: [u8; 610] = [0u8; 610];
static mut FRAME_BUF_LEN: usize = 0;

fn parse_hex_to_buf(hex: &[u8]) -> Option<()> {
    // Skip spaces
    let hex = strip_leading(hex, b' ');
    if hex.len() % 2 != 0 { return None; }
    let byte_count = hex.len() / 2;
    if byte_count > 610 { return None; }
    unsafe {
        FRAME_BUF_LEN = 0;
        for i in 0..byte_count {
            let hi = hex_val(hex[i * 2])?;
            let lo = hex_val(hex[i * 2 + 1])?;
            FRAME_BUF[i] = (hi << 4) | lo;
        }
        FRAME_BUF_LEN = byte_count;
    }
    Some(())
}

fn frame_len() -> usize {
    unsafe { FRAME_BUF_LEN }
}

fn frame_slice_ref() -> &'static [u8] {
    unsafe { &FRAME_BUF[..FRAME_BUF_LEN] }
}

fn parse_hex_byte(hex: &[u8]) -> Option<u8> {
    let hex = strip_leading(hex, b' ');
    if hex.len() < 2 { return None; }
    let hi = hex_val(hex[0])?;
    let lo = hex_val(hex[1])?;
    Some((hi << 4) | lo)
}

fn strip_leading(data: &[u8], ch: u8) -> &[u8] {
    let mut i = 0;
    while i < data.len() && data[i] == ch { i += 1; }
    &data[i..]
}

async fn write_chunked(cdc: &mut Cdc, data: &[u8]) -> Result<(), EndpointError> {
    for chunk in data.chunks(64) {
        match embassy_futures::select::select(
            cdc.write_packet(chunk),
            embassy_time::Timer::after_millis(100),
        )
        .await
        {
            embassy_futures::select::Either::First(r) => r?,
            embassy_futures::select::Either::Second(()) => {
                return Err(EndpointError::Disabled);
            }
        }
    }
    Ok(())
}

async fn write_log_line(cdc: &mut Cdc, line: &LogLine) -> Result<(), EndpointError> {
    write_chunked(cdc, line.as_bytes()).await?;
    write_chunked(cdc, b"\r\n").await?;
    Ok(())
}

fn pio_status(shared: &SharedState) -> heapless::String<768> {
    use core::fmt::Write;
    use embassy_rp::pac;

    let mut s = heapless::String::<768>::new();
    let ms = embassy_time::Instant::now().as_millis();
    let _ = write!(s, "[{:06}] status\r\n", ms);

    let pio1_ctrl = pac::PIO1.ctrl().read();
    let gpio_in = pac::SIO.gpio_in(0).read();
    let flags = shared.flags.load(Ordering::Relaxed);
    let (_, waddr, ridx, en, remain) = crate::core1::rx_dma_diag();
    let base = unsafe { crate::core1::RX_RING.0.as_ptr() } as u32;
    let widx = (waddr.wrapping_sub(base)) / 4;

    let _ = write!(s, "PIO1: sm_en={:04b} GP4(RX)={} GP5(CS)={}\r\n",
        pio1_ctrl.sm_enable(), (gpio_in >> 4) & 1, (gpio_in >> 5) & 1);
    let _ = write!(s, "Flags: 0x{:02x} alive={} rxrdy={} txsub={}\r\n",
        flags, (flags >> 5) & 1, (flags >> 3) & 1, flags & 1);
    let _ = write!(s, "DMA: en={} widx={} ridx={} remain={}\r\n", en, widx, ridx, remain);

    let good = unsafe { crate::core1::GOOD_CRC_COUNT };
    let bad = unsafe { crate::core1::BAD_CRC_COUNT };
    let ar = unsafe { crate::core1::AUTOREPLY_COUNT };
    let overruns = unsafe { crate::core1::RING_OVERRUN_COUNT };
    let dir_att = unsafe { crate::core1::DIRECTED_TX_ATTEMPTS };
    let dir_cts = unsafe { crate::core1::DIRECTED_TX_CTS_OK };
    let dir_tmo = unsafe { crate::core1::DIRECTED_TX_TIMEOUT };
    let dir_rx = unsafe { crate::core1::DIRECTED_RX_COUNT };
    let echo_skip = unsafe { crate::core1::ECHO_SKIP_COUNT };
    let bad_real = unsafe { crate::core1::BAD_REAL_COUNT };

    let _ = write!(s, "Recv: good={} bad={} bad_real={} ar={} echo={} overrun={}\r\n",
        good, bad, bad_real, ar, echo_skip, overruns);
    let _ = write!(s, "Send: dir_att={} cts_ok={} tmo={} dir_rx={}\r\n",
        dir_att, dir_cts, dir_tmo, dir_rx);

    let cts_start = unsafe { crate::core1::LAST_CTS_WAIT_START };
    let cts_end = unsafe { crate::core1::LAST_CTS_WAIT_END };
    let cts_entries = unsafe { crate::core1::LAST_CTS_ENTRIES };
    if cts_start > 0 {
        let _ = write!(s, "CTS wait: {}us, {} entries\r\n",
            cts_end.saturating_sub(cts_start), cts_entries);
    }

    s
}

fn classify(e: EndpointError) -> defmt::Str {
    match e {
        EndpointError::BufferOverflow => defmt::intern!("buffer overflow"),
        EndpointError::Disabled => defmt::intern!("endpoint disabled"),
    }
}
