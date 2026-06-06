//! Debug CDC task — TT_DEBUG.
//!
//! Drains the `tt_log` channel and emits each message to the host, and runs
//! a line-oriented command interpreter.
//!
//! ## Command model
//!
//! Input is read a line at a time (terminated by CR or LF). A line is
//! tokenised on whitespace into a verb plus arguments. The verb is looked up
//! in [`COMMANDS`] — by full name, unique prefix, or single-letter alias — so
//! adding a command is one [`CmdInfo`] table row plus one `match` arm in
//! [`dispatch`]. `help` is generated from the table.
//!
//! ## Output format
//!
//! Every line is `\r\n`-terminated and starts with a stable tag so a host can
//! demultiplex command replies from async events on the one CDC pipe:
//!
//! - `OK` / `OK k=v ...`  — command succeeded (always the last reply line)
//! - `ERR <code> <msg>`   — command failed (always the last reply line)
//! - `STAT ...`           — one field group of a status report
//! - `RX:` / `RX!:`       — received frame (good / bad CRC)
//! - `#`                  — banner / comment / help (host ignores)
//!
//! A host drives the port by writing a line and reading until it sees an
//! `OK`/`ERR` terminator; `STAT`/`RX:`/`#` lines may also arrive unsolicited.
//!
//! See `docs/debug-cli-plan.md` for the full design, including the planned
//! `Line` transport trait and password-protected telnet front-end that reuse
//! this command core on the Pico W.

use core::sync::atomic::{AtomicBool, Ordering};

use crate::core1::SharedState;
use crate::tt_log::{LogLine, CHANNEL as LOG_CHANNEL};
use crate::usb_dev::Cdc;
use defmt::{info, warn};
use embassy_futures::select::{select, Either};
use embassy_usb::driver::EndpointError;
use heapless::Vec;
use tashtalk_proto::crc::lt_crc;
use tashtalk_proto::nodemap::NodeBitmap;

// ── RX monitor and frame log ──────────────────────────────────────

/// When true, every good-CRC received frame is printed as an `RX:` line.
pub static MONITOR_ENABLED: AtomicBool = AtomicBool::new(false);

/// When true, protocol analyzer mode: all frames (including bad CRC) are
/// forwarded to rx_slot and printed on the debug port.
pub static ANALYZER_MODE: AtomicBool = AtomicBool::new(false);

const RX_LOG_SLOTS: usize = 16;
const RX_LOG_BYTES: usize = 64; // first N bytes of each frame stored
static mut RX_LOG: [[u8; RX_LOG_BYTES]; RX_LOG_SLOTS] = [[0u8; RX_LOG_BYTES]; RX_LOG_SLOTS];
static mut RX_LOG_LENS: [u16; RX_LOG_SLOTS] = [0u16; RX_LOG_SLOTS];
static mut RX_LOG_WRITE: usize = 0;
static mut RX_LOG_COUNT: usize = 0;

/// Called from rx_bridge_task for received frames.
/// `crc_ok` indicates whether the frame passed CRC validation.
pub fn log_rx_frame(frame: &[u8], crc_ok: bool) {
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

    if MONITOR_ENABLED.load(Ordering::Relaxed) || ANALYZER_MODE.load(Ordering::Relaxed) {
        let mut s: LogLine = heapless::String::new();
        let ms = embassy_time::Instant::now().as_millis();
        let tag = if crc_ok { "RX:" } else { "RX!:" };
        let _ = core::fmt::Write::write_fmt(&mut s, core::format_args!("{} t={:06} ", tag, ms));
        for &b in &frame[..frame.len().min(30)] {
            let _ = core::fmt::Write::write_fmt(&mut s, core::format_args!("{:02X}", b));
        }
        if frame.len() > 30 {
            let _ = core::fmt::Write::write_str(&mut s, "..");
        }
        let _ = crate::tt_log::CHANNEL.try_send(s);
    }
}

// ── Command table ─────────────────────────────────────────────────

/// Every interpreter verb. Add a variant here, a [`COMMANDS`] row, and a
/// `match` arm in [`dispatch`] to introduce a command.
#[derive(Copy, Clone, PartialEq, Eq)]
enum Cmd {
    Help,
    Status,
    Clear,
    Monitor,
    Analyzer,
    Frames,
    Id,
    Tx,
    Directed,
    Node,
    TestFrame,
    Stack,
    Wifi,
    Ip,
    Bootsel,
}

struct CmdInfo {
    cmd: Cmd,
    name: &'static str,
    /// Single-letter alias, or 0 for none. (`node` is reached via the legacy
    /// `n`/`N` aliases, handled specially in [`resolve`].)
    alias: u8,
    usage: &'static str,
    summary: &'static str,
}

static COMMANDS: &[CmdInfo] = &[
    CmdInfo { cmd: Cmd::Help, name: "help", alias: b'?', usage: "[cmd]", summary: "list commands / show usage" },
    CmdInfo { cmd: Cmd::Status, name: "status", alias: b's', usage: "", summary: "counters + PIO/DMA state" },
    CmdInfo { cmd: Cmd::Clear, name: "clear", alias: b'c', usage: "", summary: "clear counters" },
    CmdInfo { cmd: Cmd::Monitor, name: "monitor", alias: b'm', usage: "[on|off]", summary: "RX monitor (good-CRC RX: lines)" },
    CmdInfo { cmd: Cmd::Analyzer, name: "analyzer", alias: b'a', usage: "[on|off]", summary: "analyzer (all frames incl bad CRC)" },
    CmdInfo { cmd: Cmd::Frames, name: "frames", alias: b'f', usage: "", summary: "dump buffered RX frames" },
    CmdInfo { cmd: Cmd::Id, name: "id", alias: b'i', usage: "", summary: "USB serial number" },
    CmdInfo { cmd: Cmd::Tx, name: "tx", alias: b't', usage: "<hex>", summary: "TX raw hex frame (incl CRC) via CSMA" },
    CmdInfo { cmd: Cmd::Directed, name: "directed", alias: b'd', usage: "<dest> <hex>", summary: "directed TX (RTS/CTS dialog)" },
    CmdInfo { cmd: Cmd::Node, name: "node", alias: 0, usage: "<set|clr> <id>", summary: "set/clear node ID in bitmap" },
    CmdInfo { cmd: Cmd::TestFrame, name: "testframe", alias: b'p', usage: "", summary: "TX known test frame via CSMA" },
    CmdInfo { cmd: Cmd::Stack, name: "stack", alias: 0, usage: "", summary: "core-1 stack high-water mark" },
    CmdInfo { cmd: Cmd::Wifi, name: "wifi", alias: 0, usage: "[add|del|list] ...", summary: "WiFi networks" },
    CmdInfo { cmd: Cmd::Ip, name: "ip", alias: 0, usage: "", summary: "show IP address" },
    CmdInfo { cmd: Cmd::Bootsel, name: "bootsel", alias: b'b', usage: "", summary: "reboot to BOOTSEL" },
];

/// Resolve a verb token to a command. Returns the command plus an optional
/// argument to inject ahead of the user's args (used so the legacy `n`/`N`
/// aliases map onto `node set`/`node clr`).
fn resolve(verb: &str) -> Option<(Cmd, Option<&'static str>)> {
    // Single-letter aliases (including the n/N special case).
    if verb.len() == 1 {
        match verb.as_bytes()[0] {
            b'n' => return Some((Cmd::Node, Some("set"))),
            b'N' => return Some((Cmd::Node, Some("clr"))),
            c => {
                for ci in COMMANDS {
                    if ci.alias == c {
                        return Some((ci.cmd, None));
                    }
                }
            }
        }
    }
    // Exact name.
    for ci in COMMANDS {
        if ci.name == verb {
            return Some((ci.cmd, None));
        }
    }
    // Unique prefix.
    let mut found = None;
    for ci in COMMANDS {
        if ci.name.starts_with(verb) {
            if found.is_some() {
                return None; // ambiguous
            }
            found = Some(ci.cmd);
        }
    }
    found.map(|c| (c, None))
}

// ── Constants ─────────────────────────────────────────────────────

const BUILD_ID: &str = env!("BUILD_ID");
const LINE_BUF_SIZE: usize = 1280;
const MAX_TOKENS: usize = 16;

static mut LINE_BUF: [u8; LINE_BUF_SIZE] = [0u8; LINE_BUF_SIZE];
static mut LINE_LEN: usize = 0;

/// Local copy of the node bitmap on core 0 for debug commands.
static mut LOCAL_NODES: NodeBitmap = NodeBitmap::empty();

// ── Task entry ────────────────────────────────────────────────────

#[embassy_executor::task]
pub async fn run(mut cdc: Cdc, shared: &'static SharedState) -> ! {
    // Wait until core 1 is alive before accepting any commands.
    while shared.flags.load(Ordering::Relaxed) & crate::core1::CORE1_ALIVE == 0 {
        embassy_time::Timer::after_millis(1).await;
    }

    let mut buf = [0u8; 64];
    loop {
        // Wait for a stable connection: after DTR goes high, require it to
        // stay up for 200ms before proceeding.  USB enumeration can toggle
        // DTR several times in quick succession, causing banner/log spam.
        loop {
            cdc.wait_connection().await;
            embassy_time::Timer::after_millis(200).await;
            if cdc.dtr() {
                break;
            }
        }
        info!("TT_DEBUG: host connected");
        unsafe { LINE_LEN = 0 };

        let _ = write_banner(&mut cdc).await;

        loop {
            // Try to deliver a pending log line without blocking.
            // If none, block on CDC input. This ensures log lines are
            // never written while input/command processing is in flight.
            if let Ok(line) = LOG_CHANNEL.try_receive() {
                if let Err(e) = wline(&mut cdc, line.as_bytes()).await {
                    warn!("TT_DEBUG log write failed: {}", classify(e));
                    break;
                }
                continue; // drain all pending before blocking for input
            }
            // No log pending — wait for either input or a new log line.
            match select(cdc.read_packet(&mut buf), LOG_CHANNEL.receive()).await {
                Either::First(Ok(n)) => {
                    // Process ALL bytes in this packet before delivering
                    // any log lines — feed_bytes may echo + dispatch.
                    if let Err(e) = feed_bytes(&mut cdc, &buf[..n], shared).await {
                        warn!("TT_DEBUG write failed: {}", classify(e));
                        break;
                    }
                }
                Either::First(Err(e)) => {
                    warn!("TT_DEBUG read failed: {}", classify(e));
                    break;
                }
                Either::Second(line) => {
                    if let Err(e) = wline(&mut cdc, line.as_bytes()).await {
                        warn!("TT_DEBUG log write failed: {}", classify(e));
                        break;
                    }
                }
            }
        }
        info!("TT_DEBUG: host disconnected");
    }
}

// ── Line reader ───────────────────────────────────────────────────

/// Accumulate input bytes into the line buffer, echoing for interactive use.
/// On CR/LF the completed line is dispatched.
async fn feed_bytes(cdc: &mut Cdc, data: &[u8], shared: &'static SharedState) -> Result<(), EndpointError> {
    for &b in data {
        match b {
            b'\r' | b'\n' => {
                wline(cdc, b"").await?; // echo the newline
                let len = unsafe { LINE_LEN };
                unsafe { LINE_LEN = 0 };
                if len > 0 {
                    let line = unsafe { &LINE_BUF[..len] };
                    match core::str::from_utf8(line) {
                        Ok(s) => handle_line(cdc, shared, s).await?,
                        Err(_) => reply_err(cdc, "badinput", "non-utf8").await?,
                    }
                }
            }
            0x08 | 0x7F => {
                // Backspace (BS) / delete (DEL) — erase one character.
                if unsafe { LINE_LEN } > 0 {
                    unsafe { LINE_LEN -= 1 };
                    write_chunked(cdc, b"\x08 \x08").await?;
                }
            }
            0x15 | 0x1B => {
                // Ctrl-U / ESC — kill the whole line.
                while unsafe { LINE_LEN } > 0 {
                    unsafe { LINE_LEN -= 1 };
                    write_chunked(cdc, b"\x08 \x08").await?;
                }
            }
            _ => unsafe {
                if LINE_LEN < LINE_BUF_SIZE {
                    LINE_BUF[LINE_LEN] = b;
                    LINE_LEN += 1;
                    write_chunked(cdc, core::slice::from_ref(&b)).await?;
                }
                // else: line overflow, silently drop until CR
            },
        }
    }
    Ok(())
}

/// Tokenise, resolve the verb, and dispatch.
async fn handle_line(cdc: &mut Cdc, shared: &'static SharedState, line: &str) -> Result<(), EndpointError> {
    let mut tokens: Vec<&str, MAX_TOKENS> = Vec::new();
    for t in line.split_ascii_whitespace() {
        if tokens.push(t).is_err() {
            break;
        }
    }
    let Some(&verb) = tokens.first() else {
        return Ok(()); // blank line
    };

    let Some((cmd, inject)) = resolve(verb) else {
        return reply_err(cdc, "unknown", verb).await;
    };

    // Build the argument list, prepending any injected alias argument.
    let mut args: Vec<&str, MAX_TOKENS> = Vec::new();
    if let Some(x) = inject {
        let _ = args.push(x);
    }
    for &t in &tokens[1..] {
        if args.push(t).is_err() {
            break;
        }
    }

    dispatch(cdc, shared, cmd, &args).await
}

async fn dispatch(cdc: &mut Cdc, shared: &'static SharedState, cmd: Cmd, args: &[&str]) -> Result<(), EndpointError> {
    match cmd {
        Cmd::Help => cmd_help(cdc, args).await,
        Cmd::Status => cmd_status(cdc, shared).await,
        Cmd::Clear => cmd_clear(cdc).await,
        Cmd::Monitor => cmd_toggle(cdc, &MONITOR_ENABLED, "monitor", args).await,
        Cmd::Analyzer => cmd_toggle(cdc, &ANALYZER_MODE, "analyzer", args).await,
        Cmd::Frames => cmd_frames(cdc).await,
        Cmd::Id => cmd_id(cdc).await,
        Cmd::Tx => cmd_tx_raw(cdc, shared, args).await,
        Cmd::Directed => cmd_directed(cdc, shared, args).await,
        Cmd::Node => cmd_node(cdc, shared, args).await,
        Cmd::TestFrame => cmd_testframe(cdc, shared).await,
        Cmd::Stack => cmd_stack(cdc).await,
        Cmd::Wifi => cmd_wifi(cdc, args).await,
        Cmd::Ip => cmd_ip(cdc).await,
        Cmd::Bootsel => cmd_bootsel(cdc).await,
    }
}

// ── Command implementations ───────────────────────────────────────

async fn cmd_help(cdc: &mut Cdc, args: &[&str]) -> Result<(), EndpointError> {
    use core::fmt::Write;
    // `help <cmd>` — usage for one command.
    if let Some(&verb) = args.first() {
        if let Some((cmd, _)) = resolve(verb) {
            for ci in COMMANDS {
                if ci.cmd == cmd {
                    let mut s = heapless::String::<128>::new();
                    let _ = write!(s, "# {} {}", ci.name, ci.usage);
                    wline(cdc, s.as_bytes()).await?;
                    return reply_ok(cdc).await;
                }
            }
        }
        return reply_err(cdc, "unknown", verb).await;
    }
    // Full table.
    wline(cdc, b"# commands:").await?;
    for ci in COMMANDS {
        let mut s = heapless::String::<128>::new();
        let alias = if ci.alias != 0 { ci.alias } else { b' ' };
        let _ = write!(s, "#  {:<9} [{}] {:<20} {}", ci.name, alias as char, ci.usage, ci.summary);
        wline(cdc, s.as_bytes()).await?;
    }
    reply_ok(cdc).await
}

async fn cmd_status(cdc: &mut Cdc, shared: &'static SharedState) -> Result<(), EndpointError> {
    use core::fmt::Write;
    let status = pio_status(shared);
    write_chunked(cdc, status.as_bytes()).await?;
    // WiFi status.
    let wifi_state = crate::wifi::state_str();
    let ssid = crate::wifi::connected_ssid();
    let mut s = heapless::String::<128>::new();
    let _ = write!(s, "STAT wifi state={} ssid={}", wifi_state, if ssid.is_empty() { "(none)" } else { ssid });
    wline(cdc, s.as_bytes()).await?;
    if let Some(stack) = crate::wifi::stack() {
        if let Some(config) = stack.config_v4() {
            let mut s = heapless::String::<64>::new();
            let _ = write!(s, "STAT wifi ip={}", config.address);
            wline(cdc, s.as_bytes()).await?;
        }
    }
    reply_ok(cdc).await
}

async fn cmd_clear(cdc: &mut Cdc) -> Result<(), EndpointError> {
    crate::core1::clear_counters();
    reply_ok(cdc).await
}

/// `monitor`/`analyzer` — toggle, or set explicitly with `on`/`off`.
async fn cmd_toggle(cdc: &mut Cdc, flag: &AtomicBool, key: &str, args: &[&str]) -> Result<(), EndpointError> {
    let next = match args.first() {
        None => !flag.load(Ordering::Relaxed),
        Some(&"on") => true,
        Some(&"off") => false,
        Some(_) => return reply_err(cdc, "badarg", "expected on|off").await,
    };
    flag.store(next, Ordering::Relaxed);
    reply_ok_kv(cdc, key, if next { "on" } else { "off" }).await
}

async fn cmd_frames(cdc: &mut Cdc) -> Result<(), EndpointError> {
    let count = unsafe { RX_LOG_COUNT };
    let write_idx = unsafe { RX_LOG_WRITE };

    let start = if count < RX_LOG_SLOTS { 0 } else { write_idx };

    let mut hex_buf = [0u8; 132]; // "RX: " + 64*2 + CRLF
    for i in 0..count {
        let idx = (start + i) % RX_LOG_SLOTS;
        let len = unsafe { RX_LOG_LENS[idx] } as usize;
        let stored = len.min(RX_LOG_BYTES);
        let frame = unsafe { &RX_LOG[idx][..stored] };

        let mut pos = 0;
        for &c in b"RX: " {
            hex_buf[pos] = c;
            pos += 1;
        }
        for &b in frame {
            hex_buf[pos] = hex_char(b >> 4);
            pos += 1;
            hex_buf[pos] = hex_char(b & 0xF);
            pos += 1;
        }
        wline(cdc, &hex_buf[..pos]).await?;
    }

    unsafe {
        RX_LOG_COUNT = 0;
        RX_LOG_WRITE = 0;
    }

    reply_ok_u32(cdc, "count", count as u32).await
}

async fn cmd_id(cdc: &mut Cdc) -> Result<(), EndpointError> {
    reply_ok_kv(cdc, "serial", crate::usb_dev::serial()).await
}

/// `tx <hex>...` — TX raw hex frame (incl CRC) via CSMA.
async fn cmd_tx_raw(cdc: &mut Cdc, shared: &'static SharedState, args: &[&str]) -> Result<(), EndpointError> {
    if parse_hex_tokens(args).is_none() {
        return reply_err(cdc, "badhex", "odd/invalid hex").await;
    }
    let len = frame_len();
    if !(5..=605).contains(&len) {
        return reply_err(cdc, "badlen", "want 5..605 bytes").await;
    }
    crate::host::submit_tx_from_debug(shared, frame_slice_ref()).await;
    reply_ok(cdc).await
}

/// `directed <dest> <hex>...` — build a DDP short frame with CRC and use the
/// RTS/CTS dialog.
async fn cmd_directed(cdc: &mut Cdc, shared: &'static SharedState, args: &[&str]) -> Result<(), EndpointError> {
    if args.len() < 2 {
        return reply_err(cdc, "usage", "directed <dest> <hex>").await;
    }
    let Some(dest) = parse_hex_byte(args[0].as_bytes()) else {
        return reply_err(cdc, "baddest", "dest hex").await;
    };
    if parse_hex_tokens(&args[1..]).is_none() {
        return reply_err(cdc, "badhex", "payload hex").await;
    }
    let payload_len = frame_len();

    let src = unsafe { LOCAL_NODES.iter().next().unwrap_or(1) };

    // LLAP data frame: [dest, src, type=0x01 (DDP short), payload..., CRC]
    let mut frame_buf = [0u8; 610];
    frame_buf[0] = dest;
    frame_buf[1] = src;
    frame_buf[2] = 0x01;
    let payload = frame_slice_ref();
    let total_data = 3 + payload_len;
    frame_buf[3..total_data].copy_from_slice(&payload[..payload_len]);
    let crc = lt_crc(&frame_buf[..total_data]);
    frame_buf[total_data] = crc[0];
    frame_buf[total_data + 1] = crc[1];
    let total = total_data + 2;

    crate::host::submit_tx_from_debug(shared, &frame_buf[..total]).await;

    let flags = shared.flags.load(Ordering::Relaxed);
    if flags & crate::core1::TX_RESULT_FAIL != 0 {
        reply_err(cdc, "timeout", "no CTS").await
    } else {
        reply_ok_kv(cdc, "dest", args[0]).await
    }
}

/// `node <set|clr> <id>` — set or clear a node ID in the auto-reply bitmap.
async fn cmd_node(cdc: &mut Cdc, shared: &'static SharedState, args: &[&str]) -> Result<(), EndpointError> {
    if args.len() < 2 {
        return reply_err(cdc, "usage", "node <set|clr> <id>").await;
    }
    let set = match args[0] {
        "set" | "on" => true,
        "clr" | "clear" | "off" => false,
        _ => return reply_err(cdc, "badarg", "set|clr").await,
    };
    let Some(node_id) = parse_hex_byte(args[1].as_bytes()) else {
        return reply_err(cdc, "badid", "id hex").await;
    };
    unsafe {
        if set {
            LOCAL_NODES.set(node_id);
        } else {
            LOCAL_NODES.clear(node_id);
        }
        crate::host::update_nodes(shared, LOCAL_NODES.as_bytes());
    }
    // Wait for core 1 to acknowledge the bitmap update (clears NODES_UPDATED).
    let deadline = embassy_time::Instant::now() + embassy_time::Duration::from_millis(100);
    while shared.flags.load(Ordering::Relaxed) & crate::core1::NODES_UPDATED != 0 {
        if embassy_time::Instant::now() >= deadline {
            break;
        }
        embassy_time::Timer::after_micros(10).await;
    }
    reply_ok_kv(cdc, "node", if set { "set" } else { "clr" }).await
}

/// `stack` — report the core-1 stack high-water mark. Run a representative
/// workload (e.g. an AFP session) first, then read this to size the stack
/// down safely. The stack is sentinel-painted at boot in `main.rs`.
async fn cmd_stack(cdc: &mut Cdc) -> Result<(), EndpointError> {
    use core::fmt::Write;
    let (used, total) = crate::core1_stack_stats();
    let mut s = heapless::String::<96>::new();
    let _ = write!(s, "STAT core1_stack used={} free={} total={}", used, total - used, total);
    wline(cdc, s.as_bytes()).await?;
    reply_ok_u32(cdc, "used", used as u32).await
}

async fn cmd_testframe(cdc: &mut Cdc, shared: &'static SharedState) -> Result<(), EndpointError> {
    let frame = debug_test_frame();
    crate::host::submit_tx_from_debug(shared, &frame).await;
    reply_ok(cdc).await
}

async fn cmd_wifi(cdc: &mut Cdc, args: &[&str]) -> Result<(), EndpointError> {
    use core::fmt::Write;
    match args.first().copied() {
        Some("add") => {
            if args.len() < 3 {
                return reply_err(cdc, "usage", "wifi add <ssid> <pass>").await;
            }
            if crate::wifi::add_network(args[1], args[2]) {
                reply_ok_kv(cdc, "ssid", args[1]).await
            } else {
                reply_err(cdc, "full", "max 4 networks").await
            }
        }
        Some("del") => {
            if args.len() < 2 {
                return reply_err(cdc, "usage", "wifi del <ssid>").await;
            }
            if crate::wifi::remove_network(args[1]) {
                reply_ok_kv(cdc, "removed", args[1]).await
            } else {
                reply_err(cdc, "notfound", args[1]).await
            }
        }
        Some("list") => {
            let n = crate::wifi::network_count();
            for i in 0..n {
                let mut s = heapless::String::<64>::new();
                let _ = write!(s, "STAT wifi[{}] ssid={}", i, crate::wifi::network_ssid(i));
                wline(cdc, s.as_bytes()).await?;
            }
            reply_ok_u32(cdc, "count", n as u32).await
        }
        _ => {
            // No subcommand: show status.
            let state = crate::wifi::state_str();
            let connected = crate::wifi::connected_ssid();
            let mut s = heapless::String::<96>::new();
            if connected.is_empty() {
                let _ = write!(s, "STAT wifi state={} networks={}", state, crate::wifi::network_count());
            } else {
                let _ = write!(s, "STAT wifi state={} ssid={} networks={}", state, connected, crate::wifi::network_count());
            }
            wline(cdc, s.as_bytes()).await?;
            reply_ok(cdc).await
        }
    }
}

async fn cmd_ip(cdc: &mut Cdc) -> Result<(), EndpointError> {
    use core::fmt::Write;
    if let Some(stack) = crate::wifi::stack() {
        if let Some(config) = stack.config_v4() {
            let addr = config.address;
            let mut s = heapless::String::<64>::new();
            let _ = write!(s, "STAT ip addr={}", addr);
            wline(cdc, s.as_bytes()).await?;
            if let Some(gw) = config.gateway {
                let mut s = heapless::String::<64>::new();
                let _ = write!(s, "STAT ip gw={}", gw);
                wline(cdc, s.as_bytes()).await?;
            }
            return reply_ok(cdc).await;
        }
    }
    wline(cdc, b"STAT ip addr=(none)").await?;
    reply_ok(cdc).await
}

async fn cmd_bootsel(cdc: &mut Cdc) -> Result<(), EndpointError> {
    reply_ok(cdc).await?;
    embassy_time::Timer::after_millis(50).await;
    embassy_rp::rom_data::reset_to_usb_boot(0, 0);
    Ok(())
}

// ── Response helpers (unified output format) ──────────────────────

/// Write one line followed by CRLF (line-atomic on the CDC pipe).
async fn wline(cdc: &mut Cdc, data: &[u8]) -> Result<(), EndpointError> {
    write_chunked(cdc, data).await?;
    write_chunked(cdc, b"\r\n").await
}

async fn reply_ok(cdc: &mut Cdc) -> Result<(), EndpointError> {
    wline(cdc, b"OK").await
}

async fn reply_ok_kv(cdc: &mut Cdc, key: &str, val: &str) -> Result<(), EndpointError> {
    use core::fmt::Write;
    let mut s = heapless::String::<128>::new();
    let _ = write!(s, "OK {}={}", key, val);
    wline(cdc, s.as_bytes()).await
}

async fn reply_ok_u32(cdc: &mut Cdc, key: &str, val: u32) -> Result<(), EndpointError> {
    use core::fmt::Write;
    let mut s = heapless::String::<64>::new();
    let _ = write!(s, "OK {}={}", key, val);
    wline(cdc, s.as_bytes()).await
}

async fn reply_err(cdc: &mut Cdc, code: &str, msg: &str) -> Result<(), EndpointError> {
    use core::fmt::Write;
    let mut s = heapless::String::<128>::new();
    let _ = write!(s, "ERR {} {}", code, msg);
    wline(cdc, s.as_bytes()).await
}

async fn write_banner(cdc: &mut Cdc) -> Result<(), EndpointError> {
    use core::fmt::Write;
    let mut s = heapless::String::<160>::new();
    let ms = embassy_time::Instant::now().as_millis();
    let _ = write!(s, "# TASHTALK-PICO {} serial={} uptime={}ms", BUILD_ID, crate::usb_dev::serial(), ms);
    wline(cdc, s.as_bytes()).await?;
    wline(cdc, b"# type 'help' for commands").await?;
    reply_ok(cdc).await
}

// ── Hex / parsing helpers ─────────────────────────────────────────

fn debug_test_frame() -> [u8; 10] {
    let mut f = [0u8; 10];
    f[0] = 0x42;
    f[1] = 0x01;
    f[2] = 0x01;
    f[3] = 0x00;
    f[4] = 0x05;
    f[5] = 0xDE;
    f[6] = 0x00;
    f[7] = 0xAD;
    let crc = lt_crc(&f[..8]);
    f[8] = crc[0];
    f[9] = crc[1];
    f
}

fn hex_char(nibble: u8) -> u8 {
    if nibble < 10 {
        b'0' + nibble
    } else {
        b'A' + nibble - 10
    }
}

fn hex_val(c: u8) -> Option<u8> {
    match c {
        b'0'..=b'9' => Some(c - b'0'),
        b'a'..=b'f' => Some(c - b'a' + 10),
        b'A'..=b'F' => Some(c - b'A' + 10),
        _ => None,
    }
}

// Shared frame parse buffer (single-core-0 access).
static mut FRAME_BUF: [u8; 610] = [0u8; 610];
static mut FRAME_BUF_LEN: usize = 0;

/// Parse one or more whitespace-separated hex tokens into [`FRAME_BUF`].
/// Each token must contain an even number of hex digits. Returns None on any
/// invalid digit, odd-length token, or overflow.
fn parse_hex_tokens(tokens: &[&str]) -> Option<()> {
    let mut len = 0usize;
    for tok in tokens {
        let bytes = tok.as_bytes();
        if bytes.len() % 2 != 0 {
            return None;
        }
        for pair in bytes.chunks_exact(2) {
            if len >= 610 {
                return None;
            }
            let hi = hex_val(pair[0])?;
            let lo = hex_val(pair[1])?;
            unsafe { FRAME_BUF[len] = (hi << 4) | lo };
            len += 1;
        }
    }
    unsafe { FRAME_BUF_LEN = len };
    Some(())
}

fn frame_len() -> usize {
    unsafe { FRAME_BUF_LEN }
}

fn frame_slice_ref() -> &'static [u8] {
    unsafe { &FRAME_BUF[..FRAME_BUF_LEN] }
}

fn parse_hex_byte(hex: &[u8]) -> Option<u8> {
    if hex.len() < 2 {
        return None;
    }
    let hi = hex_val(hex[0])?;
    let lo = hex_val(hex[1])?;
    Some((hi << 4) | lo)
}

// ── CDC I/O ───────────────────────────────────────────────────────

async fn write_chunked(cdc: &mut Cdc, data: &[u8]) -> Result<(), EndpointError> {
    for chunk in data.chunks(64) {
        match select(cdc.write_packet(chunk), embassy_time::Timer::after_millis(100)).await {
            Either::First(r) => r?,
            Either::Second(()) => return Err(EndpointError::Disabled),
        }
    }
    Ok(())
}

// ── Status report ─────────────────────────────────────────────────

fn pio_status(shared: &SharedState) -> heapless::String<768> {
    use core::fmt::Write;
    use embassy_rp::pac;

    let mut s = heapless::String::<768>::new();

    let pio1_ctrl = pac::PIO1.ctrl().read();
    let gpio_in = pac::SIO.gpio_in(0).read();
    let flags = shared.flags.load(Ordering::Relaxed);
    let (_, waddr, ridx, en, remain) = crate::core1::rx_dma_diag();
    let base = unsafe { crate::core1::RX_RING.0.as_ptr() } as u32;
    let widx = (waddr.wrapping_sub(base)) / 4;

    let sm0_pc = pac::PIO1.sm(0).addr().read().addr();
    let gp4_pad = pac::PADS_BANK0.gpio(4).read();
    let gp4_ctrl = pac::IO_BANK0.gpio(4).ctrl().read();
    let _ = write!(s, "STAT pio1 sm_en={:04b} sm0_pc={} gp4={} gp5={}\r\n",
        pio1_ctrl.sm_enable(), sm0_pc, (gpio_in >> 4) & 1, (gpio_in >> 5) & 1);
    let _ = write!(s, "STAT gp4 funcsel={} ie={} od={} pue={} pde={}\r\n",
        gp4_ctrl.funcsel() as u8, gp4_pad.ie() as u8, gp4_pad.od() as u8,
        gp4_pad.pue() as u8, gp4_pad.pde() as u8);
    let _ = write!(s, "STAT flags 0x{:02x} alive={} rxrdy={} txsub={}\r\n",
        flags, (flags >> 5) & 1, (flags >> 3) & 1, flags & 1);
    let loop_count = unsafe { crate::core1::LOOP_COUNT };
    let _ = write!(s, "STAT dma en={} widx={} ridx={} remain={}\r\n", en, widx, ridx, remain);
    let _ = write!(s, "STAT core1 loops={}\r\n", loop_count);

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

    let _ = write!(s, "STAT recv good={} bad={} bad_real={} ar={} echo={} overrun={}\r\n",
        good, bad, bad_real, ar, echo_skip, overruns);
    let _ = write!(s, "STAT send dir_att={} cts_ok={} tmo={} dir_rx={}\r\n",
        dir_att, dir_cts, dir_tmo, dir_rx);

    let cts_start = unsafe { crate::core1::LAST_CTS_WAIT_START };
    let cts_end = unsafe { crate::core1::LAST_CTS_WAIT_END };
    let cts_entries = unsafe { crate::core1::LAST_CTS_ENTRIES };
    if cts_start > 0 {
        let _ = write!(s, "STAT cts_wait us={} entries={}\r\n",
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
