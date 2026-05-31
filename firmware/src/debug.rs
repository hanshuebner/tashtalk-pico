//! Debug CDC task — TT_DEBUG.
//!
//! Drains the `tt_log` channel and emits each message to the host as a
//! line. Bytes received from the host are echoed (line-editor friendly:
//! we add a `\r` after every `\n` so a `screen` session shows the cursor
//! returning to column 0).
//!
//! The combination — log lines coming out, plus echo — gives a single
//! terminal window per Pico that doubles as "watch what the firmware is
//! doing" and "send characters to provoke behaviour".

use crate::host::transmit_on_phy;
use crate::MaybeLtDialogTx;
use crate::tt_log::{LogLine, CHANNEL as LOG_CHANNEL};
use crate::usb_dev::Cdc;
use crate::{MaybeLtRx, MaybeLtTx, SharedBackoff, SharedDe};
use defmt::{info, warn};
use embassy_futures::select::{select, Either};
use embassy_usb::driver::EndpointError;
use tashtalk_proto::crc::lt_crc;

const BANNER: &[u8] = b"\r\n\
+--------------------------------------------------+\r\n\
| tashtalk-pico: TT_DEBUG online                   |\r\n\
| commands:                                        |\r\n\
|   p  - PIO TX a known frame on GP2/GP3           |\r\n\
|   l  - TX + RX self-loopback (GP2 <-> GP4 jumper)|\r\n\
|   a  - autoresponder test: ENQ -> ACK round-trip |\r\n\
|   b  - reboot into USB BOOTSEL                   |\r\n\
|   ?  - help                                      |\r\n\
+--------------------------------------------------+\r\n";

const HELP: &[u8] = b"\r\n\
commands (jumper GP2 <-> GP4 required for l / a):\r\n\
  p  Transmit a known LLAP frame on LT_DATA (GP2). Verify with a scope.\r\n\
  l  Self-loopback: same TX as p, plus RX samples GP4, decodes\r\n\
     the FM-0 stream, and prints the recovered byte sequence.\r\n\
  a  Autoresponder test: transmit an ENQ targeting node 0x42, capture\r\n\
     it via the loopback, run the autoresponder logic, transmit the\r\n\
     resulting ACK, capture and verify it on the wire.\r\n\
  b  Reboot to USB BOOTSEL for re-flashing.\r\n\
  ?  This help.\r\n\
\r\n";

/// Node ID we claim to own for the autoresponder test.
const TEST_NODE_ID: u8 = 0x42;
/// Pretend-prober ID used in the ENQ source field.
const TEST_PROBER_ID: u8 = 0xFE;

/// Build a deterministic test frame for the debug 'p' / 'l' commands.
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
fn u8_to_hex(b: u8, out: &mut [u8; 4]) -> usize {
    const HEX: &[u8] = b"0123456789abcdef";
    out[0] = HEX[(b >> 4) as usize];
    out[1] = HEX[(b & 0xF) as usize];
    2
}

#[embassy_executor::task]
pub async fn run(
    mut cdc: Cdc,
    lt_tx: &'static MaybeLtTx,
    lt_dialog_tx: &'static MaybeLtDialogTx,
    lt_rx: &'static MaybeLtRx,
    lt_de: &'static SharedDe,
    backoff: &'static SharedBackoff,
) -> ! {
    let mut buf = [0u8; 64];
    loop {
        cdc.wait_connection().await;
        info!("TT_DEBUG: host connected");

        let _ = write_chunked(&mut cdc, BANNER).await;

        loop {
            match select(cdc.read_packet(&mut buf), LOG_CHANNEL.receive()).await {
                Either::First(Ok(n)) => {
                    if let Err(e) =
                        handle_input(&mut cdc, &buf[..n], lt_tx, lt_dialog_tx, lt_rx, lt_de, backoff).await
                    {
                        warn!("TT_DEBUG echo write failed: {}", classify(e));
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

/// Process bytes typed at TT_DEBUG: echo, plus a tiny single-letter shell.
async fn handle_input(
    cdc: &mut Cdc,
    data: &[u8],
    lt_tx: &'static MaybeLtTx,
    lt_dialog_tx: &'static MaybeLtDialogTx,
    lt_rx: &'static MaybeLtRx,
    lt_de: &'static SharedDe,
    backoff: &'static SharedBackoff,
) -> Result<(), EndpointError> {
    for &b in data {
        match b {
            b'p' | b'P' => {
                write_chunked(cdc, b"\r\n[debug] firing PIO TX on GP2/GP3\r\n").await?;
                let frame = debug_test_frame();
                transmit_on_phy(lt_tx, lt_dialog_tx, lt_de, backoff, &frame).await;
                write_chunked(cdc, b"[debug] PIO TX done\r\n").await?;
            }
            b'd' | b'D' => {
                use crate::host::AUTOREPLY_IFG;
                use core::sync::atomic::Ordering;
                let was = AUTOREPLY_IFG.load(Ordering::Relaxed);
                AUTOREPLY_IFG.store(!was, Ordering::Relaxed);
                if was {
                    write_chunked(cdc, b"\r\n[debug] autoreply IFG OFF\r\n").await?;
                } else {
                    write_chunked(cdc, b"\r\n[debug] autoreply IFG ON (200us)\r\n").await?;
                }
            }
            b'l' | b'L' | b'a' | b'A' => {
                write_chunked(
                    cdc,
                    b"\r\n[debug] l/a disabled - continuous lt_rx_task owns LtRx now\r\n",
                )
                .await?;
                // Keep symbols alive so the compiler doesn't strip them.
                let _ = (lt_rx, backoff);
            }
            b'b' | b'B' => {
                write_chunked(cdc, b"\r\n[debug] rebooting to BOOTSEL...\r\n").await?;
                // Give the USB stack ~50 ms to flush so the host sees
                // the message before we vanish off the bus.
                embassy_time::Timer::after_millis(50).await;
                // (gpio_activity_pin_mask, disable_interface_mask). 0,0
                // means: no LED indicator, both PICOBOOT and MSC stay
                // enabled — exactly what picotool/elf2uf2-rs expect.
                embassy_rp::rom_data::reset_to_usb_boot(0, 0);
                // unreachable
            }
            b'?' => {
                write_chunked(cdc, HELP).await?;
            }
            b'\r' => {
                write_chunked(cdc, b"\r\n").await?;
            }
            _ => {
                write_chunked(cdc, core::slice::from_ref(&b)).await?;
            }
        }
    }
    Ok(())
}

/// Write `data` to the CDC, splitting on the 64-byte max-packet boundary.
/// Each chunk has a 100ms timeout to prevent blocking when the host
/// isn't reading.
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

/// Emit one log line followed by `\r\n`.
async fn write_log_line(cdc: &mut Cdc, line: &LogLine) -> Result<(), EndpointError> {
    write_chunked(cdc, line.as_bytes()).await?;
    write_chunked(cdc, b"\r\n").await?;
    Ok(())
}

fn classify(e: EndpointError) -> defmt::Str {
    match e {
        EndpointError::BufferOverflow => defmt::intern!("buffer overflow"),
        EndpointError::Disabled => defmt::intern!("endpoint disabled"),
    }
}
