//! Debug CDC task — TT_DEBUG.
//!
//! Drains the `tt_log` channel and emits each message to the host as a
//! line. Bytes received from the host are echoed. Single-letter debug
//! commands are processed inline.

use crate::tt_log::{LogLine, CHANNEL as LOG_CHANNEL};
use crate::usb_dev::Cdc;
use crate::{SharedBackoff, SharedTxData, SharedTxEnable};
use crate::localtalk::tx;
use defmt::{info, warn};
use embassy_futures::select::{select, Either};
use embassy_usb::driver::EndpointError;
use tashtalk_proto::crc::lt_crc;

const BANNER: &[u8] = b"\r\n\
+--------------------------------------------------+\r\n\
| tashtalk-pico: TT_DEBUG online (new PHY)         |\r\n\
| commands:                                        |\r\n\
|   p  - PIO TX a known frame via CSMA             |\r\n\
|   b  - reboot into USB BOOTSEL                   |\r\n\
|   ?  - help                                      |\r\n\
+--------------------------------------------------+\r\n";

const HELP: &[u8] = b"\r\n\
commands:\r\n\
  p  Transmit a known LLAP frame via CSMA/CA.\r\n\
  b  Reboot to USB BOOTSEL for re-flashing.\r\n\
  ?  This help.\r\n\
\r\n";

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

#[embassy_executor::task]
pub async fn run(
    mut cdc: Cdc,
    tx_data: &'static SharedTxData,
    tx_enable: &'static SharedTxEnable,
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
                        handle_input(&mut cdc, &buf[..n], tx_data, tx_enable, backoff).await
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

async fn handle_input(
    cdc: &mut Cdc,
    data: &[u8],
    tx_data: &'static SharedTxData,
    tx_enable: &'static SharedTxEnable,
    backoff: &'static SharedBackoff,
) -> Result<(), EndpointError> {
    for &b in data {
        match b {
            b'p' | b'P' => {
                write_chunked(cdc, b"\r\n[debug] TX via CSMA/CA\r\n").await?;
                let frame = debug_test_frame();
                {
                    let mut txd = tx_data.lock().await;
                    let mut txe = tx_enable.lock().await;
                    let mut bo = backoff.lock().await;
                    let ok = tx::transmit_csma(&mut txd, &mut txe, &mut bo, &frame).await;
                    if ok {
                        write_chunked(cdc, b"[debug] TX done\r\n").await?;
                    } else {
                        write_chunked(cdc, b"[debug] TX failed (collision)\r\n").await?;
                    }
                }
            }
            b'b' | b'B' => {
                write_chunked(cdc, b"\r\n[debug] rebooting to BOOTSEL...\r\n").await?;
                embassy_time::Timer::after_millis(50).await;
                embassy_rp::rom_data::reset_to_usb_boot(0, 0);
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

fn classify(e: EndpointError) -> defmt::Str {
    match e {
        EndpointError::BufferOverflow => defmt::intern!("buffer overflow"),
        EndpointError::Disabled => defmt::intern!("endpoint disabled"),
    }
}
