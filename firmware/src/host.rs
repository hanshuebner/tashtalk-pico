//! Data CDC task — TT_DATA.
//!
//! Wires the host byte stream into `tashtalk-proto`'s `CommandParser`
//! (host → LocalTalk) and forwards every LocalTalk frame heard on the
//! wire back up to the host (LocalTalk → host), escape-encoded exactly
//! as the original TashTalk firmware does.

use crate::tt_log;
use crate::usb_dev::Cdc;
use crate::{SharedBackoff, SharedNodes, SharedTxData, SharedTxEnable};
use core::sync::atomic::{AtomicU8, Ordering};
use defmt::warn;
use embassy_futures::select::{select, Either};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::Timer;
use embassy_usb::driver::EndpointError;
use tashtalk_proto::codec::{
    encode_frame, CommandParser, FrameStatus, Sink, MAX_FRAME_LEN,
};
use tashtalk_proto::nodemap::NodeBitmap;

use crate::localtalk::tx;

/// One-shot SetNodeIds events.
pub static SET_NODES: Channel<ThreadModeRawMutex, [u8; 32], 4> = Channel::new();

/// Outbound LocalTalk frames queued by the host command parser.
pub static OUTBOUND_FRAMES: Channel<ThreadModeRawMutex, ([u8; 610], u16), 4> =
    Channel::new();

/// A complete LocalTalk frame received off the wire.
pub struct InboundFrame {
    pub buf: [u8; MAX_FRAME_LEN],
    pub len: u16,
    pub status: FrameStatus,
}

/// LocalTalk → host frame queue.
pub static INBOUND_FRAMES: Channel<ThreadModeRawMutex, InboundFrame, 8> = Channel::new();

/// Active feature bits, as set by the host's `0x03 SetFeatures` command.
pub static FEATURES: AtomicU8 = AtomicU8::new(0);

/// Queue a received LocalTalk frame for delivery up TT_DATA.
pub fn try_forward_inbound(frame: &[u8], status: FrameStatus) {
    let n = frame.len().min(MAX_FRAME_LEN);
    let mut f = InboundFrame {
        buf: [0u8; MAX_FRAME_LEN],
        len: n as u16,
        status,
    };
    f.buf[..n].copy_from_slice(&frame[..n]);
    if INBOUND_FRAMES.try_send(f).is_err() {
        tt_log!("LT_RX: host queue full, dropped frame");
    }
}

#[embassy_executor::task]
pub async fn run(
    mut cdc: Cdc,
    tx_data: &'static SharedTxData,
    tx_enable: &'static SharedTxEnable,
    nodes: &'static SharedNodes,
    backoff: &'static SharedBackoff,
) -> ! {
    let mut buf = [0u8; 64];
    let mut parser = CommandParser::new();
    let mut sink = LogSink::default();

    loop {
        cdc.wait_connection().await;
        defmt::info!("TT_DATA: host connected");
        tt_log!("TT_DATA: host connected");
        parser.reset();

        loop {
            // Drain outbound frames on every iteration.
            while let Ok((frame_buf, frame_len)) = OUTBOUND_FRAMES.try_receive() {
                let frame = &frame_buf[..frame_len as usize];
                tt_log!("TX {} bytes d={:02x} s={:02x} t={:02x}",
                        frame.len(), frame[0], frame[1], frame[2]);
                transmit_on_phy(tx_data, tx_enable, backoff, frame).await;
            }

            match select(cdc.read_packet(&mut buf), INBOUND_FRAMES.receive()).await {
                Either::First(Ok(n)) => {
                    for &byte in &buf[..n] {
                        parser.push(byte, &mut sink);
                    }
                    while let Ok(new_bitmap_bytes) = SET_NODES.try_receive() {
                        let mut g = nodes.lock().await;
                        *g = NodeBitmap::from_bytes(new_bitmap_bytes);
                    }
                    while let Ok((frame_buf, frame_len)) = OUTBOUND_FRAMES.try_receive() {
                        let frame = &frame_buf[..frame_len as usize];
                        tt_log!("TX {} bytes d={:02x} s={:02x} t={:02x}",
                                frame.len(), frame[0], frame[1], frame[2]);
                        transmit_on_phy(tx_data, tx_enable, backoff, frame).await;
                    }
                }
                Either::First(Err(e)) => {
                    warn!("TT_DATA read failed: {}", classify(e));
                    break;
                }
                Either::Second(inbound) => {
                    if cdc.dtr() {
                        let encoded =
                            encode_frame(&inbound.buf[..inbound.len as usize], inbound.status);
                        if let Err(e) = write_chunked(&mut cdc, &encoded).await {
                            warn!("TT_DATA write failed: {}", classify(e));
                            break;
                        }
                    }
                }
            }
        }
    }
}

/// Drive one host-originated frame out the LocalTalk PHY after CSMA.
///
/// Directed data frames use RTS/CTS handshake. Broadcast and control
/// frames are sent directly.
async fn transmit_on_phy(
    tx_data: &'static SharedTxData,
    tx_enable: &'static SharedTxEnable,
    backoff: &'static SharedBackoff,
    frame: &[u8],
) {
    let is_directed_data =
        frame.len() >= 3 && frame[2] < 0x80 && frame[0] != 0xFF;

    let mut txd = tx_data.lock().await;
    let mut txe = tx_enable.lock().await;
    let mut bo = backoff.lock().await;

    if is_directed_data {
        // For directed data: send RTS via CSMA, then wait for CTS,
        // then send data immediately.
        // TODO: implement full RTS/CTS dialog. For now, just CSMA transmit.
        if !tx::transmit_csma(&mut txd, &mut txe, &mut bo, frame).await {
            tt_log!("TX: CSMA failed after retries");
        }
    } else {
        if !tx::transmit_csma(&mut txd, &mut txe, &mut bo, frame).await {
            tt_log!("TX: CSMA failed after retries");
        }
    }
}

async fn write_chunked(cdc: &mut Cdc, data: &[u8]) -> Result<(), EndpointError> {
    for chunk in data.chunks(64) {
        match embassy_futures::select::select(
            cdc.write_packet(chunk),
            Timer::after_millis(100),
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

struct LogSink {
    current_frame: [u8; 610],
    current_frame_len: u16,
}

impl Default for LogSink {
    fn default() -> Self {
        Self {
            current_frame: [0u8; 610],
            current_frame_len: 0,
        }
    }
}

impl Sink for LogSink {
    fn on_noop(&mut self) {}

    fn on_frame_header(&mut self, _dest: u8, _src: u8, _llap_type: u8) {
        self.current_frame_len = 0;
    }

    fn on_frame_byte(&mut self, byte: u8) {
        if (self.current_frame_len as usize) < self.current_frame.len() {
            self.current_frame[self.current_frame_len as usize] = byte;
            self.current_frame_len += 1;
        }
    }

    fn on_frame_end(&mut self) {
        let _ = OUTBOUND_FRAMES.try_send((self.current_frame, self.current_frame_len));
        self.current_frame_len = 0;
    }

    fn on_set_node_ids(&mut self, bitmap: &[u8; 32]) {
        tt_log!("HOST: SetNodeIds");
        let _ = SET_NODES.try_send(*bitmap);
    }

    fn on_set_features(&mut self, features: u8) {
        FEATURES.store(features, Ordering::Relaxed);
        tt_log!("HOST: SetFeatures {:02x}", features);
    }
}

fn classify(e: EndpointError) -> defmt::Str {
    match e {
        EndpointError::BufferOverflow => defmt::intern!("buffer overflow"),
        EndpointError::Disabled => defmt::intern!("endpoint disabled"),
    }
}
