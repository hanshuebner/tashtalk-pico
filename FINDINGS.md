# LocalTalk PHY Implementation Findings

Accumulated knowledge from two weeks of implementation attempts on the
RP2040 (Raspberry Pi Pico). This document captures everything we know
so that a fresh implementation can avoid repeating the same mistakes.

## Hardware Setup

### Pin Assignments
- **GP2** — TX data (PIO0 SM0 OUT → RS-422 driver DI)
- **GP3** — Driver enable (software GPIO → RS-422 DE)
- **GP4** — RX data (PIO1 SM0 JMP PIN ← RS-422 receiver RO)
- **GP5** — Carrier sense (PIO1 SM0 SET → used by CSMA logic)
- **GP6** — Debug output (AD2 trigger / timing measurement)

### RS-422 Transceiver
- RE tied to GND (receiver always enabled)
- This means we always see the bus, including our own transmissions
- Required for proper CSMA/CA (must sense bus during and after TX)

### Analog Discovery 2 (AD2)
- Connected via FTDI (shows as `0403:6014` on USB, not Digilent VID)
- DIO0=A, DIO1=B, DIO2=DI(TX), DIO3=DE, DIO4=RE, DIO5=RO(RX), DIO6=GP6(debug)
- VIO must be set to 3.3V via both `FDwfAnalogIOChannelNodeSet` and
  `FDwfDeviceParamSet(DwfParamDigitalVoltage, 3300)` — without this,
  data sampling returns all zeros even though triggers still work
- Python tooling in `tools/ad2.py` wraps the dwf SDK
- The AD2 buffer is only 4096 samples. At 10 MHz that's 0.4ms, at 1 MHz
  that's 4ms. Plan capture rates accordingly.

## Protocol Requirements

### RTS/CTS/DATA Dialog (MANDATORY)
The LocalTalk spec requires RTS/CTS for directed data frames. This is
NOT optional. Disabling it causes the "works but slow" behavior where
the Mac retransmits on its ATP timer. The whole point of the dual-core
architecture is to meet the timing requirements of this dialog.

The timing budget from the PIC TashTalk reference:
- **RTS→CTS window**: Peer must respond with CTS within 200µs
- **CTS→DATA**: Originator must begin DATA immediately (within IFG)
- **IFG (Inter-Frame Gap)**: ~200µs minimum
- **IDG (Inter-Dialog Gap)**: 400µs minimum

### CSMA/CA
- CSMA with carrier sense on GP5 (driven by PIO1 RX edge timer)
- Backoff: exponential, 100µs units, up to 32 attempts
- The backoff state machine in `tashtalk-proto::backoff` is a direct
  port from the PIC assembly and passes equivalent tests

### Auto-Reply
The firmware must autonomously respond to:
- **ENQ → ACK**: For LLAP address management
- **RTS → CTS**: For directed data dialog (the critical timing path)

Both must be sent as immediate responses (no CSMA), within the IFG
after the triggering frame.

## Architecture: Dual-Core

### Why Two Cores
Embassy's cooperative scheduler cannot guarantee the ~200µs response
time for RTS→CTS. Every `.await` is a potential yield point. The
solution: dedicate core 1 to a bare-metal polling loop that owns all
wire interaction.

### Core 0 (Embassy async)
- USB CDC driver (TT_DATA + TT_DEBUG)
- Host protocol parsing (TashTalk byte protocol)
- Frame pre-encoding (HDLC + FM-0 + DMA word packing)
- Submits TX requests to core 1 via shared memory + atomic flags

### Core 1 (bare-metal loop)
- PIO0 SM0 (TX data shift-out at 460.8 kHz)
- PIO1 SM0 (RX edge timer at 9.216 MHz)
- DMA CH1 (TX data to PIO0 SM0)
- DMA CH0 (RX: continuous drain of PIO1 FIFO into ring buffer)
- DE pin control (GP3, software GPIO via PAC)
- FM-0 decode, bit-unstuff, frame assembly, CRC check
- Auto-reply TX (ENQ→ACK, RTS→CTS)
- CSMA/CA TX (host-initiated frames)
- RTS/CTS/DATA dialog (directed data)

### Inter-Core Communication
- `SharedState` struct with `UnsafeCell` fields + `AtomicU32` flags
- Single-writer protocol: each field has one writer, synchronized by flags
- `TX_SUBMIT` / `TX_DONE`: core 0 submits pre-encoded frame, core 1 transmits
- `RX_READY`: core 1 writes decoded frame, core 0 reads and forwards
- `NODES_UPDATED`: core 0 updates node bitmap, core 1 reads
- No SIO FIFO, no mutexes across cores

### Critical: Embassy PIO Handle Drop
Embassy's `StateMachine::drop()` disables the SM, and `Common::drop()`
resets pin function selects to NULL. After configuring PIO in `init_phy`,
all handles must be leaked via `mem::forget()` to prevent this. Without
this, PIO1 SM0 (RX) silently stops after `init_phy` returns.

## RX Path

### PIO1 SM0: Edge Timer
Runs at 9.216 MHz (40× half-bit rate). Measures inter-edge intervals
on GP4 and pushes 5-bit remaining counts to the RX FIFO. Also drives
carrier sense on GP5.

- `remaining > 16` → short interval (half cell) → FM-0 '0' bit
- `remaining ≤ 16` → long interval (full cell) → FM-0 '1' bit  
- `remaining == 0` → timeout (idle gap) → reset decoder to Boundary

Clock recovery is implicit: each edge re-anchors timing. No free-running
clock to drift. Works with independent clock sources (Mac vs Pico).

### DMA Ring Buffer for RX
DMA CH0 continuously drains PIO1 SM0 RX FIFO into a ring buffer in RAM.
This is essential because core 1 cannot poll the FIFO during TX (it's
busy in `transmit_dma`). Without DMA, the 8-entry PIO FIFO overflows
during any TX longer than ~54µs.

Configuration:
- Ring buffer must be power-of-2 sized and aligned (e.g., 4096 entries = 16KB, align 16384)
- DMA `ring_sel=true` (ring on write address), `ring_size=14` (2^14 bytes)
- `incr_read=false` (fixed PIO FIFO address), `incr_write=true`
- DREQ = 12 (PIO1_RX0: PIO0 TX 0-3 + PIO0 RX 4-7 + PIO1 TX 8-11 + PIO1 RX 12-15)
- `trans_count = 0xFFFFFFFF` (runs for ~7.5 hours before stopping)
- Core 1 reads by comparing its read index to `DMA.ch(0).write_addr()`

Overrun detection: if distance from read to write exceeds half the ring,
DMA has lapped the reader. Reset read pointer to write pointer.

With a 256-entry ring, CSMA TX of larger frames (~50 bytes) causes the
ring to wrap multiple times, corrupting the read pointer. 4096 entries
(16KB) is sufficient for all frame sizes.

### Echo (Self-Receive)
With RE=GND, every frame we transmit is received back on RX. This echo
is captured in the DMA ring buffer and processed by the decoder.

**Key finding from AD2**: The echo signal is clean FM-0. Zero glitches,
correct edge intervals (2.10-4.40µs), DI-to-RO propagation delay
0-0.1µs. The decoder handles short echo frames (5-byte CTS) correctly.
Larger echo frames (63-byte GetStatus response) sometimes fail CRC —
likely accumulated timing drift over longer frames.

**Echo suppression approaches tried**:
1. Time-based skip (wait for idle gap after TX): hangs if data follows immediately
2. No suppression (decode everything): 1-byte garbage frames at DE edges, plus large frame CRC errors
3. Ring buffer reset after TX: works but discards data arriving during TX
4. Source-based filtering (`nodes.contains(frame[1])`): cleanly filters echo frames

The source-based filter is the most reliable: if the decoded frame's
source byte matches a node in our bitmap, it's our own echo. This
correctly handles both good-CRC and bad-CRC echoes.

1-byte garbage frames from DE enable/disable transients should be
suppressed by only forwarding frames with good CRC.

## TX Path

### Small Frame TX (Auto-Reply)
`build_immediate_tx_buffer` from tashtalk-proto allocates ~24KB on the
stack (HdlcBits 8KB + Fm0Buffer 16KB). This causes a stack overflow on
core 1 (48KB stack, but with other locals it's too tight — core 1 hard
faults silently on Cortex-M0+).

Solution: `build_small_frame_dma` builds HDLC + FM-0 directly into a
small static DMA buffer (~16 words) without large stack allocations.
Only suitable for 5-byte control frames (ACK, CTS).

### CSMA TX (Host-Initiated)
Core 0 pre-encodes the frame (HDLC + FM-0 + DMA word packing) into the
shared `TxSlot`. Core 1 picks it up when TX_SUBMIT is set, performs
CSMA wait, then transmits via DMA. For directed frames, the TxSlot also
contains a pre-encoded RTS frame.

### Directed TX (RTS/CTS/DATA)
Core 1 sends RTS (from pre-encoded buffer), then enters a tight polling
loop reading the DMA ring buffer for CTS. Local decoder state is used
(separate from the main loop's decoder). On CTS match, immediately
starts DMA of the pre-encoded DATA frame.

**Verified working**: 100% CTS success rate (100/100 in one test, 35/35
in another). Zero timeouts. The Mac receives our RTS and responds with
CTS. We then successfully transmit DATA.

## Current Status

### What Works
- RX decode from Mac traffic (hundreds of frames with good CRC)
- Auto-reply: ENQ→ACK and RTS→CTS (verified by counters and AD2)
- CSMA TX: broadcast and directed (GetStatus responses reach the Mac)
- RTS/CTS/DATA dialog: 100% CTS success, DATA transmitted
- TailTalk appears in Mac's Chooser
- GetStatus requests processed (Mac shows server name/icon)
- Directed ATP requests reach TailTalk (DirRX counter confirms)
- Zero CRC errors from TailTalk when only forwarding good-CRC frames
- Echo suppression via source-byte filtering works

### What Doesn't Work
- **AFP OpenSession never completes.** The Mac double-clicks TailTalk
  in Chooser, hangs, then reports "no response from the server."
  TailTalk's debug log shows it receives GetStatus (directed ATP) but
  never receives an OpenSession request. The Mac appears to never send
  the OpenSession ATP transaction, despite GetStatus working via the
  same directed path.

### Open Questions
1. Why does the Mac not send OpenSession after successful GetStatus?
   Both use directed ATP to socket 254. The firmware delivers both.
   Could the Mac require seeing specific LLAP control frames (ENQ/ACK
   for its own address) that we're not forwarding correctly?
2. Is there a timing requirement between GetStatus response and
   OpenSession that we're not meeting?
3. Does the Mac's ATP layer require seeing its own RTS echoed back?
   (The original TashTalk with RE=GND would echo RTS.)
4. Could the Mac be sending OpenSession but on a different socket than
   254, and TailTalk's DDP routing doesn't have a handler for it?

## Code Structure

### tashtalk-proto (no_std library, NOT modified)
Pure Rust protocol primitives. All synchronous, no async, no embassy.
- `fm0.rs`: FM-0 encoder/decoder
- `bitstuff.rs`: SDLC bit-stuffing/unstuffing
- `llap.rs`: Frame assembly, auto-reply logic, CRC validation
- `hdlc.rs`: HDLC frame builder (WARNING: 24KB stack usage)
- `backoff.rs`: CSMA/CA state machine
- `codec.rs`: TashTalk host byte protocol
- `crc.rs`: CRC-16 CCITT
- `nodemap.rs`: 256-bit node bitmap

### firmware/src/core1.rs
Core 1 entry point, shared state, DMA ring buffer, TX/RX, auto-reply.
This is where all the wire-level logic lives.

### firmware/src/main.rs
Entry point, core 1 spawn, Embassy executor, rx_bridge task.

### firmware/src/host.rs
Host protocol, frame pre-encoding, TX submission via SharedState.

### firmware/src/localtalk/phy.rs
PIO initialization with `mem::forget` to prevent Drop.
