# tashtalk-pico — development status

Last updated 2026-05-20.

Snapshot of the current state of every subsystem, the verification trail
behind it, and the known limits / open items. The bring-up plan and original
analysis live in [PICO_PORT_PLAN.md](PICO_PORT_PLAN.md); for protocol
background see the upstream TashTalk firmware (`tashtalk/firmware/one-chip.asm`).

## Bring-up scorecard

| # | Step | State |
| - | ---- | ----- |
| 1 | Host-side library tests (CRC, codec, bitstuff, nodemap, fm0, llap, backoff) | ✅ 46 tests pass |
| 2 | Firmware cross-compile (`thumbv6m-none-eabi`) | ✅ |
| 3 | Composite USB-CDC enumeration on Pico W | ✅ verified on macOS |
| 4 | Host command parser observable on TT_DEBUG | ✅ SetFeatures / SetNodeIds / TransmitFrame all decode live |
| 5 | Synthetic-frame round-trip on TT_DATA | ✅ `tools/synth_rx.py` validates CRC end-to-end |
| 6 | PIO TX FM-0 emission on GP2 | ✅ verified bit-perfect on the wire (logic analyzer) |
| 7 | PIO RX bring-up on self-loopback | ✅ full frame round-trips bit-perfect (GP2 → GP4 jumper) |
| 8 | Autonomous ENQ → ACK | ✅ end-to-end on hardware + verified on wire |
| 9 | CSMA/CA backoff | ✅ defer-and-backoff in front of every host-originated TX |
| 10 | RS-422 transceiver | ⏸ needs hardware |
| 11 | Two-Pico back-to-back | ⏸ needs hardware |
| 12 | Real Mac integration | ⏸ needs hardware |

## Subsystem state

### `tashtalk-proto` (host-testable library)

| Module | LOC | Role | Notes |
| ------ | --- | ---- | ----- |
| `crc.rs` | 219 | CRC-16/CCITT-false, table-based | Bit-identical to PIC's split `CrcLut1`/`CrcLut2` |
| `codec.rs` | 483 | Host byte-protocol parser + frame escape encoder | Drives `Sink` callbacks |
| `bitstuff.rs` | 281 | SDLC bit-stuff/unstuff + flag/abort detection | |
| `fm0.rs` | 265 | Stateful FM-0 decoder (sample → byte/flag/abort) | Has abort-resync |
| `llap.rs` | 437 | Frame assembler, `auto_reply`, `compose_ack/cts/rts` | |
| `nodemap.rs` | 127 | 256-bit node-ID bitmap | LSB-of-byte-0 = node 0 |
| `backoff.rs` | 278 | `BackoffState` — defer-and-backoff state machine | Ported from `PrepForNextFrame` / `Collided` / `Deferred` |

All 46 tests pass on the host with `cargo test`. The library is `no_std` and
shared with the firmware via the workspace.

### `firmware/` (Embassy / RP2040)

Composite USB device on Pico W with two CDC-ACM interfaces (IAD):

- **TT_DATA** (cu.usbmodem...1) — TashTalk byte protocol. Existing host drivers
  open this in place of `/dev/ttyUSB0`. The Pico's `CommandParser` expects the
  host to provide CRC bytes for both control and data frames; there is no
  "calc CRC for me" terminator. See the ENQ stimulus in `tools/wire_capture.py`
  for the correct on-wire payload format.
- **TT_DEBUG** (cu.usbmodem...3) — human-readable log + interactive shell.
  Commands: `p` (PIO TX a known frame), `b` (reboot to BOOTSEL), `?` (help).
  `l`/`a` are stubbed off because the continuous `lt_rx_task` now owns LtRx
  for the lifetime of the program.

Tasks running under the Embassy executor:

| Task | Role |
| ---- | ---- |
| `usb_run_task` | embassy-usb device driver |
| `host::run` | TT_DATA byte-protocol parser, outbound frame queue, optional periodic synthetic-frame TX |
| `debug::run` | TT_DEBUG log forwarder + command shell |
| `lt_rx_task::run` | Continuous PIO RX with ping-pong DMA, FM-0 decode, frame assembly, CRC validate, autoresponder |

### PHY: PIO TX & PIO RX (`lt_phy.rs`, 330 LOC)

- **PIO0 SM0** drives GP2 (and DE on GP3) as FM-0 at 230.4 kbit/s. Eight PIO
  cycles per bit cell. Side-set carries the clock-edge transition; a `mov pins`
  inside the cell handles the mid-cell transition for a `0`. The TX program
  has an explicit `jmp start` at the end of the `mid_zero` branch — without
  it, execution fell through past `.wrap` into the next PIO program's
  instructions, a bug that took a wire-capture session to diagnose.
- **PIO0 SM1** samples GP4 (loopback input) at 1.8432 MHz oversampling and
  pushes 32-bit words into the RX FIFO.
- **Ping-pong DMA** (`lt_rx_task.rs`): two 64-word buffers. One DMA channel
  (DMA_CH0) alternates between them. While DMA fills one, the CPU walks the
  other through `Fm0Decoder` → `FrameAssembler`. Zero per-word CPU overhead;
  eliminates the silent FIFO-overflow risk that polling-based capture had.
  Buffer size: 64 words × 16 samples/byte ≈ 555 µs of wire per buffer.

### CSMA/CA (`csma.rs`, 130 LOC)

`defer_and_backoff` runs in front of every host-originated TX:

1. `BackoffState::prep_for_next_frame()` — advances state for new frame attempt.
2. `next_backoff_units(entropy)` — computes IDG + (entropy & local_mask) in
   100 µs units.
3. Spin until `now - WIRE_LAST_ACTIVITY_US ≥ delay`.

`WIRE_LAST_ACTIVITY_US` is an `AtomicU32` (Cortex-M0+ has no 64-bit atomics)
holding a wrapping microsecond timestamp. `lt_rx_task` calls `mark_wire_active`
on every decoder event. The 4.29 billion-µs (71-minute) wrap is handled
correctly by `wrapping_sub`.

Autonomous replies (ACK / CTS) **bypass** the backoff — they mirror the PIC
ISR's immediate response. Per-frame RTS/CTS auto-pairing for outbound data
frames is **not** done in firmware (see "RTS/CTS" below).

### Self-diagnosing crashes

`firmware/src/main.rs` installs a custom `#[panic_handler]` that:

1. Writes file:line + the panic message into `PANIC_BUF`, a 512-byte buffer
   in a no-init `.uninit` section of RAM.
2. Calls `embassy_rp::rom_data::reset_to_usb_boot(0, 0)`.

Result: any panic reboots the Pico into BOOTSEL, showing up as an
`RPI-RP2` mass-storage volume. Pull `PANIC_BUF` out with `picotool save -r
<addr> <len>` for the post-mortem. This is invaluable without an SWD probe —
a panicking firmware shows up as a mounted volume instead of a silent brick.

## Verification trail

### ENQ → ACK round trip (step 8)

End-to-end test, driven by `python3 tools/wire_capture.py --stimulus enq`:

```
TT_DATA:  rx 6 bytes                            (host sent 01 42 01 81 5b f9)
HOST:     tx frame [ctrl] dest=42 src=01 type=81
HOST:     tx frame end (5 bytes total)
HOST:     PIO TX 5 bytes                        (CSMA defer satisfied, firing)
LT_RX:    frame #N len=5 d=42 s=01 t=81 crc=[5b f9] crc_ok=true   ← our ENQ on the wire
LT_RX:    -> reply [42 42 82]                   (autoresponder fires)
LT_RX:    frame #N+1 len=5 d=42 s=42 t=82       ← the ACK on the wire
```

The wire capture independently confirms the bytes on GP2 are bit-perfect:

```
[wire_capture] decode:
  flag offsets (preamble bytes): [..., ..., ...]
  payload bytes (unstuffed, LSB-first): 42 01 81 5b f9
    → dest=0x42 src=0x01 type=0x81
```

### Host-side library

```sh
cargo test
# test result: ok. 46 passed; 0 failed; 0 ignored
```

## Known limits & open items

### Firmware-side RTS/CTS dance is NOT done

LLAP requires the originator of a unicast data frame to send an RTS, wait
for the addressed node's CTS within ~200 µs, then send the data frame. The
PIC reference firmware does this inside its IOC interrupt with sub-µs
response time.

This firmware's DMA-buffered RX architecture has a minimum edge-on-wire to
decoded-event latency of about **1.1 ms** (one DMA buffer's worth of samples).
That's far past LLAP's 200 µs CTS timeout, so reproducing the dance in
firmware would be ineffective against a real Mac. The PHY therefore transmits
whatever the host hands it (after CSMA defer + backoff) and trusts the host
driver — TailTalk's `tashtalk` crate or equivalent — to construct the RTS /
Data pair if it needs LLAP-strict timing. Autonomous reply to *incoming* ENQ
and RTS frames is still done in firmware, as in the PIC original.

The pieces needed for a firmware-side dance (`compose_rts`,
`cts_matches_pending`, `PENDING_RTS_*` atomics, `CTS_RECEIVED` signal) are
implemented but not wired into the TX path. They can be enabled if the
architecture moves to a smaller-buffer or IRQ-driven RX in the future.

### Synthetic ticker temporarily disabled

`firmware/src/host.rs` has the every-2s synthetic-frame TX gated behind
`if false { ... }` while wire-capture work is the focus — the periodic data
frames flooded `lt_rx_task`'s log and obscured targeted ENQ→ACK traces.
Re-enable by deleting the `if false` block; the synthetic-frame TX-only path
on TT_DATA (encoded → host stream) is still active when `cdc.dtr()` is true.

### Wire-capture window is short of a full data frame

The current `la.dwf3work` workspace is 4096 samples at 6.25 MHz = **655 µs**
of capture. A full 10-byte LLAP data frame (3 preamble flags + 10 bytes + 1
trailing flag + abort) on the wire is ~580 µs, so it fits — but combined with
the workspace's ~10% pre-trigger, the post-trigger window is only ~590 µs and
the tail of a data frame can run off the end. To capture an ENQ + autoresponse
ACK in a single trace (the two frames span ~700-1500 µs depending on
inter-frame gap), bump the LA buffer to 8192 samples or drop the rate to 3.125
MHz and re-save the workspace.

### Apple's `AppleUSBFTDI` blocks SDK-direct device access

Programmatic access to the Analog Discovery 2 via Python ctypes against the
bundled libdwf fails with `Adept NOK / ERC 3090`. Root cause: Apple's
DriverKit `AppleUSBFTDI` extension auto-claims interface 0 of the FT232H
inside the AD2 and creates `/dev/cu.usbserial-...`. libusb-based access from
a generic process can't claim that interface while the DriverKit driver owns
it. WaveForms.app itself works because it maintains the libdpti state /
mutexes at `/private/tmp/digi-ad2-mtx-*` and the surrounding process context
that the SDK depends on — not because of any special entitlement (it has
none beyond audio/JIT/network).

Confirmed against WaveForms 3.25.1 (current latest, March 2026). Same symptom
reported on forum thread #31020 (Feb 2025, Flutter SDK use case) with no
resolution. Closed-loop capture is therefore done via Accessibility-API
automation of WaveForms.app against the checked-in workspace.

## Hard-won bring-up lessons

- `embassy-usb-cdc-acm`'s `wait_connection()` returns on `SET_CONFIG` (USB
  enumeration), **not** on DTR. So a device-side task that wakes on
  `wait_connection()` runs even when no host application has opened the port.
  Use `cdc.dtr()` to gate optional/heavy device → host traffic.
- `cdc.write_packet()` on an IN endpoint with no host reader **hangs** the
  calling task, and that hang can prevent USB enumeration from completing
  cleanly. Symptom looks like "device is completely dead" even though no
  panic occurred.
- embassy's default task arena is 4 KB. With our task fan-out this overflows,
  triggering `defmt::default_panic`. Bumped to 32 KB via the
  `task-arena-size-32768` feature in `embassy-executor`.
- The custom `#[panic_handler]` reboots to BOOTSEL. Combined with the no-init
  `PANIC_BUF`, every panic gives you a post-mortem without an SWD probe.
- Press `b` on TT_DEBUG to reboot into BOOTSEL — no more unplug / hold-BOOTSEL
  / replug dance.
- The PIO TX `mid_zero` branch needed an **explicit** `jmp start` at the end;
  falling off the end of `.wrap` ran into adjacent PIO instructions and
  produced sporadic timing glitches. Symptom: occasional bit drops on TX.
- Diagnosing PIO RX FIFO overflow needs the right FDEBUG bit: `RXSTALL` lives
  at bits 0-3 (one per SM), `TXSTALL` at 24-27. Reading the wrong bit
  ("nothing wrong here") wasted real time.
- The **ENQ "intermittent failure"** that occupied step 8's first attempt
  was a **test-stimulus bug, not a firmware bug**: the probe sent
  `\x01\x42\x01\x81\xfe` — one byte short, missing the second CRC byte —
  which left `CommandParser` stuck mid-frame and misaligned every subsequent
  injection. The fix is host-side: send `\x01\x42\x01\x81\x5b\xf9` with the
  CRC pre-computed (`lt_crc([0x42,0x01,0x81]) = [0x5b, 0xf9]`).

## Pin assignments

| Pin | Role |
| --- | ---- |
| GP2 | LocalTalk TX (FM-0 from PIO0 SM0) |
| GP3 | RS-422 DE (driver enable, active high) |
| GP4 | LocalTalk RX (FM-0 to PIO0 SM1) — loopback jumper from GP2 during bring-up |
| GP25 | onboard LED |

Steps 10-12 will need an RS-422 transceiver (e.g. ADM3485, SP485) and a
DIN-8 LocalTalk connector. GP2/GP3/GP4 are routed in software; the
transceiver-driving signals are GP2 (TX_DATA) and GP3 (DE).

## What's next when hardware arrives

1. Wire the RS-422 transceiver. Validate that GP2/GP3 drive A/B correctly,
   that DE rises before TX data and falls after, and that the receiver still
   produces clean FM-0 on GP4.
2. Two-Pico back-to-back over RS-422. Same firmware, same node ID bitmap
   pre-set to different IDs. Inject ENQ from one host's TT_DATA, observe ACK
   coming back through the other Pico's wire RX.
3. Connect to a real Mac LocalTalk segment. Verify NBP / ATP / DDP traffic
   flows end-to-end with TailTalk on the host.
