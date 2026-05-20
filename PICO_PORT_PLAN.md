# Porting TashTalk to the Raspberry Pi Pico (USB-CDC)

This document captures (1) an analysis of the existing PIC12F1840 TashTalk
firmware in `firmware/one-chip.asm`, and (2) a plan for re-implementing the
same function on a Raspberry Pi Pico (RP2040 / RP2350), presenting the host
interface as **USB-CDC** rather than a 1 Mbaud UART.

The host-facing byte protocol described in `documentation/protocol.md` is
preserved verbatim. Any existing TashTalk host driver — `tashtalkd`,
TashRouter, MultiTalk, TailTalk's Rust `tashtalk` crate, PicoATP — should
talk to the Pico build without modification, simply opening
`/dev/ttyACM0` (or `COMx`) instead of `/dev/ttyUSB0`.


## 1. Analysis of the existing firmware

### 1.1 Target chip and timing budget

- PIC12F1840 (mid-range core, 8-pin SOIC, ~$1.50).
- Internal 32 MHz oscillator with 4× PLL → **8 MIPS, 125 ns per instruction**.
- LocalTalk runs at **230.4 kbit/s**, so one bit cell ≈ **4.34 µs ≈ 34.7
  instruction cycles**.
- Nearly every line of the transmitter and receiver in `one-chip.asm`
  carries a numeric cycle count (`;00`, `;01`, …, `;34`) because the entire
  design is hand-scheduled to that 35-cycle bit cell.

### 1.2 Pin usage

From `Init` in `one-chip.asm` and `documentation/pinout.md` (v2.x):

| Pin  | Function |
| ---- | -------- |
| RA5  | LocalTalk bus, bidirectional via external RS-422 transceiver. The TX side never writes `PORTA` to drive — it pre-loads `LATA<5>` with the current bit and toggles between *output* and *tristate* with `tris 5`, while edges are produced by `xorwf LATA,F` with mask `0b00100000` (single-instruction "invert RA5"). |
| RA4  | RS-422 driver-enable, gated high while transmitting. |
| RA3  | !MCLR (v2.x). |
| RA0  | Host UART TX (1 Mbaud, 8N1). `BAUDCON=0x48` selects BRG16+BRGH; `SPBRGL=7` gives 8 MHz / (4·8) = 1 MHz. |
| RA1  | Host UART RX. |
| RA2  | CTS-to-host. Set/cleared by writing `0b00000100` into `LATA` whenever `UR_LEN` crosses 64 (queue half-full / half-empty). |

### 1.3 Peripherals used

- **EUSART**: UART RX runs under interrupt (`RCIF`), UART TX is polled.
- **Timer1**: free-running; used purely as a cheap entropy source. The
  backoff in `SetTimer2` is `(TMR1H ^ TMR1L) & LBACKOFF`.
- **Timer2**: 1:16 prescale + 1:10 postscale → ticks every 20 µs. Reused
  for inter-dialog and inter-frame gap timing (`PR2` programmed in
  multiples of 100 µs / 200 µs).
- **Interrupt-on-change on RA5** (`IOCAP5`, `IOCAN5`): every LocalTalk edge
  sets `IOCAF5`. This is the bit-clock recovery mechanism — see receiver
  description below.
- No SPI/MSSP, no CLC, no NCO, no DMA. Everything is bit-banging.

### 1.4 RAM map (192 bytes; the chip really is full)

- 16 bank-0 globals: `FLAGS`, `LR_*`, `UR_LEN`, `GBACKOFF`, `LBACKOFF`,
  `COL_HIST`, `DEF_HIST`, `ATTEMPTS`, `FEATURES`, scratch.
- 32-byte node-ID bitmap in bank 1 (`NODEBM0..31`).
- 5 LT-TX shadow registers + send buffers (`UR_DEST`, `UR_SRC`, …,
  `LT_BUF`, `LT_ONES`) in bank 2.
- 5 LR shadow registers at top of bank 2 (`LR_DEST`, `LR_SRC`, `LR_TYPE`,
  `LR_CRC1`, `LR_CRC2`).
- **128-byte circular UART-RX queue** at linear `0x2000-0x207F`. `FSR0` is
  the push pointer (saved across IRQs via `FSR0L_SHAD`), `FSR1` is the pop
  pointer. The 7-bit wrap is done by `bcf FSR{0,1}L,7` after each
  post-increment.

### 1.5 CRC

Two **256-byte CRC lookup tables, page-aligned** at `org 0x600` (`CrcLut1`)
and `org 0x700` (`CrcLut2`). This is the standard CCITT-16 used by SDLC /
LLAP. The split into two tables is a PIC code-page workaround so each table
fits below a page boundary and can be entered via `movlp` + `callw`. The
post-frame residue check looks for `LR_CCRC1 == 0xB8 && LR_CCRC2 == 0xF0`
(`InFlag`), which is the standard CCITT residue.

### 1.6 Transmitter structure (`Send*` family)

- `SendSyncPreamble` drives a 1-bit sync pulse low, then idles ~10 bit
  times before the flag bytes.
- `SendPreamble` emits **three** `0x7E` flag bytes (`LT_ONES = 24` rotation
  count = 3 × 8 bits). `documentation/other_info.md` notes this is a
  IIgs-compatibility requirement.
- `SendByte` is **8 copies of the same hand-unrolled 35-cycle stanza**, one
  per bit. Each stanza:
  - Cycle 00: toggle `LATA<5>` (mandatory mid-cell clock edge — this *is*
    the FM-0 / differential-Manchester clock).
  - Cycle 17: conditionally toggle again to encode a `0`.
  - Cycles 18-23: update `LT_ONES` consecutive-ones counter; call
    `SendByteStuff` after 5 consecutive ones.
  - Cycles 24-31: CRC update via `CrcLut1` / `CrcLut2`, using `movlp` +
    `callw` to enter the page-aligned table.
  - Cycles 03-14: `SendDoUartSvc` polls `RCIF` so host UART bytes never
    drop during TX, and updates CTS if `UR_LEN ≥ 64`.
- `SendFromQueue` is `SendByte` plus queue-pop machinery and the trailing
  CRC overstrike used when `FEATURES.CALCCRC` is set.
- `SendControlFrame`, `SendRts`, `SendCts`, `SendAck` are five-byte
  variants for 0x84/0x85/0x82 control frames.
- `SendPostamble` ends with a flag byte then 13 ones then either tristates
  directly or, if the bus would otherwise be left high, drives a final 0
  before tristating (`SendPoZ`). **This detail matters**: certain devices
  (LaserWriters, Power Mac 6100; v2.1.3 / v1.0.1 release notes) misbehave
  if the bus is yanked from `1` → `Z`.

### 1.7 CSMA/CA and backoff

`PrepForNextFrame` / `TryAgain` / `Collided` / `Deferred` /
`WaitForCts` / `WaitForSilence` implement *Inside AppleTalk*'s
collision-avoidance:

- Two 8-bit history shift registers — `COL_HIST` (collisions) and
  `DEF_HIST` (deferrals) — hold the last 8 outcomes.
- Global backoff mask `GBACKOFF` is grown left (up to 4 bits) when there
  have been >2 collisions in the last 8 frames, and shrunk right when
  there have been <2 deferrals.
- Local backoff `LBACKOFF` starts from `GBACKOFF`, grows on every
  collision within this transmission, and seeds the per-attempt random
  delay (`(TMR1H ^ TMR1L) & LBACKOFF`, then `+4` for the 400 µs IDG
  minimum).
- `ATTEMPTS = 32`: up to 32 retries per outbound frame.

### 1.8 Receiver structure (`LtReceiver`, IOC ISR)

This is the cleverest part of the firmware:

- The IOC interrupt vector *is* the receiver — once an edge fires, control
  does not return to the main loop until the frame ends or the clock is
  lost. There is no main-loop sampling.
- **Clock recovery** is the loop at `OutCheckInv` / `EmpCheckInv` /
  `InCheckInv`: after clearing `IOCAF5`, the code polls it across roughly
  one bit cell. Detecting an extra inversion in the second half of the
  bit cell means the bit was `0` (FM-0 mid-bit transition); no inversion
  means `1`. The decoded bit is carried into `STATUS,C`.
- **State machines**: `OutReceive` / `EmpReceive` / `InReceive` dispatch
  via computed `goto` into one of three hand-written FSAs that live in
  page-aligned tables starting at `org 0xA00`:
  - `Os*` — out-of-frame: waiting for a flag byte.
  - `Es*` — empty-frame: just past a flag, waiting for either another
    flag or the first data bit.
  - `Is*` — in-frame: assembling a data byte.
  - Cell names encode (state index, consecutive-ones, last bit).
- Each cell handles flag detection (`01111110`), bit-unstuffing (drop a
  `0` after five `1`s), abort detection (seven `1`s), and full-byte
  handoff to `InByte` / `InSecond` / `InFourth` / `InSixth` which feed
  the CRC LUTs and emit decoded bytes to the host UART, escaping a
  literal `0x00` as `00 FF`.
- `InFlag` is the normal end-of-frame path: validates CCITT residue,
  emits `00 FD` (or `00 FC` if CRC failed and `FEATURES.CHKCRC` is set),
  and sets `LR_FRM` so mainline `DealWithFrame` can autonomously reply
  to ENQ / RTS / CTS frames addressed to a node ID in the bitmap.
- `InLostClock` emits `00 FA` (aborted), `*FErr` emits `00 FE` (framing
  error).

### 1.9 Cross-cutting design constraint

Every tight loop in the firmware interleaves a UART-RX poll
(`PIR1,RCIF`) and a CTS update (`UR_LEN,6` → write to `LATA`). That is
how the firmware can claim to accept host bytes *at all times*, including
during active LocalTalk TX and RX. On a 1 Mbaud UART that's ~10 µs/byte,
and the longest interval without an RX poll is one half of a bit cell.
This pervasive interleaving is the single biggest reason the firmware is
so densely hand-scheduled, and it is the thing the RP2040 port can most
cleanly eliminate.


## 2. Port plan: Raspberry Pi Pico over USB-CDC

### 2.1 Why this port is interesting

On the PIC, 3107 lines exist to bit-bang one differential-Manchester line
against a 35-cycle budget per bit, with UART servicing wedged into every
spare cycle. The RP2040 has **PIO**, which was designed for exactly this
problem — dedicated 1-cycle-deterministic state machines that run on
their own clock divider, in parallel with the Cortex-M0+ cores, with
DMA-able FIFOs.

The port is therefore not a 1:1 rewrite but a re-architecture:
**PIO replaces the FM-0 encoder, bit-stuffer, and the IOC-driven
state machine; the M0+ replaces the PIC mainline; native USB-CDC
replaces the 1 Mbaud UART (and the CP210x bridge that the existing
TashTalk USB carrier needs).**

The host-facing byte protocol is preserved bit-for-bit: same command
opcodes 0x00–0x03, same escape sequences 0x00 0xFD / 0xFC / 0xFE /
0xFA / 0xFF, same node-ID bitmap layout, same features bits. Existing
host drivers don't need to know it isn't a PIC.

### 2.2 Target hardware

- **Raspberry Pi Pico** (RP2040). Pico 2 (RP2350) is a drop-in upgrade
  if you want faster PIO and more SRAM, but it isn't required.
- Same external RS-422/485 transceiver family as TashTalk v2.x:
  SN65HVD08 / SN75HVD12 / ISL83076E / GM3085E (see
  `documentation/transceivers.md`). The "receiver must float high when
  the bus is idle" requirement carries over unchanged.
- **3 GPIOs** for LocalTalk side:
  - `LT_DATA` — bidirectional to the transceiver `R` / `D` pins.
  - `LT_DE` — driver-enable; ties to transceiver `DE` and (active-low)
    `!RE`.
  - (optional) `LT_RX_SAMPLE` — only needed if you want to sample on a
    separate pin from `LT_DATA`; usually unnecessary.
- USB connector for the host interface. No CP210x, no UART pins, no
  CTS GPIO. Power can come from USB.

### 2.3 USB-CDC vs. the 1 Mbaud UART: what changes

| Concern | PIC + CP210x (existing) | RP2040 + USB-CDC (proposed) |
| --- | --- | --- |
| Host wire | 1 Mbaud UART through CP210x | Native USB Full-Speed (12 Mbit/s) bulk CDC ACM |
| Sustained host bandwidth | ~100 KB/s | ~1.2 MB/s (10× headroom) |
| Latency | byte-at-a-time, ~10 µs/byte | bulk packets, ~1 ms scheduling granularity |
| Flow control | RTS/CTS line, software-managed threshold at `UR_LEN ≥ 64` | USB hard-NAKs OUT packets when the device buffer is full — no app code needed |
| Driver on host | CP210x kernel driver + raw `tty` | Standard CDC-ACM driver, present in every OS |
| `tashtalk` Rust crate compatibility | Works as-is | Works as-is (still opens a serial port; just `/dev/ttyACM*`) |
| `documentation/protocol.md` (byte-level) | Authoritative | Authoritative — **unchanged** |

The byte-level protocol is preserved verbatim, including the 0x00/0xFD
escape sequences. Two notes:

1. The "queue half-full / half-empty CTS" mechanism described in
   `protocol.md` becomes a no-op visible to the host: the device simply
   stops ACKing OUT packets when its software queue passes the high
   watermark. From the host's point of view its writes block briefly,
   which is exactly what would have happened under RTS/CTS anyway.
2. The PIC firmware caveat "the firmware does not keep an entire frame
   in memory at once" disappears. With 264 KB SRAM on RP2040, we can
   trivially buffer the entire 605-byte maximum LocalTalk frame before
   keying the transmitter, eliminating the underrun risk entirely.
   This is a real reliability win.

### 2.4 Module layout (Rust on Embassy)

The actual implementation lives in this repository at the path layout below,
which **slightly differs from the original sketch** to enable host-side
unit-testing of the byte-level state machines:

```
tashtalk-pico/
├── Cargo.toml                       workspace (default-members = tashtalk-proto)
├── PICO_PORT_PLAN.md                this document
├── README.md                        quickstart + bring-up status
│
├── tashtalk-proto/                  no_std + host-testable library
│   └── src/
│       ├── lib.rs
│       ├── crc.rs                   CCITT-16 with PIC's split-table layout cross-tested
│       ├── codec.rs                 host byte protocol (parser + escape encoder)
│       ├── bitstuff.rs              SDLC bit-stuff / unstuff, flag & abort detection
│       └── nodemap.rs               256-bit node-ID bitmap
│
└── firmware/                        embassy-rp binary, thumbv6m-none-eabi
    ├── Cargo.toml
    ├── memory.x, build.rs, rust-toolchain.toml, .cargo/config.toml
    └── src/
        ├── main.rs                  init, spawn tasks
        ├── usb_dev.rs               composite USB (IAD) with 2× CDC-ACM
        ├── host.rs                  TT_DATA task: parser → LtPhy
        ├── debug.rs                 TT_DEBUG task: banner + (eventually) shell
        ├── lt.rs                    LtPhy trait, RxFrame/RxStatus
        ├── pio_tx.rs                FM-0 encoder + bit-stuffer (stub)
        └── pio_rx.rs                FM-0 decoder + bit-unstuffer (stub)
```

Putting the byte-level state machines (CRC, parser/encoder, bit-stuffer,
nodemap) in a separate `no_std` library means `cargo test` from the
workspace root runs all of them on the host with zero embedded toolchain
involvement. This is critical for the bring-up plan in §2.8 because we
have no TashTalk reference board to capture known-good traces from.

### 2.5 Mapping each PIC concept to the RP2040

| PIC concept | RP2040 equivalent |
| --- | --- |
| 35-cycle hand-scheduled bit cell on RA5 | One PIO state machine clock-divided to 230.4 kHz (TX) or 8× oversampled at 1.8432 MHz (RX); `side_set` pin = `LT_DE`, `OUT` pin = `LT_DATA` |
| `xorwf LATA,F` clock edge plus conditional `xorwf` data edge | PIO `out pins, 1` plus a conditional `jmp` to toggle midpoint — FM-0 fits in ~6 PIO instructions per bit |
| Bit-stuffing after 5 ones (`SendByteStuff`) | PIO `Y` countdown; on overflow, inject a `0` and reset. Or do it in software before pushing to the TX FIFO |
| Pre/postamble flag bytes 0x7E ×3 | Pre-loaded into the TX FIFO by the driver, not done in PIO |
| IOC ISR + three hand-written FSA tables (`OutFSA`/`EmpFSA`/`InFSA`) | RX PIO oversamples at 4–8× bit rate, recovers FM-0 clock from mid-bit transitions, does bit-unstuffing, pushes assembled bytes plus a side-band status word (flag / abort / frame-done) to the RX FIFO |
| `SendDoUartSvc` interleaved every 12 cycles | **Gone.** USB-CDC is a separate hardware path with its own DMA. The PIO state machines don't service USB at all |
| 128-byte software FIFO + manual CTS at `UR_LEN ≥ 64` | DMA into a 1–4 KB ring; USB stack auto-NAKs OUT packets when the ring is full. No watermark code in the LocalTalk path |
| Timer1 entropy for random backoff | RP2040 ROSC random output, or `time_us_64()` low bits |
| Timer2 for 100/200 µs IDG/IFG | `embassy_time::Timer` or a hardware alarm |
| `CrcLut1` + `CrcLut2` (split for PIC page layout) | A single 512-byte CCITT-16 table in flash, or bitwise (M0+ is plenty fast). The `0xB8 0xF0` residue check is identical |
| `DealWithFrame` autonomous ENQ/RTS/CTS handling | Plain Rust function on core0, gated by the node bitmap |
| `FEATURES.CALCCRC` / `CHKCRC` (v2.1) | Same semantics; one bit each in a config word |
| `SendPostamble` "drive final 0 before tristate" (`SendPoZ`) | Mirror exactly: final PIO instruction drives 0, then side-set drops `LT_DE`, then `OUT pindirs` tristates `LT_DATA`. v2.1.3 / v1.0.1 compatibility fix preserved |

### 2.6 PIO sketches (intent, not final code)

**TX PIO** — one state machine, clocked so that 1 PIO instruction = ⅛ bit
cell, `side_set` drives `LT_DE`:

```
.side_set 1
.wrap_target
    out  x, 1     side 1            ; pull next bit, DE held high
    set  pins, 1  side 1            ; first half-cell (always inverts prev)
    nop           side 1 [3]
    jmp  !x, bit0 side 1
    nop           side 1 [3]
    jmp  next     side 1
bit0:
    set  pins, 0  side 1 [3]        ; mid-cell toggle for a 0
next:
.wrap
```

A Rust helper feeds the TX FIFO with bytes that have already been
bit-stuffed and flag-framed. The bit-stuffer either runs as a second PIO
SM or as a tiny software shifter feeding the first SM via DMA — the PIC
does it inline, but on RP2040 either approach is cheap.

**RX PIO** — free-runs at 8× bit clock on `LT_DATA`, uses `jmp pin` and a
`Y`-register loop to count cells between transitions; emits to the RX
FIFO either a decoded data byte or a side-band marker for
`FLAG` / `ABORT` / `FRAME_END` / `CRC_RESIDUE`. This collapses the
three-FSA design of the PIC because PIO has a real loop counter and
doesn't need page-aligned dispatch.

### 2.7 USB-CDC host interface — composite from day 1

Two CDC-ACM interfaces in one composite USB device, both built into the
firmware from the very first commit:

- **TT_DATA** (interface 0+1) — the TashTalk byte protocol. Existing
  host drivers (TailTalk's `tashtalk` crate, `tashtalkd`, TashRouter,
  MultiTalk) point at this port instead of `/dev/ttyUSB0`. Wire protocol
  identical to `tashtalk/documentation/protocol.md`.
- **TT_DEBUG** (interface 2+3) — human-readable log + interactive
  command shell. *This is the bring-up substitute for a logic analyzer.*
  Because we don't have an existing TashTalk board to cross-check
  against, every interesting internal event — parsed commands, PIO
  FIFO state, frame round-trips on the internal GPIO loopback — is
  observable here. The host opens it with `cat /dev/ttyACM1` (or
  similar) during development.

Both interfaces use the OS-stock CDC-ACM driver (Linux, macOS, Windows
10+). To declare them as one logical device, the firmware sets the
composite-IAD device class (`0xEF / 0x02 / 0x01`) and emits an
Interface-Association Descriptor before each CDC pair — `embassy-usb`'s
`Config::composite_with_iads = true` is what flips that on.

The bidirectional byte stream carried on that virtual port is identical
to what the PIC emits over its UART:

- **Host → device** opcodes (`documentation/protocol.md` §"Host to TashTalk"):
  - `0x00` Noop.
  - `0x01` `<frame bytes…>` — TransmitFrame. Length inferred from LLAP
    header (control = 5 bytes; data = header + length field).
  - `0x02` `<32 bytes>` — Set node-ID bitmap.
  - `0x03` `<1 byte>` — Set features (bit 7: CRC calc; bit 6: CRC check).
- **Device → host** byte stream (`documentation/protocol.md` §"TashTalk to Host"):
  - Decoded frame bytes, with `0x00` → `0x00 0xFF`.
  - `0x00 0xFD` end-of-frame (good).
  - `0x00 0xFC` end-of-frame (bad CRC, only when CHKCRC is set).
  - `0x00 0xFE` framing error.
  - `0x00 0xFA` frame aborted.

The encoder/decoder is the same state machine that already lives at
`tashtalk/src/codec.rs` in the TailTalk repo — that crate's
`TashTalkCodec` will talk to this firmware unchanged, which is the
single best test harness available.

### 2.8 Bring-up order — no-board variant

We do not have a TashTalk reference board to capture known-good traces
from. The strategy therefore is:

- **Validate every byte-level state machine on the host first**, using
  `tashtalk-proto`'s unit tests. The CRC is cross-checked against
  TailTalk's reference port (`tashtalk/src/crc.rs`), which has its own
  round-trip tests against the same spec.
- **Use the Pico as its own reference.** A single Pico can self-loop
  through PIO TX → wire → PIO RX, which validates the wire-level
  encoding without any other LocalTalk device on the bus. TT_DEBUG
  shows the round-trip results.
- **Defer everything that needs an RS-422 transceiver** (bus-to-bus
  bridging, real Mac integration) to a later phase. The transceiver is
  a single SOIC-8 part — easy enough to acquire — but the steps below
  don't block on it.

| Step | Hardware needed | Validated by |
| ---- | --------------- | ------------ |
| 1. **Host-side library tests** — CRC residue, escape encoder, command parser, bit-stuff round-trip, nodemap wire layout. | None. | `cargo test -p tashtalk-proto` (22 tests; passing on first commit). |
| 2. **Firmware cross-compile** — confirm Embassy + USB + the proto crate link cleanly for `thumbv6m-none-eabi`. | None (rustup + thumbv6m target). | `cd firmware && cargo check`. |
| 3. **Composite USB-CDC enumeration** — flash the day-1 firmware, confirm two `/dev/ttyACM*` devices appear, banner shows on TT_DEBUG, echo works on both. | Pico + USB cable. | `cat /dev/ttyACM1` shows boot banner; bytes typed into one port echo back. |
| 4. **Host command parser wired to TT_DATA** — TailTalk's `tashtalk::TashTalkCodec` writes `SetNodeIds`/`SetFeatures` to TT_DATA; firmware decodes them and prints a summary to TT_DEBUG. | Pico + USB cable. | Visual: log line on TT_DEBUG matches the command the host sent. Can run from TailTalk's existing examples by pointing them at `/dev/ttyACM0`. |
| 5. **Synthetic-frame round-trip on TT_DATA** — firmware crafts a known LocalTalk frame on the device side, escapes it, and sends it to the host via TT_DATA's IN endpoint. Host decodes with `TashTalkCodec` and CRC-checks. | Pico + USB cable. | Round-trip succeeds for randomized payloads; exercises the escape encoder under real USB scheduling. |
| 6. **PIO TX bring-up on self-loopback** — wire `LT_DATA` to `LT_DATA_RX_PROBE` via a jumper. Encoder PIO state machine drives FM-0; a second PIO SM samples and logs cell timings to TT_DEBUG. Compare timing against the spec. | Pico + 1 jumper wire. | Bit cell ≈ 4.34 µs; flag bytes correctly framed; bit-stuffing visible after any 5-ones run. |
| 7. **PIO RX bring-up on self-loopback** — same wire, now the second PIO SM is the real FM-0 decoder. Crafted frames round-trip TX → wire → RX → host comparison. | Pico + 1 jumper wire. | Random 605-byte payloads round-trip with no byte loss; flag, abort, and CRC residue all reachable in tests. |
| 8. **`DealWithFrame` port** — autonomous ENQ→ACK and RTS→CTS replies, gated on the node bitmap. Tested by emitting an ENQ on the self-loopback and watching for ACK. | Pico + 1 jumper wire. | TT_DEBUG shows ACK emission; round-tripped ACK CRC-validates. |
| 9. **CSMA/CA + backoff** — port `PrepForNextFrame`/`TryAgain`/`Collided`/`Deferred`/`WaitFor*` from the asm. Validate against synthetic collisions injected on the self-loopback. | Pico + 1 jumper wire. | Backoff history shrinks/grows under simulated collision patterns matching the PIC behaviour. |
| 10. **RS-422 transceiver** — solder up a breakout (SN65HVD12 or similar from `tashtalk/documentation/transceivers.md`), drive `LT_DE`/`LT_DATA` through it. | + SN65HVD12 + ~3 passives. | Scope check: differential pair swings cleanly, transceiver tristates between transmissions. |
| 11. **Two-Pico back-to-back over RS-422** — both Picos run this firmware; frames cross a CAT5 cable. | + 2nd Pico + 2nd transceiver. | One Pico's TT_DEBUG shows frames originated by the other. |
| 12. **Real Mac integration** — plug a real LocalTalk-speaking Mac into the bus. | + Mac + LocalTalk cabling. | NBP lookup from `tailtalk-gui` sees the Mac; AFP copy succeeds. |

Steps 1–3 do not need any hardware beyond the Pico already in hand.
Steps 6–9 need exactly one jumper wire. Steps 10–12 add the RS-422
front-end and (eventually) a Mac.

> **Status after first hardware test (Pico W on macOS):** steps 1–4 are
> green on real hardware. Both CDC interfaces enumerate as
> `/dev/cu.usbmodem*` on macOS, the TT_DEBUG banner appears on connect,
> and the live parser correctly decodes the three TashTalk opcodes
> (0x01 TransmitFrame, 0x02 SetNodeIds, 0x03 SetFeatures), with
> control-vs-data frame discrimination working from the LLAP type byte's
> high bit. The next step is 5 (synthetic device-originated frames going
> up to the host through TT_DATA), which exercises the escape encoder
> under real USB scheduling and is still pure software — no PIO needed.

### 2.9 Risks and watch-outs

- **Tristate timing in the postamble.** `SendPoZ` deliberately drives a
  final 0 before tristating so the bus doesn't get yanked
  `1` → `Z` → `1`. The equivalent PIO sequence must preserve this or
  printers / Apple IIgs / Power Mac 6100 will misbehave (v2.1.3 / v1.0.1
  release notes).
- **DE-to-data skew.** The transceiver's t<sub>EN</sub> must be satisfied
  before the first data edge. On the PIC, `tris 5` happens at a known
  cycle. On RP2040 the `side_set` on `LT_DE` must lead the first PIO
  data edge by enough margin — verify on a scope.
- **Transceiver idle level.** `transceivers.md`'s "receiver output must
  float high when bus idle" requirement is unchanged. Stick to the
  shortlisted parts.
- **Frame-arriving-during-transmit handling.** The PIC ISR clears
  `T2CON.TMR2ON` on any LocalTalk activity during a TX-deferral wait,
  and the mainline notices and reacts. RP2040 equivalent: a PIO IRQ
  from the RX SM whenever it sees a flag mid-deferral, feeding the same
  `WaitForCts` / `WaitForSilence` logic.
- **USB suspend.** A host that suspends USB will silently stop draining
  the device's IN endpoint. Pick a sensible policy: either keep
  receiving and overflow gracefully (drop oldest frame, signal a
  framing error on resume), or pause the LocalTalk receiver until
  resume. Document whichever you pick.
- **USB latency vs. real-time response.** USB-CDC delivers in ~1 ms
  bulk frames, so a host trying to respond to an ENQ via
  software-implemented control frames would be far too slow. This is
  fine because TashTalk *itself* answers ENQ / RTS / CTS autonomously
  (`DealWithFrame`); host software only sees data frames. Preserve
  that property in the port.
- **License.** TashTalk is GPLv3 (`LICENSE`). A clean-room rewrite to a
  different MCU is permitted, but the CRC tables and the protocol byte
  values are inherited. Keep clear attribution to Tashtari /
  `lampmerchant`.

### 2.10 Stretch goals once the basic port works

- **Two LocalTalk PHYs on one Pico.** RP2040 has 8 PIO state machines
  total. Two transceivers + two TX/RX SM pairs gives you a bus-to-bus
  bridge in firmware, no host needed.
- **Onboard pcap.** Ring-buffer recent LocalTalk frames in SRAM and
  expose them as a USB MSC partition or as a third CDC interface
  alongside TT_DATA/TT_DEBUG. This is a serious debugging gift compared
  to the PIC's "no introspection at all" design, and the TT_DEBUG
  channel is already in place to grow into it.
- **defmt over TT_DEBUG.** Currently `defmt-rtt` requires a probe;
  routing `defmt` through TT_DEBUG (e.g. via `defmt-bbq`) would make
  the firmware self-debugging over the same USB cable that carries
  data, removing the last reason a developer needs an SWD probe.
- **Web-USB / WebHID config tool.** Native USB makes a browser-based
  setup and node-bitmap editor trivial; the PIC never had this option.

### 2.11 Estimate

Roughly, with no TashTalk reference board:

- Steps 1–2 (host tests + cross-compile): done.
- Steps 3–5 (composite USB-CDC + command parser + synthetic round-trip):
  ~1 evening once a Pico is flashed.
- Steps 6–7 (PIO TX + RX on self-loopback): ~2 weekends. PIO debugging
  dominates; TT_DEBUG observability shortens the loop significantly.
- Steps 8–9 (autonomous control frames + CSMA/CA): ~1 weekend.
- Steps 10–11 (transceiver + two-Pico bus): ~1 weekend once parts arrive.
- Step 12 (real Mac): open-ended; budget at least a weekend for edge
  cases and the Power Mac 6100 / LaserWriter compatibility fixes the
  PIC firmware acquired over its lifetime (v2.1.3 / v1.0.1).

The total is not very different from the original board-equipped estimate
because the self-loopback substitute is genuinely good enough for the
bit-level work; what we lose is the ability to *cheat* against a known
TashTalk capture in step 6, which we compensate for by exhaustive
round-trip testing through the in-firmware decoder.

A working v1 — single PHY, USB-CDC, full host-protocol compatibility,
no stretch goals — is realistic in 4–6 focused weekends.
