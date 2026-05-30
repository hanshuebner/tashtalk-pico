# tashtalk-pico ⇄ real Mac: investigation handoff

Written 2026-05-30, mid-session, to resume the work on a Linux machine
(to scope the LocalTalk bus with the Analog Discovery 2 — the macOS
WaveForms-via-Accessibility path is a dead end; on Linux the Digilent
`dwf` SDK works directly).

This file is self-contained: a fresh Claude session on the Linux box
should be able to pick up from here.

---

## TL;DR — where we are

A Raspberry Pi Pico running this firmware, with a user-wired RS-485
transceiver, bridges a **real Macintosh IIci** (LocalTalk, printer port)
to **TailTalk** (a userspace AppleTalk stack) over USB-CDC. As of this
handoff, end-to-end **works**: the Mac discovers TailTalk's AFP server in
the Chooser, mounts the shared volume, browses it, and opens/reads files.

**The one open problem: it is SLOW.** Specifically, *outbound* directed
frames (our responses → the Mac) are frequently lost, so the Mac
retransmits each request on its ~1–2 s ATP timer. TailTalk answers every
request in **+0.000 s**; all the wall-clock time is the Mac's retransmit
wait.

**The next step is to SCOPE THE BUS** to find out *why* our responses are
lost — collision vs. clean-but-ignored vs. signal — instead of guessing.
We spent several rounds on a theory ("the Mac strictly requires RTS/CTS
for directed data") that the evidence later **disproved** (see below). Do
not resume by writing more firmware; resume by *looking at the wire*.

---

## Git state (both branches are pushed — clone on the Linux box)

```sh
git clone -b mac-localtalk-bringup git@code.netzhansa.com:hanshuebner/tashtalk-pico.git
git clone -b localtalk-fixed-node  git@github.com:hanshuebner/TailTalk.git   # your fork
```
Keep them as siblings (`tashtalk-pico/` and `TailTalk/`).

- **`tashtalk-pico`** branch **`mac-localtalk-bringup`** — working-but-slow
  firmware. The working firmware is at `8512ae6`; this `HANDOFF.md` is a doc
  commit on top. Key commits:
  - `ec93a72` — RX→host forwarding on TT_DATA + **FM-0 clock-recovery decoder** (the decoder rewrite is what made real-Mac frames decode at all; do not lose it).
  - `424de4e` — continuous ring-DMA receiver + (an originator dance later reverted).
  - `8512ae6` — reverted the dance → **RTS-less outbound** (the working baseline).
- **`TailTalk`** fork **`hanshuebner/TailTalk`** branch
  **`localtalk-fixed-node`** (commit `762255a`) — makes `fixed_address`
  work on LocalTalk + adds an `afp-server --node N` flag + an NBP-lookup
  debug log. (Upstream is `FeralFirmware/TailTalk`; this is your fork.)

The failed "originator dialog (Option A)" experiment was **never
committed** — it was reverted with `git checkout`. Don't go looking for it.

---

## What the firmware does (architecture)

Pico = a TashTalk-compatible LocalTalk modem over native USB-CDC. Two
CDC-ACM interfaces:
- **TT_DATA** (`/dev/ttyACM0` on Linux, `cu.usbmodem*1` on macOS) — the
  byte-for-byte TashTalk host protocol (`../tashtalk/documentation/protocol.md`).
  TailTalk opens this.
- **TT_DEBUG** (`/dev/ttyACM1` / `*3`) — human-readable log + a tiny shell
  (`p` = transmit a test frame, `b` = reboot to BOOTSEL, `?` = help).

Firmware tasks (Embassy, `firmware/src/`):
- `host.rs` — TT_DATA: parses host commands, forwards received LocalTalk
  frames *up* to the host (escape-encoded), and `transmit_on_phy()` puts
  host frames on the wire (currently **RTS-less** for directed data).
- `lt_rx_task.rs` — **continuous ring-DMA receiver**. One DMA channel runs
  forever into a 512-word write-address-wrapped ring; the CPU polls the
  DMA write pointer and decodes in near-real-time, so it can fire an
  autonomous **ACK/CTS inline** within LLAP's ~200 µs window. This is the
  responder (Mac→us) fast path and it works well.
- `csma.rs` — defer-and-backoff before TX; plus pending-RTS / CTS-match
  scaffolding (`set_pending_rts`, `maybe_signal_cts`, `CTS_RECEIVED`) left
  in place for a future originator dialog.
- `lt_phy.rs` — PIO TX (FM-0 on GP2, DE on GP3) and PIO RX sampler (GP4).
- `tashtalk-proto/` — host-testable `no_std` lib (CRC, codec, **fm0.rs**
  decoder, llap, bitstuff, nodemap, backoff). `cargo test` runs 48 tests.

### Pins / wiring
- GP2 = LT_DATA TX, GP3 = LT_DE (driver enable, **active high**),
  GP4 = LT_DATA RX. The user's transceiver has **RE tied to DE** → the
  receiver is **muted during our own TX** (we never hear ourselves; no
  self-echo). This matters for the bus scope: you can't see our own TX on
  the receiver path.

---

## The journey — key findings (so they aren't re-derived)

1. **RX→host forwarding was entirely missing.** The original firmware
   never forwarded received bus frames to the host; TailTalk was deaf.
   Added it. (`ec93a72`)
2. **The FM-0 decoder was free-running (8 samples/cell, locked once).**
   That only worked against the GP2→GP4 loopback (shared clock); a real
   Mac's independent clock drifted it to garbage within a byte. Rewrote it
   as an **interval/transition-timed** decoder that re-syncs on every edge.
   *This is the fix that turned garbage into 100% CRC-valid decode.*
3. **Inbound CTS latency** (Mac→us directed): originally ~1.1 ms (one big
   DMA buffer), far past LLAP's 200 µs. Fixed with the **continuous ring
   DMA** + inline auto-CTS. Mount got much faster. (`424de4e`)
4. **Outbound directed delivery** (us→Mac) is the remaining slow part —
   see next section.

### Proven hardware facts
- The Pico's TX *does* reach the Mac (node-defense test: the Mac vacated
  node 1 → 0x6c after hearing our auto-ACK).
- Decode is clean: 1300/1300 frames CRC-valid in one capture.
- The firmware is **not** double-delivering frames (checked: 0 adjacent
  identical frames; the doubled "resolve_node" in TailTalk's log is a
  TailTalk logging artifact).

---

## The open problem (what to scope)

**Symptom:** during AFP browse/file-access, the Mac retransmits each
request ~1–2 s apart (its ATP retry timer). TailTalk responds instantly,
so the response is being **lost between us and the Mac**.

**The theory we DISPROVED:** "the Mac strictly requires the originator
RTS/CTS dialog for directed data." We built that dialog (have the RX task
transmit the data inline the instant it decodes the Mac's CTS). Result:
- The dialog *engaged correctly* — we matched the Mac's CTS and the
  instrumented log showed `CTS matched data_len=38` then `dialog 38B sent`.
- But it **broke discovery** — the same NBP reply that reaches the Mac
  fine when sent **RTS-less** did *not* reach it when sent via the dialog.

Conclusion: if the Mac required the dialog, doing it correctly would have
*helped*; instead RTS-less works and the dialog fails. **So the Mac is not
strict about CTS, and the lost-response slowness is something else.**

**Leading hypotheses to test on the scope:**
- **Collisions.** The IIci is chatty (NBP name confirmations, AEP, etc.).
  Our response may overlap a Mac transmission. With RE=DE we can't even
  detect a collision in firmware.
- **CSMA timing.** `defer_and_backoff` (csma.rs) may mistime our TX
  relative to the Mac (too eager, or based on a stale `WIRE_LAST_ACTIVITY`
  since the receiver is muted during our own TX).
- **Signal integrity** on the differential pair.

### The measurement to make
Capture, on the LocalTalk **differential bus** (so you see *both* the
Mac's TX and ours on one trace) plus **GP3 (DE)** as a marker for "we are
transmitting", during file access where the Mac is retransmitting. For a
response that the Mac later retransmits its request for, answer:

- Did our response frame go out **cleanly into an idle bus**, and the Mac
  ignored it? → the loss is protocol/addressing/TailTalk, not the wire.
- Did the Mac **start transmitting during or right after** our response
  (collision / overlapping edges)? → CSMA/collision-avoidance fix needed.
- Is the differential signal **marginal/garbled**? → signal-integrity /
  transceiver fix.

Correlate the scope trace with the two software logs (both share the
host clock if captured on the same machine): TailTalk's `RUST_LOG` trace
(which response was sent, when) and the Pico's TT_DEBUG (`HOST: PIO TX`,
`LT_RX: frame …`). A timestamped TT_DEBUG capture script is in
`tools/cap_ts.py` (see below).

---

## How to set up on the Linux machine

### 1. Toolchain + firmware
```sh
# rustup with the RP2040 target
rustup target add thumbv6m-none-eabi
cd tashtalk-pico/firmware
cargo build --release           # → ../target/thumbv6m-none-eabi/release/tashtalk-pico
# host-side lib tests (optional):  (cd .. && cargo test -p tashtalk-proto)
```
Flash via picotool (hold BOOTSEL or press `b` on TT_DEBUG first):
```sh
picotool load -x ../target/thumbv6m-none-eabi/release/tashtalk-pico -t elf
```
(There's no `elf2uf2-rs` needed; picotool loads the ELF directly.)

### 2. Serial port access
The Pico enumerates as **native USB-CDC** (VID `16c0`/PID `27dd`, *not*
the CP210x the TashTalk README mentions — that udev rule does not apply).
On Linux it appears as `/dev/ttyACM0` (TT_DATA) and `/dev/ttyACM1`
(TT_DEBUG). Add yourself to `dialout` or add a udev rule for
`idVendor==16c0, idProduct==27dd` with `TAG+="uaccess"`.

### 3. TailTalk AFP server
```sh
cd TailTalk
cargo build --release -p afp-server-example
mkdir -p /tmp/pico-share && echo hello > /tmp/pico-share/README.txt
# pin a node for repeatable scoping with --node:
RUST_LOG=info,tailtalk::afp=debug,tailtalk::asp=debug,tailtalk::atp=debug \
  ./target/release/afp-server --tashtalk /dev/ttyACM0 --path /tmp/pico-share --node 200
```
Mac side: AppleTalk Active on the **printer port**; Chooser → AppleShare →
**TailTalk** → Connect (Guest) → mount → browse → open README.txt.

### 4. Diagnostics that already exist (in `tashtalk-pico/tools/`)
- `cap_ts.py <port> <secs>` — prints each TT_DEBUG line with a host
  wall-clock timestamp (UTC, ms) so it lines up with TailTalk's `RUST_LOG`
  timestamps. Works on Linux with `pip install pyserial`.
- `cap_debug.py` — simpler fixed-window TT_DEBUG capture.
- The `lt_crc` (X.25: init 0xFFFF, poly 0x8408 reflected, xorout 0xFFFF)
  and the "send a raw TashTalk TransmitFrame" pattern are in this
  conversation's history if you need to inject test frames.

### 5. Bus scope (Analog Discovery 2, on Linux)
On Linux the `dwf` SDK enumerates the AD2 from any process (the macOS
`AppleUSBFTDI` problem documented in `README.md`/`STATUS.md` does **not**
apply). Use the Python `dwf`/`ctypes` bindings directly — do **not** use
`tools/wire_capture.py` (it drives WaveForms.app via macOS Accessibility
APIs and is macOS-only). Probe: the LocalTalk differential pair (both
directions) + GP3 (DE) as the "we're transmitting" marker. Trigger on DE
rising, capture a few hundred µs around it, and look for overlapping Mac
activity (collision) per the questions above.

---

## Bus scope findings (2026-05-30, AD2 on Linux)

Captured with `tools/bus_scope.py` (new, dwf SDK direct) on all six
RS-485 transceiver signals (A, B, DI, DE, RE, RO).

**Original conclusion (WRONG): the Mac ignores RTS-less directed data.**
This was disproved by the `transmit_frame` timing fix — see below.

- Our TX frames appeared clean on the scope but were actually **truncated**:
  `transmit_frame` disabled SM0 ~20 µs after FIFO drain, but the last
  word takes up to 139 µs to shift out of the OSR. The closing SDLC flag
  was never fully transmitted. The Mac's SCC saw an abort, not a frame.
- Signal integrity was fine: DI→B transitions 1:1, simultaneous (≤100 ns).
- The original TashTalk PIC firmware (`one-chip.asm:593`) **always sends
  RTS before directed data** and waits for CTS.

**LLAP data frame ACKs are NOT a thing** — the ACK (0x82) control frame is
only for ENQ during address acquisition. The directed dialog is exactly
RTS → CTS → DATA, three frames, no acknowledgment (Inside AppleTalk ch.1).

## Dead ends — do NOT retry these

- **Shrinking the RX ping-pong DMA buffer** (8/4 words): overran the PIO
  FIFO during our own ~300 µs reply TX and corrupted the peer's next data
  frame → broke sessions. (Why we went to the continuous ring DMA.)
- **Decoupling the auto-CTS into a separate task**: added ~1–4 ms of
  cooperative-scheduler wake latency (measured) — worse than inline.
- **Originator RTS/CTS dialog (data-after-CTS in software)**, both the
  host-task and the inline-in-RX-task (Option A) variants: even with
  busy-polling and nothing between CTS-detection and TX, the data landed
  too late for the Mac's CTS→data window and broke delivery (discovery
  stopped working). Also the busy-poll could spin if a dialog stalls.
  The fix must be hardware-driven (pre-armed TX DMA triggered by CTS
  detection, with DE via PIO side-set) — *not* cooperative software.
- **Auto-ACK for data frames**: LLAP does not ACK data frames (only ENQ).
  Adding auto-ACK broke session establishment. Reverted.

## Things that WORK and must be preserved
- The **FM-0 clock-recovery decoder** (`tashtalk-proto/src/fm0.rs`).
- **RX→host forwarding** + self-TX echo suppression + SetFeatures handling
  (`host.rs`).
- **Continuous ring-DMA receiver** + inline auto-CTS (`lt_rx_task.rs`).

## Hardware-driven originator dialog: implementation status

Code is in place (compiled, tested, disabled pending debug):

- **`dialog_tx.rs`** (new): static DMA buffer, arm/abort DMA CH1 → PIO0
  TXF0, inline `trigger()` that asserts DE + fires PIO IRQ 4.
- **`lt_phy.rs`**: `arm_dialog()` uses `exec_instr(wait 1 irq 4)` to gate
  SM0 until trigger fires; `disarm_dialog()` cleans up on timeout.
- **`host.rs`**: `transmit_directed_dialog()` orchestrates the full
  sequence (pre-encode → CSMA → RTS → arm → wait CTS → drain → DE off).
  Disabled with `let is_directed_data = false;` — flip to enable.
- **`csma.rs`**: `maybe_signal_cts()` calls `dialog_tx::trigger()` inline.

### Bugs fixed (2026-05-30, second session)

1. **`transmit_frame` truncated the closing SDLC flag.** After the TX
   FIFO drained, the code waited only `max(4*4, 20) = 20 µs` before
   disabling SM0. But the last FIFO word was still being shifted out of
   the OSR (up to 32 bits × 4.34 µs = 139 µs). The closing 0x7E flag
   and often the CRC bytes were never transmitted — the Mac's SCC saw an
   SDLC abort instead of a clean frame end. **Fixed**: wait
   `(last_word_valid_bits + 2) * bit_us` so only the frame's real bits
   are waited for, not the full padding word.

2. **DMA armed before RTS — FIFO conflict.** The old dialog code armed
   DMA (filling SM0's TX FIFO with pre-encoded data) *before* calling
   `do_inner_tx` for the RTS. When `transmit_frame` enabled SM0 to send
   the RTS, SM0 pulled the DMA data instead of the RTS bytes. **Fixed**:
   send RTS first via `do_inner_tx`, then arm dialog, then arm DMA.

3. **`arm_dialog()` called `sm.restart()`** which cleared the TX FIFO
   (losing DMA data) and set OSR to all-ones (injecting 32 garbage bits).
   Note: contrary to earlier analysis, `restart()` does NOT reset Y/X
   (RP2040 datasheet §3.5.6.1). **Fixed**: replaced with `exec out null,
   32` (drain residual OSR) + `exec wait 1 irq 4` (gate on CTS trigger),
   preserving FIFO contents.

4. **`disarm_dialog()` didn't clear the FIFO** on CTS timeout, leaving
   stale DMA data that would corrupt the next `transmit_frame`. **Fixed**:
   added `sm.restart()` in `disarm_dialog()` (safe here since we WANT to
   flush the FIFO).

5. **`trigger()` was void** — `maybe_signal_cts` always signaled
   `CTS_RECEIVED` even if the dialog wasn't armed. **Fixed**: `trigger()`
   returns `bool`; signal only fires on success.

### Current state (end of 2026-05-30 second session)

6. **Postamble added** to `BitStream::build_frame`: 13 raw `1` bits
   (SDLC abort/idle marker) + 1 `0` bit (SendPoZ — drive low before
   tristate). Matches the PIC reference firmware's `SendPostamble`.
   Without it, padding zeros after the closing flag look like a new
   frame start to the SCC, potentially causing spurious framing errors.

**What works now:**
- `transmit_frame` timing fix: waits for the valid bits of the last
  word to shift out, not just 20 µs. All frames now have a complete
  closing flag + postamble.
- Direct TX (RTS-less) delivers frames to the Mac — AFP login, volume
  mount, directory listing, and file open all work.
- Auto-CTS/ACK responses work reliably.

**What does NOT work: the Mac does not respond to our RTS with CTS.**
Tested exhaustively: bare RTS + 10 ms listen, with full diagnostic
logging for any CTS (type 0x85) frame — the Mac never sends one.
Our RTS is correctly formed ([dest, src, 0x84, crc0, crc1], CRC
verified, properly terminated with postamble). The Mac does receive
our other frames (proven by working AFP sessions). Unknown why it
rejects our RTS specifically.

**Why RTS/CTS is required (not optional):**
Trace analysis of a 145-second session (2234 bus events) shows an
**82% retransmit rate** without RTS/CTS. The Mac retransmits each
request 4-5 times at 1-2 s intervals. We respond to every request,
but our responses collide with the Mac's next transmission (RE is
tied to DE, so we can't detect collisions). RTS/CTS reserves the bus
and prevents this. Direct TX works intermittently by luck — sometimes
our response lands in a gap, sometimes it collides.

**Next step: scope our RTS on the wire with the AD2** to see what the
Mac actually receives. The AD2 is connected (DIO 0-5 = A, B, DI, DE,
RE, RO). Compare our RTS waveform with the Mac's RTS waveform to find
any encoding or timing difference that could cause the Mac to reject it.
