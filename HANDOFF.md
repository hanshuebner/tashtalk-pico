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

## Git state (push these to reach the Linux box)

- **`tashtalk-pico`**: branch **`mac-localtalk-bringup`**, HEAD **`8512ae6`**.
  This is the known-good, working-but-slow firmware. Commits:
  - `ec93a72` — RX→host forwarding on TT_DATA + **FM-0 clock-recovery decoder** (the decoder rewrite is what made real-Mac frames decode at all; do not lose it).
  - `424de4e` — continuous ring-DMA receiver + (an originator dance that was later reverted).
  - `8512ae6` — reverted the dance → **RTS-less outbound** (current HEAD).
- **`TailTalk`** (sibling repo, was at `../TailTalk`): branch
  **`localtalk-fixed-node`**, commit **`762255a`** — makes `fixed_address`
  work on LocalTalk + adds an `afp-server --node N` flag + a debug log of
  NBP lookup tuples.

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
  **The Mac doesn't require this anyway** (RTS-less works) — so unless the
  bus scope says otherwise, the dialog is not the fix. If it ever is
  needed, it must be hardware-driven (pre-armed TX DMA triggered by the RX
  task, with DE via PIO side-set) — *not* cooperative software.

## Things that WORK and must be preserved
- The **FM-0 clock-recovery decoder** (`tashtalk-proto/src/fm0.rs`).
- **RX→host forwarding** + self-TX echo suppression + SetFeatures handling
  (`host.rs`).
- **Continuous ring-DMA receiver** + inline auto-CTS (`lt_rx_task.rs`).
- **RTS-less outbound** (`transmit_on_phy` in `host.rs`) — reliable,
  just slow; the current baseline.
