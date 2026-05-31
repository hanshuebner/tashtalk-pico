# LocalTalk PHY redesign

## Problem

The current implementation layers RTS/CTS dialog TX on top of two PIO
state machines (SM0 for regular TX, SM2 for hardware-triggered dialog
data) with complex arm/disarm/claim/release cycles.  This has proven
fragile: the arm/disarm cycle corrupts SM0's output, making post-dialog
transmissions silent.  Multiple fix attempts have failed because the PIO
state machine interactions are too subtle to manage safely at runtime.

Additionally, the Mac's SCC requires a **sync pulse** before
dialog-starting frames (RTS, first control frame).  The PIC reference
firmware (`one-chip.asm:SendSyncPreamble`) generates this as 2 bit cells
of line-low followed by 4 bit cells of tristate, with exact bit-cell
timing, immediately before the SDLC preamble flags.  The current
implementation cannot produce this because the sync pulse requires
DE toggling mid-frame, which is outside the PIO program's control.

## Reference: PIC firmware architecture

The PIC firmware (`one-chip.asm`) uses a fundamentally different
architecture that we should emulate:

### TX path (bit-banged, interrupt-driven)

The PIC does **not** use a hardware HDLC transmitter.  It bit-bangs FM-0
at 230.4 kbps using cycle-counted code.  Every bit cell is exactly 35
PIC instruction cycles.  The TX routines (`SendByte`, `SendPreamble`,
`SendSyncPreamble`, `SendPostamble`) are **synchronous** — they run to
completion without yielding, and service the UART receiver inline
(`SendDoUartSvc`) during the dead cycles of each bit cell.

### RTS/CTS dialog flow

1. `PrepForNextFrame` — CSMA defer + backoff
2. `SendRts` — uses `SendSyncPreamble` (sync pulse + preamble), then
   sends `[dest, src, 0x84, crc1, crc2]` and postamble
3. `WaitForCts` — sets `CTS_PLS` flag, polls for incoming frames via
   `DealWithFrame` with a 200 µs Timer2 timeout
4. `DealWithFrame` — when it sees a CTS (type 0x85) matching the
   pending RTS source/dest, it **immediately jumps to `SendDataFrame`**
   from inside `DealWithFrame` — no return, no scheduling, no context
   switch.  This is the key to the zero-latency CTS→DATA transition.
5. `SendDataFrame` — uses `SendPreamble` (no sync pulse), then streams
   bytes from the UART receiver queue, byte-stuffing and CRC-computing
   inline.

### Key observations

- **CTS→DATA is a direct jump, not a callback.**  `DealWithFrame`
  detects CTS and jumps straight to `SendDataFrame`.  There is no
  "signal a semaphore then wait for the TX task" pattern.  The
  entire RTS→CTS→DATA sequence runs in one uninterrupted context.

- **The sync pulse is part of the frame encoding**, not a separate
  operation.  `SendSyncPreamble` drives the line low, tristates,
  then falls through to `SendPreamble` — all cycle-counted.

- **The data frame is streamed from the host UART**, not pre-encoded.
  The PIC reads bytes from the UART queue one at a time, bit-stuffs
  and CRC-computes on the fly, and sends each bit via FM-0 — all in
  the same synchronous loop.

- **RX runs in the IOC (interrupt-on-change) ISR.**  Each edge on the
  RX pin fires an interrupt.  The ISR decodes FM-0 bits, assembles
  frames, and stores the result in `LR_*` registers.  The main loop
  polls `LR_FRM` to check if a new frame arrived.

- **The transceiver is tristated between transmissions.**  After every
  TX, `SendPostamble` / `SendPoZ` drives the line low then tristates
  and switches to receive mode.  The sync pulse also tristates briefly
  after the low pulse.  Our RS-485 transceiver doesn't tristate (DE
  controls the driver), but the line levels must match.

## Proposed architecture

### Single SM for TX, pre-built bitstreams

Keep **SM0 as the only TX state machine**.  Remove SM2 entirely.
Pre-build the complete bitstream for each frame variant:

- `build_frame(payload)` — standard: 3 flags + stuffed payload + flag +
  postamble (existing)
- `build_dialog_frame(payload)` — dialog-starting: sync pulse + 3 flags
  + stuffed payload + flag + postamble

The sync pulse is encoded as FM-0 bits in the bitstream itself:
- 2 zero bits (line low for 2 bit cells — clock edge then mid-cell
  transition each)
- An abort/idle gap — but DE must drop and re-assert for the tristate
  portion.  Since the PIO can't toggle DE (it's a GPIO), the sync pulse
  should be sent as a separate mini-transmission:
  1. DE high, SM0 sends 2 zero bits + postamble, DE low (line goes low
     then tristate)
  2. Brief pause (~17 µs = 4 bit cells)
  3. DE high, SM0 sends normal preamble + frame + postamble, DE low

This two-part transmission requires SM0 enable/disable between parts,
but SM0 is always used normally (no exec'd instructions, no PINCTRL
juggling, no arm/disarm).

### CTS→DATA: pre-encode and fast-path

The PIC's "jump from DealWithFrame to SendDataFrame" pattern can be
approximated:

1. **Pre-encode the DATA frame** into a static buffer before sending
   the RTS (already done in the current implementation).
2. When `lt_rx_task` decodes the CTS, instead of triggering SM2 via
   DMA + IRQ, it **directly calls** `do_inner_tx` with the pre-encoded
   data.  This is the "inline TX from lt_rx_task" approach.

The concern is latency: the cooperative executor introduces scheduling
delay between CTS detection and DATA transmission.  However:

- With RE=GND, the decoder processes frames in near-real-time (no
  garbage period).
- `do_inner_tx` is called inline from `lt_rx_task`, not via a channel
  or signal — no scheduler hop.
- The only await points are the mutex locks (`lt_tx`, `lt_de`) and
  `Timer::after_micros(5)` for DE settle.  With no contention (the host
  task is blocked waiting for `CTS_RECEIVED`), the locks return
  immediately.

If the latency is still too high for the Mac's CTS→DATA window, we can
reduce it further:
- Use `try_lock` and skip the DE settle delay (assert DE directly via
  PAC, not through the mutex)
- Pre-lock `lt_tx` and `lt_de` before sending the RTS, holding them
  through the CTS wait, so the CTS handler has zero-latency access

### Auto-reply (ACK/CTS) path

When `lt_rx_task` detects an ENQ or RTS addressed to one of our nodes,
it should transmit the reply **inline** via SM0 — same as the current
`transmit_autoreply`.  No SM2, no PINCTRL juggling.

Auto-replies use `SendPreamble` (no sync pulse), matching the PIC
firmware.

The DE-busy check (`if DE is high, skip`) remains to prevent deadlock
when the host task is mid-TX.

### CSMA

- `defer_and_backoff` stays as-is with the 0x03 minimum mask fix.
- With RE=GND, `WIRE_LAST_ACTIVITY_US` reflects real bus state (our own
  TX + peer TX).  This is correct — the wire IS busy during our TX.
- `record_deferral` is called when we actually defer.
- Collision detection is implicit: if we send RTS and don't get CTS
  within the timeout, it's a collision (call `record_collision`).

### Wiring

- RE=GND (receiver always enabled).  With RE=DE, the decoder gets
  garbage during TX, requiring complex DMA-position-based resets.
  With RE=GND, the decoder stays synced through our own TX, echo
  suppression handles self-echo, and CSMA sees real bus state.

### What to remove

- **SM2 (LtDialogTx)** — all of `dialog_tx.rs`, `LtDialogTx` struct,
  `arm_dialog`, `disarm_dialog`, `claim_pin`, `release_pin`, DMA CH1
  usage.
- **TX_DONE_DMA_POS** — not needed with RE=GND.
- **AUTOREPLY_IFG** — not needed with RE=GND (decoder stays synced).
- **CTS_RECEIVED signal + DIALOG_ARMED/DIALOG_TX_FIRED atomics** — the
  CTS handler calls `do_inner_tx` directly instead of signaling.

### What to keep

- **FM-0 TX program on SM0** — unchanged.
- **FM-0 RX on SM1** — unchanged.
- **Continuous ring DMA on CH0** — unchanged.
- **Fm0Decoder + FrameAssembler** — unchanged.
- **Echo suppression** — unchanged, essential with RE=GND.
- **BitStream encoder** — add `build_dialog_frame` variant.
- **CSMA backoff state machine** — unchanged (with 0x03 minimum mask).
- **`auto_reply` + `compose_cts` / `compose_ack`** — unchanged.

## Risks

1. **CTS→DATA latency.**  If the inline `do_inner_tx` from `lt_rx_task`
   is too slow (>200 µs from CTS decode to first bit on wire), the Mac
   may reject the DATA.  Mitigation: pre-lock mutexes, minimize await
   points, measure with the scope.

2. **Echo suppression false matches.**  With RE=GND, every TX produces a
   self-echo.  The signature-based suppression works but relies on
   frames having unique CRC tails.  Two identical frames (e.g., repeated
   CTS to the same node) would have the same signature.  The ring-buffer
   with slot consumption handles this correctly (first echo consumes the
   slot, second is forwarded).

3. **CSMA with self-echo.**  Our own TX updates `WIRE_LAST_ACTIVITY_US`,
   adding IDG delay between consecutive outbound frames.  This is
   correct per the protocol but adds ~400-700 µs between multi-packet
   ATP responses.  The Mac's ATP layer tolerates this (its timeout is
   2+ seconds).
