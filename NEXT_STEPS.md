# Next steps without RS-422 transceiver or Mac

Captured 2026-05-21. Picks up from `STATUS.md` after step 9. Steps 10–12 in
the bring-up scorecard are labelled hardware-gated, but most of that
work — and a non-trivial chunk of the limits called out in
`STATUS.md` — can proceed with only the existing Pico + Analog Discovery 2.

This file lists those work items and goes into detail on the largest
architectural question still open: the RTS/CTS latency problem.

## Work items, ordered by leverage

### Tier 1 — nothing new (or one second Pico)

1. **Two-Pico back-to-back without RS-422.** Step 11 is described as
   transceiver-gated, but two Picos can talk at 3.3 V CMOS directly:
   cross GP2 ↔ GP4 between the two boards and share a ground. The
   transceiver only matters once you need a differential pair to reach
   a Mac. This validates the whole framing / CSMA / autoresponder loop
   against a real peer instead of self-loopback. Easily the single
   biggest de-risking step before hardware arrives.

2. **Re-enable and harden the disabled paths.** `host.rs:127` has the
   synthetic ticker behind `if false`. Gate it on a TT_DEBUG command
   (`s`/`q` toggle) instead, so it stays available for stress runs but
   doesn't drown the log by default.

3. **Fix the wire-capture window.** `STATUS.md` flags `la.dwf3work` as
   ~655 µs of capture — too short for ENQ + ACK in one trace. Bump to
   8192 samples or drop to 3.125 MHz, re-save the workspace, re-capture
   the full dialog. Pure workspace edit.

4. **`SendPoZ` tristate audit on the wire.** Plan §2.9 calls the
   `1 → drive 0 → Z` postamble a real compatibility requirement
   (LaserWriter / IIgs / PowerMac 6100). `bitstream.rs:17` says it is
   modeled in code — confirm it on the AD2 with an extended capture
   window, since this can only be re-verified pre-Mac.

### Tier 2 — known limits worth attacking

5. **RTS/CTS latency** — see the detailed analysis below. Worth at
   least prototyping a smaller-buffer or PIO-IRQ-on-flag path to see if
   the 200 µs response budget is reachable. Two-Pico back-to-back is
   the cleanest place to validate.

6. **USB suspend policy.** Plan §2.9 lists this as a watch-out; no code
   handles it today. Decide drop-oldest vs. pause-receiver, implement,
   test by suspending the host.

### Tier 3 — stretch goals already scoped in the plan

7. **Onboard pcap ring**, exposed as a third CDC interface. Turns the
   Pico into its own logic analyzer for later Mac debugging.

8. **defmt over TT_DEBUG via `defmt-bbq`.** Removes the SWD probe from
   the future debug story.

Recommendation: start with #1 (second Pico). Cheapest, highest-information
move, and a working two-Pico setup is the natural test rig for #5.

---

## The RTS/CTS architecture problem

The short version: the receiver architecture's minimum wire-to-decoded-event
latency is structurally larger than LLAP's CTS-response window, and the
PIC reference firmware only met that window by doing the response inside
an interrupt.

### What LLAP demands

For unicast data frames, the originator sends RTS → waits for the
addressed node's CTS → sends Data. The standard inter-dialog gap is
~200 µs, so the responder has on the order of 200 µs from "last bit of
RTS hits the bus" to "first bit of CTS on the bus."

The PIC reference (`one-chip.asm`) hits this because the IOC interrupt
vector *is* the receiver: the moment the closing flag is decoded, the
ISR composes the CTS and starts transmitting, in microseconds. There is
no buffer between "frame done" and "reply out."

### What this port does on RX

PIO0 SM1 samples GP4 at 1.8432 MHz, pushing 32-bit sample words into a
FIFO that a ping-pong DMA drains into 64-word RAM buffers
(`lt_rx_task.rs:33-37`, `BUF_WORDS = 64`). The CPU only sees a frame
after the DMA buffer it lived in has filled. That buffer is 2048 samples
≈ **1.1 ms of wire**. So:

- Best case (closing flag near the end of a buffer): a few hundred µs
  to autoresponder.
- Worst case (closing flag near the start of a buffer): ~1.1 ms to
  autoresponder.

Either way, past 200 µs.

### Which directions this breaks

1. **We are the addressee.** A peer sends us RTS and owes us a CTS.
   `auto_reply()` already composes the CTS and `lt_rx_task.rs:181-197`
   transmits it — but only after the enclosing DMA buffer completes. A
   strict originator times out and treats us as deferred/absent before
   our CTS appears on the wire.

2. **We are the originator.** The host driver wants to send a unicast
   Data frame, so firmware should fire RTS, wait for CTS within
   ~200 µs, then fire Data. The scaffolding exists
   (`PENDING_RTS_OUR_SRC`, `cts_matches_pending`, `CTS_RECEIVED` in
   `csma.rs:41-83`) but is not wired into the TX path. Wiring it up
   would not save it: the CTS we're waiting for sits in a DMA buffer
   for up to 1.1 ms before `CTS_RECEIVED.signal()` ever fires.

### Why the port punts to the host driver instead

Today the firmware transmits whatever the host hands it (after CSMA
defer + backoff) and trusts TailTalk's `tashtalk` crate to assemble
RTS/Data pairs if it needs LLAP-strict timing. But host-driven RTS/CTS
is fundamentally hopeless on USB-CDC: each USB scheduling slot is
~1 ms in each direction, so RTS → CTS → Data through the host is
multiple milliseconds. The current scheme works only as long as peers
tolerate a "datagram-style" device that ignores the dance.

### Why the DMA architecture was chosen anyway

Big DMA buffers amortize per-buffer setup, hide CPU jitter from
USB/Embassy, and eliminate the silent FIFO-overflow risk that polling
had. It's a robustness/throughput win; the cost is RTS-response
latency.

### Ways out, none deployed

- **Shrink the DMA buffer.** 8-word buffers ≈ 70 µs of wire, which
  clears 200 µs end-to-end with margin. The frame assembler is byte-
  streaming so buffer boundaries inside a byte are fine. Cost: ~16×
  more DMA completions per second; more sensitive to Embassy
  scheduling jitter. Easy to try.

- **PIO IRQ on closing flag.** Have the RX SM raise IRQ0 the instant
  it recognizes `0x7E` after data; the CPU drains the partial buffer
  immediately rather than waiting for DMA completion. Keeps big
  buffers for steady-state throughput while collapsing the end-of-
  frame latency. Probably the right answer.

- **CTS entirely in PIO.** The CTS payload is purely a function of the
  RTS's dest/src bytes plus a node-bitmap lookup. A second PIO SM with
  a 32-byte bitmap in scratch could recognize "RTS to one of our
  nodes" and emit the CTS with no CPU involvement at all — what the
  PIC ISR does, relocated to PIO. Most authentic to the PIC reference,
  also the most ambitious to write.

The first two are cheap experiments. Two Picos back-to-back is the
cleanest place to validate, because you can instrument both sides and
inject controlled RTS/CTS patterns without a Mac in the loop.
