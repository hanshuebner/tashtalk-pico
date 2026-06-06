# Design Note: LToUDP bridge on Pico W

Status: **proposal / not yet implemented.** Targets a future migration from
the plain Pico (RP2040) to the **Pico W** (RP2040 + CYW43439 WiFi).

Goal: bridge the physical LocalTalk bus to a virtual LocalTalk network over
IP using [LToUDP](./ltoudp-spec.md), running *alongside* the existing
USB-CDC host interface — not replacing it.

## 1. Feasibility summary

| Concern | Verdict |
|---------|---------|
| Rust TCP/UDP stack for Pico W | `embassy-net` (smoltcp) + `cyw43` / `cyw43-pio`. First-party Embassy crates; we already build on Embassy. |
| Flash | Current firmware `text` ≈ 53 KB of 2 MB. CYW43 firmware blob ≈ 224 KB + CLM ≈ 1 KB, plus ~80–120 KB of stack/driver code. Total well under 400 KB. **Ample.** |
| RAM | Current `bss` ≈ **187 KB of 256 KB** — only ~64 KB free. (Dominated by the 64 KB embassy task arena, the 24 KB static TX encoding buffers, and the 16 KB RX ring.) cyw43 buffers + embassy-net + smoltcp socket buffers ≈ 40–60 KB even when trimmed (UDP-only, minimal socket set). **Tight — fits only with care.** |
| PIO | RP2040 has 8 SMs; we use PIO0 SM0 (TX) + PIO1 SM0 (RX FM-0). `cyw43-pio` needs 1 SM + a small program. **Free.** |
| DMA | 12 channels; we use CH0 (RX ring) + CH1 (TX). cyw43-pio uses ~1. **Free.** |
| Pins | Pico W reserves GP23/24/25/29 for the CYW43 (internal SPI + WL_ON). Our bus uses GP2–GP7. **No conflict.** |

Numbers re-measure with `arm-none-eabi-size target/thumbv6m-none-eabi/release/tashtalk-pico` after any build.

**RAM is the binding constraint, not flash.** With ~64 KB free, a minimal
UDP-only LToUDP build is plausible but leaves little margin. Levers, in order
of preference:

1. Shrink the embassy task arena (`task-arena-size-65536` → smaller) once the
   actual task set is known — likely the single biggest win.
2. Minimal smoltcp config: one UDP socket, small RX/TX socket buffers, no TCP,
   no DHCP server, IPv4 only.
3. Trim or share the 24 KB static TX buffers if the bridge can reuse them.
4. **Move to the Pico 2 W (RP2350): 520 KB SRAM** makes the whole question
   moot and is the recommended target if the RAM budget proves too tight.

## 2. Wire format recap

See [ltoudp-spec.md](./ltoudp-spec.md) for the authoritative spec. The parts
that drive this design:

- UDP to `239.192.76.84:1954` (multicast preferred; broadcast is the
  alternative — all speakers on one segment must agree).
- Payload = `[ sender_id : u32 ][ LLAP packet ]`.
- The **LLAP packet**, *not* the frame: `dest, src, type, payload…` with
  **no SDLC flags, no bit-stuffing, and NO 2-byte FCS/CRC**.
- RTS/CTS must never cross the bridge. RTS → synthesise a CTS back onto the
  local wire; CTS → drop.

This is *less* work than the USB path: no CRC is carried over UDP, and our
core-1 engine already auto-replies CTS on the wire, which is exactly the
"bridging code generates CTS" responsibility the spec describes.

## 3. Architecture fit

The existing dual-core split is preserved:

- **Core 1** (bare-metal): unchanged. PIO/DMA wire engine, auto-reply,
  CSMA/CA TX. Owns `SharedState`.
- **Core 0** (Embassy async): already runs USB, host protocol, and the debug
  shell. Add three tasks:
  1. `cyw43` runner (drives the WiFi chip).
  2. `embassy-net` stack runner.
  3. **LToUDP bridge task** (new) — the subject of this note.

```
        physical LocalTalk bus
                 │
        ┌────────┴─────────┐   core 1 (unchanged)
        │  PIO + DMA + AR  │
        └────────┬─────────┘
        SharedState RX / TX  (the same queues USB uses today)
                 │
   ┌─────────────┼──────────────┐   core 0 (Embassy)
   │             │              │
┌──┴───┐   ┌─────┴─────┐   ┌────┴─────┐
│ USB  │   │  LToUDP   │   │  debug   │
│ host │   │  bridge   │   │  shell   │
└──────┘   └─────┬─────┘   └──────────┘
                 │
           embassy-net (UDP socket, multicast)
                 │
              cyw43 → WiFi → IP LAN
```

## 4. The bridge task

Two directions, both operating on the **packet** (no FCS):

### Wire → UDP (egress)

1. Receive a decoded, good-CRC RX frame via the existing RX path (the same
   hook `log_rx_frame` / `rx_bridge` uses).
2. Filter:
   - Drop control frames that must not leave the wire: type `0x84` (RTS) and
     `0x85` (CTS). (Type `>= 0x80` is control; see CLAUDE.md "LLAP Frame
     Types".)
   - Optionally drop our own auto-reply ACK/echoes per local policy.
3. Strip the trailing 2-byte FCS (the wire frame carries it; the packet must
   not).
4. Prepend the 4-byte sender ID.
5. `socket.send_to(payload, (GROUP, 1954))`.

### UDP → Wire (ingress)

1. `socket.recv_from()` on `0.0.0.0:1954`, joined to `239.192.76.84`.
2. Echo check: drop if `sender_id == OURS` **and** source IP is ours.
   With a single Pico and one IP, "drop if `sender_id == OURS`" suffices.
3. Strip the 4-byte sender ID → bare LLAP packet.
4. Submit to the wire via the existing TX path. **Core 1 computes and appends
   the FCS** as part of HDLC framing (`build_tx_buffer_into`), so the bridge
   never touches CRC. RTS/CTS handling for directed frames is already done by
   the CSMA/CA engine.

### Sender ID

The spec permits a fixed value for a single speaker. Use a constant derived
from the USB serial (or simply `0`). No per-process PID concept on bare
metal. Keep it in one place: `const LTOUDP_SENDER_ID: u32 = …;`

## 5. TX arbitration (the one real new concern)

Today the wire TX path has a single logical writer at a time
(`TX_SUBMIT`/`TX_DONE`, see `host::submit_tx_from_debug` and the
single-writer protocol in `core1.rs`). Adding the bridge makes **three**
producers: USB host, debug shell, and the LToUDP bridge.

Plan: introduce a small **TX mux** in front of `SharedState` TX submission.

- A bounded `embassy_sync::channel::Channel<TxRequest, N>` on core 0; all
  three producers `send` into it.
- A single consumer task owns the actual `submit_tx` handshake with core 1,
  serialising access and observing `TX_RESULT_FAIL` per request.
- `TxRequest` carries the packet bytes (heapless, ≤ 605) + a oneshot/signal
  for the result so the caller can report OK/timeout (the debug `d` command
  and host protocol both already want this).

This keeps core 1's single-writer invariant intact while letting any number
of core-0 producers queue work. It also cleanly subsumes the current ad-hoc
`submit_tx_from_debug`.

## 6. Out of scope / open questions

- **Loop prevention**: a frame we inject from UDP onto the wire will be
  echoed back by our own receiver (we always see the bus). Existing
  source-byte echo suppression (`nodes.contains(frame[1])`) plus the UDP
  sender-ID check should prevent a UDP→wire→UDP loop, but verify with two
  speakers.
- **AARP / address mapping**: pure LToUDP just tunnels LLAP; it does not do
  AppleTalk routing. Full router behaviour (à la tashrouter) is a later,
  larger effort.
- **WiFi configuration / provisioning**: SSID + PSK + static/DHCP IP. This is
  why the debug CLI needs an `ip` / `wifi` config surface — see
  [debug-cli-plan.md](./debug-cli-plan.md).
- **Multicast vs broadcast**: default to multicast (`239.192.76.84`); make it
  configurable.
- **Pico 2 W** (RP2350) is an alternative target with more RAM/flash; the
  same `cyw43` stack applies.

## 7. Suggested crate additions (Cargo.toml)

```toml
embassy-net      = { version = "...", features = ["udp", "proto-ipv4", "multicast", "dhcpv4", "medium-ethernet"] }
cyw43            = { version = "...", features = ["defmt"] }
cyw43-pio        = { version = "...", features = ["defmt"] }
embassy-net-driver = "..."   # if not pulled transitively
static_cell      = "2"       # already present
```

Pin to the same source (crates.io vs embassy git) as the existing embassy
crates to avoid version skew — see the note at the top of `firmware/Cargo.toml`.
Flash the CYW43 fw + CLM blobs to fixed flash addresses during development so
they don't bloat every app build (the cyw43 docs describe both approaches).
