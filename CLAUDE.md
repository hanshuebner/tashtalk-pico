# CLAUDE.md — Project context for Claude Code

## Build & Flash

```bash
make all          # build firmware + generate uf2
make flash        # build + flash via picotool (single Pico in BOOTSEL)
make test         # run tashtalk-proto tests (host target)
```

To flash a specific Pico when multiple are connected:
```bash
printf "b\r" > /dev/tashtalk-debug-<SERIAL>   # reboot to BOOTSEL
sleep 4
picotool load -u -v -x -t uf2 tashtalk-pico.uf2
```

## Hardware Setup

Two Picos may be connected simultaneously:
- **Active Pico** (connected to RS-485 transceiver + Mac): identified by USB serial
- **Analyzer Pico** (GP4 wired to transceiver RO): passive listener

Udev symlinks: `/dev/tashtalk-data-<SERIAL>`, `/dev/tashtalk-debug-<SERIAL>`

### Pin Assignments
- GP2: TX data (PIO0 SM0 OUT → RS-485 DI)
- GP3: Driver enable (software GPIO → RS-485 DE)
- GP4: RX data (PIO1 SM0 input ← RS-485 RO)
- GP5: Unused (was carrier sense, now software-based)
- GP6: Debug output 0 (DBG0, used for logic analyzer triggering)
- GP7: Debug output 1 (DBG1, available for instrumentation)

### RS-485 Transceiver
- RE tied to GND (receiver always enabled)
- We always see the bus, including our own transmissions (echo)
- Echo suppression: source-byte filtering (`nodes.contains(frame[1])`)

## Debug Tools

### Debug Shell (TT_DEBUG CDC port)
Connect with: `kermit -l /dev/tashtalk-debug-<SERIAL> -c`
Or: `socat - /dev/tashtalk-debug-<SERIAL>,raw,echo=0,cr,crnl`

Line-oriented command interpreter (`debug.rs`). Type a command and press
Enter. `help` lists everything; full names, unique prefixes, and the legacy
single-letter aliases all work (e.g. `status`, `stat`, or `s`).

Commands: `help` `status` `clear` `monitor [on|off]` `analyzer [on|off]`
`frames` `id` `tx <hex>` `directed <dest> <hex>` `node <set|clr> <id>`
`testframe` `bootsel`.

Every reply line is tagged for machine parsing: `OK`/`OK k=v…` and
`ERR <code> <msg>` are terminators; `STAT …` carry status fields; `RX:`/`RX!:`
carry frames; `#` is a banner/help comment. A host writes a line and reads
until `OK`/`ERR`. See `docs/debug-cli-plan.md` for the format spec and the
planned telnet front-end. To add a command: one `COMMANDS` row + one
`dispatch` arm; `help` is generated from the table.

### Analyzer Mode
The `analyzer` command (alias `a`) enables protocol analyzer mode. All decoded
frames (including bad CRC) are printed as tagged hex lines:
- `RX:` prefix = good CRC
- `RX!:` prefix = bad CRC

(Each line also carries a `t=<ms>` timestamp field before the hex.)

For a passive bus sniffer, flash the same firmware to a second Pico,
wire its GP4 to the transceiver's RO output, and enable analyzer mode.
The analyzer Pico has no nodes registered, so it doesn't auto-reply
or interfere with bus traffic.

### PicoProbe (SWD Debug)
A PicoProbe (or Raspberry Pi Debug Probe) can inspect the running
system via SWD without any firmware instrumentation.

**Wiring**: Probe GND→Target GND, Probe SWCLK→Target SWCLK pad, Probe SWDIO→Target SWDIO pad

**Software**: `probe-rs` (install with `cargo install probe-rs-tools`)

**Reading memory from a hung Pico**:
```bash
# Check if core 0 is in HardFault (exception 3 in bottom 9 bits)
probe-rs read --chip RP2040 --protocol swd b32 0xE000ED04 1

# Read PIO1 SM0 program counter
probe-rs read --chip RP2040 --protocol swd b32 0x503000D4 1

# Read a symbol's value (find address with nm first)
nm target/thumbv6m-none-eabi/release/tashtalk-pico | grep SYMBOL_NAME
probe-rs read --chip RP2040 --protocol swd b32 <ADDRESS> 1
```

Note: symbol addresses change between builds. Always re-run `nm` after rebuilding.

### AD2 (Analog Discovery 2)
Bus scope tool in `tools/bus_scope.py`. Channel map in `tools/ad2.py`:
DIO0=A, DIO1=B, DIO2=DI, DIO3=DE, DIO4=RE, DIO5=RO, DIO6=DBG0.
AD2 buffer is only 4096 samples; use 2 MHz for ~2ms capture window.
VIO must be 3.3V (set via both `FDwfAnalogIOChannelNodeSet` and
`FDwfDeviceParamSet(DwfParamDigitalVoltage, 3300)`).

## Architecture

### Dual-Core Design
- **Core 0** (Embassy async): USB, host protocol, frame encoding, debug shell
- **Core 1** (bare-metal loop): PIO, DMA, all wire interaction, auto-reply

### Core 1 Wire Engine
- PIO0 SM0: TX data shift-out (`out pins, 1` at 460.8 kHz)
- PIO1 SM0: RX FM-0 decoder (hardware, 16× oversampling)
- DMA CH0: RX ring buffer (PIO1 RXFIFO → 4096-word ring in RAM)
- DMA CH1: TX (tx_slot DMA buf → PIO0 TXFIFO)
- Auto-reply: ENQ→ACK and RTS→CTS (immediate, no CSMA)
- CSMA/CA TX with RTS/CTS/DATA dialog for directed frames

### Inter-Core Communication
`SharedState` with `UnsafeCell` fields + `AtomicU32` flags.
Single-writer protocol: TX_SUBMIT/TX_DONE, RX_READY, NODES_UPDATED.

### PIO FM-0 Decoder (PIO1 SM0)
32-instruction hardware FM-0 decoder. After each boundary transition,
delays to mid-cell via `nop [7]` and samples with `jmp pin`. Decoded
bits autopush every 8 into the RX FIFO. Frame-end marker 0xFFFFFFFF
pushed on idle timeout. The `wait` instructions are used ONLY in idle
state; mid-frame uses timeouts (`set x, 7` + `jmp x--`) on all paths
to prevent permanent stalls on noisy/glitchy signals.

### CSMA/CA
Carrier sense is software-based: `LAST_ACTIVITY_US` timestamp updated
when decoded bits arrive from PIO. Bus considered idle after 50µs of
no activity. During idle, PIO blocks in `wait` and pushes nothing.

### DMA Ring Buffer
4096 entries (16KB), aligned to 16384 bytes. DMA ring_sel=true,
ring_size=14. DREQ=12 (PIO1_RX0). Overrun detection: distance > half
ring → snap read pointer to write pointer.

### Critical Constraints
- **Embassy PIO Handle Drop**: All PIO handles leaked via `mem::forget()`
  after init. Without this, Drop disables state machines.
- **Stack size**: TX encoding buffers (24KB total) MUST be static, not
  stack-allocated. Use `build_tx_buffer_into()`. Stack overflow on core 0
  causes HardFault that kills USB.
- **TashTalk Protocol Framing**: CommandParser uses DDP length field to
  determine frame length. Multi-packet frames (>64 bytes USB) require
  `PENDING_FRAME_READY` flag to prevent premature submission.

### LLAP Frame Types
- 0x81 = ENQ, 0x82 = ACK, 0x84 = RTS, 0x85 = CTS
- Type < 0x80 = data frame, type >= 0x80 = control frame
- Directed data frames (dest != 0xFF, type < 0x80) use RTS/CTS/DATA dialog

### RTS/CTS Timing
- RTS→CTS window: peer must respond within 200µs
- CTS→DATA: originator transmits immediately
- IFG: ~200µs minimum, IDG: 400µs minimum

## Current Status

### Working
- PIO hardware FM-0 decode (zero bad CRC on real Mac traffic)
- Auto-reply: ENQ→ACK and RTS→CTS (sub-microsecond response)
- CSMA/CA TX: broadcast and directed, 100% CTS success rate
- TailTalk visible in Mac Chooser
- AFP login succeeds (GetStatus + OpenSession)
- AFP share browsing works (directory listing, file access)

### Known Issues
- Occasional session hangs during sustained AFP traffic (Mac shows
  watch cursor, no bus traffic). May be TailTalk application-level
  issue rather than firmware. Pico remains alive (both cores running).

## Code Structure

### tashtalk-proto (no_std library)
- `fm0.rs`: FM-0 encoder/decoder
- `bitstuff.rs`: SDLC bit-stuffing/unstuffing
- `llap.rs`: Frame assembly, auto-reply logic, CRC validation
- `hdlc.rs`: HDLC frame builder (`build_tx_buffer_into` for static bufs)
- `backoff.rs`: CSMA/CA state machine
- `codec.rs`: TashTalk host byte protocol (CommandParser)
- `crc.rs`: CRC-16 CCITT
- `nodemap.rs`: 256-bit node bitmap

### firmware/src/
- `main.rs`: Entry, core 1 spawn, Embassy executor, rx_bridge
- `core1.rs`: Core 1 bare-metal loop, SharedState, DMA, TX/RX, auto-reply
- `host.rs`: Host protocol, frame encoding, TX submission
- `localtalk/phy.rs`: PIO init (TX + RX FM-0 decoder)
- `debug.rs`: Debug shell, analyzer mode, status reporting
- `tt_log.rs`: Log channel macros
- `usb_dev.rs`: USB descriptor setup

## Future Work (design notes)

Proposals, not yet implemented. See `docs/`:

- `docs/ltoudp-spec.md`: LToUDP protocol spec (local transcription of the
  now-login-gated windswept.home.blog post). Payload = `[sender_id:u32][LLAP
  packet]`; packet carries NO SDLC framing and NO FCS/CRC. Multicast
  `239.192.76.84:1954`. RTS/CTS never cross the bridge.
- `docs/ltoudp-design.md`: Pico W LToUDP bridge plan. embassy-net + cyw43 on
  core 0 alongside USB; flash/PIO/DMA/pins fit easily, but RAM is tight
  (~64 KB free of 256 KB) — UDP-only trim or a Pico 2 W. Key new concern is a
  TX mux arbitrating USB + debug + bridge into core 1's single-writer TX.
- `docs/debug-cli-plan.md`: debug.rs refactor — unified tagged output format
  (`OK`/`ERR`/`STAT`/`RX:`), a `COMMANDS` table replacing the single-letter
  match, a `Line` trait for CDC+telnet reuse, password-protected telnet on
  Pico W. Steps 1–4 land without networking.
