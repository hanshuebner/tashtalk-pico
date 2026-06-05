# tashtalk-pico

A Raspberry Pi Pico (RP2040) reimplementation of the
[TashTalk](https://github.com/lampmerchant/tashtalk) LocalTalk interface, using
native USB-CDC for the host connection (no CP210x bridge). The host-facing byte
protocol is preserved bit-for-bit so any existing TashTalk host driver — in
particular [TailTalk](https://github.com/feralfirmware/tailtalk)'s `tashtalk`
crate — works unchanged.

## Current Status

Working end-to-end with a real Macintosh over LocalTalk:

- **AFP login and file sharing** via TailTalk's afp-server
- **Hardware FM-0 decoding** in PIO (zero bad CRC on Mac traffic)
- **RTS/CTS/DATA dialog** with sub-microsecond auto-reply
- **Dual-core architecture**: Embassy async on core 0 (USB, host protocol),
  bare-metal polling loop on core 1 (all wire interaction)

## Hardware

### Pico Pin Assignments

| Pin | Function | Direction | Notes |
|-----|----------|-----------|-------|
| GP2 | LT_DATA TX | OUT | PIO0 SM0 → RS-485 DI |
| GP3 | LT_DE | OUT | Software GPIO → RS-485 DE |
| GP4 | LT_DATA RX | IN | RS-485 RO → PIO1 SM0 |

### RS-485 Transceiver Wiring

Any 3.3V RS-485 transceiver works (e.g., MAX3485, SN65HVD72). Connect:

- **DI** ← GP2 (TX data)
- **DE** ← GP3 (driver enable)
- **RE** ← GND (receiver always enabled — required for CSMA/CA and echo)
- **RO** → GP4 (RX data)
- **A/B** → LocalTalk bus

### Bus Termination and Bias

The LocalTalk bus requires:

- **120Ω termination** resistor between A and B at each end of the bus
- **Bias resistors** to hold the bus at a defined idle state when no driver
  is active:
  - A pulled to VCC (3.3V) via 390Ω–1kΩ
  - B pulled to GND via 390Ω–1kΩ

Without bias, the receiver output (RO) oscillates during idle and the FM-0
decoder triggers on noise.

### LocalTalk DIN-8 Pinout

Standard Mini DIN-8 connector for direct connection to a Mac's printer or
modem port:

| DIN-8 Pin | Signal | RS-485 |
|-----------|--------|--------|
| 3 | TxD− | B |
| 5 | RxD− | B (tie to pin 3) |
| 6 | TxD+ | A |
| 8 | RxD+ | A (tie to pin 6) |
| 4 | GND | GND |

The Mac's SCC has separate TX and RX differential pairs. For LocalTalk,
both pairs connect to the same bus: tie TxD− to RxD− (both to RS-485 B)
and TxD+ to RxD+ (both to RS-485 A).

Note: the Mac's LocalTalk transceiver is built into the SCC chip. The DIN-8
port directly carries the RS-422 differential signals — no external
transceiver box is needed on the Mac side (the classic "LocalTalk adapter"
boxes were just passive transformers/terminators).

## Building

### Host-side tests

```sh
cargo test
```

### Firmware

```sh
rustup target add thumbv6m-none-eabi
cargo install elf2uf2-rs
make all     # builds firmware + generates UF2
make flash   # builds + flashes via picotool
```

## Layout

- `tashtalk-proto/` — pure-Rust, `no_std` library. FM-0 codec, HDLC framing,
  SDLC bit-stuffing, CRC, host byte protocol, CSMA/CA backoff state machine.
  Runs unit tests on the host with `cargo test`.
- `firmware/` — Embassy-based firmware for the Pico. Composite USB device with
  two CDC-ACM interfaces:
  - **TT_DATA** — the TashTalk byte protocol. Host drivers open this port.
  - **TT_DEBUG** — human-readable log + interactive debug shell with protocol
    analyzer mode.
- `tools/` — Python tooling for bus analysis (AD2 logic analyzer, bus scope).

## Debug Shell

Connect to the TT_DEBUG CDC port. Commands:

- `s` — status (PIO state, DMA, frame counters, CTS timing)
- `a` — toggle protocol analyzer mode (prints all decoded frames as hex)
- `m` — toggle RX monitor (good-CRC frames only)
- `c` — clear counters
- `b` — reboot to BOOTSEL for flashing
- `?` — full help

## License

GPL-3.0-or-later. The CRC table values, escape sequences, and byte-level
protocol are inherited from Tashtari / lampmerchant's TashTalk
(`firmware/one-chip.asm`, GPLv3). The Rust port is a clean-room
reimplementation; original assembly is not vendored.
