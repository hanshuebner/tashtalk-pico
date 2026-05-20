# tashtalk-pico

A Raspberry Pi Pico (RP2040) reimplementation of the
[TashTalk](https://github.com/lampmerchant/tashtalk) LocalTalk interface, using
native USB-CDC for the host connection (no CP210x bridge). The host-facing byte
protocol is preserved bit-for-bit so any existing TashTalk host driver — in
particular [TailTalk](https://github.com/feralfirmware/tailtalk)'s `tashtalk`
crate — works unchanged.

See [PICO_PORT_PLAN.md](PICO_PORT_PLAN.md) for the full analysis of the original
PIC firmware and the porting plan.

## Layout

- `tashtalk-proto/` — pure-Rust, `no_std`, host-testable library. CRC, host byte
  codec, node-ID bitmap, SDLC bit-stuffer. Runs unit tests on the host with
  `cargo test`.
- `firmware/` — Embassy-based firmware that runs on the Pico. Composite USB
  device with two CDC-ACM interfaces:
  - **TT_DATA** (first interface) — the TashTalk byte protocol, as
    documented in `tashtalk/documentation/protocol.md`. Existing host
    drivers open this port instead of `/dev/ttyUSB0`.
  - **TT_DEBUG** (second interface) — a human-readable log + interactive
    debug shell. Used during bring-up because there is no logic analyzer in
    the loop yet, and no actual TashTalk board to compare against.

## Building

### Host-side tests (no special toolchain needed)

```sh
cargo test
```

This builds and tests `tashtalk-proto` against the host target. The CRC,
codec, and bit-stuffer all have unit tests here.

### Firmware

The firmware targets `thumbv6m-none-eabi`. The user's current Rust install is
Homebrew-managed and only has host targets, so the cross-compile toolchain
needs one-time setup:

```sh
# Easiest: install rustup alongside Homebrew rust
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
rustup target add thumbv6m-none-eabi
```

Then:

```sh
cd firmware
cargo build --release
```

The resulting ELF is at `target/thumbv6m-none-eabi/release/tashtalk-pico`. To
flash via UF2, use `elf2uf2-rs` or `picotool`:

```sh
cargo install elf2uf2-rs
elf2uf2-rs target/thumbv6m-none-eabi/release/tashtalk-pico tashtalk-pico.uf2
# Hold BOOTSEL, plug in, drag UF2 onto the mass-storage device.
```

`cargo run` is preconfigured to invoke `elf2uf2-rs -d` (auto-mount) if you
prefer.

## Current state at a glance

Steps 1-9 of bring-up are complete and verified end-to-end on a single Pico
in self-loopback (GP2 → GP4 jumper). The full LLAP transmit/receive pipeline
runs on hardware: host byte protocol → frame buffer → PIO TX (FM-0 + SDLC) →
wire → PIO RX (DMA) → FM-0 decode → frame assembly → CRC validate →
autoresponder → autoresponse TX. The wire transmission is verified bit-perfect
by a logic analyzer (`tools/wire_capture.py`, see below).

Steps 10-12 (RS-422 transceiver, two-Pico back-to-back, real Mac integration)
are hardware-gated and not started.

See **[STATUS.md](STATUS.md)** for the detailed development snapshot — what
each subsystem looks like today, the exact verification trail, and known
limits & future work.

## Tooling

- `tools/wire_capture.py` — closed-loop wire capture. Drives WaveForms.app via
  Accessibility/AppleScript against the checked-in `la.dwf3work` logic-analyzer
  workspace. Arms the analyzer, fires a Pico stimulus (`p` debug command or
  proper TT_DATA ENQ), exports CSV, and runs an FM-0 decode. Used to verify
  that what the firmware *says* it transmitted is what's actually on GP2.
- `tools/synth_rx.py` — passive host-side decoder for TT_DATA's escape-encoded
  byte stream. Validates CRC of synthetic frames the firmware streams up.

### Why GUI automation, not the dwf SDK?

The Digilent dwf framework refuses to enumerate the Analog Discovery 2 from any
non-WaveForms process — `FDwfEnum` returns 0 devices with `Adept NOK / ERC 3090`.
WaveForms.app itself has no DriverKit entitlements (just audio/JIT/network), so
this isn't an Apple-permission issue — it's some libdpti-internal state that
only WaveForms.app's own process maintains. Confirmed against WaveForms 3.25.1
(current latest, March 2026); Digilent has shipped no SDK-client fix, and a
Feb 2025 forum thread (#31020) reports the exact same symptom unresolved.
Driving WaveForms.app via Accessibility APIs against a saved workspace
sidesteps the entire issue and gives us a fully scripted capture loop.

## License

GPL-3.0-or-later. The CRC table values, escape sequences, and byte-level
protocol are inherited from Tashtari / lampmerchant's TashTalk
(`firmware/one-chip.asm`, GPLv3). The Rust port itself is a clean-room
reimplementation; original assembly is not vendored.
