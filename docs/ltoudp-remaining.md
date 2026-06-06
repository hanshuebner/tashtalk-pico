# LToUDP Bridge — Remaining Work

## Must fix

- **Multicast TX**: CYW43 binary firmware blob silently drops multicast TX
  frames. Confirmed: `send_to(multicast)` returns Ok at smoltcp level, Rust
  driver sends frame to CYW43 via SPI without filtering, but firmware never
  puts it on the air. Not documented anywhere upstream. Using subnet-directed
  broadcast (derived from DHCP config via `Ipv4Cidr::broadcast()`) as the
  LToUDP spec allows. All speakers on one segment must use the same mode.

## Should fix

- **WiFi disconnect detection**: `wifi_task` sets `STATE_CONNECTED` on
  successful join but never detects disconnection. The 30-second heartbeat
  loop should check link state and break to trigger re-scan/rejoin.

- **Flash persistence for WiFi credentials**: currently RAM-only, lost on
  reboot. Plan: use embassy-rp Flash peripheral to store a config struct
  (magic + CRC) in a dedicated 4KB sector near end of flash. Read at boot,
  write via `wifi add`.

- **Bridge ingress testing**: UDP → wire path is implemented but untested.
  Needs an LToUDP peer (tashrouter, Mini vMac, or a second Pico W running
  the same firmware) to send UDP packets that get injected onto the bus.

## Nice to have

- **`wifi scan` command**: expose cyw43 scan results via debug shell for
  diagnostics. Requires sharing `Control` with the debug task (currently
  owned by wifi_task).

- **Bridge counters in `clear`**: `clear` command should also reset bridge
  egress/ingress/drop counters.

- **Multicast vs broadcast config**: make the LToUDP transport mode
  (multicast/broadcast) configurable via debug shell. All speakers on one
  segment must agree (per spec).
