# Measuring RAM usage

How to find out where the 256 KB of RP2040 SRAM actually goes, both
statically (from the ELF) and live (from a running Pico via probe-rs). The
goal is to size the two big consumers — the 64 KB embassy task arena and the
64 KB core-1 stack — down to their measured high-water marks with margin.

See `docs/ltoudp-design.md` for why this matters (RAM is the binding
constraint for adding the Pico W WiFi stack).

## 1. Static breakdown (no hardware)

Totals:

```bash
arm-none-eabi-size target/thumbv6m-none-eabi/release/tashtalk-pico
#   text   data    bss     dec
#  55376      0  191876  247252      <- bss is the RAM number
```

Largest `.bss` symbols (who is eating RAM):

```bash
arm-none-eabi-nm --print-size --size-sort --radix=d \
  target/thumbv6m-none-eabi/release/tashtalk-pico \
  | awk 'toupper($3)=="B"{print $2, $4}' | sort -rn | head -20
```

This is exact for statically-allocated buffers, but it tells you the
*reserved* size of the arena and stack, not how much is actually *used*. For
that you need a live measurement.

## 2. Core-1 stack high-water mark

The stack is painted with the sentinel word `0xC1AC57AC` at boot
(`paint_core1_stack` in `main.rs`). Whatever the stack has touched is no
longer the sentinel, so the deepest untouched sentinel marks the high-water
mark. **Run a representative workload first** (mount a share, copy files — an
AFP session) so the measurement reflects real worst-case depth.

### Easy path — the debug shell

```
stack
STAT core1_stack used=2480 free=63056 total=65536
OK used=2480
```

`used` is the high-water mark. Size the stack to roughly `used × 2` rounded up
(see §4).

### probe-rs path (no firmware interaction)

Symbol addresses change every build — always re-run `nm` after rebuilding.

```bash
ELF=target/thumbv6m-none-eabi/release/tashtalk-pico

# Address of the core-1 stack (its lowest / "deep" end).
ADDR=0x$(nm "$ELF" | awk '/CORE1_STACK/{print $1; exit}')
echo "CORE1_STACK @ $ADDR"

# Read the whole 64KB stack (16384 words) and count untouched sentinels
# from the deep end. Leading sentinels = headroom never used.
probe-rs read --chip RP2040 --protocol swd b32 "$ADDR" 16384 \
  | tr ' ' '\n' | grep -iv '^$' \
  | awk 'BEGIN{u=0;s=0} toupper($1)=="C1AC57AC"&&!hit{u++; next}{hit=1} END{
      total=16384; used=(total-u)*4; print "used="used" free="u*4" total="total*4 }'
```

If reading 16384 words in one call is slow or truncated on your probe, just
**verify headroom** instead: read a block near the deep end and confirm it is
still all sentinel — that proves the stack never reached that depth.

```bash
# Confirm the deepest 32 KB (8192 words) was never touched:
probe-rs read --chip RP2040 --protocol swd b32 "$ADDR" 8192 \
  | tr ' ' '\n' | grep -iv '^$' | sort -u
# Output of only "C1AC57AC" => >=32 KB of headroom; 32 KB stack is safe.
```

## 3. Embassy task arena

The arena (`task-arena-size-65536` in `firmware/Cargo.toml`) is not painted,
and embassy does not expose its usage at runtime on stable. Measure it
**empirically** — this is safe because over-shrinking fails *loudly at boot*
(panic "arena full"), never silently:

1. Lower `task-arena-size-NNNNN` (try `task-arena-size-16384`).
2. `make flash`, then connect to the debug port and confirm the banner +
   `READY` appears and both CDC ports work (USB enumerated, commands respond).
3. If it panicked / failed to enumerate, the arena is too small — step back up.
4. Settle on the smallest size that boots cleanly, then add one size class of
   margin.

The panic, if any, reboots to BOOTSEL (custom panic handler), so a too-small
arena is obvious: the Pico reappears as a mass-storage device instead of
running.

## 4. Sizing with margin

- **Stack:** round `used` up generously — `used × 2`, and not below ~4 KB.
  An undersized stack faults silently (HardFault → dead USB), so prefer slack.
- **Arena:** smallest-that-boots + one size class.
- Re-measure after any change that adds tasks, deepens call chains, or enables
  new features (especially the future embassy-net / cyw43 tasks).

## 5. Checking for a fault after shrinking

If a Pico hangs after a size change, check core 0 for a HardFault over SWD:

```bash
# Bits [8:0] of ICSR = current exception number; 3 = HardFault.
probe-rs read --chip RP2040 --protocol swd b32 0xE000ED04 1
```

(More SWD recipes — reading arbitrary symbols, PIO program counters — are in
the PicoProbe section of `CLAUDE.md`.)
