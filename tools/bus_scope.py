#!/usr/bin/env python3
"""
LocalTalk bus scope using Analog Discovery 2 via the dwf SDK (Linux).

Captures digital logic analyzer data from the RS-485 transceiver signals
to diagnose frame delivery on the LocalTalk bus.

## Setup

Two Picos with RS-485 transceivers on a shared bus (A-A, B-B, GND-GND).
Each Pico: GP2→DI, GP3→DE+RE (tied), GP4←RO.  Half-duplex: when DE is
high (transmitting), RE is disabled (self-muted).

Probe one transceiver's signals with the AD2:

  Channel mapping (AD2 DIO pins):
    DIO 0 = A   (RS-485 differential +)
    DIO 1 = B   (RS-485 differential -)
    DIO 2 = DI  (data in — Pico TX to transceiver)
    DIO 3 = DE  (driver enable — high = we are transmitting)
    DIO 4 = RE  (receiver enable — active low, active when DE low)
    DIO 5 = RO  (receiver output — data from bus to Pico)

Device symlinks (udev):
    /dev/tashtalk-data   → TT_DATA port (TailTalk opens this)
    /dev/tashtalk-debug  → TT_DEBUG port (log + shell)

Default trigger: DE rising edge (our TX starts).

Usage:
    python3 tools/bus_scope.py                    # single capture, trigger on DE
    python3 tools/bus_scope.py --trigger ro       # trigger on RO (incoming frame)
    python3 tools/bus_scope.py --free             # free-run (no trigger, immediate)
    python3 tools/bus_scope.py --repeat 10        # 10 consecutive captures
    python3 tools/bus_scope.py --duration 0.05    # 50ms capture window
"""

from __future__ import annotations

import argparse
import ctypes
import sys
import time
from pathlib import Path

# ── dwf SDK constants ──────────────────────────────────────────────
# From dwf.h / SDK reference

DwfStateReady = 0
DwfStateConfig = 4
DwfStateArmed = 1
DwfStateDone = 2
DwfStateTrig = 3  # not always used

trigsrcNone = 0
trigsrcDetectorDigitalIn = 3

DwfDigitalInClockSourceInternal = 0
DwfDigitalInSampleModeSimple = 0

# Trigger conditions
DwfTriggerSlopeRise = 0
DwfTriggerSlopeFall = 1
DwfTriggerSlopeEither = 2

CHANNEL_NAMES = {0: "A", 1: "B", 2: "DI", 3: "DE", 4: "RE", 5: "RO"}
NUM_CHANNELS = 6

LOCALTALK_BITRATE = 230_400  # bps


def load_dwf():
    """Load the Digilent WaveForms runtime."""
    try:
        return ctypes.cdll.LoadLibrary("libdwf.so")
    except OSError:
        sys.exit("Cannot load libdwf.so — install the Digilent WaveForms SDK")


def check(dwf, fn_name, *args):
    """Call a dwf function and check for errors."""
    fn = getattr(dwf, fn_name)
    if not fn(*args):
        err = ctypes.create_string_buffer(512)
        dwf.FDwfGetLastErrorMsg(err)
        raise RuntimeError(f"{fn_name} failed: {err.value.decode()}")


def open_device(dwf):
    """Open the first available AD2."""
    hdwf = ctypes.c_int()
    check(dwf, "FDwfDeviceOpen", ctypes.c_int(-1), ctypes.byref(hdwf))
    if hdwf.value == 0:
        sys.exit("No Analog Discovery device found")
    return hdwf


def configure_digital_in(dwf, hdwf, sample_rate_hz: int, buffer_size: int,
                         trigger_channel: int | None, trigger_edge: int,
                         pre_trigger_pct: float):
    """Configure the digital logic analyzer."""
    # Reset
    check(dwf, "FDwfDigitalInReset", hdwf)

    # Internal clock
    check(dwf, "FDwfDigitalInClockSourceSet", hdwf,
          ctypes.c_int(DwfDigitalInClockSourceInternal))

    # Divider: AD2 base clock is 100 MHz
    base_freq = 100_000_000
    divider = max(1, base_freq // sample_rate_hz)
    actual_rate = base_freq / divider
    check(dwf, "FDwfDigitalInDividerSet", hdwf, ctypes.c_int(divider))

    # Sample mode: simple (snapshot each sample)
    check(dwf, "FDwfDigitalInSampleModeSet", hdwf,
          ctypes.c_int(DwfDigitalInSampleModeSimple))

    # Sample format: 8 bits (we only use DIO 0-5)
    check(dwf, "FDwfDigitalInSampleFormatSet", hdwf, ctypes.c_int(8))

    # Buffer size
    check(dwf, "FDwfDigitalInBufferSizeSet", hdwf, ctypes.c_int(buffer_size))

    # Trigger
    if trigger_channel is not None:
        check(dwf, "FDwfDigitalInTriggerSourceSet", hdwf,
              ctypes.c_int(trigsrcDetectorDigitalIn))

        # Pre-trigger: number of samples before trigger
        pre_samples = int(buffer_size * pre_trigger_pct)
        check(dwf, "FDwfDigitalInTriggerPositionSet", hdwf,
              ctypes.c_int(buffer_size - pre_samples))

        # Trigger on specific channel edge
        rise_mask = ctypes.c_uint(0)
        fall_mask = ctypes.c_uint(0)
        if trigger_edge == DwfTriggerSlopeRise:
            rise_mask = ctypes.c_uint(1 << trigger_channel)
        elif trigger_edge == DwfTriggerSlopeFall:
            fall_mask = ctypes.c_uint(1 << trigger_channel)
        else:
            rise_mask = ctypes.c_uint(1 << trigger_channel)
            fall_mask = ctypes.c_uint(1 << trigger_channel)

        check(dwf, "FDwfDigitalInTriggerSet", hdwf,
              ctypes.c_uint(0),   # level low
              ctypes.c_uint(0),   # level high
              rise_mask,          # rising edge
              fall_mask)          # falling edge
    else:
        # Free-run: no trigger
        check(dwf, "FDwfDigitalInTriggerSourceSet", hdwf,
              ctypes.c_int(trigsrcNone))

    return actual_rate


def capture(dwf, hdwf, buffer_size: int, timeout_s: float = 10.0):
    """Arm, wait for trigger, read samples. Returns bytes array."""
    check(dwf, "FDwfDigitalInConfigure", hdwf,
          ctypes.c_int(1),  # reconfigure (force apply settings before start)
          ctypes.c_int(1))  # start

    # Wait for armed → done
    state = ctypes.c_int()
    t0 = time.time()
    while True:
        check(dwf, "FDwfDigitalInStatus", hdwf,
              ctypes.c_int(1),  # read data
              ctypes.byref(state))
        if state.value == DwfStateDone:
            break
        if time.time() - t0 > timeout_s:
            # Return whatever we have
            print(f"[bus_scope] timeout after {timeout_s}s (state={state.value})",
                  file=sys.stderr)
            break
        time.sleep(0.001)

    # Read data (8-bit samples)
    data = (ctypes.c_uint8 * buffer_size)()
    check(dwf, "FDwfDigitalInStatusData", hdwf,
          data, ctypes.c_int(buffer_size))
    return bytes(data)


# ── Decode / analysis ──────────────────────────────────────────────

def extract_channel(data: bytes, ch: int) -> list[int]:
    """Extract a single channel as a list of 0/1 values."""
    mask = 1 << ch
    return [(b & mask) >> ch for b in data]


def find_edges(signal: list[int], edge: str = "rising") -> list[int]:
    """Find sample indices where edges occur."""
    edges = []
    for i in range(1, len(signal)):
        if edge == "rising" and signal[i - 1] == 0 and signal[i] == 1:
            edges.append(i)
        elif edge == "falling" and signal[i - 1] == 1 and signal[i] == 0:
            edges.append(i)
        elif edge == "any" and signal[i] != signal[i - 1]:
            edges.append(i)
    return edges


def find_active_regions(signal: list[int]) -> list[tuple[int, int]]:
    """Find contiguous regions where signal is high. Returns (start, end) pairs."""
    regions = []
    start = None
    for i, v in enumerate(signal):
        if v == 1 and start is None:
            start = i
        elif v == 0 and start is not None:
            regions.append((start, i))
            start = None
    if start is not None:
        regions.append((start, len(signal)))
    return regions


def decode_fm0_from_signal(signal: list[int], sample_rate: float,
                           bitrate: float = LOCALTALK_BITRATE):
    """Decode FM-0 encoded data from a digital signal.
    Returns list of decoded bytes."""
    # Find transitions
    transitions = []
    for i in range(1, len(signal)):
        if signal[i] != signal[i - 1]:
            transitions.append(i)

    if len(transitions) < 16:
        return []

    cell_samples = sample_rate / bitrate
    threshold = cell_samples * 0.75

    # Decode bits
    bits = []
    i = 0
    while i < len(transitions) - 1:
        gap = transitions[i + 1] - transitions[i]
        if gap > threshold:
            bits.append(1)
            i += 1
        else:
            bits.append(0)
            i += 2 if i + 2 < len(transitions) else 1

    # Find HDLC flags (01111110)
    flag = [0, 1, 1, 1, 1, 1, 1, 0]
    flag_positions = []
    for i in range(len(bits) - 7):
        if bits[i:i + 8] == flag:
            flag_positions.append(i)

    if not flag_positions:
        return []

    # Find end of preamble (last contiguous flag)
    preamble_end = flag_positions[0] + 8
    for i in range(1, len(flag_positions)):
        if flag_positions[i] == flag_positions[i - 1] + 8:
            preamble_end = flag_positions[i] + 8
        else:
            break

    # Find closing flag
    closing = None
    for fp in flag_positions:
        if fp >= preamble_end + 8:
            closing = fp
            break

    payload_bits = bits[preamble_end:closing] if closing else bits[preamble_end:]

    # Bit-unstuff (remove 0 after five consecutive 1s)
    unstuffed = []
    ones = 0
    for b in payload_bits:
        if ones == 5 and b == 0:
            ones = 0
            continue
        unstuffed.append(b)
        ones = ones + 1 if b == 1 else 0

    # Convert to bytes (LSB first)
    frame_bytes = []
    for i in range(0, len(unstuffed) - 7, 8):
        v = 0
        for j in range(8):
            if unstuffed[i + j]:
                v |= 1 << j
        frame_bytes.append(v)

    return frame_bytes


LLAP_TYPES = {
    0x01: "DDP-short",
    0x02: "DDP-long",
    0x81: "ENQ",
    0x82: "ACK",
    0x84: "RTS",
    0x85: "CTS",
}


def describe_llap(frame_bytes: list[int]) -> str:
    """Human-readable LLAP frame description."""
    if len(frame_bytes) < 3:
        return f"  (too short: {len(frame_bytes)} bytes)"
    dst, src, typ = frame_bytes[0], frame_bytes[1], frame_bytes[2]
    type_name = LLAP_TYPES.get(typ, f"0x{typ:02x}")
    parts = [f"  dst=0x{dst:02x} src=0x{src:02x} type={type_name}"]
    if len(frame_bytes) > 5:  # has payload + 2-byte CRC
        payload = frame_bytes[3:-2]
        crc = frame_bytes[-2:]
        parts.append(f"  payload ({len(payload)}B): {' '.join(f'{b:02x}' for b in payload[:32])}"
                     + ("..." if len(payload) > 32 else ""))
        parts.append(f"  CRC: {crc[0]:02x} {crc[1]:02x}")
    elif len(frame_bytes) == 5:
        parts.append(f"  CRC: {frame_bytes[3]:02x} {frame_bytes[4]:02x}")
    return "\n".join(parts)


def analyze_capture(data: bytes, sample_rate: float):
    """Analyze a capture and print findings."""
    de = extract_channel(data, 3)   # driver enable — our TX
    ro = extract_channel(data, 5)   # receiver output — bus data seen by us
    di = extract_channel(data, 2)   # data we're sending
    a = extract_channel(data, 0)    # differential A
    b = extract_channel(data, 1)    # differential B

    us_per_sample = 1e6 / sample_rate

    # Find our TX regions (DE high)
    our_tx = find_active_regions(de)
    print(f"\n=== Capture: {len(data)} samples @ {sample_rate/1e6:.1f} MHz "
          f"({len(data) * us_per_sample / 1000:.1f} ms) ===")

    if not our_tx:
        print("  DE never went high — no outbound TX in this capture.")
    else:
        for i, (start, end) in enumerate(our_tx):
            dur_us = (end - start) * us_per_sample
            t_start_us = start * us_per_sample
            print(f"\n  TX#{i}: {t_start_us:.1f}us .. +{dur_us:.1f}us "
                  f"(samples {start}-{end})")

            # Decode the frame we sent (from DI during DE high)
            tx_signal = di[start:end]
            tx_bytes = decode_fm0_from_signal(tx_signal, sample_rate)
            if tx_bytes:
                print(f"  Our frame ({len(tx_bytes)}B):")
                print(describe_llap(tx_bytes))
            else:
                print("  (could not decode our TX)")

    # Find bus activity on RO (when DE is low = we're not transmitting)
    # RO is only valid when RE is low (active), which happens when DE is low
    # (RE is tied to DE, inverted)
    ro_when_rx = [ro[i] if de[i] == 0 else -1 for i in range(len(data))]

    # Find active regions on RO when we're not transmitting
    rx_regions = []
    start = None
    for i in range(1, len(ro_when_rx)):
        if ro_when_rx[i] != ro_when_rx[i - 1] and ro_when_rx[i] != -1:
            if start is None:
                start = i
        elif ro_when_rx[i] == -1 and start is not None:
            rx_regions.append((start, i))
            start = None
    # Also try decoding RO transitions in non-DE regions
    rx_activity_edges = []
    for i in range(1, len(data)):
        if de[i] == 0 and ro[i] != ro[i - 1]:
            rx_activity_edges.append(i)

    if rx_activity_edges:
        # Group edges into frames (gap > 2 bit times = new frame)
        gap_threshold = int(sample_rate / LOCALTALK_BITRATE * 4)
        frame_groups = []
        group_start = rx_activity_edges[0]
        prev = rx_activity_edges[0]
        for edge in rx_activity_edges[1:]:
            if edge - prev > gap_threshold:
                frame_groups.append((group_start, prev))
                group_start = edge
            prev = edge
        frame_groups.append((group_start, prev))

        print(f"\n  RX activity (bus, {len(frame_groups)} frame group(s) "
              f"while DE=low):")
        for i, (gs, ge) in enumerate(frame_groups):
            # Extend window slightly for decode
            ws = max(0, gs - int(sample_rate / LOCALTALK_BITRATE))
            we = min(len(data), ge + int(sample_rate / LOCALTALK_BITRATE * 2))
            t_us = gs * us_per_sample
            dur_us = (ge - gs) * us_per_sample
            print(f"\n    RX#{i}: {t_us:.1f}us .. +{dur_us:.1f}us")
            rx_bytes = decode_fm0_from_signal(ro[ws:we], sample_rate)
            if rx_bytes:
                print(f"    Frame ({len(rx_bytes)}B):")
                print("  " + describe_llap(rx_bytes).replace("\n", "\n  "))
            else:
                print(f"    (could not decode, {ge - gs} edge span)")
    else:
        print("\n  No RX activity on RO while DE=low")

    # Collision detection: is there RO activity during or overlapping with DE?
    # Since RE=DE (inverted), we can't see RO during our TX, but check edges
    # right before/after our TX
    if our_tx:
        print("\n  --- Collision check ---")
        for i, (start, end) in enumerate(our_tx):
            # Check 50us before our TX for bus activity
            pre_window = int(50e-6 * sample_rate)
            pre_start = max(0, start - pre_window)
            pre_edges = sum(1 for j in range(pre_start + 1, start)
                            if ro[j] != ro[j - 1])

            # Check 50us after our TX
            post_window = int(50e-6 * sample_rate)
            post_end = min(len(data), end + post_window)
            post_edges = sum(1 for j in range(end + 1, post_end)
                             if ro[j] != ro[j - 1])

            pre_us = pre_window * us_per_sample
            post_us = post_window * us_per_sample
            print(f"  TX#{i}: {pre_edges} RO edges in {pre_us:.0f}us before, "
                  f"{post_edges} RO edges in {post_us:.0f}us after")
            if pre_edges > 2:
                print(f"    ** Bus was ACTIVE just before our TX! "
                      f"Possible CSMA failure.")
            if post_edges > 2:
                # Might be the Mac's response, or a collision tail
                pass

    # A/B differential — check for toggling during DE (signal integrity)
    if our_tx:
        print("\n  --- Signal integrity (A/B during our TX) ---")
        for i, (start, end) in enumerate(our_tx):
            a_edges = sum(1 for j in range(start + 1, end)
                          if a[j] != a[j - 1])
            b_edges = sum(1 for j in range(start + 1, end)
                          if b[j] != b[j - 1])
            dur_us = (end - start) * us_per_sample
            expected_transitions = dur_us / (1e6 / LOCALTALK_BITRATE) * 1.5
            print(f"  TX#{i}: A={a_edges} edges, B={b_edges} edges "
                  f"(~{expected_transitions:.0f} expected for FM-0)")


def save_csv(data: bytes, sample_rate: float, path: str):
    """Save capture as CSV for external analysis."""
    with open(path, "w") as f:
        f.write("# Sample rate: {:.0f} Hz\n".format(sample_rate))
        f.write("# Channels: " +
                ", ".join(f"DIO{k}={v}" for k, v in CHANNEL_NAMES.items()) +
                "\n")
        f.write("time_us,A,B,DI,DE,RE,RO\n")
        us_per_sample = 1e6 / sample_rate
        for i, byte in enumerate(data):
            t = i * us_per_sample
            vals = [(byte >> ch) & 1 for ch in range(NUM_CHANNELS)]
            f.write(f"{t:.3f},{','.join(str(v) for v in vals)}\n")
    print(f"  Saved CSV: {path}")


def main() -> int:
    p = argparse.ArgumentParser(description="LocalTalk bus scope (AD2 + dwf SDK)")
    p.add_argument("--rate", type=float, default=10.0,
                   help="Sample rate in MHz (default: 10)")
    p.add_argument("--duration", type=float, default=0.02,
                   help="Capture duration in seconds (default: 0.02 = 20ms)")
    p.add_argument("--trigger", choices=["de", "ro", "a", "di"], default="de",
                   help="Trigger channel (default: de = our TX)")
    p.add_argument("--trigger-edge", choices=["rise", "fall", "any"],
                   default="rise")
    p.add_argument("--free", action="store_true",
                   help="Free-run capture (no trigger)")
    p.add_argument("--pre-trigger", type=float, default=0.2,
                   help="Pre-trigger fraction 0..1 (default: 0.2 = 20%%)")
    p.add_argument("--repeat", type=int, default=1,
                   help="Number of captures")
    p.add_argument("--timeout", type=float, default=10.0,
                   help="Trigger timeout in seconds")
    p.add_argument("--csv", type=str, default=None,
                   help="Save capture(s) to CSV (appends _N for repeats)")
    args = p.parse_args()

    trigger_ch_map = {"de": 3, "ro": 5, "a": 0, "di": 2}
    edge_map = {"rise": DwfTriggerSlopeRise, "fall": DwfTriggerSlopeFall,
                "any": DwfTriggerSlopeEither}

    sample_rate = int(args.rate * 1e6)
    buffer_size = int(sample_rate * args.duration)

    # AD2 digital buffer max is typically 16384 or 32768
    # Clamp and warn
    max_buffer = 32768
    if buffer_size > max_buffer:
        print(f"[bus_scope] buffer {buffer_size} > {max_buffer}, clamping. "
              f"Effective duration: {max_buffer / sample_rate * 1000:.1f}ms",
              file=sys.stderr)
        buffer_size = max_buffer

    dwf = load_dwf()
    hdwf = open_device(dwf)

    trigger_ch = None if args.free else trigger_ch_map[args.trigger]
    trigger_edge = edge_map[args.trigger_edge]

    try:
        actual_rate = configure_digital_in(
            dwf, hdwf, sample_rate, buffer_size,
            trigger_ch, trigger_edge, args.pre_trigger)
        print(f"[bus_scope] {actual_rate/1e6:.1f} MHz, {buffer_size} samples "
              f"({buffer_size / actual_rate * 1000:.1f}ms), "
              f"trigger: {'free-run' if args.free else f'DIO{trigger_ch} ({CHANNEL_NAMES[trigger_ch]}) {args.trigger_edge}'}")

        for rep in range(args.repeat):
            if args.repeat > 1:
                print(f"\n{'='*60}\nCapture {rep + 1}/{args.repeat}")

            print("[bus_scope] waiting for trigger..." if trigger_ch is not None
                  else "[bus_scope] capturing...", flush=True)
            data = capture(dwf, hdwf, buffer_size, args.timeout)
            analyze_capture(data, actual_rate)

            if args.csv:
                csv_path = args.csv
                if args.repeat > 1:
                    stem = Path(csv_path).stem
                    suffix = Path(csv_path).suffix or ".csv"
                    csv_path = str(Path(csv_path).parent / f"{stem}_{rep}{suffix}")
                save_csv(data, actual_rate, csv_path)

    finally:
        dwf.FDwfDeviceClose(hdwf)

    return 0


if __name__ == "__main__":
    sys.exit(main())
