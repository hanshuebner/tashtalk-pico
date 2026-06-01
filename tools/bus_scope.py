#!/usr/bin/env python3
"""
LocalTalk bus scope using Analog Discovery 2 via the ad2 module.

Captures digital logic analyzer data from the RS-485 transceiver signals
to diagnose frame delivery on the LocalTalk bus.

Channel mapping (AD2 DIO pins):
    DIO 0 = A   (RS-485 differential +)
    DIO 1 = B   (RS-485 differential -)
    DIO 2 = DI  (data in -- Pico TX to transceiver)
    DIO 3 = DE  (driver enable -- high = we are transmitting)
    DIO 4 = RE  (receiver enable -- active low)
    DIO 5 = RO  (receiver output -- data from bus to Pico)

Usage:
    python3 tools/bus_scope.py                    # single capture, trigger on DE
    python3 tools/bus_scope.py --trigger ro       # trigger on RO (incoming frame)
    python3 tools/bus_scope.py --free             # free-run (no trigger, immediate)
    python3 tools/bus_scope.py --repeat 10        # 10 consecutive captures
    python3 tools/bus_scope.py --duration 0.005   # 5ms capture window
    python3 tools/bus_scope.py --pins             # just read current pin state
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from ad2 import (
    Ad2Device, CHANNELS, LOCALTALK_BITRATE,
    extract_channel, find_active_regions, find_edges,
    decode_localtalk, summarize_capture, save_csv,
)


def analyze_capture_detailed(data: bytes, sample_rate: float):
    """Detailed analysis with collision detection and signal integrity."""
    us_per = 1e6 / sample_rate

    de = extract_channel(data, "DE")
    ro = extract_channel(data, "RO")
    di = extract_channel(data, "DI")
    a = extract_channel(data, "A")
    b = extract_channel(data, "B")

    total_ms = len(data) * us_per / 1000
    print(f"\n=== Capture: {len(data)} samples @ {sample_rate/1e6:.1f} MHz "
          f"({total_ms:.1f} ms) ===")

    # TX regions
    our_tx = find_active_regions(de)
    if not our_tx:
        print("  DE never went high -- no outbound TX in this capture.")
    else:
        for i, (start, end) in enumerate(our_tx):
            dur_us = (end - start) * us_per
            t_us = start * us_per
            print(f"\n  TX#{i}: {t_us:.1f}us .. +{dur_us:.1f}us "
                  f"(samples {start}-{end})")
            tx_frames = decode_localtalk(data, sample_rate, "DI")
            if tx_frames:
                for f in tx_frames:
                    print(f"    {f}")
            else:
                print("    (could not decode TX)")

    # RX frames
    rx_frames = decode_localtalk(data, sample_rate, "RO")
    if rx_frames:
        print(f"\n  RX activity ({len(rx_frames)} frame(s) on RO):")
        for i, f in enumerate(rx_frames):
            print(f"    RX#{i}: {f}")
    else:
        # Count RO edges when DE is low
        ro_edges = [j for j in range(1, len(data))
                    if de[j] == 0 and ro[j] != ro[j - 1]]
        if ro_edges:
            print(f"\n  RO activity: {len(ro_edges)} edges while DE=low "
                  f"(not decodable)")
        else:
            print("\n  No RX activity on RO while DE=low")

    # Collision check
    if our_tx:
        print("\n  --- Collision check ---")
        for i, (start, end) in enumerate(our_tx):
            pre_window = int(50e-6 * sample_rate)
            pre_start = max(0, start - pre_window)
            pre_edges = sum(1 for j in range(pre_start + 1, start)
                            if ro[j] != ro[j - 1])
            post_window = int(50e-6 * sample_rate)
            post_end = min(len(data), end + post_window)
            post_edges = sum(1 for j in range(end + 1, post_end)
                             if ro[j] != ro[j - 1])
            print(f"  TX#{i}: {pre_edges} RO edges in 50us before, "
                  f"{post_edges} RO edges in 50us after")
            if pre_edges > 2:
                print(f"    ** Bus was ACTIVE just before our TX! "
                      f"Possible CSMA failure.")

    # Signal integrity
    if our_tx:
        print("\n  --- Signal integrity (A/B during TX) ---")
        for i, (start, end) in enumerate(our_tx):
            a_edges = sum(1 for j in range(start + 1, end)
                          if a[j] != a[j - 1])
            b_edges = sum(1 for j in range(start + 1, end)
                          if b[j] != b[j - 1])
            dur_us = (end - start) * us_per
            expected = dur_us / (1e6 / LOCALTALK_BITRATE) * 1.5
            print(f"  TX#{i}: A={a_edges} edges, B={b_edges} edges "
                  f"(~{expected:.0f} expected for FM-0)")


def main() -> int:
    p = argparse.ArgumentParser(
        description="LocalTalk bus scope (AD2 + dwf SDK)")
    p.add_argument("--rate", type=float, default=10.0,
                   help="Sample rate in MHz (default: 10)")
    p.add_argument("--duration", type=float, default=0.005,
                   help="Capture duration in seconds (default: 0.005 = 5ms)")
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
                   help="Save capture(s) to CSV")
    p.add_argument("--pins", action="store_true",
                   help="Just read and display current pin state")
    p.add_argument("--summary", action="store_true",
                   help="Use compact summary instead of detailed analysis")
    args = p.parse_args()

    with Ad2Device() as dev:
        if args.pins:
            pins = dev.read_pins()
            print("Current pin state:")
            for name in ["A", "B", "DI", "DE", "RE", "RO"]:
                print(f"  {name:3s} = {pins[name]}")
            return 0

        sample_rate = int(args.rate * 1e6)
        buf_size = min(int(sample_rate * args.duration), dev._max_buffer)
        actual_dur = buf_size / sample_rate

        trig_desc = "free-run" if args.free else \
            f"{args.trigger.upper()} {args.trigger_edge}"
        print(f"[bus_scope] {args.rate:.1f} MHz, {buf_size} samples "
              f"({actual_dur*1000:.1f}ms), trigger: {trig_desc}")

        for rep in range(args.repeat):
            if args.repeat > 1:
                print(f"\n{'='*60}\nCapture {rep + 1}/{args.repeat}")

            if args.free:
                print("[bus_scope] capturing...", flush=True)
                data, rate = dev.capture_free(
                    duration_s=args.duration,
                    sample_rate_mhz=args.rate,
                    timeout_s=args.timeout)
            else:
                print("[bus_scope] waiting for trigger...", flush=True)
                data, rate = dev.capture_triggered(
                    channel=args.trigger.upper(),
                    edge=args.trigger_edge,
                    duration_s=args.duration,
                    pre_trigger_pct=args.pre_trigger,
                    sample_rate_mhz=args.rate,
                    timeout_s=args.timeout)

            if args.summary:
                print(summarize_capture(data, rate))
            else:
                analyze_capture_detailed(data, rate)

            if args.csv:
                csv_path = args.csv
                if args.repeat > 1:
                    stem = Path(csv_path).stem
                    suffix = Path(csv_path).suffix or ".csv"
                    csv_path = str(Path(csv_path).parent / f"{stem}_{rep}{suffix}")
                save_csv(data, rate, csv_path)
                print(f"  Saved CSV: {csv_path}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
