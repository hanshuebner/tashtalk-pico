#!/usr/bin/env python3
"""
Mass test for RTS/CTS handshake and CSMA/CA between two Picos.

Speaks the TashTalk host protocol on two serial ports, configures each
Pico with a node ID, and blasts directed data frames between them.
Verifies delivery and reports statistics.

Usage:
    python3 tools/mass_test.py                          # use /dev/tashtalk-data{,-000001}
    python3 tools/mass_test.py --port-a /dev/ttyACM0 --port-b /dev/ttyACM3
    python3 tools/mass_test.py --count 500 --payload 100
    python3 tools/mass_test.py --bidirectional          # send A→B and B→A simultaneously
"""

from __future__ import annotations

import argparse
import os
import struct
import sys
import threading
import time
from collections import defaultdict
from dataclasses import dataclass, field

import serial

# ── LocalTalk CRC (X.25: init 0xFFFF, poly 0x8408 reflected, xor 0xFFFF) ──

def lt_crc(data: bytes) -> bytes:
    """Compute the LocalTalk FCS (2 bytes, little-endian)."""
    reg = 0xFFFF
    for b in data:
        reg ^= b
        for _ in range(8):
            if reg & 1:
                reg = (reg >> 1) ^ 0x8408
            else:
                reg >>= 1
    reg ^= 0xFFFF
    return bytes([reg & 0xFF, (reg >> 8) & 0xFF])


# ── TashTalk host protocol helpers ──

def make_set_node_ids(node: int) -> bytes:
    """Build a SetNodeIds command (0x02 + 32-byte bitmap)."""
    bitmap = bytearray(32)
    bitmap[node >> 3] |= 1 << (node & 7)
    return b'\x02' + bytes(bitmap)


def make_set_features(calc_crc: bool = False, check_crc: bool = False) -> bytes:
    """Build a SetFeatures command (0x03 + 1 byte)."""
    feat = 0
    if calc_crc:
        feat |= 0x80
    if check_crc:
        feat |= 0x40
    return b'\x03' + bytes([feat])


def make_transmit_frame(dest: int, src: int, llap_type: int,
                        payload: bytes) -> bytes:
    """Build a TransmitFrame command (0x01 + frame bytes).

    For data frames (type < 0x80), bytes 3-4 carry the DDP length field.
    The length = len(payload) + 2 (for the length field itself) + 2 (CRC
    is not counted in the DDP length but IS appended to the frame).
    """
    if llap_type & 0x80:
        # Control frame: 3 header + 2 CRC = 5 bytes total
        hdr = bytes([dest, src, llap_type])
        crc = lt_crc(hdr)
        return b'\x01' + hdr + crc
    else:
        # Data frame: header + length field + payload + CRC
        # DDP length = 2 (length field) + len(payload)
        ddp_len = 2 + len(payload)
        len_hi = (llap_type & 0xFC) | ((ddp_len >> 8) & 0x03)
        len_lo = ddp_len & 0xFF
        frame = bytes([dest, src, llap_type, len_hi, len_lo]) + payload
        crc = lt_crc(frame)
        return b'\x01' + frame + crc


# ── Frame receiver (runs in a thread) ──

@dataclass
class RxStats:
    frames: list = field(default_factory=list)
    lock: threading.Lock = field(default_factory=threading.Lock)
    errors: int = 0
    aborts: int = 0

    def add_frame(self, frame: bytes, status: int):
        with self.lock:
            self.frames.append((time.monotonic(), frame, status))

    def count_good(self) -> int:
        with self.lock:
            return sum(1 for _, _, s in self.frames if s == 0xFD)


def rx_thread(port: serial.Serial, stats: RxStats, stop: threading.Event,
              name: str):
    """Read and decode TashTalk device-to-host escape-encoded frames."""
    buf = bytearray()
    in_escape = False

    while not stop.is_set():
        data = port.read(port.in_waiting or 1)
        if not data:
            continue
        for b in data:
            if in_escape:
                in_escape = False
                if b == 0xFF:
                    # Literal 0x00 in frame body
                    buf.append(0x00)
                elif b in (0xFD, 0xFC, 0xFE, 0xFA):
                    # End of frame
                    frame = bytes(buf)
                    buf.clear()
                    stats.add_frame(frame, b)
                    if b == 0xFC:
                        stats.errors += 1
                    elif b in (0xFE, 0xFA):
                        stats.aborts += 1
                else:
                    # Unknown escape — reset
                    buf.clear()
            elif b == 0x00:
                in_escape = True
            else:
                buf.append(b)


# ── Main test logic ──

def find_ports() -> tuple[str, str]:
    """Auto-detect two TashTalk data ports."""
    candidates = []
    for name in sorted(os.listdir('/dev')):
        if name.startswith('tashtalk-data'):
            candidates.append(f'/dev/{name}')
    # Filter out serial-number symlinks if generic ones exist
    generic = [p for p in candidates if p.count('-') == 1]  # /dev/tashtalk-data
    by_serial = [p for p in candidates if len(p.split('-')) > 2]

    if len(by_serial) >= 2:
        return by_serial[0], by_serial[1]
    if len(candidates) >= 2:
        return candidates[0], candidates[1]
    sys.exit(f"Need 2 TashTalk data ports, found: {candidates}")


def run_test(port_a_path: str, port_b_path: str, count: int,
             payload_size: int, bidirectional: bool, delay_ms: float,
             timeout_s: float):
    node_a, node_b = 10, 20

    print(f"Opening ports: A={port_a_path}  B={port_b_path}")
    port_a = serial.Serial(port_a_path, timeout=0.05)
    port_b = serial.Serial(port_b_path, timeout=0.05)

    # Flush
    port_a.reset_input_buffer()
    port_b.reset_input_buffer()
    time.sleep(0.1)

    # Configure nodes
    print(f"Configuring: A=node {node_a}, B=node {node_b}")
    port_a.write(make_set_features())
    port_a.write(make_set_node_ids(node_a))
    port_b.write(make_set_features())
    port_b.write(make_set_node_ids(node_b))
    time.sleep(0.2)
    port_a.reset_input_buffer()
    port_b.reset_input_buffer()

    # Start receivers
    stats_a = RxStats()  # frames received by A (sent by B)
    stats_b = RxStats()  # frames received by B (sent by A)
    stop = threading.Event()
    t_a = threading.Thread(target=rx_thread,
                           args=(port_a, stats_a, stop, "A"), daemon=True)
    t_b = threading.Thread(target=rx_thread,
                           args=(port_b, stats_b, stop, "B"), daemon=True)
    t_a.start()
    t_b.start()

    # Build test payloads — each frame carries a sequence number
    def make_payload(seq: int, size: int) -> bytes:
        # First 4 bytes = sequence number (big-endian), rest = pattern
        hdr = struct.pack('>I', seq)
        pad = bytes([(seq + i) & 0xFF for i in range(max(0, size - 4))])
        return hdr + pad

    # Transmit
    sent_a_to_b = 0
    sent_b_to_a = 0
    t0 = time.monotonic()

    print(f"\nSending {count} directed data frames "
          f"({'A↔B bidirectional' if bidirectional else 'A→B'}, "
          f"payload={payload_size}B, delay={delay_ms}ms)...\n")

    for seq in range(count):
        # A → B
        payload = make_payload(seq, payload_size)
        cmd = make_transmit_frame(node_b, node_a, 0x01, payload)
        port_a.write(cmd)
        sent_a_to_b += 1

        if bidirectional:
            # B → A
            payload = make_payload(seq + 0x80000000, payload_size)
            cmd = make_transmit_frame(node_a, node_b, 0x01, payload)
            port_b.write(cmd)
            sent_b_to_a += 1

        if delay_ms > 0:
            time.sleep(delay_ms / 1000.0)

        # Progress
        if (seq + 1) % 100 == 0 or seq == count - 1:
            elapsed = time.monotonic() - t0
            rx_b = stats_b.count_good()
            rx_a = stats_a.count_good() if bidirectional else 0
            total_sent = sent_a_to_b + sent_b_to_a
            total_rx = rx_b + rx_a
            rate = total_sent / elapsed if elapsed > 0 else 0
            print(f"  [{seq+1:5d}/{count}] sent={total_sent} "
                  f"rx={total_rx} "
                  f"({rate:.0f} frames/s, {elapsed:.1f}s)")

    # Wait for in-flight frames to arrive
    tx_done = time.monotonic()
    print(f"\nAll frames sent in {tx_done - t0:.1f}s. "
          f"Waiting {timeout_s}s for stragglers...")
    time.sleep(timeout_s)
    stop.set()
    t_a.join(timeout=1)
    t_b.join(timeout=1)
    total_time = time.monotonic() - t0

    # ── Results ──
    print("\n" + "=" * 60)
    print("RESULTS")
    print("=" * 60)

    # Analyze received frames
    def analyze(stats: RxStats, direction: str, expected: int):
        with stats.lock:
            good = [f for f in stats.frames if f[2] == 0xFD]
            bad_crc = [f for f in stats.frames if f[2] == 0xFC]
            framing = [f for f in stats.frames if f[2] == 0xFE]
            aborted = [f for f in stats.frames if f[2] == 0xFA]

        # Extract sequence numbers from good frames
        seqs = set()
        for _, frame, _ in good:
            if len(frame) >= 9:  # 3 hdr + 2 len + 4 seq
                seq = struct.unpack('>I', frame[5:9])[0]
                seqs.add(seq)

        # Check for duplicates
        dup_count = len(good) - len(seqs)

        # Check for missing
        missing = expected - len(seqs)

        loss_pct = (missing / expected * 100) if expected > 0 else 0

        print(f"\n  {direction}:")
        print(f"    Sent:       {expected}")
        print(f"    Received:   {len(good)} good, "
              f"{len(bad_crc)} bad-CRC, "
              f"{len(framing)} framing-err, "
              f"{len(aborted)} aborted")
        print(f"    Unique seq: {len(seqs)}")
        print(f"    Duplicates: {dup_count}")
        print(f"    Missing:    {missing} ({loss_pct:.1f}% loss)")

        if good:
            # Timing between received frames
            times = [f[0] for f in good]
            gaps = [times[i+1] - times[i] for i in range(len(times)-1)]
            if gaps:
                avg_gap = sum(gaps) / len(gaps) * 1000
                max_gap = max(gaps) * 1000
                print(f"    Avg gap:    {avg_gap:.1f}ms")
                print(f"    Max gap:    {max_gap:.1f}ms "
                      f"{'** POSSIBLE HANG **' if max_gap > 500 else ''}")

        return len(seqs), missing

    rx_b_ok, miss_b = analyze(stats_b, "A → B", sent_a_to_b)
    if bidirectional:
        rx_a_ok, miss_a = analyze(stats_a, "B → A", sent_b_to_a)
    else:
        rx_a_ok, miss_a = 0, 0

    total_sent = sent_a_to_b + sent_b_to_a
    total_rx = rx_b_ok + rx_a_ok
    total_miss = miss_b + miss_a

    print(f"\n  Total time:   {total_time:.1f}s")
    print(f"  Throughput:   {total_sent / total_time:.0f} frames/s sent, "
          f"{total_rx / total_time:.0f} frames/s delivered")

    if total_miss == 0:
        print("\n  ✓ ALL FRAMES DELIVERED — no loss, no hangs")
    else:
        print(f"\n  ✗ {total_miss} frames lost "
              f"({total_miss / total_sent * 100:.1f}%)")

    port_a.close()
    port_b.close()
    return 0 if total_miss == 0 else 1


def main():
    p = argparse.ArgumentParser(
        description="Mass test RTS/CTS + CSMA/CA between two Picos")
    p.add_argument("--port-a", type=str, default=None,
                   help="Serial port for Pico A (default: auto-detect)")
    p.add_argument("--port-b", type=str, default=None,
                   help="Serial port for Pico B (default: auto-detect)")
    p.add_argument("--count", "-n", type=int, default=200,
                   help="Number of frames to send (default: 200)")
    p.add_argument("--payload", type=int, default=50,
                   help="Payload size in bytes (default: 50)")
    p.add_argument("--bidirectional", "-b", action="store_true",
                   help="Send in both directions simultaneously")
    p.add_argument("--delay", type=float, default=5.0,
                   help="Delay between frames in ms (default: 5)")
    p.add_argument("--timeout", type=float, default=3.0,
                   help="Wait time after last TX for stragglers (default: 3s)")
    args = p.parse_args()

    if args.port_a and args.port_b:
        port_a, port_b = args.port_a, args.port_b
    else:
        port_a, port_b = find_ports()

    return run_test(port_a, port_b, args.count, args.payload,
                    args.bidirectional, args.delay, args.timeout)


if __name__ == "__main__":
    sys.exit(main())
