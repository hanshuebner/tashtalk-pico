#!/usr/bin/env python3
"""
Two-Pico LocalTalk test harness.

Drives two Picos via their TT_DEBUG serial ports to validate each
protocol layer bottom-up: framing, echo, bidirectional, timing,
CSMA/CA, RTS/CTS/DATA dialog, and stress.

Usage:
    python3 tools/test_harness.py [--layer N] [--verbose] [--config FILE]
"""

import argparse
import glob
import queue
import re
import serial
import sys
import threading
import time
from dataclasses import dataclass
from typing import Optional

# ── CRC-16 (ported from tashtalk-proto/src/crc.rs) ────────────────

LT_CRC_LUT = [
    0x0000, 0x1189, 0x2312, 0x329B, 0x4624, 0x57AD, 0x6536, 0x74BF,
    0x8C48, 0x9DC1, 0xAF5A, 0xBED3, 0xCA6C, 0xDBE5, 0xE97E, 0xF8F7,
    0x1081, 0x0108, 0x3393, 0x221A, 0x56A5, 0x472C, 0x75B7, 0x643E,
    0x9CC9, 0x8D40, 0xBFDB, 0xAE52, 0xDAED, 0xCB64, 0xF9FF, 0xE876,
    0x2102, 0x308B, 0x0210, 0x1399, 0x6726, 0x76AF, 0x4434, 0x55BD,
    0xAD4A, 0xBCC3, 0x8E58, 0x9FD1, 0xEB6E, 0xFAE7, 0xC87C, 0xD9F5,
    0x3183, 0x200A, 0x1291, 0x0318, 0x77A7, 0x662E, 0x54B5, 0x453C,
    0xBDCB, 0xAC42, 0x9ED9, 0x8F50, 0xFBEF, 0xEA66, 0xD8FD, 0xC974,
    0x4204, 0x538D, 0x6116, 0x709F, 0x0420, 0x15A9, 0x2732, 0x36BB,
    0xCE4C, 0xDFC5, 0xED5E, 0xFCD7, 0x8868, 0x99E1, 0xAB7A, 0xBAF3,
    0x5285, 0x430C, 0x7197, 0x601E, 0x14A1, 0x0528, 0x37B3, 0x263A,
    0xDECD, 0xCF44, 0xFDDF, 0xEC56, 0x98E9, 0x8960, 0xBBFB, 0xAA72,
    0x6306, 0x728F, 0x4014, 0x519D, 0x2522, 0x34AB, 0x0630, 0x17B9,
    0xEF4E, 0xFEC7, 0xCC5C, 0xDDD5, 0xA96A, 0xB8E3, 0x8A78, 0x9BF1,
    0x7387, 0x620E, 0x5095, 0x411C, 0x35A3, 0x242A, 0x16B1, 0x0738,
    0xFFCF, 0xEE46, 0xDCDD, 0xCD54, 0xB9EB, 0xA862, 0x9AF9, 0x8B70,
    0x8408, 0x9581, 0xA71A, 0xB693, 0xC22C, 0xD3A5, 0xE13E, 0xF0B7,
    0x0840, 0x19C9, 0x2B52, 0x3ADB, 0x4E64, 0x5FED, 0x6D76, 0x7CFF,
    0x9489, 0x8500, 0xB79B, 0xA612, 0xD2AD, 0xC324, 0xF1BF, 0xE036,
    0x18C1, 0x0948, 0x3BD3, 0x2A5A, 0x5EE5, 0x4F6C, 0x7DF7, 0x6C7E,
    0xA50A, 0xB483, 0x8618, 0x9791, 0xE32E, 0xF2A7, 0xC03C, 0xD1B5,
    0x2942, 0x38CB, 0x0A50, 0x1BD9, 0x6F66, 0x7EEF, 0x4C74, 0x5DFD,
    0xB58B, 0xA402, 0x9699, 0x8710, 0xF3AF, 0xE226, 0xD0BD, 0xC134,
    0x39C3, 0x284A, 0x1AD1, 0x0B58, 0x7FE7, 0x6E6E, 0x5CF5, 0x4D7C,
    0xC60C, 0xD785, 0xE51E, 0xF497, 0x8028, 0x91A1, 0xA33A, 0xB2B3,
    0x4A44, 0x5BCD, 0x6956, 0x78DF, 0x0C60, 0x1DE9, 0x2F72, 0x3EFB,
    0xD68D, 0xC704, 0xF59F, 0xE416, 0x90A9, 0x8120, 0xB3BB, 0xA232,
    0x5AC5, 0x4B4C, 0x79D7, 0x685E, 0x1CE1, 0x0D68, 0x3FF3, 0x2E7A,
    0xE70E, 0xF687, 0xC41C, 0xD595, 0xA12A, 0xB0A3, 0x8238, 0x93B1,
    0x6B46, 0x7ACF, 0x4854, 0x59DD, 0x2D62, 0x3CEB, 0x0E70, 0x1FF9,
    0xF78F, 0xE606, 0xD49D, 0xC514, 0xB1AB, 0xA022, 0x92B9, 0x8330,
    0x7BC7, 0x6A4E, 0x58D5, 0x495C, 0x3DE3, 0x2C6A, 0x1EF1, 0x0F78,
]

def lt_crc(data: bytes) -> bytes:
    """Compute LocalTalk CRC-16. Returns 2 bytes (wire order)."""
    reg = 0xFFFF
    for b in data:
        reg = LT_CRC_LUT[(reg & 0xFF) ^ b] ^ (reg >> 8)
    return bytes([(reg ^ 0xFF) & 0xFF, ((reg >> 8) ^ 0xFF) & 0xFF])

def make_control_frame(dest: int, src: int, llap_type: int) -> bytes:
    """Build a 5-byte LLAP control frame with CRC."""
    payload = bytes([dest, src, llap_type])
    return payload + lt_crc(payload)

def make_data_frame(dest: int, src: int, payload: bytes) -> bytes:
    """Build an LLAP data frame (type=0x01 DDP short) with CRC."""
    header = bytes([dest, src, 0x01]) + payload
    return header + lt_crc(header)


# ── Pico Connection ───────────────────────────────────────────────

class PicoConnection:
    """Wraps a serial connection to one Pico's TT_DEBUG port."""

    def __init__(self, port: str, name: str = ""):
        self.port = port
        self.name = name or port
        self.ser = serial.Serial(port, timeout=0.1)
        self.ser.dtr = True
        self.rx_queue: queue.Queue[str] = queue.Queue()
        self.resp_queue: queue.Queue[str] = queue.Queue()
        self._stop = threading.Event()
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()
        # Wait for READY signal from firmware (indicates core 1 alive, PIO configured)
        self._wait_ready()

    def _wait_ready(self):
        """Confirm firmware is running by sending 'i' and checking for ID response."""
        self._drain_queues()
        deadline = time.time() + 5.0
        while time.time() < deadline:
            self.ser.write(b"i\r")
            try:
                line = self.resp_queue.get(timeout=0.5)
                if line.startswith("TASHTALK-PICO") or line.startswith("ID:"):
                    # Drain any remaining banner/ready lines
                    while not self.resp_queue.empty():
                        try: self.resp_queue.get_nowait()
                        except queue.Empty: break
                    return
            except queue.Empty:
                continue
        raise RuntimeError(f"Pico {self.name} ({self.port}): no response within 5s")

    @staticmethod
    def _is_rx_line(text: str) -> bool:
        """Check if a line is a monitor RX frame (with or without timestamp)."""
        # Format: "[NNNNNN] RX:AABB.." or just "RX:AABB.."
        if "] RX:" in text:
            return True
        if text.startswith("RX:") and len(text) > 3:
            c = text[3]
            # RX: followed by hex = monitor frame. Followed by space = status line.
            return c != ' '
        return False

    @staticmethod
    def _extract_rx_hex(text: str) -> str:
        """Extract the hex payload from a monitor RX line."""
        if "] RX:" in text:
            return text.split("] RX:", 1)[1].strip()
        if text.startswith("RX:"):
            return text[3:].strip()
        return text

    def _read_loop(self):
        buf = b""
        while not self._stop.is_set():
            try:
                data = self.ser.read(1024)
            except Exception:
                break
            if not data:
                continue
            buf += data
            while b"\r\n" in buf:
                line, buf = buf.split(b"\r\n", 1)
                text = line.decode("utf-8", errors="replace").strip()
                if not text:
                    continue
                if self._is_rx_line(text):
                    self.rx_queue.put(text)
                else:
                    self.resp_queue.put(text)

    def _drain_queues(self):
        while not self.rx_queue.empty():
            try: self.rx_queue.get_nowait()
            except queue.Empty: break
        while not self.resp_queue.empty():
            try: self.resp_queue.get_nowait()
            except queue.Empty: break

    def send_cmd(self, cmd: str, timeout: float = 3.0) -> list[str]:
        """Send a command and collect response lines until timeout."""
        self._drain_queues()
        self.ser.write((cmd + "\r").encode())
        lines = []
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                line = self.resp_queue.get(timeout=0.1)
                lines.append(line)
                # Check for known terminators
                if line in ("OK", "END", "---") or line.startswith("ERR:") \
                        or line.startswith("DIR ") or line.startswith("NODE ") \
                        or line.startswith("MONITOR ") or line.startswith("COUNTERS") \
                        or line.startswith("ID:") or line.startswith("BOOTSEL"):
                    break
            except queue.Empty:
                continue
        return lines

    def wait_rx(self, timeout: float = 2.0) -> list[str]:
        """Wait for RX frame hex strings (from monitor mode).
        Returns list of hex strings (no prefix, no timestamp)."""
        frames = []
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                line = self.rx_queue.get(timeout=0.1)
                frames.append(self._extract_rx_hex(line))
            except queue.Empty:
                if frames:
                    break  # got some, no more coming
        return frames

    def get_status(self) -> dict:
        """Parse the 's' command output into a dict."""
        lines = self.send_cmd("s")
        status = {}
        for line in lines:
            # Parse key=value pairs
            for m in re.finditer(r'(\w+)=(\d+)', line):
                status[m.group(1)] = int(m.group(2))
        return status

    def clear_counters(self):
        self.send_cmd("c")

    def set_node(self, node_id: int):
        self.send_cmd(f"n {node_id:02X}")

    def clear_node(self, node_id: int):
        self.send_cmd(f"N {node_id:02X}")

    def tx_raw(self, frame: bytes) -> bool:
        """TX a raw frame (with CRC). Returns True on OK."""
        hex_str = frame.hex().upper()
        lines = self.send_cmd(f"t {hex_str}", timeout=5.0)
        return any("OK" in l for l in lines)

    def tx_directed(self, dest: int, payload: bytes) -> bool:
        """Directed TX to dest with payload. Returns True on success."""
        hex_str = payload.hex().upper()
        lines = self.send_cmd(f"d {dest:02X} {hex_str}", timeout=5.0)
        return any("DIR OK" in l for l in lines)

    def enable_monitor(self):
        """Ensure monitor is ON (toggle if needed)."""
        lines = self.send_cmd("m")
        if any("OFF" in l for l in lines):
            # Was on, toggled off. Toggle again to re-enable.
            lines = self.send_cmd("m")
        return any("ON" in l for l in lines)

    def disable_monitor(self):
        """Ensure monitor is OFF (toggle if needed)."""
        lines = self.send_cmd("m")
        if any("ON" in l for l in lines):
            # Was off, toggled on. Toggle again to re-disable.
            lines = self.send_cmd("m")
        return any("OFF" in l for l in lines)

    def get_identity(self) -> str:
        lines = self.send_cmd("i")
        for l in lines:
            if l.startswith("ID:"):
                return l[3:]
        return ""

    def dump_rx(self) -> list[bytes]:
        """Dump buffered RX frames. Returns list of frame bytes."""
        lines = self.send_cmd("f")
        frames = []
        for l in lines:
            # Lines from 'f' command are "RX:AABB.." (no timestamp)
            if "RX:" in l:
                hex_str = l.split("RX:", 1)[1].strip()
                try:
                    frames.append(bytes.fromhex(hex_str))
                except ValueError:
                    pass
        return frames

    def close(self):
        self._stop.set()
        self.ser.close()


# ── Test Harness ──────────────────────────────────────────────────

@dataclass
class TestResult:
    name: str
    passed: bool
    detail: str = ""

class TestHarness:
    def __init__(self, pico_a: PicoConnection, pico_b: PicoConnection,
                 verbose: bool = False):
        self.a = pico_a
        self.b = pico_b
        self.verbose = verbose
        self.results: list[TestResult] = []

    def log(self, msg: str):
        print(f"  {msg}")

    def vlog(self, msg: str):
        if self.verbose:
            print(f"    {msg}")

    def check(self, name: str, condition: bool, detail: str = ""):
        r = TestResult(name, condition, detail)
        self.results.append(r)
        status = "PASS" if condition else "FAIL"
        print(f"  [{status}] {name}" + (f" — {detail}" if detail else ""))
        return condition

    def setup(self, node_a: int = 0x42, node_b: int = 0x43):
        """Common setup: set node IDs, enable monitor, drain, clear."""
        self.a.set_node(node_a)
        self.b.set_node(node_b)
        self.a.enable_monitor()
        self.b.enable_monitor()
        # Drain any RX frames from setup commands
        self.a.wait_rx(timeout=0.3)
        self.b.wait_rx(timeout=0.3)
        self.a._drain_queues()
        self.b._drain_queues()
        self.a.clear_counters()
        self.b.clear_counters()

    def report(self):
        total = len(self.results)
        passed = sum(1 for r in self.results if r.passed)
        failed = total - passed
        print(f"\n{'='*50}")
        print(f"Results: {passed}/{total} passed, {failed} failed")
        if failed:
            print("Failures:")
            for r in self.results:
                if not r.passed:
                    print(f"  - {r.name}: {r.detail}")
        print(f"{'='*50}")
        return failed == 0


# ── Layer 1: Basic Framing ────────────────────────────────────────

def test_layer1(h: TestHarness):
    print("\n=== Layer 1: Basic Framing ===")
    h.setup()

    # Test 1.1: Control frame (ENQ)
    frame = make_control_frame(0xFF, 0x42, 0x81)  # broadcast ENQ
    h.vlog(f"TX: {frame.hex()}")
    ok = h.a.tx_raw(frame)
    h.check("1.1 TX control frame accepted", ok)

    time.sleep(0.3)
    rx = h.b.wait_rx(timeout=1.0)
    h.vlog(f"B received: {rx}")
    if rx:
        rx_hex = rx[0]
        h.check("1.1 B received control frame", rx_hex.lower() == frame.hex(),
                f"expected {frame.hex()}, got {rx_hex}")
    else:
        h.check("1.1 B received control frame", False, "no frame received")

    # Test 1.2: Data frame (20 bytes)
    h.a.clear_counters()
    h.b.clear_counters()
    payload = bytes([0x00, 0x0A]) + bytes(range(8))  # 10 bytes payload
    frame = make_data_frame(0xFF, 0x42, payload)
    h.vlog(f"TX data: {frame.hex()} ({len(frame)} bytes)")
    ok = h.a.tx_raw(frame)
    h.check("1.2 TX data frame accepted", ok)

    time.sleep(0.3)
    rx = h.b.wait_rx(timeout=1.0)
    if rx:
        rx_hex = rx[0]
        # Monitor only shows first 32 bytes as hex
        expected_prefix = frame[:32].hex().upper()
        h.check("1.2 B received data frame", rx_hex.upper().startswith(expected_prefix[:len(rx_hex)]),
                f"got {rx_hex[:40]}...")
    else:
        h.check("1.2 B received data frame", False, "no frame received")

    # Check counters
    sb = h.b.get_status()
    h.check("1.2 B good CRC count", sb.get("good", 0) >= 2,
            f"good={sb.get('good', 0)}")
    h.check("1.2 B zero bad CRC", sb.get("bad", 0) == 0,
            f"bad={sb.get('bad', 0)}")


# ── Layer 2: Echo Handling ────────────────────────────────────────

def test_layer2(h: TestHarness):
    print("\n=== Layer 2: Echo Handling ===")
    h.setup()

    # A sends frame with src=0x42 (A's node). A should filter its own echo.
    frame = make_control_frame(0xFF, 0x42, 0x81)
    h.a.tx_raw(frame)
    time.sleep(0.3)

    # A should NOT see its own frame in monitor
    rx_a = h.a.wait_rx(timeout=0.5)
    h.check("2.1 A does not see own echo", len(rx_a) == 0,
            f"A got {len(rx_a)} frames: {rx_a}")

    sa = h.a.get_status()
    h.check("2.1 A echo_skip > 0", sa.get("echo", 0) > 0,
            f"echo={sa.get('echo', 0)}")

    # B sends, then A should receive B's frame (decoder not corrupted by echo)
    h.a.clear_counters()
    frame_b = make_control_frame(0xFF, 0x43, 0x81)
    h.b.tx_raw(frame_b)
    time.sleep(0.3)

    rx_a = h.a.wait_rx(timeout=1.0)
    h.check("2.2 A receives B's frame after echo", len(rx_a) > 0,
            f"got {len(rx_a)} frames")
    if rx_a:
        rx_hex = rx_a[0].replace("RX:", "").strip()
        h.check("2.2 A frame content correct",
                rx_hex.lower() == frame_b.hex(),
                f"expected {frame_b.hex()}, got {rx_hex}")


# ── Layer 3: Bidirectional ────────────────────────────────────────

def test_layer3(h: TestHarness):
    print("\n=== Layer 3: Bidirectional ===")
    h.setup()

    frame_a = make_control_frame(0xFF, 0x42, 0x81)
    frame_b = make_control_frame(0xFF, 0x43, 0x81)

    # A sends
    h.a.tx_raw(frame_a)
    time.sleep(0.3)
    rx_b = h.b.wait_rx(timeout=1.0)
    h.check("3.1 B receives A's frame", len(rx_b) > 0)

    # B sends
    h.b.tx_raw(frame_b)
    time.sleep(0.3)
    rx_a = h.a.wait_rx(timeout=1.0)
    h.check("3.2 A receives B's frame", len(rx_a) > 0)

    # Check zero errors on both
    sa = h.a.get_status()
    sb = h.b.get_status()
    h.check("3.3 A zero bad CRC", sa.get("bad", 0) == 0)
    h.check("3.4 B zero bad CRC", sb.get("bad", 0) == 0)


# ── Layer 5: CSMA/CA ─────────────────────────────────────────────

def test_layer5(h: TestHarness):
    print("\n=== Layer 5: CSMA/CA ===")
    h.setup()

    frame_a = make_data_frame(0xFF, 0x42, b"\x00\x05HELLO")
    frame_b = make_data_frame(0xFF, 0x43, b"\x00\x05WORLD")

    # Send both near-simultaneously
    h.a.ser.write(f"t {frame_a.hex()}\r".encode())
    h.b.ser.write(f"t {frame_b.hex()}\r".encode())
    time.sleep(2.0)

    # Both should eventually receive the other's frame
    rx_a = h.a.wait_rx(timeout=3.0)
    rx_b = h.b.wait_rx(timeout=3.0)
    h.check("5.1 A receives B's frame", len(rx_a) > 0,
            f"got {len(rx_a)} frames")
    h.check("5.2 B receives A's frame", len(rx_b) > 0,
            f"got {len(rx_b)} frames")

    sa = h.a.get_status()
    sb = h.b.get_status()
    h.check("5.3 zero bad CRC on A", sa.get("bad", 0) == 0)
    h.check("5.4 zero bad CRC on B", sb.get("bad", 0) == 0)


# ── Layer 6: RTS/CTS/DATA ────────────────────────────────────────

def test_layer6(h: TestHarness):
    print("\n=== Layer 6: RTS/CTS/DATA Dialog ===")
    h.setup()

    # A sends directed data to B (node 0x43)
    payload = bytes([0xDE, 0xAD, 0xBE, 0xEF])
    ok = h.a.tx_directed(0x43, payload)
    h.check("6.1 Directed TX accepted", ok)

    time.sleep(0.5)

    # Check B received the data
    rx_b = h.b.wait_rx(timeout=1.0)
    h.check("6.2 B received directed data", len(rx_b) > 0,
            f"got {len(rx_b)} frames")

    # Check counters
    sa = h.a.get_status()
    sb = h.b.get_status()
    h.check("6.3 A directed TX CTS OK", sa.get("cts_ok", 0) > 0,
            f"cts_ok={sa.get('cts_ok', 0)}")
    h.check("6.4 A zero timeouts", sa.get("tmo", 0) == 0,
            f"tmo={sa.get('tmo', 0)}")
    h.check("6.5 B auto-replied CTS", sb.get("ar", 0) > 0,
            f"ar={sb.get('ar', 0)}")
    h.check("6.6 B directed RX count", sb.get("dir_rx", 0) > 0,
            f"dir_rx={sb.get('dir_rx', 0)}")

    # Reverse direction: B sends to A
    h.a.clear_counters()
    h.b.clear_counters()
    ok = h.b.tx_directed(0x42, bytes([0xCA, 0xFE]))
    h.check("6.7 Reverse directed TX", ok)
    time.sleep(0.5)

    rx_a = h.a.wait_rx(timeout=1.0)
    h.check("6.8 A received reverse directed data", len(rx_a) > 0)


# ── Layer 7: Full Dialog ─────────────────────────────────────────

def test_layer7(h: TestHarness):
    print("\n=== Layer 7: Full Dialog (Request/Response) ===")
    h.setup()

    for i in range(5):
        # A → B
        ok_a = h.a.tx_directed(0x43, f"REQ{i}".encode())
        time.sleep(0.5)
        rx_b = h.b.wait_rx(timeout=2.0)

        # B → A
        ok_b = h.b.tx_directed(0x42, f"RSP{i}".encode())
        time.sleep(0.5)
        rx_a = h.a.wait_rx(timeout=2.0)

        if not (ok_a and ok_b and rx_b and rx_a):
            h.check(f"7.1 Round {i} complete", False,
                    f"ok_a={ok_a} ok_b={ok_b} rx_b={len(rx_b)} rx_a={len(rx_a)}")
            return

    sa = h.a.get_status()
    sb = h.b.get_status()
    h.check("7.1 All 5 rounds complete", True)
    h.check("7.2 A zero timeouts", sa.get("tmo", 0) == 0)
    h.check("7.3 B zero timeouts", sb.get("tmo", 0) == 0)
    h.check("7.4 A zero bad CRC", sa.get("bad", 0) == 0)
    h.check("7.5 B zero bad CRC", sb.get("bad", 0) == 0)


# ── Layer 8: Stress ───────────────────────────────────────────────

def test_layer8_mass(h: TestHarness):
    """Layer 8a: Mass bidirectional directed TX — hundreds of packets."""
    print("\n=== Layer 8a: Mass Bidirectional Directed TX ===")
    h.setup()

    N = 100
    a_ok = 0
    b_ok = 0
    a_tmo = 0
    b_tmo = 0

    for i in range(N):
        # A → B
        ok = h.a.tx_directed(0x43, bytes([i & 0xFF, 0xAA]))
        if ok:
            a_ok += 1

        # B → A
        ok = h.b.tx_directed(0x42, bytes([i & 0xFF, 0xBB]))
        if ok:
            b_ok += 1

    # Read final stats
    sa = h.a.get_status()
    sb = h.b.get_status()

    a_tmo = sa.get("tmo", 0)
    b_tmo = sb.get("tmo", 0)
    a_cts = sa.get("cts_ok", 0)
    b_cts = sb.get("cts_ok", 0)
    a_dir_rx = sa.get("dir_rx", 0)
    b_dir_rx = sb.get("dir_rx", 0)

    h.log(f"A: sent={a_ok}/{N} cts_ok={a_cts} tmo={a_tmo} dir_rx={a_dir_rx}")
    h.log(f"B: sent={b_ok}/{N} cts_ok={b_cts} tmo={b_tmo} dir_rx={b_dir_rx}")

    h.check("8a.1 A all directed TX accepted", a_ok == N, f"{a_ok}/{N}")
    h.check("8a.2 B all directed TX accepted", b_ok == N, f"{b_ok}/{N}")
    h.check("8a.3 A zero CTS timeouts", a_tmo == 0, f"tmo={a_tmo}")
    h.check("8a.4 B zero CTS timeouts", b_tmo == 0, f"tmo={b_tmo}")
    h.check("8a.5 A received all from B", a_dir_rx >= N, f"dir_rx={a_dir_rx}")
    h.check("8a.6 B received all from A", b_dir_rx >= N, f"dir_rx={b_dir_rx}")
    h.check("8a.7 A zero bad CRC", sa.get("bad", 0) == 0, f"bad={sa.get('bad', 0)}")
    h.check("8a.8 B zero bad CRC", sb.get("bad", 0) == 0, f"bad={sb.get('bad', 0)}")


def test_layer8(h: TestHarness):
    print("\n=== Layer 8: Stress ===")
    h.setup()

    # 8.1: Rapid broadcast burst
    for i in range(10):
        frame = make_data_frame(0xFF, 0x42, bytes([i, i, i, i]))
        h.a.ser.write(f"t {frame.hex()}\r".encode())
        time.sleep(0.05)

    time.sleep(2.0)
    sb = h.b.get_status()
    h.check("8.1 B received burst frames", sb.get("good", 0) >= 8,
            f"good={sb.get('good', 0)}")

    # 8.2: All-zeros payload
    h.a.clear_counters()
    h.b.clear_counters()
    frame = make_data_frame(0xFF, 0x42, b"\x00" * 20)
    h.a.tx_raw(frame)
    time.sleep(0.5)
    rx_b = h.b.wait_rx(timeout=1.0)
    h.check("8.2 All-zeros payload received", len(rx_b) > 0)


# ── Discovery ────────────────────────────────────────────────────

def discover_picos() -> list[tuple[str, str]]:
    """Find all tashtalk debug ports and their serial numbers."""
    ports = sorted(glob.glob("/dev/ttyACM*"))
    results = []
    for port in ports:
        try:
            s = serial.Serial(port, timeout=1)
            time.sleep(0.3)
            s.read(4096)  # drain
            s.write(b"i\r")
            time.sleep(0.3)
            data = s.read(4096).decode("utf-8", errors="replace")
            s.close()
            for line in data.split("\n"):
                line = line.strip()
                if line.startswith("ID:"):
                    results.append((port, line[3:].strip()))
                    break
        except Exception:
            pass
    return results


# ── Main ─────────────────────────────────────────────────────────

def main():
    p = argparse.ArgumentParser(description="Two-Pico LocalTalk test harness")
    p.add_argument("--layer", type=int, default=0,
                   help="Run only this layer (0=all)")
    p.add_argument("--verbose", "-v", action="store_true")
    p.add_argument("--port-a", type=str, default=None,
                   help="Debug serial port for Pico A")
    p.add_argument("--port-b", type=str, default=None,
                   help="Debug serial port for Pico B")
    args = p.parse_args()

    if args.port_a and args.port_b:
        port_a, port_b = args.port_a, args.port_b
    else:
        print("Discovering Picos...")
        picos = discover_picos()
        if len(picos) < 2:
            print(f"Need 2 Picos, found {len(picos)}: {picos}")
            print("Use --port-a and --port-b to specify manually")
            sys.exit(1)
        port_a, port_b = picos[0][0], picos[1][0]
        print(f"  Pico A: {port_a} (serial {picos[0][1]})")
        print(f"  Pico B: {port_b} (serial {picos[1][1]})")

    a = PicoConnection(port_a, "A")
    b = PicoConnection(port_b, "B")
    h = TestHarness(a, b, verbose=args.verbose)

    layers = {
        1: test_layer1,
        2: test_layer2,
        3: test_layer3,
        5: test_layer5,
        6: test_layer6,
        7: test_layer7,
        8: test_layer8,
        9: test_layer8_mass,
    }

    try:
        if args.layer:
            if args.layer in layers:
                layers[args.layer](h)
            else:
                print(f"Unknown layer {args.layer}. Available: {sorted(layers.keys())}")
                sys.exit(1)
        else:
            for n in sorted(layers.keys()):
                layers[n](h)
    finally:
        a.close()
        b.close()

    ok = h.report()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
