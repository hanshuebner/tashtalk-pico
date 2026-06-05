#!/usr/bin/env python3
"""
Core AD2 (Analog Discovery 2) module for LocalTalk bus analysis.

Wraps the Digilent WaveForms dwf SDK to provide reliable, repeatable
digital logic captures.  All VIO / threshold setup is baked in so it
cannot be forgotten.

Usage:
    from ad2 import Ad2Device, decode_localtalk, CHANNELS

    with Ad2Device() as dev:
        pins = dev.read_pins()
        print(pins)

        data, rate = dev.capture_triggered("DE", "rise", duration_s=0.005)
        frames = decode_localtalk(data, rate, channel="RO")
        for f in frames:
            print(f)
"""

from __future__ import annotations

import ctypes
import sys
import time
from dataclasses import dataclass
from typing import Optional

# ---------------------------------------------------------------------------
# dwf SDK constants (from dwf.h)
# ---------------------------------------------------------------------------

DwfStateReady = 0
DwfStateArmed = 1
DwfStateDone = 2

trigsrcNone = 0
trigsrcDetectorDigitalIn = 3

DwfDigitalInClockSourceInternal = 0
DwfDigitalInSampleModeSimple = 0

DwfTriggerSlopeRise = 0
DwfTriggerSlopeFall = 1
DwfTriggerSlopeEither = 2

EDGE_MAP = {
    "rise": DwfTriggerSlopeRise,
    "fall": DwfTriggerSlopeFall,
    "any": DwfTriggerSlopeEither,
}

# ---------------------------------------------------------------------------
# Channel mapping  (AD2 DIO pin -> signal name)
# ---------------------------------------------------------------------------

CHANNELS = {
    "A":  0,   # RS-485 differential A
    "B":  1,   # RS-485 differential B
    "DI": 2,   # TX data to transceiver
    "DE": 3,   # driver enable
    "RE": 4,   # receiver enable (active-low)
    "RO": 5,   # receiver output
    "DBG0": 6, # core 1 debug trigger output (GP6)
}

CHANNEL_BY_DIO = {v: k for k, v in CHANNELS.items()}
NUM_CHANNELS = 6

# ---------------------------------------------------------------------------
# LocalTalk constants
# ---------------------------------------------------------------------------

LOCALTALK_BITRATE = 230_400  # bps

LLAP_TYPES = {
    0x01: "DDP-short",
    0x02: "DDP-long",
    0x81: "ENQ",
    0x82: "ACK",
    0x84: "RTS",
    0x85: "CTS",
}

AD2_BASE_CLOCK = 100_000_000   # 100 MHz
# Note: actual max buffer is queried at runtime via FDwfDigitalInBufferSizeInfo.
# On the AD2 (non-Pro) with 8-bit sample format, the max is typically 4096.
MAX_BUFFER_FALLBACK = 4096

# ---------------------------------------------------------------------------
# Low-level helpers
# ---------------------------------------------------------------------------

def _load_dwf():
    try:
        return ctypes.cdll.LoadLibrary("libdwf.so")
    except OSError:
        sys.exit("Cannot load libdwf.so -- install the Digilent WaveForms SDK")


def _check(dwf, fn_name, *args):
    fn = getattr(dwf, fn_name)
    if not fn(*args):
        err = ctypes.create_string_buffer(512)
        dwf.FDwfGetLastErrorMsg(err)
        raise RuntimeError(f"{fn_name} failed: {err.value.decode()}")


def _resolve_channel(ch) -> int:
    """Accept channel as int (DIO number) or string name."""
    if isinstance(ch, int):
        return ch
    name = ch.upper()
    if name not in CHANNELS:
        raise ValueError(f"Unknown channel {ch!r}; valid: {list(CHANNELS)}")
    return CHANNELS[name]


# ---------------------------------------------------------------------------
# Ad2Device
# ---------------------------------------------------------------------------

class Ad2Device:
    """Context-managed wrapper around an Analog Discovery 2 device.

    Digital IO voltage is set to 3.3V via FDwfDeviceParamSet on open and
    re-applied after every digital-in reset, so data sampling always works
    correctly with RP2040 signals.

    Critical: on the AD2, the digital IO voltage/threshold is set via
    FDwfDeviceParamSet(hdwf, DwfParamDigitalVoltage, millivolts), NOT via
    FDwfAnalogIOChannelNodeSet (which controls the V+ power supply).
    Without this, digital input data sampling returns all zeros even though
    the trigger detector still works.
    """

    # DwfParam constants
    _DwfParamDigitalVoltage = 2

    def __init__(self, vio_mv: int = 3300):
        self._dwf = _load_dwf()
        self._hdwf: Optional[ctypes.c_int] = None
        self._vio_mv = vio_mv

    # -- context manager -----------------------------------------------------

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *exc):
        self.close()

    # -- open / close --------------------------------------------------------

    def open(self):
        hdwf = ctypes.c_int()
        _check(self._dwf, "FDwfDeviceOpen", ctypes.c_int(-1), ctypes.byref(hdwf))
        if hdwf.value == 0:
            raise RuntimeError("No Analog Discovery device found")
        self._hdwf = hdwf
        # Query actual max buffer size
        maxbuf = ctypes.c_int()
        self._dwf.FDwfDigitalInBufferSizeInfo(hdwf, ctypes.byref(maxbuf))
        self._max_buffer = maxbuf.value if maxbuf.value > 0 else MAX_BUFFER_FALLBACK
        self._apply_vio()

    def close(self):
        if self._hdwf is not None:
            self._dwf.FDwfDeviceClose(self._hdwf)
            self._hdwf = None

    # -- VIO -----------------------------------------------------------------

    def _apply_vio(self):
        """Set digital IO voltage using both available methods.

        1. FDwfAnalogIOChannelNodeSet on V+ supply (channel 0, node 0)
        2. FDwfDeviceParamSet with DwfParamDigitalVoltage

        Both are applied because the AD2 SDK behaviour varies by firmware
        revision and platform.  Without correct VIO, the trigger detector
        still works but the data sampler returns all zeros.

        Called after open and after every digital-in reset.
        """
        dwf, h = self._dwf, self._hdwf
        # Method 1: V+ supply voltage
        vio_v = self._vio_mv / 1000.0
        dwf.FDwfAnalogIOChannelNodeSet(h,
                                       ctypes.c_int(0), ctypes.c_int(0),
                                       ctypes.c_double(vio_v))
        dwf.FDwfAnalogIOEnableSet(h, ctypes.c_int(1))
        dwf.FDwfAnalogIOConfigure(h)
        # Method 2: device parameter
        dwf.FDwfDeviceParamSet(h,
                               ctypes.c_int(self._DwfParamDigitalVoltage),
                               ctypes.c_int(self._vio_mv))
        time.sleep(0.02)

    # -- pin read ------------------------------------------------------------

    def read_pins(self) -> dict[str, int]:
        """Read current state of all monitored DIO pins.  Returns a dict
        mapping channel name to 0/1."""
        dwf, h = self._dwf, self._hdwf
        _check(dwf, "FDwfDigitalIOStatus", h)
        val = ctypes.c_uint32()
        _check(dwf, "FDwfDigitalIOInputStatus", h, ctypes.byref(val))
        raw = val.value
        return {name: (raw >> dio) & 1 for name, dio in CHANNELS.items()}

    # -- capture helpers -----------------------------------------------------

    def _configure_digital_in(self, sample_rate_hz: int, buffer_size: int,
                              trigger_channel: Optional[int],
                              trigger_edge: int,
                              pre_trigger_pct: float) -> float:
        dwf, h = self._dwf, self._hdwf

        _check(dwf, "FDwfDigitalInReset", h)
        # VIO must be re-applied after reset
        self._apply_vio()

        _check(dwf, "FDwfDigitalInClockSourceSet", h,
               ctypes.c_int(DwfDigitalInClockSourceInternal))

        divider = max(1, AD2_BASE_CLOCK // sample_rate_hz)
        actual_rate = AD2_BASE_CLOCK / divider

        _check(dwf, "FDwfDigitalInDividerSet", h, ctypes.c_int(divider))
        _check(dwf, "FDwfDigitalInSampleModeSet", h,
               ctypes.c_int(DwfDigitalInSampleModeSimple))
        _check(dwf, "FDwfDigitalInSampleFormatSet", h, ctypes.c_int(8))
        _check(dwf, "FDwfDigitalInBufferSizeSet", h, ctypes.c_int(buffer_size))

        if trigger_channel is not None:
            _check(dwf, "FDwfDigitalInTriggerSourceSet", h,
                   ctypes.c_int(trigsrcDetectorDigitalIn))
            post_trigger = int(buffer_size * (1.0 - pre_trigger_pct))
            _check(dwf, "FDwfDigitalInTriggerPositionSet", h,
                   ctypes.c_int(post_trigger))

            rise = ctypes.c_uint(0)
            fall = ctypes.c_uint(0)
            mask = 1 << trigger_channel
            if trigger_edge == DwfTriggerSlopeRise:
                rise = ctypes.c_uint(mask)
            elif trigger_edge == DwfTriggerSlopeFall:
                fall = ctypes.c_uint(mask)
            else:
                rise = ctypes.c_uint(mask)
                fall = ctypes.c_uint(mask)

            _check(dwf, "FDwfDigitalInTriggerSet", h,
                   ctypes.c_uint(0), ctypes.c_uint(0), rise, fall)
        else:
            _check(dwf, "FDwfDigitalInTriggerSourceSet", h,
                   ctypes.c_int(trigsrcNone))

        # Re-apply VIO one more time for safety
        self._apply_vio()

        return actual_rate

    def _run_capture(self, buffer_size: int, timeout_s: float) -> bytes:
        dwf, h = self._dwf, self._hdwf

        _check(dwf, "FDwfDigitalInConfigure", h,
               ctypes.c_int(1),   # reconfigure
               ctypes.c_int(1))   # start

        state = ctypes.c_int()
        t0 = time.time()
        while True:
            _check(dwf, "FDwfDigitalInStatus", h,
                   ctypes.c_int(1), ctypes.byref(state))
            if state.value == DwfStateDone:
                break
            if time.time() - t0 > timeout_s:
                print(f"[ad2] capture timeout after {timeout_s}s "
                      f"(state={state.value})", file=sys.stderr)
                break
            time.sleep(0.001)

        data = (ctypes.c_uint8 * buffer_size)()
        _check(dwf, "FDwfDigitalInStatusData", h,
               data, ctypes.c_int(buffer_size))
        return bytes(data)

    # -- public capture API --------------------------------------------------

    def capture_triggered(self, channel="DE", edge="rise",
                          duration_s: float = 0.005,
                          pre_trigger_pct: float = 0.2,
                          sample_rate_mhz: float = 10.0,
                          timeout_s: float = 10.0) -> tuple[bytes, float]:
        """Capture with trigger.  Returns (raw_samples, actual_sample_rate_hz).

        Each byte in raw_samples has one bit per DIO channel (bit 0 = DIO0, etc).
        """
        ch = _resolve_channel(channel)
        edge_val = EDGE_MAP.get(edge, edge) if isinstance(edge, str) else edge
        sr = int(sample_rate_mhz * 1e6)
        buf = min(int(sr * duration_s), self._max_buffer)
        actual = self._configure_digital_in(sr, buf, ch, edge_val, pre_trigger_pct)
        data = self._run_capture(buf, timeout_s)
        return data, actual

    def capture_free(self, duration_s: float = 0.005,
                     sample_rate_mhz: float = 10.0,
                     timeout_s: float = 5.0) -> tuple[bytes, float]:
        """Free-run capture (no trigger).  Returns (raw_samples, actual_rate)."""
        sr = int(sample_rate_mhz * 1e6)
        buf = min(int(sr * duration_s), self._max_buffer)
        actual = self._configure_digital_in(sr, buf, None, 0, 0)
        data = self._run_capture(buf, timeout_s)
        return data, actual


# ---------------------------------------------------------------------------
# Signal extraction helpers
# ---------------------------------------------------------------------------

def extract_channel(data: bytes, ch) -> list[int]:
    """Extract a single channel as a list of 0/1 values."""
    dio = _resolve_channel(ch)
    mask = 1 << dio
    return [(b & mask) >> dio for b in data]


def find_edges(signal: list[int], edge: str = "any") -> list[int]:
    """Return sample indices where the given edge type occurs."""
    edges = []
    for i in range(1, len(signal)):
        prev, cur = signal[i - 1], signal[i]
        if prev == cur:
            continue
        if edge == "rising" and prev == 0 and cur == 1:
            edges.append(i)
        elif edge == "falling" and prev == 1 and cur == 0:
            edges.append(i)
        elif edge == "any":
            edges.append(i)
    return edges


def find_active_regions(signal: list[int]) -> list[tuple[int, int]]:
    """Find contiguous high regions.  Returns (start, end) pairs."""
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


# ---------------------------------------------------------------------------
# FM-0 / HDLC / LLAP decode
# ---------------------------------------------------------------------------

def _decode_fm0_bits(signal: list[int], sample_rate: float,
                     bitrate: float = LOCALTALK_BITRATE) -> list[int]:
    """Decode FM-0 signal to a raw bit stream."""
    transitions = []
    for i in range(1, len(signal)):
        if signal[i] != signal[i - 1]:
            transitions.append(i)
    if len(transitions) < 16:
        return []

    cell_samples = sample_rate / bitrate
    threshold = cell_samples * 0.75

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
    return bits


def _extract_hdlc_payload(bits: list[int]) -> list[int]:
    """Find HDLC flags, extract and bit-unstuff the payload between them."""
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


@dataclass
class LlapFrame:
    """A decoded LLAP frame."""
    dst: int
    src: int
    frame_type: int
    payload: list[int]   # bytes after the 3-byte header, before CRC
    crc: list[int]       # last 2 bytes (may be empty if frame too short)
    raw: list[int]       # all decoded bytes

    @property
    def type_name(self) -> str:
        return LLAP_TYPES.get(self.frame_type, f"0x{self.frame_type:02x}")

    def __str__(self):
        parts = [f"dst=0x{self.dst:02x} src=0x{self.src:02x} "
                 f"type={self.type_name}"]
        if self.payload:
            hex_dump = " ".join(f"{b:02x}" for b in self.payload[:32])
            if len(self.payload) > 32:
                hex_dump += "..."
            parts.append(f"payload({len(self.payload)}B): {hex_dump}")
        if self.crc:
            parts.append(f"crc={self.crc[0]:02x}{self.crc[1]:02x}")
        return "  ".join(parts)


def decode_localtalk(data: bytes, sample_rate: float,
                     channel="RO") -> list[LlapFrame]:
    """Decode LocalTalk frames from a capture on the given channel.

    Splits the signal into frame groups (separated by idle gaps), decodes
    FM-0, extracts HDLC payload, and returns a list of LlapFrame objects.
    """
    signal = extract_channel(data, channel)
    de = extract_channel(data, "DE")

    # For RO, only consider samples where DE is low (receiver enabled)
    if _resolve_channel(channel) == CHANNELS["RO"]:
        # Mask out samples during TX
        signal = [signal[i] if de[i] == 0 else signal[i] for i in range(len(signal))]

    # Find edges and group into frames
    edges = find_edges(signal, "any")
    if not edges:
        return []

    # Gap > 4 bit times means separate frame
    gap_threshold = int(sample_rate / LOCALTALK_BITRATE * 4)
    groups: list[tuple[int, int]] = []
    g_start = edges[0]
    prev = edges[0]
    for e in edges[1:]:
        if e - prev > gap_threshold:
            groups.append((g_start, prev))
            g_start = e
        prev = e
    groups.append((g_start, prev))

    frames: list[LlapFrame] = []
    for gs, ge in groups:
        # Extend window slightly for decode margin
        margin = int(sample_rate / LOCALTALK_BITRATE)
        ws = max(0, gs - margin)
        we = min(len(signal), ge + margin * 2)
        frame_bytes = _extract_hdlc_payload(
            _decode_fm0_bits(signal[ws:we], sample_rate))
        if len(frame_bytes) >= 3:
            crc = frame_bytes[-2:] if len(frame_bytes) >= 5 else []
            payload = frame_bytes[3:-2] if len(frame_bytes) >= 5 else frame_bytes[3:]
            frames.append(LlapFrame(
                dst=frame_bytes[0],
                src=frame_bytes[1],
                frame_type=frame_bytes[2],
                payload=payload,
                crc=crc,
                raw=frame_bytes,
            ))
    return frames


# ---------------------------------------------------------------------------
# Analysis / display helpers
# ---------------------------------------------------------------------------

def summarize_capture(data: bytes, sample_rate: float) -> str:
    """Return a multi-line human-readable summary of a capture."""
    us_per = 1e6 / sample_rate
    total_ms = len(data) * us_per / 1000
    lines = [f"Capture: {len(data)} samples @ {sample_rate/1e6:.1f} MHz "
             f"({total_ms:.1f} ms)"]

    de = extract_channel(data, "DE")
    di = extract_channel(data, "DI")
    ro = extract_channel(data, "RO")

    # TX regions
    tx_regions = find_active_regions(de)
    if tx_regions:
        lines.append(f"TX regions (DE high): {len(tx_regions)}")
        for i, (s, e) in enumerate(tx_regions):
            dur = (e - s) * us_per
            lines.append(f"  TX#{i}: sample {s}-{e}, {dur:.1f} us")
            tx_frames = decode_localtalk(data, sample_rate, "DI")
            for f in tx_frames:
                lines.append(f"    {f}")
    else:
        lines.append("No TX (DE never high)")

    # RX frames
    rx_frames = decode_localtalk(data, sample_rate, "RO")
    if rx_frames:
        lines.append(f"RX frames on RO: {len(rx_frames)}")
        for i, f in enumerate(rx_frames):
            lines.append(f"  RX#{i}: {f}")
    else:
        edges = find_edges(ro, "any")
        rx_edges_no_de = [e for e in edges if de[e] == 0]
        lines.append(f"No decodable RX frames (RO edges while idle: "
                     f"{len(rx_edges_no_de)})")

    return "\n".join(lines)


def save_csv(data: bytes, sample_rate: float, path: str):
    """Save capture as CSV."""
    with open(path, "w") as f:
        f.write(f"# Sample rate: {sample_rate:.0f} Hz\n")
        f.write("# Channels: " +
                ", ".join(f"DIO{dio}={name}"
                          for name, dio in sorted(CHANNELS.items(),
                                                  key=lambda x: x[1])) +
                "\n")
        f.write("time_us,A,B,DI,DE,RE,RO\n")
        us_per = 1e6 / sample_rate
        for i, byte in enumerate(data):
            t = i * us_per
            vals = [(byte >> ch) & 1 for ch in range(NUM_CHANNELS)]
            f.write(f"{t:.3f},{','.join(str(v) for v in vals)}\n")


# ---------------------------------------------------------------------------
# Standalone quick-test
# ---------------------------------------------------------------------------

def _main():
    """Quick self-test: read pins, do a free capture, report."""
    import argparse
    p = argparse.ArgumentParser(description="AD2 quick test")
    p.add_argument("--trigger", type=str, default=None,
                   help="Trigger channel (e.g. DE, RO). Omit for free-run.")
    p.add_argument("--edge", type=str, default="fall",
                   choices=["rise", "fall", "any"])
    p.add_argument("--duration", type=float, default=0.003,
                   help="Capture duration in seconds")
    p.add_argument("--timeout", type=float, default=15.0)
    p.add_argument("--csv", type=str, default=None)
    args = p.parse_args()

    with Ad2Device() as dev:
        pins = dev.read_pins()
        print("Current pin state:")
        for name in ["A", "B", "DI", "DE", "RE", "RO"]:
            print(f"  {name:3s} = {pins[name]}")

        if args.trigger:
            print(f"\nCapturing (trigger={args.trigger} {args.edge}, "
                  f"{args.duration*1000:.1f}ms)...")
            data, rate = dev.capture_triggered(
                args.trigger, args.edge,
                duration_s=args.duration, timeout_s=args.timeout)
        else:
            print(f"\nFree-run capture ({args.duration*1000:.1f}ms)...")
            data, rate = dev.capture_free(duration_s=args.duration)

        print()
        print(summarize_capture(data, rate))

        if args.csv:
            save_csv(data, rate, args.csv)
            print(f"\nSaved CSV: {args.csv}")


if __name__ == "__main__":
    _main()
