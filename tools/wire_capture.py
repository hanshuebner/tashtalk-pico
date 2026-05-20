#!/usr/bin/env python3
"""
Closed-loop wire capture for tashtalk-pico bring-up.

Drives Digilent WaveForms.app via GUI automation (AppleScript) against the
checked-in `la.dwf3work` workspace, which has the logic analyzer pre-configured
(DIO0, 12.5 MHz, 32k samples, falling-edge trigger, pre-trigger ~20%).

Why GUI automation and not the SDK: WaveForms.app's dwf framework refuses to
enumerate the AD2 from a generic Python process (returns "Adept NOK / ERC
3090"). Empirically this is not an Apple-entitlement issue — WaveForms.app
itself has no DriverKit entitlements — but some libdpti internal state that
only WaveForms.app's own process maintains. Driving WaveForms.app via Accessi-
bility APIs sidesteps the whole problem.

Workflow:
  1. Launch WaveForms with `la.dwf3work` (if it isn't already running with it).
  2. Click the LA's "Single" button → analyzer arms, falling-edge trigger.
  3. Run the stimulus callback (e.g. send 'p' to TT_DEBUG so the firmware
     fires a known LLAP frame via PIO).
  4. Wait briefly for the capture to complete.
  5. Click "Export" → "Save" → type the target path in the file panel →
     press Enter. WaveForms writes a CSV with the same format that interactive
     File→Export produces.
  6. Read the CSV, decode the FM-0 wire trace, print result.

Usage:
    python3 tools/wire_capture.py [--out PATH] [--stimulus {p|enq|none}]
                                  [--keep-open] [--workspace PATH]

Prerequisite: the `la.dwf3work` workspace at the repo root configures the LA
correctly. If you change its configuration in WaveForms.app, save and commit.
"""

from __future__ import annotations

import argparse
import glob
import os
import subprocess
import sys
import termios
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_WORKSPACE = REPO_ROOT / "la.dwf3work"
WAVEFORMS_BIN = "/Applications/WaveForms.app/Contents/MacOS/WaveForms"


# ────────────────────────────────────────────────────────────────────
# AppleScript helpers
# ────────────────────────────────────────────────────────────────────

def osa(script: str) -> str:
    """Run an AppleScript snippet, return stdout (stripped). Raises on error."""
    r = subprocess.run(["osascript", "-e", script],
                       capture_output=True, text=True)
    if r.returncode != 0:
        raise RuntimeError(f"AppleScript failed:\n{script}\nstderr: {r.stderr}")
    return r.stdout.strip()


def wf_is_running() -> bool:
    r = subprocess.run(["pgrep", "-i", "WaveForms"], capture_output=True)
    return r.returncode == 0


def wf_has_workspace(name_substring: str) -> bool:
    """Check whether WaveForms' main window title contains the given substring."""
    try:
        names = osa('tell application "System Events" to '
                    'tell process "WaveForms" to '
                    'get name of every window')
    except RuntimeError:
        return False
    return name_substring in names


def wf_launch(workspace: Path, wait_s: float = 6.0) -> None:
    """Launch WaveForms.app with the given workspace and wait until it's up."""
    print(f"[wire_capture] launching WaveForms with {workspace.name}", flush=True)
    subprocess.Popen(
        [WAVEFORMS_BIN, str(workspace)],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        start_new_session=True,
    )
    deadline = time.time() + wait_s
    while time.time() < deadline:
        if wf_has_workspace(workspace.stem):
            time.sleep(0.5)  # let the LA panel finish configuring
            return
        time.sleep(0.2)
    raise RuntimeError(f"WaveForms did not open within {wait_s}s")


def wf_click_single() -> None:
    osa('''
        tell application "System Events"
            tell process "WaveForms"
                set g to first group of window 1 whose description is ""
                click button "Single" of g
            end tell
        end tell
    ''')


def wf_click_export_save_to(path: str) -> None:
    """Click Export → Save → enter filename → Return."""
    # Click "Export" on the LA toolbar.
    osa('''
        tell application "System Events"
            tell process "WaveForms"
                set g to first group of window 1 whose description is ""
                click button "Export" of g
            end tell
        end tell
    ''')
    time.sleep(0.7)
    # Click "Save" in the Export dialog → opens file save panel
    osa('''
        tell application "System Events"
            tell process "WaveForms"
                set w to first window whose name is "Logic Analyzer - Export"
                click button "Save" of (first group of w)
            end tell
        end tell
    ''')
    time.sleep(1.0)
    # Type the path into "File name:" field and press Return.
    # AppleScript string literal: wrap in double quotes, escape internal " and \
    as_string = '"' + path.replace('\\', '\\\\').replace('"', '\\"') + '"'
    osa(f'''
        tell application "System Events"
            tell process "WaveForms"
                set w to first window whose name is "Logic Analyzer - Export"
                set tf to text field "File name:" of w
                set focused of tf to true
                delay 0.1
                keystroke "a" using {{command down}}
                delay 0.1
                keystroke {as_string}
                delay 0.2
                keystroke return
            end tell
        end tell
    ''')
    # Wait for the file to appear (and dialogs to close)
    deadline = time.time() + 5.0
    while time.time() < deadline:
        if os.path.exists(path) and os.path.getsize(path) > 0:
            return
        time.sleep(0.1)
    raise RuntimeError(f"CSV not written to {path} within 5s")


def wf_dismiss_dialogs() -> None:
    """Press Escape a few times to close any leftover dialogs."""
    try:
        osa('''
            tell application "System Events"
                tell process "WaveForms"
                    repeat 3 times
                        key code 53
                        delay 0.2
                    end repeat
                end tell
            end tell
        ''')
    except RuntimeError:
        pass


# ────────────────────────────────────────────────────────────────────
# Pico stimulus
# ────────────────────────────────────────────────────────────────────

def find_tt_port(suffix: str) -> str:
    """suffix '1' = TT_DATA, '3' = TT_DEBUG."""
    candidates = sorted(glob.glob(f"/dev/cu.usbmodem*{suffix}"))
    if not candidates:
        sys.exit(f"no Pico CDC port with suffix {suffix} found")
    return candidates[-1]


def open_raw(path: str) -> int:
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY)
    a = termios.tcgetattr(fd)
    a[0] = 0; a[1] = 0
    a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    a[3] = 0
    a[6][termios.VMIN] = 0; a[6][termios.VTIME] = 1
    termios.tcsetattr(fd, termios.TCSANOW, a)
    return fd


def stim_p_command():
    """Send 'p' on TT_DEBUG → fires a known LLAP frame via PIO immediately."""
    fd = open_raw(find_tt_port("3"))
    try:
        time.sleep(0.05)
        os.write(fd, b'p')
        time.sleep(0.2)
    finally:
        os.close(fd)


def stim_enq():
    """Send a TransmitFrame command (ENQ) on TT_DATA. Goes through the host
    parser and the CSMA defer path. LLAP control frame:
        dest=0x42 src=0x01 type=0x81 (ENQ to node 0x42)
        followed by two pre-computed CRC bytes.
    The Pico's CommandParser expects the host to provide the CRC — there is no
    "calc CRC for me" terminator. CRC of [0x42, 0x01, 0x81] = 0x5b 0xf9."""
    fd = open_raw(find_tt_port("1"))
    try:
        time.sleep(0.05)
        os.write(fd, b'\x01\x42\x01\x81\x5b\xf9')
        time.sleep(0.2)
    finally:
        os.close(fd)


STIMULI = {
    "p": ("debug 'p' command → known LLAP frame via PIO", stim_p_command),
    "enq": ("ENQ via TT_DATA TransmitFrame", stim_enq),
    "none": ("no stimulus (passive capture)", lambda: None),
}


# ────────────────────────────────────────────────────────────────────
# FM-0 decode
# ────────────────────────────────────────────────────────────────────

def read_csv(path: str) -> tuple[list[tuple[float, int]], float]:
    with open(path) as f:
        header_lines = []
        for line in f:
            if line.startswith("Time"):
                break
            header_lines.append(line)
        rows = [(float(t), int(v)) for t, v in
                (l.strip().split(",") for l in f if l.strip())]
    if len(rows) < 2:
        raise RuntimeError(f"only {len(rows)} samples in {path}")
    sample_period = rows[1][0] - rows[0][0]
    return rows, sample_period


def decode_fm0(rows, sample_period, bitrate_bps: float = 230_400):
    """Return (bits, trans_times) — FM-0 decoded LSB-first bits and transition
    times for diagnostics."""
    trans = [rows[i][0] for i in range(1, len(rows))
             if rows[i][1] != rows[i - 1][1]]
    if not trans:
        return [], []
    cell_s = 1.0 / bitrate_bps
    threshold = cell_s * 0.75
    bits = []
    i = 0
    while i < len(trans) - 1:
        gap = trans[i + 1] - trans[i]
        if gap > threshold:
            bits.append(1)
            i += 1
        else:
            bits.append(0)
            i += 2 if i + 2 < len(trans) else 1
    return bits, trans


FLAG_BITS = [0, 1, 1, 1, 1, 1, 1, 0]


def find_flags(bits):
    return [i for i in range(len(bits) - 7)
            if bits[i:i + 8] == FLAG_BITS]


def unstuff(bits):
    out = []
    ones = 0
    for b in bits:
        if ones == 5 and b == 0:
            ones = 0
            continue
        out.append(b)
        ones = ones + 1 if b == 1 else 0
    return out


def bits_to_bytes_lsb(bits):
    out = []
    for i in range(0, len(bits) - 7, 8):
        v = 0
        for j in range(8):
            if bits[i + j]:
                v |= 1 << j
        out.append(v)
    return out


def render_decode(rows, sample_period):
    bits, trans = decode_fm0(rows, sample_period)
    if not bits:
        return "  (wire idle — no transitions)"
    out = []
    out.append(f"  transitions: {len(trans)}  span: "
               f"{(trans[-1] - trans[0]) * 1e6:.1f}µs")
    out.append(f"  raw bits: {len(bits)}: {''.join(str(b) for b in bits)}")
    flags = find_flags(bits)
    out.append(f"  flag offsets (preamble bytes): {flags}")
    if flags:
        # Take bits after final preamble flag, until trailing flag if any
        payload_start = flags[0] + 8 * sum(1 for k in range(len(flags))
                                           if flags[k] == flags[0] + 8 * k)
        # Simpler: take payload between first and last flag if multiple,
        # otherwise after first.
        if len(flags) > 1 and flags[-1] - flags[0] >= 8:
            # Find last flag whose preceding window is a contiguous run of flags
            i = 0
            while i + 1 < len(flags) and flags[i + 1] - flags[i] == 8:
                i += 1
            preamble_end = flags[i] + 8
            tail_idx = next((o for o in flags if o > preamble_end + 8), None)
            payload = bits[preamble_end:tail_idx] if tail_idx else bits[preamble_end:]
        else:
            payload = bits[flags[0] + 8:]
        unstuffed = unstuff(payload)
        bytes_dec = bits_to_bytes_lsb(unstuffed)
        if bytes_dec:
            hex_dump = " ".join(f"{b:02x}" for b in bytes_dec)
            out.append(f"  payload bytes (unstuffed, LSB-first): {hex_dump}")
            if len(bytes_dec) >= 3:
                out.append(f"    → dest=0x{bytes_dec[0]:02x} "
                           f"src=0x{bytes_dec[1]:02x} "
                           f"type=0x{bytes_dec[2]:02x}")
    return "\n".join(out)


# ────────────────────────────────────────────────────────────────────
# Main
# ────────────────────────────────────────────────────────────────────

def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--out", default="/tmp/wire_capture.csv")
    p.add_argument("--workspace", default=str(DEFAULT_WORKSPACE))
    p.add_argument("--stimulus", choices=list(STIMULI.keys()), default="p")
    p.add_argument("--arm-delay", type=float, default=0.4,
                   help="seconds between Single click and stimulus firing")
    p.add_argument("--capture-delay", type=float, default=0.8,
                   help="seconds between stimulus firing and Export click")
    p.add_argument("--keep-open", action="store_true",
                   help="don't quit WaveForms at the end")
    args = p.parse_args()

    workspace = Path(args.workspace).resolve()
    if not workspace.exists():
        sys.exit(f"workspace not found: {workspace}")

    out_path = os.path.abspath(args.out)
    # Clean previous file so we can detect a fresh write.
    if os.path.exists(out_path):
        os.remove(out_path)

    if not wf_is_running() or not wf_has_workspace(workspace.stem):
        if wf_is_running():
            # A different / no workspace is loaded; replace it.
            subprocess.run(["pkill", "-9", "-f", "WaveForms"],
                           capture_output=True)
            time.sleep(1.0)
        wf_launch(workspace)
    else:
        print("[wire_capture] WaveForms already running with workspace",
              flush=True)
    wf_dismiss_dialogs()

    desc, fn = STIMULI[args.stimulus]
    print(f"[wire_capture] stimulus: {desc}", flush=True)

    # ── The closed-loop capture ──
    print("[wire_capture] arming (Single)…", flush=True)
    wf_click_single()
    time.sleep(args.arm_delay)
    if fn is not None:
        print("[wire_capture] firing stimulus", flush=True)
        fn()
    time.sleep(args.capture_delay)
    print(f"[wire_capture] exporting → {out_path}", flush=True)
    wf_click_export_save_to(out_path)
    print(f"[wire_capture] wrote {out_path}", flush=True)

    rows, period = read_csv(out_path)
    print(f"[wire_capture] {len(rows)} samples @ {1.0/period/1e6:.3f} MHz")
    print("[wire_capture] decode:")
    print(render_decode(rows, period))

    if not args.keep_open:
        # Force-kill: avoids the "save workspace?" dialog.
        subprocess.run(["pkill", "-9", "-f", "WaveForms"],
                       capture_output=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
