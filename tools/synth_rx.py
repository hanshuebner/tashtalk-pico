#!/usr/bin/env python3
"""Bring-up step 5 helper: decode synthetic frames coming from TT_DATA.

Opens TT_DATA, reads escape-encoded bytes (0x00 0xFF -> 0x00; 0x00 0xFD
-> frame end), reassembles each frame, recomputes the LocalTalk CRC, and
prints a summary line per frame.

Usage:
    python3 tools/synth_rx.py /dev/cu.usbmodem0000011

Exit with Ctrl-C. The firmware emits one frame every 2 seconds while the
host is connected.
"""
import os
import sys
import termios


# CRC-16 table identical to the one in tashtalk-proto and TailTalk.
# Generated once at import time so this script has no external deps.
def _build_crc_table():
    table = []
    for i in range(256):
        crc = i
        for _ in range(8):
            crc = (crc >> 1) ^ (0x8408 if crc & 1 else 0)
        table.append(crc)
    return table


CRC_TABLE = _build_crc_table()


def crc_update(reg, byte):
    return CRC_TABLE[(reg ^ byte) & 0xFF] ^ (reg >> 8)


def crc_frame(payload):
    """Run the streaming CRC over `payload` and return the final reg."""
    reg = 0xFFFF
    for b in payload:
        reg = crc_update(reg, b)
    return reg


def open_raw(path):
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY)
    a = termios.tcgetattr(fd)
    a[0] = 0
    a[1] = 0
    a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    a[3] = 0
    a[6][termios.VMIN] = 1
    a[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, a)
    return fd


def main():
    if len(sys.argv) != 2:
        print("usage: synth_rx.py /dev/cu.usbmodem*", file=sys.stderr)
        sys.exit(1)

    fd = open_raw(sys.argv[1])
    frame = bytearray()
    in_escape = False
    n_frames = 0
    n_good = 0
    n_bad = 0

    print(f"listening on {sys.argv[1]} — Ctrl-C to quit", flush=True)
    try:
        while True:
            chunk = os.read(fd, 256)
            for b in chunk:
                if in_escape:
                    in_escape = False
                    if b == 0xFF:
                        # Literal 0x00 in payload.
                        frame.append(0x00)
                    elif b in (0xFD, 0xFC, 0xFE, 0xFA):
                        # End-of-frame marker.
                        n_frames += 1
                        kind = {
                            0xFD: "good",
                            0xFC: "bad-crc",
                            0xFE: "framing-err",
                            0xFA: "aborted",
                        }[b]
                        # Verify CRC for "good" frames (the only kind we
                        # currently emit synthetically).
                        residue = crc_frame(frame)
                        crc_ok = residue == 0xF0B8
                        if b == 0xFD and crc_ok:
                            n_good += 1
                        else:
                            n_bad += 1
                        print(
                            f"#{n_frames:04d} status={kind:11s} "
                            f"len={len(frame):3d} "
                            f"crc_residue={residue:#06x} ok={crc_ok} "
                            f"raw={frame.hex()}",
                            flush=True,
                        )
                        frame = bytearray()
                    else:
                        print(
                            f"!! unknown escape 0x00 0x{b:02x} after "
                            f"{len(frame)} payload bytes — discarding",
                            flush=True,
                        )
                        frame = bytearray()
                elif b == 0x00:
                    in_escape = True
                else:
                    frame.append(b)
    except KeyboardInterrupt:
        pass
    finally:
        os.close(fd)
        print(
            f"\nsummary: {n_frames} frames, {n_good} good, {n_bad} failed",
            flush=True,
        )


if __name__ == "__main__":
    main()
