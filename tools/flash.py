#!/usr/bin/env python3
"""Flash a UF2 file to a TashTalk-Pico.

Sends the 'b' command on the debug port to enter BOOTSEL, waits for
the RPI-RP2 mass-storage volume to appear, copies the UF2, and waits
for the Pico to reboot into the new firmware.

Falls back to picotool if the device is already in BOOTSEL mode.
Works on both Linux and macOS.

Usage: python3 tools/flash.py <uf2-file>
"""

import glob
import os
import platform
import shutil
import subprocess
import sys
import time

IS_LINUX = platform.system() == "Linux"
IS_MACOS = platform.system() == "Darwin"


def find_bootsel_mount():
    """Find RPI-RP2 mass-storage mount point."""
    if IS_LINUX:
        mounts = glob.glob("/media/*/RPI-RP2*")
    elif IS_MACOS:
        mounts = glob.glob("/Volumes/RPI-RP2*")
    else:
        mounts = []
    return mounts[0] if mounts else None


def find_debug_port():
    """Find the TashTalk-Pico debug serial port."""
    # Linux: udev symlink
    if os.path.exists("/dev/tashtalk-debug"):
        return "/dev/tashtalk-debug"

    if IS_LINUX:
        # Fallback: scan ttyACM devices by sysfs
        for acm in sorted(glob.glob("/dev/ttyACM*")):
            base = os.path.basename(acm)
            try:
                product = open(
                    f"/sys/class/tty/{base}/device/../product"
                ).read().strip()
            except FileNotFoundError:
                continue
            if product != "TashTalk-Pico":
                continue
            sysdev = os.path.realpath(f"/sys/class/tty/{base}/device")
            for part in sysdev.split("/"):
                if ":1.2" in part:  # interface 2 = debug
                    return acm

    elif IS_MACOS:
        # macOS: cu.usbmodem devices, debug is typically the higher-numbered one
        candidates = sorted(glob.glob("/dev/cu.usbmodem*"))
        pico_ports = [p for p in candidates if "TashTalk" in p or "usbmodem" in p]
        if len(pico_ports) >= 2:
            return pico_ports[-1]  # last = debug (higher interface)
        # Fallback: any usbmodem with suffix 3 (interface 2 → suffix 3)
        for p in candidates:
            if p.endswith("3"):
                return p

    return None


def send_bootsel(port):
    """Send 'b' on the debug port to enter BOOTSEL mode."""
    import termios

    fd = os.open(port, os.O_RDWR | os.O_NOCTTY)
    a = termios.tcgetattr(fd)
    a[0] = 0
    a[1] = 0
    a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    a[3] = 0
    a[6][termios.VMIN] = 0
    a[6][termios.VTIME] = 1
    termios.tcsetattr(fd, termios.TCSANOW, a)
    time.sleep(1.0)
    os.read(fd, 4096)  # drain banner
    os.write(fd, b"b")
    time.sleep(0.5)
    os.close(fd)


def wait_for_mount(timeout=15):
    """Wait for RPI-RP2 mass-storage to appear."""
    for _ in range(int(timeout / 0.5)):
        time.sleep(0.5)
        mount = find_bootsel_mount()
        if mount:
            return mount
    return None


def try_picotool(uf2):
    """Try flashing via picotool (device already in BOOTSEL)."""
    picotool = shutil.which("picotool")
    if not picotool:
        return False
    r = subprocess.run(
        [picotool, "load", "-u", "-v", "-x", "-t", "uf2", uf2],
        capture_output=True,
        text=True,
    )
    return r.returncode == 0


def main():
    if len(sys.argv) != 2:
        sys.exit(f"Usage: {sys.argv[0]} <uf2-file>")

    uf2 = sys.argv[1]
    if not os.path.exists(uf2):
        sys.exit(f"UF2 file not found: {uf2}")

    # If already in BOOTSEL, just flash
    mount = find_bootsel_mount()
    if mount:
        print(f"Device already in BOOTSEL at {mount}")
        shutil.copy2(uf2, mount + "/")
        print("Flashed.")
        return

    if try_picotool(uf2):
        print("Flashed via picotool.")
        return

    # Send 'b' command to enter BOOTSEL
    port = find_debug_port()
    if not port:
        sys.exit("No TashTalk-Pico debug port found. Hold BOOTSEL and plug in.")

    print(f"Sending BOOTSEL command via {port}...")
    send_bootsel(port)

    mount = wait_for_mount()
    if not mount:
        sys.exit("Timed out waiting for BOOTSEL mount. Try holding BOOTSEL button.")

    print(f"Copying {uf2} to {mount}...")
    shutil.copy2(uf2, mount + "/")
    print("Flashed. Waiting for reboot...")
    time.sleep(3)
    print("Done.")


if __name__ == "__main__":
    main()
