#!/opt/homebrew/opt/python@3.11/bin/python3.11
"""Capture TT_DEBUG output for a fixed window and print it.

Usage: cap_debug.py [port] [seconds]
"""
import sys
import time
import serial

port = sys.argv[1] if len(sys.argv) > 1 else "/dev/cu.usbmodem1203"
secs = float(sys.argv[2]) if len(sys.argv) > 2 else 15.0

ser = serial.Serial(port, 115200, timeout=0.2)
# Assert DTR so the device treats us as a connected host.
ser.dtr = True
time.sleep(0.1)

deadline = time.time() + secs
buf = b""
print(f"# capturing {port} for {secs:.0f}s ...", flush=True)
while time.time() < deadline:
    data = ser.read(4096)
    if data:
        buf += data
        # Print complete lines as they arrive.
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            sys.stdout.write(line.decode("ascii", "replace").rstrip("\r") + "\n")
            sys.stdout.flush()
if buf:
    sys.stdout.write(buf.decode("ascii", "replace"))
    sys.stdout.flush()
print("\n# done", flush=True)
ser.close()
