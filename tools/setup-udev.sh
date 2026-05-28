#!/bin/sh
# Install udev rules for TashTalk-Pico on Linux.
# Creates stable /dev/tashtalk-{data,debug} symlinks and grants
# non-root access to RP2040 devices in BOOTSEL mode.
#
# Usage: sudo tools/setup-udev.sh

set -e

RULES=/etc/udev/rules.d/99-tashtalk-pico.rules

cat > "$RULES" << 'EOF'
# RP2040 in BOOTSEL mode — allow non-root access for picotool
SUBSYSTEM=="usb", ATTRS{idVendor}=="2e8a", MODE="0666"

# TashTalk-Pico CDC interfaces → stable /dev symlinks
# Generic symlinks (last device wins — for single-Pico setups):
#   /dev/tashtalk-data   → TT_DATA
#   /dev/tashtalk-debug  → TT_DEBUG
# Per-board symlinks (unique flash serial):
#   /dev/tashtalk-data-<SERIAL>
#   /dev/tashtalk-debug-<SERIAL>
KERNEL=="ttyACM*", ENV{ID_VENDOR_ID}=="16c0", ENV{ID_MODEL_ID}=="27dd", ENV{ID_USB_INTERFACE_NUM}=="00", SYMLINK+="tashtalk-data tashtalk-data-$env{ID_SERIAL_SHORT}", MODE="0666"
KERNEL=="ttyACM*", ENV{ID_VENDOR_ID}=="16c0", ENV{ID_MODEL_ID}=="27dd", ENV{ID_USB_INTERFACE_NUM}=="02", SYMLINK+="tashtalk-debug tashtalk-debug-$env{ID_SERIAL_SHORT}", MODE="0666"
EOF

udevadm control --reload-rules
udevadm trigger

echo "Installed $RULES"
echo "Replug your Pico to create /dev/tashtalk-{data,debug} symlinks."
