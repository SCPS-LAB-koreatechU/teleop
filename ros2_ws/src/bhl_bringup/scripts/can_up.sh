#!/usr/bin/env bash
# Bring CAN interfaces up at 1 Mbit/s by hand. robot.launch.py does not do this;
# system/80-can.network does it automatically once installed.
#
#   scripts/can_up.sh              every can* interface present
#   scripts/can_up.sh can0 can1    just these
set -e
if [ $# -eq 0 ]; then
  set -- $(ls /sys/class/net 2>/dev/null | grep -E '^can[0-9]+$' || true)
fi
if [ $# -eq 0 ]; then
  echo "no CAN interface. USB-CAN adapter plugged in?  lsmod | grep gs_usb ; dmesg | tail" >&2
  exit 1
fi
for ch in "$@"; do
  sudo ip link set "$ch" down 2>/dev/null || true
  sudo ip link set "$ch" up type can bitrate 1000000 restart-ms 100
  ip -details link show "$ch" | sed -n '1,3p'
done
