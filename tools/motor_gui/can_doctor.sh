#!/usr/bin/env bash
# Why is nothing answering on the CAN bus?
#
# A CAN frame is acknowledged by EVERY node that receives it correctly, whatever
# its arbitration id -- so a CAN_ERR_ACK (0x20) means no node at all is
# listening at this bitrate, and scanning device ids would tell us nothing.
# Bitrate is the only setting left that can cause that, so walk the usual ones.
# Needs sudo because each try reconfigures the interface.
set -uo pipefail
IF="${IF:-can0}"
RATES="${RATES:-1000000 800000 500000 250000 125000}"

command -v python3 >/dev/null || { echo "[FAIL] python3 missing" >&2; exit 1; }
python3 -c "import can" 2>/dev/null || { echo "[FAIL] python-can missing" >&2; exit 1; }
ip link show "$IF" >/dev/null 2>&1 || {
  echo "[FAIL] $IF does not exist - the USB adapter is not plugged in or not enumerated"
  lsusb | grep -i 1d50:606f || echo "        (no gs_usb adapter in lsusb either)"
  exit 1; }

probe() {   # one frame out; report whether anyone acknowledged it
  python3 - "$IF" <<'PY'
import sys, can, time
bus = can.interface.Bus(channel=sys.argv[1], interface="socketcan")
acked, other = 0, 0
for _ in range(5):
    try:
        bus.send(can.Message(arbitration_id=0x001, data=[0], is_extended_id=False), timeout=0.3)
    except Exception:
        pass
    time.sleep(0.05)
end = time.time() + 1.0
while time.time() < end:                 # 0x20 = CAN_ERR_ACK
    m = bus.recv(timeout=0.2)
    if m is None:
        continue
    if m.is_error_frame:
        acked += 1 if (m.arbitration_id & 0x20) else 0
        other += 0 if (m.arbitration_id & 0x20) else 1
    else:
        other += 1
bus.shutdown()
print("NOACK" if acked else ("TRAFFIC" if other else "SILENT"))
PY
}

echo "=== adapter ==="
lsusb | grep -i 1d50:606f || echo "  (not an OpenMoko/gs_usb adapter)"
ip -d link show "$IF" | grep -o "parentdev.*" || true

for rate in $RATES; do
  sudo ip link set "$IF" down 2>/dev/null
  if ! sudo ip link set "$IF" up type can bitrate "$rate" 2>/dev/null; then
    printf "  %-8s  interface refused this bitrate\n" "$rate"; continue
  fi
  case "$(probe)" in
    NOACK)   printf "  %-8s  no ACK - nothing is listening here\n" "$rate" ;;
    SILENT)  printf "  %-8s  no ACK error either - suspect the adapter or the wiring\n" "$rate" ;;
    TRAFFIC) printf "  %-8s  *** SOMETHING ANSWERED *** set bus bitrate to this\n" "$rate"; FOUND="$rate" ;;
  esac
done

sudo ip link set "$IF" down 2>/dev/null
sudo ip link set "$IF" up type can bitrate 1000000 2>/dev/null
echo "=== $IF restored to 1000000 ==="
ip -d link show "$IF" | grep "can state"

if [[ -n "${FOUND:-}" ]]; then
  echo
  echo "The motor speaks ${FOUND}, not 1000000. Bring the bus up at that rate, or"
  echo "put the motor back to 1 Mbit; hardware_joints.yaml assumes 1 Mbit everywhere."
  exit 0
fi
echo
echo "No bitrate got an acknowledgement, so no CAN node is reachable at all."
echo "That is electrical, not configuration. In order of likelihood:"
echo "  1. the motor's logic/transceiver rail is not powered (check its LED)"
echo "  2. CAN-H and CAN-L swapped at the connector"
echo "  3. termination: CAN-H to CAN-L should read ~60 ohm with the power OFF"
echo "  4. a broken conductor in the CAN cable"
exit 1
