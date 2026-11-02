#!/usr/bin/env python3
# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.
"""Who is actually on the CAN bus? Read-only, no sudo, no torque.

Answers the three questions that separate "the GUI is wrong" from "the motor is
not there", in the order that narrows fastest:

  1. Does the interface exist and at what bitrate?
  2. Is anything acknowledging our frames? Judged by the controller's own error
     counters, never by whether the transmit queue drained: when nothing
     acknowledges, the counters climb until the controller goes BUS-OFF, and
     going BUS-OFF discards the pending frames, so the queue empties and a dead
     bus looks exactly like a healthy one.
  3. Does any device id answer the recoil protocol?

The interesting case is 2 yes / 3 no: the board is powered and its CAN hardware
is alive, but the firmware is not answering -- a hung or wrong firmware, not a
wiring problem.

    ./bus_probe.py                 # can0, ids 0..127
    ./bus_probe.py -c can1 --ids 1 3 5
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "source/berkeley_humanoid_lite_lowlevel"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from can_util import acknowledged, bring_up_command, usable  # noqa: E402


def interface_state(channel: str) -> dict:
    out = subprocess.run(["ip", "-details", "-statistics", "link", "show", channel],
                         capture_output=True, text=True)
    if out.returncode != 0:
        return {}
    text = out.stdout
    info = {"raw": text}
    for key in ("bitrate", "can state"):
        if key in text:
            after = text.split(key, 1)[1].split()
            info[key] = after[0] if after else "?"
    lines = text.splitlines()
    for i, line in enumerate(lines):
        if line.strip().startswith("TX:"):
            info["tx_packets"] = int(lines[i + 1].split()[1])
        if line.strip().startswith("RX:"):
            info["rx_packets"] = int(lines[i + 1].split()[1])
    return info


def scan(channel: str, ids: list[int], timeout: float) -> tuple[list[int], int]:
    import berkeley_humanoid_lite_lowlevel.recoil as recoil
    bus = recoil.Bus(channel=channel, bitrate=1000000)
    answered = []
    try:
        for dev in ids:
            if bus.ping(dev, timeout=timeout):
                answered.append(dev)
    finally:
        bus.stop()
    return answered, len(ids)


def listen(channel: str, seconds: float) -> list[str]:
    """Frames seen on the wire, minus our own echoes (function 4 = our ping)."""
    import can
    bus = can.interface.Bus(channel=channel, interface="socketcan", receive_own_messages=False)
    seen = []
    end = time.time() + seconds
    try:
        while time.time() < end:
            m = bus.recv(timeout=0.2)
            if m is None or m.is_error_frame:
                continue
            func, dev = (m.arbitration_id >> 7) & 0xF, m.arbitration_id & 0x7F
            seen.append(f"id={m.arbitration_id:#05x} func={func:2} dev={dev:3} {bytes(m.data).hex()}")
    finally:
        bus.shutdown()
    return seen


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("-c", "--channel", default="can0")
    ap.add_argument("--ids", type=int, nargs="+", default=None, help="device ids to ping (default 0..127)")
    ap.add_argument("--timeout", type=float, default=0.03, help="ping timeout per id [s]")
    ap.add_argument("--listen", type=float, default=2.0, help="seconds to watch the bus first")
    opts = ap.parse_args(argv)
    ids = opts.ids if opts.ids is not None else list(range(128))

    print(f"=== interface {opts.channel}")
    ok, why = usable(opts.channel)
    if not ok:
        print(f"  {why}")
        return 2
    info = interface_state(opts.channel)
    if not info:
        print(f"  {opts.channel} does not exist. The USB adapter is not plugged in or not enumerated.")
        print("  check: lsusb | grep 1d50:606f")
        return 2
    state = info.get("can state", "?")
    restart = info.get("raw", "").split("restart-ms", 1)
    restart_ms = restart[1].split()[0] if len(restart) > 1 else "?"
    print(f"  bitrate {info.get('bitrate', '?')}   state {state}   restart-ms {restart_ms}"
          f"   tx {info.get('tx_packets', '?')}   rx {info.get('rx_packets', '?')}")
    if state not in ("ERROR-ACTIVE", "?"):
        print(f"  NOTE: the controller is {state}. Unacknowledged frames pushed its error")
        print("        counter up - every frame sent while the motor was off counts.")
        if restart_ms == "0":
            print("        Auto-restart is off, so it will not recover by itself. Reset it with:")
            print(bring_up_command(opts.channel))

    print(f"=== listening for {opts.listen:.0f} s (other traffic, including a running GUI)")
    for line in listen(opts.channel, opts.listen)[:8]:
        print("   ", line)

    print("=== acknowledgement (judged by the controller's error counters)")
    acked, detail = acknowledged(opts.channel)
    label = {True: "ACKed", False: "NOT ACKed", None: "inconclusive"}[acked]
    print(f"  {label}: {detail}")

    print(f"=== pinging {len(ids)} device id(s)")
    answered, total = scan(opts.channel, ids, opts.timeout)
    print(f"  answered: {answered if answered else 'none'}")

    print("\n=== verdict")
    if answered:
        print(f"  {len(answered)} device(s) are answering: {answered}")
        print("  The bus is fine. If control still fails, it is above this layer -")
        print("  gains, mode, or the firmware refusing to leave IDLE.")
        return 0
    if acked is None:
        print("  The controller was already unhealthy, so nothing could be measured.")
        print(bring_up_command(opts.channel))
        return 2
    if acked:
        print("  Our frames are acknowledged, but no device answers the protocol.")
        print("  Transmission succeeding normally means a node acknowledged them, i.e. a")
        print("  board is powered with its CAN peripheral up while its firmware stays mute:")
        print("    1. power-cycle the motor - a hung MCU still acknowledges in hardware")
        print("    2. check the board's power LED")
        print("    3. confirm the stored device id (scripts/motor/read_configurations.py)")
        return 1
    print("  Nothing acknowledges our frames, so no CAN node is reachable at all.")
    print("  That is electrical or a bitrate mismatch, in order of likelihood:")
    print("    1. the motor's logic rail is not powered (check its LED)")
    print("    2. CAN-H and CAN-L swapped at the connector")
    print("    3. termination: CAN-H to CAN-L should read about 60 ohm with the power OFF")
    print("    4. a broken conductor in the cable")
    print("    5. wrong bitrate - try ./can_doctor.sh (needs sudo)")
    return 1


if __name__ == "__main__":
    sys.exit(main())
