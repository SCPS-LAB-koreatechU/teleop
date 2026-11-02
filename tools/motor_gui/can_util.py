#!/usr/bin/env python3
# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.
"""What the CAN controller itself reports, and what to do about it.

Shared by bus_probe.py and boot_watch.py so both give the same answer.

The controller's error state is the fact that explains a bus which stops working
after a motor is switched off: every frame nobody acknowledges pushes its
transmit error counter up, past 127 it goes ERROR-PASSIVE and past 255 BUS-OFF,
and with `restart-ms 0` it never comes back on its own.
"""

from __future__ import annotations

import subprocess
import time

HEALTHY = "ERROR-ACTIVE"
DEAD_STATES = ("STOPPED", "BUS-OFF", "SLEEPING")


def can_state(channel: str) -> dict:
    """error state, restart-ms, link up/down and the error-transition counters."""
    try:
        out = subprocess.run(["ip", "-details", "-statistics", "link", "show", channel],
                             capture_output=True, text=True, timeout=3)
    except Exception:  # noqa: BLE001 - diagnostics must not raise
        return {}
    if out.returncode != 0:
        return {}
    text = out.stdout
    info: dict[str, str] = {"raw": text}
    if "can state" in text:
        info["state"] = text.split("can state", 1)[1].split()[0]
    if "restart-ms" in text:
        info["restart_ms"] = text.split("restart-ms", 1)[1].split()[0]
    first = text.splitlines()[0] if text else ""
    info["link"] = "UP" if ",UP" in first or "<UP" in first else "DOWN"
    lines = text.splitlines()
    for i, line in enumerate(lines):
        if "bus-off" in line and "error-pass" in line:
            fields = lines[i + 1].split()
            if len(fields) >= 6:
                info.update(zip(("restarts", "bus_errors", "arbit_lost",
                                 "error_warn", "error_pass", "bus_off"), fields[:6]))
            break
    return info


def supports_restart_ms(channel: str) -> bool:
    """Whether this adapter can restart itself from bus-off.

    gs_usb / candleLight adapters cannot: `ip link ... restart-ms 100` is
    refused with "Device doesn't support restart from Bus Off", and the whole
    command fails, leaving the interface down. So the advice has to know which
    adapter it is talking about rather than always recommending restart-ms.
    """
    info = can_state(channel)
    if not info:
        return False
    return "gs_usb" not in info.get("raw", "")


def bring_up_command(channel: str) -> str:
    extra = " restart-ms 100" if supports_restart_ms(channel) else ""
    text = (f"    sudo ip link set {channel} down\n"
            f"    sudo ip link set {channel} up type can bitrate 1000000{extra}\n"
            f"  Run them as two separate lines. A leading '!' in bash negates the first\n"
            f"  command, which makes '&&' skip the second one and leaves the link down.")
    if not extra:
        text += ("\n  This adapter cannot restart itself from bus-off, so a down/up cycle\n"
                 "  is the only way back once the bus wedges. Stop the GUI before switching\n"
                 "  the motor's power off and it will not wedge in the first place.")
    return text


def usable(channel: str) -> tuple[bool, str]:
    """(can we talk on this interface, why not) - check before opening a socket."""
    info = can_state(channel)
    if not info:
        return False, (f"{channel} does not exist. The USB adapter is not plugged in "
                       f"or not enumerated (lsusb | grep 1d50:606f).")
    state = info.get("state", "?")
    if info.get("link") == "DOWN" or state in DEAD_STATES:
        return False, (f"{channel} is {state.lower() if state != '?' else 'down'} - "
                       f"nothing can be sent or received. Bring it up:\n"
                       + bring_up_command(channel))
    return True, f"{channel}: state {state}, restart-ms {info.get('restart_ms', '?')}"


def state_hint(info: dict, channel: str = "can0") -> str | None:
    """Advice when the controller is not in its healthy state."""
    state = info.get("state", "")
    if state in ("", HEALTHY):
        return None
    hint = f"controller is {state}"
    if info.get("restart_ms", "0") == "0":
        hint += ", and it will not recover by itself. Reset it with:\n" + bring_up_command(channel)
    return hint


def _counters(info: dict) -> tuple[int, int, int]:
    def get(key: str) -> int:
        try:
            return int(info.get(key, 0))
        except (TypeError, ValueError):
            return 0
    return get("error_warn"), get("error_pass"), get("bus_off")


def acknowledged(channel: str, frames: int = 20) -> tuple[bool | None, str]:
    """Is anything on the bus acknowledging our frames?

    Judged by the controller's own error counters, not by whether the transmit
    queue drained. The queue is a trap: when nothing acknowledges, the error
    counters climb until the controller goes BUS-OFF, and going BUS-OFF
    *discards* the pending frames - so the queue empties and an unacknowledged
    bus looks exactly like a healthy one. The counters cannot be fooled that
    way.

    Returns (True | False | None, detail). None means the controller was
    already unhealthy, so the test could not say anything.
    """
    import can
    before = can_state(channel)
    if before.get("state") != HEALTHY:
        return None, (f"controller is {before.get('state', '?')} before the test; "
                      f"reset it first and try again")
    warn0, pass0, off0 = _counters(before)
    bus = can.interface.Bus(channel=channel, interface="socketcan")
    sent = 0
    try:
        for _ in range(frames):
            try:
                # 0x100 = function TIME to node 0. The recoil firmware has no
                # handler for it, so every node ACKs it in hardware and does
                # nothing else. (0x600 was RECEIVE_SDO broadcast with a bad
                # command byte: every motor latched ERROR_CAN_RX_FAULT 0x400.)
                bus.send(can.Message(arbitration_id=0x100, data=b"\x00", is_extended_id=False),
                         timeout=0)
                sent += 1
            except can.CanError:
                break
        time.sleep(1.0)
    finally:
        bus.shutdown()
    after = can_state(channel)
    warn1, pass1, off1 = _counters(after)
    degraded = (warn1 > warn0 or pass1 > pass0 or off1 > off0
                or after.get("state") != HEALTHY)
    if degraded:
        return False, (f"{sent} frames sent, controller went {before.get('state')} -> "
                       f"{after.get('state')} (error-warn +{warn1 - warn0}, "
                       f"error-passive +{pass1 - pass0}, bus-off +{off1 - off0})")
    return True, f"{sent} frames sent, controller stayed {after.get('state')} with no new errors"


def transient(e: Exception) -> bool:
    """ENOBUFS and friends: a bus condition, not a broken program."""
    return isinstance(e, OSError) or type(e).__name__.startswith("Can")
