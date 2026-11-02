"""/diagnostics content for arm_controller, as plain dicts (no ROS here, so it is
testable next to the controller core).

One entry for the controller as a whole and one per joint, in the shape of
diagnostic_msgs/DiagnosticStatus: level, name, message, hardware_id, values.
rqt_robot_monitor / diagnostic_aggregator on the PC then show the arm's health
without knowing anything about CAN.

    level   OK     joint live (position mode, being commanded), or not driven on purpose
            WARN   enabled in the map but output off -- nothing wrong, call ~/enable
            ERROR  a fault: NO_REPLY, NO_POSITION, OUT_OF_RANGE, NO_PDO_REPLY, LOST, TRACKING
"""

from __future__ import annotations

OK, WARN, ERROR, STALE = 0, 1, 2, 3
PREFIX = "arm_controller"


def joint_entry(name: str, joint: dict) -> dict:
    if joint["fault"]:
        level, message = ERROR, joint["fault"]
    elif joint["live"]:
        level, message = OK, "live"
    elif not joint["enabled"]:
        level, message = OK, "not driven (enabled: false in the joint map)"
    else:
        level, message = WARN, "output off (call ~/enable)"
    return {
        "level": level,
        "name": f"{PREFIX}: {name}",
        "message": message,
        "hardware_id": f"{joint['bus']}:{joint['device_id']}",
        "values": {
            "measured_deg": f"{joint['measured_deg']:.2f}",
            "command_deg": f"{joint['command_deg']:.2f}",
            "goal_deg": f"{joint['goal_deg']:.2f}",
            "limits_deg": f"{joint['limits_deg'][0]:g}..{joint['limits_deg'][1]:g}",
            "fault": joint["fault"],
        },
    }


def summary_entry(status: dict) -> dict:
    joints = status["joints"]
    live = sum(1 for j in joints.values() if j["live"])
    faults = {n: j["fault"] for n, j in joints.items() if j["fault"]}
    driven = sum(1 for j in joints.values() if j["enabled"])
    if status["estop"]:
        level, message = ERROR, f"E-STOP latched: {status['message']}"
    elif faults:
        level, message = ERROR, ", ".join(f"{n} {f}" for n, f in faults.items())
    elif live:
        level, message = OK, f"{live}/{driven} joints live"
    else:
        level, message = WARN, "output off"
    return {
        "level": level,
        "name": PREFIX,
        "message": ("DRY RUN: " if status["dry_run"] else "") + message,
        "hardware_id": ",".join(sorted({j["bus"] for j in joints.values()})),
        "values": {
            "dry_run": str(status["dry_run"]),
            "estop": str(status["estop"]),
            "live": str(live),
            "driven": str(driven),
            "max_speed_deg": f"{status['max_speed_deg']:g}",
            "message": status["message"],
        },
    }


def diagnostics(status: dict) -> list[dict]:
    """Everything /diagnostics carries, summary first, from ArmController.status()."""
    return [summary_entry(status)] + [joint_entry(n, j) for n, j in status["joints"].items()]
