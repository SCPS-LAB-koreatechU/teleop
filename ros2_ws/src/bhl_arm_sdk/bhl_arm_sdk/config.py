"""Joint map: which motor drives which joint, and how to turn degrees into motor units.

    motor_position = motor_zero + direction * gear_ratio * joint_rad
    joint_rad      = direction * (motor_position - motor_zero) / gear_ratio

`gear_ratio` is in motor units per joint radian. This lab's actuators keep the
firmware gear ratio at 1, so they report rotor radians and the 15:1 gearbox
goes here. `motor_zero` is the motor reading at joint 0: taken from the pose
the arm is in when control starts (`zero: start`, what the upstream Bimanual
class does) or fixed per joint (`zero: config` + `motor_zero`).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

import yaml


@dataclass
class JointSpec:
    name: str
    bus: str                    # key into `buses`
    device_id: int
    direction: int = 1
    gear_ratio: float = 15.0
    lower_deg: float = -45.0
    upper_deg: float = 45.0
    kp: float = 0.3
    kd: float = 0.01
    torque_limit: float = 0.5
    enabled: bool = True
    motor_zero: float | None = None   # [motor units]; None = take it at start
    estop: str = "tracking"           # "tracking": lag > max error stops all; "range": leaving the limits stops all
    range_margin_deg: float = 5.0     # "range" only: how far past lower/upper before stopping

    def to_motor(self, joint_rad: float, zero: float) -> float:
        return zero + self.direction * self.gear_ratio * joint_rad

    def to_joint(self, motor: float, zero: float) -> float:
        return self.direction * (motor - zero) / self.gear_ratio

    def clamp_deg(self, value: float) -> float:
        return min(max(value, self.lower_deg), self.upper_deg)


@dataclass
class ArmConfig:
    buses: dict[str, str]                     # bus key -> socketcan channel
    joints: list[JointSpec]
    zero: str = "start"                       # start | config
    control_rate_hz: float = 50.0             # G1 example: 50 Hz (control_dt 0.02)
    max_speed_deg: float = 30.0               # per-joint slew limit [deg/s]
    init_duration: float = 2.0                # [s] ramp of the "go to pose" call (G1: 2 s)
    reply_timeout: float = 0.004              # [s] PDO reply wait
    heartbeat_period: float = 0.2             # [s] firmware watchdog is 1 s
    extra: dict = field(default_factory=dict)

    def joint(self, name: str) -> JointSpec:
        for spec in self.joints:
            if spec.name == name:
                return spec
        raise KeyError(name)

    @property
    def names(self) -> list[str]:
        return [j.name for j in self.joints]


def load_config(path: str | Path) -> ArmConfig:
    data = yaml.safe_load(Path(path).read_text())
    defaults = data.get("defaults") or {}
    joints = []
    for name, spec in (data.get("joints") or {}).items():
        merged = {**defaults, **(spec or {})}
        joints.append(JointSpec(
            name=name,
            bus=str(merged["bus"]),
            device_id=int(merged["device_id"]),
            direction=int(merged.get("direction", 1)),
            gear_ratio=float(merged.get("gear_ratio", 15.0)),
            lower_deg=float(merged.get("lower_deg", -45.0)),
            upper_deg=float(merged.get("upper_deg", 45.0)),
            kp=float(merged.get("kp", 0.3)),
            kd=float(merged.get("kd", 0.01)),
            torque_limit=float(merged.get("torque_limit", 0.5)),
            enabled=bool(merged.get("enabled", True)),
            motor_zero=None if merged.get("motor_zero") is None else float(merged["motor_zero"]),
            estop=str(merged.get("estop", "tracking")),
            range_margin_deg=float(merged.get("range_margin_deg", 5.0)),
        ))
        if joints[-1].direction not in (-1, 1):
            raise ValueError(f"{name}: direction must be +1 or -1")
        if joints[-1].lower_deg > joints[-1].upper_deg:
            raise ValueError(f"{name}: lower_deg > upper_deg")
        if joints[-1].estop not in ("tracking", "range"):
            raise ValueError(f"{name}: estop must be tracking or range")
    control = data.get("control") or {}
    config = ArmConfig(
        buses={str(k): str(v) for k, v in (data.get("buses") or {}).items()},
        joints=joints,
        zero=str(control.get("zero", "start")),
        control_rate_hz=float(control.get("control_rate_hz", 50.0)),
        max_speed_deg=float(control.get("max_speed_deg", 30.0)),
        init_duration=float(control.get("init_duration", 2.0)),
        reply_timeout=float(control.get("reply_timeout", 0.004)),
        heartbeat_period=float(control.get("heartbeat_period", 0.2)),
        extra={k: v for k, v in data.items() if k not in ("buses", "joints", "defaults", "control")},
    )
    if config.zero not in ("start", "config"):
        raise ValueError("control.zero must be start or config")
    missing = {j.bus for j in joints} - set(config.buses)
    if missing:
        raise ValueError(f"joints use buses with no channel: {sorted(missing)}")
    if config.zero == "config":
        unset = [j.name for j in joints if j.enabled and j.motor_zero is None]
        if unset:
            raise ValueError(f"zero: config needs motor_zero for {unset}")
    return config


def deg(rad: float) -> float:
    return math.degrees(rad)


def rad(degrees: float) -> float:
    return math.radians(degrees)
