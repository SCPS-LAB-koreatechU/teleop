"""Core tests without ROS or CAN hardware: python3 -m pytest -q test"""

from __future__ import annotations

import math
import struct
from pathlib import Path

import can
import pytest

from bhl_arm_sdk.config import ArmConfig, JointSpec, load_config
from bhl_arm_sdk.controller import ArmController
from bhl_arm_sdk.diagnostics import ERROR, OK, WARN, diagnostics
from bhl_arm_sdk.recoil_can import Function, Mode, Param, RecoilBus

CONFIG = Path(__file__).resolve().parents[1] / "config" / "bhl_arms.yaml"


class FakeMotor:
    def __init__(self, position=0.0, answers=True, follows=True, reversed_=False):
        self.position = position          # rotor rad
        self.answers = answers
        self.follows = follows
        self.reversed = reversed_
        self.mode = Mode.IDLE
        self.gains = None


class FakeBus:
    """Stands in for RecoilBus: motors follow the PDO target in position mode."""

    def __init__(self, channel, reply_timeout=0.004):
        self.channel = channel
        self.motors: dict[int, FakeMotor] = {}
        self.feeds = 0

    def ping(self, device_id, timeout=0.05):
        m = self.motors.get(device_id)
        return bool(m and m.answers)

    def read_f32(self, device_id, param, timeout=0.05):
        m = self.motors.get(device_id)
        if not (m and m.answers):
            return None
        assert param == Param.POSITION_MEASURED
        return m.position

    def set_mode(self, device_id, mode):
        if device_id in self.motors:
            self.motors[device_id].mode = mode

    def write_gains(self, device_id, kp, kd, torque_limit):
        self.motors[device_id].gains = (kp, kd, torque_limit)

    def feed(self, device_id):
        self.feeds += 1

    def exchange_pdo_2(self, device_id, position, velocity=0.0):
        m = self.motors.get(device_id)
        if not (m and m.answers):
            return None
        if m.mode == Mode.POSITION and m.follows:
            m.position = 2 * m.position - position if m.reversed else position
        return m.position, 0.0

    def close(self):
        pass


def two_joint_config(**control) -> ArmConfig:
    return ArmConfig(
        buses={"left": "can0", "right": "can0"},
        joints=[JointSpec("a", "left", 1, direction=-1, lower_deg=-90, upper_deg=45),
                JointSpec("b", "right", 2, direction=1, lower_deg=-45, upper_deg=90)],
        **control)


def live(config, motors):
    buses = {}

    def factory(channel, reply_timeout=0.004):
        bus = FakeBus(channel, reply_timeout)
        bus.motors = motors
        buses[channel] = bus
        return bus

    return ArmController(config, dry_run=False, bus_factory=factory), buses


def run(arm, seconds, dt=0.02):
    for _ in range(round(seconds / dt)):
        arm.step(dt)


# ------------------------------------------------------------------------ config
def test_shipped_config_loads():
    config = load_config(CONFIG)
    assert len(config.joints) == 10
    ids = sorted(j.device_id for j in config.joints)
    assert ids == list(range(1, 11))
    assert config.joint("arm_right_elbow_pitch_joint").kp == 0.8
    assert config.joint("arm_left_shoulder_yaw_joint").kp == 0.3      # from defaults


def test_unit_conversion_round_trip():
    spec = JointSpec("x", "left", 1, direction=-1, gear_ratio=15.0)
    zero = 2.5
    for degrees in (-30.0, 0.0, 12.5, 90.0):
        motor = spec.to_motor(math.radians(degrees), zero)
        assert math.isclose(math.degrees(spec.to_joint(motor, zero)), degrees, abs_tol=1e-9)
    # 10 deg at the joint is 150 deg at the rotor, negated
    assert math.isclose(spec.to_motor(math.radians(10), 0.0), -math.radians(150))


# ---------------------------------------------------------------------- dry run
def test_dry_run_slews_at_max_speed_and_clamps():
    arm = ArmController(two_joint_config(max_speed_deg=30.0), dry_run=True)
    assert arm.enable() == ["a", "b"]
    assert arm.set_goal("a", 100.0) == 45.0          # clamped to upper
    run(arm, 1.0)
    assert math.isclose(arm.joints["a"].measured_deg, 30.0, abs_tol=1e-6)   # 30 deg/s * 1 s
    run(arm, 1.0)
    assert math.isclose(arm.joints["a"].measured_deg, 45.0, abs_tol=1e-6)
    assert arm.settled()


def test_nothing_moves_until_enabled():
    arm = ArmController(two_joint_config(), dry_run=True)
    arm.set_goal("a", 20.0)
    run(arm, 1.0)
    assert arm.joints["a"].measured_deg == 0.0


# ------------------------------------------------------------- live (fake bus)
def test_enable_holds_current_pose_as_zero_and_moves_in_motor_units():
    motors = {1: FakeMotor(position=3.0), 2: FakeMotor(position=-1.0)}
    arm, _ = live(two_joint_config(max_speed_deg=1000.0), motors)
    assert arm.enable() == ["a", "b"]
    assert motors[1].mode == Mode.POSITION and motors[1].gains == (0.3, 0.01, 0.5)
    assert arm.joints["a"].measured_deg == 0.0
    arm.set_goal("a", 10.0)
    run(arm, 0.1)
    # direction -1, gear 15: +10 deg at the joint = -150 deg at the rotor from 3.0
    assert math.isclose(motors[1].position, 3.0 - math.radians(150), abs_tol=1e-9)
    assert math.isclose(arm.joints["a"].measured_deg, 10.0, abs_tol=1e-9)


def test_silent_motor_is_not_enabled():
    motors = {1: FakeMotor(answers=False), 2: FakeMotor()}
    arm, _ = live(two_joint_config(), motors)
    assert arm.enable() == ["b"]
    assert arm.joints["a"].fault == "NO_REPLY"


def test_motor_that_goes_quiet_is_dropped():
    motors = {1: FakeMotor(), 2: FakeMotor()}
    arm, _ = live(two_joint_config(), motors)
    arm.enable()
    motors[1].answers = False
    run(arm, 1.0)
    assert arm.joints["a"].fault == "LOST" and not arm.joints["a"].live
    assert arm.joints["b"].live


def test_reversed_motor_trips_tracking_estop():
    motors = {1: FakeMotor(), 2: FakeMotor(reversed_=True)}
    arm, _ = live(two_joint_config(max_speed_deg=60.0), motors)
    arm.enable()
    arm.set_goal("b", 40.0)
    run(arm, 1.0)
    assert arm.estopped and arm.joints["b"].fault == "TRACKING"
    assert motors[1].mode == Mode.DAMPING and motors[2].mode == Mode.DAMPING
    assert arm.enable() == []                       # latched until reset
    arm.reset_estop()
    motors[2].reversed = False
    assert "b" in arm.enable()


def test_range_estop_ignores_lag_but_trips_outside_limits():
    def range_config():
        config = two_joint_config(max_speed_deg=60.0)
        config.joint("b").estop = "range"
        config.joint("b").range_margin_deg = 5.0
        return config

    motors = {1: FakeMotor(), 2: FakeMotor(follows=False)}
    arm, _ = live(range_config(), motors)
    arm.enable()
    arm.set_goal("b", 40.0)                         # b stays at 0: 40 deg lag would trip TRACKING
    run(arm, 1.0)
    assert not arm.estopped and arm.joints["b"].live

    motors = {1: FakeMotor(), 2: FakeMotor(reversed_=True)}
    arm, _ = live(range_config(), motors)
    arm.enable()
    arm.set_goal("b", 40.0)                         # reversed b runs away past [-45, 90] +- 5
    run(arm, 2.0)
    assert arm.estopped and arm.joints["b"].fault == "RANGE"
    assert motors[1].mode == Mode.DAMPING and motors[2].mode == Mode.DAMPING


def test_out_of_range_start_is_refused():
    config = two_joint_config()
    config.zero = "config"
    config.joints[1].motor_zero = 0.0
    config.joints[0].motor_zero = 0.0
    motors = {1: FakeMotor(position=0.0), 2: FakeMotor(position=math.radians(-60) * 15)}   # b at -60 deg
    arm, _ = live(config, motors)
    assert arm.enable() == ["a"]
    assert arm.joints["b"].fault.startswith("OUT_OF_RANGE")


def test_one_adapter_shared_by_both_bus_keys():
    motors = {1: FakeMotor(), 2: FakeMotor()}
    arm, buses = live(two_joint_config(), motors)
    assert len(buses) == 1 and arm.buses["left"] is arm.buses["right"]


# ----------------------------------------------------------------- CAN framing
class CaptureBus:
    def __init__(self, replies=()):
        self.sent = []
        self.replies = list(replies)

    def send(self, message):
        self.sent.append(message)

    def recv(self, timeout=None):
        return self.replies.pop(0) if self.replies else None

    def shutdown(self):
        pass


def make_bus(replies=()):
    bus = RecoilBus.__new__(RecoilBus)
    bus.channel, bus.reply_timeout = "test", 0.004
    bus._bus = CaptureBus(replies)
    return bus


def reply(function, device_id, data):
    return can.Message(arbitration_id=(function << 7) | device_id, is_extended_id=False, data=data)


def test_frames_match_upstream_layout():
    bus = make_bus()
    bus.set_mode(8, Mode.POSITION)
    bus.write_f32(8, Param.POSITION_KP, 0.8)
    bus.feed(8)
    nmt, sdo, beat = bus._bus.sent
    assert nmt.arbitration_id == (Function.NMT << 7) | 8 and bytes(nmt.data) == bytes([0x13, 8])
    assert sdo.arbitration_id == (Function.RECEIVE_SDO << 7) | 8
    assert bytes(sdo.data) == struct.pack("<BHB", 0x20, 0x020, 0) + struct.pack("<f", 0.8)
    assert beat.arbitration_id == (Function.HEARTBEAT << 7) | 8 and len(beat.data) == 0


def test_pdo_reply_ignores_other_functions():
    # a late SDO reply from the same motor must not be read as the position
    stale_sdo = reply(Function.TRANSMIT_SDO, 8, struct.pack("<ff", 99.0, 99.0))
    pdo = reply(Function.TRANSMIT_PDO_2, 8, struct.pack("<ff", 1.5, -0.2))
    bus = make_bus([stale_sdo, pdo])
    position, velocity = bus.exchange_pdo_2(8, 1.0)
    assert math.isclose(position, 1.5) and math.isclose(velocity, -0.2, rel_tol=1e-6)
    sent = bus._bus.sent[0]
    assert sent.arbitration_id == (Function.RECEIVE_PDO_2 << 7) | 8
    assert bytes(sent.data) == struct.pack("<ff", 1.0, 0.0)


def test_pdo_reply_from_other_device_is_ignored():
    other = reply(Function.TRANSMIT_PDO_2, 6, struct.pack("<ff", 5.0, 0.0))
    bus = make_bus([other])
    assert bus.exchange_pdo_2(8, 0.0) is None


# ----------------------------------------------------------------- diagnostics
def by_name(entries):
    return {e["name"]: e for e in entries}


def test_diagnostics_output_off_then_live():
    arm = ArmController(two_joint_config(), dry_run=True)
    entries = by_name(diagnostics(arm.status()))
    assert entries["arm_controller"]["level"] == WARN                 # nothing enabled yet
    assert entries["arm_controller"]["message"].startswith("DRY RUN")
    assert entries["arm_controller: a"]["level"] == WARN
    assert entries["arm_controller: a"]["hardware_id"] == "left:1"
    arm.enable()
    arm.set_goal("a", 10.0)
    run(arm, 1.0)
    entries = by_name(diagnostics(arm.status()))
    assert entries["arm_controller"]["level"] == OK
    assert entries["arm_controller"]["values"]["live"] == "2"
    assert entries["arm_controller: a"]["level"] == OK
    assert entries["arm_controller: a"]["values"]["measured_deg"] == "10.00"
    assert entries["arm_controller: a"]["values"]["limits_deg"] == "-90..45"


def test_diagnostics_faults_and_estop_are_errors():
    motors = {1: FakeMotor(answers=False), 2: FakeMotor(reversed_=True)}
    arm, _ = live(two_joint_config(max_speed_deg=60.0), motors)
    arm.enable()                                   # a: NO_REPLY, b: live
    entries = by_name(diagnostics(arm.status()))
    assert entries["arm_controller: a"]["level"] == ERROR
    assert entries["arm_controller: a"]["message"] == "NO_REPLY"
    assert entries["arm_controller: b"]["level"] == OK
    assert entries["arm_controller"]["level"] == ERROR and "a NO_REPLY" in entries["arm_controller"]["message"]
    arm.set_goal("b", 40.0)
    run(arm, 1.0)                                  # b runs away -> TRACKING -> E-STOP
    entries = by_name(diagnostics(arm.status()))
    assert entries["arm_controller"]["level"] == ERROR
    assert entries["arm_controller"]["message"].startswith("E-STOP latched")
    assert entries["arm_controller"]["values"]["estop"] == "True"
    assert entries["arm_controller: b"]["message"] == "TRACKING"


def test_diagnostics_disabled_joint_is_ok_not_warn():
    config = two_joint_config()
    config.joints[1].enabled = False
    arm = ArmController(config, dry_run=True)
    entries = by_name(diagnostics(arm.status()))
    assert entries["arm_controller: b"]["level"] == OK
    assert "enabled: false" in entries["arm_controller: b"]["message"]
    assert entries["arm_controller"]["values"]["driven"] == "1"


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-q"]))
