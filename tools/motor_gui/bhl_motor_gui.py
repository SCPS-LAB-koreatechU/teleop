#!/usr/bin/env python3
# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.
"""BHL arm motor GUI: all ten CAN actuators, in URDF joint units.

Every joint of `berkeley_humanoid_lite_*.urdf` that is driven by a recoil
actuator (5 per arm: shoulder pitch/roll/yaw, elbow pitch/roll) gets one row.
Sliders are ABSOLUTE joint angles in degrees, bounded by the URDF limits; the
same angle can be dragged, stepped (buttons / arrow keys / mouse wheel) or
typed in, so a fraction of a degree never depends on hitting a pixel. The
CAN-ID / direction / gear-ratio / offset mapping is read from the same
`hardware_joints.yaml` the MoveIt hardware bridge uses, so a value that is
right here is right for MoveIt too.

Modes
    --sim        no CAN at all: a fake bus with first-order motors (for trying
                 the GUI, for RViz, for teaching)
    (default)    live: one socketcan channel per arm (bus.left / bus.right in
                 the yaml); a channel that does not exist is shown as NO BUS
                 and its rows stay read-only
    --ros        additionally publish sensor_msgs/JointState on /joint_states
                 so `display.launch.py gui:=false` shows the arm in RViz

Safety (same design as gui_arm_control.py / hardware_bridge.py)
    - a joint is only energised after it answers, its error register is
      cleared, gains are written, and its position is read twice and agrees
    - a joint whose reading is outside the URDF range is REFUSED
      (OUT_OF_RANGE): that is an uncalibrated offset, use "Zero here" first
    - the slider is set to the measured angle on enable, so enabling never
      moves the arm; every target is slew-rate limited and clamped inside the
      URDF limits minus a margin
    - joints marked `enabled: false` in the yaml are locked until the global
      "unlock" box is ticked
    - E-STOP is out-of-band (event flag), sends IDLE twice to every motor
    - a motor that stops answering PDOs is frozen, then auto-disabled (LOST)
    - control threads are exception-guarded and IDLE everything on the way out
    - all CAN traffic for one channel happens in exactly one thread; the GUI
      talks to it through a queue and shared state objects where each field
      is written by exactly one thread
"""

from __future__ import annotations

import argparse
import math
import os
import queue
import re
import struct
import sys
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent
DEFAULT_CONFIG = REPO / "ros2_ws/src/berkeley_humanoid_lite_moveit_config/config/hardware_joints.yaml"
DEFAULT_URDF = REPO / "ros2_ws/src/berkeley_humanoid_lite_description/urdf/berkeley_humanoid_lite_v1arm.urdf"
DEFAULT_SRDF = REPO / "ros2_ws/src/berkeley_humanoid_lite_moveit_config/config/berkeley_humanoid_lite_v1arm.srdf"

# gains reaching live motors are clamped into these ranges (a kp typo of 7
# instead of 0.7 must never hit the hardware)
GAIN_LIMITS = {"kp": (0.0, 5.0), "kd": (0.0, 0.5), "torque_limit": (0.0, 2.0)}
SPEED_LIMITS = (0.05, 3.0)          # [rad/s] slew-rate slider range
STEP_FINE = 0.1                     # [deg] slider resolution, wheel+shift, smallest step button
STEP_COARSE = 1.0                   # [deg] plain wheel notch and arrow key

# A refused Enable used to show only a red word; every one of these has a
# specific next step, so say it instead of leaving the operator guessing.
FAULT_HELP = {
    "OUT_OF_RANGE": "the reading is outside the URDF limits, so the offset is not calibrated yet. "
                    "Pose the joint at its URDF zero, press 'Zero here', then 'Save offsets'.",
    "NO REPLY": "no answer on the bus - check power, the CAN wiring and the device id in the yaml",
    "BAD READ": "two position reads disagreed - noisy link; Ping, then try Enable again",
    "LOCKED": "the yaml has enabled: false for this joint - tick 'unlock yaml-disabled joints'",
    "LOST": "the motor stopped answering and was disabled - fix the link, then Enable again",
    "NO BUS": "this CAN channel does not exist - bring it up, or the arm is simply not connected",
}
ERROR_POLL_PERIOD = 2.0             # [s] per live motor
IDLE_POLL_PERIOD = 0.10             # [s] round-robin position read of idle motors
IDLE_PING_BACKOFF = 1.0             # [s] a silent motor is pinged this often
SDO_TIMEOUT = 0.03                  # [s] idle position read; a motor that answers
                                    #     a little late must not read as absent
IDLE_MISS_LIMIT = 3                 # consecutive idle reads missed before "offline"
DEAD_BUS_AFTER = 5.0                # [s] with nobody answering before backing off
DEAD_BUS_POLL_PERIOD = 2.0          # [s] how often to look after that
PDO_TIMEOUT = 0.004                 # [s] wait for a motor's position reply each cycle
FEED_PERIOD = 0.2                   # [s] heartbeat per live motor (firmware watchdog)
FUNC_TRANSMIT_PDO_2 = 0b0101        # firmware FUNC_TRANSMIT_PDO_2 (0b0111 is PDO_3 -- was wrong until 2026-10-02)


def can_controller_state(channel: str) -> tuple[str, str]:
    """(error state, restart-ms) as the CAN controller itself reports them.

    Every frame nobody acknowledges pushes the controller's transmit error
    counter up; past 127 it goes ERROR-PASSIVE and past 255 BUS-OFF. That is
    what switching a motor off underneath a running window does, and with
    restart-ms 0 the interface never recovers on its own - so the banner says
    so instead of leaving the user guessing.
    """
    try:
        import subprocess
        out = subprocess.run(["ip", "-details", "link", "show", channel],
                             capture_output=True, text=True, timeout=2)
        text = out.stdout if out.returncode == 0 else ""
    except Exception:  # noqa: BLE001 - diagnostics must never break the GUI
        return "?", "?"
    state = text.split("can state", 1)[1].split()[0] if "can state" in text else "?"
    restart = text.split("restart-ms", 1)[1].split()[0] if "restart-ms" in text else "?"
    return state, restart


def clamp(value: float, lo: float, hi: float) -> float:
    return min(max(value, lo), hi)


def clamp_gain(name: str, value: float) -> float:
    return clamp(value, *GAIN_LIMITS[name])


# ----------------------------------------------------------------------------
# configuration: URDF limits, SRDF presets, hardware map
# ----------------------------------------------------------------------------

def load_urdf_limits(path: Path) -> dict[str, tuple[float, float]]:
    limits = {}
    for joint in ET.parse(path).getroot().findall("joint"):
        if joint.get("type") == "fixed":
            continue
        lim = joint.find("limit")
        if lim is None:
            continue
        limits[joint.get("name")] = (float(lim.get("lower", -math.pi)), float(lim.get("upper", math.pi)))
    return limits


def load_srdf_presets(path: Path | None, joint_names: list[str]) -> dict[str, dict[str, float]]:
    """{preset: {joint: rad}} merged over every group_state that names our joints."""
    presets: dict[str, dict[str, float]] = {}
    if path is None or not path.is_file():
        return presets
    wanted = set(joint_names)
    for gs in ET.parse(path).getroot().findall("group_state"):
        values = {j.get("name"): float(j.get("value")) for j in gs.findall("joint") if j.get("name") in wanted}
        if values:
            presets.setdefault(gs.get("name"), {}).update(values)
    return presets


class JointSpec:
    """Where one actuator lives and how its radians map onto the URDF joint.

    Conversions are identical to hardware_bridge.JointHandle so an `offset`
    zeroed here is valid for MoveIt.
    """

    def __init__(self, name: str, spec: dict, channel: str, limits: tuple[float, float]):
        self.name = name
        self.bus_key = spec["bus"]
        self.channel = channel
        self.device_id = int(spec["device_id"])
        self.direction = float(spec.get("direction", 1.0))
        self.gear_ratio = float(spec.get("gear_ratio", 1.0))
        if self.gear_ratio == 0.0:
            raise SystemExit(f"{name}: gear_ratio must not be 0")
        self.offset = float(spec.get("offset", 0.0))       # written by control thread (zero here)
        self.config_enabled = bool(spec.get("enabled", False))
        self.lower, self.upper = limits

    @property
    def short(self) -> str:
        s = self.name.replace("arm_", "").replace("_joint", "")
        return s.replace("left_", "L ").replace("right_", "R ")

    def to_actuator(self, urdf_rad: float) -> float:
        return (urdf_rad + self.offset) * self.direction * self.gear_ratio

    def to_urdf(self, raw: float) -> float:
        return raw / self.gear_ratio * self.direction - self.offset

    def offset_for_zero_here(self, raw: float) -> float:
        return raw / self.gear_ratio * self.direction


def load_config(config_path: Path, urdf_limits: dict[str, tuple[float, float]],
                only_ids: set[int] | None = None,
                bus_override: dict[str, str] | None = None):
    cfg = yaml.safe_load(config_path.read_text())
    buses: dict[str, str] = dict(cfg.get("bus", {}))
    # On the bench there is usually one adapter, while the yaml describes the
    # finished robot: the right arm lives on can1 there, so a right-arm motor
    # plugged into the only adapter could not be opened at all.
    for key, channel in (bus_override or {}).items():
        if key not in buses:
            raise SystemExit(f"--bus {key}={channel}: the config has no bus '{key}' "
                             f"(it defines {sorted(buses)})")
        buses[key] = channel
    gains = cfg.get("gains", {})
    safety = cfg.get("safety", {})
    specs: list[JointSpec] = []
    for name, spec in cfg.get("joints", {}).items():
        if name not in urdf_limits:
            print(f"[WARN] {name} is in {config_path.name} but not in the URDF; skipped")
            continue
        if only_ids is not None and int(spec["device_id"]) not in only_ids:
            continue
        channel = buses.get(spec["bus"])
        if channel is None:
            raise SystemExit(f"{name}: bus '{spec['bus']}' has no channel under bus:")
        specs.append(JointSpec(name, spec, channel, urdf_limits[name]))
    if not specs:
        which = f" matching --ids {sorted(only_ids)}" if only_ids else ""
        raise SystemExit(f"no CAN joints found in {config_path}{which}")
    if only_ids:
        missing = only_ids - {s.device_id for s in specs}
        if missing:
            raise SystemExit(f"--ids: no joint in {config_path.name} uses device id {sorted(missing)}")
    return buses, gains, safety, specs


def save_offsets(config_path: Path, offsets: dict[str, float]) -> Path:
    """Rewrite `offset:` lines under joints: in place, keeping comments; .bak kept."""
    lines = config_path.read_text().splitlines(keepends=True)
    out, current, in_joints = [], None, False
    for line in lines:
        stripped = line.rstrip("\n")
        if re.match(r"^\S", stripped):                    # top-level key
            in_joints = stripped.startswith("joints:")
            current = None
        elif in_joints:
            m = re.match(r"^  ([A-Za-z0-9_]+):\s*$", stripped)
            if m:
                current = m.group(1)
            elif current in offsets:
                m2 = re.match(r"^(\s+offset:\s*)([^#\s]+)(.*)$", stripped)
                if m2:
                    # trim trailing zeros so an unchanged 0.0 stays "0.0" and the
                    # file does not pick up a diff it did not earn
                    text = f"{offsets[current]:.6f}".rstrip("0")
                    line = f"{m2.group(1)}{text + '0' if text.endswith('.') else text}{m2.group(3)}\n"
        out.append(line)
    backup = config_path.with_suffix(config_path.suffix + ".bak")
    backup.write_text("".join(lines))
    config_path.write_text("".join(out))
    return backup


# ----------------------------------------------------------------------------
# bus backends
# ----------------------------------------------------------------------------

class SimMotor:
    def __init__(self, raw0: float):
        self.raw = raw0
        self.vel = 0.0
        self.target = raw0
        self.mode = 0x01
        self.error = 0
        self.t = time.monotonic()

    def step(self):
        now = time.monotonic()
        dt = min(now - self.t, 0.05)
        self.t = now
        if self.mode == 0x13:  # POSITION
            tau = 0.06
            new = self.raw + (self.target - self.raw) * min(1.0, dt / tau)
            self.vel = (new - self.raw) / dt if dt > 0 else 0.0
            self.raw = new
        else:
            self.vel = 0.0


class SimFrame:
    """Just enough of recoil's CANFrame for the control loop to unpack."""

    def __init__(self, data: bytes):
        self.data = data
        self.size = len(data)


class SimBus:
    """Stand-in for recoil.Bus with the subset of methods the GUI uses."""

    def __init__(self, channel: str, device_ids: list[int], raw0: dict[int, float], offline: set[int]):
        self.channel = channel
        self.device_ids = list(device_ids)
        self.raw0 = dict(raw0)
        self.motors = {d: SimMotor(raw0.get(d, 0.0)) for d in device_ids if d not in offline}
        self.params: dict[tuple[int, int], float] = {}

    def set_present(self, dev: int, present: bool) -> None:
        """Test hook: pull a motor off the bus (or put it back) while running.

        Nothing in the GUI calls this; the sim verification uses it to make a
        live motor go silent, which is the one failure the safety logic has to
        catch on its own (freeze, then auto-disable).
        """
        if present:
            self.motors.setdefault(dev, SimMotor(self.raw0.get(dev, 0.0)))
        else:
            self.motors.pop(dev, None)

    def _m(self, dev: int) -> SimMotor | None:
        return self.motors.get(dev)

    def ping(self, dev: int, timeout=0.1) -> bool:
        return dev in self.motors

    def feed(self, dev: int) -> None:
        pass

    def set_mode(self, dev: int, mode: int) -> None:
        m = self._m(dev)
        if m:
            m.mode = mode

    def receive(self, timeout=0, filter_device_id=None, filter_function=None, **_kw):
        """Only the one case the control loop needs: a motor's PDO_2 reply."""
        if filter_function != FUNC_TRANSMIT_PDO_2 or filter_device_id is None:
            return None
        m = self._m(filter_device_id)
        if not m:
            return None                      # absent motor: silence, as on real hardware
        m.step()
        return SimFrame(struct.pack("<ff", m.raw, m.vel))

    def transmit_pdo_2(self, dev: int, position_target: float, velocity_target: float) -> None:
        m = self._m(dev)
        if m:
            m.target = position_target

    def receive_pdo_2(self, dev: int):
        m = self._m(dev)
        if not m:
            return None, None
        m.step()
        return m.raw, m.vel

    def _read_parameter_f32(self, dev: int, param: int, timeout=None):
        m = self._m(dev)
        if not m:
            return None
        m.step()
        return m.raw if param == 0x060 else self.params.get((dev, param), 0.0)

    def _read_parameter_u32(self, dev: int, param: int, timeout=None):
        m = self._m(dev)
        return None if m is None else (m.error if param == 0x014 else 0)

    def _write_parameter_u32(self, dev: int, param: int, value: int) -> None:
        m = self._m(dev)
        if m and param == 0x014:
            m.error = value

    def write_position_kp(self, dev, v): self.params[(dev, 0x020)] = v
    def write_position_kd(self, dev, v): self.params[(dev, 0x028)] = v
    def write_torque_limit(self, dev, v): self.params[(dev, 0x030)] = v

    def stop(self) -> None:
        pass


def open_bus(channel: str, sim_ids: list[int] | None, sim_raw0: dict[int, float], sim_offline: set[int]):
    """Return (bus, reason). bus is None when the channel is unusable."""
    if sim_ids is not None:
        return SimBus(channel, sim_ids, sim_raw0, sim_offline), "sim"
    try:
        import berkeley_humanoid_lite_lowlevel.recoil as recoil
    except ImportError as e:
        return None, f"recoil import failed: {e} (run via run.sh or set PYTHONPATH to the lowlevel package)"
    # Check the interface exists first: constructing a Bus that fails leaves a
    # half-built object whose __del__ prints a second, confusing traceback.
    if not Path(f"/sys/class/net/{channel}").exists():
        return None, f"no such CAN interface ({channel})"
    state, _restart = can_controller_state(channel)
    if state in ("STOPPED", "SLEEPING"):
        return None, f"{channel} is down - bring it up (see the banner)"
    try:
        return recoil.Bus(channel=channel, bitrate=1000000), "live"
    except Exception as e:  # noqa: BLE001 - python-can raises many types; all mean "no bus"
        return None, f"{type(e).__name__}: {str(e)[:80]}"


# ----------------------------------------------------------------------------
# shared state + per-bus control thread
# ----------------------------------------------------------------------------

class MotorState:
    def __init__(self, spec: JointSpec):
        self.spec = spec
        self.dev = spec.device_id
        self.online = False
        self.seen = False          # ever answered
        self.live = False          # in POSITION mode, being driven
        self.fault: str | None = None
        self.raw: float | None = None
        self.measured: float | None = None      # URDF rad
        self.measured_vel: float | None = None
        self.error_code: int | None = None
        self.target = 0.0          # slider goal, URDF rad (GUI thread)
        self.command = 0.0         # slew-limited, URDF rad (control thread)
        self.timeouts = 0
        self.idle_misses = 0       # consecutive idle polls with no answer
        self.cmd_pending = 0       # GUI thread
        self.cmd_done = 0          # control thread
        self.enable_epoch = 0      # bumps every successful enable (GUI syncs slider)
        # goal clamp band [rad]: URDF limits shrunk by the safety margin, then
        # widened to include URDF zero and wherever the joint was when armed
        # (see BusThread._do_enable)
        self.lo_m = spec.lower
        self.hi_m = spec.upper

    @property
    def busy(self) -> bool:
        return self.cmd_pending != self.cmd_done


class BusThread(threading.Thread):
    MODE_IDLE = 0x01
    MODE_POSITION = 0x13
    PARAM_ERROR = 0x014
    PARAM_POS_MEASURED = 0x060

    def __init__(self, channel: str, bus, motors: list[MotorState], gains: dict, safety: dict, rate_hz: float):
        super().__init__(daemon=True, name=f"bus-{channel}")
        self.channel = channel
        self.bus = bus
        self.motors = {m.dev: m for m in motors}
        self.gains = gains
        self.max_speed = float(safety.get("max_joint_speed", 1.0))       # GUI writes
        self.limit_margin = float(safety.get("limit_margin", 0.05))
        self.link_flaky = int(safety.get("link_flaky_cycles", 5))
        self.pdo_loss = int(safety.get("pdo_loss_threshold", 100))
        self.pdo_timeout = float(safety.get("pdo_timeout", PDO_TIMEOUT))
        self.blocked: str | None = None     # set while the bus refuses traffic
        self.quiet = False                  # nothing on this bus has answered lately
        self._last_answer = time.monotonic()   # last time ANY motor here replied
        self._blocked_backoff = 0.1
        self.sdo_timeout = float(safety.get("sdo_timeout", SDO_TIMEOUT))
        self.idle_miss_limit = max(1, int(safety.get("idle_miss_limit", IDLE_MISS_LIMIT)))
        self._last_feed = {d: 0.0 for d in self.motors}
        self.rate_hz = rate_hz
        self.commands: queue.Queue = queue.Queue()
        self.estop_flag = threading.Event()
        self.running = True
        self.dead: str | None = None
        self.unlocked = False       # GUI writes: allow config-disabled joints
        self._last_error_poll = {d: 0.0 for d in self.motors}
        self._last_idle_poll = 0.0
        self._idle_idx = 0
        self._silent_since = {d: 0.0 for d in self.motors}

    # --- GUI side ---
    def request(self, cmd: str, dev: int | None = None, payload=None):
        self.commands.put((cmd, dev, payload))

    def request_motor_cmd(self, cmd: str, dev: int):
        self.motors[dev].cmd_pending += 1
        self.commands.put((cmd, dev, None))

    def trigger_estop(self):
        if self.estop_flag.is_set():
            return
        for m in self.motors.values():
            m.cmd_pending += 1
        self.estop_flag.set()

    def force_idle_all(self):
        for dev in self.motors:
            try:
                self.bus.set_mode(dev, self.MODE_IDLE)
            except Exception:
                pass

    # --- bus side ---
    @staticmethod
    def _is_transient(e: Exception) -> bool:
        """A blocked transmit queue is a bus condition, not a broken program.

        When no node acknowledges a frame the controller retries it forever,
        the socket's queue fills, and python-can raises ENOBUFS. That happens
        every time the motor's power is switched off. Killing the control
        thread for it meant the window went dead and had to be restarted, so
        these are absorbed and reported instead.
        """
        return isinstance(e, OSError) or type(e).__name__.startswith("Can")

    def _flush_rx(self):
        while self.bus.receive(timeout=0) is not None:
            pass

    def _recv_pdo(self, dev: int):
        """One motor's position/velocity reply, or None.

        Deliberately not recoil's `receive_pdo_2`: that hardcodes a 1 ms wait,
        which a motor answering a little later misses every cycle, and it
        prints a line per miss, which floods the console at 100 Hz. A reply
        that arrives after the window is still picked up on the next cycle,
        because the socket keeps it and the filter matches only this frame.
        """
        frame = self.bus.receive(filter_device_id=dev, filter_function=FUNC_TRANSMIT_PDO_2,
                                 timeout=self.pdo_timeout)
        if frame is None or getattr(frame, "size", 0) < 8:
            return None
        return struct.unpack("<ff", bytes(frame.data[0:8]))

    def _read_pos(self, dev: int, timeout: float):
        return self.bus._read_parameter_f32(dev, self.PARAM_POS_MEASURED, timeout=timeout)

    def _store_measurement(self, m: MotorState, raw: float, vel: float | None):
        m.idle_misses = 0
        self._last_answer = time.monotonic()
        m.raw = raw
        m.measured = m.spec.to_urdf(raw)
        m.measured_vel = None if vel is None else vel / m.spec.gear_ratio * m.spec.direction
        m.seen = True
        m.online = True

    def _do_enable(self, dev: int):
        m = self.motors[dev]
        s = m.spec
        try:
            if self.estop_flag.is_set():
                return
            if not s.config_enabled and not self.unlocked:
                m.fault = "LOCKED"
                return
            if self.quiet or self.blocked:
                # Nothing on this bus is answering. Enabling would queue a ping,
                # three parameter writes and two reads, and an unacknowledged
                # frame is retried by the controller forever - so that burst
                # turns into a permanent error storm that only an interface
                # down/up clears. Refuse before sending anything.
                m.fault = "NO BUS" if self.blocked else "NO REPLY"
                return
            m.fault = None
            if not self.bus.ping(dev, timeout=0.05):
                m.online = False
                m.fault = "NO REPLY"
                return
            m.online = True
            self.bus._write_parameter_u32(dev, self.PARAM_ERROR, 0)
            self.bus.write_position_kp(dev, self.gains["kp"])
            self.bus.write_position_kd(dev, self.gains["kd"])
            self.bus.write_torque_limit(dev, self.gains["torque_limit"])
            self._flush_rx()
            p1 = self._read_pos(dev, 0.2)
            p2 = self._read_pos(dev, 0.2)
            if p1 is None or p2 is None or abs(p1 - p2) > 0.5:
                m.fault = "BAD READ"
                return
            urdf = s.to_urdf(p2)
            if not (s.lower - self.limit_margin <= urdf <= s.upper + self.limit_margin):
                self._store_measurement(m, p2, None)
                m.fault = "OUT_OF_RANGE"
                return
            if self.estop_flag.is_set():
                return
            self._store_measurement(m, p2, 0.0)
            m.command = urdf
            m.target = urdf
            lo = s.lower + self.limit_margin
            hi = s.upper - self.limit_margin
            # Both elbow_pitch joints have a URDF limit sitting exactly on zero,
            # so a plain margin would put their own home pose out of reach and
            # the home preset would stop 3 deg short. Zero is a pose the model
            # declares valid, so it stays commandable; the margin still guards
            # the far end and every other joint.
            if s.lower <= 0.0 <= s.upper:
                lo, hi = min(lo, 0.0), max(hi, 0.0)
            # ... and the position it was armed at, so enabling never moves it.
            m.lo_m = min(lo, urdf)
            m.hi_m = max(hi, urdf)
            m.timeouts = 0
            self.bus.set_mode(dev, self.MODE_POSITION)
            self.bus.feed(dev)
            self._last_feed[dev] = time.monotonic()
            m.live = True
            m.enable_epoch += 1
        finally:
            m.cmd_done += 1

    def _do_disable(self, dev: int):
        m = self.motors[dev]
        try:
            m.live = False
            self.bus.set_mode(dev, self.MODE_IDLE)
        finally:
            m.cmd_done += 1

    def _do_estop(self):
        try:
            while True:
                cmd, dev, _ = self.commands.get_nowait()
                if cmd in ("enable", "disable", "zero"):
                    self.motors[dev].cmd_done += 1
        except queue.Empty:
            pass
        for m in self.motors.values():
            m.live = False
        for _ in range(2):
            for dev in self.motors:
                self.bus.set_mode(dev, self.MODE_IDLE)
        for m in self.motors.values():
            m.cmd_done += 1
        self.estop_flag.clear()

    def _do_gains(self, payload: dict):
        self.gains.update(payload)
        for dev, m in self.motors.items():
            if m.live:
                self.bus.write_position_kp(dev, self.gains["kp"])
                self.bus.write_position_kd(dev, self.gains["kd"])
                self.bus.write_torque_limit(dev, self.gains["torque_limit"])

    def _do_ping(self, dev: int | None):
        devs = [dev] if dev is not None else list(self.motors)
        for d in devs:
            m = self.motors[d]
            if m.live:
                continue
            ok = self.bus.ping(d, timeout=0.1)
            m.online = ok
            m.idle_misses = 0 if ok else self.idle_miss_limit
            if ok:
                m.seen = True
                if m.fault in ("NO REPLY", "LOST"):
                    m.fault = None
                raw = self._read_pos(d, 0.05)
                if raw is not None:
                    self._store_measurement(m, raw, None)
            if self.estop_flag.is_set():
                break

    def _do_zero(self, dev: int):
        """Make the current reading URDF zero (idle motors only)."""
        m = self.motors[dev]
        try:
            if m.live:
                return
            raw = self._read_pos(dev, 0.1)
            if raw is None:
                if m.raw is None:
                    m.fault = "NO REPLY"
                    return
                raw = m.raw
            m.spec.offset = m.spec.offset_for_zero_here(raw)
            self._store_measurement(m, raw, None)
            if m.fault == "OUT_OF_RANGE":
                m.fault = None
        finally:
            m.cmd_done += 1

    def run(self):
        period = 1.0 / self.rate_hz
        next_tick = time.monotonic()
        try:
            while self.running:
              try:
                if self.estop_flag.is_set():
                    self._do_estop()
                try:
                    while not self.estop_flag.is_set():
                        cmd, dev, payload = self.commands.get_nowait()
                        if cmd == "enable":
                            self._do_enable(dev)
                        elif cmd == "disable":
                            self._do_disable(dev)
                        elif cmd == "gains":
                            self._do_gains(payload)
                        elif cmd == "ping":
                            self._do_ping(dev)
                        elif cmd == "zero":
                            self._do_zero(dev)
                except queue.Empty:
                    pass
                if self.estop_flag.is_set():
                    continue

                max_step = self.max_speed * period
                active = [m for m in self.motors.values() if m.live]
                for m in active:
                    s = m.spec
                    if m.timeouts <= self.link_flaky:
                        goal = clamp(m.target, m.lo_m, m.hi_m)
                        m.command += clamp(goal - m.command, -max_step, max_step)
                    self.bus.transmit_pdo_2(m.dev, position_target=s.to_actuator(m.command), velocity_target=0.0)
                for m in active:
                    r = self._recv_pdo(m.dev)
                    if r is not None:
                        self._store_measurement(m, r[0], r[1])
                        m.timeouts = 0
                    else:
                        m.timeouts += 1
                        if m.timeouts > self.pdo_loss:
                            m.live = False
                            m.online = False
                            m.fault = "LOST"
                            self.bus.set_mode(m.dev, self.MODE_IDLE)

                now = time.monotonic()
                # keep the firmware watchdog (motor_configuration.json: 1000)
                # fed, so a motor cannot fault out mid-move
                for m in active:
                    if m.live and now - self._last_feed[m.dev] > FEED_PERIOD:
                        self._last_feed[m.dev] = now
                        self.bus.feed(m.dev)
                for m in active:
                    if m.live and now - self._last_error_poll[m.dev] > ERROR_POLL_PERIOD:
                        self._last_error_poll[m.dev] = now
                        err = self.bus._read_parameter_u32(m.dev, self.PARAM_ERROR, timeout=0.05)
                        if err is not None:
                            m.error_code = err

                # idle motors: read their position round-robin so the user can
                # back-drive a joint and watch the sign; silent ones back off
                # Every frame sent to a bus nobody answers pushes the CAN
                # controller's error counter up, and this adapter cannot
                # restart itself from bus-off. So once nothing has answered for
                # a few seconds, look only occasionally instead of at 10 Hz -
                # the window stays open and picks the motor up when it returns.
                silent_for = now - self._last_answer
                self.quiet = silent_for > DEAD_BUS_AFTER
                poll_period = (DEAD_BUS_POLL_PERIOD
                               if silent_for > DEAD_BUS_AFTER and not any(m.live for m in self.motors.values())
                               else IDLE_POLL_PERIOD)
                if now - self._last_idle_poll > poll_period:
                    self._last_idle_poll = now
                    idle = [m for m in self.motors.values() if not m.live]
                    if idle:
                        m = idle[self._idle_idx % len(idle)]
                        self._idle_idx += 1
                        if m.online or now - self._silent_since[m.dev] > IDLE_PING_BACKOFF:
                            raw = self._read_pos(m.dev, self.sdo_timeout)
                            if raw is not None:
                                m.idle_misses = 0
                                self._store_measurement(m, raw, None)
                            else:
                                # One lost or late reply is normal on a shared
                                # bus; only a run of them means the motor is
                                # gone. Flipping on every miss made the
                                # indicator flap while the motor was fine.
                                m.idle_misses += 1
                                if m.idle_misses >= self.idle_miss_limit:
                                    m.online = False
                                    self._silent_since[m.dev] = now

                self.blocked = None
                self._blocked_backoff = 0.1
                next_tick += period
                delay = next_tick - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
                else:
                    next_tick = time.monotonic()
              except Exception as e:  # noqa: BLE001 - see _is_transient
                if not self._is_transient(e):
                    raise
                # nothing is acknowledging: treat every motor as unreachable,
                # let the usual loss counters run, and keep trying
                self.blocked = f"{type(e).__name__}: {str(e)[:60]}"
                for m in self.motors.values():
                    m.online = False
                    if m.live:
                        m.timeouts += 1
                        if m.timeouts > self.pdo_loss:
                            m.live = False
                            m.fault = "LOST"
                # Back off hard. Every frame that reaches a bus nobody answers
                # pushes the controller's error counter up, and this adapter
                # cannot restart itself from bus-off, so retrying at loop rate
                # would be the fastest way to wedge the interface for good.
                self._blocked_backoff = min(2.0, max(0.2, self._blocked_backoff * 2))
                time.sleep(self._blocked_backoff)
                next_tick = time.monotonic()
        except Exception as e:  # noqa: BLE001 - any bus error must still IDLE the motors
            self.dead = f"{type(e).__name__}: {e}"
            print(f"[FATAL] {self.name} died: {self.dead}")
        finally:
            for dev in self.motors:
                try:
                    self.bus.set_mode(dev, self.MODE_IDLE)
                except Exception:
                    pass
            for m in self.motors.values():
                m.live = False
            try:
                self.bus.stop()
            except Exception:
                pass


class Rig:
    """Everything that exists per run: specs, states, one thread per bus."""

    def __init__(self, args):
        self.args = args
        urdf_limits = load_urdf_limits(args.urdf)
        self.urdf_limits = urdf_limits
        self.buses, gains_cfg, self.safety, self.specs = load_config(
            args.config, urdf_limits,
            set(args.ids) if getattr(args, "ids", None) else None,
            dict(getattr(args, "bus", None) or {}))
        self.gains = {
            "kp": clamp_gain("kp", float(args.kp if args.kp is not None else gains_cfg.get("position_kp", 0.7))),
            "kd": clamp_gain("kd", float(args.kd if args.kd is not None else gains_cfg.get("position_kd", 0.03))),
            "torque_limit": clamp_gain("torque_limit", float(
                args.torque_limit if args.torque_limit is not None else gains_cfg.get("torque_limit", 0.5))),
        }
        if args.max_speed is not None:
            self.safety["max_joint_speed"] = float(args.max_speed)
        self.presets = load_srdf_presets(args.srdf, [s.name for s in self.specs])
        self.states = {s.name: MotorState(s) for s in self.specs}
        self.threads: dict[str, BusThread] = {}
        self.bus_status: dict[str, str] = {}
        by_channel: dict[str, list[MotorState]] = {}
        for st in self.states.values():
            by_channel.setdefault(st.spec.channel, []).append(st)
        sim_raw0 = {}
        if args.sim:
            for st in self.states.values():
                # deterministic "arbitrary encoder boot value" so OUT_OF_RANGE and
                # Zero-here can be exercised; --sim-zeroed makes them all 0
                sim_raw0[st.dev] = 0.0 if args.sim_zeroed else ((st.dev * 13.7) % 60.0 - 30.0)
        for channel, motors in by_channel.items():
            ids = [m.dev for m in motors]
            bus, why = open_bus(channel, ids if args.sim else None, sim_raw0, set(args.sim_offline or []))
            if bus is None:
                self.bus_status[channel] = f"NO BUS ({why})"
                for m in motors:
                    m.fault = "NO BUS"
                print(f"[WARN] {channel}: {why}")
                continue
            self.bus_status[channel] = why
            t = BusThread(channel, bus, motors, dict(self.gains), self.safety, args.rate)
            self.threads[channel] = t
        for t in self.threads.values():
            t.start()

    def thread_for(self, st: MotorState) -> BusThread | None:
        return self.threads.get(st.spec.channel)

    def estop_all(self):
        for t in self.threads.values():
            t.trigger_estop()

    def shutdown(self):
        self.estop_all()
        for t in self.threads.values():
            t.running = False
        for t in self.threads.values():
            t.join(timeout=3.0)
            if t.is_alive():
                t.force_idle_all()


# ----------------------------------------------------------------------------
# optional ROS 2 joint_states publisher
# ----------------------------------------------------------------------------

class JointStatePublisher:
    """Publish /joint_states for the WHOLE URDF.

    The GUI drives ten joints, but the v1arm model has 48 movable ones (wrist
    servos and both DexHands). A joint missing from /joint_states gets no TF,
    and RViz then shows RobotModel in error with the hands absent -- so the
    passive joints are published at a fixed pose (zero, which every limit
    admits) alongside the measured ones.
    """

    def __init__(self, rig: Rig, rate_hz: float = 30.0):
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
        self.rclpy = rclpy
        rclpy.init(args=None)
        self.node: Node = rclpy.create_node("bhl_motor_gui")
        self.pub = self.node.create_publisher(JointState, "joint_states", 10)
        self.JointState = JointState
        self.rig = rig
        self.period = 1.0 / rate_hz
        driven = {st.spec.name for st in rig.states.values()}
        self.passive = [n for n in rig.urdf_limits if n not in driven]
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True, name="joint_states")
        self.thread.start()

    def _loop(self):
        while self.running and self.rclpy.ok():
            msg = self.JointState()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            for st in self.rig.states.values():
                msg.name.append(st.spec.name)
                if st.measured is not None:
                    msg.position.append(float(st.measured))
                else:
                    msg.position.append(float(st.command))
                msg.velocity.append(float(st.measured_vel or 0.0))
            for name in self.passive:
                msg.name.append(name)
                msg.position.append(0.0)
                msg.velocity.append(0.0)
            try:
                self.pub.publish(msg)
            except Exception:  # noqa: BLE001 - context torn down during shutdown
                break
            time.sleep(self.period)

    def stop(self):
        self.running = False
        self.thread.join(timeout=1.0)
        try:
            self.node.destroy_node()
            if self.rclpy.ok():
                self.rclpy.shutdown()
        except Exception:
            pass


# ----------------------------------------------------------------------------
# GUI
# ----------------------------------------------------------------------------

def build_gui(rig: Rig, args):
    """The window: a clickable robot diagram, one detail panel, one function bar.

    Layout follows the bench sketch -- the arms are drawn as they are seen
    facing the robot (screen left = the robot's RIGHT arm), every actuator is a
    numbered node coloured by its state, and the panel on the right drives
    whichever node is selected: ID, a Min..Max slider in degrees, a rolling
    Status trace of target vs measured.
    """
    import tkinter as tk
    import tkinter.font as tkfont
    from tkinter import ttk, messagebox
    from collections import deque

    D = math.degrees

    C = {
        "page": "#10151c",
        "panel": "#182029",
        "panel2": "#1e2833",
        "line": "#2b3644",
        "text": "#e7eef6",
        "muted": "#8b9aab",
        "accent": "#4da3ff",
        "ok": "#3ddc84",
        "warn": "#ffb020",
        "danger": "#ff5964",
        "locked": "#6b7683",
        "estop": "#d7263d",
    }
    STATE_COLOR = {
        "ACTIVE": C["ok"], "online": C["warn"], "offline": C["danger"], "LOCKED": C["locked"],
        "busy": C["muted"], "fault": C["danger"], "NO REPLY": C["warn"], "DEAD": C["danger"],
    }

    def joint_state_of(st: MotorState) -> tuple[str, str]:
        """(label, colour) for one motor, in the same order of precedence as the bus logic."""
        thread = rig.thread_for(st)
        if thread is None:
            return "NO BUS", C["danger"]
        if thread.dead is not None:
            return "DEAD", C["danger"]
        if st.live:
            if st.timeouts > thread.link_flaky:
                return "NO REPLY", C["warn"]
            return "ACTIVE", C["ok"]
        if st.busy:
            return "busy", C["muted"]
        if st.fault:
            return st.fault, C["danger"]
        locked = not st.spec.config_enabled and not thread.unlocked
        if locked:
            return "LOCKED", C["locked"]
        return ("online", C["warn"]) if st.online else ("offline", C["danger"])

    root = tk.Tk()
    root.title(f"BHL arm motors - {'SIM' if args.sim else 'LIVE'}")

    # Canvas drawings are laid out in real pixels, but a Tk font size -- points
    # or "pixels" alike -- comes out multiplied by this display's font DPI (on
    # the bench machine roughly 3.7x), which buries the diagram under its own
    # labels. Measure the factor once and ask for sizes through font_px().
    _probe = tkfont.Font(root=root, font=("TkDefaultFont", -40))
    FONT_RATIO = max(0.5, _probe.metrics("linespace") / 40.0)

    def font_px(pixels: float, bold: bool = False, mono: bool = False) -> tuple:
        """A font spec whose rendered line height is about `pixels` pixels."""
        size = -max(4, int(round(pixels / FONT_RATIO)))
        spec = ("TkFixedFont" if mono else "TkDefaultFont", size)
        return spec + ("bold",) if bold else spec

    def px(pixels: float) -> int:
        """Widget geometry, grown the same way this display grows text.

        tk.Scale sizes its trough and its grip in raw pixels while everything
        around it is text that this display renders several times larger than
        asked for; left alone the slider ends up a hairline with a nub on it
        that is genuinely hard to hit. Ask for sizes through here instead.
        """
        return max(1, int(round(pixels * FONT_RATIO)))
    root.configure(bg=C["page"])
    root.minsize(1180, 740)

    style = ttk.Style(root)
    try:
        style.theme_use("clam")
    except tk.TclError:
        pass
    style.configure(".", background=C["panel"], foreground=C["text"], fieldbackground=C["panel2"],
                    bordercolor=C["line"], lightcolor=C["panel"], darkcolor=C["panel"])
    style.configure("TFrame", background=C["panel"])
    style.configure("Page.TFrame", background=C["page"])
    style.configure("Card.TFrame", background=C["panel"], relief="flat")
    style.configure("TLabel", background=C["panel"], foreground=C["text"], font=("TkDefaultFont", 10))
    style.configure("Muted.TLabel", background=C["panel"], foreground=C["muted"], font=("TkDefaultFont", 9))
    style.configure("Head.TLabel", background=C["panel"], foreground=C["text"], font=("TkDefaultFont", 12, "bold"))
    style.configure("Big.TLabel", background=C["panel"], foreground=C["accent"], font=("TkDefaultFont", 20, "bold"))
    style.configure("Mono.TLabel", background=C["panel"], foreground=C["text"], font=("TkFixedFont", 9))
    style.configure("TButton", background=C["panel2"], foreground=C["text"], borderwidth=0, padding=(10, 5),
                    font=("TkDefaultFont", 10))
    style.map("TButton", background=[("active", C["line"]), ("disabled", C["panel"])],
              foreground=[("disabled", C["muted"])])
    style.configure("Accent.TButton", background=C["accent"], foreground="#0b1220",
                    font=("TkDefaultFont", 10, "bold"))
    # the step row sits under the slider and must not widen the panel enough to
    # squeeze the diagram, so it gets a tighter button than the rest
    style.configure("Step.TButton", background=C["panel2"], foreground=C["text"], borderwidth=0,
                    padding=(2, 4), font=("TkDefaultFont", 8))
    style.map("Step.TButton", background=[("active", C["line"]), ("disabled", C["panel"])],
              foreground=[("disabled", C["muted"])])
    style.map("Accent.TButton", background=[("active", "#6fb6ff"), ("disabled", C["line"])])
    style.configure("TCheckbutton", background=C["panel"], foreground=C["text"],
                    indicatorsize=px(10), indicatormargin=px(3))
    style.map("TCheckbutton", background=[("active", C["panel"])])
    style.configure("TEntry", fieldbackground=C["panel2"], foreground=C["text"], insertcolor=C["text"],
                    borderwidth=0, padding=3)
    style.configure("TSeparator", background=C["line"])

    def card(parent, **grid) -> ttk.Frame:
        holder = tk.Frame(parent, bg=C["line"], padx=1, pady=1)
        holder.grid(**grid)
        inner = ttk.Frame(holder, style="Card.TFrame", padding=10)
        inner.pack(fill="both", expand=True)
        return inner

    # ---------------- robot diagram ----------------

    class Diagram:
        """Canvas of the two arms; one node per actuator, click to select.

        The drawing is laid out from the canvas's CURRENT size and from the
        MEASURED height of the text, never from the requested font size: this
        display renders a font several times larger than asked for, which had
        the labels sitting on top of each other and running off the canvas.
        Fonts are chosen first, then the geometry is fitted around them; if a
        label cannot fit it is abbreviated, and failing that dropped, so the
        diagram stays readable in a narrow window instead of turning to soup.
        """

        ROWS = ["shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow_pitch", "elbow_roll"]
        SHORT = {"shoulder_pitch": "sh pitch", "shoulder_roll": "sh roll", "shoulder_yaw": "sh yaw",
                 "elbow_pitch": "el pitch", "elbow_roll": "el roll"}

        def __init__(self, parent, on_select):
            self.on_select = on_select
            self.canvas = tk.Canvas(parent, width=520, height=520, bg=C["panel"], highlightthickness=0)
            self.canvas.pack(fill="both", expand=True)
            self.nodes: dict[str, dict] = {}
            self.selected: str | None = None
            self.bus_lbl: dict[str, int] = {}
            self._size = (0, 0)
            self.canvas.bind("<Configure>", self._on_resize)
            self.canvas.bind("<Button-1>", self._on_click)

        # --- fonts first: pick the largest that fits, and report what it measures ---
        def _fit(self, target_px: float, texts, max_w: float, max_h: float | None = None):
            """(spec, line_height, widest, fits) for the largest font that fits."""
            size = max(target_px, 4.0)
            while True:
                spec = font_px(size)
                fo = tkfont.Font(root=self.canvas, font=spec)
                lh = fo.metrics("linespace")
                widest = max((fo.measure(t) for t in texts), default=0)
                fits = widest <= max_w and (max_h is None or lh <= max_h)
                if fits or size <= 4.0:
                    return spec, lh, widest, fits
                size *= 0.85

        def _on_resize(self, event):
            if (event.width, event.height) == self._size or event.width < 60 or event.height < 60:
                return
            self._size = (event.width, event.height)
            self._redraw(event.width, event.height)

        def _redraw(self, w: int, h: int):
            cv = self.canvas
            cv.delete("all")
            self.nodes.clear()
            self.bus_lbl.clear()
            pad = max(4, int(min(w, h) * 0.02))
            cx = w / 2.0

            # ---- header ----
            base = min(w, h) * 0.055
            hint_spec, hint_h, _wd, hint_ok = self._fit(base * 0.8, ["viewed facing the robot"], w - 2 * pad)
            y = pad
            if hint_ok:
                cv.create_text(cx, y + hint_h / 2, text="viewed facing the robot",
                               fill=C["muted"], font=hint_spec)
                y += hint_h + pad * 0.5
            col_w = w * 0.46                       # room each arm column may use for text
            titles = ["RIGHT ARM", "LEFT ARM"]
            t_spec, t_h, _w2, t_ok = self._fit(base, titles, col_w)
            if not t_ok:
                titles = ["R", "L"]
                t_spec, t_h, _w2, t_ok = self._fit(base, titles, col_w)
            bus_texts = [f"{ch}: {st}" for ch, st in rig.bus_status.items()] or ["can0"]
            b_spec, b_h, _w3, b_ok = self._fit(base * 0.75, bus_texts, col_w)
            header_bottom = y + t_h + (b_h if b_ok else 0) + pad

            # ---- vertical budget: rows between the header and the hand blocks ----
            foot_spec, foot_h, _w4, foot_ok = self._fit(base * 0.72,
                                                       ["wrist and hand joints are not driven here"],
                                                       w - 2 * pad)
            r = min(min(w, h) * 0.05, w * 0.07)
            hand_h = r * 2.0
            rows_top = header_bottom + r * 1.6
            rows_bot = h - (foot_h + pad if foot_ok else pad) - hand_h - pad * 2 - r
            if rows_bot - rows_top < r * 2:        # desperate window: shrink the nodes
                r = max(6.0, (rows_bot - rows_top) / 8.0)
                rows_top = header_bottom + r * 1.6
                rows_bot = h - (foot_h + pad if foot_ok else pad) - r * 2 - pad * 2 - r
            row_y = [rows_top + (rows_bot - rows_top) * i / 4.0 for i in range(5)]
            pitch = (rows_bot - rows_top) / 4.0
            r = min(r, pitch * 0.40)
            col = {"right": w * 0.27, "left": w * 0.73}
            tw = min(r * 1.9, (col["left"] - col["right"]) / 2 - r * 1.6)

            # ---- labels: two lines per node if they fit, else one, else none ----
            gap = r * 0.5
            avail = min(col["right"] - r - gap - pad, w - col["left"] - r - gap - pad)
            names = [self.ROWS, [self.SHORT[k] for k in self.ROWS], None]
            value_probe = ["-000.0 deg  !"]
            name_spec = val_spec = None
            name_h = val_h = 0.0
            for variant in names:
                probe = list(variant) + value_probe if variant else value_probe
                spec, lh, _wid, fits = self._fit(base * 0.82, probe, avail,
                                                 max_h=pitch * (0.42 if variant else 0.85))
                if fits:
                    name_spec = spec if variant else None
                    val_spec, val_h, name_h = spec, lh, lh
                    self._names = {k: (variant[i] if variant else "") for i, k in enumerate(self.ROWS)} \
                        if variant else {k: "" for k in self.ROWS}
                    break
            else:
                spec, lh, _wid, fits = self._fit(4, value_probe, avail, max_h=pitch * 0.85)
                name_spec, val_spec, val_h = None, (spec if fits else None), lh
                self._names = {k: "" for k in self.ROWS}

            # ---- draw ----
            if t_ok:
                for side, title in zip(("right", "left"), titles):
                    x = col[side]
                    cv.create_text(x, y + t_h / 2, text=title, fill=C["text"], font=t_spec)
                    if b_ok:
                        self.bus_lbl[side] = cv.create_text(x, y + t_h + b_h / 2, text="",
                                                            fill=C["muted"], font=b_spec)
            if foot_ok:
                cv.create_text(cx, h - pad - foot_h / 2,
                               text="wrist and hand joints are not driven here",
                               fill=C["muted"], font=foot_spec)

            head_r = r * 1.25
            cv.create_oval(cx - head_r, row_y[0] - r - head_r * 2.2, cx + head_r, row_y[0] - r - pad * 0.2,
                           fill=C["panel2"], outline=C["line"], width=2)
            cv.create_rectangle(cx - tw, row_y[0] - r, cx + tw, row_y[-1] + r,
                                fill=C["panel2"], outline=C["line"], width=2)
            base_spec, base_h, _w5, base_ok = self._fit(base * 0.75, ["base"], tw * 1.8, max_h=pitch)
            if base_ok:
                cv.create_text(cx, (row_y[0] + row_y[-1]) / 2, text="base", fill=C["muted"], font=base_spec)
            bone = max(2, int(r * 0.22))
            cv.create_line(cx, row_y[-1] + r, cx, row_y[-1] + r + hand_h * 0.5, fill=C["line"], width=bone)
            cv.create_rectangle(cx - tw * 0.75, row_y[-1] + r + hand_h * 0.5,
                                cx + tw * 0.75, row_y[-1] + r + hand_h,
                                fill=C["panel2"], outline=C["line"], width=2)

            hands_top = row_y[-1] + r + hand_h * 0.35
            for side, x in col.items():
                torso_x = cx - tw if side == "right" else cx + tw
                cv.create_line(torso_x, row_y[0], x, row_y[0], fill=C["line"], width=bone)
                for i in range(4):
                    cv.create_line(x, row_y[i], x, row_y[i + 1], fill=C["line"], width=bone)
                cv.create_line(x, row_y[-1], x, hands_top, fill=C["line"], width=bone)
                cv.create_rectangle(x - r * 1.15, hands_top, x + r * 1.15, hands_top + hand_h * 0.9,
                                    fill=C["panel2"], outline=C["line"], width=2)
                hspec, hh, _w6, hok = self._fit(base * 0.7, ["DexHand"], r * 2.3, max_h=hand_h * 0.8)
                if hok:
                    cv.create_text(x, hands_top + hand_h * 0.45, text="DexHand", fill=C["muted"], font=hspec)

            num_spec, num_h, num_w, num_ok = self._fit(r * 1.1, ["10"], r * 1.5, max_h=r * 1.8)
            for st in rig.states.values():
                side = st.spec.bus_key
                if side not in col:
                    continue
                key = st.spec.name.replace("arm_", "").replace(f"{side}_", "").replace("_joint", "")
                if key not in self.ROWS:
                    continue
                x, yy = col[side], row_y[self.ROWS.index(key)]
                ring = cv.create_oval(x - r - r * 0.22, yy - r - r * 0.22, x + r + r * 0.22, yy + r + r * 0.22,
                                      outline="", width=max(2, int(r * 0.16)))
                circle = cv.create_oval(x - r, yy - r, x + r, yy + r, fill=C["locked"],
                                        outline=C["page"], width=max(1, int(r * 0.1)))
                if num_ok:
                    cv.create_text(x, yy, text=str(st.dev), fill="#0b1220", font=num_spec)
                anchor = "e" if side == "right" else "w"
                lx = x - r - gap if side == "right" else x + r + gap
                name = value = None
                if name_spec is not None and self._names[key]:
                    name = cv.create_text(lx, yy - name_h * 0.55, text=self._names[key], fill=C["text"],
                                          anchor=anchor, font=name_spec)
                    vy = yy + val_h * 0.55
                else:
                    vy = yy
                if val_spec is not None:
                    value = cv.create_text(lx, vy, text="--", fill=C["muted"], anchor=anchor, font=val_spec)
                self.nodes[st.spec.name] = {"st": st, "x": x, "y": yy, "r": r, "ring": ring,
                                            "circle": circle, "name": name, "value": value}
            self.refresh()

        def _on_click(self, event):
            best, best_d, hit = None, 1e9, 0.0
            for jname, n in self.nodes.items():
                d = math.hypot(event.x - n["x"], event.y - n["y"])
                if d < best_d:
                    best, best_d, hit = jname, d, n["r"] * 1.8
            if best is not None and best_d <= hit:
                self.on_select(best)

        def select(self, joint_name: str):
            self.selected = joint_name

        def refresh(self):
            cv = self.canvas
            for side, item in self.bus_lbl.items():
                ch = rig.buses.get(side, "?")
                thread = rig.threads.get(ch)
                state = rig.bus_status.get(ch, "n/a")
                if thread is not None and thread.blocked:
                    state = "NO ACK"
                elif thread is not None and thread.quiet:
                    state = "no answer"
                cv.itemconfigure(item, text=f"{ch}: {state}")
            for jname, n in self.nodes.items():
                st = n["st"]
                _label, colour = joint_state_of(st)
                cv.itemconfigure(n["circle"], fill=colour)
                cv.itemconfigure(n["ring"], outline=C["accent"] if jname == self.selected else "")
                if n["value"] is None:
                    continue
                if st.measured is None:
                    text, fg = "--", C["muted"]
                else:
                    inside = st.spec.lower <= st.measured <= st.spec.upper
                    text = f"{D(st.measured):+.1f} deg" + ("" if inside else "  !")
                    fg = C["text"] if inside else C["danger"]
                cv.itemconfigure(n["value"], text=text, fill=fg)

    # ---------------- status trace ----------------

    class Trace:
        """Rolling target-vs-measured plot for the selected joint, sized to its canvas."""

        PAD = 8

        def __init__(self, parent):
            self.canvas = tk.Canvas(parent, width=380, height=200, bg=C["panel2"], highlightthickness=0)
            self.canvas.pack(fill="both", expand=True)
            self.history: dict[str, deque] = {}
            self.joint: str | None = None
            self.width = 380
            self.height = 200
            self.canvas.bind("<Configure>", self._on_resize)

        def _on_resize(self, event):
            self.width, self.height = event.width, event.height
            for h in self.history.values():
                # keep one sample per pixel column, no more
                while len(h) > max(16, self.width):
                    h.popleft()

        def show(self, joint_name: str):
            self.joint = joint_name

        def sample(self):
            keep = max(16, self.width)
            for st in rig.states.values():
                h = self.history.setdefault(st.spec.name, deque(maxlen=4096))
                h.append((st.command, st.measured if st.measured is not None else st.command))
                while len(h) > keep:
                    h.popleft()

        def refresh(self):
            cv = self.canvas
            cv.delete("all")
            if self.joint is None:
                return
            w, h = self.width, self.height
            f = clamp(min(w / 32, h / 10), 9, 26)
            st = rig.states[self.joint]
            lo, hi = st.spec.lower, st.spec.upper
            span = (hi - lo) or 1.0
            top, bot = self.PAD + f * 1.6, h - self.PAD - f
            gutter = self.PAD + f * 3.2

            def y_of(v: float) -> float:
                return bot - (clamp(v, lo, hi) - lo) / span * (bot - top)

            for frac in (0.0, 0.25, 0.5, 0.75, 1.0):
                y = bot - frac * (bot - top)
                val = D(lo + frac * span)
                cv.create_line(gutter, y, w - self.PAD, y, fill=C["line"],
                               dash=(2, 4) if frac not in (0.0, 1.0) else ())
                cv.create_text(gutter - 6, y, text=f"{val:+.0f}", fill=C["muted"], anchor="e",
                               font=font_px(f * 0.85, mono=True))
            hist = self.history.get(self.joint)
            if hist and len(hist) > 1:
                usable = w - self.PAD - gutter
                step = usable / max(len(hist) - 1, 1)
                cmd_pts, meas_pts = [], []
                for i, (cmd, meas) in enumerate(hist):
                    x = gutter + i * step
                    cmd_pts += [x, y_of(cmd)]
                    meas_pts += [x, y_of(meas)]
                cv.create_line(*cmd_pts, fill=C["accent"], width=2)
                cv.create_line(*meas_pts, fill=C["ok"], width=2)
            cv.create_text(w - self.PAD, self.PAD, text="target", fill=C["accent"], anchor="ne",
                           font=font_px(f))
            cv.create_text(w - self.PAD - f * 5, self.PAD, text="measured", fill=C["ok"], anchor="ne",
                           font=font_px(f))
            cv.create_text(gutter, self.PAD, text="deg", fill=C["muted"], anchor="nw",
                           font=font_px(f * 0.85))

    # ---------------- window ----------------

    class App:
        def __init__(self):
            self._dead_notified = set()
            self._blocked_shown: list[str] = []
            self._quiet_shown: list[str] = []
            self._seen_epoch: dict[str, int] = {}
            self._seen_fault: dict[str, str | None] = {}
            self._tick = 0
            root.columnconfigure(0, weight=3)
            root.columnconfigure(1, weight=2)
            root.rowconfigure(1, weight=1)

            header = tk.Frame(root, bg=C["page"])
            header.grid(row=0, column=0, columnspan=2, sticky="ew", padx=14, pady=(12, 6))
            tk.Label(header, text="BHL ARM MOTORS", bg=C["page"], fg=C["text"],
                     font=("TkDefaultFont", 15, "bold")).pack(side="left")
            tk.Label(header, text=f"  {'simulated bus' if args.sim else 'live CAN'}  ",
                     bg=C["page"], fg=C["accent"], font=("TkDefaultFont", 10)).pack(side="left", padx=(8, 0))
            self.mode_lbl = tk.Label(header, text="", bg=C["page"], fg=C["muted"], font=("TkFixedFont", 9))
            self.mode_lbl.pack(side="right")

            left = card(root, row=1, column=0, sticky="nsew", padx=(14, 7), pady=6)
            self.diagram = Diagram(left, self.select)

            right = tk.Frame(root, bg=C["page"])
            right.grid(row=1, column=1, sticky="nsew", padx=(7, 14), pady=6)
            right.columnconfigure(0, weight=1)
            right.rowconfigure(0, weight=0)
            right.rowconfigure(1, weight=1)

            detail = card(right, row=0, column=0, sticky="ew")
            detail.columnconfigure(1, weight=1)
            self.id_lbl = ttk.Label(detail, text="ID: -", style="Big.TLabel")
            self.id_lbl.grid(row=0, column=0, sticky="w")
            self.name_lbl = ttk.Label(detail, text="", style="Head.TLabel")
            self.name_lbl.grid(row=0, column=1, sticky="w", padx=(10, 0))
            self.state_lbl = tk.Label(detail, text="", bg=C["panel"], fg=C["muted"],
                                      font=("TkDefaultFont", 11, "bold"))
            self.state_lbl.grid(row=0, column=2, sticky="e")

            self.enable_var = tk.BooleanVar(value=False)
            row = ttk.Frame(detail)
            row.grid(row=1, column=0, columnspan=3, sticky="ew", pady=(8, 4))
            self.enable_chk = ttk.Checkbutton(row, text="Enable (torque on)", variable=self.enable_var,
                                              command=self.on_enable)
            self.enable_chk.pack(side="left")
            ttk.Button(row, text="Hold", command=self.on_hold).pack(side="right", padx=2)
            ttk.Button(row, text="Zero here", command=self.on_zero).pack(side="right", padx=2)
            ttk.Button(row, text="Ping", command=self.on_ping).pack(side="right", padx=2)

            # a grip and a trough sized in px() so they stay a real mouse
            # target on a display that renders text several times larger
            # bg is also the grip's colour in tk.Scale: keep it clear of the
            # trough colour or the handle disappears into the trough
            self.slider = tk.Scale(detail, from_=-1, to=1, resolution=STEP_FINE, orient="horizontal",
                                  showvalue=True, command=self.on_slider, bg=C["panel2"], fg=C["text"],
                                  troughcolor=C["page"], highlightthickness=0, bd=0,
                                  activebackground=C["accent"], font=("TkFixedFont", 9),
                                  length=px(240), width=px(17), sliderlength=px(36),
                                  sliderrelief="raised", takefocus=0)
            self.slider.grid(row=2, column=0, columnspan=3, sticky="ew")
            for seq, deg in (("<Button-4>", +STEP_COARSE), ("<Button-5>", -STEP_COARSE),
                             ("<Shift-Button-4>", +STEP_FINE), ("<Shift-Button-5>", -STEP_FINE)):
                self.slider.bind(seq, lambda _e, d=deg: self.wheel(d))
            self.slider.bind("<MouseWheel>",   # same wheel, Windows/macOS style event
                             lambda e: self.wheel(STEP_COARSE if e.delta > 0 else -STEP_COARSE))
            minmax = ttk.Frame(detail)
            minmax.grid(row=3, column=0, columnspan=3, sticky="ew")
            self.min_lbl = ttk.Label(minmax, text="Min", style="Muted.TLabel")
            self.min_lbl.pack(side="left")
            ttk.Label(minmax, text="degrees", style="Muted.TLabel").pack(side="left", expand=True)
            self.max_lbl = ttk.Label(minmax, text="Max", style="Muted.TLabel")
            self.max_lbl.pack(side="right")

            # dragging is never the only way to reach an angle: fixed steps on
            # both sides and a box to type the exact number into
            steps = ttk.Frame(detail)
            steps.grid(row=4, column=0, columnspan=3, sticky="ew", pady=(8, 0))
            self.step_btns = []
            for label, deg in (("-5", -5.0), ("-1", -1.0), (f"-{STEP_FINE:g}", -STEP_FINE)):
                b = ttk.Button(steps, text=label, width=4, style="Step.TButton",
                               command=lambda d=deg: self.nudge(d))
                b.pack(side="left", padx=2)
                self.step_btns.append(b)
            for label, deg in (("+5", 5.0), ("+1", 1.0), (f"+{STEP_FINE:g}", STEP_FINE)):
                b = ttk.Button(steps, text=label, width=4, style="Step.TButton",
                               command=lambda d=deg: self.nudge(d))
                b.pack(side="right", padx=2)
                self.step_btns.append(b)
            goto = ttk.Frame(steps)          # packed last so it centres in what is left
            goto.pack(side="left", expand=True)
            self.goto_var = tk.StringVar()
            self.goto_entry = ttk.Entry(goto, textvariable=self.goto_var, width=6, justify="right")
            self.goto_entry.pack(side="left")
            self.goto_entry.bind("<Return>", lambda _e: self.on_goto())
            self.goto_btn = ttk.Button(goto, text="Go", width=3, style="Step.TButton", command=self.on_goto)
            self.goto_btn.pack(side="left", padx=(4, 0))
            self.zero_btn = ttk.Button(goto, text="to 0", width=5, style="Step.TButton",
                                       command=self.on_goto_zero)
            self.zero_btn.pack(side="left", padx=(8, 0))
            self._drive_state = None
            self.set_drive_state("disabled")   # nothing drives a motor that is not enabled

            self.read_lbl = ttk.Label(detail, text="", style="Mono.TLabel", justify="left")
            self.read_lbl.grid(row=5, column=0, columnspan=3, sticky="w", pady=(8, 0))

            status = card(right, row=1, column=0, sticky="nsew", pady=(12, 0))
            ttk.Label(status, text="Status", style="Head.TLabel").pack(anchor="w", pady=(0, 6))
            self.trace = Trace(status)

            bar = card(root, row=2, column=0, columnspan=2, sticky="ew", padx=14, pady=(6, 12))
            line1 = ttk.Frame(bar)
            line1.pack(fill="x")
            self.estop_btn = tk.Button(line1, text="E-STOP", bg=C["estop"], fg="white", bd=0,
                                       activebackground="#ff6b7a", activeforeground="white",
                                       font=("TkDefaultFont", 13, "bold"), width=11, command=self.on_estop)
            self.estop_btn.pack(side="left", padx=(0, 12), ipady=4)
            for text, cmd in (("Ping all", self.on_ping_all), ("Disable all", self.on_disable_all),
                              ("Enable arm L", lambda: self.on_enable_arm("left")),
                              ("Enable arm R", lambda: self.on_enable_arm("right"))):
                ttk.Button(line1, text=text, command=cmd).pack(side="left", padx=3)
            ttk.Separator(line1, orient="vertical").pack(side="left", fill="y", padx=10)
            for name in sorted(rig.presets):
                for side_key, label in (("left", "L"), ("right", "R")):
                    ttk.Button(line1, text=f"{name} {label}", width=8,
                               command=lambda n=name, k=side_key: self.on_preset(n, k)).pack(side="left", padx=2)
            # URDF zero is not an SRDF preset but it is the pose the whole rig
            # is defined against, so it belongs beside them
            for side_key, label in (("left", "L"), ("right", "R")):
                ttk.Button(line1, text=f"zero {label}", width=8,
                           command=lambda k=side_key: self.on_zero_arm(k)).pack(side="left", padx=2)
            ttk.Separator(line1, orient="vertical").pack(side="left", fill="y", padx=10)
            ttk.Button(line1, text="Save offsets", command=self.on_save_offsets).pack(side="left", padx=3)

            line2 = ttk.Frame(bar)
            line2.pack(fill="x", pady=(8, 0))
            self.unlock_var = tk.BooleanVar(value=args.unlock)
            ttk.Checkbutton(line2, text="unlock yaml-disabled joints", variable=self.unlock_var,
                            command=self.on_unlock).pack(side="left")
            self.on_unlock()
            ttk.Separator(line2, orient="vertical").pack(side="left", fill="y", padx=10)
            ttk.Label(line2, text="kp").pack(side="left", padx=(0, 3))
            self.kp_var = tk.StringVar(value=str(rig.gains["kp"]))
            ttk.Entry(line2, textvariable=self.kp_var, width=6).pack(side="left")
            ttk.Label(line2, text="kd").pack(side="left", padx=(8, 3))
            self.kd_var = tk.StringVar(value=str(rig.gains["kd"]))
            ttk.Entry(line2, textvariable=self.kd_var, width=6).pack(side="left")
            ttk.Label(line2, text="torque").pack(side="left", padx=(8, 3))
            self.tq_var = tk.StringVar(value=str(rig.gains["torque_limit"]))
            ttk.Entry(line2, textvariable=self.tq_var, width=6).pack(side="left")
            ttk.Button(line2, text="Apply gains", style="Accent.TButton",
                       command=self.on_gains).pack(side="left", padx=8)
            ttk.Label(line2, text="max speed [rad/s]").pack(side="left", padx=(12, 4))
            self.speed = tk.Scale(line2, from_=SPEED_LIMITS[0], to=SPEED_LIMITS[1], resolution=0.05,
                                  orient="horizontal", length=px(150), showvalue=True, command=self.on_speed,
                                  bg=C["panel"], fg=C["text"], troughcolor=C["panel2"], highlightthickness=0,
                                  bd=0, activebackground=C["accent"], font=("TkFixedFont", 8),
                                  width=px(10), sliderlength=px(22), sliderrelief="raised")
            self.speed.set(float(rig.safety.get("max_joint_speed", 1.0)))
            self.speed.pack(side="left")

            self.banner = tk.Label(root, text="", anchor="w", justify="left", bg=C["page"], fg=C["muted"],
                                   font=("TkDefaultFont", 9))
            self.banner.grid(row=3, column=0, columnspan=2, sticky="ew", padx=14, pady=(0, 8))
            self.set_banner("click a joint to select it  |  drag / wheel / arrows / step buttons / type an angle"
                            f"  |  wheel {STEP_COARSE:g} deg, shift+wheel {STEP_FINE:g} deg  |  "
                            "0 or Home = back to URDF zero  |  "
                            f"targets stay inside the URDF limits  |  config: {args.config.name}", C["muted"])

            self.selected = next(iter(rig.states))
            self.select(self.selected)
            root.bind("<Escape>", lambda _e: self.on_estop())
            for seq, deg in (("<Up>", +STEP_COARSE), ("<Right>", +STEP_COARSE),
                             ("<Down>", -STEP_COARSE), ("<Left>", -STEP_COARSE),
                             ("<Shift-Up>", +5.0), ("<Shift-Right>", +5.0),
                             ("<Shift-Down>", -5.0), ("<Shift-Left>", -5.0)):
                root.bind(seq, lambda e, d=deg: self.key_nudge(e, d))
            for seq in ("<Home>", "<Key-0>"):
                root.bind(seq, self.key_zero)
            root.protocol("WM_DELETE_WINDOW", self.on_close)
            self.refresh()

        # --- helpers ---
        def set_banner(self, text: str, fg: str):
            self.banner.configure(text=text, fg=fg)

        @property
        def st(self) -> MotorState:
            return rig.states[self.selected]

        def select(self, joint_name: str):
            self.selected = joint_name
            st = self.st
            self.diagram.select(joint_name)
            self.trace.show(joint_name)
            self.id_lbl.configure(text=f"ID: {st.dev}")
            self.name_lbl.configure(text=st.spec.name.replace("arm_", "").replace("_joint", ""))
            lo, hi = D(st.spec.lower), D(st.spec.upper)
            self.slider.configure(from_=round(lo, 1), to=round(hi, 1))
            self.min_lbl.configure(text=f"Min {lo:+.0f}")
            self.max_lbl.configure(text=f"Max {hi:+.0f}")
            self._seen_epoch[joint_name] = st.enable_epoch - 1   # force a slider resync
            self.goto_var.set(f"{D(st.target):.1f}")
            self.enable_chk.configure(state="normal" if rig.thread_for(st) else "disabled")

        def nudge(self, deg: float):
            st = self.st
            if st.live:
                st.target = clamp(st.target + math.radians(deg), st.spec.lower, st.spec.upper)
                self.slider.set(D(st.target))

        def key_nudge(self, event, deg: float):
            """Arrow keys drive the joint, except inside a text box.

            The binding is on the toplevel, so it also sees the keys the entry
            boxes (go-to, kp/kd/torque) already used; `event.widget` is what
            the key was actually delivered to.
            """
            if isinstance(event.widget, (tk.Entry, ttk.Entry)):
                return
            self.nudge(deg)

        def key_zero(self, event):
            """Home / 0 sends the joint to URDF zero, except while typing a number."""
            if isinstance(event.widget, (tk.Entry, ttk.Entry)):
                return
            self.on_goto_zero()

        def wheel(self, deg: float) -> str:
            self.nudge(deg)
            return "break"      # the wheel must not also scroll the Scale itself

        def set_drive_state(self, state: str):
            """Slider, step buttons and the go-to box are live together."""
            if self._drive_state == state:
                return
            self._drive_state = state
            self.slider.configure(state=state)
            for w in (*self.step_btns, self.goto_entry, self.goto_btn, self.zero_btn):
                w.configure(state=state)

        # --- callbacks ---
        def on_enable(self):
            st = self.st
            thread = rig.thread_for(st)
            if thread is None:
                return
            thread.request_motor_cmd("enable" if self.enable_var.get() else "disable", st.dev)

        def on_slider(self, value: str):
            st = self.st
            if st.live:
                st.target = math.radians(float(value))

        def go_to(self, asked: float, what: str = "target"):
            """Send the selected joint to an absolute angle [deg].

            The one place a whole-number goal is set, so the clamp, the slider
            resync and the "how far is it about to travel" warning cannot drift
            apart between the typed box, the 0 button and anything added later.
            """
            st = self.st
            if not st.live:
                self.set_banner(f"  {what} needs an ACTIVE joint - Enable it first", C["warn"])
                return
            deg = D(clamp(math.radians(asked), st.spec.lower, st.spec.upper))
            travel = abs(deg - D(st.measured if st.measured is not None else st.target))
            st.target = math.radians(deg)
            self.slider.set(deg)
            self.goto_var.set(f"{deg:.1f}")
            if abs(deg - asked) > 1e-6:
                self.set_banner(f"  {asked:+.1f} deg is outside the URDF limit - clamped to {deg:+.1f} deg",
                                C["warn"])
            else:
                # a long move is the one that surprises an operator, so give the
                # distance and the speed it will ramp at rather than just "ok"
                self.set_banner(f"  {what} {deg:+.1f} deg, about {travel:.1f} deg to travel "
                                f"at {self.speed.get()} rad/s (Esc = E-STOP)",
                                C["warn"] if travel > 20.0 else C["muted"])

        def on_goto(self):
            try:
                asked = float(self.goto_var.get())
            except ValueError:
                self.set_banner("  Go to: type the target angle in degrees", C["warn"])
                return
            self.go_to(asked, "target")

        def on_goto_zero(self):
            """Straight back to URDF zero: the pose everything else is measured from."""
            self.go_to(0.0, "URDF zero")

        def on_zero_arm(self, side_key: str):
            """URDF zero for every ACTIVE joint on one arm."""
            moved = [st for st in rig.states.values() if st.spec.bus_key == side_key and st.live]
            for st in moved:
                st.target = clamp(0.0, st.spec.lower, st.spec.upper)
            if self.selected in {st.spec.name for st in moved}:
                self.slider.set(D(self.st.target))
            self.set_banner(f"  URDF zero sent to {len(moved)} active joint(s) on the {side_key} arm"
                            f"  (Esc = E-STOP)", C["warn"] if moved else C["muted"])

        def on_hold(self):
            st = self.st
            if st.live and st.measured is not None:
                st.target = st.measured
                self.slider.set(D(st.measured))

        def on_zero(self):
            st = self.st
            thread = rig.thread_for(st)
            if thread is None:
                self.set_banner(f"  {st.spec.short}: no bus for this joint, nothing to zero", C["warn"])
                return
            if st.live:
                # the bus thread drops a zero request from a live motor; say so
                # rather than looking like the button did nothing
                self.set_banner("  Zero here only works with the torque OFF - untick Enable first", C["warn"])
                return
            if not messagebox.askokcancel(
                    "Zero here", f"{st.spec.name}\n\nthe current reading becomes URDF zero (offset).\n"
                                 f"Only do this with the joint physically at its URDF zero pose.\n\n"
                                 f"'Save offsets' afterwards to keep it."):
                return
            thread.request_motor_cmd("zero", st.dev)

        def on_ping(self):
            thread = rig.thread_for(self.st)
            if thread:
                thread.request("ping", self.st.dev)

        def on_estop(self):
            rig.estop_all()
            self.set_banner("  E-STOP: every motor was sent IDLE. Re-enable deliberately.", C["danger"])

        def on_ping_all(self):
            for t in rig.threads.values():
                t.request("ping", None)

        def on_disable_all(self):
            for st in rig.states.values():
                t = rig.thread_for(st)
                if t and (st.live or st.busy):
                    t.request_motor_cmd("disable", st.dev)

        def on_enable_arm(self, side_key: str):
            for st in rig.states.values():
                t = rig.thread_for(st)
                if t and st.spec.bus_key == side_key and not st.live and not st.busy:
                    t.request_motor_cmd("enable", st.dev)

        def on_preset(self, name: str, side_key: str):
            values = rig.presets.get(name, {})
            moved = 0
            for st in rig.states.values():
                if st.spec.bus_key == side_key and st.live and st.spec.name in values:
                    st.target = clamp(values[st.spec.name], st.spec.lower, st.spec.upper)
                    moved += 1
            self.set_banner(f"  preset '{name}' applied to {moved} active joint(s) on the {side_key} arm",
                            C["muted"] if moved else C["warn"])

        def on_gains(self):
            try:
                raw = {"kp": float(self.kp_var.get()), "kd": float(self.kd_var.get()),
                       "torque_limit": float(self.tq_var.get())}
            except ValueError:
                self.set_banner("  gains must be numbers", C["warn"])
                return
            payload = {k: clamp_gain(k, v) for k, v in raw.items()}
            self.kp_var.set(str(payload["kp"]))
            self.kd_var.set(str(payload["kd"]))
            self.tq_var.set(str(payload["torque_limit"]))
            rig.gains.update(payload)
            for t in rig.threads.values():
                t.request("gains", payload=dict(payload))
            self.set_banner(f"  gains applied: {payload}", C["muted"])

        def on_speed(self, value: str):
            v = clamp(float(value), *SPEED_LIMITS)
            for t in rig.threads.values():
                t.max_speed = v

        def on_unlock(self):
            for t in rig.threads.values():
                t.unlocked = bool(self.unlock_var.get())

        def on_save_offsets(self):
            offsets = {st.spec.name: st.spec.offset for st in rig.states.values()}
            if not messagebox.askokcancel("Save offsets", f"write offset: values into\n{args.config}\n(.bak kept)?"):
                return
            try:
                backup = save_offsets(args.config, offsets)
            except Exception as e:  # noqa: BLE001
                messagebox.showerror("Save offsets", str(e))
                return
            self.set_banner(f"  offsets saved to {args.config.name} (backup {backup.name})", C["ok"])

        # --- periodic ---
        def refresh(self):
            self._tick += 1
            quiet = [ch for ch, t in rig.threads.items() if t.quiet and not t.blocked]
            if quiet != self._quiet_shown:
                self._quiet_shown = quiet
                if quiet:
                    self.set_banner(
                        f"  {', '.join(quiet)}: no motor has answered for a few seconds, so the "
                        f"window is now checking every {DEAD_BUS_POLL_PERIOD:.0f} s instead of 10 times a "
                        f"second.\n  Unanswered frames age the CAN controller, and this adapter cannot "
                        f"reset itself. Check the motor's power and its CAN connector; it will be picked "
                        f"up automatically.", C["warn"])
            blocked = [ch for ch, t in rig.threads.items() if t.blocked]
            if blocked != self._blocked_shown:
                self._blocked_shown = blocked
                if blocked:
                    channel = blocked[0]
                    state, restart = can_controller_state(channel)
                    text = (f"  {', '.join(blocked)}: no node is acknowledging our frames - "
                            f"the motor's power is off, or the CAN cable is unplugged.")
                    if state not in ("ERROR-ACTIVE", "?"):
                        text += f"\n  The controller is {state}"
                        if restart == "0":
                            text += (" and will not recover by itself. Run these as two lines:"
                                     f"\n    sudo ip link set {channel} down"
                                     f"\n    sudo ip link set {channel} up type can bitrate 1000000")
                        else:
                            text += "."
                    else:
                        text += " The window keeps trying; it will come back on its own."
                    self.set_banner(text, C["warn"])
                else:
                    self.set_banner("  bus recovered", C["ok"])
            for ch, t in rig.threads.items():
                if t.dead is not None and ch not in self._dead_notified:
                    self._dead_notified.add(ch)
                    self.set_banner(f"  {ch}: CONTROL THREAD DEAD: {t.dead} - motors were sent IDLE, restart the GUI",
                                    C["danger"])
            live = sum(st.live for st in rig.states.values())
            online = sum(st.online for st in rig.states.values())
            self.mode_lbl.configure(text=f"{live} active / {online} online / {len(rig.states)} joints")

            st = self.st
            label, colour = joint_state_of(st)
            self.state_lbl.configure(text=label, fg=colour)
            if st.fault != self._seen_fault.get(self.selected, "-"):
                self._seen_fault[self.selected] = st.fault
                if st.fault:
                    self.set_banner(f"  {st.spec.short} {st.fault}: {FAULT_HELP.get(st.fault, '')}", C["warn"])
            dead = rig.thread_for(st) is None or (rig.thread_for(st) and rig.thread_for(st).dead is not None)
            if st.live and not dead:
                self.set_drive_state("normal")
                if self._seen_epoch.get(self.selected) != st.enable_epoch:
                    self._seen_epoch[self.selected] = st.enable_epoch
                    self.slider.set(D(st.command))
                elif abs(math.radians(float(self.slider.get())) - st.target) > 1e-3:
                    self.slider.set(D(st.target))
                if root.focus_get() is not self.goto_entry:
                    self.goto_var.set(f"{D(st.target):.1f}")
            else:
                self.set_drive_state("disabled")
            if (dead or not st.busy) and bool(self.enable_var.get()) != st.live:
                self.enable_var.set(st.live)

            s = st.spec
            meas = f"{D(st.measured):+8.2f} deg" if st.measured is not None else "      --   "
            raw = f"{st.raw:+8.3f}" if st.raw is not None else "      --"
            err = hex(st.error_code) if st.error_code else "--"
            self.read_lbl.configure(
                text=f"measured {meas}      target {D(st.command):+8.2f} deg\n"
                     f"raw      {raw} rad      offset {s.offset:+8.3f} rad\n"
                     f"dir {s.direction:+.0f}   gear {s.gear_ratio:g}   bus {s.channel}   err {err}"
                     + (f"   missed {st.timeouts}" if st.live and st.timeouts else ""))

            self.diagram.refresh()
            self.trace.sample()
            if self._tick % 4 == 0:        # ~5 Hz redraw keeps the plot cheap
                self.trace.refresh()
            self._after_id = root.after(50, self.refresh)

        def on_close(self):
            self.stop_refresh()
            rig.shutdown()
            root.destroy()

        def stop_refresh(self):
            if getattr(self, "_after_id", None) is not None:
                try:
                    root.after_cancel(self._after_id)
                except Exception:
                    pass
                self._after_id = None

    app = App()
    root.diagram = app.diagram      # layout check reaches in through these
    root.trace = app.trace
    root.slider = app.slider
    root.font_ratio = FONT_RATIO
    root.app = app
    root.stop_refresh = app.stop_refresh
    return root


class DemoMotion(threading.Thread):
    """Sim-only: arm everything and swing between the SRDF presets.

    Purely so the RViz link can be watched without a hand on the hardware;
    main() refuses it outside --sim.
    """

    def __init__(self, rig: Rig, period: float = 4.0):
        super().__init__(daemon=True, name="demo")
        self.rig = rig
        self.period = period
        self.running = True

    def run(self):
        for t in self.rig.threads.values():
            t.unlocked = True
        for st in self.rig.states.values():
            thread = self.rig.thread_for(st)
            if thread:
                thread.request_motor_cmd("zero", st.dev)
        time.sleep(0.5)
        for st in self.rig.states.values():
            thread = self.rig.thread_for(st)
            if thread:
                thread.request_motor_cmd("enable", st.dev)
        time.sleep(1.0)
        order = [n for n in ("ready", "home") if n in self.rig.presets] or [None]
        i = 0
        while self.running:
            values = self.rig.presets.get(order[i % len(order)], {})
            for st in self.rig.states.values():
                if st.live:
                    st.target = clamp(values.get(st.spec.name, 0.0), st.spec.lower, st.spec.upper)
            i += 1
            for _ in range(int(self.period / 0.1)):
                if not self.running:
                    return
                time.sleep(0.1)


# ----------------------------------------------------------------------------
# headless self-test (sim): enable all, drive presets, estop
# ----------------------------------------------------------------------------

def selftest(rig: Rig) -> int:
    def wait(pred, timeout: float, what: str) -> bool:
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            if pred():
                return True
            time.sleep(0.02)
        print(f"[FAIL] timeout: {what}")
        return False

    ok = True
    sts = list(rig.states.values())
    for st in sts:
        rig.thread_for(st).request("ping", st.dev)
    ok &= wait(lambda: all(st.seen for st in sts), 2.0, "ping all")

    # arbitrary boot values -> most joints must be refused, none may go live
    for st in sts:
        rig.thread_for(st).request_motor_cmd("enable", st.dev)
    ok &= wait(lambda: all(not st.busy for st in sts), 3.0, "enable round 1")
    refused = [st for st in sts if st.fault == "OUT_OF_RANGE"]
    live_locked = [st for st in sts if st.live and not st.spec.config_enabled]
    print(f"round 1: live={sum(st.live for st in sts)} out_of_range={len(refused)} "
          f"locked={sum(st.fault == 'LOCKED' for st in sts)}")
    if live_locked:
        print("[FAIL] yaml-disabled joint went live while locked")
        ok = False
    if not refused and not rig.args.sim_zeroed:
        print("[FAIL] expected at least one OUT_OF_RANGE refusal from arbitrary boot values")
        ok = False
    for st in sts:
        if st.live and not (st.spec.lower <= st.measured <= st.spec.upper):
            print(f"[FAIL] {st.spec.name} live outside URDF range")
            ok = False

    # zero everything, unlock, enable all
    for st in sts:
        t = rig.thread_for(st)
        if st.live:
            t.request_motor_cmd("disable", st.dev)
    ok &= wait(lambda: all(not st.busy for st in sts), 3.0, "disable")
    for st in sts:
        rig.thread_for(st).request_motor_cmd("zero", st.dev)
    ok &= wait(lambda: all(not st.busy for st in sts), 3.0, "zero")
    for t in rig.threads.values():
        t.unlocked = True
    for st in sts:
        rig.thread_for(st).request_motor_cmd("enable", st.dev)
    ok &= wait(lambda: all(not st.busy for st in sts), 3.0, "enable round 2")
    snap = {st.dev: st.measured for st in sts}
    time.sleep(0.5)
    moved = [st.spec.short for st in sts if st.live and abs(st.measured - snap[st.dev]) > 1e-6]
    if moved:
        print("[FAIL] enable moved joints:", moved)
        ok = False
    if not all(st.live for st in sts):
        print("[FAIL] not all joints live after zero+unlock:",
              [(st.spec.name, st.fault) for st in sts if not st.live])
        ok = False
    for st in sts:
        if st.measured is None or abs(st.measured) > 1e-6:
            print(f"[FAIL] {st.spec.name} measured {st.measured} after zero")
            ok = False

    # drive to 'ready' preset: must converge, must never exceed limits
    preset = rig.presets.get("ready", {})
    for st in sts:
        st.target = preset.get(st.spec.name, 0.0)
    t0 = time.monotonic()
    worst = 0.0
    while time.monotonic() - t0 < 4.0:
        for st in sts:
            if st.measured is not None:
                over = max(st.spec.lower - st.measured, st.measured - st.spec.upper)
                worst = max(worst, over)
        time.sleep(0.01)
    errs = {st.spec.short: abs(st.measured - st.target) for st in sts}
    print("ready preset tracking error [rad]:", {k: round(v, 4) for k, v in errs.items()})
    if max(errs.values()) > 0.02:
        print("[FAIL] preset not reached")
        ok = False
    if worst > 1e-6:
        print(f"[FAIL] limit exceeded by {worst:.4f} rad")
        ok = False

    # limit clamp: ask for far beyond the upper limit
    st = sts[0]
    st.target = st.spec.upper + 5.0
    time.sleep(2.0)
    margin = rig.threads[st.spec.channel].limit_margin
    if st.measured > st.spec.upper - margin + 1e-3:
        print(f"[FAIL] clamp: {st.measured} > {st.spec.upper - margin}")
        ok = False
    else:
        print(f"clamp ok: {st.spec.short} held at {st.measured:.3f} (upper {st.spec.upper:.3f}, margin {margin})")

    rig.estop_all()
    ok &= wait(lambda: all(not st.live and not st.busy for st in sts), 2.0, "estop")
    print("E-STOP ok: all idle")
    rig.shutdown()
    print("SELFTEST", "PASS" if ok else "FAIL")
    return 0 if ok else 1


# ----------------------------------------------------------------------------

def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--config", type=Path, default=DEFAULT_CONFIG, help="hardware_joints.yaml")
    p.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    p.add_argument("--srdf", type=Path, default=DEFAULT_SRDF, help="group_state presets (home/ready)")
    p.add_argument("--sim", action="store_true", help="fake bus, no CAN needed")
    p.add_argument("--sim-zeroed", action="store_true", help="sim motors boot at raw 0 instead of arbitrary values")
    p.add_argument("--sim-offline", type=int, nargs="*", help="sim: these device ids never answer")
    p.add_argument("--ros", action="store_true", help="publish /joint_states (needs a sourced ROS 2)")
    p.add_argument("--unlock", action="store_true", help="start with yaml-disabled joints unlocked")
    p.add_argument("--kp", type=float, default=None)
    p.add_argument("--kd", type=float, default=None)
    p.add_argument("--torque-limit", type=float, default=None)
    p.add_argument("--max-speed", type=float, default=None, help="[rad/s] overrides safety.max_joint_speed")
    p.add_argument("--rate", type=float, default=None, help="control loop [Hz], default safety.control_rate_hz")
    p.add_argument("--demo", action="store_true",
                   help="sim only: enable everything and cycle home<->ready, to check the RViz link")
    p.add_argument("--selftest", action="store_true", help="headless sim check, exit code = result")
    p.add_argument("--layout-check", action="store_true",
                   help="sim: build the window invisibly and assert nothing in the diagram overlaps or is clipped")
    p.add_argument("--ids", type=int, nargs="+", metavar="ID", default=None,
                   help="only touch these CAN device ids (bench work with a partly populated arm). "
                        "Frames sent to an id with no motor on it are never ACKed, which drives the "
                        "controller to ERROR-PASSIVE and then BUS-OFF; this keeps them off the bus.")
    p.add_argument("--bus", metavar="KEY=CHANNEL", action="append",
                   help="put one of the config's buses on a different CAN channel, e.g. "
                        "--bus right=can0 to bench-test a right-arm motor on the only adapter. "
                        "Repeatable.")
    p.add_argument("--zero-now", action="store_true",
                   help="no window: make each joint's current reading URDF zero and write the offsets "
                        "into the yaml (.bak kept), then exit. Only correct when the joints really are "
                        "at their URDF zero pose. Torque is never switched on.")
    p.add_argument("--control-check", action="store_true",
                   help="sim: drive one joint through every control (drag, step, wheel, keys, typed angle)")
    args = p.parse_args(argv)
    if args.selftest and args.ids:
        p.error("--selftest exercises the whole config (both arms, refusals, presets); "
                "it cannot judge a subset chosen with --ids")
    if args.selftest or args.layout_check or args.control_check:
        args.sim = True
    if args.control_check:
        # the check has to get one joint ARMED: start the fake motors at zero
        # (a random pose would be refused as OUT_OF_RANGE) and ignore the yaml
        # `enabled: false` the way the unlock box does
        args.sim_zeroed = True
        args.unlock = True
    overrides: dict[str, str] = {}
    for item in args.bus or []:
        if "=" not in item:
            p.error(f"--bus expects KEY=CHANNEL, got {item!r}")
        key, _, channel = item.partition("=")
        overrides[key.strip()] = channel.strip()
    args.bus = overrides
    if args.demo and not args.sim:
        p.error("--demo drives every joint on a loop; it is only allowed with --sim")
    if args.demo:
        args.unlock = True      # otherwise the GUI re-locks what the demo just armed
    for f in ("config", "urdf"):
        if not getattr(args, f).is_file():
            p.error(f"--{f}: {getattr(args, f)} not found")
    return args


def layout_check(rig: Rig, args, sizes=((1180, 760), (1500, 950), (1920, 1100), (2950, 1040))) -> int:
    """Build the real window off-screen and verify the diagram reads cleanly.

    Catches exactly what went wrong on the bench display: labels rendered far
    larger than the geometry assumed, so they overlapped each other and ran off
    the canvas. Checked at several window sizes because the drawing is
    resolution-dependent.
    """
    ok = True
    for w, h in sizes:
        root = build_gui(rig, args)
        root.overrideredirect(True)          # no focus stealing, no window decorations
        try:
            root.attributes("-alpha", 0.0)   # and invisible while it lays itself out
        except Exception:
            pass
        root.geometry(f"{w}x{h}+0+0")
        for _ in range(6):
            root.update()
        cv = root.diagram.canvas
        cw, ch = cv.winfo_width(), cv.winfo_height()
        boxes = []
        for item in cv.find_all():
            if cv.type(item) != "text":
                continue
            text = cv.itemcget(item, "text")
            box = cv.bbox(item)
            if box is None:
                continue
            if box[0] < -1 or box[1] < -1 or box[2] > cw + 1 or box[3] > ch + 1:
                print(f"[FAIL] {w}x{h}: '{text}' is clipped: {box} outside 0,0,{cw},{ch}")
                ok = False
            boxes.append((text, box))
        for i in range(len(boxes)):
            for j in range(i + 1, len(boxes)):
                (t1, b1), (t2, b2) = boxes[i], boxes[j]
                if b1[0] < b2[2] and b2[0] < b1[2] and b1[1] < b2[3] and b2[1] < b1[3]:
                    print(f"[FAIL] {w}x{h}: '{t1}' overlaps '{t2}'  {b1} {b2}")
                    ok = False
        # the slider is sized in raw pixels while everything around it is text
        # this display magnifies: check the grip stayed a real mouse target
        grip = int(root.slider.cget("sliderlength"))
        trough = int(root.slider.cget("width"))
        need = int(root.font_ratio * 9)
        if min(grip, trough) < need:
            print(f"[FAIL] {w}x{h}: slider grip {grip}x{trough}px is thinner than one line of text ({need}px)")
            ok = False
        labelled = sum(1 for n in root.diagram.nodes.values() if n["value"] is not None)
        if w >= 1900 and labelled < len(rig.states):
            print(f"[FAIL] {w}x{h}: only {labelled}/{len(rig.states)} joints carry a readout")
            ok = False
        print(f"{w}x{h}: canvas {cw}x{ch}, {len(boxes)} labels checked, {labelled} joint readouts")
        root.stop_refresh()
        root.destroy()
    print("LAYOUT", "PASS" if ok else "FAIL")
    return 0 if ok else 1


def zero_now(rig: Rig, args) -> int:
    """Headless 'Zero here' + 'Save offsets' for every joint in the rig.

    The encoders boot at an arbitrary count and the firmware keeps no offset, so
    this has to be redone after every power cycle -- doing it without the window
    is what makes that bearable, and it is the same bus command the button
    sends. Torque is never switched on.
    """
    D = math.degrees
    bad = [ch for ch, s in rig.bus_status.items() if s.startswith("NO BUS")]
    if bad:
        print(f"[FAIL] no usable CAN bus: {', '.join(f'{c} {rig.bus_status[c]}' for c in bad)}")
        return 1

    for t in rig.threads.values():
        t.request("ping", None)
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline and not all(st.online for st in rig.states.values()):
        time.sleep(0.05)

    silent = [st for st in rig.states.values() if not st.online]
    for st in silent:
        print(f"[FAIL] id {st.dev} ({st.spec.short}) did not answer on {st.spec.channel}")
    if silent:
        print("nothing was zeroed; check power, wiring and --ids")
        return 1

    for st in rig.states.values():
        before_raw, before_deg = st.raw, D(st.measured) if st.measured is not None else float("nan")
        done = st.cmd_done
        rig.thread_for(st).request_motor_cmd("zero", st.dev)
        wait = time.monotonic() + 2.0
        while st.cmd_done == done and time.monotonic() < wait:
            time.sleep(0.02)
        now = D(st.measured) if st.measured is not None else float("nan")
        if abs(now) > 0.5:
            print(f"[FAIL] id {st.dev} ({st.spec.short}) still reads {now:+.2f} deg after zeroing")
            return 1
        print(f"  id {st.dev:<3} {st.spec.short:<18} raw {before_raw:+9.3f} rad   "
              f"{before_deg:+9.2f} -> {now:+.2f} deg   offset {st.spec.offset:+.6f} rad")

    offsets = {st.spec.name: st.spec.offset for st in rig.states.values()}
    backup = save_offsets(args.config, offsets)
    print(f"saved {len(offsets)} offset(s) to {args.config.name}  (backup {backup.name})")
    print("ZERO PASS")
    return 0


def control_check(rig: Rig, args) -> int:
    """Drive one simulated joint through every way the GUI offers to move it.

    The slider alone is a poor control on a high-DPI display, so the angle can
    also be stepped, wheeled, typed and arrowed; each of those paths ends in
    the same clamp, and this walks all of them so none can quietly rot.
    """
    D = math.degrees
    root = build_gui(rig, args)
    root.overrideredirect(True)
    try:
        root.attributes("-alpha", 0.0)
    except Exception:  # noqa: BLE001
        pass
    root.geometry("1920x1100+0+0")
    root.focus_force()          # X11 only delivers key events to a focused window
    app = root.app
    fails: list[str] = []

    def pump(seconds: float = 0.5):
        end = time.time() + seconds
        while time.time() < end:
            root.update()
            time.sleep(0.01)

    def check(name: str, cond: bool, extra: str = ""):
        print(("  ok  " if cond else "[FAIL]"), name, extra)
        if not cond:
            fails.append(name)

    name = next(iter(rig.states))
    app.select(name)
    st = rig.states[name]
    pump(0.4)

    # nothing may drive a motor that is not energised
    check("controls disabled while idle",
          str(app.slider.cget("state")) == "disabled"
          and all(str(b.cget("state")) == "disabled" for b in app.step_btns)
          and str(app.goto_entry.cget("state")) == "disabled")
    held = st.target
    app.nudge(5.0)
    check("step ignored while idle", st.target == held)

    rig.thread_for(st).request_motor_cmd("enable", st.dev)
    pump(1.5)
    check("motor went live", st.live)
    check("controls follow the motor",
          str(app.slider.cget("state")) == "normal"
          and all(str(b.cget("state")) == "normal" for b in app.step_btns))

    base = D(st.target)
    app.nudge(+1.0)
    check("step button +1 deg", abs(D(st.target) - (base + 1.0)) < 1e-6, f"{D(st.target):+.3f}")
    app.wheel(-STEP_FINE)
    check(f"wheel notch -{STEP_FINE:g} deg", abs(D(st.target) - (base + 0.9)) < 1e-6, f"{D(st.target):+.3f}")
    root.event_generate("<Shift-Up>", when="now")
    root.update()
    check("shift+arrow +5 deg", abs(D(st.target) - (base + 5.9)) < 1e-6, f"{D(st.target):+.3f}")

    app.goto_entry.focus_set()
    root.update()
    held = st.target
    app.goto_entry.event_generate("<Up>", when="now")
    root.update()
    check("arrow key left alone while typing", st.target == held)
    root.focus_force()
    root.update()

    app.goto_var.set("12.5")
    app.on_goto()
    check("typed 12.5 deg", abs(D(st.target) - 12.5) < 1e-6, f"{D(st.target):+.3f}")
    check("slider followed the typed angle", abs(float(app.slider.get()) - 12.5) < 1e-6)
    hi = D(st.spec.upper)
    app.goto_var.set(f"{hi + 40:.1f}")
    app.on_goto()
    check("typed angle clamped to the URDF limit", abs(D(st.target) - hi) < 1e-6,
          f"{D(st.target):+.3f} <= {hi:+.1f}")
    app.goto_var.set("not a number")
    app.on_goto()
    check("junk text rejected, target untouched", abs(D(st.target) - hi) < 1e-6)

    app.slider.set(-10.0)
    root.update()
    check("dragging the slider sets the target", abs(D(st.target) + 10.0) < 1e-6, f"{D(st.target):+.3f}")

    # back to URDF zero from the button, the key, and for a whole arm
    app.on_goto_zero()
    check("'to 0' button reaches URDF zero", abs(D(st.target)) < 1e-6, f"{D(st.target):+.3f}")
    check("slider followed it back to zero", abs(float(app.slider.get())) < 1e-6)
    app.nudge(+7.0)
    root.event_generate("<Key-0>", when="now")
    root.update()
    check("0 key reaches URDF zero", abs(D(st.target)) < 1e-6, f"{D(st.target):+.3f}")
    app.goto_entry.focus_set()
    root.update()
    app.nudge(+7.0)
    held = st.target
    app.goto_entry.event_generate("<Key-0>", when="now")
    root.update()
    check("0 key left alone while typing", st.target == held, f"{D(st.target):+.3f}")
    root.focus_force()
    root.update()

    side = st.spec.bus_key
    arm = [m for m in rig.states.values() if m.spec.bus_key == side and m.live]
    for m in arm:
        m.target = math.radians(6.0)
    app.on_zero_arm(side)
    check(f"arm zero moved every active joint on {side}",
          arm and all(abs(D(m.target)) < 1e-6 for m in arm), f"{len(arm)} joint(s)")

    # ... but only ones that are armed: an idle joint must stay untouched
    idle = next((m for m in rig.states.values() if m.spec.bus_key == side and not m.live), None)
    if idle is not None:
        idle.target = math.radians(3.0)
        app.on_zero_arm(side)
        check("arm zero skips joints that are not enabled", abs(D(idle.target) - 3.0) < 1e-6)

    app.on_estop()
    pump(0.3)
    check("E-STOP dropped the joint", not st.live)

    root.stop_refresh()
    root.destroy()
    print("CONTROL", "PASS" if not fails else f"FAIL {fails}")
    return 0 if not fails else 1


def main(argv=None) -> int:
    args = parse_args(argv)
    if args.rate is None:
        cfg = yaml.safe_load(args.config.read_text())
        args.rate = float(cfg.get("safety", {}).get("control_rate_hz", 100.0))
    rig = Rig(args)
    print(f"joints: {len(rig.specs)}  buses: {rig.bus_status}  gains: {rig.gains}  presets: {sorted(rig.presets)}")
    if args.selftest:
        return selftest(rig)
    if args.layout_check:
        return layout_check(rig, args)
    if args.control_check:
        return control_check(rig, args)
    if args.zero_now:
        try:
            return zero_now(rig, args)
        finally:
            rig.shutdown()

    ros = None
    if args.ros:
        try:
            ros = JointStatePublisher(rig)
            print("publishing /joint_states")
        except Exception as e:  # noqa: BLE001
            print(f"[WARN] --ros requested but rclpy unavailable: {e}")
            args.ros = False
    demo = None
    if args.demo:
        demo = DemoMotion(rig)
        demo.start()
        print("demo motion running (sim): home <-> ready")

    root = build_gui(rig, args)

    import signal

    def _on_signal(_signum, _frame):
        root.after(0, root.quit)

    signal.signal(signal.SIGTERM, _on_signal)
    signal.signal(signal.SIGINT, _on_signal)
    try:
        root.mainloop()
    finally:
        if demo:
            demo.running = False
        if ros:
            ros.stop()
        rig.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
