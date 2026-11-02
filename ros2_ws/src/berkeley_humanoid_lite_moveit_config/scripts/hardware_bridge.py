#!/usr/bin/env python3
"""Drive the real arm motors from MoveIt, over recoil CAN.

Same FollowJointTrajectory interface as sim_trajectory_server.py, so nothing on
the MoveIt side changes -- only which node is listening.  This one talks to
actuators, so it is built to refuse rather than to guess.

  * **Dry run is the default.** Without --live no bus is opened at all; the
    bridge integrates commands internally and publishes them, which is how you
    check joint mapping, limits and trajectory flow with the robot unpowered.
  * **Output starts disabled**, even with --live. Motors stay in IDLE until
    somebody calls the enable service. Trajectories before that are rejected,
    not silently dropped.
  * Commands are slew-rate limited, so a step goal becomes a ramp.
  * Commands are clamped into the URDF limits less a margin.
  * A joint that stops answering PDOs freezes, then auto-disables.
  * Idle joints are still read (one per bus per cycle), so /joint_states shows
    the real arm before anything is enabled -- back-drive a joint by hand to
    check `direction`/`offset` -- and enabling refuses a joint that is silent
    or sitting outside its URDF limits (an unset offset) instead of yanking it.
  * CAN adapters are hot-plugged: each channel is watched twice a second
    (sysfs), opened when it comes up, dropped when it goes down or vanishes
    and reopened after a replug. Its joints are read-only while it is away,
    and any that were energised come back idle, never live on their own.
  * E-STOP is out of band: it sets a flag the control loop checks every cycle
    and puts every motor in IDLE regardless of what is queued.  It latches, and
    clearing it is a separate service so recovery cannot happen by accident.

Joints marked `enabled: false` in the map are read but never commanded, and a
trajectory naming a joint that is not in the map at all is rejected outright.

    ros2 run berkeley_humanoid_lite_moveit_config hardware_bridge.py
    ros2 run berkeley_humanoid_lite_moveit_config hardware_bridge.py --live

    ros2 service call /hardware_bridge/enable std_srvs/srv/SetBool '{data: true}'
    ros2 service call /hardware_bridge/estop std_srvs/srv/Trigger
    ros2 service call /hardware_bridge/reset_estop std_srvs/srv/Trigger
    ros2 service call /hardware_bridge/zero std_srvs/srv/Trigger    # pose = URDF zero (output off)

The V1 wrist (Feetech SCS2332/SCS15 serial servos, three per arm) is driven
from the same node over a half-duplex TTL adapter, so a v1arm trajectory --
shoulder, elbow AND wrist -- lands on one action server. Wrist joints follow the
same rules: mapped in `wrist_servos`, `enabled: false` until verified, dry run
unless --live, torque off until enabled, torque off on E-STOP. Per bus one
SYNC_WRITE carries every goal each cycle and present positions are polled one
servo per cycle round-robin, so a 3-servo bus reports each joint at ~33 Hz.

    ros2 run berkeley_humanoid_lite_moveit_config hardware_bridge.py --live --buses wrist

opens only the wrist serial ports and leaves CAN alone (useful before the arm
motors are wired), `--buses can` the reverse; joints on a bus that is not opened
are reported but refuse commands.

The hands are not handled here; dexhand_bringup's driver already speaks
FollowJointTrajectory. See config/hardware_joints.yaml.
"""

from __future__ import annotations

import argparse
import json
import math
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

PACKAGE = "berkeley_humanoid_lite_moveit_config"


CAN_WATCH_PERIOD = 0.5            # [s] how often CAN adapters are looked for
SYSFS_NET = Path("/sys/class/net")
IFF_UP = 0x1


def can_link(channel: str) -> tuple[str, int | None]:
    """("up" | "down" | "missing", ifindex) from sysfs: no subprocess, no
    socket, so it is cheap enough to ask twice a second and safe in dry run."""
    base = SYSFS_NET / channel
    try:
        ifindex = int((base / "ifindex").read_text())
        flags = int((base / "flags").read_text(), 16)
    except (OSError, ValueError):
        return "missing", None
    return ("up" if flags & IFF_UP else "down"), ifindex


def duration_to_seconds(duration: Duration) -> float:
    return duration.sec + duration.nanosec * 1e-9


class FeetechBus:
    """Minimal Feetech SCS (SCS15 / SCS2332) half-duplex serial protocol.

    Big-endian words, the "SCSCL" register map. Only what the bridge needs:
    ping, torque on/off, read present position, sync-write goal positions.
    Every call holds the port lock, so the control loop and the enable service
    cannot interleave bytes on the wire.
    """

    PING, READ, WRITE, SYNC_WRITE = 0x01, 0x02, 0x03, 0x83
    BROADCAST = 0xFE
    REG_TORQUE_ENABLE = 40
    REG_GOAL_POSITION = 42      # position(2) time(2) speed(2)
    REG_PRESENT_POSITION = 56

    def __init__(self, port: str, baud: int = 1_000_000, timeout: float = 0.02) -> None:
        import serial  # pyserial; only needed with --live
        self.serial = serial.Serial(port, baudrate=baud, timeout=timeout)
        self.lock = threading.Lock()

    def close(self) -> None:
        self.serial.close()

    # -- framing ---------------------------------------------------------
    @staticmethod
    def _checksum(payload: bytes) -> int:
        return (~sum(payload)) & 0xFF

    def _send(self, servo_id: int, instruction: int, params: bytes) -> None:
        body = bytes([servo_id, len(params) + 2, instruction]) + params
        self.serial.reset_input_buffer()
        self.serial.write(b"\xff\xff" + body + bytes([self._checksum(body)]))
        self.serial.flush()

    def _recv(self, servo_id: int) -> bytes | None:
        """Status packet: FF FF ID LEN ERR params CHK -> params, or None."""
        head = self.serial.read(4)
        if len(head) < 4 or head[0] != 0xFF or head[1] != 0xFF or head[2] != servo_id:
            return None
        length = head[3]
        rest = self.serial.read(length)
        if len(rest) < length:
            return None
        if self._checksum(head[2:] + rest[:-1]) != rest[-1]:
            return None
        error, params = rest[0], rest[1:-1]
        if error:
            # Servo answered but flags a fault (overload, voltage...). Treat as
            # a valid link; the caller can inspect it if it cares.
            pass
        return params

    # -- commands --------------------------------------------------------
    def ping(self, servo_id: int) -> bool:
        with self.lock:
            self._send(servo_id, self.PING, b"")
            return self._recv(servo_id) is not None

    def read_word(self, servo_id: int, address: int) -> int | None:
        with self.lock:
            self._send(servo_id, self.READ, bytes([address, 2]))
            params = self._recv(servo_id)
        if params is None or len(params) < 2:
            return None
        return (params[0] << 8) | params[1]

    def read_position(self, servo_id: int) -> int | None:
        return self.read_word(servo_id, self.REG_PRESENT_POSITION)

    def write_torque(self, servo_id: int, on: bool) -> bool:
        with self.lock:
            self._send(servo_id, self.WRITE, bytes([self.REG_TORQUE_ENABLE, 1 if on else 0]))
            return self._recv(servo_id) is not None

    def sync_write_goals(self, goals: dict[int, tuple[int, int]]) -> None:
        """goals: servo_id -> (position ticks, speed ticks/s); one packet, no reply."""
        if not goals:
            return
        params = bytearray([self.REG_GOAL_POSITION, 6])
        for servo_id, (position, speed) in goals.items():
            position = min(max(int(position), 0), 1023)
            speed = min(max(int(speed), 0), 1023)
            params += bytes([servo_id, position >> 8, position & 0xFF,
                             0, 0, speed >> 8, speed & 0xFF])
        with self.lock:
            self._send(self.BROADCAST, self.SYNC_WRITE, bytes(params))


class JointHandle:
    """One actuator: where it lives, which way it turns, and how it is doing.

    `kind` is "can" (recoil actuator, radians on the wire) or "wrist" (Feetech
    serial servo, 0..1023 ticks on the wire).
    """

    def __init__(self, name: str, spec: dict, kind: str = "can",
                 servo_models: dict | None = None) -> None:
        self.name = name
        self.kind = kind
        self.bus_key = spec["bus"]
        self.device_id = int(spec.get("device_id", spec.get("servo_id", 0)))
        self.direction = float(spec.get("direction", 1.0))
        self.offset = float(spec.get("offset", 0.0))
        # Actuator units per URDF radian. The recoil firmware reports whatever
        # its own gear_ratio parameter says; with the factory default of 1.0
        # that is ROTOR radians, so a 15:1 gearbox needs 15 here (or -15 --
        # sign folds into direction, keep it positive and use direction).
        self.gear_ratio = float(spec.get("gear_ratio", 1.0))
        if self.gear_ratio == 0.0:
            raise SystemExit(f"{name}: gear_ratio must not be 0")
        self.enabled = bool(spec.get("enabled", False))
        # Per-joint overrides of the global `gains` block (e.g. the shoulder
        # roll carries the whole arm and needs more than the rest).
        self.gains = dict(spec.get("gains") or {})

        # Wrist servos: URDF radians <-> servo ticks. `range_deg` is the travel
        # covered by 0..1023 and `center_ticks` the tick count at URDF zero.
        model = (servo_models or {}).get(spec.get("model", ""), {})
        range_deg = float(spec.get("range_deg", model.get("range_deg", 200.0)))
        self.ticks_per_rad = 1024.0 / math.radians(range_deg)
        self.center_ticks = float(spec.get("center_ticks", model.get("center_ticks", 512)))
        self.max_speed_ticks = int(spec.get("speed_ticks", model.get("speed_ticks", 0)))

        self.lower = -3.14159
        self.upper = 3.14159
        self.command = 0.0        # slew-limited target, URDF convention [rad]
        self.measured = 0.0       # URDF convention [rad]
        self.raw = 0.0            # last value as the actuator reported it
        self.missed = 0
        self.fault: str | None = None
        self.stream_target: float | None = None   # latest ~/joint_command goal
        # Operator travel limits inside the URDF ones (~/set_limits, --limits-file).
        self.user_lower: float | None = None
        self.user_upper: float | None = None
        self.last_reply = 0.0     # monotonic time of the last answer from the motor
        self.live = False         # actually energised right now
        self.seen = False         # has ever answered on its bus

    def to_actuator(self, value: float) -> float:
        return (value + self.offset) * self.direction * self.gear_ratio

    def to_urdf(self, value: float) -> float:
        self.raw = value
        return value / self.gear_ratio * self.direction - self.offset

    def offset_for_zero_here(self) -> float:
        """The `offset` that would make the current raw reading URDF zero."""
        return self.raw / self.gear_ratio * self.direction

    def to_ticks(self, value: float) -> int:
        return int(round(self.center_ticks + self.to_actuator(value) * self.ticks_per_rad))

    def from_ticks(self, ticks: int) -> float:
        return self.to_urdf((ticks - self.center_ticks) / self.ticks_per_rad)

    def effective_limits(self, margin: float = 0.0) -> tuple[float, float]:
        """URDF limits less `margin`, narrowed further by the operator limits."""
        lower, upper = self.lower + margin, self.upper - margin
        if self.user_lower is not None:
            lower = max(lower, self.user_lower)
        if self.user_upper is not None:
            upper = min(upper, self.user_upper)
        return lower, upper

    def clamp(self, value: float, margin: float) -> float:
        lower, upper = self.effective_limits(margin)
        return min(max(value, lower), upper)


class HardwareBridge(Node):
    def __init__(self, config: dict, live: bool, buses: str = "all",
                 description_node: str = "/move_group",
                 limits_file: Path | None = None, zero_at_start: bool = False,
                 reference_file: Path | None = None) -> None:
        super().__init__("hardware_bridge")
        self.zero_at_start = zero_at_start
        # With --zero-at-start, the pose each motor had when it first answered
        # after power-on is kept here as raw actuator readings, so restarting
        # the bridge keeps the power-on reference instead of re-zeroing
        # wherever the arm has drifted to. Delete it after a power cycle.
        self.reference_file = reference_file
        self.reference: dict[str, float] = {}
        if zero_at_start and reference_file is not None and reference_file.is_file():
            data = yaml.safe_load(reference_file.read_text()) or {}
            self.reference = {k: float(v) for k, v in (data.get("raw") or {}).items()}
            self.reference_taken = data.get("taken", "?")
        self.description_node = description_node
        self.live = live
        self.config = config
        self.safety = config["safety"]
        self.gains = config["gains"]
        self.open_can = buses in ("all", "can")
        self.open_wrist = buses in ("all", "wrist")

        self.lock = threading.Lock()
        self.estop = threading.Event()
        self.output_enabled = False
        # Operator arm selection (~/set_arms): an unticked arm is idled and
        # takes no commands, whatever the enable state.
        self.arm_active = {"left": True, "right": True}

        self.joints = {name: JointHandle(name, spec, "can")
                       for name, spec in (config.get("joints") or {}).items()}
        wrist = config.get("wrist_servos") or {}
        models = wrist.get("models") or {}
        for name, spec in (wrist.get("joints") or {}).items():
            if name in self.joints:
                raise SystemExit(f"{name} is listed under both joints and wrist_servos")
            self.joints[name] = JointHandle(name, spec, "wrist", models)
        self.wrist_speed_default = float(wrist.get("speed_scale", 1.0))
        # Round-robin cursors for read-only polling of idle joints.
        self._wrist_poll: dict[str, int] = {}
        self._can_poll: dict[str, int] = {}

        # In live mode a bus we were told not to open makes its joints
        # read-only: they stay in the map (MoveIt still sees them) but any
        # trajectory touching them is refused, exactly like enabled: false.
        if self.live:
            for handle in self.joints.values():
                wanted = self.open_can if handle.kind == "can" else self.open_wrist
                if not wanted and handle.enabled:
                    handle.enabled = False
                    self.get_logger().warn(
                        f"{handle.name}: its bus is not being opened (--buses); "
                        "left read-only")

        # What the yaml (and --buses) allow; `enabled` itself drops to False
        # while a joint's CAN adapter is absent and comes back with it.
        for handle in self.joints.values():
            handle.configured_enabled = handle.enabled

        self._apply_urdf_limits()
        if limits_file is not None:
            self._load_user_limits(limits_file)
        self.buses: dict[str, object] = {}
        self.wrist_buses: dict[str, FeetechBus] = {}
        # CAN hot-plug: channel -> open socket / the ifindex it was bound to /
        # what the watcher last saw ("open", "up", "down", "missing").
        self.channels: dict[str, object] = {}
        self.channel_ifindex: dict[str, int] = {}
        self.channel_state: dict[str, str] = {}
        self._channel_note: dict[str, str] = {}
        self._bus_error_logged: dict[str, float] = {}
        self.recoil = None
        if self.live:
            self._open_buses()

        callback_group = ReentrantCallbackGroup()
        # Adapters come and go (the gs_usb one drops off USB); follow them
        # instead of deciding once at start-up. In dry run this only reports.
        if self._can_channels():
            self._watch_can()
            self.create_timer(CAN_WATCH_PERIOD, self._watch_can, callback_group=callback_group)
        self.publisher = self.create_publisher(JointState, "/joint_states", 10)
        rate = float(self.safety.get("control_rate_hz", 100.0))
        self.period = 1.0 / rate
        self.create_timer(self.period, self._tick, callback_group=callback_group)

        self.create_service(SetBool, "~/enable", self._on_enable,
                            callback_group=callback_group)
        self.create_service(Trigger, "~/estop", self._on_estop,
                            callback_group=callback_group)
        self.create_service(Trigger, "~/reset_estop", self._on_reset_estop,
                            callback_group=callback_group)
        self.create_service(Trigger, "~/zero", self._on_zero,
                            callback_group=callback_group)
        # Streaming goals (drag_teleop.py): the newest message wins, and the
        # control loop walks each joint towards it at max_joint_speed. Same
        # enable/limit/fault rules as a trajectory; a trajectory arriving takes
        # over and drops any streamed goal for its joints.
        self.create_subscription(JointState, "~/joint_command", self._on_joint_command, 10,
                                 callback_group=callback_group)
        self._stream_warned = False
        # Operator panel (arm_panel.py): JSON status out, JSON limits in.
        self.status_publisher = self.create_publisher(String, "~/status", 10)
        self.create_timer(0.1, self._publish_status, callback_group=callback_group)
        self.create_subscription(String, "~/set_limits", self._on_set_limits, 10,
                                 callback_group=callback_group)
        self.create_subscription(String, "~/set_arms", self._on_set_arms, 10,
                                 callback_group=callback_group)
        self.action = ActionServer(
            self, FollowJointTrajectory,
            "/arms_controller/follow_joint_trajectory",
            execute_callback=self._execute,
            goal_callback=lambda _g: GoalResponse.ACCEPT,
            cancel_callback=lambda _g: CancelResponse.ACCEPT,
            callback_group=callback_group,
        )

        commandable = [j.name for j in self.joints.values() if j.enabled]
        mode = "LIVE" if self.live else "dry run (no bus opened)"
        self.get_logger().info(
            f"hardware bridge up in {mode}; {len(commandable)} of "
            f"{len(self.joints)} mapped joints are commandable: {commandable}"
        )
        self.get_logger().warn(
            "output is DISABLED. enable with: "
            "ros2 service call /hardware_bridge/enable std_srvs/srv/SetBool '{data: true}'"
        )

    # ------------------------------------------------------------------ setup

    def _apply_urdf_limits(self) -> None:
        """Take each joint's travel from the robot description a running node
        holds -- move_group by default, robot_state_publisher without MoveIt."""
        service = f"{self.description_node}/get_parameters"
        client = self.create_client(GetParameters, service)
        if not client.wait_for_service(timeout_sec=10.0):
            raise SystemExit(
                f"{service} unavailable; start the node holding robot_description "
                "first so joint limits can be read from it"
            )
        request = GetParameters.Request(names=["robot_description"])
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        result = future.result()
        if result is None or not result.values or not result.values[0].string_value:
            raise SystemExit("could not read robot_description")

        root = ET.fromstring(result.values[0].string_value)
        found = 0
        for joint in root.findall("joint"):
            handle = self.joints.get(joint.get("name"))
            limit = joint.find("limit")
            if handle is None or limit is None:
                continue
            handle.lower = float(limit.get("lower", handle.lower))
            handle.upper = float(limit.get("upper", handle.upper))
            found += 1
        described = {j.get("name") for j in root.findall("joint")}
        missing = [n for n in self.joints if n not in described]
        for name in missing:
            # The map covers the fullest model (v1arm); a lesser model simply
            # lacks some of these joints. Dropping them keeps /joint_states and
            # the commandable set honest for whatever move_group loaded, and a
            # misspelt name still ends up rejected as "not in the hardware map".
            del self.joints[name]
        if missing:
            self.get_logger().warn(
                f"{len(missing)} mapped joints are not in this robot model and were "
                f"dropped: {missing}")
        self.get_logger().info(f"joint limits read from robot_description ({found})")

    def _take_reference(self, handle: JointHandle) -> None:
        """Zero `handle` at the stored power-on reading, or at its current
        reading (and store that) if it has none yet."""
        if handle.name in self.reference:
            handle.offset = self.reference[handle.name] / handle.gear_ratio * handle.direction
            self.get_logger().warn(
                f"{handle.name}: reference from power-on ({self.reference_taken}), "
                f"now at {math.degrees(handle.to_urdf(handle.raw)):+.1f} deg")
            return
        handle.offset = handle.offset_for_zero_here()
        self.reference[handle.name] = handle.raw
        self.get_logger().warn(
            f"{handle.name}: first answer, this pose taken as 0 (offset {handle.offset:+.4f})")
        if self.reference_file is not None:
            if not getattr(self, "reference_taken", None):
                self.reference_taken = time.strftime("%Y-%m-%d %H:%M:%S")
            self.reference_file.parent.mkdir(parents=True, exist_ok=True)
            self.reference_file.write_text(
                "# Raw actuator readings taken as 0 deg for every joint (hardware_bridge\n"
                "# --zero-at-start). drag_teleop.launch.py deletes this file on every start\n"
                "# (new_reference defaults to true), so each run re-reads the motors and zeroes\n"
                "# at the pose they are actually in. Launch new_reference:=false to reuse it.\n"
                + yaml.safe_dump({"taken": self.reference_taken, "raw": self.reference}, sort_keys=True))

    def _set_user_limits(self, name: str, lower: float | None, upper: float | None) -> str | None:
        """Apply one joint's operator limits [rad]; returns an error or None.
        Either bound may be None (= the URDF one). They must lie inside the
        URDF limits and leave some travel."""
        handle = self.joints.get(name)
        if handle is None:
            return f"{name}: not in the hardware map"
        if lower is not None and not handle.lower <= lower <= handle.upper:
            return f"{name}: lower {math.degrees(lower):.1f} deg is outside the URDF range"
        if upper is not None and not handle.lower <= upper <= handle.upper:
            return f"{name}: upper {math.degrees(upper):.1f} deg is outside the URDF range"
        if lower is not None and upper is not None and upper - lower < math.radians(1.0):
            return f"{name}: limits must leave at least 1 deg of travel"
        handle.user_lower, handle.user_upper = lower, upper
        # Pull any pending goal inside the new range right away.
        margin = float(self.safety["limit_margin"])
        if handle.stream_target is not None:
            handle.stream_target = handle.clamp(handle.stream_target, margin)
        if handle.live:
            handle.stream_target = handle.clamp(
                handle.command if handle.stream_target is None else handle.stream_target, margin)
        return None

    def _load_user_limits(self, path: Path) -> None:
        if not path.is_file():
            self.get_logger().info(f"no operator limits file yet ({path})")
            return
        data = yaml.safe_load(path.read_text()) or {}
        for name, bounds in data.items():
            lower, upper = (None if b is None else math.radians(float(b)) for b in bounds)
            error = self._set_user_limits(name, lower, upper)
            if error:
                self.get_logger().error(f"{path.name}: {error}; ignored")
            else:
                self.get_logger().info(f"operator limits {name}: {bounds} deg")

    def _on_set_limits(self, message: String) -> None:
        """{"joint": [lower_deg | null, upper_deg | null], ...}"""
        try:
            data = json.loads(message.data)
        except json.JSONDecodeError as error:
            self.get_logger().error(f"set_limits: bad JSON: {error}")
            return
        with self.lock:
            for name, bounds in data.items():
                try:
                    lower, upper = (None if b is None else math.radians(float(b)) for b in bounds)
                except (TypeError, ValueError):
                    self.get_logger().error(f"set_limits: {name}: expected [lower, upper] in deg")
                    continue
                error = self._set_user_limits(name, lower, upper)
                if error:
                    self.get_logger().error(f"set_limits: {error}")
                else:
                    self.get_logger().warn(f"operator limits {name}: {bounds} deg")

    def _publish_status(self) -> None:
        now = time.monotonic()
        with self.lock:
            joints = {}
            for name, h in self.joints.items():
                if h.kind != "can":
                    continue
                lower, upper = h.effective_limits()
                joints[name] = {
                    "device_id": h.device_id, "bus": h.bus_key,
                    "enabled": h.enabled, "seen": h.seen, "live": h.live,
                    "fault": h.fault, "missed": h.missed,
                    "age": None if not h.last_reply else round(now - h.last_reply, 3),
                    "measured": h.measured, "command": h.command,
                    "urdf": [h.lower, h.upper], "limits": [lower, upper],
                    "user": [h.user_lower, h.user_upper],
                    "arm_active": self._arm_active(h),
                    "configured_enabled": h.configured_enabled,
                    "bus_open": h.bus_key in self.buses,
                }
            status = {"live": self.live, "output_enabled": self.output_enabled,
                      "estop": self.estop.is_set(), "joints": joints,
                      "arms": dict(self.arm_active),
                      "buses": {k: self.config.get("bus", {}).get(k) for k in self.buses},
                      "channels": dict(self.channel_state)}
        self.status_publisher.publish(String(data=json.dumps(status)))

    def _open_buses(self) -> None:
        can_keys = {j.bus_key for j in self.joints.values() if j.kind == "can"}
        if self.open_can and can_keys:
            try:
                import berkeley_humanoid_lite_lowlevel.recoil as recoil
            except ImportError as error:
                raise SystemExit(
                    f"--live needs the lowlevel package on PYTHONPATH: {error} "
                    "(or pass --buses wrist to leave CAN alone)"
                )
            self.recoil = recoil
            for channel, keys in self._can_channels().items():
                state, ifindex = can_link(channel)
                self.channel_state[channel] = state
                if state != "up" or not self._attach_channel(channel, keys, ifindex):
                    # An arm whose adapter is not plugged in should not take
                    # the other arm down with it, and plugging it in later is
                    # picked up by _watch_can. Until then its joints stay
                    # mapped (so MoveIt still sees them) but read-only.
                    self._mark_channel_absent(channel, keys)
                    self._note(channel, self._waiting_text(channel, state))
            for handle in self.joints.values():
                if handle.kind != "can" or handle.bus_key not in self.buses:
                    continue
                measured = self._read_can_position(handle, timeout=0.05)
                if measured is None:
                    self.get_logger().warn(
                        f"{handle.name}: motor {handle.device_id} on "
                        f"'{handle.bus_key}' did not answer; reported as 0 until it does")
                    continue
                handle.seen = True
                handle.last_reply = time.monotonic()
                if self.zero_at_start:
                    # Power-on pose is URDF zero (--zero-at-start): every
                    # angle, limit and drag is relative to it.
                    self._take_reference(handle)
                    measured = handle.to_urdf(handle.raw)
                handle.measured = measured
                handle.command = measured
                self.get_logger().info(
                    f"{handle.name}: motor {handle.device_id} at {measured:+.3f} rad "
                    f"(URDF limits {handle.lower:+.2f}..{handle.upper:+.2f})")

        wrist_keys = {j.bus_key for j in self.joints.values() if j.kind == "wrist"}
        if self.open_wrist and wrist_keys:
            ports = (self.config.get("wrist_servos") or {}).get("bus") or {}
            for key in sorted(wrist_keys):
                if key not in ports:
                    raise SystemExit(f"wrist_servos.bus has no entry for '{key}'")
                port = ports[key]
                device, baud = port["port"], int(port.get("baud", 1_000_000))
                self.get_logger().info(f"opening wrist servo bus '{key}' on {device} @ {baud}")
                try:
                    self.wrist_buses[key] = FeetechBus(device, baud)
                except Exception as error:  # noqa: BLE001 - surface the OS message
                    raise SystemExit(f"cannot open {device}: {error}")
                self._wrist_poll[key] = 0
                # Read once at start-up so /joint_states shows the real wrist
                # before anything is enabled, and so a missing servo shows now.
                for handle in self.joints.values():
                    if handle.kind != "wrist" or handle.bus_key != key:
                        continue
                    ticks = self.wrist_buses[key].read_position(handle.device_id)
                    if ticks is None:
                        self.get_logger().warn(
                            f"{handle.name}: servo id {handle.device_id} on '{key}' "
                            "did not answer; it will fault when enabled")
                    else:
                        handle.measured = handle.from_ticks(ticks)
                        handle.command = handle.measured

    # ------------------------------------------------------------ CAN hot-plug

    def _can_channels(self) -> dict[str, list[str]]:
        """channel -> the bus keys on it, for every CAN joint in the map."""
        if not self.open_can:
            return {}
        can_keys = {j.bus_key for j in self.joints.values() if j.kind == "can"}
        channels: dict[str, list[str]] = {}
        for key, channel in (self.config.get("bus") or {}).items():
            if key in can_keys and channel:
                channels.setdefault(channel, []).append(key)
        return channels

    @staticmethod
    def _waiting_text(channel: str, state: str) -> str:
        return (f"CAN {channel} is {state}; waiting for it (plug the adapter in / bring the "
                f"link up and it is picked up within {CAN_WATCH_PERIOD:.1f} s)")

    def _note(self, channel: str, text: str, error: bool = False) -> None:
        """Log `text` once, not every watch period while nothing changes."""
        if self._channel_note.get(channel) == text:
            return
        self._channel_note[channel] = text
        # Two call sites on purpose: rclpy binds a severity to the line that
        # logs and raises if the same line later logs at another one.
        if error:
            self.get_logger().error(text)
        else:
            self.get_logger().warn(text)

    def _attach_channel(self, channel: str, keys: list[str], ifindex: int | None) -> bool:
        """Open `channel` and hand its joints back. Both arms on one adapter
        (the bench) share this one socket, or each would receive -- and
        eventually misread -- the other's replies."""
        try:
            bus = self.recoil.Bus(channel)
        except Exception as error:  # noqa: BLE001 - socketcan raises OSError variants
            self._note(channel, f"CAN {channel} is up but could not be opened: {error}", error=True)
            return False
        self.channels[channel] = bus
        self.channel_ifindex[channel] = ifindex
        self.channel_state[channel] = "open"
        for key in keys:
            self.buses[key] = bus
            self._can_poll[key] = 0
        for handle in self.joints.values():
            if handle.kind == "can" and handle.bus_key in keys:
                handle.enabled = handle.configured_enabled
                handle.missed = 0
                # Back, but idle: it drives again only after ~/enable.
                if handle.fault == "CAN_LOST":
                    handle.fault = None
        self._note(channel, f"CAN {channel} connected (bus {', '.join(keys)}); "
                            "motors are read as they answer")
        return True

    def _mark_channel_absent(self, channel: str, keys: list[str]) -> list[str]:
        """Joints on a closed channel: read-only, not live, not answering.
        Returns the ones that were energised. A motor that comes back keeps
        its power-on reference (--zero-at-start): that is keyed by name."""
        lost = []
        for handle in self.joints.values():
            if handle.kind != "can" or handle.bus_key not in keys:
                continue
            if handle.live:
                # Nothing can be sent any more, IDLE included; the firmware
                # watchdog stops the motor once the PDOs stop arriving.
                handle.fault = "CAN_LOST"
                lost.append(handle.name)
            handle.live = False
            handle.stream_target = None
            handle.enabled = False
            handle.seen = False
            handle.missed = 0
        return lost

    def _detach_channel(self, channel: str, reason: str) -> None:
        bus = self.channels.pop(channel, None)
        self.channel_ifindex.pop(channel, None)
        keys = self._can_channels().get(channel, [])
        for key in keys:
            self.buses.pop(key, None)
            self._can_poll.pop(key, None)
        lost = self._mark_channel_absent(channel, keys)
        if bus is not None:
            try:
                bus.stop()
            except Exception:  # noqa: BLE001 - the device is already gone
                pass
        text = f"CAN {channel} disconnected ({reason}); its joints are read-only until it returns"
        if lost:
            text += (f". WAS DRIVING {lost}: they stay idle when it returns, "
                     "until the output is enabled again")
        self._note(channel, text, error=True)

    def _watch_can(self) -> None:
        """Follow every CAN channel in the map: open it when it comes up,
        drop it when it goes down or vanishes, reopen it when the adapter was
        replugged (a new netdev has a new ifindex, the old socket is dead).
        Dry run only reports, it never opens a socket."""
        for channel, keys in self._can_channels().items():
            state, ifindex = can_link(channel)
            if not self.live or self.recoil is None:
                self.channel_state[channel] = state
                continue
            with self.lock:
                if channel in self.channels:
                    if state != "up":
                        self._detach_channel(channel, f"link {state}")
                    elif ifindex != self.channel_ifindex.get(channel):
                        self._detach_channel(channel, "adapter replugged")
                if channel not in self.channels:
                    if state == "up":
                        self._attach_channel(channel, keys, ifindex)
                    else:
                        self._note(channel, self._waiting_text(channel, state))
                self.channel_state[channel] = "open" if channel in self.channels else state

    def _on_bus_error(self, key: str, error: Exception) -> bool:
        """A send/receive on bus `key` raised. If the link is gone, drop the
        channel now (True); otherwise it is a passing condition such as
        ENOBUFS on an unacknowledged bus, logged at most once a second."""
        channel = (self.config.get("bus") or {}).get(key)
        state, _ifindex = can_link(channel) if channel else ("missing", None)
        if channel in self.channels and state != "up":
            self._detach_channel(channel, f"link {state}: {error}")
            return True
        now = time.monotonic()
        if now - self._bus_error_logged.get(key, 0.0) > 1.0:
            self._bus_error_logged[key] = now
            self.get_logger().warn(f"CAN bus '{key}' ({channel}): {error}")
        return False

    def _can_idle(self, handle: JointHandle) -> None:
        bus = self.buses.get(handle.bus_key)
        if bus is None:
            return
        try:
            bus.set_mode(handle.device_id, self.recoil.Mode.IDLE)
        except Exception as error:  # noqa: BLE001 - see _on_bus_error
            self._on_bus_error(handle.bus_key, error)

    def _read_can_position(self, handle: JointHandle, timeout: float = 0.002) -> float | None:
        """Measured position over SDO with a bounded wait. recoil's own
        read_position_measured() blocks forever when the motor is absent, which
        would freeze the control loop; this never waits more than `timeout`."""
        bus = self.buses.get(handle.bus_key)
        if bus is None:
            return None
        try:
            value = bus._read_parameter_f32(  # noqa: SLF001 - the public wrapper has no timeout
                handle.device_id,
                self.recoil.Parameter.POSITION_CONTROLLER_POSITION_MEASURED,
                timeout=timeout)
        except Exception as error:  # noqa: BLE001 - see _on_bus_error
            self._on_bus_error(handle.bus_key, error)
            return None
        return None if value is None else handle.to_urdf(value)

    def _observe_can(self, key: str) -> None:
        """Read-only pass over one CAN bus: one not-yet-energised motor per
        cycle, round-robin. This is what lets you back-drive a joint with the
        output disabled and watch /joint_states to settle `direction` and
        `offset`, and it means enabling starts from the real position instead
        of a stale zero."""
        handles = [h for h in self.joints.values()
                   if h.kind == "can" and h.bus_key == key and not h.live]
        if not handles:
            return
        cursor = self._can_poll.get(key, 0)
        self._can_poll[key] = cursor + 1
        handle = handles[cursor % len(handles)]
        # A motor that has never answered (not wired yet) costs a full timeout
        # per poll, so ask it only every 100th turn to keep the loop at rate.
        if not handle.seen and handle.missed >= 3 and cursor % 100:
            return
        measured = self._read_can_position(handle)
        if measured is None:
            handle.missed += 1
            return
        if not handle.seen and self.zero_at_start:
            # First answer (e.g. power came on after the bridge): same rule
            # as at start-up. A motor that drops out and returns keeps it.
            self._take_reference(handle)
            measured = handle.to_urdf(handle.raw)
        if handle.missed and not handle.seen:
            self.get_logger().info(f"{handle.name}: motor {handle.device_id} answering")
        handle.missed = 0
        handle.seen = True
        handle.last_reply = time.monotonic()
        handle.measured = measured

    # --------------------------------------------------------------- services

    def _on_enable(self, request, response):
        if request.data:
            if self.estop.is_set():
                response.success = False
                response.message = "E-STOP latched; clear it before enabling"
                return response
            self._energise()
            response.message = f"output enabled ({'live' if self.live else 'dry run'})"
        else:
            self._deenergise()
            response.message = "output disabled"
        response.success = True
        self.get_logger().warn(response.message)
        return response

    def _on_estop(self, _request, response):
        self.estop.set()
        self._deenergise()
        response.success = True
        response.message = "E-STOP: all motors idle, output latched off"
        self.get_logger().error(response.message)
        return response

    def _on_reset_estop(self, _request, response):
        """Clear the latch. Separate from ~/enable on purpose: recovering from an
        E-STOP should be a deliberate act, not a side effect of re-enabling."""
        if not self.estop.is_set():
            response.success = True
            response.message = "no E-STOP was latched"
            return response
        self.estop.clear()
        for handle in self.joints.values():
            handle.fault = None
            handle.missed = 0
        response.success = True
        response.message = "E-STOP cleared; output is still disabled"
        self.get_logger().warn(response.message)
        return response

    def _on_zero(self, _request, response):
        """Take the arm's current pose as URDF zero: recompute `offset` for
        every CAN joint that is answering, apply it now, and print the yaml to
        make it permanent. Same idea as bimanual.py, which captures its offsets
        at start-up -- but here it is deliberate, with the output disabled and
        the arm physically posed at zero first. Refused while anything is live."""
        if self.zero_at_start:
            response.success = False
            response.message = ("refused: --zero-at-start fixes each motor's first pose as 0 "
                                "for the whole run")
            return response
        if self.output_enabled:
            response.success = False
            response.message = "disable the output first; zeroing a live joint would jump it"
            return response
        if not self.live:
            response.success = False
            response.message = "dry run: there is nothing to zero"
            return response
        lines, skipped = [], []
        with self.lock:
            for handle in self.joints.values():
                if handle.kind != "can":
                    continue
                if not handle.seen:
                    skipped.append(handle.name)
                    continue
                handle.offset = handle.offset_for_zero_here()
                handle.measured = handle.to_urdf(handle.raw)
                handle.command = handle.measured
                handle.fault = None if handle.fault == "OUT_OF_RANGE" else handle.fault
                lines.append(f"  {handle.name}: offset: {handle.offset:.4f}")
        text = ("applied in memory only -- paste into hardware_joints.yaml:\n"
                + "\n".join(lines))
        if skipped:
            text += f"\n(not answering, untouched: {skipped})"
        response.success = bool(lines)
        response.message = text
        self.get_logger().warn(text)
        return response

    @staticmethod
    def _side(name: str) -> str | None:
        return "left" if "_left_" in name else "right" if "_right_" in name else None

    def _arm_active(self, handle: JointHandle) -> bool:
        return self.arm_active.get(self._side(handle.name), True)

    def _on_set_arms(self, message: String) -> None:
        """{"left": bool, "right": bool}: which arms may be driven."""
        try:
            data = json.loads(message.data)
        except json.JSONDecodeError:
            self.get_logger().error("set_arms: bad JSON")
            return
        switched_on = False
        with self.lock:
            for side in ("left", "right"):
                if side not in data:
                    continue
                active = bool(data[side])
                if active == self.arm_active[side]:
                    continue
                self.arm_active[side] = active
                self.get_logger().warn(f"{side} arm {'selected' if active else 'deselected'}")
                if active:
                    switched_on = True
                    continue
                for handle in self.joints.values():
                    if self._side(handle.name) == side and handle.live:
                        if self.live:
                            if handle.kind == "wrist":
                                self.wrist_buses[handle.bus_key].write_torque(handle.device_id, False)
                            else:
                                self._can_idle(handle)
                        handle.live = False
                        handle.stream_target = None
        # An arm ticked back on while output is on starts holding where it is.
        if switched_on and self.output_enabled and not self.estop.is_set():
            self._energise()

    def _energise(self) -> None:
        with self.lock:
            for handle in self.joints.values():
                if not handle.enabled or handle.fault or handle.live or not self._arm_active(handle):
                    continue
                # Start from where the joint actually is, so enabling cannot
                # snap it to a stale target.
                if self.live and handle.kind == "wrist":
                    bus = self.wrist_buses[handle.bus_key]
                    ticks = bus.read_position(handle.device_id)
                    if ticks is None:
                        handle.fault = "LOST"
                        self.get_logger().error(
                            f"{handle.name}: servo did not answer at enable; skipped")
                        continue
                    handle.measured = handle.from_ticks(ticks)
                    handle.command = handle.measured
                    if not bus.write_torque(handle.device_id, True):
                        handle.fault = "LOST"
                        self.get_logger().error(
                            f"{handle.name}: torque-on not acknowledged; skipped")
                        continue
                    handle.live = True
                    continue
                if self.live:
                    measured = self._read_can_position(handle, timeout=0.02)
                    if measured is None:
                        handle.fault = "LOST"
                        self.get_logger().error(
                            f"{handle.name}: motor did not answer at enable; skipped")
                        continue
                    handle.measured = measured
                    # 3 deg of slack: with --zero-at-start a joint whose URDF
                    # limit is exactly 0 (elbow pitch) starts on the limit, and
                    # noise or a little sag since power-on must not read as
                    # out of range. The first streamed goal is clamped back in.
                    slack = math.radians(3.0)
                    if not (handle.lower - slack <= measured <= handle.upper + slack):
                        # Enabling here would be legal for the motor but MoveIt
                        # will refuse to start from outside the URDF limits and
                        # the very first clamped target would yank the joint
                        # towards the limit. Almost always an unset offset.
                        handle.fault = "OUT_OF_RANGE"
                        self.get_logger().error(
                            f"{handle.name}: measured {measured:+.3f} rad is outside "
                            f"URDF limits {handle.lower:+.2f}..{handle.upper:+.2f}; "
                            "fix `offset` in hardware_joints.yaml before enabling")
                        continue
                handle.command = handle.measured
                handle.stream_target = None
                if self.live:
                    bus = self.buses.get(handle.bus_key)
                    if bus is None:       # adapter went away since the read above
                        continue
                    gains = {**self.gains, **handle.gains}
                    try:
                        bus.set_mode(handle.device_id, self.recoil.Mode.IDLE)
                        bus.write_position_kp(handle.device_id, gains["position_kp"])
                        bus.write_position_kd(handle.device_id, gains["position_kd"])
                        bus.write_torque_limit(handle.device_id, gains["torque_limit"])
                        bus.feed(handle.device_id)
                        bus.set_mode(handle.device_id, self.recoil.Mode.POSITION)
                    except Exception as error:  # noqa: BLE001 - see _on_bus_error
                        handle.fault = "LOST"
                        self.get_logger().error(f"{handle.name}: enable failed on the bus: {error}")
                        self._on_bus_error(handle.bus_key, error)
                        self._can_idle(handle)
                        continue
                handle.live = True
            self.output_enabled = True

    def _deenergise(self) -> None:
        with self.lock:
            for handle in self.joints.values():
                if handle.live and self.live:
                    if handle.kind == "wrist":
                        self.wrist_buses[handle.bus_key].write_torque(handle.device_id, False)
                    else:
                        self._can_idle(handle)
                handle.live = False
                handle.stream_target = None
            self.output_enabled = False

    # ------------------------------------------------------------ control loop

    def _on_joint_command(self, message: JointState) -> None:
        if self.estop.is_set() or not self.output_enabled:
            if not self._stream_warned:
                self.get_logger().warn(
                    "joint_command ignored: output is disabled or E-STOP is latched")
                self._stream_warned = True
            return
        self._stream_warned = False
        margin = float(self.safety["limit_margin"])
        with self.lock:
            for name, position in zip(message.name, message.position):
                handle = self.joints.get(name)
                if handle is None or not handle.enabled or not handle.live or handle.fault:
                    continue
                if math.isfinite(position):
                    handle.stream_target = handle.clamp(position, margin)

    def _follow_stream(self) -> None:
        """Walk every streamed joint one slew-limited step towards its goal."""
        max_step = float(self.safety["max_joint_speed"]) * self.period
        flaky = int(self.safety["link_flaky_cycles"])
        for handle in self.joints.values():
            if handle.stream_target is None or not handle.live or handle.fault:
                continue
            if handle.missed >= flaky:
                continue
            step = handle.stream_target - handle.command
            handle.command += min(max(step, -max_step), max_step)

    def _tick(self) -> None:
        if self.estop.is_set():
            return
        with self.lock:
            self._follow_stream()
            for handle in self.joints.values():
                if handle.kind == "wrist":
                    continue          # wrist buses are serviced per bus below
                if self.live and handle.live:
                    self._exchange(handle)
                elif not self.live:
                    # Dry run: the joint follows its command exactly.
                    handle.measured = handle.command
            if self.live:
                for key in list(self.buses):     # a failing bus may drop out mid-loop
                    if key in self.buses:
                        self._observe_can(key)
                for key in self.wrist_buses:
                    self._exchange_wrist_bus(key)
            else:
                for handle in self.joints.values():
                    if handle.kind == "wrist":
                        handle.measured = handle.command
        self._publish()

    def _exchange_wrist_bus(self, key: str) -> None:
        """One cycle on one serial bus: every live goal in a single SYNC_WRITE,
        then one present-position read (round-robin), so the bus never blocks
        the 100 Hz loop for more than one reply."""
        bus = self.wrist_buses[key]
        handles = [h for h in self.joints.values()
                   if h.kind == "wrist" and h.bus_key == key]
        if not handles:
            return
        goals = {}
        for handle in handles:
            if handle.live:
                speed = handle.max_speed_ticks or int(
                    float(self.safety["max_joint_speed"]) * handle.ticks_per_rad
                    * self.wrist_speed_default)
                goals[handle.device_id] = (handle.to_ticks(handle.command), speed)
        bus.sync_write_goals(goals)

        index = self._wrist_poll.get(key, 0) % len(handles)
        self._wrist_poll[key] = index + 1
        handle = handles[index]
        ticks = bus.read_position(handle.device_id)
        if ticks is None:
            handle.missed += 1
            # Polling is 1-in-N, so scale the loss thresholds accordingly.
            if handle.live and handle.missed * len(handles) >= int(self.safety["pdo_loss_threshold"]):
                handle.fault = "LOST"
                handle.live = False
                bus.write_torque(handle.device_id, False)
                self.get_logger().error(
                    f"{handle.name}: no reply for {handle.missed} polls, disabled")
            return
        handle.missed = 0
        handle.measured = handle.from_ticks(ticks)

    def _exchange(self, handle: JointHandle) -> None:
        bus = self.buses.get(handle.bus_key)
        if bus is None:
            return
        try:
            bus.transmit_pdo_2(handle.device_id,
                               position_target=handle.to_actuator(handle.command),
                               velocity_target=0.0)
            measured, _velocity = bus.receive_pdo_2(handle.device_id)
        except Exception as error:  # noqa: BLE001 - see _on_bus_error
            if self._on_bus_error(handle.bus_key, error):
                return            # channel dropped; the joint is already LOST
            measured = None
        if measured is None:
            handle.missed += 1
            if handle.missed >= int(self.safety["pdo_loss_threshold"]):
                handle.fault = "LOST"
                handle.live = False
                self._can_idle(handle)
                self.get_logger().error(
                    f"{handle.name}: no PDO reply for "
                    f"{handle.missed} cycles, disabled"
                )
            return
        handle.missed = 0
        handle.last_reply = time.monotonic()
        handle.measured = handle.to_urdf(measured)

    def _publish(self) -> None:
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        with self.lock:
            message.name = list(self.joints)
            message.position = [self.joints[n].measured for n in message.name]
        self.publisher.publish(message)

    # ---------------------------------------------------------------- action

    def _reject(self, goal_handle, code, reason):
        goal_handle.abort()
        result = FollowJointTrajectory.Result()
        result.error_code = code
        result.error_string = reason
        self.get_logger().error(f"trajectory rejected: {reason}")
        return result

    def _execute(self, goal_handle):
        trajectory = goal_handle.request.trajectory
        names = list(trajectory.joint_names)
        points = list(trajectory.points)
        result = FollowJointTrajectory.Result()

        unmapped = [n for n in names if n not in self.joints]
        if unmapped:
            return self._reject(
                goal_handle, FollowJointTrajectory.Result.INVALID_JOINTS,
                f"joints are not in the hardware map: {unmapped}")

        blocked = [n for n in names if not self.joints[n].enabled]
        if blocked:
            return self._reject(
                goal_handle, FollowJointTrajectory.Result.INVALID_JOINTS,
                f"joints are mapped but disabled in hardware_joints.yaml: {blocked}")

        faulted = [n for n in names if self.joints[n].fault]
        if faulted:
            return self._reject(
                goal_handle, FollowJointTrajectory.Result.INVALID_JOINTS,
                f"joints are faulted and need re-enabling: {faulted}")

        if self.estop.is_set():
            return self._reject(goal_handle, -1, "E-STOP is latched")
        if not self.output_enabled:
            return self._reject(goal_handle, -1,
                                "output is disabled; call ~/enable first")
        if not points:
            goal_handle.succeed()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            return result

        margin = float(self.safety["limit_margin"])
        max_step = float(self.safety["max_joint_speed"]) * self.period
        flaky = int(self.safety["link_flaky_cycles"])

        self.get_logger().info(
            f"executing {len(points)} points over "
            f"{duration_to_seconds(points[-1].time_from_start):.2f}s "
            f"for {len(names)} joints"
        )

        start = self.get_clock().now()
        rate = self.create_rate(1.0 / self.period)
        previous_time = 0.0
        with self.lock:
            previous = [self.joints[n].command for n in names]
            for name in names:
                self.joints[name].stream_target = None

        for point in points:
            target_time = duration_to_seconds(point.time_from_start)
            segment = target_time - previous_time

            while rclpy.ok():
                if self.estop.is_set():
                    return self._reject(goal_handle, -1, "E-STOP during execution")
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    result.error_string = "cancelled"
                    return result

                elapsed = (self.get_clock().now() - start).nanoseconds * 1e-9
                if elapsed >= target_time:
                    break

                alpha = 1.0 if segment <= 0.0 else (elapsed - previous_time) / segment
                alpha = min(max(alpha, 0.0), 1.0)
                with self.lock:
                    for index, name in enumerate(names):
                        handle = self.joints[name]
                        if handle.fault:
                            return self._reject(
                                goal_handle, -1,
                                f"{name} faulted mid-trajectory ({handle.fault})")
                        # A flaky link freezes the goal rather than letting it
                        # walk away from where the joint actually is.
                        if handle.missed >= flaky:
                            continue
                        wanted = (previous[index]
                                  + alpha * (point.positions[index] - previous[index]))
                        wanted = handle.clamp(wanted, margin)
                        step = wanted - handle.command
                        step = min(max(step, -max_step), max_step)
                        handle.command += step
                rate.sleep()

            previous = list(point.positions)
            previous_time = target_time

        goal_handle.succeed()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result


def load_config(path: Path | None) -> dict:
    if path is None:
        path = Path(get_package_share_directory(PACKAGE)) / "config" / "hardware_joints.yaml"
    if not path.is_file():
        raise SystemExit(f"joint map not found: {path}")
    return yaml.safe_load(path.read_text())


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--live", action="store_true",
                        help="open the CAN buses and wrist serial ports. "
                             "Without this nothing is touched.")
    parser.add_argument("--config", type=Path, default=None)
    parser.add_argument("--buses", choices=("all", "can", "wrist"), default="all",
                        help="with --live: which buses to open. Joints on a bus "
                             "that stays closed are read-only.")
    parser.add_argument("--description-node", default="/move_group",
                        help="node whose robot_description parameter supplies the "
                             "joint limits (/robot_state_publisher without MoveIt)")
    parser.add_argument("--limits-file", type=Path, default=None,
                        help="yaml of operator travel limits, {joint: [lower_deg, upper_deg]}; "
                             "arm_panel.py writes it")
    parser.add_argument("--zero-at-start", action="store_true",
                        help="with --live: take each motor's position when it first answers "
                             "(at start-up, or later if powered later) as URDF zero, for the "
                             "whole run (overrides the yaml offsets)")
    parser.add_argument("--reference-file", type=Path, default=None,
                        help="with --zero-at-start: where the power-on readings are kept so "
                             "a bridge restart keeps the same zero")
    args, _ = parser.parse_known_args()

    config = load_config(args.config)
    rclpy.init()
    node = HardwareBridge(config, live=args.live, buses=args.buses,
                          description_node=args.description_node,
                          limits_file=args.limits_file,
                          zero_at_start=args.zero_at_start,
                          reference_file=args.reference_file)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._deenergise()
        for bus in node.wrist_buses.values():
            bus.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
