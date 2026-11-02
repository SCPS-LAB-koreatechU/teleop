"""Per-joint position control of the BHL arm actuators, without ROS.

The control flow follows Unitree's g1_arm7_sdk_dds_example.cpp:

  * read the current joint positions first, and start every command there
  * move each joint towards its goal by at most `max_speed_deg * dt` per step
    (the example's max_joint_velocity / max_joint_delta), at a fixed rate
  * on stop, hand the joints back gently (here: DAMPING, then IDLE)

and the actuator side follows the upstream Bimanual class: per joint, IDLE ->
write kp / kd / torque limit -> feed the watchdog -> POSITION, the pose at
start taken as joint zero, then one PDO_2 exchange per joint per cycle.

On top of that, the safety behaviour this lab needed on its own arms:

  * a joint that does not answer the ping is never enabled
  * a joint that stops answering for `max_missed` cycles is dropped (fault LOST)
  * a joint lagging its command by more than `max_tracking_error_deg` stops
    every joint (fault TRACKING) -- a reversed direction or a bad encoder shows
    up here instead of as a runaway
  * a joint set to `estop: range` skips that check and instead stops every
    joint when it leaves its limits by more than `range_margin_deg` (fault RANGE)
  * dry_run (default) opens no bus: the measured angle simply follows the command
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass

from .config import ArmConfig, JointSpec
from .recoil_can import Mode, Param, RecoilBus


@dataclass
class JointState:
    spec: JointSpec
    zero: float = 0.0               # motor reading at joint 0
    measured_deg: float = 0.0
    velocity_dps: float = 0.0
    command_deg: float = 0.0        # the slew-limited setpoint actually sent
    goal_deg: float = 0.0           # where the user asked it to go
    live: bool = False              # position mode, being commanded
    fault: str = ""
    missed: int = 0
    last_reply: float = 0.0


class ArmController:
    def __init__(self, config: ArmConfig, dry_run: bool = True, bus_factory=RecoilBus,
                 max_missed: int = 25, max_tracking_error_deg: float = 20.0) -> None:
        self.config = config
        self.dry_run = dry_run
        self.max_missed = max_missed
        self.max_tracking_error_deg = max_tracking_error_deg
        self.max_speed_deg = config.max_speed_deg
        self.joints = {spec.name: JointState(spec=spec) for spec in config.joints}
        self.buses: dict[str, RecoilBus] = {}
        self.estopped = False
        self.message = ""
        self._last_feed = 0.0
        if not dry_run:
            for key, channel in config.buses.items():
                if any(j.bus == key and j.enabled for j in config.joints) and channel not in \
                        {b.channel for b in self.buses.values()}:
                    self.buses[key] = bus_factory(channel, reply_timeout=config.reply_timeout)
            # several bus keys may share one channel (one adapter for both arms)
            for key, channel in config.buses.items():
                if key not in self.buses:
                    shared = next((b for b in self.buses.values() if b.channel == channel), None)
                    if shared is not None:
                        self.buses[key] = shared

    # ------------------------------------------------------------------ helpers
    def _bus(self, state: JointState) -> RecoilBus | None:
        return self.buses.get(state.spec.bus)

    def _select(self, names) -> list[JointState]:
        if names is None:
            return [s for s in self.joints.values() if s.spec.enabled]
        unknown = [n for n in names if n not in self.joints]
        if unknown:
            raise KeyError(f"unknown joints: {unknown}")
        return [self.joints[n] for n in names if self.joints[n].spec.enabled]

    @property
    def any_live(self) -> bool:
        return any(s.live for s in self.joints.values())

    # ----------------------------------------------------------------- lifecycle
    def enable(self, names=None) -> list[str]:
        """Put joints in position mode holding where they are. Returns the joints
        that came up; the others carry a fault saying why."""
        if self.estopped:
            self.message = "E-STOP is latched; reset it first"
            return []
        up = []
        for state in self._select(names):
            if state.live:
                up.append(state.spec.name)
                continue
            state.fault = ""
            state.missed = 0
            if self.dry_run:
                if self.config.zero == "config" and state.spec.motor_zero is not None:
                    state.zero = state.spec.motor_zero
                state.command_deg = state.goal_deg = state.measured_deg
                state.live = True
                up.append(state.spec.name)
                continue
            bus = self._bus(state)
            spec = state.spec
            if bus is None or not bus.ping(spec.device_id):
                state.fault = "NO_REPLY"
                continue
            motor = bus.read_f32(spec.device_id, Param.POSITION_MEASURED)
            if motor is None:
                state.fault = "NO_POSITION"
                continue
            state.zero = motor if self.config.zero == "start" else float(spec.motor_zero)
            state.measured_deg = math.degrees(spec.to_joint(motor, state.zero))
            if not spec.lower_deg - 5.0 <= state.measured_deg <= spec.upper_deg + 5.0:
                # Enabling would pull the joint straight to its limit.
                state.fault = f"OUT_OF_RANGE {state.measured_deg:+.1f} deg"
                continue
            state.command_deg = state.goal_deg = spec.clamp_deg(state.measured_deg)
            bus.set_mode(spec.device_id, Mode.IDLE)
            bus.write_gains(spec.device_id, spec.kp, spec.kd, spec.torque_limit)
            bus.feed(spec.device_id)
            bus.set_mode(spec.device_id, Mode.POSITION)
            reply = bus.exchange_pdo_2(spec.device_id, spec.to_motor(math.radians(state.command_deg), state.zero))
            if reply is None:
                bus.set_mode(spec.device_id, Mode.IDLE)
                state.fault = "NO_PDO_REPLY"
                continue
            state.last_reply = time.monotonic()
            state.live = True
            up.append(spec.name)
        self.message = f"enabled {len(up)} joint(s)"
        return up

    def disable(self, names=None, damping: bool = True) -> None:
        """Stop commanding. DAMPING lets the arm settle slowly instead of dropping."""
        for state in self._select(names) if names is not None else list(self.joints.values()):
            if state.live and not self.dry_run:
                bus = self._bus(state)
                if bus is not None:
                    bus.set_mode(state.spec.device_id, Mode.DAMPING if damping else Mode.IDLE)
            state.live = False
            state.goal_deg = state.command_deg = state.measured_deg
        self.message = "disabled"

    def estop(self, reason: str = "E-STOP") -> None:
        self.disable(damping=True)
        self.estopped = True
        self.message = reason

    def reset_estop(self) -> None:
        self.estopped = False
        self.message = "E-STOP cleared; joints are still disabled"

    def close(self) -> None:
        if not self.dry_run:
            for state in self.joints.values():
                bus = self._bus(state)
                if bus is not None and state.spec.enabled:
                    try:
                        bus.set_mode(state.spec.device_id, Mode.IDLE)
                    except Exception:  # noqa: BLE001
                        pass
            for bus in {id(b): b for b in self.buses.values()}.values():
                bus.close()
        for state in self.joints.values():
            state.live = False

    # ------------------------------------------------------------------ targets
    def set_goal(self, name: str, degrees: float) -> float:
        """Clamp to the joint's limits and set it as the goal. Returns the clamped value."""
        state = self.joints[name]
        state.goal_deg = state.spec.clamp_deg(float(degrees))
        return state.goal_deg

    def set_goals(self, goals: dict[str, float]) -> dict[str, float]:
        return {name: self.set_goal(name, value) for name, value in goals.items()}

    def settled(self, tolerance_deg: float = 0.5) -> bool:
        return all(abs(s.command_deg - s.goal_deg) <= 1e-6 and abs(s.measured_deg - s.goal_deg) <= tolerance_deg
                   for s in self.joints.values() if s.live)

    # --------------------------------------------------------------------- loop
    def step(self, dt: float) -> None:
        """One control cycle: slew every live command towards its goal, exchange PDOs."""
        if self.estopped:
            return
        max_step = self.max_speed_deg * dt
        now = time.monotonic()
        feed = now - self._last_feed >= self.config.heartbeat_period
        if feed:
            self._last_feed = now
        for state in self.joints.values():
            if not state.live:
                continue
            delta = state.goal_deg - state.command_deg
            state.command_deg += max(-max_step, min(max_step, delta))
            if self.dry_run:
                previous = state.measured_deg
                state.measured_deg = state.command_deg
                state.velocity_dps = (state.measured_deg - previous) / dt if dt > 0 else 0.0
                continue
            spec, bus = state.spec, self._bus(state)
            if feed:
                bus.feed(spec.device_id)
            reply = bus.exchange_pdo_2(spec.device_id, spec.to_motor(math.radians(state.command_deg), state.zero))
            if reply is None:
                state.missed += 1
                if state.missed >= self.max_missed:
                    state.fault = "LOST"
                    state.live = False
                    bus.set_mode(spec.device_id, Mode.DAMPING)
                continue
            state.missed = 0
            state.last_reply = now
            state.measured_deg = math.degrees(spec.to_joint(reply[0], state.zero))
            state.velocity_dps = math.degrees(state.spec.direction * reply[1] / spec.gear_ratio)
            if spec.estop == "range":
                if not spec.lower_deg - spec.range_margin_deg <= state.measured_deg <= spec.upper_deg + spec.range_margin_deg:
                    state.fault = "RANGE"
                    self.estop(f"{spec.name}: {state.measured_deg:+.1f} deg outside "
                               f"[{spec.lower_deg:+.1f}, {spec.upper_deg:+.1f}] +- {spec.range_margin_deg:.1f} deg "
                               f"-- stopped everything")
                    return
            elif abs(state.measured_deg - state.command_deg) > self.max_tracking_error_deg:
                state.fault = "TRACKING"
                self.estop(f"{spec.name}: {state.measured_deg:+.1f} deg vs command "
                           f"{state.command_deg:+.1f} deg -- stopped everything")
                return

    def status(self) -> dict:
        return {
            "dry_run": self.dry_run, "estop": self.estopped, "message": self.message,
            "max_speed_deg": self.max_speed_deg,
            "joints": {name: {
                "bus": s.spec.bus, "device_id": s.spec.device_id, "enabled": s.spec.enabled,
                "live": s.live, "fault": s.fault,
                "measured_deg": round(s.measured_deg, 3), "command_deg": round(s.command_deg, 3),
                "goal_deg": round(s.goal_deg, 3),
                "limits_deg": [s.spec.lower_deg, s.spec.upper_deg],
            } for name, s in self.joints.items()},
        }
