# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.
# Multi-motor slider GUI for one arm (up to 5 motors on one CAN bus).
#
# Safety design:
#   - each motor's slider commands a RELATIVE offset from the position captured
#     at enable time (slider 0 = wherever the joint was when you enabled it)
#   - commanded target is slew-rate limited, so yanking a slider cannot demand
#     an instantaneous jump; the goal freezes while PDO replies are missing so
#     it cannot walk away during a link outage
#   - E-STOP is handled out-of-band (event flag, not the command queue),
#     discards all queued work, and sets every motor to IDLE
#   - the control loop is exception-guarded: on any bus error it IDLEs all
#     motors, marks itself dead, and the GUI shows a red banner
#   - a motor that stops answering PDOs for ~1 s is auto-disabled (status LOST)
#     and must be explicitly re-enabled, which re-captures its center
#   - widget states (checkbox, slider) are re-derived from the actual motor
#     state every refresh tick, so they cannot stay out of sync with reality
#
# Example:
#   python gui_arm_control.py -c can0 -i 1 2 3 4 5
#
# All CAN traffic happens in a single background thread; the GUI communicates
# with it through a command queue, an E-STOP event flag, and shared MotorState
# objects (each field is written by exactly one thread).

import time
import queue
import argparse
import threading
import tkinter as tk
from tkinter import ttk

from loop_rate_limiters import RateLimiter

import berkeley_humanoid_lite_lowlevel.recoil as recoil
from berkeley_humanoid_lite_lowlevel.recoil import Parameter, Mode


parser = argparse.ArgumentParser()
parser.add_argument("-c", "--channel", type=str, default="can0")
parser.add_argument("-i", "--ids", type=int, nargs="+", default=None,
                    help="custom CAN device IDs (max 5); omit to use the arm presets")
parser.add_argument("--kp", type=float, default=0.7)
parser.add_argument("--kd", type=float, default=0.03)
parser.add_argument("--torque-limit", type=float, default=0.8)
parser.add_argument("--range", type=float, default=3.14,
                    help="slider range [rad], slider spans +/- this value")
parser.add_argument("--max-speed", type=float, default=3.0,
                    help="slew rate limit for commanded target [rad/s]")
parser.add_argument("--rate", type=float, default=100.0, help="control loop rate [Hz]")
args = parser.parse_args()

# selectable motor groups: left arm = odd IDs, right arm = even IDs;
# passing -i adds a "custom" group and starts on it
ARM_GROUPS: dict[str, list[int]] = {
    "left": [1, 3, 5, 7, 9],
    "right": [2, 4, 6, 8, 10],
}
if args.ids:
    if len(args.ids) > 5:
        raise SystemExit("[FAIL] at most 5 motors are supported")
    ARM_GROUPS["custom"] = args.ids
    INITIAL_GROUP = "custom"
else:
    INITIAL_GROUP = "left"

ALL_IDS = list(dict.fromkeys(dev for devs in ARM_GROUPS.values() for dev in devs))

ERROR_POLL_PERIOD = 2.0    # [s] per-motor error register poll while enabled
AUTO_PING_PERIOD = 1.0     # [s] round-robin ping of not-enabled motors
LINK_FLAKY_CYCLES = 5      # missed replies before the commanded goal freezes
PDO_LOSS_THRESHOLD = 100   # missed replies before the motor is auto-disabled

# gains applied to live motors are clamped to these ranges (a kp typo of 7
# instead of 0.7, or a negative kd, must not reach the hardware)
GAIN_LIMITS = {"kp": (0.0, 5.0), "kd": (0.0, 0.5), "torque_limit": (0.0, 2.0)}


def clamp_gain(name: str, value: float) -> float:
    lo, hi = GAIN_LIMITS[name]
    return min(max(value, lo), hi)


for _name, _attr in (("kp", "kp"), ("kd", "kd"), ("torque_limit", "torque_limit")):
    _v = getattr(args, _attr)
    _c = clamp_gain(_name, _v)
    if _c != _v:
        print(f"[WARN] {_name}={_v} outside safe range {GAIN_LIMITS[_name]}, clamped to {_c}")
        setattr(args, _attr, _c)


class MotorState:
    def __init__(self, dev: int):
        self.dev = dev
        self.online = False
        self.enabled = False
        self.fault = None        # short reason shown after auto-disable ("LOST")
        self.center = 0.0        # position captured at enable time [rad]
        self.offset = 0.0        # slider value [rad], written by GUI thread
        self.command = 0.0       # slew-limited absolute target [rad]
        self.measured_pos = None
        self.measured_vel = None
        self.error_code = None
        self.timeouts = 0        # consecutive missed PDO replies
        # request/ack sequence for enable/disable/estop commands:
        # cmd_pending is written only by the GUI thread, cmd_done only by the
        # control thread, so no lock is needed
        self.cmd_pending = 0
        self.cmd_done = 0

    @property
    def busy(self) -> bool:
        return self.cmd_pending != self.cmd_done


class ControlThread(threading.Thread):
    """Owns the CAN bus. All bus I/O happens here."""

    def __init__(self, bus: recoil.Bus, motors: dict[int, MotorState]):
        super().__init__(daemon=True)
        self.bus = bus
        self.motors = motors
        self.commands = queue.Queue()
        self.gains = {"kp": args.kp, "kd": args.kd, "torque_limit": args.torque_limit}
        self.running = True
        self.estop_flag = threading.Event()
        self.dead = None         # exception text once the control thread has died
        # only motors in the currently displayed group get auto-pinged;
        # replaced wholesale by the GUI thread on group switch (atomic swap)
        self.visible_devs = set(motors)
        self._last_error_poll = {dev: 0.0 for dev in motors}
        self._last_auto_ping = 0.0
        self._auto_ping_idx = 0

    # --- called from the GUI thread ---

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
        # emergency use only: on_close found this thread stuck and calls this
        # from the GUI thread (socketcan sends are atomic per frame)
        for dev in self.motors:
            try:
                self.bus.set_mode(dev, Mode.IDLE)
            except Exception:
                pass

    # --- bus operations, executed only inside this thread ---

    def _flush_rx(self):
        while self.bus.receive(timeout=0) is not None:
            pass

    def _mark_online(self, dev: int, ok: bool):
        m = self.motors[dev]
        m.online = ok
        if ok:
            m.fault = None

    def _do_enable(self, dev: int):
        m = self.motors[dev]
        try:
            if self.estop_flag.is_set():
                return
            m.fault = None
            if not self.bus.ping(dev, timeout=0.05):
                m.online = False
                return
            m.online = True
            self.bus._write_parameter_u32(dev, Parameter.ERROR, 0)
            self.bus.write_position_kp(dev, self.gains["kp"])
            self.bus.write_position_kd(dev, self.gains["kd"])
            self.bus.write_torque_limit(dev, self.gains["torque_limit"])
            # a stale reply from an earlier timed-out SDO read must never be
            # mistaken for the center position (SDO replies carry no parameter
            # echo): flush the socket, then read twice and require agreement
            self._flush_rx()
            p1 = self.bus._read_parameter_f32(dev, Parameter.POSITION_CONTROLLER_POSITION_MEASURED, timeout=0.2)
            p2 = self.bus._read_parameter_f32(dev, Parameter.POSITION_CONTROLLER_POSITION_MEASURED, timeout=0.2)
            if p1 is None or p2 is None or abs(p1 - p2) > 0.5:
                return
            if self.estop_flag.is_set():
                return
            m.center = p2
            m.command = p2
            m.offset = 0.0
            m.measured_pos = p2
            m.timeouts = 0
            self.bus.set_mode(dev, Mode.POSITION)
            self.bus.feed(dev)
            m.enabled = True
        finally:
            m.cmd_done += 1

    def _do_disable(self, dev: int):
        m = self.motors[dev]
        try:
            m.enabled = False
            self.bus.set_mode(dev, Mode.IDLE)
        finally:
            m.cmd_done += 1

    def _do_estop(self):
        # discard queued work first: a stale enable must not fire after E-STOP
        try:
            while True:
                cmd, dev, _payload = self.commands.get_nowait()
                if cmd in ("enable", "disable"):
                    self.motors[dev].cmd_done += 1
        except queue.Empty:
            pass
        for m in self.motors.values():
            m.enabled = False
        # send twice for good measure
        for _ in range(2):
            for dev in self.motors:
                self.bus.set_mode(dev, Mode.IDLE)
        for m in self.motors.values():
            m.cmd_done += 1
        self.estop_flag.clear()

    def _do_apply_gains(self, payload):
        self.gains.update(payload)
        for dev, m in self.motors.items():
            if m.enabled:
                self.bus.write_position_kp(dev, self.gains["kp"])
                self.bus.write_position_kd(dev, self.gains["kd"])
                self.bus.write_torque_limit(dev, self.gains["torque_limit"])

    def _do_ping_all(self):
        for dev, m in self.motors.items():
            if m.enabled or dev not in self.visible_devs:
                continue  # liveness of enabled motors is judged by PDO replies
            self._mark_online(dev, self.bus.ping(dev, timeout=0.05))
            if self.estop_flag.is_set():
                break

    def _do_ping_one(self, dev: int):
        if self.motors[dev].enabled:
            return
        self._mark_online(dev, self.bus.ping(dev, timeout=0.1))

    def run(self):
        rate = RateLimiter(frequency=args.rate)
        dt = 1.0 / args.rate
        max_step = args.max_speed * dt

        try:
            while self.running:
                if self.estop_flag.is_set():
                    self._do_estop()

                # process pending GUI commands (E-STOP preempts them)
                try:
                    while not self.estop_flag.is_set():
                        cmd, dev, payload = self.commands.get_nowait()
                        if cmd == "enable":
                            self._do_enable(dev)
                        elif cmd == "disable":
                            self._do_disable(dev)
                        elif cmd == "gains":
                            self._do_apply_gains(payload)
                        elif cmd == "ping":
                            self._do_ping_all()
                        elif cmd == "ping_one":
                            self._do_ping_one(dev)
                except queue.Empty:
                    pass

                if self.estop_flag.is_set():
                    continue  # handled at the top of the next iteration

                active = [m for m in self.motors.values() if m.enabled]

                # slew-limit each command toward center + slider offset;
                # transmit all targets first, then collect replies (pipelined)
                for m in active:
                    if m.timeouts <= LINK_FLAKY_CYCLES:
                        goal = m.center + m.offset
                        delta = goal - m.command
                        if delta > max_step:
                            delta = max_step
                        elif delta < -max_step:
                            delta = -max_step
                        m.command += delta
                    # else: replies are missing, hold the frozen command so the
                    # goal cannot walk away during the outage
                    self.bus.transmit_pdo_2(m.dev, position_target=m.command, velocity_target=0.0)

                for m in active:
                    r = self.bus.receive_pdo_2(m.dev)
                    if r is not None and r[0] is not None:
                        m.measured_pos, m.measured_vel = r
                        m.timeouts = 0
                        m.online = True
                    else:
                        m.timeouts += 1
                        if m.timeouts > PDO_LOSS_THRESHOLD:
                            # link lost: stop driving this motor; an explicit
                            # re-enable re-captures its center position
                            m.enabled = False
                            m.online = False
                            m.fault = "LOST"
                            self.bus.set_mode(m.dev, Mode.IDLE)

                now = time.monotonic()

                # low-rate error register poll
                for m in active:
                    if m.enabled and now - self._last_error_poll[m.dev] > ERROR_POLL_PERIOD:
                        self._last_error_poll[m.dev] = now
                        err = self.bus._read_parameter_u32(m.dev, Parameter.ERROR, timeout=0.05)
                        if err is not None:
                            m.error_code = err

                # round-robin auto-ping so connection LEDs stay fresh; only
                # motors not under closed-loop control
                if now - self._last_auto_ping > AUTO_PING_PERIOD:
                    self._last_auto_ping = now
                    idle_devs = [d for d, mm in self.motors.items()
                                 if not mm.enabled and d in self.visible_devs]
                    if idle_devs:
                        dev_to_ping = idle_devs[self._auto_ping_idx % len(idle_devs)]
                        self._auto_ping_idx += 1
                        self._mark_online(dev_to_ping, self.bus.ping(dev_to_ping, timeout=0.03))

                rate.sleep()

        except Exception as e:  # noqa: BLE001 - any bus error must not skip the IDLE fallback
            self.dead = f"{type(e).__name__}: {e}"
            print(f"[FATAL] control thread died: {self.dead}")
        finally:
            # last line of defense: whatever happened, try to leave every
            # motor in IDLE before releasing the bus
            for dev in self.motors:
                try:
                    self.bus.set_mode(dev, Mode.IDLE)
                except Exception:
                    pass
            for m in self.motors.values():
                m.enabled = False
            try:
                self.bus.stop()
            except Exception:
                pass


class MotorRow:
    def __init__(self, parent, m: MotorState, ctrl: ControlThread, row: int):
        self.m = m
        self.ctrl = ctrl

        frame = ttk.LabelFrame(parent, text=f"Motor ID {m.dev}")
        frame.grid(row=row, column=0, sticky="ew", padx=8, pady=4)
        frame.columnconfigure(3, weight=1)
        self.frame = frame

        self.led = tk.Canvas(frame, width=18, height=18, highlightthickness=0)
        self.led_id = self.led.create_oval(3, 3, 15, 15, fill="#888888", outline="#333333")
        self.led.grid(row=0, column=0, padx=(6, 2))

        self.status_lbl = ttk.Label(frame, text="offline", width=8, foreground="red")
        self.status_lbl.grid(row=0, column=1, padx=4)

        self.enable_var = tk.BooleanVar(value=False)
        self.enable_btn = ttk.Checkbutton(frame, text="Enable", variable=self.enable_var,
                                          command=self.on_enable_toggle)
        self.enable_btn.grid(row=0, column=2, padx=4)

        self.slider = tk.Scale(frame, from_=-args.range, to=args.range, resolution=0.01,
                               orient="horizontal", length=380, showvalue=True,
                               command=self.on_slider)
        self.slider.grid(row=0, column=3, sticky="ew", padx=4)
        self.slider.configure(state="disabled")

        self.zero_btn = ttk.Button(frame, text="Zero", width=5, command=self.on_zero)
        self.zero_btn.grid(row=0, column=4, padx=4)

        self.ping_btn = ttk.Button(frame, text="Ping", width=5, command=self.on_ping)
        self.ping_btn.grid(row=0, column=5, padx=(0, 4))

        self.info_lbl = ttk.Label(frame, text="pos: --  vel: --  err: --", font=("TkFixedFont", 9))
        self.info_lbl.grid(row=1, column=0, columnspan=6, sticky="w", padx=6, pady=(0, 3))

    def on_enable_toggle(self):
        if self.enable_var.get():
            self.m.offset = 0.0
            self.ctrl.request_motor_cmd("enable", self.m.dev)
        else:
            self.ctrl.request_motor_cmd("disable", self.m.dev)

    def on_slider(self, value: str):
        self.m.offset = float(value)

    def on_zero(self):
        self.slider.set(0.0)
        self.m.offset = 0.0

    def on_ping(self):
        self.ctrl.request("ping_one", self.m.dev)

    def destroy(self):
        self.frame.destroy()

    def refresh(self):
        m = self.m
        dead = self.ctrl.dead is not None

        # slider usability strictly follows the actual motor state; the value
        # reset must happen AFTER enabling the widget, because set() on a
        # disabled tk.Scale is a silent no-op and a stale value would jump the
        # arm on the first touch
        if m.enabled and not dead:
            if str(self.slider.cget("state")) == "disabled":
                self.slider.configure(state="normal")
                self.slider.set(0.0)
        elif str(self.slider.cget("state")) != "disabled":
            self.slider.configure(state="disabled")

        # checkbox strictly follows the actual motor state once no request is
        # in flight (covers failed enables, auto-disable, E-STOP, races)
        if (dead or not m.busy) and bool(self.enable_var.get()) != m.enabled:
            self.enable_var.set(m.enabled)

        if dead:
            status, fg, led_color = "DEAD", "red", "#cc2222"
        elif m.enabled:
            if m.timeouts > LINK_FLAKY_CYCLES:
                status, fg, led_color = "NO REPLY", "orange", "#ff9900"
            else:
                status, fg, led_color = "ACTIVE", "green", "#22cc44"
        elif m.busy:
            status, fg, led_color = "busy...", "gray", "#888888"
        elif m.fault:
            status, fg, led_color = m.fault, "red", "#cc2222"
        elif m.online:
            status, fg, led_color = "online", "orange", "#22cc44"
        else:
            status, fg, led_color = "offline", "red", "#cc2222"
        self.status_lbl.configure(text=status, foreground=fg)
        self.led.itemconfigure(self.led_id, fill=led_color)

        pos = f"{m.measured_pos:+.3f}" if m.measured_pos is not None else "--"
        vel = f"{m.measured_vel:+.2f}" if m.measured_vel is not None else "--"
        err = hex(m.error_code) if m.error_code is not None else "--"
        warn = f"  [missed {m.timeouts}]" if m.timeouts > LINK_FLAKY_CYCLES else ""
        self.info_lbl.configure(
            text=f"pos: {pos} rad  vel: {vel} rad/s  err: {err}  target: {m.command:+.3f}{warn}")


class App:
    def __init__(self, root: tk.Tk, motors: dict[int, MotorState], ctrl: ControlThread):
        self.root = root
        self.motors = motors
        self.ctrl = ctrl
        self._dead_notified = False

        root.title(f"Arm Motor Control - {args.channel}")

        top = ttk.Frame(root)
        top.grid(row=0, column=0, sticky="ew", padx=8, pady=6)

        self.estop_btn = tk.Button(top, text="E-STOP", bg="#cc2222", fg="white",
                                   font=("TkDefaultFont", 12, "bold"), width=10,
                                   command=self.on_estop)
        self.estop_btn.pack(side="left", padx=4)

        ttk.Button(top, text="Ping all", command=lambda: ctrl.request("ping")).pack(side="left", padx=4)

        ttk.Label(top, text="kp").pack(side="left", padx=(16, 2))
        self.kp_var = tk.StringVar(value=str(args.kp))
        ttk.Entry(top, textvariable=self.kp_var, width=6).pack(side="left")
        ttk.Label(top, text="kd").pack(side="left", padx=(8, 2))
        self.kd_var = tk.StringVar(value=str(args.kd))
        ttk.Entry(top, textvariable=self.kd_var, width=6).pack(side="left")
        ttk.Label(top, text="torque").pack(side="left", padx=(8, 2))
        self.tq_var = tk.StringVar(value=str(args.torque_limit))
        ttk.Entry(top, textvariable=self.tq_var, width=6).pack(side="left")
        ttk.Button(top, text="Apply gains", command=self.on_apply_gains).pack(side="left", padx=8)

        # arm group selector: switching disables any motor leaving the view
        group_bar = ttk.Frame(root)
        group_bar.grid(row=1, column=0, sticky="ew", padx=8)
        ttk.Label(group_bar, text="Group:").pack(side="left", padx=(4, 8))
        self.group_var = tk.StringVar(value=INITIAL_GROUP)
        group_labels = {
            "left": "Left arm (1 3 5 7 9)",
            "right": "Right arm (2 4 6 8 10)",
            "custom": f"Custom ({' '.join(map(str, ARM_GROUPS.get('custom', [])))})",
        }
        for key in ARM_GROUPS:
            ttk.Radiobutton(group_bar, text=group_labels[key], value=key,
                            variable=self.group_var,
                            command=self.on_group_change).pack(side="left", padx=6)

        self.banner = tk.Label(root, text="", anchor="w", justify="left")
        self.banner.grid(row=2, column=0, sticky="ew")

        self.body = ttk.Frame(root)
        self.body.grid(row=3, column=0, sticky="nsew")
        self.body.columnconfigure(0, weight=1)

        self.rows: list[MotorRow] = []
        self.build_rows(INITIAL_GROUP)

        hint = ttk.Label(root, foreground="gray",
                         text=f"check Enable to arm a motor - its slider unlocks once it goes ACTIVE  |  "
                              f"slider = offset from enable position [rad], +/-{args.range}, "
                              f"slew {args.max_speed} rad/s  |  "
                              f"LED: green=online, red=offline (auto-ping {AUTO_PING_PERIOD:.0f}s)")
        hint.grid(row=4, column=0, sticky="w", padx=10, pady=(0, 6))

        root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.refresh()

    def build_rows(self, group_key: str):
        for row in self.rows:
            row.destroy()
        devs = ARM_GROUPS[group_key]
        self.rows = [MotorRow(self.body, self.motors[d], self.ctrl, i) for i, d in enumerate(devs)]
        self.ctrl.visible_devs = set(devs)
        self.root.title(f"Arm Motor Control - {args.channel} - {group_key}")

    def on_group_change(self):
        group_key = self.group_var.get()
        # motors leaving the view must not keep running invisibly
        keep = set(ARM_GROUPS[group_key])
        for dev, m in self.motors.items():
            if dev not in keep and (m.enabled or m.busy):
                self.ctrl.request_motor_cmd("disable", dev)
        self.build_rows(group_key)

    def on_estop(self):
        self.ctrl.trigger_estop()

    def on_apply_gains(self):
        try:
            raw = {
                "kp": float(self.kp_var.get()),
                "kd": float(self.kd_var.get()),
                "torque_limit": float(self.tq_var.get()),
            }
        except ValueError:
            return
        payload = {k: clamp_gain(k, v) for k, v in raw.items()}
        # reflect clamping back into the entries so the UI never lies
        self.kp_var.set(str(payload["kp"]))
        self.kd_var.set(str(payload["kd"]))
        self.tq_var.set(str(payload["torque_limit"]))
        self.ctrl.request("gains", payload=payload)

    def refresh(self):
        if self.ctrl.dead is not None and not self._dead_notified:
            self._dead_notified = True
            self.banner.configure(
                text=f"  CONTROL THREAD DEAD: {self.ctrl.dead}\n"
                     f"  Motors were sent IDLE (best effort). Check the bus and restart this GUI.",
                bg="#cc2222", fg="white")
            self.estop_btn.configure(state="disabled", text="DEAD")
        for row in self.rows:
            row.refresh()
        self.root.after(50, self.refresh)

    def on_close(self):
        self.ctrl.trigger_estop()
        self.ctrl.running = False
        self.ctrl.join(timeout=3.0)
        if self.ctrl.is_alive():
            # control thread is stuck (e.g. blocked in a long CAN read):
            # best-effort direct IDLE broadcast before exiting
            self.ctrl.force_idle_all()
        self.root.destroy()


def main():
    bus = recoil.Bus(channel=args.channel, bitrate=1000000)
    motors = {dev: MotorState(dev) for dev in ALL_IDS}

    for dev, m in motors.items():
        m.online = bus.ping(dev)
        print(f"ID {dev}: {'online' if m.online else 'OFFLINE'}")

    ctrl = ControlThread(bus, motors)
    ctrl.start()

    root = tk.Tk()
    App(root, motors, ctrl)
    root.mainloop()


if __name__ == "__main__":
    main()
