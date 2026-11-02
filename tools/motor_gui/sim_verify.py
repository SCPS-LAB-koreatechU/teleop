#!/usr/bin/env python3
# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.
"""Headless simulation checks for the BHL motor GUI.

Every safety rule the GUI claims is exercised here against the simulated bus,
with no hardware and no window. Each check prints one line and the exit code is
the verdict, so this can gate a commit:

    ./sim_verify.py            # all checks
    ./sim_verify.py -k estop   # only checks whose name contains "estop"

What is NOT covered here: anything that needs the real CAN link (electrical
directions, gear ratio sign, encoder offsets). Those still have to be walked
through on the bench, one joint at a time, per the README.
"""

from __future__ import annotations

import argparse
import math
import shutil
import sys
import time
from pathlib import Path

import bhl_motor_gui as G

HERE = Path(__file__).resolve().parent


class Check:
    """One named check; `run` returns a list of failure strings (empty = pass)."""

    registry: list["Check"] = []

    def __init__(self, name: str, fn):
        self.name = name
        self.fn = fn

    @classmethod
    def add(cls, name: str):
        def deco(fn):
            cls.registry.append(Check(name, fn))
            return fn
        return deco


def rig_for(extra: list[str] | None = None) -> G.Rig:
    args = G.parse_args(["--sim"] + (extra or []))
    args.rate = 200.0          # checks run faster than the GUI's 100 Hz
    return G.Rig(args)


def wait_for(pred, timeout: float, step: float = 0.01) -> bool:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if pred():
            return True
        time.sleep(step)
    return False


def settle(rig: G.Rig, timeout: float = 3.0) -> bool:
    return wait_for(lambda: all(not st.busy for st in rig.states.values()), timeout)


def enable_all(rig: G.Rig, unlock: bool = True) -> None:
    if unlock:
        for t in rig.threads.values():
            t.unlocked = True
    for st in rig.states.values():
        thread = rig.thread_for(st)
        if thread:
            thread.request_motor_cmd("zero", st.dev)
    settle(rig)
    for st in rig.states.values():
        thread = rig.thread_for(st)
        if thread:
            thread.request_motor_cmd("enable", st.dev)
    settle(rig)


# ---------------------------------------------------------------------------


@Check.add("config-matches-urdf")
def check_config(rig: G.Rig) -> list[str]:
    """Every mapped joint exists in the URDF, with sane limits and unique ids."""
    bad = []
    if len(rig.states) != 10:
        bad.append(f"expected 10 actuated joints, got {len(rig.states)}")
    seen: dict[tuple[str, int], str] = {}
    for st in rig.states.values():
        s = st.spec
        if s.lower >= s.upper:
            bad.append(f"{s.name}: limits {s.lower}..{s.upper}")
        if not (s.lower <= 0.0 <= s.upper):
            bad.append(f"{s.name}: URDF zero is outside its own limits")
        key = (s.channel, s.device_id)
        if key in seen:
            bad.append(f"{s.name} and {seen[key]} share {s.channel} id {s.device_id}")
        seen[key] = s.name
        if s.gear_ratio == 0:
            bad.append(f"{s.name}: gear_ratio 0")
    return bad


@Check.add("unit-conversion-roundtrip")
def check_units(rig: G.Rig) -> list[str]:
    """URDF radians -> actuator units -> back is the identity, offsets included."""
    bad = []
    for st in rig.states.values():
        s = st.spec
        s.offset = 0.37                      # a deliberately awkward offset
        for value in (s.lower, 0.0, s.upper, (s.lower + s.upper) / 2):
            back = s.to_urdf(s.to_actuator(value))
            if abs(back - value) > 1e-9:
                bad.append(f"{s.name}: {value} -> {back}")
        s.offset = 0.0
    return bad


@Check.add("enable-refuses-locked")
def check_locked(rig: G.Rig) -> list[str]:
    """A joint marked enabled:false in the yaml cannot be armed while locked."""
    bad = []
    for st in rig.states.values():
        thread = rig.thread_for(st)
        if thread:
            thread.unlocked = False
            thread.request_motor_cmd("zero", st.dev)
    settle(rig)
    for st in rig.states.values():
        thread = rig.thread_for(st)
        if thread:
            thread.request_motor_cmd("enable", st.dev)
    settle(rig)
    for st in rig.states.values():
        if not st.spec.config_enabled and st.live:
            bad.append(f"{st.spec.name}: went live while locked")
        if not st.spec.config_enabled and st.fault != "LOCKED":
            bad.append(f"{st.spec.name}: fault is {st.fault!r}, expected LOCKED")
    return bad


@Check.add("enable-refuses-out-of-range")
def check_out_of_range(rig: G.Rig) -> list[str]:
    """An uncalibrated reading (outside the URDF limits) must not be armed."""
    bad = []
    for t in rig.threads.values():
        t.unlocked = True
    for st in rig.states.values():          # no zeroing: raw boot values stand
        rig.thread_for(st).request_motor_cmd("enable", st.dev)
    settle(rig)
    refused = 0
    for st in rig.states.values():
        outside = not (st.spec.lower <= st.spec.to_urdf(st.raw or 0.0) <= st.spec.upper)
        if outside:
            refused += 1
            if st.live:
                bad.append(f"{st.spec.name}: live at {st.measured:+.2f} rad, outside "
                           f"{st.spec.lower:+.2f}..{st.spec.upper:+.2f}")
            elif st.fault != "OUT_OF_RANGE":
                bad.append(f"{st.spec.name}: fault {st.fault!r}, expected OUT_OF_RANGE")
    if refused == 0:
        bad.append("no joint booted outside its range, so the refusal path was not exercised")
    return bad


@Check.add("enable-refuses-silent-motor")
def check_silent(rig: G.Rig) -> list[str]:
    """A motor that does not answer is never armed."""
    bad = []
    missing = set(rig.args.sim_offline or [])
    if not missing:
        return ["check needs --sim-offline ids"]
    enable_all(rig)
    for st in rig.states.values():
        if st.dev in missing:
            if st.live:
                bad.append(f"{st.spec.name}: armed a silent motor")
            if st.online:
                bad.append(f"{st.spec.name}: reported online while silent")
    return bad


@Check.add("zero-here")
def check_zero(rig: G.Rig) -> list[str]:
    """'Zero here' makes the present reading URDF zero, and only for idle joints."""
    bad = []
    for st in rig.states.values():
        rig.thread_for(st).request_motor_cmd("zero", st.dev)
    settle(rig)
    for st in rig.states.values():
        if st.measured is None or abs(st.measured) > 1e-9:
            bad.append(f"{st.spec.name}: measured {st.measured} after zeroing")
        if abs(st.spec.offset - st.spec.offset_for_zero_here(st.raw)) > 1e-9:
            bad.append(f"{st.spec.name}: offset does not match the reading")
    # a live joint must ignore the request rather than shift under load
    enable_all(rig)
    victim = next(st for st in rig.states.values() if st.live)
    before = victim.spec.offset
    rig.thread_for(victim).request_motor_cmd("zero", victim.dev)
    settle(rig)
    if victim.spec.offset != before:
        bad.append(f"{victim.spec.name}: offset changed while live")
    return bad


@Check.add("enable-does-not-move")
def check_no_jump(rig: G.Rig) -> list[str]:
    """Arming a joint must not command any motion."""
    bad = []
    for t in rig.threads.values():
        t.unlocked = True
    for st in rig.states.values():
        rig.thread_for(st).request_motor_cmd("zero", st.dev)
    settle(rig)
    # park each joint somewhere away from zero before arming it
    for st in rig.states.values():
        bus = rig.threads[st.spec.channel].bus
        motor = bus.motors.get(st.dev)
        if motor:
            motor.raw = st.spec.to_actuator(st.spec.upper * 0.5)
            motor.target = motor.raw
    for st in rig.states.values():
        rig.thread_for(st).request_motor_cmd("enable", st.dev)
    settle(rig)
    snapshot = {st.dev: st.measured for st in rig.states.values()}
    time.sleep(0.8)
    for st in rig.states.values():
        if not st.live:
            continue
        drift = abs((st.measured or 0.0) - (snapshot[st.dev] or 0.0))
        if drift > 1e-6:
            bad.append(f"{st.spec.name}: drifted {drift:.4f} rad after enable")
    return bad


@Check.add("slew-rate-limit")
def check_slew(rig: G.Rig) -> list[str]:
    """However hard the target is yanked, the command ramps at max_joint_speed."""
    bad = []
    enable_all(rig)
    limit = 0.5
    for t in rig.threads.values():
        t.max_speed = limit
    st = next(s for s in rig.states.values() if s.live)
    st.target = st.spec.upper
    t0, c0 = time.monotonic(), st.command
    time.sleep(0.4)
    dt, dc = time.monotonic() - t0, abs(st.command - c0)
    rate = dc / dt
    if rate > limit * 1.35:
        bad.append(f"{st.spec.name}: ramped at {rate:.2f} rad/s, limit {limit}")
    if dc < 1e-6:
        bad.append(f"{st.spec.name}: did not move at all")
    return bad


@Check.add("limit-clamp")
def check_limits(rig: G.Rig) -> list[str]:
    """A target past the limits settles on the allowed band, never past the stop.

    The band is the URDF limits shrunk by the margin, widened to include URDF
    zero and the arming position -- so the test is against that band, plus the
    hard rule that the band itself never runs past a URDF limit by more than
    the margin.
    """
    bad = []
    enable_all(rig)
    for t in rig.threads.values():
        t.max_speed = 3.0
    for end in ("upper", "lower"):
        for st in rig.states.values():
            if st.live:
                st.target = st.spec.upper + 10.0 if end == "upper" else st.spec.lower - 10.0
        time.sleep(2.5)
        for st in rig.states.values():
            if not st.live:
                continue
            margin = rig.threads[st.spec.channel].limit_margin
            edge = st.hi_m if end == "upper" else st.lo_m
            if abs(st.measured - edge) > 3e-3:
                bad.append(f"{st.spec.name}: settled at {st.measured:+.3f}, band {end} edge {edge:+.3f}")
            if end == "upper" and edge > st.spec.upper + margin:
                bad.append(f"{st.spec.name}: band reaches {edge:+.3f}, limit {st.spec.upper:+.3f}")
            if end == "lower" and edge < st.spec.lower - margin:
                bad.append(f"{st.spec.name}: band reaches {edge:+.3f}, limit {st.spec.lower:+.3f}")
    return bad


@Check.add("home-preset-reachable")
def check_home_exact(rig: G.Rig) -> list[str]:
    """Home (zero for every arm joint) is reachable from anywhere, not 3 deg short.

    Regression guard: with a plain safety margin the two elbow_pitch joints,
    whose URDF limit sits exactly on zero, could never be driven home.
    """
    bad = []
    enable_all(rig)
    for t in rig.threads.values():
        t.max_speed = 3.0
    for st in rig.states.values():          # drive well away from home first
        if st.live:
            st.target = G.clamp(st.spec.upper * 0.7 + st.spec.lower * 0.3,
                                st.spec.lower, st.spec.upper)
    time.sleep(2.0)
    for st in rig.states.values():
        if st.live:
            st.target = 0.0
    time.sleep(2.5)
    for st in rig.states.values():
        if st.live and abs(st.measured) > 2e-3:
            bad.append(f"{st.spec.name}: stopped at {math.degrees(st.measured):+.2f} deg short of home")
    return bad


@Check.add("presets-reachable")
def check_presets(rig: G.Rig) -> list[str]:
    """Both SRDF presets are inside the limits and are actually reached."""
    bad = []
    enable_all(rig)
    for t in rig.threads.values():
        t.max_speed = 3.0
    for name in ("home", "ready"):
        values = rig.presets.get(name)
        if not values:
            bad.append(f"preset {name} missing from the SRDF")
            continue
        for st in rig.states.values():
            if st.spec.name in values and st.live:
                target = values[st.spec.name]
                if not (st.spec.lower <= target <= st.spec.upper):
                    bad.append(f"{name}/{st.spec.name}: {target} outside the URDF limits")
                st.target = G.clamp(target, st.spec.lower, st.spec.upper)
        time.sleep(2.5)
        for st in rig.states.values():
            if st.live and st.spec.name in values:
                err = abs(st.measured - st.target)
                if err > 0.02:
                    bad.append(f"{name}/{st.spec.name}: error {err:.3f} rad")
    return bad


@Check.add("estop")
def check_estop(rig: G.Rig) -> list[str]:
    """E-STOP idles everything, and a slider left mid-travel cannot restart it."""
    bad = []
    enable_all(rig)
    live_before = sum(st.live for st in rig.states.values())
    if live_before == 0:
        return ["nothing was live, so E-STOP proved nothing"]
    for st in rig.states.values():
        st.target = st.spec.upper * 0.9
    rig.estop_all()
    if not wait_for(lambda: all(not st.live and not st.busy for st in rig.states.values()), 3.0):
        bad.append("joints still live after E-STOP")
    positions = {st.dev: st.measured for st in rig.states.values()}
    time.sleep(0.8)
    for st in rig.states.values():
        bus = rig.threads[st.spec.channel].bus
        motor = bus.motors.get(st.dev)
        if motor and motor.mode != G.BusThread.MODE_IDLE:
            bad.append(f"{st.spec.name}: mode {motor.mode:#x}, expected IDLE")
        if st.measured is not None and abs(st.measured - (positions[st.dev] or 0.0)) > 1e-6:
            bad.append(f"{st.spec.name}: kept moving after E-STOP")
    return bad


@Check.add("link-loss-auto-disable")
def check_link_loss(rig: G.Rig) -> list[str]:
    """A live motor that goes silent freezes, then is dropped to IDLE as LOST."""
    bad = []
    enable_all(rig)
    st = next((s for s in rig.states.values() if s.live), None)
    if st is None:
        return ["no joint could be armed"]
    thread = rig.threads[st.spec.channel]
    st.target = st.spec.upper * 0.8
    time.sleep(0.2)
    thread.bus.set_present(st.dev, False)      # the link drops
    frozen = st.command
    if not wait_for(lambda: st.timeouts > thread.link_flaky, 2.0):
        bad.append(f"{st.spec.name}: missed replies were not counted")
    time.sleep(0.3)
    if abs(st.command - frozen) > 0.05:
        bad.append(f"{st.spec.name}: command walked from {frozen:.3f} to {st.command:.3f} while silent")
    if not wait_for(lambda: not st.live, 4.0):
        bad.append(f"{st.spec.name}: still live after the link dropped")
    if st.fault != "LOST":
        bad.append(f"{st.spec.name}: fault {st.fault!r}, expected LOST")
    # and it must not come back on its own
    thread.bus.set_present(st.dev, True)
    time.sleep(0.5)
    if st.live:
        bad.append(f"{st.spec.name}: re-armed itself when the link returned")
    return bad


@Check.add("online-indicator-does-not-flap")
def check_flap(rig: G.Rig) -> list[str]:
    """One lost reply must not drop a joint to offline.

    The bench symptom: the connection indicator blinked on and off while the
    motor was answering most of the time. Every other reply is dropped here,
    which no real motor should be judged offline for.
    """
    bad = []
    st = next(iter(rig.states.values()))
    bus = rig.threads[st.spec.channel].bus
    thread = rig.threads[st.spec.channel]
    thread.request("ping", st.dev)
    settle(rig)
    if not st.online:
        return ["the joint was not online to begin with"]

    real_read = bus._read_parameter_f32
    state = {"n": 0}

    def flaky(dev, param, timeout=None):
        state["n"] += 1
        if state["n"] % 2 == 0:            # every other reply is lost
            return None
        return real_read(dev, param, timeout)

    bus._read_parameter_f32 = flaky
    try:
        drops = 0
        end = time.monotonic() + 3.0
        while time.monotonic() < end:
            if not st.online:
                drops += 1
            time.sleep(0.02)
        if drops:
            bad.append(f"{st.spec.name}: read as offline in {drops} samples while "
                       f"answering half the polls")
    finally:
        bus._read_parameter_f32 = real_read
    return bad


@Check.add("blocked-bus-does-not-kill-the-thread")
def check_blocked(rig: G.Rig) -> list[str]:
    """ENOBUFS (nothing acknowledging) must be survivable, not fatal.

    Switching the motor's power off blocks the transmit queue; python-can then
    raises, and a control thread that treats that as fatal leaves a dead window
    behind that has to be restarted.
    """
    bad = []
    enable_all(rig)
    st = next((s for s in rig.states.values() if s.live), None)
    if st is None:
        return ["no joint could be armed"]
    thread = rig.threads[st.spec.channel]
    bus = thread.bus
    real_tx = bus.transmit_pdo_2

    def blocked(*_a, **_k):
        raise OSError(105, "No buffer space available")

    bus.transmit_pdo_2 = blocked
    try:
        time.sleep(1.0)
        if thread.dead is not None:
            bad.append(f"control thread died on a blocked bus: {thread.dead}")
        if not thread.blocked:
            bad.append("the blocked bus was not reported")
        if st.online:
            bad.append(f"{st.spec.name}: still shown online while nothing gets through")
    finally:
        bus.transmit_pdo_2 = real_tx
    if not bad:
        time.sleep(1.0)
        if thread.dead is not None:
            bad.append("thread died after the bus recovered")
        if thread.blocked:
            bad.append("still reported blocked after the bus recovered")
    return bad


@Check.add("dead-bus-polling-backs-off")
def check_dead_bus_backoff(rig: G.Rig) -> list[str]:
    """With nothing answering, the window must stop hammering the bus.

    Every unanswered frame ages the CAN controller, and the bench adapter
    cannot restart itself from bus-off - so a window left open on a dead bus
    used to be enough to wedge the interface.
    """
    bad = []
    thread = next(iter(rig.threads.values()))
    bus = thread.bus
    for dev in list(bus.motors):            # everything goes away
        bus.set_present(dev, False)
    reads = {"n": 0}
    real_read = bus._read_parameter_f32

    def counted(dev, param, timeout=None):
        reads["n"] += 1
        return real_read(dev, param, timeout)

    bus._read_parameter_f32 = counted
    try:
        time.sleep(G.DEAD_BUS_AFTER + 1.0)   # let it notice
        if not thread.quiet:
            bad.append("the thread never noticed that nothing answers")
        reads["n"] = 0
        window = 4.0
        time.sleep(window)
        allowed = window / G.DEAD_BUS_POLL_PERIOD + 2
        if reads["n"] > allowed:
            bad.append(f"{reads['n']} polls in {window:.0f} s, expected at most {allowed:.0f}")
    finally:
        bus._read_parameter_f32 = real_read
    return bad


@Check.add("enable-refuses-a-silent-bus")
def check_enable_on_dead_bus(rig: G.Rig) -> list[str]:
    """Enabling while nothing answers must not put a single frame on the bus.

    An unacknowledged frame is retried by the CAN controller forever, so the
    burst an enable sends (ping, three writes, two reads) becomes a permanent
    error storm that only an interface down/up clears.
    """
    bad = []
    thread = next(iter(rig.threads.values()))
    bus = thread.bus
    for dev in list(bus.motors):
        bus.set_present(dev, False)
    thread.unlocked = True
    time.sleep(G.DEAD_BUS_AFTER + 1.0)
    if not thread.quiet:
        return ["the thread never noticed that nothing answers"]

    sends = {"n": 0}
    for name in ("ping", "transmit_pdo_2", "set_mode", "feed"):
        real = getattr(bus, name)

        def counted(*a, _real=real, **k):
            sends["n"] += 1
            return _real(*a, **k)

        setattr(bus, name, counted)
    real_read = bus._read_parameter_f32

    def counted_read(*a, **k):
        sends["n"] += 1
        return real_read(*a, **k)

    bus._read_parameter_f32 = counted_read
    try:
        st = next(s for s in rig.states.values() if s.spec.channel == thread.channel)
        thread.request_motor_cmd("enable", st.dev)
        settle(rig)
        if sends["n"]:
            bad.append(f"enable put {sends['n']} call(s) on a silent bus")
        if st.live:
            bad.append("the joint went live on a bus where nothing answers")
        if st.fault not in ("NO REPLY", "NO BUS"):
            bad.append(f"fault is {st.fault!r}, expected NO REPLY")
    finally:
        bus._read_parameter_f32 = real_read
    return bad


@Check.add("bus-override-reaches-the-config")
def check_bus_override(_rig: G.Rig) -> list[str]:
    """--bus KEY=CHANNEL must actually move the joints onto that channel.

    The option was once parsed and then dropped on the floor, because the call
    site it was wired into got refactored: the flag was accepted, printed
    nothing, and quietly did nothing. On the bench that means a right-arm motor
    silently stays on the absent can1.
    """
    bad = []
    args = G.parse_args(["--sim", "--ids", "2", "--bus", "right=can0"])
    args.rate = 200.0
    rig = G.Rig(args)
    try:
        if rig.buses.get("right") != "can0":
            bad.append(f"buses say right={rig.buses.get('right')!r}, expected 'can0'")
        for st in rig.states.values():
            if st.spec.channel != "can0":
                bad.append(f"{st.spec.name} is on {st.spec.channel}, expected can0")
    finally:
        rig.shutdown()

    try:
        G.parse_args(["--sim", "--bus", "nosuch=can0"])
        limits = G.load_urdf_limits(G.DEFAULT_URDF)
        G.load_config(G.DEFAULT_CONFIG, limits, None, {"nosuch": "can0"})
        bad.append("an unknown bus key was accepted")
    except SystemExit:
        pass
    return bad


@Check.add("gain-clamping")
def check_gains(_rig: G.Rig) -> list[str]:
    """Typos in the gain boxes are clamped before they can reach a motor."""
    bad = []
    cases = {("kp", 7.0): 5.0, ("kp", -1.0): 0.0, ("kd", 0.9): 0.5,
             ("kd", -0.1): 0.0, ("torque_limit", 50.0): 2.0}
    for (name, value), expected in cases.items():
        got = G.clamp_gain(name, value)
        if got != expected:
            bad.append(f"{name}={value} clamped to {got}, expected {expected}")
    return bad


@Check.add("offset-save-roundtrip")
def check_save(rig: G.Rig) -> list[str]:
    """Saving offsets rewrites only the offset lines and keeps a backup."""
    bad = []
    src = rig.args.config
    tmp = HERE / "_sim_verify_config.yaml"
    shutil.copy(src, tmp)
    try:
        before = tmp.read_text().splitlines()
        offsets = {st.spec.name: round(0.1 * (i + 1), 6) for i, st in enumerate(rig.states.values())}
        backup = G.save_offsets(tmp, offsets)
        if not backup.is_file():
            bad.append("no .bak written")
        after = tmp.read_text().splitlines()
        if len(before) != len(after):
            bad.append(f"line count changed: {len(before)} -> {len(after)}")
        changed = [i for i, (a, b) in enumerate(zip(before, after)) if a != b]
        if len(changed) != len(offsets):
            bad.append(f"{len(changed)} lines changed, expected {len(offsets)}")
        for i in changed:
            if "offset:" not in after[i]:
                bad.append(f"line {i + 1} changed but is not an offset: {after[i].strip()!r}")
        # reload through the real loader and compare
        limits = G.load_urdf_limits(rig.args.urdf)
        _b, _g, _s, specs = G.load_config(tmp, limits)
        for spec in specs:
            want = offsets[spec.name]
            if abs(spec.offset - want) != 0:
                bad.append(f"{spec.name}: reloaded offset {spec.offset}, wrote {want}")
        if any(sp.enabled if hasattr(sp, "enabled") else False for sp in []):
            pass
    finally:
        for path in (tmp, tmp.with_suffix(tmp.suffix + ".bak")):
            if path.exists():
                path.unlink()
    return bad


@Check.add("no-bus-is-inert")
def check_no_bus(_rig: G.Rig) -> list[str]:
    """With a missing CAN channel the joints are shown, and nothing is driven.

    Points the config at channels that cannot exist, so the result does not
    depend on whether a CAN adapter happens to be plugged in right now.
    """
    bad = []
    src = G.parse_args(["--sim"]).config
    tmp = HERE / "_sim_verify_nobus.yaml"
    text = src.read_text().replace("  left: can0", "  left: can_absent0").replace(
        "  right: can1", "  right: can_absent1")
    tmp.write_text(text)
    args = G.parse_args(["--unlock", "--config", str(tmp)])      # live mode, absent buses
    args.rate = 200.0
    rig = G.Rig(args)
    try:
        if len(rig.states) != 10:
            bad.append(f"{len(rig.states)} joints listed, expected 10")
        if rig.threads:
            bad.append(f"threads started for absent buses: {list(rig.threads)}")
        for st in rig.states.values():
            if st.live or st.fault != "NO BUS":
                bad.append(f"{st.spec.name}: live={st.live} fault={st.fault!r}")
            if rig.thread_for(st) is not None:
                bad.append(f"{st.spec.name}: has a control thread without a bus")
    finally:
        rig.shutdown()
        tmp.unlink(missing_ok=True)
    return bad


# ---------------------------------------------------------------------------

NEEDS_OWN_RIG = {"no-bus-is-inert"}
RIG_ARGS = {
    "enable-refuses-silent-motor": ["--sim-offline", "5", "8"],
    "enable-refuses-out-of-range": [],
}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("-k", dest="filter", default="", help="only run checks whose name contains this")
    ap.add_argument("-v", "--verbose", action="store_true")
    opts = ap.parse_args(argv)

    checks = [c for c in Check.registry if opts.filter in c.name]
    if not checks:
        print(f"no check matches {opts.filter!r}")
        return 2

    width = max(len(c.name) for c in checks)
    failures: dict[str, list[str]] = {}
    t0 = time.monotonic()
    for check in checks:
        rig = None
        started = time.monotonic()
        try:
            if check.name in NEEDS_OWN_RIG:
                problems = check.fn(None)
            else:
                rig = rig_for(RIG_ARGS.get(check.name))
                problems = check.fn(rig)
        except Exception as e:  # noqa: BLE001 - a crashing check is a failing check
            problems = [f"{type(e).__name__}: {e}"]
        finally:
            if rig is not None:
                rig.shutdown()
        took = time.monotonic() - started
        if problems:
            failures[check.name] = problems
        mark = "PASS" if not problems else "FAIL"
        print(f"  {check.name:<{width}}  {mark}  {took:5.1f}s")
        for line in problems:
            print(f"      - {line}")

    print(f"\n{len(checks) - len(failures)}/{len(checks)} checks passed "
          f"in {time.monotonic() - t0:.0f}s")
    return 0 if not failures else 1


if __name__ == "__main__":
    sys.exit(main())
