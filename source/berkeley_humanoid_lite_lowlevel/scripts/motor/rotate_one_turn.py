# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.
# Rotate the motor a given number of turns with a smooth position ramp.

import time
import math
import argparse

from loop_rate_limiters import RateLimiter

import berkeley_humanoid_lite_lowlevel.recoil as recoil
from berkeley_humanoid_lite_lowlevel.recoil import Parameter, Mode


parser = argparse.ArgumentParser()
parser.add_argument("-c", "--channel", type=str, default="can0", help="CAN transport channel")
parser.add_argument("-i", "--id", type=int, default=1, help="CAN device ID")
parser.add_argument("-n", "--turns", type=float, default=1.0, help="number of turns (+CW/-CCW)")
parser.add_argument("-t", "--time", type=float, default=1.0, help="ramp duration [s] (smaller = faster)")
parser.add_argument("--kp", type=float, default=0.7)
parser.add_argument("--kd", type=float, default=0.03)
parser.add_argument("--torque-limit", type=float, default=0.8)
args = parser.parse_args()

bus = recoil.Bus(channel=args.channel, bitrate=1000000)
dev = args.id

if not bus.ping(dev):
    print("[FAIL] motor did not respond to ping. Aborting.")
    bus.stop()
    raise SystemExit(1)

bus._write_parameter_u32(dev, Parameter.ERROR, 0)
bus.write_position_kp(dev, args.kp)
bus.write_position_kd(dev, args.kd)
bus.write_torque_limit(dev, args.torque_limit)

p0 = bus._read_parameter_f32(dev, Parameter.POSITION_CONTROLLER_POSITION_MEASURED, timeout=0.5)
delta = 2 * math.pi * args.turns
print(f"start: {p0:+.2f} rad -> goal: {p0 + delta:+.2f} rad ({args.turns} turn(s) in {args.time}s)")

bus.set_mode(dev, Mode.POSITION)
bus.feed(dev)

rate = RateLimiter(frequency=200.0)
t_start = time.time()
peak_v = 0.0
last = p0
while True:
    t = time.time() - t_start
    if t > args.time + 0.5:
        break
    s = min(t / args.time, 1.0)
    blend = (1 - math.cos(math.pi * s)) / 2  # smooth accel/decel
    target = p0 + delta * blend
    mp, mv = bus.write_read_pdo_2(dev, target, 0.0)
    if mp is not None:
        last = mp
        peak_v = max(peak_v, abs(mv))
    rate.sleep()

bus.set_mode(dev, Mode.IDLE)
err = bus._read_parameter_u32(dev, Parameter.ERROR, timeout=0.5)
moved = last - p0
print(f"moved: {moved:+.2f} rad (goal {delta:+.2f}), peak speed: {peak_v:.1f} rad/s, error: {hex(err) if err is not None else '?'}")
if abs(moved - delta) < 0.3:
    print("[OK] target reached")
else:
    print("[WARN] did not reach target - check phases/calibration")
bus.stop()
