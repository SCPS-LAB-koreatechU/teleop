# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.
# Multi-motor arm test: drive several motors on one CAN bus simultaneously
# with a small sine wave around each motor's current position.
#
# Example (left arm, gentle):
#   python move_arm_test.py -c can0 -i 1 3 5 7 -a 0.5 -f 0.5 --duration 10

import time
import math
import argparse

import numpy as np
from loop_rate_limiters import RateLimiter

import berkeley_humanoid_lite_lowlevel.recoil as recoil
from berkeley_humanoid_lite_lowlevel.recoil import Parameter, Mode


parser = argparse.ArgumentParser()
parser.add_argument("-c", "--channel", type=str, default="can0")
parser.add_argument("-i", "--ids", type=int, nargs="+", default=[1, 3, 5, 7, 9], help="CAN device IDs")
parser.add_argument("-a", "--amplitude", type=float, default=0.5, help="rotor amplitude [rad]")
parser.add_argument("-f", "--frequency", type=float, default=0.5, help="[Hz]")
parser.add_argument("--kp", type=float, default=0.2)
parser.add_argument("--kd", type=float, default=0.005)
parser.add_argument("--torque-limit", type=float, default=0.5)
parser.add_argument("--duration", type=float, default=10.0)
parser.add_argument("--stagger", action="store_true", help="phase-shift each motor's sine")
args = parser.parse_args()

bus = recoil.Bus(channel=args.channel, bitrate=1000000)

# --- ping check: only drive motors that respond ---
active = []
for dev in args.ids:
    if bus.ping(dev):
        active.append(dev)
        print(f"ID {dev}: online")
    else:
        print(f"ID {dev}: OFFLINE - skipped")

if not active:
    print("[FAIL] no motors online")
    bus.stop()
    raise SystemExit(1)

# --- setup: gains, error clear, capture center positions ---
centers = {}
for dev in active:
    bus._write_parameter_u32(dev, Parameter.ERROR, 0)
    bus.write_position_kp(dev, args.kp)
    bus.write_position_kd(dev, args.kd)
    bus.write_torque_limit(dev, args.torque_limit)
    p = bus._read_parameter_f32(dev, Parameter.POSITION_CONTROLLER_POSITION_MEASURED, timeout=0.5)
    if p is None:
        print(f"ID {dev}: position read failed - skipped")
        continue
    centers[dev] = p

active = [d for d in active if d in centers]
print(f"driving {len(active)} motors: {active}")
print(f"amplitude {args.amplitude} rad, {args.frequency} Hz, kp {args.kp}, torque {args.torque_limit}")

for dev in active:
    bus.set_mode(dev, Mode.POSITION)
    bus.feed(dev)

rate = RateLimiter(frequency=200.0)
t_start = time.time()
measured = {d: (0.0, 0.0) for d in active}

try:
    while time.time() - t_start < args.duration:
        t = time.time() - t_start
        # transmit all targets first, then receive all replies (pipelined like humanoid.py)
        for k, dev in enumerate(active):
            phase = (2 * math.pi * k / len(active)) if args.stagger else 0.0
            target = centers[dev] + args.amplitude * math.sin(2 * math.pi * args.frequency * t + phase)
            bus.transmit_pdo_2(dev, position_target=target, velocity_target=0.0)
        for dev in active:
            r = bus.receive_pdo_2(dev)
            if r is not None and r[0] is not None:
                measured[dev] = r

        if int(t * 2) != int((t - 0.005) * 2):  # ~2 Hz status print
            line = "  ".join(f"ID{d}:{measured[d][0]:+.2f}" for d in active)
            print(f"t={t:5.1f}  {line}")

        rate.sleep()

except KeyboardInterrupt:
    pass

for dev in active:
    bus.set_mode(dev, Mode.IDLE)
bus.stop()
print("done - all motors set to IDLE")
