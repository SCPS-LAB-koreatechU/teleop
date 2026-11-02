# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.
# Rotate multiple motors simultaneously by N turns with a smooth ramp.
#
# Example: rotate motors 1 and 3 five turns in 30 s (slow):
#   python rotate_multi.py -c can0 -i 1 3 -n 5 -t 30

import time
import math
import argparse

from loop_rate_limiters import RateLimiter

import berkeley_humanoid_lite_lowlevel.recoil as recoil
from berkeley_humanoid_lite_lowlevel.recoil import Parameter, Mode


parser = argparse.ArgumentParser()
parser.add_argument("-c", "--channel", type=str, default="can0")
parser.add_argument("-i", "--ids", type=int, nargs="+", required=True)
parser.add_argument("-n", "--turns", type=float, default=1.0, help="rotor turns (+/-)")
parser.add_argument("-t", "--time", type=float, default=10.0, help="ramp duration [s]")
parser.add_argument("--kp", type=float, default=0.4)
parser.add_argument("--kd", type=float, default=0.01)
parser.add_argument("--torque-limit", type=float, default=1.0)
args = parser.parse_args()

bus = recoil.Bus(channel=args.channel, bitrate=1000000)

active = []
centers = {}
for dev in args.ids:
    if not bus.ping(dev):
        print(f"ID {dev}: OFFLINE - skipped")
        continue
    fo = bus._read_parameter_f32(dev, Parameter.ENCODER_FLUX_OFFSET, timeout=0.5)
    if fo is None or fo == 0.0:
        print(f"ID {dev}: 캘리브레이션 안 됨(flux_offset=0) - skipped")
        continue
    p = bus._read_parameter_f32(dev, Parameter.POSITION_CONTROLLER_POSITION_MEASURED, timeout=0.5)
    if p is None:
        print(f"ID {dev}: 위치 읽기 실패 - skipped")
        continue
    bus._write_parameter_u32(dev, Parameter.ERROR, 0)
    bus.write_position_kp(dev, args.kp)
    bus.write_position_kd(dev, args.kd)
    bus.write_torque_limit(dev, args.torque_limit)
    centers[dev] = p
    active.append(dev)
    print(f"ID {dev}: online, start={p:+.2f} rad")

if not active:
    print("[FAIL] 구동 가능한 모터 없음")
    bus.stop()
    raise SystemExit(1)

delta = 2 * math.pi * args.turns
joint_deg = args.turns * 360 / 15
print(f"{active} -> {args.turns}회전 ({args.time}s, 관절 약 {joint_deg:.0f}도 이동)")

for dev in active:
    bus.set_mode(dev, Mode.POSITION)
    bus.feed(dev)

rate = RateLimiter(frequency=200.0)
t_start = time.time()
last = dict(centers)

try:
    while True:
        t = time.time() - t_start
        if t > args.time + 0.5:
            break
        s = min(t / args.time, 1.0)
        blend = (1 - math.cos(math.pi * s)) / 2
        for dev in active:
            bus.transmit_pdo_2(dev, position_target=centers[dev] + delta * blend, velocity_target=0.0)
        for dev in active:
            r = bus.receive_pdo_2(dev)
            if r is not None and r[0] is not None:
                last[dev] = r[0]
        if int(t) != int(t - 0.005):
            print("  ".join(f"ID{d}:{last[d]:+.2f}" for d in active))
        rate.sleep()
except KeyboardInterrupt:
    pass

for dev in active:
    bus.set_mode(dev, Mode.IDLE)
bus.stop()

for dev in active:
    moved = last[dev] - centers[dev]
    ok = abs(moved - delta) < 0.5
    print(f"ID {dev}: moved {moved:+.2f} rad (목표 {delta:+.2f}) {'[OK]' if ok else '[WARN]'}")
