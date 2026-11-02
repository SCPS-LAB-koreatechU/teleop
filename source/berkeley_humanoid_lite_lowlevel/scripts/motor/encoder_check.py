# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.
# Encoder sanity check: motor stays in IDLE (no torque). Rotate the output
# flange BY HAND while this runs - the printed position must track your motion
# smoothly. Frozen or jumpy readings mean the encoder/magnet is faulty.

import time
import argparse

import berkeley_humanoid_lite_lowlevel.recoil as recoil
from berkeley_humanoid_lite_lowlevel.recoil import Parameter, Mode


parser = argparse.ArgumentParser()
parser.add_argument("-c", "--channel", type=str, default="can0")
parser.add_argument("-i", "--id", type=int, default=1)
parser.add_argument("-t", "--time", type=float, default=15.0, help="observation time [s]")
args = parser.parse_args()

bus = recoil.Bus(channel=args.channel, bitrate=1000000)
dev = args.id

if not bus.ping(dev):
    print("[FAIL] motor offline")
    bus.stop()
    raise SystemExit(1)

bus.set_mode(dev, Mode.IDLE)  # ensure no torque
print(f"{args.time:.0f}초 동안 출력단 플랜지를 손으로 천천히 돌려주세요...")
print("(모터에 전류가 흐르지 않는 IDLE 상태입니다)")

readings = []
t_start = time.time()
last_print = 0.0
while time.time() - t_start < args.time:
    p = bus._read_parameter_f32(dev, Parameter.POSITION_CONTROLLER_POSITION_MEASURED, timeout=0.3)
    t = time.time() - t_start
    if p is not None:
        readings.append(p)
        if t - last_print > 0.5:
            bar_pos = int((p % 6.283) / 6.283 * 40)
            bar = "-" * bar_pos + "O" + "-" * (40 - bar_pos)
            print(f"t={t:5.1f}s  pos={p:+8.3f} rad  [{bar}]")
            last_print = t
    time.sleep(0.05)

bus.stop()

if len(readings) < 10:
    print("[FAIL] could not read encoder")
    raise SystemExit(1)

span = max(readings) - min(readings)
# detect large single-step jumps (glitches), ignoring legitimate fast motion
jumps = sum(1 for a, b in zip(readings, readings[1:]) if abs(b - a) > 3.0)
print(f"\n샘플 {len(readings)}개, 위치 변화 폭: {span:.2f} rad, 급점프(>3rad/스텝): {jumps}회")
if span < 0.5:
    print("[FAIL] 손으로 돌렸는데 값이 거의 안 변함 -> 엔코더가 회전을 못 읽음 (자석 탈락/미끄러짐 의심)")
elif jumps > 3:
    print("[WARN] 값이 튐 -> 엔코더 신호 불안정 (자석 간격/정렬 또는 배선 문제)")
else:
    print("[OK] 엔코더가 손 회전을 부드럽게 추적함")
