# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.
# Set MOTOR_PHASE_ORDER over CAN and store it to flash.
# Wrong phase order turns position control into positive feedback (vibration).

import argparse

import berkeley_humanoid_lite_lowlevel.recoil as recoil
from berkeley_humanoid_lite_lowlevel.recoil import Parameter


parser = argparse.ArgumentParser()
parser.add_argument("-c", "--channel", type=str, default="can0")
parser.add_argument("-i", "--id", type=int, default=1)
parser.add_argument("-p", "--phase-order", type=int, choices=[1, -1], required=True)
args = parser.parse_args()

bus = recoil.Bus(channel=args.channel, bitrate=1000000)
dev = args.id

if not bus.ping(dev):
    print("[FAIL] motor offline")
    bus.stop()
    raise SystemExit(1)

before = bus._read_parameter_i32(dev, Parameter.MOTOR_PHASE_ORDER, timeout=0.5)
bus._write_parameter_i32(dev, Parameter.MOTOR_PHASE_ORDER, args.phase_order)
after = bus._read_parameter_i32(dev, Parameter.MOTOR_PHASE_ORDER, timeout=0.5)
print(f"phase_order: {before} -> {after}")

bus.store_settings_to_flash(dev)
print("stored to flash.")
print("다음: 캘리브레이션을 다시 실행하세요 (phase_order가 바뀌면 flux_offset도 다시 측정해야 합니다)")
bus.stop()
