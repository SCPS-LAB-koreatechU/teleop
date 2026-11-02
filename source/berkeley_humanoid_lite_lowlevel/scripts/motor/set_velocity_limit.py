# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.
# Set the position controller velocity limit over CAN and store to flash.

import argparse

import berkeley_humanoid_lite_lowlevel.recoil as recoil
from berkeley_humanoid_lite_lowlevel.recoil import Parameter


parser = argparse.ArgumentParser()
parser.add_argument("-c", "--channel", type=str, default="can0")
parser.add_argument("-i", "--id", type=int, default=1)
parser.add_argument("-v", "--velocity-limit", type=float, required=True, help="velocity limit [rad/s], rotor side")
args = parser.parse_args()

bus = recoil.Bus(channel=args.channel, bitrate=1000000)
dev = args.id

if not bus.ping(dev):
    print("[FAIL] motor offline")
    bus.stop()
    raise SystemExit(1)

before = bus._read_parameter_f32(dev, Parameter.POSITION_CONTROLLER_VELOCITY_LIMIT, timeout=0.5)
bus._write_parameter_f32(dev, Parameter.POSITION_CONTROLLER_VELOCITY_LIMIT, args.velocity_limit)
after = bus._read_parameter_f32(dev, Parameter.POSITION_CONTROLLER_VELOCITY_LIMIT, timeout=0.5)
print(f"velocity_limit: {before} -> {after} rad/s")

bus.store_settings_to_flash(dev)
print("stored to flash.")
bus.stop()
