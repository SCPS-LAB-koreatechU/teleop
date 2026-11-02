# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.
# Variant of move_actuator.py with tunable output parameters.

import time
import argparse

import numpy as np
from loop_rate_limiters import RateLimiter

import berkeley_humanoid_lite_lowlevel.recoil as recoil


parser = argparse.ArgumentParser()
parser.add_argument("-c", "--channel", help="CAN transport channel", type=str, default="can0")
parser.add_argument("-i", "--id", help="CAN device ID", type=int, default=1)
parser.add_argument("-a", "--amplitude", help="motion amplitude [rad]", type=float, default=1.0)
parser.add_argument("-f", "--frequency", help="motion frequency [Hz]", type=float, default=1.0)
parser.add_argument("--kp", help="position P gain", type=float, default=0.2)
parser.add_argument("--kd", help="position D gain", type=float, default=0.005)
parser.add_argument("--torque-limit", help="torque limit", type=float, default=0.2)
parser.add_argument("--duration", help="test duration [s], 0 = run until Ctrl+C", type=float, default=0.0)
args = parser.parse_args()

bus = recoil.Bus(channel=args.channel, bitrate=1000000)
device_id = args.id

print(f"[test] id={device_id} amplitude={args.amplitude} rad, frequency={args.frequency} Hz, "
      f"kp={args.kp}, kd={args.kd}, torque_limit={args.torque_limit}")

if not bus.ping(device_id):
    print("[test] WARNING: motor did not respond to ping. Aborting for safety.")
    bus.stop()
    raise SystemExit(1)

bus.write_position_kp(device_id, args.kp)
bus.write_position_kd(device_id, args.kd)
bus.write_torque_limit(device_id, args.torque_limit)

bus.set_mode(device_id, recoil.Mode.POSITION)
bus.feed(device_id)

rate = RateLimiter(frequency=200.0)
start_time = time.time()

try:
    while True:
        t = time.time() - start_time
        if args.duration > 0 and t > args.duration:
            break

        target_angle = np.sin(2 * np.pi * args.frequency * t) * args.amplitude

        measured_position, measured_velocity = bus.write_read_pdo_2(device_id, target_angle, 0.0)
        if measured_position is not None and measured_velocity is not None:
            print(f"t: {t:6.2f}  target: {target_angle:+.3f}  pos: {measured_position:+.3f}  vel: {measured_velocity:+.3f}")

        rate.sleep()

except KeyboardInterrupt:
    pass

bus.set_mode(device_id, recoil.Mode.IDLE)
bus.stop()
print("[test] done, motor set to IDLE")
