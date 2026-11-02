# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.
# Print a full health summary of one or more motors.
#
#   python motor_status.py -c can0 -i 3
#   python motor_status.py -c can0 -i 1 3 5 7 9

import argparse

import berkeley_humanoid_lite_lowlevel.recoil as recoil
from berkeley_humanoid_lite_lowlevel.recoil import Parameter


parser = argparse.ArgumentParser()
parser.add_argument("-c", "--channel", type=str, default="can0")
parser.add_argument("-i", "--ids", type=int, nargs="+", default=[1, 3, 5, 7, 9])
args = parser.parse_args()

bus = recoil.Bus(channel=args.channel, bitrate=1000000)
T = 0.5
MODE_NAMES = {0: "DISABLED", 1: "IDLE", 2: "DAMPING", 5: "CALIBRATION",
              0x10: "CURRENT", 0x11: "TORQUE", 0x12: "VELOCITY", 0x13: "POSITION"}
# NOTE: firmware bit layout (motor_controller_conf.h) — repo's Python ErrorCode is off by one bit
ERROR_BITS = {0x01: "GENERAL", 0x02: "ESTOP", 0x04: "INIT_ERROR", 0x08: "CALIB_ERROR",
              0x10: "POWERSTAGE", 0x20: "INVALID_MODE", 0x40: "WATCHDOG",
              0x80: "OVER_VOLT", 0x100: "OVER_CURR", 0x200: "OVER_TEMP",
              0x400: "CAN_RX_FAULT", 0x800: "CAN_TX_FAULT", 0x1000: "I2C_FAULT", 0x2000: "ENCODER_FAULT"}

for dev in args.ids:
    if not bus.ping(dev):
        print(f"ID {dev}: OFFLINE")
        continue
    md = bus._read_parameter_u32(dev, Parameter.MODE, timeout=T)
    er = bus._read_parameter_u32(dev, Parameter.ERROR, timeout=T)
    po = bus._read_parameter_i32(dev, Parameter.MOTOR_PHASE_ORDER, timeout=T)
    fo = bus._read_parameter_f32(dev, Parameter.ENCODER_FLUX_OFFSET, timeout=T)
    vb = bus._read_parameter_f32(dev, Parameter.POWERSTAGE_BUS_VOLTAGE_MEASURED, timeout=T)
    pp = bus._read_parameter_f32(dev, Parameter.POSITION_CONTROLLER_POSITION_MEASURED, timeout=T)
    err_names = [n for b, n in ERROR_BITS.items() if er is not None and er & b] or ["없음"]
    calib = "미실시!" if (fo is not None and fo == 0.0) else (f"{fo:.3f}" if fo is not None else "?")
    volt = f"{vb:.1f}V" if vb is not None else "?"
    pos = f"{pp:+.3f} rad" if pp is not None else "?"
    print(f"ID {dev}: {MODE_NAMES.get(md, hex(md) if md is not None else '?')}"
          f" | 에러: {','.join(err_names)}"
          f" | phase_order: {po}"
          f" | flux_offset: {calib}"
          f" | 전압: {volt}"
          f" | 위치: {pos}")

bus.stop()
