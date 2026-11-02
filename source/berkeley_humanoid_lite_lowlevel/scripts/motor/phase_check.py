# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.
# Diagnose phase current balance: command a small position offset and sample
# per-phase currents. A dead/loose phase shows near-zero current while the
# other two carry the load.

import time
import math
import argparse

import berkeley_humanoid_lite_lowlevel.recoil as recoil
from berkeley_humanoid_lite_lowlevel.recoil import Parameter, Mode


parser = argparse.ArgumentParser()
parser.add_argument("-c", "--channel", type=str, default="can0")
parser.add_argument("-i", "--id", type=int, default=1)
args = parser.parse_args()

bus = recoil.Bus(channel=args.channel, bitrate=1000000)
dev = args.id
T = 0.5

if not bus.ping(dev):
    print("[FAIL] motor offline")
    bus.stop()
    raise SystemExit(1)

vbus = bus._read_parameter_f32(dev, Parameter.POWERSTAGE_BUS_VOLTAGE_MEASURED, timeout=T)
print(f"bus voltage: {vbus:.1f} V")
if vbus is None or vbus < 10:
    print("[FAIL] motor power missing - connect battery first")
    bus.stop()
    raise SystemExit(1)

bus._write_parameter_u32(dev, Parameter.ERROR, 0)
bus.write_position_kp(dev, 0.4)
bus.write_position_kd(dev, 0.01)
bus.write_torque_limit(dev, 0.8)

p0 = bus._read_parameter_f32(dev, Parameter.POSITION_CONTROLLER_POSITION_MEASURED, timeout=T)
target = p0 + 0.5  # small rotor offset to make sustained torque flow

bus.set_mode(dev, Mode.POSITION)
bus.feed(dev)

ia_sq = ib_sq = ic_sq = 0.0
iq_sum = 0.0
n = 0
t_start = time.time()
while time.time() - t_start < 3.0:
    bus.write_read_pdo_2(dev, target, 0.0)
    ia = bus._read_parameter_f32(dev, Parameter.CURRENT_CONTROLLER_I_A_MEASURED, timeout=T)
    ib = bus._read_parameter_f32(dev, Parameter.CURRENT_CONTROLLER_I_B_MEASURED, timeout=T)
    ic = bus._read_parameter_f32(dev, Parameter.CURRENT_CONTROLLER_I_C_MEASURED, timeout=T)
    iq = bus._read_parameter_f32(dev, Parameter.CURRENT_CONTROLLER_I_Q_MEASURED, timeout=T)
    if None not in (ia, ib, ic):
        ia_sq += ia * ia; ib_sq += ib * ib; ic_sq += ic * ic
        n += 1
    if iq is not None:
        iq_sum += abs(iq)
    time.sleep(0.02)

bus.set_mode(dev, Mode.IDLE)
bus.stop()

if n == 0:
    print("[FAIL] could not read phase currents")
    raise SystemExit(1)

ia_rms = math.sqrt(ia_sq / n)
ib_rms = math.sqrt(ib_sq / n)
ic_rms = math.sqrt(ic_sq / n)
print(f"samples: {n}")
print(f"phase A RMS: {ia_rms:.2f} A")
print(f"phase B RMS: {ib_rms:.2f} A")
print(f"phase C RMS: {ic_rms:.2f} A")
print(f"|Iq| mean:   {iq_sum / max(n,1):.2f} A")

rms = [ia_rms, ib_rms, ic_rms]
hi = max(rms)
lo = min(rms)
names = "ABC"
if hi < 0.2:
    print("[WARN] almost no current flowing - powerstage not driving (check mode/power)")
elif lo < hi * 0.25:
    dead = names[rms.index(lo)]
    print(f"[FAIL] phase {dead} carries almost no current -> phase {dead} wiring/solder is broken")
else:
    print("[OK] all three phases carry balanced current - wiring is fine")
