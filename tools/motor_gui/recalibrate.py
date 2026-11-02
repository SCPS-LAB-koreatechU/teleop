#!/usr/bin/env python3
# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.
"""모터 한 개를 다시 캘리브레이션한다 (phase order + flux offset).

bringup.py 의 1~6 단계(어댑터, 컨트롤러, ACK, 응답, 상태, 엔코더 손 확인)를
통과한 뒤에만 캘리브레이션을 건다. 펌웨어 동작은 소스에서 확인한 그대로다
(Recoil-Motor-Controller-B-G431B-ESC1, motor_controller.c):

  * CALIBRATION 모드에서 로터를 정확히 한 바퀴 정방향, 한 바퀴 역방향으로
    돌린다 = 관절 기준 약 24 deg (기어비 15) 갔다가 돌아온다. 중간에 멈출
    방법은 없다(펌웨어가 루프 안에서 블록된다). 멈추려면 전원을 끈다.
  * 버스 전압이 9 V 미만이면 전압이 오를 때까지 무한히 기다린다
    → 여기서는 9.5 V 미만이면 아예 시작하지 않는다.
  * 끝나면 스스로 플래시에 저장하고 IDLE 로 돌아간다. 다만 저장이 실제로
    됐는지는 응답이 없으므로, 전원을 껐다 켠 뒤 --verify 로 확인해야 한다.
  * 결과 중 동작에 쓰이는 것은 flux_offset 상수 하나뿐이다. 함께 계산되는
    flux_offset_table(128칸)은 펌웨어가 쓰지 않는다. 대신 이 표가 곧
    "한 바퀴 동안 엔코더가 얼마나 비선형인가" 이므로 판정에 쓴다:
    표의 최대 오차가 전기각 90 deg 를 넘는 구간에서는 토크가 반대로 걸린다
    (= 3번 모터의 떨림/역주행 증상).

    ./recalibrate.py --id 2                  # 점검 → 손 확인 → 현재 phase order 로 2회
    ./recalibrate.py --id 2 --compare-po     # -1/+1 각각 1회 → 나은 쪽으로 1회 더 (총 3회)
    ./recalibrate.py --id 2 --po -1          # phase order 를 지정
    ./recalibrate.py --id 2 --verify         # 전원 재투입 후: 저장이 유지됐는지 (움직이지 않음)

결과와 이전 값은 ~/.config/bhl/calibration/motor<ID>.json 에 남는다.
"""

from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import bringup as B  # noqa: E402
from bringup import FAIL, OK, WARN, Stop, ask, say  # noqa: E402

# MotorController 구조체 오프셋 = SDO 파라미터 id (core.py Parameter 와 같음)
P_MODE, P_ERROR = 0x010, 0x014
P_VOLTS = 0x100
P_POLE_PAIRS, P_PHASE_ORDER, P_CAL_CURRENT = 0x104, 0x10C, 0x110
P_CPR, P_POSITION_OFFSET = 0x120, 0x124
P_FLUX_OFFSET, P_FLUX_TABLE = 0x13C, 0x140
P_LUT_VALID = 0x11C   # LUT firmware: marks flux_offset_table as built by its calibration (padding on stock firmware)
LUT_ENTRIES = 128

MODE_IDLE, MODE_CALIBRATION = 0x01, 0x05
ERR_CALIBRATION = 0x0008

MIN_CAL_VOLTS = 9.5          # 펌웨어는 9 V 미만이면 무한 대기
CAL_TIMEOUT = 45.0           # [s] 보통 10~15 s
REPEAT_TOL = math.radians(8.0)     # 같은 phase order 두 번의 flux offset 차 (전기각)
SAME_PO_TOL = math.radians(15.0)   # -1 과 +1 결과가 이보다 가까우면 로터가 안 돈 것
LUT_OK, LUT_BAD = 20.0, 45.0       # [deg 전기각] 표 최대 오차 판정 기준

OUT_DIR = Path.home() / ".config/bhl/calibration"


def wrap(angle: float) -> float:
    """(-pi, pi] 로. flux offset 은 전기각이라 2pi 배수 차이는 같은 값이다
    (로터가 어느 극에 붙었느냐에 따라 2pi*k 만큼 달라진다)."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


# ---------------------------------------------------------------------------

class CalMotor(B.Motor):
    def raw(self, pid: int, kind: str, timeout: float = 0.05, tries: int = 4):
        fn = {"u32": self.bus._read_parameter_u32, "i32": self.bus._read_parameter_i32,
              "f32": self.bus._read_parameter_f32}[kind]
        for _ in range(tries):
            self.flush()
            value = self._guard(fn, self.dev, pid, timeout=timeout)
            if value is not None:
                return value
        return None

    def stable(self, pid: int, kind: str):
        """같은 값이 두 번 연속 나와야 믿는다 (SDO 응답에는 어떤 파라미터인지 표시가 없다)."""
        previous = None
        for _ in range(6):
            value = self.raw(pid, kind)
            if value is not None and value == previous:
                return value
            previous = value
        return previous

    def write_i32(self, pid: int, value: int) -> None:
        self._guard(self.bus._write_parameter_i32, self.dev, pid, value)

    def write_u32(self, pid: int, value: int) -> None:
        self._guard(self.bus._write_parameter_u32, self.dev, pid, value)

    def table(self) -> list[float] | None:
        values = []
        for k in range(LUT_ENTRIES):
            value = self.raw(P_FLUX_TABLE + 4 * k, "f32")
            if value is None or not math.isfinite(value):
                return None
            values.append(value)
        return values

    def snapshot(self) -> dict:
        return {
            "phase_order": self.stable(P_PHASE_ORDER, "i32"),
            "flux_offset": self.stable(P_FLUX_OFFSET, "f32"),
            "pole_pairs": self.stable(P_POLE_PAIRS, "u32"),
            "cpr": self.stable(P_CPR, "u32"),
            "calibration_current": self.stable(P_CAL_CURRENT, "f32"),
            "position_offset": self.stable(P_POSITION_OFFSET, "f32"),
            "error": self.stable(P_ERROR, "u32"),
        }


def table_stats(table: list[float] | None) -> dict:
    if not table:
        return {"max_deg": None, "pp_deg": None}
    return {"max_deg": round(math.degrees(max(abs(v) for v in table)), 1),
            "pp_deg": round(math.degrees(max(table) - min(table)), 1)}


def table_verdict(max_deg: float | None) -> tuple[str, str]:
    if max_deg is None:
        return FAIL, "표를 읽지 못했다"
    if max_deg < LUT_OK:
        return OK, f"한 바퀴 최대 정류 오차 {max_deg:.0f} deg (전기각) - 엔코더 정상"
    if max_deg < 90:
        # Not a FAIL: the forward/backward average cancels friction but not
        # gravity, so an arm hanging off the joint adds lag to this table, and
        # no known-good motor has been measured on this arm yet to set a bar.
        return WARN, (f"최대 정류 오차 {max_deg:.0f} deg - 그 구간에서 토크가 "
                      f"{100 * math.cos(math.radians(max_deg)):.0f}% 로 준다"
                      + (". 자세(중력)를 바꿔 다시 재서 표 모양이 따라 변하면 부하, "
                         "그대로면 엔코더" if max_deg >= LUT_BAD else ""))
    return FAIL, (f"최대 정류 오차 {max_deg:.0f} deg (> 90) - 토크가 반대로 걸리는 구간이 있다 "
                  "(phase order 가 틀렸거나 엔코더가 로터를 제대로 못 본다)")


# ---------------------------------------------------------------------------

def others_on_bus() -> list[str]:
    """같은 버스를 쓰는 다른 프로그램. 캘리브레이션 중 PDO 를 쏘면 안 된다."""
    try:
        out = subprocess.run(["pgrep", "-af", r"[h]ardware_bridge\.py|[b]hl_motor_gui|[b]ringup\.py"],
                             capture_output=True, text=True, timeout=2).stdout
    except Exception:  # noqa: BLE001
        return []
    return [line for line in out.splitlines() if line.strip()]


def run_once(motor: CalMotor, po: int, label: str) -> dict:
    print(f"\n  --- {label}: phase order {po:+d}")
    motor.idle()
    time.sleep(0.2)
    motor.write_i32(P_PHASE_ORDER, po)
    if motor.stable(P_PHASE_ORDER, "i32") != po:
        say("phase order 쓰기", FAIL, "읽어 보니 반영되지 않았다")
        raise Stop
    motor.write_u32(P_ERROR, 0)
    motor.flush()
    motor.set_mode(MODE_CALIBRATION)
    start = time.monotonic()
    seen_cal = False
    print("      돌고 있다", end="", flush=True)
    try:
        while True:
            time.sleep(0.5)
            print(".", end="", flush=True)
            mode = motor.raw(P_MODE, "u32", timeout=0.05, tries=1)
            if mode == MODE_CALIBRATION:
                seen_cal = True
            elapsed = time.monotonic() - start
            if mode == MODE_IDLE and (seen_cal or elapsed > 3.0):
                break
            if elapsed > CAL_TIMEOUT:
                print()
                motor.idle()
                say("캘리브레이션", FAIL, f"{CAL_TIMEOUT:.0f} s 안에 끝나지 않았다 (mode {mode})")
                print("      버스 전압이 9 V 아래로 떨어지면 펌웨어가 무한히 기다린다.")
                print("      모터 전원을 껐다 켜고 전압부터 확인할 것.")
                raise Stop
    except KeyboardInterrupt:
        print()
        motor.idle()
        print("      [중단] IDLE 을 보냈다. 펌웨어는 캘리브레이션 중 명령을 받지 않을 수 있다 -")
        print("      로터가 계속 돌면 모터 전원을 끌 것.")
        raise
    took = time.monotonic() - start
    print(f" {took:.1f} s")

    error = motor.stable(P_ERROR, "u32")
    flux = motor.stable(P_FLUX_OFFSET, "f32")
    table = motor.table()
    stats = table_stats(table)
    result = {"phase_order": po, "flux_offset": flux, "error": error, "seconds": round(took, 1),
              "table": table, **stats}
    if error and error & ERR_CALIBRATION:
        say("캘리브레이션", FAIL, "펌웨어가 CALIBRATION_ERROR 를 냈다 - 목표 전류에 못 미침 "
                              "(상 단선, 커넥터, 전원 전류 제한)")
        raise Stop
    flux_txt = f"{flux:+.3f}" if flux is not None else "읽기 실패"
    print(f"      flux_offset {flux_txt} rad (전기각, 2pi 기준 {math.degrees(wrap(flux or 0.0)):+.0f} deg), "
          f"표 최대 {stats['max_deg']} deg / p-p {stats['pp_deg']} deg, error {hex(error or 0)}")
    return result


def restore(motor: CalMotor, before: dict) -> None:
    """The firmware stores every calibration, garbage included. Put the old
    phase order / flux offset back and ask it to store them again."""
    if before.get("phase_order") not in (-1, 1) or before.get("flux_offset") is None:
        say("되돌리기", WARN, "이전 값이 없어 되돌리지 못했다")
        return
    motor.idle()
    motor.write_i32(P_PHASE_ORDER, before["phase_order"])
    motor._guard(motor.bus._write_parameter_f32, motor.dev, P_FLUX_OFFSET, before["flux_offset"])
    # the table from the run being undone does not match the old flux offset
    motor.write_u32(P_LUT_VALID, 0)
    ok = (motor.stable(P_PHASE_ORDER, "i32") == before["phase_order"]
          and motor.stable(P_FLUX_OFFSET, "f32") == before["flux_offset"])
    motor._guard(motor.bus.store_settings_to_flash, motor.dev)
    time.sleep(0.5)
    say("되돌리기", OK if ok else FAIL,
        f"이전 값(phase order {before['phase_order']:+d}, flux {before['flux_offset']:+.3f})을 "
        "다시 쓰고 저장을 요청했다" if ok else "이전 값을 다시 쓰지 못했다")


def calibrate(motor: CalMotor, opts, health: dict, record: dict) -> int:
    print("\n[캘리브레이션 1/2] 준비")
    volts = health.get("volts")
    if volts is None or volts < MIN_CAL_VOLTS:
        say("bus voltage", FAIL, f"{volts} V - {MIN_CAL_VOLTS} V 이상이어야 한다 "
                                 "(펌웨어는 9 V 미만이면 끝없이 기다리고 멈출 방법이 없다)")
        raise Stop
    busy = others_on_bus()
    if busy:
        say("버스 독점", FAIL, "같은 CAN 을 쓰는 프로그램이 떠 있다. 먼저 종료할 것:")
        for line in busy:
            print("        " + line[:120])
        raise Stop
    before = motor.snapshot()
    before["table"] = motor.table()
    before.update(table_stats(before["table"]))
    record["before"] = before
    save(record, opts.id)
    print(f"      현재 값: phase order {before['phase_order']}, flux_offset {before['flux_offset']}, "
          f"pole pairs {before['pole_pairs']}, cpr {before['cpr']}, "
          f"보정 전류 {before['calibration_current']} A")
    print(f"      현재 표 최대 오차 {before['max_deg']} deg  (백업: {path_for(opts.id)})")
    if before["pole_pairs"] != 14:
        say("pole pairs", WARN, f"{before['pole_pairs']} - 이 로봇의 모터는 14")
    if before["position_offset"]:
        say("position offset", WARN, f"{before['position_offset']} - 정류각 계산에 들어간다. "
                                     "캘리브레이션 뒤에 바꾸면 정류가 틀어진다")

    current = before["phase_order"] if before["phase_order"] in (-1, 1) else -1
    chosen = opts.po if opts.po is not None else current
    plan = [-1, 1] if opts.compare_po else [chosen, chosen]
    print(f"\n      실행 계획: {len(plan) + (1 if opts.compare_po else 0)}회 "
          + ("(-1, +1 각 1회 → 나은 쪽으로 1회 더)" if opts.compare_po else f"(phase order {chosen:+d} 로 2회)"))
    print("      매 회 로터가 한 바퀴 돌았다 돌아온다 = 관절 약 24 deg 왕복, 10~15 초.")
    print("      * 관절이 양쪽으로 25 deg 이상 자유롭게 움직일 수 있는 자세에 둘 것")
    print("      * 팔이 다른 관절에 걸리거나 사람 손이 닿지 않게 할 것")
    print("      * 도중에 멈출 수 없다. 이상하면 모터 전원을 끌 것")
    if not ask("지금 시작해도 됩니까?"):
        print("      취소했다. 아무 것도 바꾸지 않았다.")
        return 1

    print("\n[캘리브레이션 2/2] 실행")
    runs = [run_once(motor, po, f"{k + 1}회차") for k, po in enumerate(plan)]
    if opts.compare_po:
        a, b = runs
        same_flux = (a["flux_offset"] is not None and b["flux_offset"] is not None
                     and abs(wrap(a["flux_offset"] - b["flux_offset"])) < SAME_PO_TOL)
        # Exactly one phase order is right; if neither gives a sane table the
        # encoder is not following the rotor (a static encoder reads as a
        # ramp: table max ~ pole_pairs * 180 deg, 2520 deg for 14).
        both_bad = all(r["max_deg"] is None or r["max_deg"] >= 90 for r in (a, b))
        if same_flux or both_bad:
            maxes = sorted((r["max_deg"] or 1e9, r["phase_order"]) for r in (a, b))
            static = 180.0 * (before.get("pole_pairs") or 14)    # encoder not moving at all
            if not same_flux and maxes[1][0] > 1.5 * static and maxes[0][0] < 0.5 * static:
                # The wrong phase order reads ~2x static (encoder runs the other
                # way), so the encoder does follow the rotor -- just unevenly.
                say("phase order 비교", FAIL,
                    f"{maxes[0][1]:+d} 이 맞는 방향이고 엔코더도 회전을 따라오지만, 한 바퀴 안의 "
                    f"오차가 {maxes[0][0]:.0f} deg (전기각) 로 너무 크다 = 자석이 축 중심에서 "
                    "벗어났거나 기울었다. 자석을 다시 맞출 것")
            else:
                say("phase order 비교", FAIL,
                    f"-1/+1 표 최대 {a['max_deg']}/{b['max_deg']} deg (엔코더 정지 시 약 {static:.0f}) "
                    "= 로터는 돌았는데 엔코더가 거의 따라오지 않았다 (자석 이탈/헛돎, 엔코더 칩, 케이블)")
            record["runs"] = runs
            save(record, opts.id)
            restore(motor, before)
            return 1
        best = min(runs, key=lambda r: r["max_deg"] if r["max_deg"] is not None else 1e9)
        chosen = best["phase_order"]
        say("phase order 비교", OK, f"-1: 표 최대 {a['max_deg']} deg, +1: {b['max_deg']} deg "
                                  f"→ {chosen:+d} 선택")
        runs.append(run_once(motor, chosen, "확인 회차"))
        pair = [best, runs[-1]]
    else:
        pair = runs
    record["runs"] = runs
    final = runs[-1]
    record["after"] = {k: final[k] for k in ("phase_order", "flux_offset", "max_deg", "pp_deg")}
    record["calibrated_at"] = time.strftime("%Y-%m-%d %H:%M:%S")
    record["verified_after_power_cycle"] = False
    save(record, opts.id)

    print("\n=== 판정")
    verdicts = []
    if None in (pair[0]["flux_offset"], pair[1]["flux_offset"]):
        verdicts.append(FAIL)
        say("반복성", FAIL, "flux offset 을 읽지 못했다")
    else:
        diff = math.degrees(abs(wrap(pair[0]["flux_offset"] - pair[1]["flux_offset"])))
        verdict = OK if diff <= math.degrees(REPEAT_TOL) else FAIL
        verdicts.append(verdict)
        say("반복성", verdict, f"같은 phase order 두 번의 차 {diff:.1f} deg (전기각, 기준 "
                              f"{math.degrees(REPEAT_TOL):.0f} deg)")
    verdict, text = table_verdict(final["max_deg"])
    verdicts.append(verdict)
    say("엔코더 선형성", verdict, text)
    if verdict == FAIL:
        # A result that reverses torque somewhere is worse than whatever was
        # there before; do not leave it in flash.
        restore(motor, before)
        record["after"] = None
        save(record, opts.id)
    if before.get("flux_offset") is not None and final["flux_offset"] is not None:
        print(f"      이전 대비: phase order {before['phase_order']} → {final['phase_order']}, "
              f"flux offset 차 {math.degrees(wrap(final['flux_offset'] - before['flux_offset'])):+.0f} deg")

    if FAIL in verdicts:
        print("\n캘리브레이션 결과를 쓰지 않았다. 원인을 해결한 뒤 다시 실행할 것.")
        return 1
    print("\n다음 순서:")
    print("  1. 모터 전원을 껐다 켠다 (펌웨어가 플래시에 저장했는지는 재부팅 후에만 알 수 있다)")
    print(f"  2. ./recalibrate.py --id {opts.id} --verify")
    print(f"  3. ./bringup.py --id {opts.id} --drive --degrees 3   (작은 왕복 구동 확인)")
    print("  4. 드래그 텔레옵은 엔코더 기준이 바뀌었으니 new_reference:=true 로 시작")
    return 0 if all(v != FAIL for v in verdicts) else 1


def verify(motor: CalMotor, opts) -> int:
    print("\n[저장 확인] 전원을 껐다 켠 뒤")
    saved = load(opts.id).get("after")
    if not saved:
        say("기록", FAIL, f"{path_for(opts.id)} 에 캘리브레이션 결과가 없다")
        return 1
    now = motor.snapshot()
    ok = True
    if now["phase_order"] == saved["phase_order"]:
        say("phase order", OK, f"{now['phase_order']:+d}")
    else:
        ok = False
        say("phase order", FAIL, f"지금 {now['phase_order']}, 캘리브레이션 결과 {saved['phase_order']}")
    if now["flux_offset"] is None or not math.isfinite(now["flux_offset"]):
        ok = False
        say("flux offset", FAIL, f"{now['flux_offset']} - 설정 페이지가 비었다(NaN). 이 경우 펌웨어는 "
                                 "device id 까지 전부 기본값으로 뜬다")
    else:
        diff = math.degrees(abs(wrap(now["flux_offset"] - saved["flux_offset"])))
        verdict = OK if diff < 1.0 else FAIL
        ok = ok and verdict == OK
        say("flux offset", verdict, f"지금 {now['flux_offset']:+.3f}, 저장 {saved['flux_offset']:+.3f} "
                                   f"(차 {diff:.2f} deg)")
    if not ok:
        print("      저장이 유지되지 않았다. 펌웨어의 저장(storeConfig)은 결과를 알려주지 않는다.")
        print("      구동 전원이 약하면 플래시 쓰기가 실패할 수 있다 - 전압을 확인하고 다시 캘리브레이션.")
        return 1
    record = load(opts.id)
    record["verified_after_power_cycle"] = time.strftime("%Y-%m-%d %H:%M:%S")
    save(record, opts.id)
    print("\n=== 저장 유지 확인. 다음: ./bringup.py --id %d --drive --degrees 3" % opts.id)
    return 0


# ---------------------------------------------------------------------------

def path_for(dev: int) -> Path:
    return OUT_DIR / f"motor{dev}.json"


def load(dev: int) -> dict:
    try:
        return json.loads(path_for(dev).read_text())
    except (OSError, ValueError):
        return {}


def save(record: dict, dev: int) -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    path_for(dev).write_text(json.dumps(record, indent=1))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--id", type=int, required=True, help="CAN device id")
    ap.add_argument("-c", "--channel", default="can0")
    ap.add_argument("--po", type=int, choices=(-1, 1), default=None,
                    help="phase order 지정 (기본: 모터에 지금 들어 있는 값)")
    ap.add_argument("--compare-po", action="store_true",
                    help="-1/+1 을 둘 다 돌려 비교하고 나은 쪽으로 한 번 더")
    ap.add_argument("--verify", action="store_true", help="전원 재투입 후 저장 확인만 (움직이지 않음)")
    ap.add_argument("--skip-encoder", action="store_true", help="손 확인 건너뛰기")
    ap.add_argument("--hand-seconds", type=float, default=8.0)
    opts = ap.parse_args(argv)
    if opts.compare_po and opts.po is not None:
        ap.error("--compare-po 와 --po 는 함께 쓸 수 없다")

    what = "저장 확인" if opts.verify else "캘리브레이션"
    print(f"=== 모터 {opts.id} {what} ({opts.channel})")
    motor = None
    try:
        info = B.step_adapter(opts.channel)
        B.step_controller(opts.channel, info)
        B.step_ack(opts.channel)
        motor = CalMotor(opts.channel, opts.id)
        B.step_answer(motor, False, opts.channel)
        health = B.step_health(motor)
        if opts.verify:
            return verify(motor, opts)
        if not opts.skip_encoder:
            B.step_encoder(motor, opts.hand_seconds)
        record = load(opts.id)
        record.pop("verified_after_power_cycle", None)
        return calibrate(motor, opts, health, record)
    except Stop:
        print("\n=== 여기서 멈춘다. 위 안내를 처리한 뒤 다시 실행할 것.")
        return 1
    except KeyboardInterrupt:
        print("\n=== 중단됨")
        return 130
    finally:
        if motor is not None:
            motor.idle()
            motor.close()


if __name__ == "__main__":
    sys.exit(main())
