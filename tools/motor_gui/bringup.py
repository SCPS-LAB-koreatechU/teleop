#!/usr/bin/env python3
# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.
"""한 개의 모터를 연결부터 구동까지 순서대로 점검한다.

앞 단계가 실패하면 거기서 멈추고 무엇을 확인해야 하는지 알려준다. 앞 단계를
통과하지 못한 채 뒤 단계를 시도하면 버스만 상하기 때문이다(응답 없는 프레임은
컨트롤러가 무한 재시도하고, 이 어댑터는 bus-off 에서 스스로 복구하지 못한다).

  1 어댑터      USB 에 있는지, 인터페이스가 올라와 있는지, 비트레이트
  2 컨트롤러    ERROR-ACTIVE 인지 (아니면 down/up 명령 안내)
  3 ACK         버스에 듣고 있는 노드가 있는지 (송신 큐가 비워지는지로 판정)
  4 응답        해당 device id 가 recoil 프로토콜에 답하는지
  5 상태        device id/펌웨어/에러/모드/전압/위치가 말이 되는지
  6 엔코더      손으로 돌릴 때 위치가 변하는지 (토크 OFF, 사용자 참여)
  7 구동        --drive 를 줬을 때만. 작은 각도로 왕복 후 IDLE (토크 ON)

기본은 읽기 전용이다. 7단계만 모터를 움직이며, `--drive` 와 직접 입력하는
확인을 모두 요구한다.

    ./bringup.py --id 2                    # 1~6 단계 (안전)
    ./bringup.py --id 2 --drive            # 7단계까지, 기본 5도 왕복
    ./bringup.py --id 2 --drive --degrees 3 --torque-limit 0.15
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, str(REPO / "source/berkeley_humanoid_lite_lowlevel"))
sys.path.insert(0, str(HERE))

from can_util import acknowledged, bring_up_command, can_state, transient  # noqa: E402

PARAM = {
    "device_id": (0x000, "u32"),
    "firmware": (0x004, "u32"),
    "mode": (0x010, "u32"),
    "error": (0x014, "u32"),
    "kp": (0x020, "f32"),
    "kd": (0x028, "f32"),
    "torque_limit": (0x030, "f32"),
    "position_target": (0x05C, "f32"),
    "position": (0x060, "f32"),
    "volts": (0x100, "f32"),
    "encoder_raw": (0x12C, "u32"),
}
MODE_IDLE, MODE_POSITION = 0x01, 0x13
CAN_RX_FAULT = 0x400     # firmware ERROR_CAN_RX_FAULT (core.py calls this bit CAN_TX_FAULT)
EXPECTED_FIRMWARE = 0x20250226
LUT_FIRMWARE = 0x20261014       # local build: encoder nonlinearity table + I2C bad-frame recovery

OK, FAIL, WARN = "통과", "실패", "주의"


class Stop(Exception):
    """이 단계에서 멈춰야 한다."""


def say(step: str, verdict: str, detail: str = "") -> None:
    mark = {OK: "  [통과]", FAIL: "  [실패]", WARN: "  [주의]"}[verdict]
    print(f"{mark} {step}" + (f" - {detail}" if detail else ""), flush=True)


def ask(question: str) -> bool:
    try:
        answer = input(f"  {question} [y/N] ").strip().lower()
    except EOFError:
        return False
    return answer in ("y", "yes", "예", "ㅇ")


# ---------------------------------------------------------------------------

class Motor:
    """recoil 버스 호출을 감싼다. 막힌 큐(ENOBUFS)에도 죽지 않는다."""

    def __init__(self, channel: str, dev: int):
        import berkeley_humanoid_lite_lowlevel.recoil as recoil
        self.recoil = recoil
        self.bus = recoil.Bus(channel=channel, bitrate=1000000)
        self.dev = dev
        self.blocked = False

    def _guard(self, fn, *args, **kwargs):
        try:
            value = fn(*args, **kwargs)
            self.blocked = False
            return value
        except Exception as e:  # noqa: BLE001
            if not transient(e):
                raise
            self.blocked = True
            time.sleep(0.1)
            return None

    def flush(self) -> None:
        while self._guard(self.bus.receive, timeout=0) is not None:
            pass

    def ping(self, timeout: float = 0.05) -> bool:
        return bool(self._guard(self.bus.ping, self.dev, timeout=timeout))

    def read(self, name: str, timeout: float = 0.05):
        pid, kind = PARAM[name]
        fn = self.bus._read_parameter_u32 if kind == "u32" else self.bus._read_parameter_f32
        self.flush()
        return self._guard(fn, self.dev, pid, timeout=timeout)

    def read_stable(self, name: str, tries: int = 5, timeout: float = 0.05):
        """같은 값이 두 번 연속 나올 때만 믿는다(SDO 응답에는 파라미터 표시가 없다)."""
        previous = None
        for _ in range(tries):
            value = self.read(name, timeout)
            if value is not None and value == previous:
                return value
            previous = value
            time.sleep(0.01)
        return previous

    def set_mode(self, mode: int) -> None:
        self._guard(self.bus.set_mode, self.dev, mode)

    def idle(self) -> None:
        for _ in range(2):
            self.set_mode(MODE_IDLE)

    def write_gains(self, kp: float, kd: float, torque_limit: float) -> None:
        self._guard(self.bus.write_position_kp, self.dev, kp)
        self._guard(self.bus.write_position_kd, self.dev, kd)
        self._guard(self.bus.write_torque_limit, self.dev, torque_limit)

    def command(self, position: float):
        self._guard(self.bus.transmit_pdo_2, self.dev, position, 0.0)
        frame = self._guard(self.bus.receive, filter_device_id=self.dev,
                            filter_function=0b0101, timeout=0.004)   # TRANSMIT_PDO_2
        if frame is None or getattr(frame, "size", 0) < 8:
            return None
        import struct
        return struct.unpack("<ff", bytes(frame.data[0:8]))

    def feed(self) -> None:
        self._guard(self.bus.feed, self.dev)

    def close(self) -> None:
        try:
            self.bus.stop()
        except Exception:
            pass


# ---------------------------------------------------------------------------

def step_adapter(channel: str) -> dict:
    print(f"\n[1/7] 어댑터와 인터페이스 ({channel})")
    info = can_state(channel)
    if not info:
        say("인터페이스", FAIL, f"{channel} 이 없다")
        print("      USB 어댑터를 다시 꽂고 확인: lsusb | grep 1d50:606f")
        raise Stop
    if info.get("link") == "DOWN":
        say("인터페이스", FAIL, f"{channel} 이 내려가 있다")
        print("      두 줄로 따로 실행:")
        print(bring_up_command(channel))
        raise Stop
    bitrate = info["raw"].split("bitrate", 1)[1].split()[0] if "bitrate" in info["raw"] else "?"
    say("인터페이스", OK, f"UP, bitrate {bitrate}")
    if bitrate != "1000000":
        say("비트레이트", WARN, f"{bitrate} - 이 로봇의 모터는 1000000 을 쓴다")
    return info


def step_controller(channel: str, info: dict) -> None:
    print("\n[2/7] CAN 컨트롤러 상태")
    state = info.get("state", "?")
    if state == "ERROR-ACTIVE":
        say("컨트롤러", OK, f"{state} (error-passive 전이 {info.get('error_pass', '?')}회 누적)")
        return
    say("컨트롤러", FAIL, f"{state}")
    print("      응답 없는 프레임이 쌓여 컨트롤러가 상했다. 두 줄로 따로 실행:")
    print(bring_up_command(channel))
    raise Stop


def step_ack(channel: str) -> None:
    print("\n[3/7] 버스에 듣고 있는 노드가 있는가")
    print("      프레임 20개를 보내고 컨트롤러의 오류 카운터가 오르는지 본다.")
    print("      (송신 큐가 비워지는지로는 판정할 수 없다. BUS-OFF 가 되면 큐의 프레임이")
    print("       버려지면서 큐가 비워져, 아무도 없는 버스가 정상처럼 보인다.)")
    ok, detail = acknowledged(channel)
    if ok is None:
        say("ACK", FAIL, detail)
        print(bring_up_command(channel))
        raise Stop
    if not ok:
        say("ACK", FAIL, detail)
        print("      아무도 ACK 하지 않는다. 버스에 살아 있는 노드가 없다는 뜻이다.")
        print("      펌웨어 문제가 아니라 전기 문제다. 확인 순서:")
        print("        1. 모터 보드 전원 LED - 꺼져 있으면 여기서 끝")
        print("        2. 모터 쪽 CAN 커넥터 체결 (어댑터 쪽이 아니라 모터 쪽)")
        print("        3. 전원 OFF 상태에서 CAN-H ~ CAN-L 저항: 약 60옴이 정상")
        print("           120옴이면 종단이 한쪽만, 무한대면 단선")
        print("        4. CAN-H 와 CAN-L 이 뒤바뀌지 않았는지")
        print("        5. 비트레이트가 의심되면 ./can_doctor.sh (sudo 필요)")
        print("      테스트 중 컨트롤러가 상했을 수 있으니 다시 하기 전에:")
        print(bring_up_command(channel))
        raise Stop
    say("ACK", OK, detail)


def step_answer(motor: Motor, scan: bool, channel: str) -> None:
    print(f"\n[4/7] device id {motor.dev} 응답")
    state = can_state(channel).get("state", "?")
    if state != "ERROR-ACTIVE":
        say("버스", FAIL, f"컨트롤러가 {state} 로 떨어졌다 - 더 보내봐야 소용없다")
        print(bring_up_command(channel))
        raise Stop
    for _ in range(5):
        if motor.ping(timeout=0.1):
            say("ping", OK, f"id {motor.dev} 응답")
            return
    say("ping", FAIL, f"id {motor.dev} 무응답")
    if scan:
        print("      0~127 번을 훑는다...")
        found = [d for d in range(128)
                 if (setattr(motor, "dev", d) or True) and motor.ping(timeout=0.02)]
        motor.dev = motor.dev if not found else found[0]
        if found:
            print(f"      대신 응답한 id: {found}")
            print(f"      --id {found[0]} 로 다시 실행하거나, 모터의 저장된 id 를 확인할 것")
        else:
            print("      응답하는 id 가 하나도 없다. 3단계에서 ACK 는 확인됐으므로")
            print("      보드에 전원은 있고 CAN 하드웨어도 살아 있다. 펌웨어가 응답하지")
            print("      않는 상태다(멈춤 또는 부트로더). 전원을 껐다 켜고,")
            print("      그래도 같으면 펌웨어를 다시 올릴 것.")
    raise Stop


def step_health(motor: Motor) -> dict:
    print("\n[5/7] 모터가 보고하는 상태")
    values = {}
    for name in ("device_id", "firmware", "error", "mode", "volts", "position", "encoder_raw"):
        values[name] = motor.read_stable(name)
    dev_id, firmware = values["device_id"], values["firmware"]
    if dev_id == motor.dev:
        say("device id", OK, f"{dev_id}")
    else:
        say("device id", WARN, f"보고값 {dev_id}, 요청 {motor.dev}")
    if firmware in (EXPECTED_FIRMWARE, LUT_FIRMWARE):
        say("firmware", OK, hex(firmware) + (" (LUT)" if firmware == LUT_FIRMWARE else ""))
    else:
        say("firmware", WARN, f"{hex(firmware) if firmware is not None else None} "
                              f"(설정 파일 기준 {hex(EXPECTED_FIRMWARE)})")
    error = values["error"]
    if error == CAN_RX_FAULT:
        # Latched by a malformed frame (older ACK tests broadcast one), not a
        # motor fault. Clear it and carry on.
        motor._guard(motor.bus._write_parameter_u32, motor.dev, PARAM["error"][0], 0)
        error = values["error"] = motor.read_stable("error")
        say("error", WARN if error == 0 else FAIL,
            "0x400 (CAN_RX_FAULT: 잘못된 형식의 프레임을 받았던 흔적) - 지웠다"
            + ("" if error == 0 else f", 그래도 {hex(error) if error is not None else '읽기 실패'}"))
    else:
        say("error", OK if error == 0 else FAIL, hex(error) if error is not None else "읽기 실패")
    say("mode", OK, hex(values["mode"]) if values["mode"] is not None else "읽기 실패")

    volts = values["volts"]
    if volts is None:
        say("bus voltage", FAIL, "읽기 실패")
    elif volts < 6.0:
        say("bus voltage", FAIL, f"{volts:.2f} V - 구동 전원이 들어오지 않았다")
        print("      로직은 살아 있으나 파워스테이지에 전원이 없다.")
        print("      배터리 연결과 전압, 퓨즈, 전원 커넥터를 확인할 것.")
        print("      이 상태로는 통신은 되지만 모터는 절대 움직이지 않는다.")
    else:
        say("bus voltage", OK, f"{volts:.2f} V")
    say("position", OK if values["position"] is not None else FAIL,
        f"{values['position']:+.4f} rad" if values["position"] is not None else "읽기 실패")
    say("encoder raw", OK if values["encoder_raw"] else WARN,
        f"{values['encoder_raw']}" + (" - 정확히 0 이면 엔코더가 읽히지 않는 것"
                                      if not values["encoder_raw"] else ""))
    if error:
        raise Stop
    return values


def step_encoder(motor: Motor, seconds: float) -> None:
    print("\n[6/7] 엔코더 확인 (토크 OFF)")
    motor.idle()
    print(f"      지금부터 {seconds:.0f}초 동안 관절을 손으로 천천히 움직여 주세요.")
    input("      준비되면 Enter: ")
    samples = []
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        value = motor.read("position", timeout=0.03)
        if value is not None:
            samples.append(value)
        time.sleep(0.02)
    if not samples:
        say("엔코더", FAIL, "위치를 한 번도 읽지 못했다")
        raise Stop
    span = max(samples) - min(samples)
    print(f"      최소 {min(samples):+.4f} / 최대 {max(samples):+.4f} rad, "
          f"변화폭 {span:.4f} rad ({math.degrees(span):.1f} deg)")
    if span < 0.01:
        say("엔코더", FAIL, "움직였는데 값이 변하지 않았다")
        print("      엔코더가 읽히지 않는다. 구동 전원과 엔코더 케이블을 확인할 것.")
        raise Stop
    say("엔코더", OK, f"{math.degrees(span):.1f} deg 만큼 값이 따라왔다")
    print("      부호 확인: 관절을 URDF 기준 + 방향으로 돌렸을 때 값이 커져야 한다.")
    print("      반대라면 hardware_joints.yaml 의 direction 을 뒤집을 것.")


def step_drive(motor: Motor, opts, health: dict) -> None:
    print("\n[7/7] 구동 테스트 (토크 ON)")
    if not opts.drive:
        print("      건너뜀. 실행하려면 --drive 를 붙일 것.")
        return
    volts = health.get("volts")
    if volts is None or volts < 6.0:
        say("구동", FAIL, "구동 전원이 없다. 5단계를 먼저 해결할 것")
        return
    print(f"      모터를 현재 위치에서 ±{opts.degrees:.1f} deg 만큼 천천히 움직인다.")
    print(f"      kp {opts.kp}, kd {opts.kd}, torque_limit {opts.torque_limit}, "
          f"속도 {opts.speed} rad/s")
    print("      관절 주변을 비우고, 전원 스위치에 손이 닿는 위치에 있을 것.")
    print("      중단하려면 Ctrl+C - 어떤 경우에도 IDLE 로 되돌린다.")
    if not ask("지금 움직여도 됩니까?"):
        print("      취소했다. 모터는 IDLE 상태로 둔다.")
        motor.idle()
        return

    start = motor.read_stable("position")
    if start is None:
        say("구동", FAIL, "시작 위치를 읽지 못했다")
        return
    amplitude = math.radians(opts.degrees) * 15.0    # 로터 라디안 (기어비 15)
    step = opts.speed * 0.01
    motor.flush()
    motor.write_gains(opts.kp, opts.kd, opts.torque_limit)
    command = start
    replies = misses = 0
    worst = 0.0
    reach = 0.0                       # largest |measured - start| actually seen
    stopped = False
    try:
        motor.set_mode(MODE_POSITION)
        motor.feed()
        for target in (start + amplitude, start - amplitude, start):
            deadline = time.monotonic() + opts.seconds
            while time.monotonic() < deadline and abs(command - target) > 1e-4:
                command += max(-step, min(step, target - command))
                reply = motor.command(command)
                if reply is None:
                    misses += 1
                else:
                    replies += 1
                    measured = reply[0]
                    worst = max(worst, abs(measured - command))
                    reach = max(reach, abs(measured - start))
                    if abs(measured - command) > opts.max_error:
                        print(f"      [중단] 지령 {command:+.3f} 과 측정 {measured:+.3f} 이 너무 벌어졌다")
                        raise Stop
                time.sleep(0.01)
            print(f"      목표 {target:+.3f} 도달, 응답 {replies} / 미응답 {misses}")
    except (KeyboardInterrupt, Stop):
        stopped = True
        print("      중단 요청 - IDLE 로 되돌린다.")
    finally:
        motor.idle()
        print("      IDLE 로 되돌렸다.")
    final = motor.read_stable("position")
    err = motor.read_stable("error")
    print(f"      최종 위치 {final:+.4f} rad (시작 {start:+.4f}), error {hex(err) if err is not None else '?'}")
    print(f"      지령 대비 최대 추종 오차 {worst:.3f} rad (로터) = {math.degrees(worst / 15.0):.2f} deg (관절)")
    if replies == 0:
        say("구동", FAIL, "위치 응답을 한 번도 받지 못했다")
        print("      SDO 는 되는데 PDO 응답이 없다면 파워스테이지가 돌지 않는 것이다.")
        raise Stop
    if stopped:
        say("구동", FAIL, "추종 오차가 한계를 넘어 중단했다")
        raise Stop
    # A round trip ends where it started, so judge by how far it went, not where it stopped.
    elif reach < 0.5 * amplitude:
        say("구동", WARN, f"지령은 나갔으나 최대 {math.degrees(reach / 15.0):.1f} deg 밖에 "
                         f"움직이지 않았다 (목표 {opts.degrees:.1f} deg)")
        print("      torque_limit 이 너무 낮거나, 전원 전류 제한, 또는 기계적 간섭을 확인할 것.")
    else:
        say("구동", OK, f"지령을 따라 최대 {math.degrees(reach / 15.0):.1f} deg 까지 움직였다")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--id", type=int, required=True, help="CAN device id")
    ap.add_argument("-c", "--channel", default="can0")
    ap.add_argument("--scan", action="store_true", help="무응답이면 0~127 을 훑는다")
    ap.add_argument("--steps", type=int, default=7, metavar="N",
                    help="N 단계까지만 실행(기본 7). 1~2 는 프레임을 전혀 보내지 않는다")
    ap.add_argument("--hand-seconds", type=float, default=10.0, help="6단계 손 확인 시간")
    ap.add_argument("--skip-encoder", action="store_true", help="6단계 건너뛰기")
    ap.add_argument("--drive", action="store_true", help="7단계 실행 (토크 ON)")
    ap.add_argument("--degrees", type=float, default=5.0, help="7단계 왕복 각도")
    ap.add_argument("--speed", type=float, default=0.3, help="7단계 속도 [rad/s, 로터]")
    ap.add_argument("--seconds", type=float, default=8.0, help="7단계 구간별 제한 시간")
    ap.add_argument("--kp", type=float, default=0.3)
    ap.add_argument("--kd", type=float, default=0.02)
    ap.add_argument("--torque-limit", type=float, default=0.2)
    ap.add_argument("--max-error", type=float, default=0.6,
                    help="7단계: 지령과 측정 차가 이 값을 넘으면 중단 [rad, 로터] (0.6 = 관절 2.3 deg)")
    opts = ap.parse_args(argv)

    print(f"=== 모터 {opts.id} 연결/구동 점검 ({opts.channel})")
    motor = None
    try:
        info = step_adapter(opts.channel)
        if opts.steps < 2:
            return 0
        step_controller(opts.channel, info)
        if opts.steps < 3:
            return 0
        step_ack(opts.channel)
        if opts.steps < 4:
            return 0
        motor = Motor(opts.channel, opts.id)
        step_answer(motor, opts.scan, opts.channel)
        if opts.steps < 5:
            return 0
        health = step_health(motor)
        if opts.steps < 6:
            return 0
        if not opts.skip_encoder:
            step_encoder(motor, opts.hand_seconds)
        if opts.steps < 7:
            return 0
        step_drive(motor, opts, health)
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
        end = can_state(opts.channel)
        if end:
            print(f"\n컨트롤러 상태: {end.get('state', '?')} "
                  f"(error-passive 전이 누적 {end.get('error_pass', '?')})")
    print("\n=== 모든 단계 통과")
    return 0


if __name__ == "__main__":
    sys.exit(main())
