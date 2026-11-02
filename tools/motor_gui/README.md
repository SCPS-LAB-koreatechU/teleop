# BHL 팔 모터 GUI (10축)

`berkeley_humanoid_lite_*.urdf` 의 recoil CAN 액추에이터 10개(팔당 5개: shoulder pitch/roll/yaw, elbow pitch/roll)를
한 창에서 선택·제어·모니터링하고, 같은 값을 RViz2 모델에 실시간으로 반영하는 도구입니다.

CAN ID·방향·기어비·오프셋은 MoveIt 하드웨어 브리지와 **같은** 파일
`ros2_ws/src/berkeley_humanoid_lite_moveit_config/config/hardware_joints.yaml` 에서 읽습니다.
여기서 맞춘 오프셋은 MoveIt 에서도 그대로 유효합니다.

## 실행

```bash
cd ~/Berkeley-Humanoid-Lite/tools/motor_gui
./run.sh --sim              # 하드웨어 없이 GUI 체험 (가짜 모터)
./run.sh --sim --rviz       # + RViz2 3D 모델 (한 명령으로 같이 뜨고 같이 닫힘)
./run.sh                    # 실물: can0(왼팔), can1(오른팔)
./run.sh --rviz             # 실물 + RViz2
./run.sh --sim --demo       # 시뮬 관절이 home↔ready 를 왕복 (RViz 연동 확인용)
```

## 시뮬 검증

하드웨어 없이 전체를 검증합니다. 종료코드 0 이면 전부 통과입니다.

```bash
./verify_sim.sh             # 전체 (제어·안전 16항목 + selftest + 레이아웃 + RViz 연동)
./verify_sim.sh --no-ros    # RViz 단계 생략
./sim_verify.py -k estop    # 특정 항목만
```

개별 점검:

```bash
./run.sh --selftest         # 헤드리스 제어 로직 검증 (시뮬)
./run.sh --layout-check     # 창을 보이지 않게 띄워 라벨 겹침·잘림 검사
./run.sh --control-check    # 관절 하나를 드래그·스텝·휠·키·직접입력으로 다 움직여 봄
```

`sim_verify.py` 가 확인하는 것: yaml↔URDF 정합(ID 중복·리밋), 단위 환산 왕복, Enable 거부 3종(잠금·범위 밖·무응답),
Zero here(구동 중에는 무시), Enable 시 무동작, slew rate 제한, 리밋 클램프, home/ready 프리셋 도달, E-STOP,
통신 두절 시 동결→자동 IDLE(LOST), 게인 클램프, 오프셋 저장 왕복(offset 줄만 수정·백업), CAN 없는 상태에서 무동작.

`verify_sim.sh` 의 RViz 단계: URDF 48관절이 모두 `/joint_states` 에 실리는지, 손끝 TF 가 실제로 움직이는지,
그리고 **양방향 종료**(창을 닫으면 RViz 종료 / 런처를 Ctrl+C 하면 GUI 종료)까지 확인합니다.

**검증하지 못하는 것**: 실제 CAN 링크가 필요한 것 — 회전 방향 부호, 기어비, 인코더 오프셋.
이것들은 README "실물 순서" 대로 관절 하나씩 확인해야 합니다.

## 화면

```
┌──────────────────────────────┬──────────────────────────────┐
│  로봇 다이어그램              │  ID: 7  left_elbow_pitch     │
│  (마주 본 기준: 화면 왼쪽이   │  [Enable] Ping Zero here Hold│
│   로봇의 오른팔)              │  Min ───[███]────── Max      │
│  번호 = CAN ID, 색 = 상태     │  -5 -1 -0.1 [0.0]Go to 0 +1 │
│  클릭해서 선택                │  measured / target / raw ... │
│                              ├──────────────────────────────┤
│                              │  Status: target·measured 그래프│
├──────────────────────────────┴──────────────────────────────┤
│ E-STOP | Ping all | Disable all | Enable arm L/R | 프리셋 | zero L/R | Save offsets │
│ unlock | kp kd torque [Apply gains] | max speed                         │
└─────────────────────────────────────────────────────────────┘
```

- **다이어그램**: 관절 노드를 클릭하면 오른쪽 패널이 그 모터로 바뀝니다. 노드 옆에 현재 각도가 표시되고,
  URDF 범위를 벗어나면 빨간색과 `!` 로 표시됩니다. 손목·손가락은 이 창에서 구동하지 않습니다.
- **노드 색**: 초록 ACTIVE(위치 제어 중) · 주황 online(응답, 토크 OFF) · 빨강 offline/오류 · 회색 LOCKED(yaml 비활성).
- **각도 조작**: 선택한 관절의 **절대 각도(deg)**, 범위는 URDF 리밋. 목표에 도달하는 길이 여러 개지만
  전부 같은 한 곳(`go_to`)을 지나므로 클램프·슬라이더 동기화가 서로 어긋날 수 없습니다.

  | 방법 | 크기 |
  |---|---|
  | 슬라이더 드래그 | 0.1도 단위 |
  | 스텝 버튼 `-5 -1 -0.1 / +0.1 +1 +5` | 버튼에 적힌 값 |
  | 마우스 휠 (슬라이더 위) | 1도, Shift+휠 0.1도 |
  | 방향키 ←→↑↓ | 1도, Shift+방향키 5도 |
  | 숫자 입력 후 `Go`/Enter | 정확히 그 각도 (리밋 밖이면 클램프하고 배너로 알림) |
  | `to 0` 버튼 / `0` 키 / `Home` 키 | **URDF 0 으로 바로 복귀** |
  | 하단 `zero L` / `zero R` | 그 팔의 **ACTIVE 관절 전부**를 URDF 0 으로 |

  입력칸에 타이핑하는 동안에는 방향키와 `0` 키가 관절을 움직이지 않고 커서만 움직입니다.
  Enable 되지 않은 관절에서는 슬라이더·스텝 버튼·입력칸이 모두 비활성입니다.
  0 으로 보내는 것도 **slew rate 제한을 받는 보통의 목표**입니다 — 순간이동이 아니라 램프이고,
  이동량이 20도를 넘으면 배너가 주황색으로 "몇 도를 움직인다"고 미리 알려 줍니다. Esc = E-STOP.
- **Status**: 파란선 target, 초록선 measured. 추종 지연이나 진동을 바로 볼 수 있습니다.
- **Esc** 키 = E-STOP.

## 안전 설계

- Enable 순서: ping → 에러 레지스터 클리어 → 게인 기록 → 위치 2회 읽어 일치 확인 → URDF 범위 검사 → POSITION 모드.
  하나라도 실패하면 토크가 켜지지 않습니다.
- **Enable 이 관절을 움직이지 않습니다.** 슬라이더가 측정 각도로 맞춰지고, 목표는 slew rate(기본 1 rad/s)로만 변합니다.
- 목표는 URDF 리밋 − margin(0.05 rad) 안으로 클램프됩니다. 예외 2가지: Enable 시점 위치(그래야 Enable 이 관절을 안 움직임),
  그리고 URDF 0(양쪽 elbow_pitch 는 리밋이 정확히 0 이라 마진을 그대로 적용하면 home 자세에 3° 못 미칩니다).
- 응답이 5회 연속 끊기면 목표가 동결되고, 100회(1초)면 자동 IDLE(`LOST`) 후 재-Enable 이 필요합니다.
- E-STOP 은 큐를 비우고 모든 모터에 IDLE 을 두 번 보냅니다. 창을 닫아도 같은 처리를 합니다.
- 버스 스레드에서 예외가 나면 해당 버스 모터를 IDLE 로 두고 빨간 배너를 띄웁니다.
- `--demo` 는 모든 관절을 자동으로 움직이므로 `--sim` 에서만 동작합니다(실물에서는 실행 거부).

## RViz2 연동

`--rviz` 는 `display.launch.py gui:=false` 를 함께 띄웁니다. GUI 가 `/joint_states` 를 발행하고
robot_state_publisher 가 TF 를 만들어 RViz 의 RobotModel 이 실제 모터 각도를 따라갑니다.
GUI 창을 닫으면 RViz 와 robot_state_publisher 도 함께 종료됩니다.

- 구동하지 않는 손목·손가락 관절(총 48개 중 38개)도 0 으로 함께 발행합니다. 빠뜨리면 해당 링크의 TF 가 없어
  RobotModel 이 오류로 표시되고 손이 사라집니다.
- 다른 모델을 보려면 `./run.sh --rviz --rviz-model dexhand` 처럼 지정합니다(`v1arm`·`stock`·`dexhand`·`tuning`).
- RViz 를 따로 띄우려면:
  ```bash
  cd ~/Berkeley-Humanoid-Lite/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
  ROS_DOMAIN_ID=77 ros2 launch berkeley_humanoid_lite_description display.launch.py gui:=false
  ```
  `gui:=false` 는 joint_state_publisher_gui 를 띄우지 않아 이 GUI 의 값이 덮이지 않게 합니다.

## 모터 한 개 연결/구동 점검 (bringup.py)

벤치에서 모터 하나를 처음 붙일 때 쓰는 단계별 점검입니다. 앞 단계가 실패하면 거기서 멈추고
무엇을 확인할지 알려줍니다. 통과하지 못한 채 다음 단계로 가면 버스만 상하기 때문입니다.

```bash
./bringup.py --id 2 --steps 2     # 1~2단계만. 프레임을 전혀 보내지 않음
./bringup.py --id 2               # 1~6단계. 읽기 전용, 토크 OFF
./bringup.py --id 2 --scan        # 무응답이면 0~127 훑어서 실제 id 찾기
./bringup.py --id 2 --drive       # 7단계까지. 토크 ON, 기본 ±5도 왕복
```

| 단계 | 확인 | 실패하면 |
|---|---|---|
| 1 | 어댑터·인터페이스·비트레이트 | USB 재연결, `ip link set ... up` |
| 2 | 컨트롤러가 ERROR-ACTIVE 인지 | down/up 두 줄 |
| 3 | 버스에 듣는 노드가 있는지(ACK) | 전원 LED, CAN 커넥터, 60옴 종단, 비트레이트 |
| 4 | 해당 id 가 응답하는지 | `--scan` 으로 실제 id 확인, 전원 재투입, 펌웨어 |
| 5 | id·펌웨어·에러·모드·**전압**·위치·엔코더 | 전압 0 V 면 구동 전원 문제 |
| 6 | 손으로 돌릴 때 위치가 변하는지 | 엔코더/구동 전원, 부호는 `direction` 으로 교정 |
| 7 | 작은 각도 왕복 (토크 ON) | torque_limit·전류 제한·기계 간섭 |

7단계는 `--drive` 와 화면에서 직접 입력하는 확인을 **둘 다** 요구합니다. 기본값은 ±5도,
`kp 0.3 / kd 0.02 / torque_limit 0.2`, 속도 0.3 rad/s 이며 `--degrees --kp --torque-limit` 로 줄일 수 있습니다.
Ctrl+C 를 포함해 어떤 경로로 끝나든 마지막에 IDLE 로 되돌립니다.
구동 전 전압이 6 V 미만이면 아예 시작하지 않습니다.

## 벤치에서 오른팔 모터를 can0 로 테스트하기

`hardware_joints.yaml` 은 완성된 로봇 기준이라 **오른팔(짝수 id)이 can1** 에 매핑돼 있습니다.
어댑터가 하나뿐인 벤치에서는 버스를 바꿔서 열 수 있습니다.

```bash
./run.sh --ids 2 --bus right=can0
```

## 실물이 응답하지 않을 때

```bash
./bus_probe.py                 # can0 전체 ID 스캔 + ACK 판정 (읽기 전용, sudo 불필요)
./bus_probe.py -c can1 --ids 1 3 5
./can_doctor.sh                # 비트레이트 탐색 (sudo 필요)
```

`bus_probe.py` 는 세 가지를 순서대로 가릅니다.

1. 인터페이스 존재/비트레이트
2. 우리 프레임이 실제로 컨트롤러를 떠나는지. CAN 은 수신한 노드가 하드웨어로 ACK 하므로,
   **송신 큐가 막히면** 버스에 듣는 노드가 없다는 확정적 증거입니다. 큐가 비워지면 보통 누군가 ACK 한 것이지만,
   이쪽은 확정적이지 않으니 **보드 전원 LED 가 꺼져 있으면 LED 를 믿으세요.**
3. 0~127 번 중 recoil 프로토콜에 응답하는 ID

전송은 되는데 아무도 응답하지 않는 조합이 제일 헷갈립니다. CAN 하드웨어는 MCU 가 멈춰 있어도 ACK 하므로,
**보드는 켜져 있는데 펌웨어가 응답하지 않는 상태**(멈춤 또는 부트로더)를 뜻합니다. 먼저 모터 전원을 껐다 켜 보세요.

### 전원을 껐다 켤 때마다 붙었다 떨어지는 경우

CAN 은 프레임을 받은 노드가 **하드웨어로 ACK** 합니다. 모터 전원이 꺼진 상태에서 GUI 나 스크립트가 계속 프레임을 보내면
아무도 ACK 하지 않고, 그때마다 컨트롤러의 송신 오류 카운터가 올라갑니다. 127 을 넘으면 `ERROR-PASSIVE`,
255 를 넘으면 `BUS-OFF` 가 되고, **`restart-ms 0` 이면 스스로 복구하지 않습니다.** 어댑터가 이 상태로 굳으면
모터를 다시 켜도 통신이 안 되거나 간헐적으로만 됩니다.

상태 확인과 복구:

```bash
ip -details link show can0 | grep -o "can state [A-Z-]*"     # ERROR-ACTIVE 가 정상
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 1000000
```

**이 벤치의 gs_usb(candleLight) 어댑터는 `restart-ms` 를 지원하지 않습니다.**
`restart-ms 100` 을 붙이면 `Device doesn't support restart from Bus Off` 로 **명령 전체가 실패**해서
인터페이스가 내려간 채로 남습니다. 자동 복구가 없으므로 한 번 굳으면 down/up 만이 방법입니다.
**예방이 최선입니다: 모터 전원을 끄기 전에 GUI 와 스크립트를 먼저 종료하세요.**

> **두 줄로 따로 실행하세요.** bash 에서 맨 앞의 `!` 는 파이프라인 부정 연산자라
> `! sudo ip link set can0 down && sudo ip link set can0 up ...` 은 첫 명령의 성공을 실패로 뒤집어
> `&&` 뒤를 건너뜁니다. 그러면 can0 가 내려간 채로 남습니다(state STOPPED).
> (`!` 접두사는 Claude Code 프롬프트용이지 셸용이 아닙니다.)

GUI 는 죽은 버스를 계속 두드리지 않습니다. 5 초 동안 아무 모터도 응답하지 않으면 조회 주기를 10 Hz 에서 **2 초**로 낮추고
("no answer" 표시 + 배너), 송신이 막히면 재시도 간격을 2 초까지 늘립니다. 모터가 돌아오면 자동으로 다시 붙습니다.
실측: 응답 없는 버스에 GUI 를 20 초 띄웠을 때 error-passive 전이가 **1 회**(이전에는 수백 회), 로그는 **1 줄**입니다.

라이브러리(`recoil/core.py`)도 함께 고쳤습니다. 에러 프레임이 올 때마다 수신 타임아웃이 재시작돼
에러 폭풍 중 한 번의 읽기가 몇 초씩 블록되던 것을 **절대 마감 시각**으로 바꿨고, 에러 프레임 출력은
초당 한 줄 요약(no-ACK/protocol/controller 분류와 개수)으로 제한했습니다.

GUI 는 이 상황에서 죽지 않습니다. 버스 표시가 `NO ACK` 로 바뀌고 배너에 위 복구 명령을 띄운 뒤,
전원이 돌아오면 스스로 다시 붙습니다.

## 실물 순서 (권장)

1. 전원 ON, `./run.sh --rviz`. 노드가 주황(online)이 되는지 확인(`Ping all`). 토크는 아직 OFF.
2. 관절을 손으로 돌려 다이어그램의 각도 부호가 URDF 방향과 맞는지 확인. 반대면 yaml 의 `direction` 을 뒤집고 재실행.
3. 관절을 URDF 0 자세에 두고 **Zero here** → 각도가 0 이 됨. 10개 모두 끝나면 **Save offsets**
   (yaml 의 `offset:` 값만 제자리 수정, `.bak` 백업 생성). 인코더는 전원을 껐다 켜면 값이 바뀌므로 매번 다시 필요합니다.
4. 한 관절만 Enable → 슬라이더로 몇 도 움직여 확인 → 나머지 순차 진행.
   오른팔·팔꿈치는 yaml 에서 `enabled: false` 이므로 "unlock yaml-disabled joints" 를 켜야 Enable 됩니다
   (검증이 끝나면 yaml 을 `enabled: true` 로 바꾸는 편이 안전합니다).
5. 프리셋(home/ready)과 `zero L/R` 은 ACTIVE 인 관절에만 적용됩니다.
   시험 중 자세가 꼬였으면 `to 0`(또는 `0` 키)으로 그 관절만 URDF 0 으로 되돌리는 것이 가장 빠릅니다.

## 옵션

`--config/--urdf/--srdf` 경로 지정 · `--kp --kd --torque-limit --max-speed --rate` 로 yaml 값 덮어쓰기 ·
`--unlock` 시작부터 잠금 해제 · `--ros` RViz 없이 `/joint_states` 만 발행 ·
`--sim-zeroed` 시뮬 모터를 0 에서 시작 · `--sim-offline 7 9` 시뮬에서 특정 ID 무응답 ·
**`--ids 1`** 그 CAN ID 들만 건드림 (아래 참조).

## 모터를 일부만 꽂고 쓸 때 (`--ids`)

**팔이 다 채워지지 않았으면 `--ids` 를 쓰세요.** 안 쓰면 CAN 버스가 망가집니다.

```bash
./run.sh --ids 1          # 1번 모터만 물려 있을 때
./run.sh --ids 1 3 5      # 왼쪽 어깨 3개만
```

없는 ID 로 프레임을 보내면 **아무도 ACK 를 주지 않습니다.** CAN 컨트롤러는 이걸 에러로 세어
(TEC += 8) 금방 `ERROR-PASSIVE` 로 떨어지고, 계속되면 `BUS-OFF` 로 버스가 죽습니다.
`--ids` 로 없는 모터를 아예 목록에서 빼면 그 프레임이 나가지 않습니다.

지금 상태 확인:

```bash
ip -d link show can0 | grep "can state"     # ERROR-ACTIVE 가 정상
```

`ERROR-PASSIVE` / `BUS-OFF` 면 인터페이스를 내렸다 올려야 복구됩니다:

```bash
sudo ip link set can0 down && sudo ip link set can0 up type can bitrate 1000000
```

## 의존성

시스템 python3(3.10): tkinter, pyyaml, python-can, (RViz 연동 시) rclpy.
recoil 드라이버는 `run.sh` 가 PYTHONPATH 로 잡아 줍니다. ROS 2 Humble 은 `--ros`/`--rviz` 일 때만 필요합니다.

## 표시 크기 관련 메모

이 벤치 PC 는 요청한 폰트 크기의 약 3.7배로 글자를 렌더링합니다. 그래서 다이어그램은 요청 크기가 아니라
**실제 측정된 글자 높이**를 기준으로 배치하고, 공간이 부족하면 라벨을 줄여 쓰거나 숨깁니다.
레이아웃이 깨지지 않았는지는 `--layout-check` 로 확인할 수 있습니다.

`tk.Scale` 은 홈(trough)과 손잡이를 **픽셀**로 잡기 때문에 그냥 두면 거대한 글자 사이에 머리카락 같은
줄과 좁쌀만 한 손잡이가 놓여 조준이 거의 불가능합니다. 그래서 슬라이더 치수는 `px()` 로 같은 배율을
곱해서 지정하고(손잡이 127×60 px), `--layout-check` 가 손잡이가 글자 한 줄보다 작아지지 않았는지 검사합니다.
손잡이 색은 위젯의 `bg` 이므로 홈 색과 반드시 다르게 둡니다 — 같으면 손잡이가 보이지 않습니다.
