# bhl_arm_sdk

Berkeley Humanoid Lite 양팔 모터(recoil 액추에이터, CAN)를 **모터별 각도(도)** 로 제어하는 ROS 2 패키지입니다.

- 파라미터 하나로 관절 하나의 각도를 지정합니다: `target.<관절이름>` [deg]
- 슬라이더 GUI로 모터마다 각도를 조절합니다
- Unitree G1 `g1_arm7_sdk_dds_example.cpp`와 같은 순서(현재 자세 → 목표 → 복귀 → 해제)로 동작을 스크립트로 실행합니다

로봇 컴퓨터와 원격 PC로 나눠 띄우는 bring-up(`/joint_states` 병합, `/tf`, `/diagnostics`, 네트워크, CAN 자동 설정)은 [`bhl_bringup`](../bhl_bringup/README.md)에 있습니다. 아래 `arm_sdk.launch.py`는 한 PC에서 벤치 작업할 때의 구성입니다.

참고한 코드:
- [HybridRobotics/Berkeley-Humanoid-Lite-Lowlevel@652777c](https://github.com/HybridRobotics/Berkeley-Humanoid-Lite-Lowlevel/tree/652777cc7c49884e7cd7ddfada758dc1979bf627): `recoil/core.py`(CAN 프레임), `robot/bimanual.py`(모터 초기화 순서, 시작 자세를 0점으로)
- [unitree_sdk2 g1_arm7_sdk_dds_example.cpp](https://github.com/unitreerobotics/unitree_sdk2/blob/main/example/g1/high_level/g1_arm7_sdk_dds_example.cpp): 현재 자세에서 시작, 관절당 최대 속도로 목표까지 이동, 해제 단계

## 빠른 시작

```bash
cd ~/humanoid_sdk/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select bhl_arm_sdk
source install/setup.bash

ros2 launch bhl_arm_sdk arm_sdk.launch.py                   # 연습 모드 + 슬라이더 GUI + RViz (CAN 안 엶)
ros2 launch bhl_arm_sdk arm_sdk.launch.py dry_run:=false    # 실물 (CAN 필요)
ros2 launch bhl_arm_sdk arm_sdk.launch.py rviz:=false       # RViz 없이
```

RViz에 상반신 모델(`berkeley_humanoid_lite_v1arm.urdf`)을 그리려면 Berkeley-Humanoid-Lite 워크스페이스도 source 해야 합니다(메시가 `package://berkeley_humanoid_lite_description` 경로). 없으면 RViz만 빠지고 나머지는 그대로 뜹니다. 다른 URDF는 `urdf:=<파일>`로 줍니다.

```bash
source ~/Berkeley-Humanoid-Lite/ros2_ws/install/setup.bash
source ~/humanoid_sdk/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=92      # 다른 세션(DexHand MoveIt은 도메인 0)과 /joint_states 가 섞이지 않게
ros2 launch bhl_arm_sdk arm_sdk.launch.py
```

RViz를 켜면 arm_controller의 `/joint_states`(팔 10개)는 `/arm_controller/joint_states`로 바뀌고, `joint_state_publisher`가 나머지 관절(손목, 손가락)을 0으로 채워 `/joint_states`(48개)로 내보냅니다.

`dry_run`은 기본값이 `true`입니다. 연습 모드에서는 CAN을 열지 않고, 측정 각도가 명령 각도를 그대로 따라갑니다.

## 사용법

토크는 꺼진 상태로 시작합니다. 먼저 켭니다(현재 자세를 그대로 유지하며 위치 제어 모드로 들어감):

```bash
ros2 service call /arm_controller/enable std_srvs/srv/SetBool "{data: true}"
```

### 1. 파라미터로 모터별 각도 지정

```bash
ros2 param set /arm_controller target.arm_left_elbow_pitch_joint 30.0     # 소수점 필요 (double)
ros2 param set /arm_controller target.arm_right_shoulder_pitch_joint -20.0
ros2 param get /arm_controller target.arm_left_elbow_pitch_joint
ros2 param set /arm_controller max_speed_deg 15.0                          # 관절당 최대 속도 [deg/s]
```

각 파라미터에는 관절 한계가 범위로 걸려 있어서, 범위를 벗어난 값은 거부됩니다. 여러 관절을 한 번에 지정하려면 YAML 파일을 씁니다:

```yaml
# pose.yaml
arm_controller:
  ros__parameters:
    target.arm_left_elbow_pitch_joint: 30.0
    target.arm_right_elbow_pitch_joint: -30.0
```
```bash
ros2 param load /arm_controller pose.yaml
```

### 2. 토픽으로 지정 (라디안, 일부 관절만 보내도 됨)

```bash
ros2 topic pub --once /arm_controller/target sensor_msgs/msg/JointState \
  "{name: [arm_left_elbow_pitch_joint], position: [0.5]}"
```

### 3. 슬라이더 GUI

`arm_sdk.launch.py`가 함께 띄웁니다(따로 실행하려면 `ros2 run bhl_arm_sdk slider_gui`).
모터마다 슬라이더(0.1° 단위, 관절 한계 범위), 목표/측정 각도, 상태(구동 중 / 토크 꺼짐 / 오류)가 표시됩니다.
위쪽 버튼은 토크 켜기 / 끄기 / 비상정지 / 비상정지 해제 / 모두 0°이고, 속도 슬라이더로 `max_speed_deg`를 바꿉니다.

### 4. G1 예제 방식의 동작 스크립트

```bash
ros2 run bhl_arm_sdk arm_sequence --pose arm_left_elbow_pitch_joint=30 arm_right_elbow_pitch_joint=-30 --hold 2
```
켜기 → 목표 각도로 이동 → `--hold`초 유지 → 0°로 복귀 → 해제(댐핑) 순서입니다. `--no-release`를 주면 목표에서 멈추고 토크를 유지합니다.

### 서비스 / 토픽

| 이름 | 형식 | 설명 |
|---|---|---|
| `/arm_controller/enable` | `std_srvs/SetBool` | true: 현재 자세 유지하며 위치 제어 시작, false: 댐핑 |
| `/arm_controller/estop` | `std_srvs/Trigger` | 모든 관절 댐핑, 해제 전까지 다시 켜지지 않음 |
| `/arm_controller/reset_estop` | `std_srvs/Trigger` | 비상정지 해제 (토크는 꺼진 채) |
| `/arm_controller/home` | `std_srvs/Trigger` | 모든 목표를 0° |
| `/arm_controller/target` | `sensor_msgs/JointState` (입력) | 목표 [rad] |
| `/joint_states` | `sensor_msgs/JointState` (출력) | 측정 각도 [rad] (bring-up에서는 `/arm_controller/joint_states`로 리맵되고 `joint_state_publisher`가 `/joint_states`를 냄) |
| `/arm_controller/status` | `std_msgs/String` (출력) | JSON: 관절별 측정 / 명령 / 목표 / 오류 |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` (출력, 1 Hz) | 요약 1개 + 관절별 1개: OK 구동 중, WARN 출력 꺼짐, ERROR 고장(`NO_REPLY`, `LOST`, `TRACKING` …)이나 비상정지. `rqt_robot_monitor`로 봄 |

## 설정 (`config/bhl_arms.yaml`)

관절마다 `bus`, `device_id`, `direction`, `gear_ratio`, 한계(`lower_deg`, `upper_deg`), 게인(`kp`, `kd`, `torque_limit`)을 적습니다.

```
모터 위치 = motor_zero + direction × gear_ratio × 관절각[rad]
```

- 이 연구실 모터는 펌웨어 기어비가 1이라 로터 라디안을 보고합니다. 그래서 `gear_ratio: 15`(감속기)를 여기서 곱합니다.
- `zero: start`(기본)는 토크를 켤 때의 자세를 0°로 잡습니다(원본 Bimanual과 같음). **켜기 전에 팔을 기준 자세에 두세요.**
- `zero: config`이면 관절마다 `motor_zero`(로터 rad)를 고정합니다.
- 어댑터 하나로 양팔을 쓰면 `buses`의 `left`, `right`를 둘 다 `can0`으로 둡니다(현재 기본값).
- 게인 기본값(0.3 / 0.01 / 0.5)은 이 연구실 하드웨어 브리지 값입니다. 원본 Bimanual의 20 / 2 / 1은 펌웨어 기어비를 감속기 값으로 둔 경우라 여기서는 너무 셉니다.

## 안전 장치

- 핑에 응답하지 않는 모터는 켜지 않습니다 (`NO_REPLY`)
- 켤 때 각도가 한계에서 5° 넘게 벗어나 있으면 켜지 않습니다 (`OUT_OF_RANGE`)
- 응답이 25주기(0.5초) 연속 끊기면 그 모터만 댐핑으로 빼냅니다 (`LOST`)
- 명령과 측정이 20° 넘게 벌어지면 **모든 모터를 멈춥니다** (`TRACKING`, 비상정지 걸림). 방향 부호가 틀렸거나 엔코더가 이상할 때 폭주 대신 여기서 멈춥니다
- 관절에 `estop: range`를 주면 그 관절은 위 추종 오차 검사를 건너뛰고, 측정 각도가 `lower_deg`~`upper_deg`를 `range_margin_deg`(기본 5°) 넘게 벗어날 때 **모든 모터를 멈춥니다** (`RANGE`). 떨림 때문에 추종 오차가 잠깐 커지는 관절용입니다
- 위치 응답은 기능 코드(TRANSMIT_PDO_2)로 골라 받습니다. 원본은 같은 모터의 아무 프레임이나 받아서, 늦게 온 SDO 응답을 위치로 읽을 수 있었습니다
- 0.2초마다 하트비트를 보냅니다(펌웨어 watchdog 1초)

## 테스트

```bash
cd ~/humanoid_sdk/ros2_ws/src/bhl_arm_sdk
python3 -m pytest -q test          # ROS, CAN 없이: 가짜 모터 버스로 제어 코어 + CAN 프레임 형식
```

## 현재 상태

- 코어 테스트와 연습 모드(ROS 2 노드, 파라미터, 토픽, 서비스, 시퀀스) 동작은 확인했습니다
- **실물 모터로는 아직 실행하지 않았습니다.** 처음에는 관절 하나만 `enabled: true`로 두고, 작은 각도(±3°)부터 방향을 확인하세요
