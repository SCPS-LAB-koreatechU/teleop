# teleop

Berkeley Humanoid Lite 양팔(recoil CAN 모터 10개)을 **제어 PC에서** 움직이고 상태를 보는 코드 모음입니다.
GUI, 슬라이더, RViz 드래그 텔레옵과 모터 점검·캘리브레이션 도구가 들어 있습니다.

| 도구 | 위치 | 하는 일 | 모터 연결 |
|---|---|---|---|
| **ROS 슬라이더 제어** | `ros2_ws/src/bhl_arm_sdk`, `ros2_ws/src/bhl_bringup` | 관절마다 슬라이더 하나, 각도(°)로 목표 지정. RViz 표시 | 로봇 컴퓨터(RDK X5)가 CAN을 잡고, 제어 PC는 ROS 2로 명령 |
| **3D 드래그 텔레옵** | `ros2_ws/src/berkeley_humanoid_lite_moveit_config` (`drag_teleop.launch.py`) | RViz에서 손 마커를 끌면 IK로 팔이 따라옴. 팔 패널(비상정지, 관절 한계), 손가락 패널 | 제어 PC에 CAN 어댑터 직접 연결 |
| **모터 GUI 도구** | `tools/motor_gui` | 10축 2D GUI, bring-up 점검, 전기각 재캘리브레이션, CAN 진단 | 제어 PC에 CAN 어댑터 직접 연결 |

`source/berkeley_humanoid_lite_lowlevel`은 recoil CAN 드라이버입니다
([HybridRobotics/Berkeley-Humanoid-Lite-Lowlevel](https://github.com/HybridRobotics/Berkeley-Humanoid-Lite-Lowlevel)에 연구실 수정 포함).
`berkeley_humanoid_lite_description`, `berkeley_humanoid_lite_moveit_config`는
[Berkeley-Humanoid-Lite](https://github.com/HybridRobotics/Berkeley-Humanoid-Lite)의 `ros2_ws` 패키지를 고친 것입니다.

## 폴더 구조

```
teleop/
├── env.sh                  # ROS 2 + 이 워크스페이스 + ROS_DOMAIN_ID + DDS 유선 고정
├── ros2_ws/
│   ├── fetch_vendor.sh     # 메시 내려받기 (BHL assets, DexHand)
│   ├── run_drag_teleop.sh  # 3D 드래그 텔레옵 실행
│   ├── run_demo.sh         # MoveIt 데모 실행
│   └── src/
│       ├── bhl_arm_sdk/                         # 모터별 각도 제어 노드 + 슬라이더 GUI
│       ├── bhl_bringup/                         # robot / remote / demo launch
│       ├── berkeley_humanoid_lite_description/  # URDF
│       └── berkeley_humanoid_lite_moveit_config/# 드래그 텔레옵, 하드웨어 브리지, MoveIt 설정
├── source/
│   └── berkeley_humanoid_lite_lowlevel/         # recoil CAN 드라이버
└── tools/
    └── motor_gui/                               # 2D 모터 GUI, bringup.py, recalibrate.py
```

BHL 저장소와 같은 상대 경로를 유지해서, 코드 안의 경로를 고치지 않고 그대로 동작합니다.

## 설치 (Ubuntu 22.04, ROS 2 Humble)

```bash
git clone https://github.com/SCPS-LAB-koreatechU/teleop.git ~/teleop
cd ~/teleop/ros2_ws
./fetch_vendor.sh                      # 메시: source/berkeley_humanoid_lite_assets, ros2_ws/vendor/ (약 130 MB)
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
pip install python-can pyyaml          # CAN 드라이버, 설정 파일
sudo apt install python3-pyqt5 python3-tk
```

이후 터미널마다:

```bash
source ~/teleop/env.sh                 # 기본 ROS_DOMAIN_ID=92, 바꾸려면 BHL_ROS_DOMAIN_ID=94 source ~/teleop/env.sh
```

## 1. ROS 슬라이더 제어 (로봇 컴퓨터 + 제어 PC)

로봇 컴퓨터(RDK X5)가 CAN으로 모터를 잡고 `/arm_controller`를 띄웁니다. 제어 PC는 같은 `ROS_DOMAIN_ID`로 RViz와 슬라이더만 띄웁니다.
로봇 컴퓨터 쪽은 [humanoid_sdk](https://github.com/SCPS-LAB-koreatechU/humanoid_sdk)에 같은 `bhl_arm_sdk`/`bhl_bringup`이 있습니다.

```bash
# 로봇 컴퓨터 (RDK X5)
BHL_ROS_DOMAIN_ID=94 source ~/humanoid_sdk/env.sh
ros2 launch bhl_bringup robot.launch.py dry_run:=false config:=$HOME/.config/bhl_arm_sdk/bhl_arms_x5.yaml

# 제어 PC
BHL_ROS_DOMAIN_ID=94 source ~/teleop/env.sh
ros2 launch bhl_bringup remote.launch.py        # RViz + 슬라이더 GUI
```

하드웨어 없이 한 PC에서 연습:

```bash
ros2 launch bhl_bringup demo.launch.py          # 연습 모드(dry run): 측정값 = 명령값
```

슬라이더 GUI 사용 순서:

1. **팔을 0° 자세(차렷)로 둔 뒤 Enable.** 설정의 `zero: start`는 Enable 순간의 자세를 모든 관절 0°로 잡습니다. 관절 한계(`lower_deg`/`upper_deg`)가 이 0° 기준이라, 다른 자세에서 켜면 한계가 어긋납니다.
2. 관절을 하나씩 작게(±5°) 움직여 방향을 확인합니다. 반대로 가면 설정의 `direction`을 바꿉니다.
3. 끝낼 때는 **Home → Disable**. 이상하면 **E-STOP**.

명령줄로도 제어할 수 있습니다:

```bash
ros2 service call /arm_controller/enable std_srvs/srv/SetBool "{data: true}"
ros2 param set /arm_controller target.arm_right_elbow_pitch_joint -5.0     # 도(°)
ros2 service call /arm_controller/home std_srvs/srv/Trigger
ros2 service call /arm_controller/enable std_srvs/srv/SetBool "{data: false}"   # damping
```

### 안전 장치 (`bhl_arm_sdk`)

| 조건 | 결과 |
|---|---|
| Enable 때 ping 무응답 | 그 관절은 켜지 않음 (`NO_REPLY`) |
| Enable 때 측정 각도가 한계 밖 | 켜지 않음 (`OUT_OF_RANGE`) |
| 25 주기 연속 무응답 | 그 관절만 damping (`LOST`) |
| 명령과 측정이 20° 넘게 차이 (기본) | **전체 정지** (`TRACKING`) |
| 관절에 `estop: range`: 측정이 한계 ± `range_margin_deg`(기본 5°)를 벗어남 | **전체 정지** (`RANGE`) |

`estop: range`는 떨림 때문에 추종 오차가 잠깐 커지는 관절용입니다(현재 8번 오른쪽 팔꿈치 pitch).
이 모드에서는 엔코더가 고장 나 엉뚱한 값이 **범위 안**에서 멈추면 잡지 못하니, 모터 error 플래그도 함께 확인하세요.

## 2. 3D 드래그 텔레옵 (제어 PC에 CAN 직접 연결)

```bash
cd ~/teleop/ros2_ws
./run_drag_teleop.sh                 # dry run: CAN으로 아무것도 안 보냄
./run_drag_teleop.sh live:=true      # 실물 팔
./run_drag_teleop.sh model:=plain    # 손 없는 팔만
```

- RViz에서 손 마커를 끌면 IK가 관절 각을 풀고, `hardware_bridge.py`가 속도 제한·URDF/운영자 한계·비상정지를 거쳐 CAN으로 보냅니다.
- 팔 패널(`arm_panel.py`): 링크 상태, E-STOP, 관절별 한계. 손가락 패널(`finger_panel.py`): DexHand 손가락 슬라이더.
- 관절 매핑(CAN ID, 방향, 기어비, 오프셋, 사용 여부): `ros2_ws/src/berkeley_humanoid_lite_moveit_config/config/hardware_joints_left_distal.yaml`
- `ROS_DOMAIN_ID` 기본 77. DexHand MoveIt 세션(도메인 0)과 섞이지 않게 0은 거부합니다.
- 엔코더 기준이 바뀐 뒤(재캘리 등)에는 `new_reference:=true`로 시작합니다(기본값).

## 3. 모터 GUI 도구 (`tools/motor_gui`)

자세한 내용은 [`tools/motor_gui/README.md`](tools/motor_gui/README.md).

```bash
cd ~/teleop/tools/motor_gui
./run.sh --sim --rviz                # 하드웨어 없이 GUI + RViz
./run.sh --rviz                      # 실물: can0(왼팔), can1(오른팔)
./verify_sim.sh --no-ros             # 시뮬 검증 (제어·안전 항목)
```

| 스크립트 | 용도 |
|---|---|
| `bhl_motor_gui.py` | 10축 2D GUI: 선택·구동·모니터링, 오프셋 저장 |
| `bringup.py --id N` | 모터 하나 7단계 점검: 어댑터 → 버스 → ping → 상태 → 엔코더(손 회전) → `--drive` 작은 왕복 구동 |
| `recalibrate.py --id N` | 전기각 재캘리브레이션(phase order + flux offset). `--compare-po`, `--po -1`, 전원 재투입 후 `--verify` |
| `bus_probe.py`, `boot_watch.py`, `can_doctor.sh` | CAN 버스·어댑터 진단 |

재캘리브레이션 주의:

- 실행 중 로터가 한 바퀴 돌았다 돌아옵니다(관절 약 24°). **도중에 멈출 수 없습니다.**
- 실패하면 펌웨어가 `CALIBRATION_ERROR (0x8)`를 냅니다. 도구 문구는 "목표 전류 미달"이지만, 대부분은 **엔코더가 로터를 반 바퀴도 못 따라온 경우**입니다(자석 정렬·착자 방향, 엔코더 케이블). 캘리 전에 엔코더가 한 바퀴를 제대로 읽는지 먼저 확인하세요.
- 같은 모터를 두 프로그램이 동시에 읽으면 SDO 응답이 한 칸씩 밀립니다. **모터 하나에 프로그램 하나만** 붙이세요.

## 라이선스

MIT ([LICENSE](LICENSE)). 원본 Berkeley Humanoid Lite 코드의 저작권 표시를 유지합니다.
메시는 각자의 라이선스를 따릅니다: BHL assets CC BY-SA 4.0, DexHand(및 `meshes_dexhand_left`) CC BY-NC-SA 4.0.
