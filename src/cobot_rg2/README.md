# cobot_rg2 — M0609 + RG2 + RealSense D435i

Doosan M0609 + OnRobot RG2 + RealSense D435i(eye-to-hand) 통합 패키지 묶음.

> **실행 절차·기능확인·트러블슈팅은 워크스페이스 루트 `README.md`에 있다.**
> 이 문서는 **패키지 구성과 설치**만 다룬다. 두 곳에 같은 걸 적으면 한쪽이 먼저 썩는다.

- 최종 갱신: 2026-08-04

---

## 패키지 지도

```
src/cobot_rg2/
├── doosan-robot2/     외부 — read-only. dsr_bringup2 / dsr_controller2 / dsr_hardware2 / dsr_description2 ...
├── onrobot-ros2/      외부 — read-only. onrobot_rg_control (Modbus TCP), onrobot_rg_msgs
└── rg2/               ← 이 ws에서 직접 만든 것
    ├── m0609_rg2_bringup   로봇+그리퍼 bringup, 카메라 드라이버, 캘리브 TF, URDF/xacro
    └── m0609_rg2_moveit    move_group 설정 (SRDF, OMPL, JTC 컨트롤러, octomap)
```

`rg2/` 두 개만 이 워크스페이스가 유지보수한다. 나머지는 업스트림이므로 수정하지 않는다.

### m0609_rg2_bringup

| 경로 | 내용 |
|---|---|
| `launch/bringup.launch.py` | M0609 + RG2, ros2_control, TF(`world→base_link`), 관측용 RViz |
| `launch/camera.launch.py` | D435i 드라이버 + `base_link→camera_link` static TF (npy에서 매 실행 계산) |
| `launch/bringup_camera.launch.py` | eye-**in**-hand(그리퍼 부착) 변형. 현재 구성(eye-to-hand)에서는 쓰지 않는다 |
| `config/T_cam2base.npy` | 캘리브 결과. `corecode/Calibration_Tutorial/`에서 **수동 `cp`** 로 동기화 |
| `scripts/calib_npy_to_tf.py` | npy → static TF 인자 변환 (OpenCV optical → REP-103 body 규약 보정 포함) |
| `scripts/gripper_virtual_node.py` | virtual 모드 그리퍼 RViz 애니메이션 (Modbus 미포함) |
| `urdf/m0609_with_rg2.urdf.xacro` | 기본 모델. `moveit`도 이 파일을 경로로 직접 읽는다 |

### m0609_rg2_moveit

| 경로 | 내용 |
|---|---|
| `launch/moveit.launch.py` | move_group + `dsr_moveit_controller`(JTC) + MotionPlanning RViz |
| `config/m0609_rg2.srdf` | 플래닝 그룹, `all-zeros` / `gripper_open` / `gripper_close` |
| `config/moveit_controllers.yaml` | 컨트롤러 이름이 `/dsr01/...` — 네임스페이스와 **짝**이다 |
| `config/sensors_3d.yaml` | RealSense 포인트클라우드 → octomap (3D 장애물). `[튜닝]` 주석 참고 |
| `config/ompl_planning.yaml` | RRTConnect 등 |

---

## 설치

### 1. 의존성

apt/pip 목록은 워크스페이스 루트 `requirements.txt` 한 곳에 모아뒀다 (apt 블록이 주석으로 들어 있다).

```bash
# apt: requirements.txt 상단 주석의 apt 블록을 그대로 실행
pip3 install -r requirements.txt   # pymodbus만
rosdep install -r --from-paths src --ignore-src --rosdistro humble -y
```

> `onrobot_rg_control`의 `message_runtime` 키는 ROS1 잔재라 rosdep 경고가 뜬다. `-r`로 무시된다.
> ⚠️ 이 랩탑은 계정 공유다. `sudo apt`로 ROS 패키지를 **제거·다운그레이드하지 말 것.** 추가 설치만.

### 2. 빌드

```bash
cd ~/cobot2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select m0609_rg2_bringup m0609_rg2_moveit
```

### 3. 최초 1회 설정

**실기 UDP 포트 권한** (없으면 real 모드 연결 실패):
```bash
echo 'net.ipv4.ip_unprivileged_port_start=0' | sudo tee /etc/sysctl.d/99-ros2-doosan.conf
sudo sysctl --system
```

**RealSense udev rules** (없으면 스트리밍 중 `xioctl(VIDIOC_QBUF) failed — No such device`):
```bash
sudo curl -L https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules \
  -o /etc/udev/rules.d/99-realsense-libusb.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```
적용 후 USB 재연결 필요.

**DRCF 에뮬레이터** (`mode:=virtual`에서 `movej` 등 motion service를 쓸 때만):
```bash
sudo usermod -aG docker $USER && newgrp docker
cd src/cobot_rg2/doosan-robot2 && chmod +x install_emulator.sh && sudo ./install_emulator.sh
```

---

## 하드웨어

| 항목 | 값 |
|---|---|
| 로봇 | M0609, 네임스페이스 `dsr01`, IP `192.168.1.100`, port `12345` |
| 그리퍼 | OnRobot RG2, Modbus TCP `192.168.1.1:502` (컴퓨트박스 고정 IP) |
| 카메라 | RealSense D435i, USB. **eye-to-hand** — 로봇에 붙어있지 않다 |

카메라를 옮기면 `T_cam2base.npy`가 전부 무효다 → 루트 README「재캘리브」.

---

## Virtual / Real 그리퍼 차이

| 항목 | real | virtual |
|---|---|---|
| 제어 | OnRobot 드라이버 (Modbus TCP) | 없음 (`gripper_virtual_node` 시각화만) |
| 완료 신호 | 디지털 입력 핀 | `/onrobot/sendCommand` 응답 |
| 파지력·접촉 | 실제 | 시뮬레이션 없음 |
| Tool/TCP 프리셋 | DRCF 등록값 | 스킵 (에뮬레이터 미등록) |

그리퍼는 **MoveIt 컨트롤러가 없다.** `/onrobot/sendCommand` 서비스로 직접 제어한다.

---

## TF 구조 (eye-to-hand, `bringup` + `camera`)

```
world
└── base_link ──────────────────────────── camera_link      (static TF, T_cam2base.npy)
    └── link1 → … → link6 → tool0              └── camera_color_frame / _optical_frame
                             └── rg2_base_link      camera_depth_frame  / _optical_frame
                                 ├── rg2_left_outer_knuckle → inner_knuckle / inner_finger
                                 └── rg2_right_outer_knuckle → inner_knuckle / inner_finger
```

- `world → base_link`: `static_transform_publisher` (identity)
- `base_link → camera_link`: `camera.launch.py`가 npy에서 계산. **하드코딩 금지**
- `tool0 → rg2_base_link`: `joint0` (fixed)
- `rg2_left/right_inner_knuckle`: mimic, `rg2_finger_joint` 기준

> ⚠️ **플래닝 프레임은 `world`가 아니라 `base_link`다.** 장애물 `header.frame_id`도 `base_link`.
> 이유와 증상은 루트 README 8절.

---

## 다음

| 하고 싶은 것 | 문서 |
|---|---|
| 켜고 동작 확인 (로봇/카메라/MoveIt 3터미널) | 루트 `README.md` 3~4절 |
| 회피 경로 테스트 (장애물 놓고 Plan) | 루트 `README.md` 8절 |
| 재캘리브 | 루트 `README.md`「재캘리브」 |
| 실기로 알아낸 제약 | `md/context/constraints.md` |
