<!-- meta
updated: 2026-08-06 12:40
status:  live
owns:    실기·현장에서 확인된 사실 (하드웨어, TF, MoveIt, QoS, 리소스)
-->

# 실기/현장 제약 (실측 사실만 기록. 추측 금지)

> 📁 문서 지도: [[ws/cobot2/README]] · 현재 상태: [[ws/cobot2/state]] · 오류 이력: [[ws/cobot2/errors-log]]
> **이 문서가 소유하는 것: "무엇이 참인가".** 지금 뭘 할지는 `state.md`, 틀린 이력은 `errors-log.md`.
> 튜닝값은 **파일이 정본이다**(`sensors_3d.yaml` 등) — 여기 표는 변경 이력이지 현재값 조회용이 아니다.

## 카메라 마운트 강성 (2026-08-01, `archive/2026-08-01-session.md` §3에서 이관)
- 캘리브 정확도는 마운트 강성에 통째로 종속된다. 헐거우면 mm 단위 정확도는 못 얻는다. 다만:
  - Octomap·OMPL·상태머신·인식 로직 **개발**에는 대충 맞는 TF로 충분 — 영향 없음.
  - **인식 정확도 실측**은 의미 없다. 마운트가 확정될 때까지 미룰 것 — 헐거운 마운트에서 오차 재고 코드 부호를 만지는 건 시간 낭비.
- 그래서 마운트 미확정 상태에서는 "잠정 캘리브"로 찍고 넘어간다. 대신:
  1. 마운트 위치를 마스킹테이프로 표시 + 사진 (나중에 재현·비교 가능하게)
  2. bag 녹화와 캘리브는 같은 세션에서, 카메라를 안 건드리고 연속으로 한다 — 둘 사이에 움직이면 짝이 안 맞는다

## doosan-robot2 / 토픽 (2026-08-01 실측, `state.md`에서 이관)
- **doosan-robot2 launch의 `model` 기본값이 `m1013`** — M0609 쓸 때마다 `model:=m0609` 명시 필요. `dsr_bringup2_{rviz,gazebo,mujoco,moveit}.launch.py` 모두 해당.
- 시뮬 경로 3종 존재: virtual 모드(DRCF 에뮬레이터, `install_emulator.sh` 선행 필요), Gazebo(`dsr_gazebo2`), MuJoCo(`dsr_mujoco`).
- **D435i 토픽 네임스페이스는 `/camera/camera/...`** (2026-08-01 `ros2 topic list` 실측). 계획서 초안의 `/d435i/...`는 오기다. 토픽 이름은 **런치 명령이 정하지 마운트 방식(eye-in-hand/eye-to-hand)이 정하지 않는다** — 마운트를 바꿔도 이름은 그대로고 TF 부모 프레임만 바뀐다.
- `align_depth.enable:=true`일 때 `aligned_depth_to_color`의 해상도는 **depth가 아니라 color 프로파일을 따른다.** 대역폭 계산 시 주의.

## RealSense D435I (2026-08-01)
- ⚠️ **`reals` alias가 2026-08-03 기준 `ros2 launch m0609_rg2_bringup camera.launch.py`로 바뀌었다**(사용자 확인). 아래 08-01 기록의 `rs_align_depth_launch.py` 설명은 **낡았다** — 이 낡은 줄을 근거로 공식 런치를 써서 `base_link→camera_link` 없는 bag 4.8GB를 찍은 사고가 있었다([[ws/cobot2/rosbag-d435i]] §4). 두 런치는 **다른 것을 준다**: ws 런치만 `camera_calib_tf`(캘리브 TF)와 IMU를 준다.
- (2026-08-01, 구 alias 기준) `rs_align_depth_launch.py` + `align_depth.enable:=true enable_rgbd:=true pointcloud.enable:=true`, depth 848x480x30 / color 1280x720x30 조합은 **ROS_DOMAIN_ID를 지정하지 않으므로 기본값(비어있음 → 0)에서 뜬다.** 현 alias도 도메인을 지정하지 않는 건 동일.
- `.bashrc`의 `rdm` alias는 `ROS_DOMAIN_ID=93`을 설정한다. 카메라(도메인 0)와 rqt/다른 터미널(도메인 93)이 섞이면 rqt에 토픽이 전혀 안 보인다 — 실측으로 확인(도메인 93에서 `ros2 topic list`는 `/parameter_events`, `/rosout`만 나옴).
- CPU는 병목이 아님: `align_depth+enable_rgbd+pointcloud` 동시 실행 중에도 `load average 0.5~0.6`, realsense 노드 CPU 18.8%. 그런데도 `/camera/camera/color/image_raw`, `/rgbd`, `/depth/color/points`의 `ros2 topic hz`가 최소 간격(~0.031~0.033s, 30fps대)은 정상이면서 가끔 0.2~0.3s까지 벌어져 누적 평균이 계속 떨어짐 — USB/드라이버 쪽 간헐적 프레임 드롭으로 추정(미검증, CPU 원인은 배제됨).

## 카메라·로봇 하드웨어
- 로봇 M0609(네임스페이스 `/dsr01`, IP `192.168.1.100`) + OnRobot RG2. **2026-08-02 실기 확인** — `bringup.launch.py mode:=real`로 연결 후 MoveIt Plan/Execute 성공.

## MoveIt 실기 검증 (2026-08-02)
- **실기에서 MoveIt Plan → Execute 동작 확인됨.** 궤적은 `/dsr01/dsr_moveit_controller/follow_joint_trajectory`로 나간다.
- **`dsr_moveit_controller`는 아무도 자동으로 spawn하지 않는다.** `dsr_controller2.yaml`에 파라미터만 있고 bringup은 `joint_state_broadcaster` + `dsr_controller2`만 띄운다. `moveit.launch.py`가 `standalone:=false`일 때 spawn하도록 추가했다.
- **`dsr_controller2`와 JTC는 공존한다.** `dsr_controller2`는 command/state 인터페이스를 하나도 claim하지 않는 순수 서비스 래퍼다(`dsr_controller2.cpp:89-101`). 업스트림 a0509 예제가 `dsr_controller2` spawner를 주석 처리해둔 것과 달리 둘 다 살려도 된다.
- **MoveIt 컨트롤러 이름에 네임스페이스를 절대경로로 박아야 한다.** MoveItSimpleControllerManager는 액션 이름을 `<controller_name>/<action_ns>`로 조립한다(`action_based_controller_handle.h:204`). controller_manager가 `dsr01` ns라 `moveit_controllers.yaml`에 `/dsr01/dsr_moveit_controller`로 적어야 한다. **어긋나면 Plan은 되고 Execute만 ABORTED** — 에러 메시지가 원인을 안 가리킨다.
- RG2 그리퍼는 MoveIt이 제어하지 않는다. `rg2_gripper_controller`라는 ros2_control 컨트롤러는 **존재하지 않는다**(`/onrobot/sendCommand` 서비스로 제어). moveit_controllers.yaml에서 제거했다.
- `dsr_moveit_config_m0609`는 쓰지 않는다: ① `MoveItConfigsBuilder("m0609")`가 `m0609_moveit_config` 패키지를 찾는 업스트림 버그 ② 그 config의 URDF엔 RG2가 없다.

## Planning Scene / octomap (2026-08-02)
- **`publish_geometry_updates` 등 planning_scene_monitor 발행 파라미터는 기본값으로 두면 안 된다.** 안 켜면 RViz Scene Objects에 놓은 장애물이 **화면에는 보이는데 계획에는 안 먹는다.** `moveit.launch.py`에서 4개 모두 True로 설정.
- `moveit_ros_perception`은 **기본 설치되어 있지 않다**(Humble). 없으면 `PointCloudOctomapUpdater` 클래스가 없어 3D 장애물 감지만 조용히 죽는다 — 계획·실행은 정상이라 놓치기 쉽다.
- octomap 파라미터 이름은 `libmoveit_ros_occupancy_map_monitor.so` 심볼로 확인: `sensors`, `octomap_frame`, `octomap_resolution`, `<sensor_name>.sensor_plugin`.
- **플래닝 프레임은 `world`가 아니라 `base_link`다 (2026-08-02 실측).** SRDF에 `virtual_joint(FixedBase, parent_frame="world")`가 있어서 world일 것 같지만 아니다 — MoveIt은 **fixed 타입 virtual joint로는 모델 프레임을 만들지 않아** 루트 링크가 그대로 플래닝 프레임이 된다. `world`는 TF에는 있지만 **planning scene은 모른다.**
  - `frame_id='world'`로 CollisionObject 발행 → `[ERROR] moveit_planning_scene: Unknown frame: world`, **장애물이 조용히 무시된다**
  - `frame_id='base_link'` → `/monitored_planning_scene`에 정상 등록 (echo로 확인)
  - 따라서 `octomap_frame`도 `base_link`. RViz Scene Objects의 프레임도 마찬가지다.
- `octomap_frame`은 고정 프레임이어야 한다. 움직이는 프레임(`camera_link` 등)을 주면 로봇이 움직일 때마다 지도가 통째로 흔들린다.

## 핸드아이 캘리브레이션 (2026-08-02)

### 체커보드 (사용자 확인)
- **내부 코너 10x7 (= 칸 11x8), 한 칸 24mm** → 코드의 `checkerboard_size = (10, 7)`, `square_size = 24.0`.
  `corecode/Calibration_Tutorial/{handeye,eye2hand}_calibration.py` 양쪽에 반영 완료.
- `cv2.findChessboardCorners`가 요구하는 건 **칸이 아니라 내부 코너 개수**다. 헷갈리면 코너를 한 장도 못 찾고 "체커보드 코너를 충분히 찾지 못하였습니다"로 멈춘다 — 조용히 틀리지 않고 요란하게 실패하는 종류라 바로 안다.
- `find_checkerboard_pose()`의 `objp`가 `square_size` 대신 `25`로 하드코딩돼 있던 버그는 **2026-08-02 수정함.** 수정 전이었다면 24mm 보드로 잰 거리가 25/24 = 4.2% 부풀어 나왔다(500mm → 521mm). 합성 보드(11x8칸/24mm) 렌더링 테스트로 확인.
- **인쇄물의 실제 칸 크기를 캘리퍼스로 재서 `square_size`에 넣을 것.** 프린터가 배율을 틀리게 뽑으면 그 비율이 그대로 거리 오차가 된다.

### 마운트 (사용자 확정)
- **eye-in-hand 카메라(C270)는 플랜지↔그리퍼 커넥터부에 부착한다.** 카메라가 그리퍼보다 로봇 쪽에 있으므로 결과 오프셋의 Z가 음수로 나오는 게 정상이다.
- **eye-to-hand 체커보드는 그리퍼에 부착한다. 부착 위치의 정밀도는 필요 없다** — AX=XB에서 보드↔그리퍼 변환은 소거된다. 대신 아래 두 가지는 타협 불가:
  - **강성**: 수집 20분 내내 팔에 대해 움직이지 않아야 한다. 밀려도 알 방법이 없고 결과만 조용히 틀어진다.
  - **평면도**: 알고리즘이 모든 코너를 완전한 평면으로 가정한다. 종이를 테이프로 붙이면 휜다 → 알루미늄/아크릴/두꺼운 MDF에 전면 접착.
- 보드는 그리퍼 손가락에 가리지 않도록 옆이나 앞으로 빼서 붙인다.
- 오차가 1cm를 넘으면 코드보다 **강성·평면도·실측 칸 크기**를 먼저 의심한다.

### 수집 시 주의
- **자세마다 회전을 충분히(30° 이상, 여러 축으로) 섞을 것.** 평행이동만 하면 `logR()`이 0으로 나눠 NaN이 되어 eye-to-hand 결과가 통째로 죽는다 — 합성 데이터로 재현 확인.
- `data_recording.py`는 `set_tcp` 적용된 `posx`를 기록한다. **`set_tcp`를 0으로 두고 수집하면 결과의 부모 프레임이 flange가 되어 CAD/줄자로 검산할 수 있다.** TCP를 걸어둔 채 수집하면 검산 기준이 없어진다.
- **티치펜던트 실제 등록명은 `Tool Weight` / `GripperDA_v1`이다 (2026-08-03 실기 확인).**
  ~~"Tool Weight_2FG", "2FG_TCP"~~는 낡은 값이라 폐기. 확인 명령(list API는 없다 — 현재 선택된 것 하나만 읽힌다):
  ```bash
  ros2 service call /dsr01/tool/get_current_tool dsr_msgs2/srv/GetCurrentTool
  ros2 service call /dsr01/tcp/get_current_tcp  dsr_msgs2/srv/GetCurrentTcp
  ```
  `dsr_msgs2/srv/{tool,tcp}/`에 `GetToolList` 같은 건 없다. 등록 목록 전체는 펜던트 화면에서만 본다.
- ⛔ **미해결 (2026-08-03): 이름이 맞는데도 `set_tool('Tool Weight')`가 `-1`을 돌려준다.**
  `data_recording.py:75-78`이 여기서 `SystemExit`으로 끊긴다. 코드 경로상 서비스는 **응답했고 `success=False`**였다
  (없으면 `spin_until_future_complete`가 무한 대기, 예외면 `UnboundLocalError`였을 것 — 둘 다 아니었다).
  미배제 가설: ① 로봇 상태가 STANDBY 아님(펜던트 수동모드/서보오프) ② 이미 같은 값이라 컨트롤러가 False 반환
  ③ bringup이 virtual 모드. 배제 순서 ①→③→②, ①은 `get_robot_state`/`get_robot_mode` 한 줄이면 갈린다.
  ⚠️ `data_recording.py:76`의 에러 문구 "티치펜던트 등록명을 확인할 것"은 **오진을 유도한다** — 이름은 맞았다.

### 알고리즘 검증 (합성 데이터, 2026-08-02)
- `handeye_calibration.py`(eye-in-hand, `cv2.calibrateHandEye` PARK): 정답 복원, max|err| 1.1e-13.
- `eye2hand_calibration.py`(eye-to-hand, Park-Martin 자체 구현): 정답 복원, max|err| 2.0e-13.
- 변수명이 의미와 반대인 곳이 많지만(`T_gripper2base = T_base2gripper` 등) **넘기는 값 자체는 옳다.** 이름을 믿고 고치지 말 것.
- 회전 규약 ZYZ = 두산 `posx` 규약과 일치.
- **단위는 전 구간 mm.** ROS/FoundationPose는 m다. 변환은 `src/cobot_rg2/rg2/m0609_rg2_bringup/scripts/calib_npy_to_tf.py` 한 곳에서만 한다.
- 내부 파라미터를 `camera_info`(D435i 공장값) 대신 `calibrateCamera`로 재추정한다 — 오차가 크면 여기도 의심 대상.

## TF 프레임 이름 — `base_0`이 아니라 `base_link` (2026-08-02, 실측)

`m0609_rg2_bringup`은 URDF를 **두 개** 쓴다. 이름이 다른 두 체계가 공존하는 이유다.

| URDF | 쓰는 곳 | 프레임 이름 | TF에 발행? |
|---|---|---|---|
| `dsr_description2/xacro/m0609.urdf.xacro` | `ros2_control_node` 하드웨어 인터페이스 | `base_0`, `link6` | ❌ **안 한다** |
| `m0609_rg2_bringup/urdf/m0609_with_rg2.urdf.xacro` | `robot_state_publisher` | `base_link`, `link_1`..`link_6`, `tool0`, `rg2_*` | ✅ |

→ **`base_0`는 TF 트리에 절대 나타나지 않는다.** 두산 예제/문서를 그대로 따라 쓰면 전부
`Invalid frame ID "base_0" ... frame does not exist`로 죽는다. `base_link`를 쓸 것.
(`bringup.launch.py:76-87` = ros2_control용, `:176-186` = RSP용. 실측 `ros2 topic echo /tf`로 확인)

실제 TF 트리:
```
world → base_link → link_1 … link_6 → tool0 → rg2_base_link → rg2_*
camera_link → camera_depth_frame/camera_color_frame → *_optical_frame   ← 캘리브 TF 없으면 별개의 섬
```

## 카메라 TF 연결 (2026-08-02, 실측 확인)

`base_link → camera_link`는 URDF에 없다(eye-to-hand라 카메라가 로봇에 안 붙어 있다).
**`camera.launch.py`가 `config/T_cam2base.npy`를 읽어 자동 발행한다 (2026-08-02).**
launch는 단위로 나눠져 있다 — `bringup`(로봇만) / `camera`(RealSense+TF) / MoveIt `demo`.
`bringup.launch.py`는 카메라를 모른다. 카메라가 필요하면 `camera.launch.py`를 따로 띄운다.
숫자를 launch나 alias에 하드코딩하지 않는다 — **재캘리브하면 자동 반영된다. `cp`는 이제 필요 없다**(2026-08-03):
```
install/.../config/T_cam2base.npy → src/.../config/T_cam2base.npy → corecode/Calibration_Tutorial/T_cam2base.npy
        (colcon --symlink-install)          (git에 커밋된 상대경로 symlink)
```
~~`cp corecode/... src/.../config/`~~ 는 이제 `cp: are the same file`로 **실패**한다. 쓰지 말 것.
npy가 없으면 이 TF만 빠지고 bringup은 정상 진행한다. 값만 확인하려면:
```bash
ros2 run m0609_rg2_bringup calib_npy_to_tf.py corecode/Calibration_Tutorial/T_cam2base.npy base_link camera_link
```
**하드코딩된 static_transform_publisher 명령을 다시 만들지 말 것** — npy가 갱신돼도 그 숫자는 안 따라온다.
실제로 2026-08-02에 붙여쓰던 명령이 갱신 전 캘리브 값이라 340 mm 어긋나 있었다.
- ~~평행이동 993.4 mm~~ → 폐기(좌표 규약 버그 수정 전 값).
- ~~약 1.48 m `[1.148, 0.640, 0.678]` (2026-08-02)~~ → **폐기(2026-08-03 재캘리브).**
- **현행 카메라 위치를 문서에 적지 않는다.** 하루에 세 번 바뀌었다(2026-08-03: 1.48 → 1.542
  → 1.684 m). 적는 순간 낡는다. 알아야 할 땐 읽는다:
  ```bash
  python3 -c "import numpy as np; T=np.load('corecode/Calibration_Tutorial/T_cam2base.npy'); \
  print(T[:3,3], np.linalg.norm(T[:3,3])/1000, 'm')"
  ```
  **거리에 의존하는 임계값을 다른 파일에 하드코딩하지 말 것** — `sensors_3d.yaml`의
  `max_range: 1.5`가 낡은 1.48 m를 근거로 잡혔고, 그 사이 실제 거리가 그걸 넘어섰다.
- 발행 자체는 검증됨: 회전행렬 직교성 검사 통과, `/tf_static` 값이 `calib_npy_to_tf.py`
  출력과 일치, `tf2_echo base_link camera_depth_optical_frame` 체인 연결(2026-08-03).
- **캘리브 이미지셋으로 추정한 내부파라미터는 못 쓴다** (2026-08-03). 카메라가 고정이라 보드
  거리 다양성이 부족해 `cv2.calibrateCamera` 추정 fx가 공장값보다 7.7% 낮게 나왔다
  (839.78 vs 909.53). 거리 추정이 통째로 그만큼 틀어진다. → D435i **개체 고유** 공장값을
  쓴다. 값과 재취득 명령은 `eye2hand_calibration.py`의 `FACTORY_INTRINSICS`에 있다
  (여기 베껴 적지 않는다 — 카메라를 교체하면 한 곳만 고쳐야 하니까).
- **`eye2hand_calibration.py`를 진단 목적으로 돌릴 땐 `--no-save`를 붙인다** (2026-08-03).
  이 스크립트는 실행할 때마다 실기 TF의 소스인 `T_cam2base.npy`를 덮어쓴다. 리뷰용으로
  "읽기만" 하려고 돌렸다가 실제로 덮어썼다(계산이 결정적이라 값은 같았지만 운이 좋았을 뿐이다).
  **읽기 의도의 실행에 쓰기 부작용이 있는 게 원인**이라 플래그로 끌 수 있게 만들었다.
  과거 수집분을 재계산할 땐 디렉토리를 인자로 준다(`... data1` → `T_cam2base_data1.npy`로 분리 저장).
- **캘리브 수집은 1280x720으로 띄운 뒤 한다** — `data_recording.py`는 해상도를 지정하지 않고
  구독만 하므로, `camera.launch.py` 기본값(424x240)으로 띄운 채 찍으면 코너 정밀도가 무너진다.
- **아직 미검증**: `T_cam2base`의 방향(parent가 base_link가 맞는지). `eye2hand_calibration.py:305`가 AX=XB의 X를 그대로 저장하는데, 코드 명명 관행(`gripper2base` = base 좌표계의 gripper pose)상 parent=base_link로 추정. RViz에서 포인트클라우드가 엉뚱한 곳/뒤집혀 뜨면 `np.linalg.inv(T)`가 답이다. **부호를 만지지 말 것.**

## realsense-viewer와 ROS 노드는 동시에 못 쓴다 (2026-08-02, 실측)

`realsense-viewer`가 USB 디바이스를 **독점**한다. 뷰어를 켜두면 `realsense2_camera` 노드가 죽거나
프레임을 못 받고, `/camera/*` 토픽이 통째로 사라진다. 증상이 "TF 프레임 없음"으로 나타나서
캘리브 문제로 오진하기 쉽다. **뷰어를 먼저 닫고 노드를 띄운다.**

## 🔴 이 랩탑 하드웨어 — 2026-08-05 재측정 (아래 2026-08-02 기록은 다른 랩탑 것이었다)

**실기 랩탑(hostname `rokey`)에서 직접 측정한 값이다. 이 절이 우선한다.**

| 항목 | 실측 (2026-08-05, hostname `rokey`) | 확인 방법 |
|---|---|---|
| CPU | **Intel i7-13620H** — 10코어/16스레드 | `lscpu` |
| GPU | **NVIDIA GeForce RTX 4060 Laptop 8GB** (+ Intel iGPU) | `lspci`, `nvidia-smi` |
| 드라이버 | 595.84, `nvidia-smi`가 보고하는 CUDA 런타임 13.2 | `nvidia-smi` |
| CUDA 툴킷 | `/usr/local/cuda-12.4` 존재하고 `nvcc` 바이너리도 있으나 **PATH에 없다** — 맨셸에서 `nvcc` = command not found. 쓰려면 PATH/LD_LIBRARY_PATH 설정 필요 | `ls -l /usr/local/cuda*/bin/nvcc`, `nvcc --version` |
| GPU 스택 | **`torch` 미설치, `curobo` 미설치.** `~/cobot2_ws/isaac_ros-dev/`는 존재하나 `COLCON_IGNORE`가 있고 `src/`만 있음 | `python3 -c "import torch"`, `ls` |
| 상시 부하 | `ros2_control_node` **208%**, move_group 23%, rviz2 21%+15%, joint_state_publisher 8% | `top -o %CPU` |
| GPU 유휴 시 | util 0%, 16.68 W, 24 MiB. ⚠️ `nvidia-smi` 첫 샘플이 **590W/80W·17%** 같은 쓰레기 값을 뱉은 적 있음 — **한 번 더 재고 쓸 것** | `nvidia-smi --query-gpu=... --format=csv` |

⚠️ **RViz가 어느 GPU로 렌더링 중인지는 미확인이다.** `prime-select`는 `on-demand`이고
`nvidia-smi` 프로세스 목록에 Xorg(4MiB)·gnome-shell(2MiB)뿐 rviz2가 없다는 **정황**은 iGPU를
가리키지만, `glxinfo`(mesa-utils)가 미설치라 실제 GL renderer 문자열을 확인하지 못했다.
확정하려면 `sudo apt install mesa-utils` 후 `glxinfo -B | grep "OpenGL renderer"`.

**"GPU 없음"을 근거로 내려진 결정 중 재검토 대상 — 다만 항목마다 근거가 다르다:**
- nvblox / FoundationPose / GraspGenX / cuMotion / cuRobo 제외
  ([[ws/cobot2/M0609_perception_motion_sprint_plan]] 3절·438행) → **GPU가 진짜 전제였으므로 무효.**
  단 위 표대로 torch·curobo가 아직 없어 "가능해졌다"이지 "준비됐다"가 아니다.
- `camera.launch.py`의 `424x240x15` 기본값 → 주석 근거가 거짓이므로 재검토 대상.
- 🔴 `sensors_3d.yaml`의 `point_subsample: 3`, `max_update_rate: 1.0` → **이건 GPU 근거가 아니었다.**
  원래 근거는 CPU다(`review_moveit.md:57` "CPU 부하 축소", `:60` "ros2_control_node 204% 경합 방지").
  그리고 그 204%는 재측정에서 **208%로 사실상 그대로**다 — 하드웨어 정정이 이 결정을 무너뜨리지 않는다.
  octomap 삽입에는 GPU가 한 줄도 안 쓰인다. 2026-08-05에 `max_update_rate`만 5.0으로 올리고
  `point_subsample`은 3으로 되돌렸다(한 번에 하나만). 상세는 `sensors_3d.yaml` 주석.

⚠️ **`PointCloudOctomapUpdater`는 단일 스레드다.** 헤더
`/opt/ros/humble/include/moveit/pointcloud_octomap_updater/pointcloud_octomap_updater.h`의
`octomap::KeyRay key_ray_;` — 레이캐스팅 버퍼가 인스턴스 멤버 1개라 구조적으로 동시 실행이 안 된다.
**이건 소프트웨어 속성이라 랩탑이 바뀌어도 유효하다.** 아래 2026-08-02 절에도 같은 말이 있지만
그 절은 하드웨어 기록이 틀려 강등됐으므로, 이 사실만 여기로 끌어올린다.
→ "16스레드니까 여유 있다"는 논거를 octomap 갱신 지연에 쓰면 안 된다.

## ✅ cuMotion은 Humble(Isaac ROS 3.2)에 있다 — 2026-08-05 확인

[[ws/cobot2/M0609_perception_motion_sprint_plan]] 229행의 **"cuMotion은 사실상 Jazzy 전용으로
재편되어 Humble 지원이 불확실"은 부정확하다.** `git ls-remote`로 확인한 실제 태그:

```
isaac_ros_cumotion 태그: … v3.2-12 v3.2-13 v3.2-14 v3.2-15 v3.2.0 v4.0-0 … v4.5-0
```

- **v4.x가 Jazzy 라인**이고, **3.2가 Humble 라인**이다. cuMotion은 3.2에도 있다.
- 이미 클론된 `isaac_ros_common` / `isaac_ros_nvblox` / `isaac_ros_pose_estimation`이
  전부 **`v3.2-14`** 태그다 → cuMotion도 같은 `v3.2-14`가 존재해 **버전 정렬이 깨끗하다.**
- 같은 문서 114행·462행의 "Humble 유지하려면 3.2로 고정" 정책 자체는 옳다. 틀린 건
  "그래서 cuMotion을 못 쓴다"는 결론뿐이다.

### 착수 전 실제 블로커 (2026-08-05 실측) — GPU 유무가 아니다

| 항목 | 상태 | 성격 |
|---|---|---|
| `nvidia-container-toolkit` | ❌ **미설치** (`dpkg -l` 0건, apt 저장소에도 없음) | 🔴 컨테이너에서 GPU를 못 본다. NVIDIA apt 저장소 추가 + `sudo apt` 필요 |
| docker 그룹 | ❌ `kimkh`가 **docker 그룹에 없음** (`sudo`에는 있음) | 🔴 `docker ps`가 socket permission denied |
| Docker | ✅ 29.7.0 설치됨 | — |
| 네트워크 | ✅ github clone 가능 (`git ls-remote` 성공) | — |
| `isaac_ros_cumotion` | ❌ 아직 클론 안 됨 (다른 3개는 `v3.2-14`로 있음) | 작업 항목 |
| cuMotion용 로봇 구 모델(XRDF) | ❌ M0609+RG2용은 존재하지 않음 | 🟡 **직접 작성해야 함. 여기가 진짜 일정 리스크다** |

> ⚠️ 위 🔴 두 개는 **팀 공유 랩탑의 시스템 전역 변경**이다(`~/.claude/CLAUDE.md` 5절 —
> 다른 계정이 의존하는 자원은 임의로 건드리지 않는다). 팀 합의 후 사용자가 직접 실행할 것.

> `/home/rokey/`는 `drwxr-x---`로 **권한 거부**라 그 계정의 curobo를 참고할 수 없다(2026-08-05 확인).
> 접근 가능한 곳(`/home/jjh`, `/home/kimkh`, `/opt`)에 curobo 설치본은 없다. `~/.cache/uv`에
> `nvidia-curobo` 메타데이터(567 B)만 있는데 이건 PyPI 조회 흔적이지 설치가 아니다.

⚠️ RViz를 dGPU로 돌리려면 명시적 오프로드가 필요하다(on-demand라 기본은 iGPU):
`__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia rviz2 ...`
단 이건 **렌더링만** 옮긴다. octomap 갱신 지연(`max_update_rate`)과는 무관하다.

---

## 🔴 cuMotion은 MoveIt의 octomap을 **안 본다** (2026-08-05, 소스 확인)

`isaac_ros_cumotion` v3.2 기준. 이 워크스페이스의 "사람 팔 우회" 목표에 직결되는 사실이다.

cuMotion 플래너가 세계를 받는 경로는 한 곳뿐이다:

```python
# cumotion_planner.py:662-665
scene = goal_handle.request.planning_options.planning_scene_diff
world_objects = scene.world.collision_objects        # ← collision_objects "만"
```

- `cumotion_planner.py` 전체에 **`octomap` 문자열 0건** (grep 확인)
- MoveIt 플러그인은 `getPlanningSceneMsg()`로 **전체** 씬을 넘겨준다
  (`cumotion_move_group_client.cpp:72,81`) — 버리는 쪽은 **받는 노드**다

| 장애물 종류 | OMPL | cuMotion (`read_esdf_world:=False`) | cuMotion + nvblox |
|---|---|---|---|
| 테이블·박스 (CollisionObject) | 본다 | 본다 | 본다 |
| **사람 팔 (octomap 복셀)** | **본다** | **❌ 못 본다** | 본다 (ESDF) |

**결론 2개:**
1. **nvblox는 선택이 아니라 필수다.** cuMotion이 미모델링 장애물을 보는 유일한 경로다.
2. **가장 위험한 실패 방식이다 — 성공처럼 보인다.** 계획은 빠르게 성공하고 에러도 안 난다.
   사람 팔을 통과하는 궤적이 나와도 로그로는 안 드러난다.
   → **nvblox 없이 cuMotion으로 사람 팔 실기를 돌리지 않는다.**

관련: [[ws/cobot2/plans/2026-08-05-cumotion-bringup]] §4-3

---

## 🔴 cuMotion은 `/joint_states`에 **velocity 배열을 요구한다** (2026-08-06, 실측)

요청의 `start_state`가 비어 있으면(RViz "current" 시작자세나 우리 스크립트가 그렇다)
`cumotion_planner_node`는 `/joint_states`를 **직접** 읽는다. 이때 velocity 길이가
position과 다르면 **계획을 아예 포기한다** (`cumotion_planner.py:698-704`):

```
start joint position shape is torch.Size([1, 12]) start velocity shape is torch.Size([1, 0]),
both should match. JointState was read from /joint_states
```

- `joint_state_publisher`의 `publish_default_velocities` **기본값이 False**라 velocity가 아예 안 실린다
  → 길이 0 vs 12로 불일치 → 게이트 E 첫 측정에서 **계획 10/10 실패**(`ERROR(-1)`)
- **OMPL은 멀쩡히 됐다.** OMPL은 planning scene의 current state를 쓰고 이 토픽을 직접 안 읽는다.
  → "OMPL 되는데 cuMotion만 안 되면 `/joint_states`의 velocity부터 본다."
- 조치: `bringup.launch.py` / `moveit.launch.py`의 joint_state_publisher에
  `publish_default_velocities: True` 추가(커밋됨). 소스가 velocity를 주면 그대로 전달되고,
  안 주는 관절(그리퍼)은 0.0으로 채워진다
- ⚠️ **`ros2 topic info /joint_states`로 publisher가 1개인지 먼저 확인할 것.**
  옛 launch가 안 죽고 남아 있으면 velocity 있는 메시지와 없는 메시지가 **번갈아** 오고,
  계획이 산발적으로만 실패해서(2026-08-06: 20회 중 3회) 원인 추적이 훨씬 어려워진다

---

## 🔴 이 랩탑은 **세 계정이 동시에 로그인해 같은 ROS 그래프와 같은 GPU를 쓴다** (2026-08-06, 실측)

`state.md`가 "팀 공유 랩탑"이라고만 적어 둔 것의 **구체적 결과**다. 2026-08-06 실측:

```
$ who
joonwon  :1   09:33      ← 동시 로그인
kimkh    :2   09:37
rokey    :3   12:55

$ docker ps
cumotion-joonwon                 isaac_ros_dev-x86_64:latest   ← 별개 컨테이너, 같은 GPU
isaac_ros_dev-x86_64-container   isaac_ros_dev-x86_64          ← kimkh
od_kimkh / object_detection_x11 / portainer
```

- 컨테이너가 `--network host` + 다들 `ROS_DOMAIN_ID=93` → **세 계정이 같은 ROS 그래프를 본다.**
  `joonwon`이 띄운 `move_group`이 내 `ros2 node list`에 그냥 나온다
- **GPU도 하나다(RTX 4060 8GB).** 상대가 cuMotion을 돌리는 중이면 내 VRAM이 모자란다
- ⚠️ **"중복 노드"의 상당수가 사실 남의 프로세스다.** 2026-08-06에 `/move_group`이 2개라서
  한참 헤맸는데 하나가 `joonwon` 것이었다. `kill`이 조용히 실패하면(uid가 달라서) 이걸 의심한다

**누구 프로세스인지 먼저 본다 — `ps`에 `user`를 꼭 넣는다:**

```bash
ps -eo pid,user,lstart,cmd | grep -E "move_group|nvblox|cumotion|camera" | grep -v grep
```

**남의 프로세스는 죽이지 않는다.** 내 것만 PID로 지목해 내린다.
`pkill -f`는 이 환경에서 특히 위험하다 — 패턴이 남의 프로세스와 자기 셸에도 걸린다.

### GPU를 넘길 때 (세션 종료 절차)

```bash
nvidia-smi --query-compute-apps=pid,used_memory --format=csv   # 누가 쥐고 있나
ps -o pid,user,cmd -p <pid>                                     # 내 것인지 확인 후에만 kill
nvidia-smi --query-gpu=memory.used --format=csv,noheader        # 아무도 안 쓰면 ~33 MiB
```

컨테이너는 **지우지 말고 남긴다**(`docker stop`도 하지 않는다) — 다음에 `run_dev.sh`를 다시 돌리면
새로 만들어져 `container_setup.sh`를 또 돌려야 한다. GPU는 안의 노드만 내리면 반납된다.

---

## 🔴 nvblox 경로에는 `robot_segmenter_node`가 **필수다** — 없으면 로봇이 자기 몸을 장애물로 본다 (2026-08-06, 실측)

증상: cuMotion 계획이 **전부** 실패하고 사유가

```
MotionGenStatus.INVALID_START_STATE_WORLD_COLLISION
```

= "지금 자세가 이미 세계와 충돌". 로봇을 어디에 두든 똑같이 실패한다.

**원인**: nvblox는 RealSense **원본 depth**를 먹는다. MoveIt octomap 경로에 있는 self-filter
(`sensors_3d.yaml`의 `padding_offset`/`padding_scale`)를 **안 거친다.** 그래서 카메라에 보이는
로봇 팔·그리퍼가 그대로 TSDF/ESDF에 장애물로 들어간다. RViz에서 그리퍼 주변에 점이 찍히면 이것이다.

**조치**: NVIDIA가 이걸 위해 만든 노드가 `isaac_ros_cumotion`에 들어 있다.
파이프라인 사이에 끼운다 — **depth → `robot_segmenter_node` → nvblox**:

```bash
ros2 run isaac_ros_cumotion robot_segmenter_node --ros-args \
  -p robot:=m0609_rg2.xrdf \
  -p urdf_path:=/workspaces/isaac_ros-dev/m0609/m0609_kinematics.urdf \
  -p distance_threshold:=0.15 \
  -p depth_image_topics:="[/camera/camera/aligned_depth_to_color/image_raw]" \
  -p depth_camera_infos:="[/camera/camera/aligned_depth_to_color/camera_info]" \
  -p robot_mask_publish_topics:="[/cumotion/camera_1/robot_mask]" \
  -p world_depth_publish_topics:="[/cumotion/camera_1/world_depth]"
```

그리고 nvblox의 depth 입력을 `/cumotion/camera_1/world_depth`로 바꾼다
(**camera_info와 color는 원본 그대로** — 세그멘터는 depth만 만든다).
⚠️ **nvblox를 재시작해야 한다.** 기존 지도에 이미 로봇이 박혀 있으면 입력만 바꿔도 안 지워진다.

- `distance_threshold`(m)는 로봇 구에서 얼마나 여유를 두고 지울지. 0.15로 통과 확인
- 결과(2026-08-06): cuMotion 계획 **5/5 성공**, `/curobo/voxels`에 점유 복셀 정상 적재

---

## 🔴 이미지의 numpy 2.2.6이 `cv2`를 깨서 `robot_segmenter_node`가 못 뜬다 (2026-08-06, 실측)

```
File ".../robot_segmenter.py", line 19, in <module>
    import cv2
ImportError: numpy.core.multiarray failed to import
```

- Isaac ROS 3.2 이미지: `/usr/local/.../numpy 2.2.6`이 apt numpy 1.21.5를 가린다.
  그런데 `cv2`는 apt판(`/usr/lib/python3/dist-packages/cv2...so`)이라 numpy 1.x 빌드다 → 깨진다
- 조치: `pip3 install "numpy==1.26.4"` (컨테이너 안, `~/.local`에 깔려 `/usr/local`을 가린다).
  검증: `cv2 4.5.4 OK` / `torch 2.13.0+cu130 cuda=True` / `warp 1.5.0` / `curobo` 전부 정상
- ⚠️ **대가**: `cupy-cuda12x`가 numpy≥2.0을 요구해서 깨진다. 우리 파이프라인에서 cupy를 쓰는 건
  `isaac_ros_cumotion_object_attachment`뿐이고(전수 grep) 그 노드는 안 띄우므로 무해하다.
  **object_attachment를 쓰게 되면 이 결정을 다시 봐야 한다.**
- ⚠️ **`.claude/hooks/guard.sh:13`이 이 조치를 오탐으로 막는다.** 패턴 `numpy[=><]*2`가
  `numpy<2`에도 걸린다 — 금지하려던 건 `>=2`인데 **해결책까지 차단한다.**
  지금은 `numpy==1.26.4`처럼 명시 핀으로 우회한다.

---

## 🔴 `run_dev.sh`는 컨테이너를 **재사용하지 않고 새로 만든다** → pip 설치가 매번 날아간다 (2026-08-06)

2026-08-06 하루에 같은 자리에서 **두 번** 막혔다 — `AttributeError: module 'warp' has no attribute 'torch'`.
`docker inspect`의 `StartedAt`이 갱신되고 `/usr/local/.../warp` mtime이 이미지 빌드 시각으로 돌아가 있으면 재생성된 것이다.

- 살아남는 것: 바인드 마운트 안의 모든 것 — curobo 패치(`isaac_ros-dev/src/...`), colcon 산출물
- 날아가는 것: `pip3 install`로 넣은 warp 1.5.0, numpy 1.26.4
- 조치: **컨테이너에 들어갈 때마다 `bash /workspaces/cobot2_ws/scripts/container_setup.sh`**

---

## ⚠️ RealSense 드라이버를 두 번 띄우면 depth가 절반 이하로 떨어진다 (2026-08-06, 실측)

`ros2 node list`에 `/camera/camera`가 **2개** 보이면 이것이다(`ros2 topic info`가 아니라 node list로 본다).
한 D435i를 두 드라이버가 물면 USB 경합으로 `aligned_depth_to_color`가 15 Hz → **5.6 Hz**로 떨어졌다.
드라이버 하나를 죽이니 7.3 Hz로 회복(여전히 15 Hz 미달 — 별개 미해결).
→ **카메라를 띄우기 전에 `ros2 node list | grep camera`로 먼저 확인한다.**

---

## 🔴 nvblox `esdf_mode` 기본값 `2d`가 cuMotion의 첫 ESDF 요청에 **프로세스째 죽는다** (2026-08-06, 실측)

```
[FATAL] nvblox_node: The ESDF service is only intended for mapping with 3D ESDFs.
        You're in 2D mode. To use this function set esdf_mode: 3d. Exiting.
```

- 기본값 출처: `nvblox_examples_bringup/config/nvblox/nvblox_base.yaml:33` → `esdf_mode: "2d"`
- 조치: `nvblox_node`에 **`-p esdf_mode:=3d`**. 그러면 `static_mapper.esdf_slice_*`는 의미가 없어진다(2d 슬라이스용)
- ⚠️ **증상이 엉뚱한 데를 가리킨다.** cuMotion 로그에는 `Calling ESDF service` 뒤에 계획 실패만
  남고, 정작 죽은 건 nvblox다. → **cuMotion 계획이 실패하면 `pgrep -f nvblox_node`부터 본다.**
- "노드가 뜬다"는 검증으로는 안 잡힌다. **서비스를 실제로 한 번 불러봐야** 드러난다:
  `ros2 service call /nvblox_node/get_esdf_and_gradient nvblox_msgs/srv/EsdfAndGradients "{update_esdf: true, use_aabb: true, frame_id: base_link, aabb_min_m: {x: -1.0, y: -1.0, z: -1.0}, aabb_size_m: {x: 2.0, y: 2.0, z: 2.0}}"`
  → 정상이면 41×41×41 그리드 반환(`voxel_size_m: 0.05`, 미관측 voxel은 `-1000.0`)

---

## ✅ 컨테이너 RMW는 **기본값(Fast DDS)** 그대로 둔다 (2026-08-06, 실측 정정)

nvblox·cuMotion을 굳이 `rmw_cyclonedds_cpp`로 띄울 이유가 없다 — 기본 RMW로
서비스 조회·파라미터 조회·ESDF 서비스 호출이 전부 정상 동작했다.

🔴 **오히려 cyclonedds를 켜면 실기가 깨진다.** `moveit.launch.py standalone:=false`의
`dsr_moveit_controller` spawner가 **호스트** `/dsr01/controller_manager` **서비스**를 부르는데,
교차 벤더에서 **토픽은 되고 서비스는 안 된다**(2026-08-06 실측). 호스트가 기본 Fast DDS이므로
컨테이너도 기본값이어야 spawner가 산다. → `RMW_IMPLEMENTATION`을 **설정하지 않는다.**

---

## 🔴 XRDF 충돌 구가 뚱뚱해서 all-zeros가 자기충돌로 잡혔다 (2026-08-06, 실측)

게이트 E에서 velocity를 고친 뒤 다음 벽:
`MotionGenStatus.INVALID_START_STATE_SELF_COLLISION` — all-zeros 시작자세에서 계획 전부 실패.

`scripts/diag_self_collision.py`(링크쌍별 침투량을 이름으로 찍는다)로 확인:
겹친 링크쌍 25개 중 **XRDF `self_collision.ignore`에 없던 6쌍**이 원인.

| 링크쌍 | all-zeros 침투 | 성격 |
|---|---|---|
| `base_link ↔ link_2` | 77.7 mm | 실제로 움직이는 쌍 ⚠️ |
| `link_4 ↔ rg2_base_link` | 49.2 mm | 실제로 움직이는 쌍 ⚠️ |
| `rg2_left_outer_knuckle ↔ rg2_right_inner_knuckle` | 28.2 mm | 그리퍼 내부(lock) |
| `rg2_left_inner_knuckle ↔ rg2_right_outer_knuckle` | 28.2 mm | 그리퍼 내부(lock) |
| `rg2_base_link ↔ rg2_left_inner_finger` | 11.9 mm | 그리퍼 내부(lock) |
| `rg2_base_link ↔ rg2_right_inner_finger` | 11.1 mm | 그리퍼 내부(lock) |

**링크가 겹친 게 아니라 구가 뚱뚱한 것이다** — 근거는 **같은 자세를 OMPL이 10/10 통과**했다는 것.
OMPL은 실제 메시로, cuMotion은 XRDF 구로 충돌을 본다. XRDF `geometry:` 절 주석이 이미
"팔 링크는 실제 단면보다 1.5~2.1배 뚱뚱하다"고 적고 있었다.

- 조치: 6쌍을 `m0609_rg2.xrdf`의 `self_collision.ignore`에 추가 → 계획 **20/20 성공**
- ⚠️ **SRDF에는 이 6쌍이 없다.** MoveIt/OMPL은 계속 검사하고 cuMotion만 안 본다
- ⚠️ **앞 2쌍은 보호를 포기한 것이다.** 특히 `link_4 ↔ rg2_base_link`는 joint_5/6을 접으면
  그리퍼가 팔뚝으로 돌아오는 실제 경로다. **실기 모션 전 재검토 필수.**
  정공법은 ignore가 아니라 `base_link`·`link_2`·`link_4`·`rg2_base_link` 구 재피팅(`scripts/fit_spheres.py`)
- XRDF 정본은 `src/cobot_rg2/rg2/m0609_rg2_moveit/config/m0609_rg2.xrdf`.
  고치면 `isaac_ros-dev/m0609/`와 `isaac_ros_cumotion_robot_description/xrdf/`에 **다시 복사**한다
  (후자는 build/install이 symlink 체인이라 재빌드는 불필요 — 2026-08-06 확인)

---

## ~~octomap_server — 이 랩탑 리소스로는 기본 설정이 안 돌아간다 (2026-08-02)~~ ← 다른 랩탑 기록

> 🔴 **2026-08-05 확인: 아래 하드웨어 사양은 이 실기 랩탑(`rokey`)이 아니라 개인PC에서 측정된 것이다.**
> 사용자 확인 완료. 하드웨어 수치는 위 절을 보고, 아래는 "저사양 머신에서의 octomap 튜닝 사례"로만 읽는다.

측정한 실제 하드웨어 (❌ 개인PC, 이 랩탑 아님):
- CPU **Intel i7-10510U** — 4코어/8스레드, 1.8GHz base, **15W 노트북 U-시리즈**
- GPU **없음**. Intel UHD 내장(CometLake-U GT2)뿐 — `nvidia-smi` 미설치, `lspci`에 외장 GPU 없음
- RAM 15 GB
- **상시 부하: `ros2_control_node` 204% (= 2코어 점유)**, rviz2 ~12%, joint_state_publisher ~10%

octomap 부하 산술 추정(미실측, 추정):
- 848×480 = 407k point/frame × 30 Hz = **12.2 M point/s**
- `sensor_model.max_range` 기본값이 **-1(무제한)** 이라 ray 하나가 수백 voxel을 free로 갱신
- `octomap_server`는 **단일 스레드**. 1.8GHz 코어 하나로는 불가능

동작하는 설정(부하 약 1/20):
```bash
ros2 launch realsense2_camera rs_launch.py \
  enable_depth:=true align_depth.enable:=true \
  depth_module.depth_profile:=424x240x15          # 100k point × 15Hz

ros2 run octomap_server octomap_server_node --ros-args \
  -r cloud_in:=/camera/camera/depth/points_xyz \
  -p frame_id:=base_link \
  -p resolution:=0.03 \
  -p sensor_model.max_range:=1.5 \
  -p pointcloud_min_z:=-0.1 -p pointcloud_max_z:=1.2
```
~~`max_range:=1.5`는 카메라가 base에서 993mm 떨어져 있음을 근거로 한 값~~ → **993 mm는 좌표 규약 버그
수정 전의 폐기된 거리다. 근거로 쓰지 말 것.** 현재 거리는 npy를 읽어 확인한다(위 "카메라 TF 연결" 절).
`topic_tools` 미설치라 throttle 노드는 못 쓴다 — **카메라 프로파일에서 줄이는 게 정답**.

### octomap 로그 읽는 법
- `Message Filter dropping message ... 'discarding message because the queue is full'`
  → **CPU가 밀린 게 아니라 TF를 못 구한 것.** message_filter 큐(기본 5)가 넘친 것이다.
  TF 체인부터 확인한다. (단, TF를 고치면 위 리소스 문제로 같은 메시지가 다시 뜬다 — 원인이 두 개다)
- `Could not open file` → `octomap_path` 파라미터가 비어서 나는 정상 메시지. 무시.
- `Nothing to publish, octree is empty` → 아직 포인트가 하나도 안 들어옴. 위와 같은 원인.

### RViz 디버깅 — `projected_map`은 쓰지 말 것
| 토픽 | 타입 | 판정 |
|---|---|---|
| `/projected_map` | `nav_msgs/OccupancyGrid` | ❌ 2D 평면 투영. 높이가 뭉개져 매니퓰레이터엔 정보 없음 |
| `/octomap_binary`, `/octomap_full` | `octomap_msgs/Octomap` | ❌ **표시 불가** — `octomap_rviz_plugins` 미설치(`dpkg -l` 확인). 필요하면 `sudo apt install ros-humble-octomap-rviz-plugins` |
| `/octomap_point_cloud_centers` | `sensor_msgs/PointCloud2` | ✅ **이걸 쓴다.** 플러그인 없이 RViz 기본 PointCloud2 display로 3D 확인 가능 |

**디버깅 순서 (위에서 실패하면 아래로 내려가지 않는다):**
1. `ros2 run tf2_ros tf2_echo base_link camera_depth_optical_frame` — 값 나오나
2. `ros2 topic hz /camera/camera/depth/points_xyz` — 흐르나
3. RViz `Fixed Frame=base_link` + PointCloud2 on `/camera/camera/depth/points_xyz`
   — 로봇 모델 옆 제자리에 뜨나. **캘리브 정확도 검증이 여기서 같이 끝난다.**
4. 그 다음에야 `/octomap_point_cloud_centers`

RViz **TF display → Tree**를 켜두면 어느 마디가 끊겼는지 로그 없이 바로 보인다.

## ROS_DOMAIN_ID 함정 재확인 (2026-08-02)
`ros2 node list`가 비어 보이면 노드가 죽은 게 아니라 **도메인이 다른 것**부터 의심한다.
작업 도메인은 **93**(`rdm` alias). 새 터미널·스크립트마다 `export ROS_DOMAIN_ID=93`이 필요하다.

## QoS 불일치 — 센서 토픽은 BEST_EFFORT다 (2026-08-02, 실측)

`depth_image_proc/point_cloud_xyz_node`는 `/camera/camera/depth/points_xyz`를 **BEST_EFFORT**로 발행한다.
RViz와 `ros2 topic hz`는 기본이 **RELIABLE**이라 그냥 붙이면 한 개도 못 받는다.

```
[WARN] [PointCloudXyzNode]: New subscription discovered on topic '...', requesting
       incompatible QoS. No messages will be sent to it. Last incompatible policy:
       RELIABILITY_QOS_POLICY
```

- **RViz**: PointCloud2 display → Topic → **Reliability Policy = Best Effort**
- **CLI**: `ros2 topic hz <topic> --qos-reliability best_effort`
- 발행자 쪽 `qos_overrides` 파라미터는 `parameter_events`용뿐이라 노드에서 못 바꾼다. **구독자가 맞춘다.**
- `ros2 topic info -v <topic>`으로 양쪽 Reliability를 직접 비교하는 게 가장 빠른 확인법이다.

### 침묵의 3원인을 가르는 순서
`topic hz`가 아무것도 안 뱉는 이유는 세 가지고, `hz`만으로는 구분이 안 된다. 위에서부터 확인한다.
1. `ros2 topic list | grep <이름>` — **토픽 자체가 없다**(발행 노드가 죽음). 실제로 `point_cloud_xyz_node`가
   카메라 재기동 때 같이 죽어 여기 걸린 적 있다(2026-08-02).
2. `ros2 topic info -v <이름>` — **QoS 불일치**(위 항목)
3. 그제서야 진짜 처리량/TF 문제

## 좌표 규약: npy는 OpenCV optical, ROS camera_link는 body (2026-08-02, 실측으로 걸림)

`T_cam2base.npy` / `T_gripper2camera.npy`는 `cv2` 출력이라 **OpenCV optical 규약**이다.
ROS `camera_link`는 REP-103 body 규약이다. 둘은 90° 짝만큼 다르다.

| 규약 | 전방 | 우 | 하/상 |
|---|---|---|---|
| OpenCV optical (`*_optical_frame`) | **+z** | +x | +y(하) |
| ROS body (`camera_link`, REP-103) | **+x** | -y | +z(상) |

**증상**: 포인트클라우드가 로봇 옆에 통째로 떨어져 나간다. TF 트리는 멀쩡히 연결돼 있어서
"캘리브 값이 나쁜가" / "inv(T)인가"로 오진하기 쉽다. **둘 다 아니다.**

**지문**: 발행 중인 TF의 `RPY`에서 **roll ≈ ±90°**. 어제 값이 `[-95.7, 12.9, 110.9]`였다.
보정 후 `[12.9, 5.6, -157.8]` — 1m 옆에서 로봇 쪽을 되돌아보는 자세로 물리적으로 말이 된다.

**판정법 (규약 가설을 숫자로 채점한다)**: 카메라 위치에서 시선 벡터를 뽑아 로봇 base를 향하는
각도를 잰다. 실제 4가지 가설 채점 결과:

| 가설 | 시선각 |
|---|---|
| T 그대로, body 규약 (틀린 상태) | 81.0° |
| `inv(T)`, body 규약 | 42.0° |
| `inv(T)`, optical 규약 | 52.0° |
| **T 그대로, optical 규약** ✅ | **32.5°** |

남은 32.5°는 카메라가 base 원점이 아니라 그 앞 작업대를 겨냥해서다 — 정상.

**수정**: `src/cobot_rg2/rg2/m0609_rg2_bringup/scripts/calib_npy_to_tf.py`가 변환을 **기본으로 적용**한다(`--no-optical`로 생략 가능).
호출부마다 고치면 재발하므로 변환 지점 한 곳에서만 처리한다. `--selfcheck`에 검증 assert 있음.
roll이 여전히 ±90° 근처면 경고를 찍는다.

> ⚠️ **이전 기록에 있던 "틀리면 `np.linalg.inv(T)`가 답"은 오답이었다.** 방향(parent/child) 문제가
> 아니라 규약 문제였다. 규약을 먼저 의심할 것.

**육안 판정 기준**: depth 이미지에 로봇 팔이 찍혀 있으면 포인트클라우드에도 팔이 있어야 하고,
그게 **로봇 모델 위에 포개져야** 한다. "로봇 근처에 있다"는 통과가 아니다.

## MoveIt octomap 실기 Execute (2026-08-03, 사용자 구두 보고 — 정량 미측정)
- 실기 Execute까지 진행해 장애물 회피 확인. 단 **캘리브 오차로 장애물 영역 경계가 모호하게 잡혔다** — 그래도 무시되지는 않고 회피는 수행됨.
- 오차 정량값(cm)은 아직 안 잼. `padding_offset:0.1`(현재값)이 오차를 흡수해서 완전히 안 지워지고 모호하게만 잡힌 것으로 보이나 **이건 추론**이다. 실측은 알려진 좌표 물체로 다음 세션에.
- 상세 리뷰·cuRobo 비교 설계는 [[ws/cobot2/review_moveit]].

## MoveIt octomap 연동 (2026-08-02)

### octomap_server와 MoveIt의 octomap은 **별개다**
`/octomap_binary`를 구독하는 MoveIt 기능은 **없다.** MoveIt은 `move_group` 안에서
`occupancy_map_monitor` + `PointCloudOctomapUpdater`로 **자기 octree를 직접 만든다.**

```
                        ┌─→ octomap_server ─→ /octomap_binary, /projected_map
depth → points_xyz ─────┤   (독립 지도. RViz·nav2용. MoveIt과 무관)
                        └─→ move_group 내부 occupancy_map_monitor
                            → PlanningScene.world.octomap   ← MoveIt은 이쪽만 본다
```
→ **충돌 회피가 목적이면 `octomap_server`를 켜지 않는다.** 둘 다 돌리면 같은 클라우드로
octree를 두 번 만들어 CPU를 이중으로 먹는다(이 랩탑에선 치명적).

### 막힌 것 (2026-08-02 확인)
- ⛔ **`ros-humble-moveit-ros-perception` 미설치.** `apt-cache policy` → `Installed: (none)`.
  `PointCloudOctomapUpdater` 플러그인 본체가 여기 있다. 설치된 moveit 10개 중
  `moveit-ros-occupancy-map-monitor`(뼈대)만 있어서, 이대로 설정하면 **플러그인 로드 실패로 조용히** octomap이 안 생긴다.
- ~~`m0609_rg2_moveit`에는 `sensors_3d.yaml`이 아예 없다~~ → **2026-08-02 작성 완료.** 아래 "설정" 절 참고.
  (`dsr_moveit_config_m0609` 쪽에도 있으나 `sensors: []`라 어차피 무의미하다.)

### 어느 moveit config를 쓰나 — **`m0609_rg2_moveit`**
`dsr_moveit_config_m0609`는 URDF에 RG2가 없어 **self-filter가 그리퍼를 못 거른다.**
`m0609_rg2_moveit`의 SRDF에 `virtual_joint(parent_frame="world", child_link="base_link")`가 있지만
**플래닝 프레임은 `world`가 아니라 `base_link`다.** `octomap_frame`도 **`base_link`**.
(상단 "플래닝 프레임" 절과 같은 결론이다 — 근거·실측은 거기 있다.)

> 📌 **판단 번복 이력 (2026-08-02) — 결론은 `base_link`다.**
> 순서: ① 내가 근거 없이 `base_link`를 넣음 → ② 다음 세션에 "`world`는 발행이 빠질 수 있다"는
> **사실이 아닌 이유로 `world`로 바꾸고 정당화** → ③ 실측으로 `base_link`가 맞다고 확인하고 되돌림.
> ①은 근거가 없었을 뿐 값 자체는 맞았고, ②가 틀렸다.
>
> ②의 오류: SRDF의 virtual_joint가 `world`를 가리키니 플래닝 프레임도 `world`일 것이라 **추론**했다.
> 아니다 — MoveIt은 **fixed** 타입 virtual joint로는 모델 프레임을 만들지 않아 플래닝 프레임이
> 루트 링크(`base_link`)로 남는다. `world`는 TF에는 있지만 **planning scene은 모른다.**
> 실측: `frame_id='world'`로 CollisionObject 발행 → `[ERROR] Unknown frame: world`, 장애물 **조용히 무시**.
> `frame_id='base_link'` → `/monitored_planning_scene`에 정상 등록.
>
> **교훈: TF에 프레임이 있는 것과 planning scene이 그 프레임을 아는 것은 별개다.**
> 그리고 "정당화 문장을 쓰기 전에 명령을 한 번 돌려라" — 이때 필요했던 건 발행 한 번이었다.

### 🔴 PlanningScene diff의 `allowed_collision_matrix`는 **병합이 아니라 전체 교체**다 (2026-08-06, 실측)

`is_diff=true`로 보내도 ACM은 덧붙지 않고 **통째로 갈린다.** `world.collision_objects`가
diff로 동작하는 것과 규칙이 다르다 — 같은 메시지 안인데 필드마다 의미가 다르다.

- 증상: 그리퍼 링크 7개짜리 ACM만 diff로 보냈더니 SRDF의 `disable_collisions` 34개가 사라져
  `rg2_base_link ↔ rg2_left_outer_knuckle` 같은 **인접 링크가 자기충돌**로 잡혔다.
  → `/compute_ik`가 `avoid_collisions=true`에서 **모든 포즈에 대해** `NO_IK_SOLUTION(-31)`.
- **오진 유도**: 증상이 "그 포즈는 도달 불가"로 보인다. 실제로는 씬을 내가 망가뜨린 것이다.
- 가르는 순서 (위에서 갈리면 아래로 안 내려간다):
  1. 같은 포즈를 `avoid_collisions=false`로 → 풀리면 **도달성 문제가 아니다**
  2. `/check_state_validity`의 `contacts` → 전부 인접 링크쌍이면 ACM이 범인
  3. `/get_planning_scene`(`components=ALLOWED_COLLISION_MATRIX`)로 `entry_names` 확인
- **규칙: ACM은 반드시 `/get_planning_scene`으로 읽어서 얹어 되돌린다.**
  구현·회귀테스트는 `src/pick_fsm/pick_fsm/moveit_bridge.py: merge_acm()`.

### 대상 물체를 CollisionObject로 등록해도 octomap 복셀은 안 사라진다 (2026-08-06)

그리퍼 링크 ↔ 대상 물체를 ACM에서 허용하는 건 **필수**다(안 하면 grasp pose에서 손가락이
물체와 겹쳐 IK가 collision으로 실패). 하지만 그걸로 그 자리의 octomap 복셀은 안 없어진다.
남은 선택지는 둘 다 대가가 있다 — `clear_octomap`(사람 팔까지 같이 사라짐) 또는
그리퍼 링크 ↔ `<octomap>` 허용(그 링크의 octomap 충돌검사가 통째로 꺼짐).
**둘 다 기본 off로 두고, 계획 실패(=안 움직임)라는 안전한 실패를 기본값으로 삼는다.**
ACM에서 octomap을 부르는 이름은 `<octomap>`(`collision_detection::World::OCTOMAP_NS`).

### 캘리브 npy의 정본은 `corecode/` 쪽이다 — symlink로 해결 (2026-08-03)
- **정본**: `corecode/Calibration_Tutorial/T_cam2base.npy` — 재캘리브 결과가 나오는 위치가 여기로 고정돼 있다.
- `m0609_rg2_bringup/config/T_cam2base.npy`는 이제 **상대경로 symlink**(`../../../../../corecode/...`)다.
  git에 mode `120000`으로 커밋되므로 clone한 다른 계정·PC에서도 그대로 동작한다.

> 🔥 **사고 기록 (2026-08-03): `cp`를 잊어 사본이 480 mm 낡은 채로 조용히 돌았다.**
> 두 npy의 평행이동 차이가 `[-184.31, 425.18, -118.45]` mm. 에러가 안 난다 — 틀린 TF로 정상 동작한다.
> 2026-08-02의 340 mm 사건과 **같은 구조의 재발**이다(그때는 하드코딩, 이번엔 수동 사본).
> **규칙: 캘리브 결과처럼 "생성 위치가 정해진 산출물"은 사본을 만들지 않는다. symlink 아니면 경로 참조다.**

- ⚠️ 남은 함정: 상대경로 symlink는 **`--merge-install`에서 깨진다**(`install/share/<pkg>/config` 기준 5단계 위가
  ws 루트가 아니게 됨). 이 ws는 기본 isolated install이라 현재는 맞다. merge-install로 바꾸면 여기부터 확인할 것.
- ⚠️ 깨진 symlink는 `os.path.exists()`가 `False`라 `camera.launch.py`가 **경고만 찍고 정상 종료**한다.
  증상이 "포인트클라우드가 로봇과 안 붙음"으로만 나타난다.

### 설정 — **실제 값은 파일이 단일 출처다**
`src/cobot_rg2/rg2/m0609_rg2_moveit/config/sensors_3d.yaml` (2026-08-02 작성).
여기 복붙해 두면 갈라지므로 값은 옮겨 적지 않는다. 파일에 `[튜닝]` 주석으로 손잡이를 표시해 뒀다.

계획 단계의 값과 **실제 채택값이 다르다** — 아래가 채택된 쪽이다:

| 항목 | 계획(폐기) | 실제 |
|---|---|---|
| 센서명 | `default_sensor` | `realsense_pointcloud` |
| `point_cloud_topic` | `/camera/camera/depth/points_xyz` | `/camera/camera/depth/color/points` (RealSense가 직접 발행 → `depth_image_proc` 불필요) |
| `max_range` | 1.5 | 2.5 → 1.5 → **2.0 (2026-08-03 확정)** — 1.5는 카메라~base 거리(1.684 m)보다 작아 베이스 부근을 잘라냈다. **`max_range`는 CPU 손잡이가 아니다**(비용은 점 개수에 비례) |
| `point_subsample` | 1 | **3** (2026-08-03, CPU) |
| `padding_offset` | 0.03 | **0.1** (2026-08-03, self-filter 잔여점) |
| `padding_scale` | 1.2 | 1.0 → **2.0** (2026-08-03) |
| `max_update_rate` | 2.0 | 1.0 |
| `filtered_cloud_topic` | `/filtered_cloud` | `/moveit/filtered_cloud` |
| `octomap_frame` | `world` | **`base_link`** (`world`는 planning scene이 모른다 — 위 📌 참고) |
| `octomap_resolution` | 0.03 | **0.02** |

`octomap_frame`/`octomap_resolution`은 yaml이 아니라 `moveit.launch.py`의 `octomap_params`에서 주입한다.

### MoveIt이 실제로 받는 것
매 프레임: TF 조회(클라우드 stamp 기준) → **self-filter(ShapeMask, URDF collision 형상으로 로봇 자신 제거)**
→ raycast(경로는 free, 끝점은 occupied) → PlanningScene 반영 → `/monitored_planning_scene` 발행.
플래닝 시 FCL이 **로봇 링크 mesh vs octree cell**로 충돌검사 → 충돌 샘플 버림 → 궤적 생성.
**MoveIt은 "점유된 공간 덩어리"만 안다.** 물체 종류·그립 지점은 전혀 모른다(Day4 FoundationPose/GraspGenX 몫,
그쪽이 `CollisionObject`/grasp pose를 PlanningScene에 따로 넣는다).

### 함정
1. **self-filter가 최우선.** `padding_offset`이 캘리브 오차보다 작으면 자기 팔의 잔여 점이 남아
   **로봇이 자기 몸을 장애물로 보고 한 발짝도 못 움직인다.** 검증은 `/moveit/filtered_cloud`를 RViz에
   띄워 팔이 지워졌는지 **눈으로 보는 것**뿐이다.
2. **QoS가 또 걸릴 수 있다**(추론, 미확인). 클라우드가 안 들어오면 `ros2 topic info -v`로 양쪽 Reliability 비교.
3. **잔상**: octomap은 시간으로 안 사라진다. free 공간을 **다시 관측해야** 지워진다. 팔에 가려진
   뒤쪽은 계속 장애물로 남는다. 초기화는 `/clear_octomap` 서비스.
4. `max_update_rate: 2.0`이 이 랩탑의 실질적 CPU 방어선. 30Hz를 그대로 먹이면 `ros2_control`(상시 2코어)과 부딪힌다.

---

## OnRobot RG2 — 커맨드 단위 (2026-08-04 확인)

**근거**: `src/cobot_rg2/onrobot-ros2/onrobot_rg_msgs/msg/OnRobotRGOutput.msg`

| 필드 | 단위 | RG2 유효범위 | 실제 |
|---|---|---|---|
| `rgwd` (폭) | **1/10 밀리미터** | 0 ~ 1100 | 0 ~ 110.0 mm |
| `rgfr` (힘) | **1/10 뉴턴** | 0 ~ 400 | 0 ~ 40.0 N |

**미터 → mm로만 바꿔 넣으면 10배 좁게 명령해 물체를 으깬다.**
```python
rgwd = int(round(width_m * 10000))   # 0.048 m → 480
```

- **폭↔각도 변환을 새로 짜지 말 것** — `onrobot_rg_control/_OnRobotRGIsaacSimController.py:131`
  `widthToJointValue()` / `jointValueToWidth()`가 이미 있다. 실기 명령엔 `rgwd`를 직접 쓰고,
  관절각은 RViz/Isaac 시각화용으로만 쓴다.
- `rctr` 0x0001=grip(핑거팁 오프셋 미적용), 0x0010=grip_w_offset(적용), 0x0008=stop.

## 개구 폭 상수가 출처마다 다르다

| 값 | 출처 | 용도 |
|---|---|---|
| 0.102 m | GraspGenX `onrobot_RG2/config.json` `sweep_volume.extents[0]` | **grasp 선별 기준** (모델이 이 볼륨으로 conditioning) |
| 0.110 m | 드라이버 `max_width=1100` | 하드웨어 물리 한계 |
| 0.152 m | `XGripperInfo.width` (`bbox` x폭) | **개구 폭 아님 — 그리퍼 몸통 폭.** 혼동 주의 |

## M0609 도달 범위 (2026-08-04, URDF 실측)

**근거**: `src/cobot_rg2/doosan-robot2/dsr_description2/xacro/macro.m0609.white.xacro`
```
:34   base → joint_1   z = 0.1345      (shoulder 높이)
:118  0.411  +  :148  0.368  +  :220  0.121  =  0.900
```
**0.900 m는 shoulder(joint_2) 기준 flange까지**의 최대 신장이다.
base 원점 기준이 아니다 — 원점에서 재면 테이블 높이만큼 낙관적이 되어
못 가는 자세를 통과시킨다.

## 캘리브레이션 결과 파일 (2026-08-04 확인)

`corecode/Calibration_Tutorial/T_cam2base.npy`
- 이름은 cam2base지만 **내용은 `T_base←cam`** (`eye2hand_calibration.py:696` 주석에 명시)
- **병진이 mm 단위**. GraspGenX/FoundationPose는 m를 쓴다.
  → `T[:3,3] /= 1000.0` 없이 쓰면 1000배 어긋난다.
  (norm 값은 여기 적지 않는다 — 위 "현행 카메라 위치를 문서에 적지 않는다" 정책과 같은 이유다.
  이 줄에 박혀 있던 "1683.6"이 실제로 낡아 있었다: 2026-08-06 코드 감사로 다시 읽으니 1481.3이었다.)
- ⚠️ **`report_residuals()`(같은 파일 `:318`)는 잔차를 stdout에만 찍고 파일로 남기지 않는다**
  (2026-08-06 코드 감사로 확인). 그래서 지금 커밋된 `T_cam2base.npy`가 어느 세션·몇 mm 병진잔차로
  나온 결과인지 코드베이스 어디에도 안 남는다 — npy는 바이너리라 `git log`로도 못 찾는다.
  **재캘리브할 때마다 이 정보 공백이 반복된다.** 고치려면 `report_residuals()` 끝에
  `{timestamp, data_dir, n_images, translation_residual_median_mm, verdict}`를
  `calib_report.json`으로 옆에 남기면 된다(5줄) — `T_cam2base.npy`와 함께 커밋되므로 이후엔
  `git log`로 "이 커밋이 어떤 잔차 근거로 나왔는지"가 추적된다. **다음 재캘리브 전에 넣을 것을 권장.**
- `data_recording.py:77`의 "결과 부모 프레임도 flange"는 **틀린 주석**이다.
  eye-to-hand AX=XB에서 TCP 오프셋은 `A_i` 계산에서 소거되므로 부모는 항상 base
  (같은 파일 `:44-46`이 맞다).

## GraspGenX 관련

> **이 절이 소유하는 건 실측 사실(체크포인트 sha256, VRAM 측정치)뿐이다.**
> 출력 규약·폭 계산 함수·상류 버그·설계 결정은 [[ws/cobot2/detect_graspx]]가 단일 출처다 — 여기 옮겨 적지 않는다.

- **LFS 포인터 문제는 로컬 체크아웃 한정.** Lightning AI 원격에서는 RG2 데모 정상 동작(2026-08-04).
  로컬에서 메시 로드/시각화할 때만 `git lfs pull` 필요.
- `uv sync`가 CPython 3.14를 잡으면 torch 휠이 없다 → `uv sync --python 3.12`.
- grasp pose 규약: **+Z=접근축, +X=닫히는 방향, 원점=그리퍼 base_link**(`robot.py:59`).
  `fingertip=[0,0,0.18]` — TCP는 원점에서 +Z로 18cm.
  `graspmoe.py:289` 주석이 이를 반대로 적어놨다(**코드가 맞고 주석이 틀림**).
- `isaac_ros_foundationpose`는 **CAD 메시 필수**. model-free 모드 없음(`mesh_file_path` 파라미터).

### 체크포인트 무결성 — 크기가 맞아도 내용이 0일 수 있다 (2026-08-05, 실측)

`ext/graspgenx_checkpoints/release/gen/epoch_736.pth`의 정상 sha256:
```
8b55f31cdb8340a573b4df27b027c15cff326bd6debcb389bf631d2aaab7ac44   gen/epoch_736.pth   (1,210,918,342 B)
cbf3f3bdb2e4c03fca8486ed24de0e6a8a859e6bd22bce2f1434a610335abd3e   dis/epoch_1056.pth  (483,889,478 B)
```
- **LFS 포인터가 아니라 "내용만 0으로 채워진" 손상이 실재한다.** 크기·머리 바이트(`PK\x03\x04`)가 정상이라
  `ls`로는 절대 안 보인다. 증상은 `RuntimeError: PytorchStreamReader failed reading zip archive:
  failed finding central directory` — **원인과 무관한 문구다**(`.pth`는 zip이고, 꼬리가 없다는 뜻).
- 작업트리 사본과 `.git/lfs/objects/` 캐시 사본이 **서로 다른 지점에서**(19.9% / 87.7%) 잘려 있었다.
  즉 LFS 캐시도 신뢰할 수 없다 → `git lfs pull`로 HF(`adithyamurali/GraspGenXModel`)에서 재수신해야 하고,
  **썩은 캐시 객체를 먼저 지워야** lfs가 "이미 있음"으로 건너뛰지 않는다.
- 네 파일 권한이 전부 `-rwxrwxrwx`였다 = FAT 계열 매체(USB) 경유 흔적. **대용량 자산을 매체·PC 간
  복사한 뒤에는 `sha256sum` 검증을 습관으로 한다.**

### ✅ GraspGenX grasp 4×4 = `tool0` 목표 자세 (2026-08-05, 양쪽 URDF 대조로 확정)

계획서 §1-3 6번("조용히 틀리는 지점")의 답이다. **추가 변환이 필요 없다.**

| | 부모 → 그리퍼 base 조인트 |
|---|---|
| 우리 (`onrobot_rg2.xacro:33-35`) | `tool0 → rg2_base_link`, `xyz="0 0 0" rpy="0 0 1.57"` |
| GraspGenX (`x_grippers/onrobot_RG2/gripper.urdf:8-12`) | `world → onrobot_rg2_base_link`, `xyz="0 0 0" rpy="0 0 1.5708"` |

같은 메시(`meshes/rg2/visual/base_link.stl`), 같은 90° 요. 그리고 grasp 프레임이 `world` 쪽인 근거:
`robot.py:61` "approach는 +Z, **contact는 +X**"인데 `onrobot_rg2_base_link` 안에서는 너클이
**Y**로 벌어진다(`xyz="0 -0.007678 0.142297"`, `axis="-1 0 0"`). 90° 요가 Y→X로 돌린다.
→ **grasp 프레임 = `world` = 우리 `tool0`.** 요가 양쪽에서 상쇄되므로 그대로 쓰면 된다.

- 손끝(TCP) = grasp 원점 + **0.18 m** × grasp의 +Z축 (`config.json` `fingertip[2]`).
  **grasp 원점을 물체 위치로 읽으면 안 된다** — 접근이 30° 기울면 원점은 물체에서 9 cm 옆에 앉는다.
  2026-08-05에 이걸 "좌표 오차 7 cm"로 오진했다. 손끝으로 보면 사과 실측과 **1.1 cm**였다.
- ⚠️ **미해결: URDF가 실물보다 짧을 수 있다.** URDF는 `tool0 → rg2_base_link` 오프셋이 **0**이고
  그리퍼 bbox 최대 z가 `0.18999`(≈190 mm)인데, 사용자 확인으로 **RG2 매뉴얼은 ≈220 mm + 브라켓 10 mm**다.
  차이가 실재하면 **MoveIt이 손끝을 40 mm 더 깊이 밀어넣는다**(§6 분기 F).
  → 실기 전 **`tool0` 플랜지 면 → 손끝**을 줄자로 실측하고, 차이가 있으면 `tcp_offset_m`이 아니라
  **`onrobot_rg2.xacro`의 `origin xyz`에 어댑터 두께를 넣는다.**

### ⚠️ 단일 시점 "표면중심"은 물체 중심이 아니다 — 캘리브 오차로 오해하지 말 것 (2026-08-05)

depth 마스크 픽셀의 평균은 **보이는 표면**의 무게중심이다. 반지름 `r`인 구를 한 시점에서 보면
시선축을 따라 중심보다 카메라 쪽으로 **`2r/3`** 앞에 앉는다
(투영 원판 균등 샘플: `(1/πr²)∫₀ʳ √(r²-ρ²)·2πρ dρ = 2r/3`).

- 사과 `r=2.5 cm` → **1.7 cm**. 2026-08-05 실측에서 관측된 1.7 cm와 정확히 일치한다.
- **이 값을 근거로 hand-eye 오프셋을 보정하면 없는 오차를 만들어 넣는 것이다.**
- 캘리브를 정말 검증하려면 평면 타겟을 쓰거나, 물체라면 `Z.max()`(표면 최고점 ≈ 실제 상단)를 본다.

### `real_world` 씬 포맷 계약 — 우리 캡처가 이 규약을 따른다 (2026-08-05, loader 왕복 검증)

`scripts/capture_graspgenx_scene.py` → `demo_scene_pc.py` 경로. 합성 장면으로 GraspGenX **자체 loader**를
통과시켜 확인했다(`scripts/test_scene_roundtrip.py`, PASS).

| 파일 | 규약 |
|---|---|
| `depth.npy` | float32, **미터**, (H,W) |
| `rgb.png` / `seg.png` | 같은 해상도. seg는 정수 라벨맵 |
| `meta_data.json` | `intrinsics`(3×3) / `camera_pose`(4×4) / `label_map` / `scene_bounds` |

- `camera_pose`는 **카메라 점을 world로 보내는** 변환이다(`scene_loaders.py:86`). 여기에 tf2의
  `base_link ← camera_color_optical_frame`을 넣으면 GraspGenX의 "world"가 곧 `base_link`가 된다.
- **`obj_` 접두 라벨만 grasp 대상이 된다.** 그런데 씬 점군에는 라벨과 무관하게 유효 depth가 전부 들어간다
  → **장애물은 seg 라벨이 없어도 충돌 필터에 잡힌다**(`build_scene_pc_excluding_object`는 대상 물체
  픽셀만 뺀다). 테이블도 자동으로 장애물이다.
- `--sample_data_dir`에는 **씬의 부모 디렉토리**를 준다. `collect_scene_items`가 `meta_data.json`이 있는
  하위 디렉토리를 전부 긁는다(`:226`). 한 씬만 보려면 `--scene 00`.
- `demo_scene_pc.py`는 **씬마다 `input("Press Enter…")`로 멈춘다**(`:381`). 멈춘 게 아니라 기다리는 것.
  시각화는 RViz가 아니라 **viser** <http://localhost:8080>.
- 기본값: `grasp_threshold 0.7` / `num_grasps 200` / `collision_threshold 0.02 m` /
  `max_scene_points 8192` / `min_obj_points 100` / `filter_collisions True`.
  **8 GB VRAM이므로 `--num_grasps 64`로 시작한다**(플랜의 12 GB 가정은 이 랩탑에 안 맞다).

---

## nvblox / DDS — 대여 GPU와 무관한 보편 사실 (2026-08-04 실측, `gpu-rental-checklist.md` §8에서 이관)

> 아래 4가지는 **어느 머신에서 nvblox를 돌리든 재현되는 소프트웨어 속성**이라 여기로 옮겼다.
> 대여 절차·밟은 지뢰 목록·확정 명령어 자체는 여전히 [[ws/cobot2/plans/2026-08-04-gpu-rental-checklist]] §1~§7이 단일 출처다.

- **Fast DDS가 848×480 depth(814 KB/샘플)를 못 흘린다.** 14~15 Hz여야 할 게 0.2~0.7 Hz로 떨어진다 —
  `camera_info`(작은 메시지)는 같은 bag에서 14 Hz로 멀쩡해 **메시지 크기 의존**이 지문이다.
  `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`로 회복(15.2 Hz). `/dev/shm`·`rmem_max`·CPU는 배제됨.
  **`docker exec`로 새로 여는 셸마다 초기화된다** — 컨테이너 `~/.bashrc`에 넣어야 전 노드에 적용된다.
- **nvblox `global_frame` 기본값이 `odom`이다**(`nvblox_ros/include/nvblox_ros/node_params.hpp:45`).
  `world → base_link → camera_link`뿐인 TF엔 `odom`이 없어 depth가 조용히 전량 버려진다
  (`Lookup transform failed for frame base_link`처럼 **엉뚱한 프레임 이름이 로그에 찍혀 오진을 유발**한다 —
  실제로 없는 건 목적지 `odom` 쪽). `-p global_frame:=base_link`로 해결.
- **`ros2 bag play -l`(루프)이 TF 버퍼를 깬다.** sim time이 뒤로 점프해
  `[WARN] [tf2_buffer]: Detected jump back in time. Clearing TF buffer.` → 프레임 96%가 폐기된다.
  **루프 대신 감속**(`-r 0.25`)을 쓴다. `--clock`은 소비 노드의 `use_sim_time:=true`와 반드시 짝이다.
- **nvblox 매퍼 파라미터는 `static_mapper.`/`dynamic_mapper.` 접두사가 붙는다**(yaml이 중첩 구조라).
  `-p esdf_slice_height:=0.0`은 `cannot be set because it was not declared`로 죽는다 —
  `-p static_mapper.esdf_slice_height:=0.0`처럼 접두사를 붙여야 한다. `use_lidar`는 yaml 기본값이
  **`true`**다 — LiDAR 없으면 `-p use_lidar:=false`를 빠뜨리지 말 것.
- **`--params-file`이 실제로 실렸는지는 타이밍 표로 확인한다** — 이름이 맞아도 경로가 틀리면
  조용히 기본값으로 되돌아간다. `ros/update_esdf`가 10 Hz가 아니라 5.0, `ros/tick`이 100 Hz가
  아니라 400이면 **params-file이 안 실린 것이다**(yaml `update_esdf_rate_hz: 10.0`, `tick_period_ms: 10`).

## Isaac ROS 컨테이너 (Lightning AI / AWS EC2 GPU)

> 📤 **2026-08-04에 [[ws/cobot2/plans/2026-08-04-gpu-rental-checklist]]로 전부 옮겼다.**
> 대여 GPU(클라우드) 환경의 사실은 **실기 제약이 아니다** — 이 문서는 로컬 실물 하드웨어만 소유한다.
> Foxglove, FoundationPose 걸림돌은 그쪽 §6~§8에 있다. nvblox/DDS 보편 사실은 위 절로 옮겼다.
