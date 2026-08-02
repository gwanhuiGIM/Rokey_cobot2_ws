# 실기/현장 제약 (실측 사실만 기록. 추측 금지)

## RealSense D435I (2026-08-01)
- alias `reals` (`rs_align_depth_launch.py` + `align_depth.enable:=true enable_rgbd:=true pointcloud.enable:=true`, depth 848x480x30 / color 1280x720x30)는 **ROS_DOMAIN_ID를 지정하지 않으므로 기본값(비어있음 → 0)에서 뜬다.**
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
- `set_tool`/`set_tcp` 이름("Tool Weight_2FG", "2FG_TCP")은 티치펜던트 등록명이다. RG2용 실제 등록명으로 바꾸지 않으면 원점이 어긋난다.

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
숫자를 launch나 alias에 하드코딩하지 않는다 — 재캘리브 후엔 npy만 갈아끼운다:
```bash
cp corecode/Calibration_Tutorial/T_cam2base.npy \
   src/cobot_rg2/rg2/m0609_rg2_bringup/config/T_cam2base.npy   # symlink-install이면 rebuild 불필요
```
npy가 없으면 이 TF만 빠지고 bringup은 정상 진행한다. 값만 확인하려면:
```bash
ros2 run m0609_rg2_bringup calib_npy_to_tf.py corecode/Calibration_Tutorial/T_cam2base.npy base_link camera_link
```
**하드코딩된 static_transform_publisher 명령을 다시 만들지 말 것** — npy가 갱신돼도 그 숫자는 안 따라온다.
실제로 2026-08-02에 붙여쓰던 명령이 갱신 전 캘리브 값이라 340 mm 어긋나 있었다.
- ~~평행이동 993.4 mm~~ → **폐기(좌표 규약 버그 수정 전 값).** **유효값은 약 1.48 m**
  (`Translation: [1.148, 0.640, 0.678]`, 사용자 확인 2026-08-02). 회전행렬 직교성 검사 통과,
  `tf2_echo base_link camera_depth_optical_frame` 정상 동작 확인.
- **아직 미검증**: `T_cam2base`의 방향(parent가 base_link가 맞는지). `eye2hand_calibration.py:305`가 AX=XB의 X를 그대로 저장하는데, 코드 명명 관행(`gripper2base` = base 좌표계의 gripper pose)상 parent=base_link로 추정. RViz에서 포인트클라우드가 엉뚱한 곳/뒤집혀 뜨면 `np.linalg.inv(T)`가 답이다. **부호를 만지지 말 것.**

## realsense-viewer와 ROS 노드는 동시에 못 쓴다 (2026-08-02, 실측)

`realsense-viewer`가 USB 디바이스를 **독점**한다. 뷰어를 켜두면 `realsense2_camera` 노드가 죽거나
프레임을 못 받고, `/camera/*` 토픽이 통째로 사라진다. 증상이 "TF 프레임 없음"으로 나타나서
캘리브 문제로 오진하기 쉽다. **뷰어를 먼저 닫고 노드를 띄운다.**

## octomap_server — 이 랩탑 리소스로는 기본 설정이 안 돌아간다 (2026-08-02)

측정한 실제 하드웨어:
- CPU **Intel i7-10510U** — 4코어/8스레드, 1.8GHz base, **15W 노트북 U-시리즈**
- GPU **없음**. Intel UHD 내장(CometLake-U GT2)뿐 — `nvidia-smi` 미설치, `lspci`에 외장 GPU 없음
- RAM 15 GB
- **상시 부하: `ros2_control_node` 204% (= 2코어 점유)**, rviz2 ~12%, joint_state_publisher ~10%

> ⚠️ 이 사실은 스프린트 계획의 **nvblox/FoundationPose/GraspGenX 전제(CUDA GPU)를 정면으로 깬다.**
> 계획서 1절의 "GPU: x86_64 + NVIDIA GPU(CUDA)" 가정은 이 랩탑에서 **거짓**이다.

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
`max_range:=1.5`는 카메라가 base에서 993mm 떨어져 있음을 근거로 한 값. 테이블 먼 쪽이 잘리면 2.0으로.
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

### 캘리브 npy의 정본은 `corecode/` 쪽이다 (2026-08-02 사용자 결정)
`T_cam2base.npy`가 `corecode/Calibration_Tutorial/`와 `m0609_rg2_bringup/config/` 두 곳에 있다.
후자는 내가 `camera.launch.py`를 만들면서 수동 `cp`로 만든 사본이고, **정본이 아니다.**

- **정본**: `corecode/Calibration_Tutorial/T_cam2base.npy` — 재캘리브 시 결과가 나오는 위치가 여기로 고정돼 있다.
- 로봇 bringup 패키지에는 결국 캘리브 결과가 들어가지 않을 예정 → `m0609_rg2_bringup/config/` 사본은 **삭제 대상**.
- 지우기 전에 `camera.launch.py`가 corecode 경로를 직접 읽도록 먼저 바꿔야 한다.
  순서를 반대로 하면 `base_link→camera_link` static TF가 통째로 사라진다(npy 없으면 TF 노드가 빠지는 설계).
- ⚠️ 그 전까지는 **재캘리브 후 `cp`를 잊으면 낡은 값으로 발행된다.** 340 mm 어긋난 전례가 이 구조다.

### 설정 — **실제 값은 파일이 단일 출처다**
`src/cobot_rg2/rg2/m0609_rg2_moveit/config/sensors_3d.yaml` (2026-08-02 작성).
여기 복붙해 두면 갈라지므로 값은 옮겨 적지 않는다. 파일에 `[튜닝]` 주석으로 손잡이를 표시해 뒀다.

계획 단계의 값과 **실제 채택값이 다르다** — 아래가 채택된 쪽이다:

| 항목 | 계획(폐기) | 실제 |
|---|---|---|
| 센서명 | `default_sensor` | `realsense_pointcloud` |
| `point_cloud_topic` | `/camera/camera/depth/points_xyz` | `/camera/camera/depth/color/points` (RealSense가 직접 발행 → `depth_image_proc` 불필요) |
| `max_range` | 1.5 | 2.5 (카메라~로봇 약 1.48 m) |
| `padding_scale` | 1.2 | 1.0 |
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
