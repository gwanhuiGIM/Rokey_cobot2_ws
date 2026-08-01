# Sprint Plan: M0609 Perception-Guided 6DoF 모션 제어 (nvblox + TAMP-lite)

**기간:** Day 1 – Day 5 (1주, 집중 스프린트) | **팀:** 1인 (본인)
**환경:** ROS 2 Humble / Doosan M0609 (6축) / RealSense D435i (eye-to-hand, 고정) / Logitech C270 (eye-in-hand, 플랜지 부착) / VoiceProcess

**스프린트 목표:**
> D435i(고정, 전역 3D 재구성)로 nvblox 충돌 회피맵을 만들어 MoveIt2 모션에 연결하고, C270(팔 부착, 근접 시야)은 카메라 pose(FK+hand-eye) 기반 ray-plane intersection으로 그립 직전 물체 위치를 보정하는 2단계(coarse-to-fine) 파이프라인의 최소 동작 버전을 M0609 실기에서 검증한다.

---

## 0. 두 카메라의 역할 분담 (확정)

| 카메라 | 마운트 | 역할 | 원리 | 한계 |
|---|---|---|---|---|
| **D435i** | eye-to-hand (고정) | 전역 3D 재구성 → nvblox 충돌 회피맵 | GPU 가속 TSDF/ESDF 누적 | 팔이 최종 접근 시 self-occlusion 발생 |
| **C270** | eye-in-hand (플랜지 부착) | 근접 물체 검출, 그립 직전 미세 정렬 | **camera pose(FK × hand-eye offset) 기반 ray-plane intersection** — 물체 크기 가정 없이, 알려진 테이블 평면과 광선의 교차점으로 3D 위치 산출 | 평면(단일 레이어) 가정 필요. 물체 높이가 제각각이면 오차 발생 → 이 경우 2뷰 삼각측량 또는 D435i로 대체 필요 |

**핵심 근거:** C270이 팔에 고정되어 있으므로 매 순간 `camera_optical_frame → base_0` pose를 `easy_handeye2`로 구한 고정 오프셋과 그 순간의 순기구학(FK)의 곱으로 정확히 알 수 있다. 따라서 픽셀→3D 역투영 시 "물체 크기를 안다"는 약한 가정 대신 "카메라가 어디서 찍었는지 안다"는 강한 정보를 사용해 광선을 알려진 z=테이블높이 평면과 교차시키는 방식(ray-plane intersection)을 쓴다.

---

## 1. 가정 및 확인 필요 사항

| 항목 | 가정 | 비고 |
|---|---|---|
| D435i 마운트 | 고정형(eye-to-hand), 작업공간 내려다보는 배치 | Day1 확정 |
| C270 마운트 | M0609 플랜지 부착(eye-in-hand) | Day1 확정, 그리퍼와 간섭 없는 위치 선정 |
| 픽업 대상 | 테이블 위 단일 레이어(높이 균일) | 다층/적재 시 ray-plane 가정 깨짐 → 별도 대응 필요 |
| GPU | x86_64 + NVIDIA GPU(CUDA), nvidia-docker 런타임 설치됨 | nvblox core는 CUDA 필수 |
| VoiceProcess | 음성 명령 → 문자열 토픽 발행 가능 | Day4 어댑터로 흡수 |
| M0609 패키지 | `doosan-robotics/doosan_robot2` 기반 MoveIt2 설정 완료 | 본인 확인 사항 반영 |

---

## 2. 데일리 백로그 (터미널 명령어 포함)

### Day1 — 카메라 파이프라인 구성

**P0. D435i → isaac_ros_nvblox 파이프라인 구성**

> ⚠️ **버전 고정 필수:** 최신 `release-4.4`는 Docker dev container 기능이 Isaac ROS CLI로 이전되었고 사실상 Jazzy 중심으로 재편되어 `run_dev.sh`가 없다. Humble 환경을 유지하려면 `run_dev.sh`가 살아있는 **`release-3.2`** 태그로 관련 저장소를 모두 통일해서 클론한다.

```bash
mkdir -p ~/workspaces/isaac_ros-dev/src
cd ~/workspaces/isaac_ros-dev/src
export ISAAC_ROS_WS=~/workspaces/isaac_ros-dev

# 버전 고정: isaac_ros_common, isaac_ros_nvblox 모두 release-3.2로 통일
git clone -b release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git isaac_ros_common
git clone -b release-3.2 --recursive https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox.git isaac_ros_nvblox
git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-master

# RealSense 지원 이미지 키 설정
cd ${ISAAC_ROS_WS}/src/isaac_ros_common/scripts
touch .isaac_ros_common-config
echo CONFIG_IMAGE_KEY=ros2_humble.realsense > .isaac_ros_common-config

cd ${ISAAC_ROS_WS}/src/isaac_ros_common
./scripts/run_dev.sh ${ISAAC_ROS_WS}   # Isaac ROS dev docker 컨테이너 진입 (nvidia-docker runtime 필요)

# 컨테이너 내부
sudo apt update && rosdep update
rosdep install -i -r --from-paths src --rosdistro humble -y
colcon build --symlink-install --packages-up-to isaac_ros_nvblox realsense2_camera
source install/setup.bash

# 실행 (별도 터미널들, 모두 컨테이너 attach 후)
ros2 launch realsense2_camera rs_launch.py \
  enable_depth:=true enable_color:=true align_depth.enable:=true \
  camera_name:=d435i pointcloud.enable:=true

ros2 launch isaac_ros_nvblox isaac_ros_nvblox.launch.py
rviz2   # nvblox mesh/voxel 토픽 추가해 재구성 확인 (이번 스프린트에서는 시각화·향후 확장용으로만 사용)
```
**DoD:** RViz에서 3D mesh/voxel 재구성 실시간 확인, `tf2_ros` 트리에 `camera_link(d435i) → base_0` 정상 표시.

**P1. C270 웹캠 노드 등록**
```bash
sudo apt install ros-humble-usb-cam
# 또는
sudo apt install ros-humble-v4l2-camera

v4l2-ctl --list-devices    # C270 장치 노드(/dev/videoX) 확인, D435i와 충돌 없는지 체크
ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -p video_device:="/dev/video2" \
  -p image_size:="[1280,720]" \
  -r image_raw:=/webcam/image_raw

ros2 topic hz /webcam/image_raw   # 프레임레이트 확인, D435i 동시 구동 시 드랍 여부 체크
```
**DoD:** `/webcam/image_raw`, `/webcam/camera_info` 정상 퍼블리시, USB 대역폭 분리 확인(D435i·C270 각각 다른 USB 컨트롤러 포트에 연결 권장).

---

### Day2 — 이중 캘리브레이션 + PlanningScene 연동

**P0. D435i eye-to-hand 캘리브레이션**
```bash
sudo apt install ros-humble-easy-handeye2
git clone https://github.com/marcoesposito1988/easy_handeye2.git src/easy_handeye2  # humble 브랜치 확인
colcon build --packages-select easy_handeye2 && source install/setup.bash

# 캘리브레이션 타겟(체커보드/AprilTag)을 M0609 플랜지에 부착 후 여러 자세로 이동하며 수집
ros2 launch easy_handeye2 calibrate.launch.py \
  calibration_type:=eye_on_base \
  tracking_base_frame:=camera_link \
  tracking_marker_frame:=calib_tag \
  robot_base_frame:=base_0 \
  robot_effector_frame:=flange

# 결과 저장된 정적 TF publish 확인
ros2 run tf2_ros tf2_echo base_0 camera_link
```
**DoD:** 알려진 좌표 물체 배치 후 D435i가 인식한 3D 위치와 실측값 오차 < 1cm.

**P0. C270 eye-in-hand 캘리브레이션 (flange 오프셋)**
```bash
# 캘리브레이션 타겟은 이번엔 작업공간에 고정, 로봇(C270)이 여러 자세로 움직이며 수집
ros2 launch easy_handeye2 calibrate.launch.py \
  calibration_type:=eye_in_hand \
  tracking_base_frame:=camera_link_webcam \
  tracking_marker_frame:=calib_tag_fixed \
  robot_base_frame:=base_0 \
  robot_effector_frame:=flange

ros2 run tf2_ros tf2_echo flange camera_link_webcam   # 고정 오프셋 확인
```
**DoD:** `flange → camera_link_webcam` 정적 TF 확보. 이후 실시간 `camera_link_webcam → base_0`는 FK(현재 조인트 상태)로 자동 계산됨을 `tf2_echo base_0 camera_link_webcam`로 확인.

**P0. 실제 3D 포인트클라우드 → MoveIt2 충돌 회피 연동 (선택 A: cuMotion 제외, 표준 Humble 조합)**

> ⚠️ **방향 전환:** cuMotion(`isaac_ros_cumotion_moveit`)은 현재 사실상 Jazzy 전용으로 재편되어 Humble 지원이 불확실하다. M0609 MoveIt2 스택 전체가 Humble 기반이므로, 이번 스프린트는 cuMotion 없이 **수년간 검증된 표준 조합**(`depth_image_proc` → `octomap_server` → MoveIt2 OMPL)으로 충돌 회피를 구현한다. nvblox(release-3.2)는 예쁜 3D 시각화·향후 확장용으로만 병행 실행하고, 실제 충돌 회피 판단에는 관여시키지 않는다.

```bash
sudo apt install ros-humble-depth-image-proc ros-humble-octomap-server ros-humble-moveit-msgs

# D435i 정렬된 depth 이미지 → 3D 포인트클라우드 변환
ros2 run depth_image_proc point_cloud_xyz_node --ros-args \
  -r image_rect:=/d435i/depth/image_rect_raw \
  -r camera_info:=/d435i/depth/camera_info \
  -r points:=/d435i/depth/points_xyz

# 포인트클라우드를 octomap_server에 연결해 3D occupancy map 생성
ros2 run octomap_server octomap_server_node --ros-args \
  -r cloud_in:=/d435i/depth/points_xyz \
  -p frame_id:=base_0 \
  -p resolution:=0.02

# MoveIt2 move_group 설정(sensors_3d.yaml)에 PointCloudOctomapUpdater 플러그인 등록
#   point_cloud_topic: /d435i/depth/points_xyz  (octomap_server 없이 MoveIt이 직접 구독하는 방식도 가능)
ros2 launch <m0609_moveit_config> move_group.launch.py

rviz2   # MoveIt 플러그인에서 Octomap 충돌 지오메트리 확인
```
**DoD:** RViz MoveIt 플러그인에서 실제 depth 기반 3D 충돌 지오메트리(Octomap)가 반영되고, OMPL이 이를 반영해 계획된 궤적이 장애물을 회피함.
**비고:** MoveIt2의 `occupancy_map_monitor/PointCloudOctomapUpdater` 플러그인을 쓰면 별도 `octomap_server` 노드 없이도 MoveIt이 포인트클라우드를 직접 구독해 내부 Octomap을 구성할 수 있다 — 리소스를 아끼고 싶으면 이 방식으로 단순화 가능(Day1 저녁 스파이크로 둘 중 하나 확정).

---

### Day3 — 충돌 회피 모션 실기 검증 및 튜닝

**P0. 장애물 회피 실기 테스트**
```bash
ros2 launch <m0609_moveit_config> demo.launch.py   # 또는 실기 드라이버 launch
ros2 run moveit_commander moveit_commander_cmdline.py   # 대화형 목표 pose 테스트(선택)

# 속도 제한 필수 — 실기 안전
ros2 param set /move_group velocity_scaling_factor 0.2
ros2 param set /move_group acceleration_scaling_factor 0.2
```
**DoD:** 임의 배치 장애물 3종 각각에서 충돌 없이 목표 pose 도달, 실패 시 재계획 로그 확보.

**P1. 플래너 튜닝 비교**
```bash
# ompl_planning.yaml에서 planner_id 교체하며 비교: RRTConnect, RRTstar, LBKPIECE 등
ros2 param get /move_group planning_pipeline
# 각 플래너별 성공률/평균 계획시간 기록 (rosbag으로 planning_scene, trajectory 기록)
ros2 bag record -o day3_tuning /move_group/monitored_planning_scene /joint_states
```
**DoD:** 튜닝 로그 표(성공률, 평균 계획 시간) 작성.

---

### Day4 — TAMP-lite 상태머신 + ray-plane intersection + 음성 트리거

**P0. C270 기반 ray-plane intersection 노드**
```bash
# 신규 패키지 생성
ros2 pkg create --build-type ament_python webcam_pose_estimator \
  --dependencies rclpy sensor_msgs tf2_ros tf2_geometry_msgs image_geometry

# 노드 내부 로직 개요 (Python):
# 1) image_geometry.PinholeCameraModel로 /webcam/camera_info의 intrinsic 로드
# 2) 검출된 픽셀 (u,v) → projectPixelTo3dRay()로 카메라 좌표계 광선 생성
# 3) tf2_ros.Buffer.lookup_transform('base_0', 'camera_link_webcam', now)로 현재 카메라 pose 획득
# 4) 광선을 base_0로 변환 후 z=table_height 평면과 교차 계산 → 3D 물체 위치

colcon build --packages-select webcam_pose_estimator
source install/setup.bash
ros2 run webcam_pose_estimator ray_plane_node --ros-args -p table_height:=0.0
```
**DoD:** 테이블 위 물체 1종에 대해 ray-plane intersection 결과와 실측 좌표 오차 확인(허용 오차 사전 정의, 예 <5mm).

**P0. TAMP-lite 상태머신 (인식→서브골→모션)**
```bash
ros2 pkg create --build-type ament_python tamp_lite_statemachine \
  --dependencies rclpy moveit_msgs geometry_msgs std_msgs

# 상태: IDLE → APPROACH(D435i/nvblox 회피 경로) → ALIGN(C270 ray-plane 보정)
#      → GRASP → LIFT → DONE/FAIL
ros2 run tamp_lite_statemachine sm_node
ros2 topic pub /tamp/start std_msgs/msg/Empty "{}"   # 수동 트리거 테스트
```
**DoD:** 상태머신 노드 1개, 물체 1종 대상 pick 서브골 시퀀스가 로그로 확인됨.

**P1. VoiceProcess 트리거 어댑터**
```bash
# VoiceProcess가 발행하는 실제 토픽/메시지 타입 확인 후 매핑
ros2 topic list | grep -i voice
ros2 topic echo /voice/command   # 예시, 실제 토픽명 확인 필요

# 어댑터 노드: /voice/command(String) 수신 시 특정 키워드 매칭 → /tamp/start 발행
ros2 run tamp_lite_statemachine voice_trigger_adapter
```
**DoD:** 음성 명령 1개("잡아" 등)로 파이프라인 트리거 확인.

---

### Day5 — 통합 검증, 회고, 백로그

**P0. 통합 반복 시행**
```bash
# 전체 파이프라인 동시 기동 (launch 파일로 통합 권장)
ros2 launch tamp_lite_statemachine full_pipeline.launch.py

# 반복 시행 로그 기록
ros2 bag record -o day5_integration_test -a
```
**DoD:** 10회 반복 시행, 성공률·실패 유형(인식 오탐/충돌맵 노이즈/IK 실패/ray-plane 오차) 분류 기록.

**P1. 회고 및 다음 스프린트 백로그 정리** — 문서 작업, 명령어 없음.

**P2(Stretch). 실패 케이스 rosbag 보관**
```bash
mkdir -p ~/failure_cases
ros2 bag record -o ~/failure_cases/case_$(date +%s) -a
```

---

## 3. 리스크 (업데이트)

| 리스크 | 영향 | 완화책 |
|---|---|---|
| isaac_ros_common/nvblox 버전 불일치 (release-3.2 vs 최신 태그 혼용) | 빌드 실패, Day1~2 지연 | 모든 Isaac ROS 저장소를 release-3.2로 통일해서 클론, 다른 릴리스 태그와 섞지 않기. 빌드 에러 시 `.isaac_ros_common-config`의 이미지 키와 태그 조합 재확인 |
| MoveIt2 sensors_3d.yaml 플러그인 설정 경험 부재로 Day2 지연 | Day2 지연 | Day1 저녁에 `depth_image_proc`+`octomap_server` 조합을 M0609 없이 데스크탑에서 먼저 단독 테스트해 토픽 흐름부터 검증. 막히면 임시로 RViz에 수동 박스 충돌 오브젝트만 넣고 진행, 실제 depth 연동은 Day3로 이월 |
| eye-to-hand(D435i) 캘리브레이션 오차 누적 | 충돌 회피 정확도 저하 | 알려진 좌표 물체로 오차 측정 후 진행, 1cm 초과 시 Day3 보류 |
| eye-in-hand(C270) hand-eye 오프셋 오차 | ray-plane intersection 오차로 그립 실패 | Day2에 별도 검증, `flange→camera_link_webcam` 오프셋을 여러 로봇 자세에서 반복 측정해 일관성 확인 |
| **ray-plane intersection의 평면 가정 위반** (물체 높이 불균일) | 물체 위치 오차, 그립 실패 | 이번 스프린트는 단일 레이어로 스코프 한정. 다층 시 D435i 근접 depth로 대체하거나 2뷰 삼각측량으로 확장(다음 스프린트) |
| D435i 최종 접근 시 self-occlusion (팔이 시야 가림) | 그립 직전 depth 데이터 끊김 | 바로 이 구간을 C270이 커버하도록 설계됨 — Day4에서 ALIGN 상태 진입 조건(팔이 근접했을 때)을 명확히 정의 |
| VoiceProcess 인터페이스 사양 불명확 | Day4 P1 지연 | P1로 낮춰뒀으므로 지연 시 이월, 최악의 경우 키보드 트리거로 대체 |
| D435i + C270 동시 구동 시 USB 대역폭 이슈 | 인식 프레임 드랍 | Day1에 별도 USB 컨트롤러 분리 연결, 필요 시 C270 저해상도 다운그레이드 |
| 실기 충돌/안전 | 하드웨어 손상, 안전사고 | Day3부터 속도 스케일링 20~30% 제한, protective stop 여유 확보, 비상정지 상시 대기 |

---

## 4. Definition of Done (스프린트 전체)

- [ ] nvblox 3D 재구성이 RViz에서 실시간 확인됨 (D435i, eye-to-hand)
- [ ] D435i depth 기반 실제 3D 포인트클라우드가 MoveIt2 PlanningScene의 Octomap 충돌 지오메트리로 반영됨 (nvblox는 시각화 용도로만 병행)
- [ ] 임의 배치 장애물에 대해 M0609가 실기에서 충돌 없이 회피 경로로 도달
- [ ] C270 eye-in-hand 캘리브레이션 완료, ray-plane intersection으로 테이블 위 물체 위치 오차가 허용 범위 이내
- [ ] 인식(D435i coarse + C270 fine) → 서브골 → 모션 실행 상태머신이 실기에서 최소 1개 물체에 대해 동작
- [ ] 10회 반복 시행 결과와 실패 유형이 기록됨
- [ ] 다음 스프린트 백로그(다층 대응, TAMP 확장, GPU 가속 등) 문서화 완료

## 5. Key Dates

| Day | 이벤트 |
|---|---|
| Day1 | 스프린트 시작, D435i/C270 파이프라인 구성 |
| Day2 | 이중 캘리브레이션 + PlanningScene 연동 (최대 난관 구간) |
| Day3 | 충돌 회피 실기 검증 및 튜닝 |
| Day4 | ray-plane intersection + TAMP-lite 상태머신 + 음성 트리거 |
| Day5 | 통합 검증, 회고, 차기 백로그 |

---

## 6. 다음 스프린트 후보 (이번엔 스코프 아웃)

- 다층/적재 상황 대응: ray-plane 단일 평면 가정을 2뷰 삼각측량 또는 D435i 근접 depth로 보완
- cuTAMP 스타일 GPU 배치 최적화 (skeleton search + 배치 IK/충돌/안정성 제약 만족)
- VLM 기반 자연어 지시 → 서브골 자동 생성 (OWL-TAMP/VLM-TAMP 개념)
- 실패 사례 기반 skill-effect 모델 재학습 (Fail2Progress 개념 본격 적용)
- People/dynamic reconstruction 모드로 nvblox 전환 (협동로봇 협업 시나리오 대비)
- ROS 2 Jazzy 전체 마이그레이션 후 cuMotion(`isaac_ros_cumotion_moveit`) 재도전 — GPU 가속 충돌 회피로 전환

> **참고:** 위 명령어들은 패키지/저장소 버전에 따라 인자명이나 launch 파일명이 다를 수 있습니다(특히 `doosan_robot2`, `easy_handeye2`, VoiceProcess 인터페이스는 본인 환경의 실제 저장소 문서로 재확인 필요). 실행 전 각 저장소 README의 Humble 대응 브랜치를 먼저 확인하세요.
>
> **버전 정책:** 이번 스프린트는 Humble 유지를 위해 Isaac ROS 관련 저장소를 모두 `release-3.2` 태그로 고정한다. 최신 `release-4.x`는 Docker dev container 기능이 Isaac ROS CLI로 이전되었고 cuMotion 등 신규 패키지가 사실상 Jazzy 중심으로 재편되어, 이번 스코프(Humble/M0609)에서는 사용하지 않는다.
