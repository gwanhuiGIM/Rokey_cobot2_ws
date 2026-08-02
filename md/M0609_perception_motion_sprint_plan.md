# Sprint Plan: M0609 Perception-Guided 6DoF 모션 제어 (nvblox + TAMP-lite)

**기간:** Day 1 – Day 5 (1주, 집중 스프린트) | **팀:** 1인 (본인)
**환경:** ROS 2 Humble / Doosan M0609 (6축) / RealSense D435i (eye-to-hand, 고정) / Logitech C270 (eye-in-hand, 플랜지 부착) / OnRobot RG2 그리퍼 / VoiceProcess

**스프린트 목표:**
> D435i(고정, 전역 3D 재구성)로 depth 기반 Octomap 충돌 회피맵을 만들어 MoveIt2 모션에 연결하고, 물체 인식은 **FoundationPose**(6D pose 추정, ray-plane intersection 대체)로, 그립 지점 생성은 **GraspGenX**(RG2 대응, 재학습 없는 cross-embodiment 그립 생성)로 수행하는 인식→그립→모션 파이프라인의 최소 동작 버전을 M0609 실기에서 검증한다.

> **채택 결정(개발 착수 전 반영):** 원래 계획했던 C270 기반 ray-plane intersection과 하드코딩 그립 지점 방식을 각각 FoundationPose, GraspGenX로 대체한다. C270은 이제 "근접 확인/보조" 역할로 축소되고, 메인 물체 인식은 D435i RGB-D + FoundationPose가 담당한다.

---

## 0. 두 카메라의 역할 분담 (확정)

| 카메라 | 마운트 | 역할 | 원리 | 한계 |
|---|---|---|---|---|
| **D435i** | eye-to-hand (고정) | 전역 3D 재구성 → nvblox 충돌 회피맵 | GPU 가속 TSDF/ESDF 누적 | 팔이 최종 접근 시 self-occlusion 발생 |
| **D435i** | (위와 동일) | **물체 6D pose 추정 (FoundationPose)** | RGB-D + 첫 프레임 세그멘테이션 마스크 → CAD 모델 있으면 model-based, 없으면 model-free(참조 이미지 몇 장)로 6D pose 실시간 추적. 평면/단일 레이어 가정 불필요 | 세그멘테이션 마스크 준비 필요(간단한 색상 기반 또는 수동 박스로 대체 가능) |
| **C270** | eye-in-hand (플랜지 부착) | 근접 확인·보조 (기존 ray-plane 방식은 폐기, 보류) | 그립 직전 시각적 확인용 보조 카메라로 역할 축소 | 이번 스프린트에서는 메인 파이프라인에서 제외, Day4 스코프 아님 |

**핵심 변경 근거:** ray-plane intersection은 물체가 평평한 단일 레이어 위에 있다는 가정이 필요한 임시방편이었다. FoundationPose는 이 가정 없이 D435i RGB-D만으로 실제 6D pose(위치+회전)를 추정하므로, 개발 착수 전 시점에 처음부터 이 방식으로 설계하는 게 재작업을 줄인다. C270은 hand-eye 캘리브레이션 인프라(Day2)는 유지하되, 메인 인식 경로에서는 빠지고 향후 그립 직전 근접 확인용으로만 남긴다.

**그립 생성:** RG2(2핑거 병렬 그리퍼)는 원래 GraspGen의 사전학습 그리퍼 3종(Franka, Robotiq 2F-140, 흡착)에 포함되지 않는다. 대신 **GraspGenX**(cross-embodiment, 그리퍼 URDF만으로 재학습 없이 그립 생성)를 사용한다.

---

## 1. 가정 및 확인 필요 사항

| 항목 | 가정 | 비고 |
|---|---|---|
| D435i 마운트 | 고정형(eye-to-hand), 작업공간 내려다보는 배치 | Day1 확정 |
| C270 마운트 | M0609 플랜지 부착(eye-in-hand) | Day1 확정, 그리퍼와 간섭 없는 위치 선정 |
| 픽업 대상 | 단일 레이어 가정 불필요(FoundationPose 채택으로 완화) | 다층/적재도 원칙적으로 가능하나 이번 스프린트는 단일 물체로 검증 범위 한정 |
| 그리퍼 | OnRobot RG2 (2핑거 병렬, 최대 스트로크 110mm) | GraspGenX 미지원 시 원래 GraspGen의 Robotiq 2F-140 체크포인트를 폭 오프셋 보정해 임시 대체 |
| GPU | x86_64 + NVIDIA GPU(CUDA), nvidia-docker 런타임 설치됨 | nvblox core는 CUDA 필수 |
| VoiceProcess | 음성 명령 → 문자열 토픽 발행 가능 | Day4 어댑터로 흡수 |
| M0609 패키지 | `doosan-robotics/doosan_robot2` 기반 MoveIt2 설정 완료 | 본인 확인 사항 반영 |

---

## 1.5 빠른 초안 검증 경로 (정확도 튜닝 이전, 최소 동작만 확인)

> Day1~3 전체를 순서대로 다 하지 않고, "장애물을 놓으면 M0609가 궤적을 바꾼다"는 것만 빠르게 보여주고 싶을 때 쓰는 압축 경로. 여기서는 **정밀 캘리브레이션(easy_handeye2), C270, ray-plane, TAMP-lite, 플래너 튜닝을 전부 생략**하고, 카메라-로봇 TF는 줄자/CAD 기반 대략값으로 대체한다. 정확도가 필요해지면 그때 Day2의 정식 `easy_handeye2` 캘리브레이션으로 교체.

**Step 1. RealSense 실행 + depth → 포인트클라우드**
```bash
ros2 launch realsense2_camera rs_launch.py \
  enable_depth:=true enable_color:=true align_depth.enable:=true \
  camera_name:=camera pointcloud.enable:=false   # 여기선 depth_image_proc이 변환 담당
# align_depth는 point_cloud_xyz_node가 raw depth를 쓰므로 이 경로에선 불필요. RViz 확인용으로만 켠다.

ros2 run depth_image_proc point_cloud_xyz_node --ros-args \
  -r image_rect:=/camera/camera/depth/image_rect_raw \
  -r camera_info:=/camera/camera/depth/camera_info \
  -r points:=/camera/camera/depth/points_xyz
```

**Step 2. 카메라→로봇 TF, 대략값으로 임시 발행 (정밀 캘리브레이션 생략)**
```bash
# 줄자/CAD로 잰 대략적인 x y z (m) + roll pitch yaw (rad) 를 base_0 기준으로 입력
ros2 run tf2_ros static_transform_publisher \
  --x 0.0 --y -0.5 --z 0.6 --roll 0 --pitch 0.6 --yaw 1.57 \
  --frame-id base_0 --child-frame-id camera_link
```
**주의:** 이 TF는 "임시"임을 명확히 표시해두고, 정밀도 튜닝 단계(Day2)에서 반드시 `easy_handeye2` 결과로 교체할 것.

> ⚠️ **rosbag 녹화 중에는 이 임시 TF를 띄우지 않는다.** bag의 `/tf_static`에 가짜 값이 박히면 나중에 진짜 캘리브 값과 충돌해 bag을 못 쓰게 된다. 출근 후 순서는 **rosbag 녹화 → 캘리브 → Day1.5**이며, 상세 절차는 `md/state.md`의 "출근 후 D435i 세션" 절이 기준이다.

**Step 3. Octomap → MoveIt2 PlanningScene 연동**
```bash
sudo apt install ros-humble-octomap-server
ros2 run octomap_server octomap_server_node --ros-args \
  -r cloud_in:=/camera/camera/depth/points_xyz \
  -p frame_id:=base_0 -p resolution:=0.02

ros2 launch <m0609_moveit_config> demo.launch.py   # 실기 대신 fake execution으로 먼저
rviz2   # MoveIt 플러그인에서 Octomap 충돌 지오메트리 확인
```

**Step 4. 궤적 확인 (fake execution)**
- 카메라 앞에 박스 등 장애물을 놓고 RViz MoveIt 플러그인에서 임의 목표 pose로 드래그 → Plan
- 궤적이 장애물을 피해가면 성공. 이 단계는 아직 실기를 움직이지 않음.

**Step 5. (확인되면) 실기로 전환**
```bash
ros2 param set /move_group velocity_scaling_factor 0.2
ros2 param set /move_group acceleration_scaling_factor 0.2
ros2 launch <m0609_moveit_config> move_group.launch.py   # 실기 드라이버 포함 launch
```
그리퍼 없이, 여유 공간 크게 두고 저속으로 먼저 확인.

**DoD (빠른 초안):** 장애물 유무에 따라 계획된 궤적이 달라지는 것을 RViz(fake) 및 실기(저속)에서 육안 확인. 좌표/그립 정밀도는 이 단계의 목표가 아님 — Day2 정식 캘리브레이션에서 다룸.

---

## 2. 데일리 백로그 (터미널 명령어 포함)

### Day1 — 카메라 파이프라인 구성

**P0. D435i → isaac_ros_nvblox 파이프라인 구성**

> ⚠️ **버전 고정 필수:** 최신 `release-4.4`는 Docker dev container 기능이 Isaac ROS CLI로 이전되었고 사실상 Jazzy 중심으로 재편되어 `run_dev.sh`가 없다. Humble 환경을 유지하려면 `run_dev.sh`가 살아있는 **`release-3.2`** 태그로 관련 저장소를 모두 통일해서 클론한다.

> 아래 블록은 `scripts/setup_isaac_ros.sh`로 스크립트화되어 있다. 실제 실행은 스크립트를 쓰고, 이 블록은 설명용으로만 본다 (둘이 어긋나면 스크립트가 기준).

```bash
mkdir -p ~/workspaces/isaac_ros-dev/src
cd ~/workspaces/isaac_ros-dev/src
export ISAAC_ROS_WS=~/workspaces/isaac_ros-dev

# 버전 고정: isaac_ros_common, isaac_ros_nvblox 모두 release-3.2로 통일
git clone -b release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git isaac_ros_common
git clone -b release-3.2 --recursive https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox.git isaac_ros_nvblox
# realsense-ros는 클론하지 않는다 — apt ros-humble-realsense2-camera 4.58.2 설치 확인됨 (2026-08-01)

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
  -r image_rect:=/camera/camera/depth/image_rect_raw \
  -r camera_info:=/camera/camera/depth/camera_info \
  -r points:=/camera/camera/depth/points_xyz

# 포인트클라우드를 octomap_server에 연결해 3D occupancy map 생성
ros2 run octomap_server octomap_server_node --ros-args \
  -r cloud_in:=/camera/camera/depth/points_xyz \
  -p frame_id:=base_0 \
  -p resolution:=0.02

# MoveIt2 move_group 설정(sensors_3d.yaml)에 PointCloudOctomapUpdater 플러그인 등록
#   point_cloud_topic: /camera/camera/depth/points_xyz  (octomap_server 없이 MoveIt이 직접 구독하는 방식도 가능)
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

### Day4 — TAMP-lite 상태머신 + FoundationPose + GraspGenX + 음성 트리거

**P0. FoundationPose 기반 6D 물체 pose 추정 (ray-plane intersection 대체)**
```bash
# 버전 정책 준수: release-3.2 계열로 통일 (isaac_ros_common과 동일 태그 확인 필수)
git clone -b release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_pose_estimation.git isaac_ros_pose_estimation
# ⚠️ release-3.2에 FoundationPose 패키지가 없으면 해당 시점의 최신 Humble 호환 태그로 대체 확인 필요

cd ${ISAAC_ROS_WS}/src/isaac_ros_common
./scripts/run_dev.sh ${ISAAC_ROS_WS}

# 컨테이너 내부
cd ${ISAAC_ROS_WS}
rosdep install -i -r --from-paths src --rosdistro humble -y
colcon build --symlink-install --packages-up-to isaac_ros_foundationpose
source install/setup.bash

# 첫 프레임 세그멘테이션 마스크 필요 (간단한 색상 기반 마스크 노드로 임시 대체 가능)
# CAD 모델 있으면 model-based, 없으면 참조 이미지 몇 장으로 model-free 실행
ros2 launch isaac_ros_foundationpose isaac_ros_foundationpose.launch.py \
  input_depth_topic:=/camera/camera/depth/image_rect_raw \
  input_rgb_topic:=/camera/camera/color/image_raw
```
**DoD:** 물체 1종에 대해 FoundationPose가 추정한 6D pose와 실측 좌표 오차 확인(허용 오차 사전 정의, 예 <5mm, 회전 오차도 함께 기록).

**P0. GraspGenX 기반 RG2 그립 지점 생성**

> ⚠️ **UNVERIFIED:** 아래 URL·스크립트명·인자는 실재 확인이 안 된 상태다(`NVlabs/GraspGen`은 확인, `GraspGenX`는 미확인). Day0에 URL부터 열어보고, 없으면 리스크표의 대체안으로 간다. 이 블록을 그대로 실행해서 안 되면 그건 환경 문제가 아니라 문서 문제다.

```bash
git clone https://github.com/NVlabs/GraspGenX.git
cd GraspGenX
# RG2 URDF로 신규 그리퍼 통합 (인터랙티브 마법사가 config.json 자동 생성)
# 정확한 CLI는 저장소의 "Integrating a New Gripper" 섹션 참고
python integrate_gripper.py --urdf <RG2_URDF_경로>   # 예시, 실제 스크립트명 저장소에서 확인 필요

# FoundationPose가 준 물체 pose + 물체 메시(또는 점군)를 입력으로 그립 후보 생성
python run_graspgenx.py --gripper rg2 --object_pose <foundationpose_output>
```
**DoD:** 물체 1종에 대해 GraspGenX가 생성한 그립 후보 중 실행 가능한(충돌 없는) 그립 1개 이상 확보. GraspGenX 저장소 상태(설치 난이도, 문서화 수준)가 예상보다 미성숙하면 원래 GraspGen의 Robotiq 2F-140 체크포인트를 RG2 스트로크에 맞게 오프셋 보정해 임시 대체(리스크 참고).

**P0. TAMP-lite 상태머신 (인식→그립→서브골→모션)**
```bash
ros2 pkg create --build-type ament_python tamp_lite_statemachine \
  --dependencies rclpy moveit_msgs geometry_msgs std_msgs

# 상태: IDLE → APPROACH(D435i/Octomap 회피 경로) → PERCEIVE(FoundationPose 6D pose)
#      → GRASP_SELECT(GraspGenX 후보 중 선택) → GRASP → LIFT → DONE/FAIL
ros2 run tamp_lite_statemachine sm_node
ros2 topic pub /tamp/start std_msgs/msg/Empty "{}"   # 수동 트리거 테스트
```
**DoD:** 상태머신 노드 1개, 물체 1종 대상 pick 서브골 시퀀스(인식→그립 선택→모션)가 로그로 확인됨.

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
| **🔴 GPU PC 미확인 (`nvidia-smi`, `docker info \| grep -i runtime`)** | **스프린트 단일 실패점.** 개인 노트북엔 NVIDIA GPU가 없어(2026-08-01 확인) nvblox·FoundationPose 둘 다 실행 불가. GPU PC가 없거나 nvidia-docker가 없으면 Day4 전체가 무산된다 | **Day0에 최우선 확인.** 실패 시 대안: 인식을 CPU 경로(색상/AprilTag 기반 위치 추정 + 하드코딩 그립)로 되돌리고 FoundationPose·GraspGenX는 다음 스프린트로 이월. Day1.5~Day3(Octomap 충돌 회피)는 GPU 없이도 개인PC에서 그대로 가능 |
| **FoundationPose는 TensorRT/CUDA 필수** | nvblox를 "GPU 없음"을 이유로 Octomap으로 강등했는데, FoundationPose는 그보다 무거운 GPU 의존을 갖는다. Day4는 **GPU PC 전용 작업**이 된다 | Day4 작업 장소를 GPU PC로 명시 고정. 개인PC에서는 rosbag 재생 + Octomap/플래너/상태머신 골격만 개발하고, 인식 노드는 인터페이스(토픽·메시지 타입)만 먼저 확정해 나중에 갈아끼운다 |
| isaac_ros_common/nvblox 버전 불일치 (release-3.2 vs 최신 태그 혼용) | 빌드 실패, Day1~2 지연 | 모든 Isaac ROS 저장소를 release-3.2로 통일해서 클론, 다른 릴리스 태그와 섞지 않기. 빌드 에러 시 `.isaac_ros_common-config`의 이미지 키와 태그 조합 재확인 |
| MoveIt2 sensors_3d.yaml 플러그인 설정 경험 부재로 Day2 지연 | Day2 지연 | Day1 저녁에 `depth_image_proc`+`octomap_server` 조합을 M0609 없이 데스크탑에서 먼저 단독 테스트해 토픽 흐름부터 검증. 막히면 임시로 RViz에 수동 박스 충돌 오브젝트만 넣고 진행, 실제 depth 연동은 Day3로 이월 |
| eye-to-hand(D435i) 캘리브레이션 오차 누적 | 충돌 회피 정확도 저하 | 알려진 좌표 물체로 오차 측정 후 진행, 1cm 초과 시 Day3 보류 |
| eye-in-hand(C270) hand-eye 오프셋 오차 | 향후 C270을 근접 확인 용도로 쓸 때 오차 누적 | Day2에 캘리브레이션 인프라는 유지하되, 이번 스프린트 메인 경로에서는 C270 정밀도가 결과에 영향 없음(사용 안 함) |
| **FoundationPose 세그멘테이션 마스크 품질 의존성** | 마스크 부정확 시 6D pose 추정 오차/실패 | Day4 초반에 간단한 색상 기반 마스크로 먼저 검증, 필요 시 수동 박스 지정으로 폴백 |
| **isaac_ros_foundationpose가 release-3.2에 없거나 Humble 미지원일 가능성** | Day4 전체 지연 | Day1 저녁에 저장소 태그/브랜치를 미리 확인. 없으면 해당 시점 최신 Humble 호환 태그로 대체하거나, 최악의 경우 FoundationPose 컨테이너를 별도 이미지로 분리 실행(트레이드오프: 통합 복잡도↑) |
| **GraspGenX는 저장소 실재 자체가 미확인** (`NVlabs/GraspGen`은 확인되나 `GraspGenX`는 아님. 아래 Day4 블록의 `integrate_gripper.py`/`run_graspgenx.py`도 문서 자체가 "스크립트명 확인 필요"로 적어둠) | Day4 그립 생성 전면 재설계 | **Day0에 URL 접속 한 번으로 끝나는 확인이므로 즉시 한다.** 없으면 `NVlabs/GraspGen`의 Robotiq 2F-140 체크포인트를 RG2 스트로크(110mm)에 맞게 오프셋 보정해 쓰거나, 물체 1종만 다루므로 그립 지점 하드코딩으로 스코프 축소 |
| D435i 최종 접근 시 self-occlusion (팔이 시야 가림) | FoundationPose 추적이 그립 직전 끊길 수 있음 | 접근 마지막 단계는 미리 계획된 궤적(open-loop)으로 처리하고, C270을 필요시 근접 확인용 폴백으로 재도입 검토(다음 스프린트) |
| VoiceProcess 인터페이스 사양 불명확 | Day4 P1 지연 | P1로 낮춰뒀으므로 지연 시 이월, 최악의 경우 키보드 트리거로 대체 |
| D435i + C270 동시 구동 시 USB 대역폭 이슈 | 인식 프레임 드랍 | Day1에 별도 USB 컨트롤러 분리 연결, 필요 시 C270 저해상도 다운그레이드 |
| 실기 충돌/안전 | 하드웨어 손상, 안전사고 | Day3부터 속도 스케일링 20~30% 제한, protective stop 여유 확보, 비상정지 상시 대기 |

---

## 4. Definition of Done (스프린트 전체)

- [ ] nvblox 3D 재구성이 RViz에서 실시간 확인됨 (D435i, eye-to-hand)
- [ ] D435i depth 기반 실제 3D 포인트클라우드가 MoveIt2 PlanningScene의 Octomap 충돌 지오메트리로 반영됨 (nvblox는 시각화 용도로만 병행)
- [ ] 임의 배치 장애물에 대해 M0609가 실기에서 충돌 없이 회피 경로로 도달
- [ ] FoundationPose가 D435i RGB-D로 물체 1종의 6D pose를 추정, 위치/회전 오차가 허용 범위 이내
- [ ] GraspGenX(또는 임시 대체)가 RG2 기준 실행 가능한 그립 후보를 생성
- [ ] 인식(FoundationPose) → 그립 선택(GraspGenX) → 서브골 → 모션 실행 상태머신이 실기에서 최소 1개 물체에 대해 동작
- [ ] 10회 반복 시행 결과와 실패 유형이 기록됨
- [ ] 다음 스프린트 백로그(다층 대응, TAMP 확장, GPU 가속 등) 문서화 완료

## 5. Key Dates

| Day | 이벤트 |
|---|---|
| Day0 | **GPU PC 확인(`nvidia-smi`, docker runtime) + GraspGenX URL 실재 확인** — 둘 다 몇 분이면 끝나고, 실패 시 Day4 설계가 통째로 바뀐다 |
| Day1 | 스프린트 시작, D435i/C270 파이프라인 구성 |
| Day1.5 | 빠른 초안 검증(장애물 회피 궤적 시연) — GPU 불필요, 개인PC 가능 |
| Day2 | 이중 캘리브레이션 + PlanningScene 연동 (최대 난관 구간) |
| Day3 | 충돌 회피 실기 검증 및 튜닝 |
| Day4 | FoundationPose + GraspGenX + TAMP-lite 상태머신 + 음성 트리거 |
| Day5 | 통합 검증, 회고, 차기 백로그 |

---

## 6. 다음 스프린트 후보 (이번엔 스코프 아웃)

- 다층/적재(bin-picking) 상황 대응: FoundationPose+GraspGenX 조합을 여러 물체/겹침 상황으로 확장, 순서 결정(picking order) 로직 추가
- C270 근접 확인 역할 재도입 검토: FoundationPose가 self-occlusion 구간에서 끊길 경우의 폴백으로
- cuTAMP 스타일 GPU 배치 최적화 (skeleton search + 배치 IK/충돌/안정성 제약 만족)
- VLM 기반 자연어 지시 → 서브골 자동 생성 (OWL-TAMP/VLM-TAMP 개념)
- 실패 사례 기반 skill-effect 모델 재학습 (Fail2Progress 개념 본격 적용)
- People/dynamic reconstruction 모드로 nvblox 전환 (협동로봇 협업 시나리오 대비)
- ROS 2 Jazzy 전체 마이그레이션 후 cuMotion(`isaac_ros_cumotion_moveit`) 재도전 — GPU 가속 충돌 회피로 전환

> **참고:** 위 명령어들은 패키지/저장소 버전에 따라 인자명이나 launch 파일명이 다를 수 있습니다(특히 `doosan_robot2`, `easy_handeye2`, VoiceProcess 인터페이스는 본인 환경의 실제 저장소 문서로 재확인 필요). 실행 전 각 저장소 README의 Humble 대응 브랜치를 먼저 확인하세요.
>
> **버전 정책:** 이번 스프린트는 Humble 유지를 위해 Isaac ROS 관련 저장소를 모두 `release-3.2` 태그로 고정한다. 최신 `release-4.x`는 Docker dev container 기능이 Isaac ROS CLI로 이전되었고 cuMotion 등 신규 패키지가 사실상 Jazzy 중심으로 재편되어, 이번 스코프(Humble/M0609)에서는 사용하지 않는다.
