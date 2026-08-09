<!-- meta
updated: 2026-08-08
status:  live
owns:    실행 명령어(호스트/컨테이너 구분) · 노드 지도 · 단계별 검증 명령
         2026-08-08: md/context/test_grap_plan.md(경로 B)를 여기로 합쳤다. 그 파일은
         비교용으로 남아 있으나 갱신하지 않는다 — 값의 정본은 이 문서다
-->

# 실행 명령서 — 로봇 + MoveIt 위의 두 경로

**T1~T3(로봇·카메라·MoveIt)를 공유하고 거기서 갈라진다.**

| | 무엇 | 어디서 | 목적 |
|---|---|---|---|
| **경로 A** | cuMotion + nvblox | 컨테이너(GPU) | 동적 장애물 회피. 로봇은 계획만 |
| **경로 B** | GraspGenX + pick_fsm | 호스트(GPU) | 실제로 집는다 |

> 왜 이렇게 하는지·함정의 근거는 [[ws/cobot2/plans/2026-08-05-cumotion-bringup]]과
> [[ws/cobot2/context/constraints]]가 단일 출처다. 여기엔 **치는 것**만 둔다.
> 경로 A는 2026-08-06 실기 전 구간 관통 확인(OMPL 10/10, cuMotion 10/10).

---

## ⚡ 명령어만 — 복붙용

**공통 T1~T3** (모든 터미널에서 `rdm` = `export ROS_DOMAIN_ID=93` 먼저)

```bash
# T1 카메라 — 이미 떠 있으면 띄우지 않는다 (ros2 node list | grep camera)
#   🔴 alias(`reals`)를 쓰지 말고 아래처럼 인자를 직접 친다. **alias 정의가 머신마다 다르다**
#      (개인PC `.bashrc:174` 는 인자 없음 → 424x240x15. GPU PC 는 다를 수 있다)
#      내 머신 것 확인: alias reals
ros2 launch m0609_rg2_bringup camera.launch.py depth_profile:=424x240x15 color_profile:=424x240x15
#   ✅ 2026-08-09 실기 확정: 이 값은 런치 기본값과 같다 — 사실 인자를 안 줘도 된다.
#      ~~480x320x15~~ 는 **D435i 가 지원하지 않는 프로파일**이라 폐기했다(rs-enumerate-devices 실측).
#      Color: 320x180 320x240 424x240 640x360 640x480 848x480 960x540 1280x720 1920x1080
#      Depth: 256x144 424x240 480x270 640x360 640x480 848x100 848x480 1280x720
#      ⚠️ depth 의 `480x270` 과 헷갈리지 말 것 — `480x320` 은 어느 쪽에도 없다.
#   ⚠️ 캘리브 데이터 수집 때만 예외로 1280x720 (424x240 으로 찍으면 코너가 안 잡혀 불합격난다)

# T2 로봇 (실기) — rviz:=false 필수, moveit이 자기 RViz를 띄운다
ros2 launch m0609_rg2_bringup bringup.launch.py mode:=real host:=192.168.1.100 rviz:=false

# T3 MoveIt — standalone:=false 필수. cumotion:=true는 경로 A(컨테이너)에서만
ros2 launch m0609_rg2_moveit moveit.launch.py standalone:=false octomap:=true cumotion:=true
```

**경로 A — cuMotion + nvblox** (T4~T7, 전부 컨테이너. 셸마다 §3의 source 4줄 먼저)

```bash
# T4 robot_segmenter — 빼면 로봇 자기 몸이 장애물이 된다
ros2 run isaac_ros_cumotion robot_segmenter_node --ros-args \
  -p robot:=m0609_rg2.xrdf \
  -p urdf_path:=/workspaces/isaac_ros-dev/m0609/m0609_kinematics.urdf \
  -p distance_threshold:=0.15 \
  -p depth_image_topics:="[/camera/camera/aligned_depth_to_color/image_raw]" \
  -p depth_camera_infos:="[/camera/camera/aligned_depth_to_color/camera_info]" \
  -p robot_mask_publish_topics:="[/cumotion/camera_1/robot_mask]" \
  -p world_depth_publish_topics:="[/cumotion/camera_1/world_depth]"

# T5 nvblox — esdf_mode:=3d 없으면 cuMotion 첫 요청에 FATAL
ros2 run nvblox_ros nvblox_node --ros-args \
  --params-file /workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_examples/nvblox_examples_bringup/config/nvblox/nvblox_base.yaml \
  -p global_frame:=base_link -p use_lidar:=false -p num_cameras:=1 -p esdf_mode:=3d \
  -r camera_0/depth/image:=/cumotion/camera_1/world_depth \
  -r camera_0/depth/camera_info:=/camera/camera/aligned_depth_to_color/camera_info \
  -r camera_0/color/image:=/camera/camera/color/image_raw \
  -r camera_0/color/camera_info:=/camera/camera/color/camera_info

# T6 cuMotion 플래너 — read_esdf_world:=False면 장애물을 못 보는데 계획은 성공한다
ros2 run isaac_ros_cumotion cumotion_planner_node --ros-args \
  -p robot:=m0609_rg2.xrdf \
  -p urdf_path:=/workspaces/isaac_ros-dev/m0609/m0609_kinematics.urdf \
  -p read_esdf_world:=True \
  -p esdf_service_name:=/nvblox_node/get_esdf_and_gradient \
  -p update_esdf_on_request:=True \
  -p publish_curobo_world_as_voxels:=True

# 검증 — 로봇 안 움직인다
python3 /workspaces/cobot2_ws/scripts/bench_planning_time.py --repeat 10
```

**경로 B — GraspGenX + pick_fsm** (T4~T6, 호스트)

```bash
# T4 grasp 브리지 — GPU 워커를 자식으로 띄운다. 첫 실행은 모델 로드로 수십 초
ros2 run graspgenx_perception grasp_bridge_node --ros-args \
  -p out_dir:=$(pwd)/data/graspgenx_scene -p scene:=01
#   out_dir 비우면 임시 디렉토리에 썼다가 지운다. scene 같으면 덮어쓴다

# T5 인식만 단독 확인 — 로봇 안 움직인다 ⭐
ros2 service call /grasp/compute std_srvs/srv/Trigger {}
ros2 topic echo /grasp/best_tcp --once    # 손끝 좌표를 자로 잰 물체 위치와 대조

# T6 FSM — 여기서부터 실제로 움직인다
ros2 launch pick_fsm pick_fsm.launch.py \
  grasp_source:=legacy_trigger voice:=false target:=apple dry_run:=false

# 조작 (다른 터미널)
ros2 topic echo /pick/state &
ros2 service call /pick/start   std_srvs/srv/Trigger {}   # → WAIT_APPROVAL 에서 멈춘다
ros2 service call /pick/approve std_srvs/srv/Trigger {}   # ✋ 여기서 로봇이 움직인다
ros2 service call /safety/stop  std_srvs/srv/Trigger {}   # 즉시 정지
```

**T0 사전 점검** (경로 B 원문에 있던 것 — 경로 A에도 유효하다)

```bash
nvidia-smi                                 # GPU PC인지 판별. 없으면 개인PC다
ping -c1 192.168.1.100                     # 로봇
ping -c1 192.168.1.1                       # RG2 Modbus
rdm && ros2 node list                      # 남의 계정 move_group이 있는지
```

---

## 🔴 합치면서 드러난 파라미터 불일치 — 사람이 정해야 한다

두 문서가 **같은 명령을 다른 파라미터로** 적고 있었다. 아래는 launch 파일 소스로 확인한
사실이고, **어느 쪽이 맞는지는 실기에서 정한다**(이 판정은 CPU PC에서 못 한다).
원문 비교가 필요하면 `md/context/test_grap_plan.md`가 그대로 남아 있다.

| 명령 | 경로 A (이 문서 원본) | 경로 B (`test_grap_plan`) | 실제 차이 |
|---|---|---|---|
| `bringup.launch.py` | `rviz:=false` | `model:=m0609` (rviz 미지정) | 🔴 **B가 위험하다.** `rviz` 기본값이 `true`라 B는 bringup RViz를 띄우고 moveit RViz와 2개가 된다 — launch 파일 26~28행 주석이 "moveit과 함께 쓸 땐 false"라고 직접 적어놨다. `model`은 **선언된 인자가 아니라 조용히 무시된다**(실측 확인: 미선언 인자는 에러 없이 무시). bringup이 `model='m0609'`를 하드코딩(46행)하므로 결과는 같지만 **아무 일도 안 하는 인자**다 |
| `camera.launch.py` | `848x480x15` (alias `reals` 경유로 표기) | 인자 없음 | ✅ **해결(2026-08-09 실기).** ~~2026-08-08 결정: `480x320x15` 로 통일~~ 은 **그 값이 D435i 미지원이라 폐기됐다** → **`424x240x15`(= 런치 기본값)로 통일**(위 T1). 🔴 **alias 를 경유하지 말고 인자를 직접 쓴다** — `reals` 의 정의가 **머신마다 다르다**(개인PC `.bashrc:174` 에서는 인자가 없어 기본 `424x240x15` 로 뜬다. GPU PC 의 `.bashrc` 는 이 문서 표기대로 848 일 수 있으나 **확인 못 했다**). 그래서 "같은 명령을 쳤는데 해상도가 다르다"가 성립한다. 참고로 GraspGenX 실측 기록(README:34 → :258)은 848×480 → **1280×720** 로 바뀌는데, 그건 `alias realsense`(다른 런치, color `1280x720x30`)로 띄웠고 `aligned_depth_to_color` 가 **color 를 따라가기** 때문이다(`constraints.md:25`) |
| `moveit.launch.py` | `octomap:=true cumotion:=true` | `standalone:=false` 만 | `octomap` 기본값은 `true`라 같다. **`cumotion`은 기본값 `false`** (51행) → 경로 B에는 cuMotion 파이프라인이 **안 올라온다.** `pick_fsm ... planning_pipeline:=isaac_ros_cumotion`을 쓰려면 T3를 `cumotion:=true`로 띄워야 한다. 단 그 인자는 **Isaac ROS 컨테이너에서만** 켤 수 있다(48행 주석) |

**정하고 나면 이 표를 지우고 위 "명령어만" 블록에 반영한다.**

---

## 0. 파이프라인 한 장 (경로 A)

```
호스트                                              컨테이너 (Isaac ROS 3.2)
─────────────────────────────────────────────────────────────────────────────────
T1  camera.launch.py
      └ /camera/camera/aligned_depth_to_color/image_raw ──┐
      └ camera_calib_tf (base_link→camera_link)           │
                                                          ▼
T2  bringup.launch.py (실기 로봇)              T4  robot_segmenter_node
      └ /dsr01/* 컨트롤러                            (로봇 몸을 depth에서 지움)
      └ /joint_states (12관절 + velocity)               │ /cumotion/camera_1/world_depth
                                    │                    ▼
                                    │            T5  nvblox_node (esdf_mode:=3d)
                                    │                    │ get_esdf_and_gradient (서비스)
                                    │                    ▼
                                    └──────────▶ T6  cumotion_planner_node
                                                         │ /cumotion/move_group (액션)
                                                         ▼
                                                 T7  move_group (cumotion:=true)
                                                         └ RViz 드롭다운 OMPL ↔ cuMotion
```

**장애물이 두 경로로 들어간다. 헷갈리지 말 것:**

| 플래너 | 장애물 출처 | self-filter |
|---|---|---|
| **OMPL** | MoveIt octomap (`/camera/camera/depth/color/points`) | `sensors_3d.yaml`의 padding |
| **cuMotion** | nvblox ESDF (서비스로 pull) | **`robot_segmenter_node`** |

🔴 **cuMotion은 octomap을 아예 안 본다.** 그래서 `robot_segmenter` + nvblox가 빠지면
"계획은 성공하는데 장애물을 통과"한다. 실패가 아니라 **성공처럼 보이는 실패**다.

---

## 1. 호스트 — T1: 카메라

```bash
rdm                                    # ROS_DOMAIN_ID=93
ros2 node list | grep camera           # ⚠️ 먼저 확인. 이미 있으면 띄우지 않는다
reals
# = ros2 launch m0609_rg2_bringup camera.launch.py depth_profile:=848x480x15 color_profile:=848x480x15
```

- ⚠️ `realsense-viewer`가 떠 있으면 먼저 닫는다 (USB 독점 → 노드가 죽는데 증상은 "TF 없음"으로 나온다)
- ⚠️ **드라이버를 두 번 띄우면 depth가 반토막 난다.** `ros2 node list`에 `/camera/camera`가
  2개면 그것이다 (2026-08-06 실측: 15 → 5.6 Hz)

**검증**

```bash
ros2 node list | grep -c "camera/camera"                         # 1
ros2 topic hz /camera/camera/aligned_depth_to_color/image_raw    # 실측 9.65 Hz
ros2 run tf2_ros tf2_echo base_link camera_link                  # [1.237, -0.223, 0.784]
```

## 2. 호스트 — T2: 실기 로봇

```bash
rdm && br
# = ros2 launch m0609_rg2_bringup bringup.launch.py mode:=real host:=192.168.1.100 rviz:=false
```

**검증** — `/joint_states`가 cuMotion의 전제조건이다.

```bash
ros2 topic info /joint_states          # Publisher count: 1  ← 2면 옛 launch가 살아있는 것
ros2 topic echo /joint_states --once   # name 12개 / position 12개 / velocity 12개
```

🔴 **velocity가 비어 있으면 cuMotion 계획이 전부 실패한다.** `publish_default_velocities: True`가
`bringup.launch.py`에 들어가 있어야 한다(커밋됨).

> `[OnRobot Modbus]: Connection failed!`는 그리퍼 통신 실패다. 계획에는 영향 없다
> (`rg2_finger_joint`는 XRDF에서 lock). 그리퍼를 실제로 여닫으려면 이걸 먼저 고쳐야 한다.

## 3. 호스트 — T3: 컨테이너 기동

```bash
rdm                                    # ⚠️ 먼저. run_dev.sh가 -e ROS_DOMAIN_ID로 넘긴다
cd ~/cobot2_ws/isaac_ros-dev/src/isaac_ros_common/scripts
./run_dev.sh -a "-v $HOME/cobot2_ws:/workspaces/cobot2_ws"
```

`rdm` 없이 열면 컨테이너가 도메인 0이 되어 로봇을 못 본다.

### 컨테이너에 들어가면 **맨 처음 한 번**

```bash
bash /workspaces/cobot2_ws/scripts/container_setup.sh
```

🔴 **`run_dev.sh`는 컨테이너를 재사용하지 않고 새로 만든다.** pip 설치(warp 1.5.0, numpy 1.26.4)가
매번 날아간다. 안 하면 `AttributeError: module 'warp' has no attribute 'torch'`로 죽는다.

### 컨테이너 **셸마다** (T4~T7 전부)

```bash
source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash
source /workspaces/cobot2_ws/install_container/setup.bash
export ROS_DOMAIN_ID=93
```

⚠️ **`RMW_IMPLEMENTATION`은 설정하지 않는다.** cyclonedds로 바꾸면 T7의 컨트롤러 spawner가
호스트 `controller_manager` **서비스**를 못 불러 멈춘다(교차 벤더는 토픽만 된다).

## 4. 컨테이너 — T4: robot_segmenter (로봇을 depth에서 지움)

```bash
cd /workspaces/isaac_ros-dev
ros2 run isaac_ros_cumotion robot_segmenter_node --ros-args \
  -p robot:=m0609_rg2.xrdf \
  -p urdf_path:=/workspaces/isaac_ros-dev/m0609/m0609_kinematics.urdf \
  -p distance_threshold:=0.15 \
  -p depth_image_topics:="[/camera/camera/aligned_depth_to_color/image_raw]" \
  -p depth_camera_infos:="[/camera/camera/aligned_depth_to_color/camera_info]" \
  -p robot_mask_publish_topics:="[/cumotion/camera_1/robot_mask]" \
  -p world_depth_publish_topics:="[/cumotion/camera_1/world_depth]"
```

🔴 **이걸 빼면 cuMotion이 로봇 자기 몸을 장애물로 보고 계획이 전부 실패한다**
(`INVALID_START_STATE_WORLD_COLLISION`). nvblox는 MoveIt의 self-filter를 안 거친다.

**검증**: `ros2 topic hz /cumotion/camera_1/world_depth` → 실측 **3.7 Hz**
⚠️ 여기가 파이프라인 병목이다(카메라 9.65 → 3.7 Hz, 최대 공백 3.1초).

## 5. 컨테이너 — T5: nvblox

```bash
ros2 run nvblox_ros nvblox_node --ros-args \
  --params-file /workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_examples/nvblox_examples_bringup/config/nvblox/nvblox_base.yaml \
  -p global_frame:=base_link \
  -p use_lidar:=false \
  -p num_cameras:=1 \
  -p esdf_mode:=3d \
  -r camera_0/depth/image:=/cumotion/camera_1/world_depth \
  -r camera_0/depth/camera_info:=/camera/camera/aligned_depth_to_color/camera_info \
  -r camera_0/color/image:=/camera/camera/color/image_raw \
  -r camera_0/color/camera_info:=/camera/camera/color/camera_info
```

- 🔴 **`esdf_mode:=3d` 없으면 cuMotion의 첫 요청에 nvblox가 FATAL로 죽는다.**
  기본값이 `2d`다. cuMotion 로그에는 계획 실패만 남으므로 **`pgrep -f nvblox_node`로 확인할 것**
- **depth 입력만** 세그멘터 출력으로 바꾼다. `camera_info`·color는 원본 그대로
- 세그멘터를 나중에 끼웠다면 **nvblox를 재시작한다** — 기존 지도의 로봇은 안 지워진다
- `nvblox_realsense.yaml`은 얹지 않는다 (`map_clearing_frame_id`가 우리 TF와 안 맞는다)

**검증**

```bash
ros2 param get /nvblox_node global_frame       # base_link
ros2 service list | grep esdf                  # /nvblox_node/get_esdf_and_gradient
pgrep -f nvblox_node                           # 살아 있어야 한다
```

## 6. 컨테이너 — T6: cuMotion 플래너

```bash
cd /workspaces/isaac_ros-dev
ros2 run isaac_ros_cumotion cumotion_planner_node --ros-args \
  -p robot:=m0609_rg2.xrdf \
  -p urdf_path:=/workspaces/isaac_ros-dev/m0609/m0609_kinematics.urdf \
  -p read_esdf_world:=True \
  -p esdf_service_name:=/nvblox_node/get_esdf_and_gradient \
  -p update_esdf_on_request:=True \
  -p publish_curobo_world_as_voxels:=True
```

- `robot:=`은 **파일명만** 준다(경로 아님). `isaac_ros_cumotion_robot_description/xrdf/`에서 찾는다
- 워밍업에 5~10초 걸린다. `cuMotion is ready for planning queries!`가 나와야 준비 완료
- 🔴 **`read_esdf_world:=False`로 띄우면 장애물을 못 본다.** 계획은 성공한다 — 그래서 위험하다

**검증**: `ros2 action list | grep cumotion` → `/cumotion/move_group`

## 7. 컨테이너 — T7: move_group (+ RViz)

```bash
ros2 launch m0609_rg2_moveit moveit.launch.py standalone:=false octomap:=true cumotion:=true
#   RViz를 따로 띄울 거면 rviz:=false
```

**검증** — 로그에 이 세 줄이 다 나와야 한다.

```
Loading planning pipeline 'ompl'                 → Using planning interface 'OMPL'
Loading planning pipeline 'isaac_ros_cumotion'   → Using planning interface 'Generate minimum-jerk ... cuMotion'
Configured and activated dsr_moveit_controller   ← 이게 있어야 Execute가 된다
```

```bash
ros2 topic hz /moveit/filtered_cloud    # 실측 2.3 Hz — OMPL octomap용 self-filter 결과
```

> `Controller already loaded, skipping load_controller` → `Failed to configure controller`는
> **옛 move_group이 이미 spawn해 둔 것**이다. 컨트롤러는 이미 active이므로 Execute는 된다.

> ⚠️ **`ros2 node list`에 `/move_group`이 2개로 보이는 것은 정상이다.** MoveIt이 내부적으로
> 같은 이름의 노드를 하나 더 만든다(궤적 실행 관리자). **중복 실행이 아니다.**
> 진짜로 판정하려면 이 둘을 본다 — 둘 다 1이어야 한다:
> ```bash
> ps -eo cmd | grep -c "moveit_ros_move_group/move_group"   # 1
> ros2 action list | grep -cE "^/move_action$"              # 1
> ```

---

## 8. RViz에서 볼 것

| 보고 싶은 것 | Display | Topic |
|---|---|---|
| **cuMotion이 쥔 장애물** | Marker | `/curobo/voxels` |
| nvblox 지도 | NvbloxMesh / PointCloud2 | `/nvblox_node/mesh`, `/nvblox_node/color_layer` |
| OMPL octomap 입력 | PointCloud2 | `/moveit/filtered_cloud` |
| octomap 결과 | PointCloud2 | `/octomap_point_cloud_centers` |

- 🔴 **`/nvblox_node/static_esdf_pointcloud`는 `esdf_mode:=3d`에서 발행되지 않는다** (2d 슬라이스 전용).
  RViz에서 비어 보이는 게 정상이다 — `mesh`/`color_layer`를 쓴다
- 🔴 **`/curobo/voxels`는 계획을 한 번 돌려야 나온다.** 구독자가 있을 때만, 계획 요청 처리 중에만
  발행한다. 대기 중 `topic hz`로 판정하지 말 것
- `octomap_rviz_plugins` 미설치라 `/octomap_binary`는 못 본다

---

## 9. 검증 — 계획 시간 재기 (로봇 안 움직임)

```bash
python3 /workspaces/cobot2_ws/scripts/bench_planning_time.py --repeat 10
```

`plan_only=True` 고정이라 **로봇은 움직이지 않는다.** RViz 드롭다운을 사람이 번갈아 누르는 대신
`pipeline_id`만 바꿔 같은 목표를 N회 계획한다.

**2026-08-06 실기 실측** (로봇+카메라+nvblox 전부 살아 있는 상태, 관절목표, 각 10회):

| | server 중앙값 | wall 중앙값 | 성공 |
|---|---|---|---|
| OMPL | 42.4 ms | 106.0 ms | 10/10 |
| cuMotion | 110.6 ms | 204.1 ms | **10/10** |

cuMotion이 쥔 장애물 복셀: **27,646개** (`/curobo/voxels`)

> ⚠️ 이 숫자로 "cuMotion이 느리다"고 결론내지 말 것 — 관절공간 목표는 OMPL(RRTConnect)에
> 가장 유리한 조건이다. 판단 근거는 장애물이 궤적을 실제로 막는 씬에서의 비교다.

---

## 10. 종료 — GPU를 다음 사람에게 넘기는 절차

🔴 **이 랩탑은 세 계정(`joonwon`·`kimkh`·`rokey`)이 동시 로그인해 같은 GPU와 같은 ROS 도메인(93)을
쓴다.** 다른 계정도 자기 Isaac ROS 컨테이너(`cumotion-joonwon` 등)를 띄운다.
→ **`ps`에 `user`를 넣지 않으면 남의 프로세스를 내 것으로 착각한다.** 2026-08-06에 실제로 헤맸다.

```bash
# ① 누가 GPU를 쥐고 있나
nvidia-smi --query-compute-apps=pid,used_memory --format=csv

# ② 그 PID가 내 것인지 확인 — 남의 것이면 절대 kill하지 않는다
ps -o pid,user,cmd -p <pid>
```

**종료는 올린 순서의 반대로**: T7 move_group → T6 플래너 → T5 nvblox → T4 세그멘터 →
T2 bringup → T1 카메라. 각 터미널에서 Ctrl+C.

```bash
# ③ 정말 죽었는지 확인 — 오늘 실패의 절반이 "죽은 줄 알았던 노드"였다
ps -eo pid,user,cmd | grep -E "move_group|nvblox|cumotion|segmenter|realsense2_camera_node" | grep -v grep
# 내 것이 남아 있으면 PID로: kill <pid>   (안 죽으면 kill -9)
```

⚠️ **`pkill -f`를 쓰지 말 것.** `docker exec bash -c`에서 **자기 명령줄에도 매칭돼 자기 셸을 먼저
죽인다** — 뒤 명령이 조용히 실행되지 않는데 출력은 깨끗해서 "정리됨"으로 오독한다.
공유 랩탑에서는 남의 프로세스까지 걸린다.

```bash
# ④ 반납 확인
nvidia-smi --query-gpu=memory.used --format=csv,noheader   # 아무도 안 쓰면 ~33 MiB
```

**컨테이너는 지우지도 stop하지도 않는다.** 안의 노드만 내리면 GPU는 반납된다.
`run_dev.sh`를 다시 돌리면 컨테이너가 **새로 만들어져** `container_setup.sh`를 또 돌려야 한다.

**VRAM 실측 (2026-08-06, full-up)** — 8 GB의 31%라 셋 동시 실행에 여유가 있다:

| 노드 | VRAM |
|---|---|
| `cumotion_planner_node` | 1,508 MiB |
| `robot_segmenter_node` | 660 MiB |
| `nvblox_node` | 334 MiB |
| 합계 | **약 2.5 GB / 8 GB** |

---

## 11. 증상 → 원인 빠른 표

| 증상 | 원인 | 조치 |
|---|---|---|
| `module 'warp' has no attribute 'torch'` | 컨테이너 재생성으로 pip 설치 유실 | `container_setup.sh` |
| `import cv2` → `numpy.core.multiarray failed` | 이미지 numpy 2.2.6 vs apt cv2 | `container_setup.sh` |
| cuMotion 계획 전부 실패, 로그엔 `Calling ESDF service`만 | **nvblox가 죽었다** (`esdf_mode` 2d) | `-p esdf_mode:=3d` |
| `INVALID_START_STATE_WORLD_COLLISION` | 로봇이 nvblox 지도에 들어감 | `robot_segmenter_node` + nvblox 재시작 |
| `INVALID_START_STATE_SELF_COLLISION` | XRDF 구 과대추정 | XRDF `self_collision.ignore` (해결됨) |
| cuMotion만 실패, velocity 오류 | `/joint_states`에 velocity 없음 | `publish_default_velocities: True` (해결됨) |
| 계획이 **산발적으로** 실패 | 옛 노드가 안 죽고 중복 발행 | `ros2 topic info` / `ros2 action list`로 개수 확인 |
| 계획은 성공하는데 장애물을 통과 | `read_esdf_world:=False` | `True` + nvblox |
| depth가 절반 이하 | RealSense 드라이버 2개 | `ros2 node list \| grep camera` |

---

## 12. 아직 안 된 것

- **장애물이 궤적을 실제로 바꾸는지 미검증.** 계획 성공과 복셀 적재까지만 확인했다.
  손을 작업공간에 넣고 같은 목표로 OMPL/cuMotion 각각 계획해 궤적이 달라지는지 봐야 한다
- **depth 9.65 Hz** — 요청은 15 Hz다. 원인 미특정
- **세그멘터 3.7 Hz** — 파이프라인 병목. 최대 공백 3.1초는 사람 팔 반응에 부족할 수 있다
- **그리퍼 Modbus 연결 실패** — 여닫기 불가
- **XRDF `link_4 ↔ rg2_base_link` 자기충돌 검사를 꺼 뒀다** — 실기 모션 전 재검토 필수
