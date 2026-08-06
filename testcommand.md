<!-- meta
updated: 2026-08-06 17:00
status:  live
owns:    실행 명령어(호스트/컨테이너 구분) · 노드 지도 · 단계별 검증 명령
-->

# 실행 명령서 — MoveIt + cuMotion + nvblox 장애물 회피

> **2026-08-06 이 순서 그대로 실기에서 전 구간 관통 확인.** 마지막 검증: OMPL 10/10, cuMotion 10/10.
> 왜 이렇게 하는지·함정의 근거는 [[ws/cobot2/plans/2026-08-05-cumotion-bringup]]과
> [[ws/cobot2/context/constraints]]가 단일 출처다. 여기엔 **치는 것**만 둔다.

---

## 0. 파이프라인 한 장

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

## 10. 종료 — **순서 반대로, 그리고 반드시 확인**

오늘 실패의 절반이 **죽은 줄 알았던 노드**였다. `pkill -f`는 `docker exec bash -c`에서
**자기 명령줄에도 매칭돼 자기 셸을 먼저 죽인다** — 뒤 명령이 조용히 실행되지 않는다.

```bash
# 컨테이너: T7 → T6 → T5 → T4 순으로 Ctrl+C, 그 다음 반드시 확인
pgrep -a -f "move_group|cumotion|nvblox|segmenter"
# 남아 있으면 PID로: kill <pid>

# 호스트: T2 → T1
ps -eo pid,cmd | grep -E "ros2 launch" | grep -v grep
```

```bash
nvidia-smi --query-gpu=memory.used --format=csv,noheader   # 정리되면 ~33 MiB
```

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
