<!-- meta
updated: 2026-08-06
status:  live (미검증 — 실기에서 아직 안 돌렸다)
owns:    T4~T7 노드 파라미터 파일의 지도 · 작업영역 정의 · 파일 간 결합 규칙
-->

# config/ — cuMotion 장애물 회피 파이프라인 파라미터

실행 순서·검증 명령은 **`config/testcommand.md`** 가 단일 출처다(같은 디렉토리). 여기엔
**무엇을 어디서 고치는지**만 둔다.

> 2026-08-08: `testcommand.md` 가 두 경로를 다 담는다 — **경로 A**(cuMotion+nvblox, 아래 표의
> T4~T7)와 **경로 B**(GraspGenX+pick_fsm, 호스트). 이 파일의 표는 **경로 A 전용**이다.
> 문서 맨 위 "명령어만" 블록에 복붙용 명령이 모여 있다.

## 파일 지도

| 터미널 | 노드 | 파일 | 적용 방법 |
|---|---|---|---|
| T4 | `robot_segmenter_node` | `cumotion_segmenter.yaml` | `--params-file` |
| T5 | `nvblox_node` | `nvblox_realtime.yaml` | `--params-file` |
| T6 | `cumotion_planner_node` | `cumotion_planner.yaml` | `--params-file` |
| T7 | `move_group` (octomap) | `moveit_sensors_3d.yaml` → **심볼릭 링크** | launch가 패키지에서 자동 로드 |

T7만 링크인 이유: 실물은 `src/cobot_rg2/rg2/m0609_rg2_moveit/config/sensors_3d.yaml`이고
`moveit.launch.py`가 **패키지 share에서** 읽는다. share의 그 파일도 src로의 심볼릭 링크라
**편집하면 빌드 없이 즉시 반영된다.** 여기에 복사본을 두면 두 개의 진실이 생겨서, 고쳤는데
안 먹는 상황이 만들어진다. 링크만 걸어 `config/`에서도 보이게 했다.

## 작업영역과 감시상자 (base_link 기준, m)

```
              작업영역 (사용자 지정)        감시상자 (= 작업영역 + 0.2 여유)
   x          0.00 ~ 0.70                  -0.20 ~ 0.90
   y         -0.30 ~ 0.30                  -0.50 ~ 0.50
   z          테이블 위 물체                -0.05 ~ 0.70
```

**감시상자를 작업영역보다 넓게 잡는 이유 둘** — ① 팔꿈치·상완은 TCP 작업영역 밖을 지난다,
② 사람 손은 작업영역에 *들어오기 전에* 보여야 피할 시간이 생긴다.

🔴 **cuRobo는 상자 밖을 "자유공간"으로 취급한다.** octomap의 "모르면 막힘"과 반대다.
상자를 좁히면 장애물이 사라지지 계획이 막히지 않는다 — 실패가 조용하다.

복셀 22 × 20 × 15 = **6,600개** (기본 2×2×2 m 상자의 64,000 대비 10%).

### 상자를 바꿀 때 같이 고쳐야 하는 곳 (하나라도 빠지면 조용히 어긋난다)

| 고치는 곳 | 파일 | 안 맞추면 |
|---|---|---|
| `workspace_bounds_min/max_*` | `nvblox_realtime.yaml` | 지도에 안 들어옴 |
| `grid_center_m` / `grid_size_m` | `cumotion_planner.yaml` | 플래너가 그 영역을 요청 안 함 |
| `projective_integrator_max_integration_distance_m` | `nvblox_realtime.yaml` | 상자 먼 구석이 미관측 |
| `map_clearing_radius_m` | `nvblox_realtime.yaml` | 상자 밖인데 지도에 남음 |

`grid_size_m` 성분은 `voxel_size`(0.05)의 정수배여야 한다 — 아니면 그리드 shape 불일치로
`cumotion_planner.py:432`에서 FATAL. `voxel_size`는 T5/T6가 **같아야** 한다(불일치 시 FATAL).

## 실행 (컨테이너 T4·T5·T6)

```bash
# 컨테이너 셸마다 먼저
source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash
source /workspaces/cobot2_ws/install_container/setup.bash
export ROS_DOMAIN_ID=93

# T4 — 세그멘터

cd /workspaces/isaac_ros-dev && ros2 run isaac_ros_cumotion robot_segmenter_node --ros-args \
  --params-file /workspaces/cobot2_ws/config/cumotion_segmenter.yaml

# T5 — nvblox (리매핑은 params-file로 못 준다. -r 은 명령줄에 남는다)
ros2 run nvblox_ros nvblox_node --ros-args \
  --params-file /workspaces/cobot2_ws/config/nvblox_realtime.yaml \
  -r camera_0/depth/image:=/cumotion/camera_1/world_depth \
  -r camera_0/depth/camera_info:=/camera/camera/aligned_depth_to_color/camera_info

# T6 — 플래너

cd /workspaces/isaac_ros-dev && ros2 run isaac_ros_cumotion cumotion_planner_node --ros-args \
  --params-file /workspaces/cobot2_ws/config/cumotion_planner.yaml

# T7 — move_group (변경 없음. sensors_3d.yaml은 launch가 알아서 읽는다)
ros2 launch m0609_rg2_moveit moveit.launch.py standalone:=false octomap:=true cumotion:=true
```

`use_color: false`로 바꿨으므로 T5의 color 리매핑 2줄은 이제 필요 없다.

⚠️ **nvblox 파라미터는 노드 생성 시 1회만 읽는다**(`nvblox_node.cpp:195`).
`ros2 param set`으로는 안 바뀐다 — 고쳤으면 T5를 재시작한다.
T4·T6도 마찬가지로 재시작이 필요하다.

## 증상 → 어느 파일을 볼 것인가

| 증상 | 파일 | 파라미터 |
|---|---|---|
| 장애물이 사라졌는데 복셀이 남는다 | T5 | `tsdf_decay_factor`, `decay_tsdf_rate_hz`, `tsdf_set_free_distance_on_decayed` |
| 로봇에 가까이 간 손이 안 보인다 | T4 | `distance_threshold` |
| 로봇 몸이 장애물로 잡힌다 | T4 | `distance_threshold` (반대 방향) |
| 상자 밖 장애물을 통과한다 | T5+T6 | 감시상자 범위 (위 표) |
| 쓸데없는 복셀이 많다 | T5 | `workspace_bounds_*`, `max_integration_distance_m`, `esdf_integrator_max_site_distance_vox` |
| 계획이 자주 실패한다 | T6 | `max_attempts`, `num_trajopt_seeds` |
| 계획은 되는데 장애물을 통과한다 | T6 | `read_esdf_world` (False면 이 증상) |
| OMPL(octomap)만 이상하다 | T7 | `moveit_sensors_3d.yaml` |

## 아직 안 된 것

- **이 설정으로 실기를 안 돌렸다.** 값의 출처는 소스 코드와 계산이지 실측이 아니다.
- **z 하한 -0.05는 "base_link의 z=0이 테이블 상판"이라는 가정**에 서 있다. 틀리면
  테이블이 지도에서 빠지고 팔이 상판을 뚫는 경로가 나온다. 실기 전에 확인할 것.
- **실행 중 동적 회피는 여전히 안 된다.** T6은 계획 요청 1회당 지도를 1번만 읽는다.
  지도를 실시간으로 만든 것은 그 다음 단계(실행 중 재계획 루프)의 전제 조건일 뿐이다.
- 파이프라인 지연 하한 ≈ 0.6초 (카메라 0.10 + 세그멘터 0.27 + nvblox 0.1 + 계획 0.11).
  세그멘터 3.7 Hz가 병목이다.
