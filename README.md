# cobot2_ws — M0609 pick 파이프라인 실행 가이드

팀원용. 이 문서 하나만 보고 **켜고, 동작을 확인하고, 다른 PC에서도 전체 플로우를 재현**할 수 있게 쓴다.
제목이 하드웨어만 나열하던 것을 고쳤다 — 이 문서는 로봇·카메라뿐 아니라 인식(YOLO·GraspGenX)·
상태머신(pick_fsm)까지 **집는 동작 전체**를 다룬다.

- 대상: `m0609_rg2_bringup`·`m0609_rg2_moveit`(`src/cobot_rg2/rg2/`), `graspgenx_perception`,
  `pick_fsm`. cuMotion(옵션 경로)은 `config/testcommand.md`·`src/PACKAGES.md`(cumotion 절)가 단일 출처
- 전제: **환경(카메라 위치·조명)이 바뀌지 않는다.** 카메라를 옮기면 캘리브부터 다시 해야 한다 → [재캘리브](#재캘리브-카메라를-옮겼다면)
- **패키지별 상세 레퍼런스(인터페이스·파라미터·빌드·검증 상태)는 이 문서가 아니라
  [`src/PACKAGES.md`](src/PACKAGES.md)다** — 이 문서는 "켜고 재현하는 법"만 다룬다
- 최종 갱신: 2026-08-09 (제목·범위 수정, 설정 파일 지도 추가, `graspx.launch.py` 명령을
  실제 소스 기본값과 대조해 3곳 수정, 패키지별 README 4개를 `src/PACKAGES.md`로 통합,
  **cuMotion 실행 명령을 [6절](#6-cumotion--동적-장애물-회피-ompl-대체-플래너-실기-검증됨)로
  추가**(OMPL만 있던 데서 확장) — 아래 "문서 간 대조" 참고)

---

## 0. 빠른 재현 — 명령 한 벌로 전체 플로우 확인

터미널 7개를 순서대로 켜면 **로봇 기동 → 카메라 인식 → MoveIt 장애물 회피 → 물체 탐지 →
grasp 계산 → 집기 상태머신**까지, 이 ws의 전체 플로우가 **다른 PC에서도 그대로 재현**된다.
각 줄에 **역할 / 조정 가능한 파라미터 / 주의점**을 붙였다 — 막히면
[4절 기능 확인](#4-기능-확인-순서대로-각-단계-통과-후-다음으로)·[7절 상세](#7-상태머신--graspgenx--실행-흐름)로 내려간다.
cuMotion으로 계획하는 옵션 경로는 [6절](#6-cumotion--동적-장애물-회피-ompl-대체-플래너-실기-검증됨) 참고.

> 전제: [2절 준비](#2-준비-최초-1회)(빌드) 완료 + `ROS_DOMAIN_ID=93`(팀 규약 — 안 하면 노드끼리 안 보인다).
> 로봇·카메라·GPU가 없는 PC는 `mode:=virtual`로 [1]~[3]만 켜도 "MoveIt이 도는지"는 확인된다.
> [4]~[6]은 실기 카메라 + GPU가 있어야 뜬다.

```bash
# 모든 터미널 공통으로 먼저
cd ~/cobot2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=93
```

| # | 명령 | 역할 | 조정 파라미터 | 주의 |
|---|---|---|---|---|
| 1 | `ros2 launch m0609_rg2_bringup bringup.launch.py mode:=real host:=192.168.1.100 rviz:=false` | 로봇+RG2 기동, ros2_control, TF(world→base_link) | `mode`(real\|virtual), `host`(실기 IP) | 실기 없으면 `mode:=virtual` — Docker 에뮬레이터 자동 기동. `rviz:=false`는 [3]이 자기 RViz를 띄우기 때문(둘 다 켜면 RViz 2개가 octomap을 각각 렌더) |
| 2 | `ros2 launch m0609_rg2_bringup camera.launch.py` | D435i 드라이버 + 캘리브 TF(base_link→camera_link) | `dxyz`/`drpy`(캘리브 손보정), `depth_profile`/`color_profile`(해상도, 기본 `424x240x15`) | 카메라를 옮겼으면 먼저 [재캘리브](#재캘리브-카메라를-옮겼다면) |
| 3 | `ros2 launch m0609_rg2_moveit moveit.launch.py standalone:=false` | move_group + RViz MotionPlanning, 궤적 실행 | `octomap`(3D 장애물 감지 on/off), `use_sim_time`(실기는 `false` 고정) | `standalone:=false` 빠뜨리면 TF·관절값이 조용히 중복/덮어써진다(에러 없음) |
| 4 | `scripts/graspx_container.sh run_bridge:=false device:=0 publish_overlay:=true classes:='[39,41,44,46,47,49,64]'` | (컨테이너) YOLO-seg로 물체 탐지 | `classes`(COCO id 목록 — 볼 물체 종류) | `person(0)` 넣지 말 것 — yolo 경로엔 self-filter 없음 |
| 5 | `ros2 launch graspgenx_perception graspx.launch.py run_yolo:=false run_bridge:=true target_classes:=apple` | (호스트 GPU) 마스크 → 3D grasp pose 계산 | `target_classes`(집을 물체 이름) | `run_bridge:=true`를 빠뜨리면 **아무 노드도 안 뜬다**(`run_yolo`/`run_bridge` 기본값이 각각 `true`/`false`라 둘 다 꺼진 상태가 된다) — [4]는 컨테이너, [5]는 호스트라 헷갈리기 쉽다. `ROS_DOMAIN_ID=93` 빠뜨리면 [4]가 안 보인다 |
| 6 | `ros2 launch pick_fsm pick_fsm.launch.py grasp_source:=legacy_trigger voice:=false target:=apple` | 접근→집기 상태머신 (기본 dry_run — 계획만 하고 실행 안 함) | `target`(집을 물체), `grasp_source`(manual\|legacy_trigger\|voice) | 🚨 실기로 실제로 움직이려면 `dry_run:=false` 명시 + 사람 감시 필수 |
| 7 (선택) | `rqt --standalone pick_fsm` | 상태머신 감시 UI | — | — |

각 런치의 인자 전체·기본값은 [3절 런치 인자](#런치-인자) 표, 노드별 세부 파라미터는
[`md/launch-params.md`](md/launch-params.md)가 단일 출처다. **설정 파일이 실제로 어디에 있고
무엇을 담고 있는지는 [0-1절](#0-1-설정-파일-지도--뭘-고치면-뭐가-바뀌나)**. 조작 명령(`/pick/start`
등)과 GPU 없이 상태 전이만 보는 법(`grasp_source:=manual`)은 [7절](#7-상태머신--graspgenx--실행-흐름) 참고.

**기능별 검증 상태·결합점(한 곳 바꾸면 조용히 깨지는 것)·알려진 함정은 README가 아니라
[`md/state.md`](md/state.md)에 있다** — 재현 절차와 성격이 달라 분리했다.

### 문서 간 대조 (2026-08-09) — 이 절을 쓰면서 발견한 불일치

이 ws는 같은 실행 흐름을 **세 문서**(이 README §0/§6, `config/testcommand.md`,
당시 `src/pick_fsm/README.md` §2 — 2026-08-09 이후 `src/PACKAGES.md#pick_fsm` §2로 이관됨)가
각자 적고 있었다. 소스(`graspx.launch.py`, `bringup.launch.py`)와 대조해 아래를 고쳤다/남긴다:

| 발견 | 근거 | 조치 |
|---|---|---|
| 이 README의 [5] 명령이 **아무것도 안 띄우고 있었다** | `graspx.launch.py` `ARGS`: `run_yolo` 기본값 `true`, `run_bridge` 기본값 **`false`**. 옛 명령은 `run_yolo:=false`만 주고 `run_bridge:=true`를 안 줘서 둘 다 꺼졌다 | 위 표에서 `run_bridge:=true` 추가 |
| `md/launch-params.md` §3이 이 런치의 기본값을 **반대로** 적고 있었다 | 그 문서: `run_yolo`/`run_bridge` 기본값 `true`/`true`, `seg_source` 기본값 `geometric` — 전부 현재 소스와 다르다(`run_bridge`는 `false`, `seg_source`는 2026-08-08부터 `yolo`) | `md/launch-params.md` §3 수정함(아래) |
| `pick_fsm/README.md` §2 "3.5와 4" 절 — 4번 명령이 실제로는 `graspx.launch.py`를 안 쓴다 | 그 문서는 "둘 다 같은 `graspx.launch.py`를 쓴다"고 적었지만, 4번 명령(`ros2 run graspgenx_perception grasp_bridge_node`)은 런치 없이 노드를 직접 띄운다. 3.5번(`graspx_container.sh`)만 그 런치를 쓴다 | 문구 수정함(같은 날 이후 `src/PACKAGES.md`의 pick_fsm 절로 이관됨) |
| 카메라 해상도 기본값이 문서마다 다르다 (`424x240x15` vs `testcommand.md`의 `480x320x15`) | 서로 다른 경로를 위한 값이다 — 여기(MoveIt octomap 경로)는 옥토맵 단일 스레드 기준, `testcommand.md`는 nvblox 경로용 2026-08-08 결정값. **버그 아님**, 값을 맞추지 말 것 | 그대로 둠. `480x320x15`가 D435i 지원 프로파일인지는 `md/state.md` "⏭ 다음에 GPU PC 앞에 앉으면"에 미해결로 남아 있다 |

> ⚠️ 위 표의 `pick_fsm/README.md`·`md/launch-params.md` 참조는 **발견 당시(2026-08-09 이른 시각)
> 파일 위치 기준**이다. 같은 날 뒤이어 패키지별 README 4개를 `src/PACKAGES.md`로 통합했으므로
> "지금" 그 내용을 보려면 `src/PACKAGES.md`의 해당 절을 본다.

---

## 0-1. 설정 파일 지도 — 뭘 고치면 뭐가 바뀌나

값 자체는 여기 베끼지 않는다 — 아래 "단일 출처" 문서를 본다. 여기는 **어디에 뭐가 있는지**만.

| 실행 단위 | config 파일 | 담긴 것 | 단일 출처 |
|---|---|---|---|
| bringup (로봇) | `src/cobot_rg2/rg2/m0609_rg2_bringup/config/rg2_driver.yaml` | RG2 그리퍼 IP·포트·모델 (`mode:=real`에서만 로드) | `md/launch-params.md` §1 |
| bringup (로봇) | `src/cobot_rg2/rg2/m0609_rg2_bringup/config/T_cam2base.npy` | 캘리브 결과(카메라→로봇 변환). 재캘리브 시 이 파일만 교체 | [재캘리브](#재캘리브-카메라를-옮겼다면) |
| MoveIt | `src/cobot_rg2/rg2/m0609_rg2_moveit/config/{joint_limits,sensors_3d,kinematics,ompl_planning,moveit_controllers}.yaml` | 관절 제한·충돌 여유, octomap, IK, 플래너, 컨트롤러 이름 | `md/launch-params.md` §2 |
| YOLO / GraspGenX | (config yaml 없음 — 전부 런치 인자) | 대상 클래스·디바이스·신뢰도. 실행마다 바뀌는 값이라 파일로 안 뺐다 | `md/launch-params.md` §3 |
| pick_fsm | `src/pick_fsm/config/pick_fsm.yaml` (60여 개) | 안전(`dry_run` 등), MoveIt 연동, 접근/파지 자세, 그리퍼, 인식 소스 | `src/PACKAGES.md#pick_fsm` §5 |
| cuMotion (옵션 경로) | `config/{cumotion_segmenter,nvblox_realtime,cumotion_planner}.yaml` + `moveit_sensors_3d.yaml`(symlink) | 작업영역·감시상자, ESDF, 플래너 튜닝 | `src/PACKAGES.md#cumotion` |

> ⚠️ **yaml 고친 뒤 rebuild가 필요한지는 패키지 빌드타입에 따라 다르다.** `ament_cmake`
> (`m0609_rg2_bringup`·`m0609_rg2_moveit`)는 즉시 반영, `ament_python`(`pick_fsm`·`cumotion`)은
> `colcon build` 재실행 필요 — 상세는 `md/launch-params.md` 상단.

---

## 1. 전체 그림

```
[1] bringup.launch.py     로봇          M0609 + RG2 그리퍼, ros2_control, TF(world→base_link)
[2] camera.launch.py      카메라        D435i 드라이버 + 캘리브 TF(base_link→camera_link)
[3] moveit.launch.py      모션계획      move_group + RViz MotionPlanning + 궤적 컨트롤러
```

셋은 **독립 실행**이다. 통합 런치는 일부러 만들지 않았다 — 어느 단계가 깨졌는지 바로 보이게 하려는 것이다.
의존은 한 방향뿐: `[3]`이 `[1]`의 `controller_manager`를 기다린다. 그래서 **순서는 [1] → [3]**이다.
`[2]`는 아무 때나 켜도 된다.

---

## 2. 준비 (최초 1회)

### 2-1. 시스템 패키지

전체 의존성 목록(apt + pip)은 **`requirements.txt`** 한 파일에 모아뒀다.
처음 세팅하는 머신이면 그쪽을 먼저 본다. 아래는 **MoveIt 4개만** — 빠뜨렸을 때 증상이
특이해서 따로 남긴다.

```bash
sudo apt install \
  ros-humble-moveit-configs-utils \
  ros-humble-moveit-simple-controller-manager \
  ros-humble-moveit-ros-visualization \
  ros-humble-moveit-ros-perception
```

넷 다 없으면 각각 이렇게 실패한다 (증상으로 역추적할 때 쓰라고 남긴다):

| 없는 패키지 | 증상 |
|---|---|
| `moveit-configs-utils` | 런치 로드 자체가 실패 — `ModuleNotFoundError: No module named 'moveit_configs_utils'` |
| `moveit-simple-controller-manager` | `[FATAL] ... MoveItSimpleControllerManager ... does not exist` → Execute 불가 |
| `moveit-ros-visualization` | RViz는 뜨는데 **MotionPlanning 패널이 없다** |
| `moveit-ros-perception` | `Failed to load sensor: realsense_pointcloud ... PointCloudOctomapUpdater ... does not exist` → **3D 장애물 감지만 죽는다.** 계획·실행은 정상이라 모르고 지나치기 쉽다 |

> ⚠️ 이 랩탑은 계정 공유다. `sudo apt`로 ROS 패키지를 **제거·다운그레이드하지 말 것.** 위 셋은 추가 설치라 안전하다.

### 2-2. 빌드

```bash
cd ~/cobot2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select m0609_rg2_bringup m0609_rg2_moveit
```

### 2-3. 하드웨어 확인

| 항목 | 값 |
|---|---|
| 로봇 | M0609, 네임스페이스 `dsr01`, IP `192.168.1.100` |
| 그리퍼 | OnRobot RG2, Modbus TCP `192.168.1.1:502` |
| 카메라 | RealSense D435i (USB) |

`ROS_DOMAIN_ID`가 팀원마다 다르면 서로 안 보인다. **이 ws 의 규약은 `93` 이다**
(`src/PACKAGES.md#pick_fsm` §2 가 단일 출처). 호스트 셸은 기본이 0 이라 **매 터미널에서
`export ROS_DOMAIN_ID=93`** 을 해야 한다 — 컨테이너 이미지에는 93 이 박혀 있어 그쪽은
안 해도 된다. 확인: `echo $ROS_DOMAIN_ID`.

> ⚠️ **`~/M0609_VLA_system`(VLA) 은 도메인을 지정하지 않는다** — `scripts/`·`src/`·`config/`
> 어디에도 `ROS_DOMAIN_ID` 가 없어 기본 **0** 으로 뜬다(2026-08-08 grep 확인).
> 두 시스템을 붙일 때 **가장 먼저 터지는 지점**이다. 상세는
> [`md/plans/2026-08-08-vla-integration.md`](md/plans/2026-08-08-vla-integration.md).

---

## 3. 실행 — 터미널 3개

**모든 터미널에서 먼저:**
```bash
cd ~/cobot2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
```

```bash
# [터미널 1] 로봇
ros2 launch m0609_rg2_bringup bringup.launch.py mode:=real host:=192.168.1.100

# [터미널 2] 카메라 + 캘리브 TF
ros2 launch m0609_rg2_bringup camera.launch.py

# [터미널 3] MoveIt  (※ standalone:=false 필수)
ros2 launch m0609_rg2_moveit moveit.launch.py standalone:=false
```

로봇 없이 소프트웨어만 보려면 `mode:=virtual`(Docker DRCF 에뮬레이터가 자동으로 뜬다).

> 터미널 4개 더(cuMotion+nvblox 로 동적 장애물 회피, GraspGenX 없이 계획만) 쓰는 **옵션 경로**는
> [6절](#6-cumotion--동적-장애물-회피-ompl-대체-플래너-실기-검증됨)에 명령이 있다 —
> [7절](#7-상태머신--graspgenx--실행-흐름)이 다루는 pick 경로와는 별개다.

### 런치 인자

위 터미널 3개에서 자주 쓰는 것만 추렸다. **런치 7개 전부 + 어떤 값이 어느 config 파일에
있는지는 [`md/launch-params.md`](md/launch-params.md)** 가 단일 출처다.

| 런치 | 인자 | 기본값 | 설명 |
|---|---|---|---|
| `bringup` | `mode` | `virtual` | `real` \| `virtual` |
| | `host` | `127.0.0.10` | 실기 IP. **실기는 반드시 `192.168.1.100`** |
| | `port` | `12345` | |
| | `rviz` | `true` | bringup RViz. **moveit도 켤 거면 `false`** (RViz 2개가 octomap을 각각 렌더한다) |
| `camera` | `driver` | `true` | `false`면 TF만 발행 (realsense-viewer를 쓰는 중이거나 bag 재생 시) |
| | `dxyz` | `0 0 0` | 캘리브 평행이동 손보정 `"x y z"` (m, `base_link` 축) |
| | `drpy` | `0 0 0` | 캘리브 회전 손보정 `"roll pitch yaw"` (deg, `camera_link` 축). 테이블이 기울어 보이면 pitch부터 |
| | `depth_profile` | `424x240x15` | 낮게 잡은 값. 이유는 **MoveIt octomap updater가 단일 스레드**라서다 — GPU·코어 수를 늘려도 콜백 하나의 처리 시간은 안 줄어든다 |
| | `color_profile` | `424x240x15` | `align_depth`가 이 해상도를 따라가므로 depth만 낮춰선 의미 없다 |
| `moveit` | `standalone` | `true` | **bringup 위에 얹을 땐 `false`.** `true`는 로봇 없이 MoveIt만 볼 때 |
| | `rviz` | `true` | MoveIt RViz spawn 여부 |
| | `octomap` | `true` | RealSense로 3D 장애물 감지. `false`면 RViz로 직접 놓은 장애물만 반영 → [알고리즘 디버깅](#5-시뮬레이션에서-장애물-놓고-회피-디버깅) |
| | `use_sim_time` | `false` | `ros2 bag play --clock`과 한 짝. **실기는 false 고정** |
| | `cumotion` | `false` | 두 번째 플래닝 파이프라인 등록. Isaac ROS 컨테이너 전용 |

> **`standalone:=false`를 빠뜨리면** `robot_state_publisher`·`joint_state_publisher`·`static_transform_publisher`·`rviz2`가 bringup과 **중복 실행**된다. TF가 깜빡이고 실기 관절값이 시뮬값에 덮인다. 에러 메시지는 안 나온다.

---

## 4. 기능 확인 (순서대로, 각 단계 통과 후 다음으로)

### ✅ 체크 1 — 로봇이 살아 있나
```bash
ros2 control list_controllers -c /dsr01/controller_manager
```
기대 출력 (터미널 3까지 켠 상태):
```
joint_state_broadcaster  joint_state_broadcaster/JointStateBroadcaster          active
dsr_controller2          dsr_controller2/RobotController                        active
dsr_moveit_controller    joint_trajectory_controller/JointTrajectoryController  active
```
`dsr_moveit_controller`가 없으면 → 터미널 3이 안 떴거나 `standalone:=true`로 켠 것이다.

```bash
ros2 topic echo /dsr01/joint_states --once   # 관절 6개 값이 나와야 한다   ⚠️ 미검증
```

### ✅ 체크 2 — 카메라와 캘리브 TF
> ⚠️ **이 절의 두 명령은 아직 실행해 보지 않았다.** 아래 기대값은 실측이 아니라
> 설정에서 유도한 것이다. 처음 돌리는 사람은 결과가 다를 수 있다고 보고, 확인되면 이 경고를 지울 것.

```bash
ros2 topic hz /camera/camera/depth/color/points     # 포인트클라우드가 흐르나   ⚠️ 미검증
ros2 run tf2_ros tf2_echo base_link camera_link     # 캘리브 TF가 붙었나        ⚠️ 미검증
```
`tf2_echo` 기대값 — **`tf2_echo` 출력이 아니라 `calib_npy_to_tf.py`가 `T_cam2base.npy`에서
계산해 찍은 값이다.** 같은 변환이므로 일치해야 하지만, 대조는 아직 안 했다:
```
Translation: [1.148, 0.640, 0.678]        # base로부터 약 1.48 m
```
`Could not find a connection between 'base_link' and 'camera_link'`가 뜨면
→ `config/T_cam2base.npy`가 없는 것이다. 터미널 2 로그에 경고가 찍혀 있다.

**눈으로 확인:** bringup RViz(터미널 1)에서 PointCloud2가 로봇 모델과 **겹쳐서** 보여야 한다.
로봇 옆 엉뚱한 곳에 90° 돌아가 떠 있으면 좌표 규약 문제다 → `md/state.md`의 "알려진 함정" 참고.

### ✅ 체크 3 — 모션 계획
```bash
ros2 action list | grep follow
# → /dsr01/dsr_moveit_controller/follow_joint_trajectory   (이 이름이 정확히 나와야 한다)
```

RViz MotionPlanning 패널에서:
1. Displays → MotionPlanning → **Planning Request → Query Goal State** 체크
2. 주황색 로봇의 마커를 드래그해 목표 자세 지정 (**빨개지면** 충돌/IK 실패 — 그 자세는 계획 안 된다)
3. Planning 탭 → **Velocity Scaling / Accel Scaling을 0.1로 낮춘다** (실기 첫 시도 필수)
4. **Plan** → Planned Path 애니메이션으로 궤적 확인
5. 궤적이 납득되면 **Execute**

`Goal State` 드롭다운의 `all-zeros`는 SRDF에 정의된 자세다. 그리퍼는 `gripper_open` / `gripper_close`.

> 🚨 **실기 Execute는 사람이 지켜보는 상태에서만.** Plan 없이 `Plan & Execute`를 바로 누르지 말 것.

---

## 5. 시뮬레이션에서 장애물 놓고 회피 디버깅

회피 알고리즘만 따로 보려면 **카메라 클라우드를 끄고** 손으로 놓은 장애물만 남긴다.
실제 클라우드가 섞여 있으면 무엇이 경로를 막았는지 구분이 안 된다.

```bash
# 터미널 1 — 로봇 없이 가상 모드
ros2 launch m0609_rg2_bringup bringup.launch.py mode:=virtual

# 터미널 2 — 카메라는 켜지 않는다

# 터미널 3 — octomap 끄고 MoveIt만
ros2 launch m0609_rg2_moveit moveit.launch.py standalone:=false octomap:=false
```

### 장애물 놓기 (RViz, 코드 불필요)

MotionPlanning 패널 → **Scene Objects 탭**:

1. 우하단 **Box / Sphere / Cylinder / Cone** 중 선택 → **`+`** 버튼 → 씬 원점에 물체가 생긴다
2. 목록에서 물체를 클릭하면 **인터랙티브 마커**가 붙는다. 드래그해서 로봇 앞을 막아라
3. `Scale` 슬라이더와 `Position/Rotation` 숫자칸으로 크기·위치 미세조정
4. **⚠️ `Publish` 버튼을 반드시 누른다.** 이걸 안 누르면 RViz 화면에만 있고 move_group은 모른다
5. Planning 탭으로 돌아가 목표 자세를 장애물 반대편에 잡고 **Plan**

**확인 포인트** — 장애물을 통과하는 직선 경로 대신 **우회 경로**가 나와야 한다.
장애물에 완전히 갇힌 목표를 주면 계획이 실패해야 한다(`No motion plan found`). 실패가 나와야 정상이다.

### ⚠️ 프레임은 `base_link`다 (`world` 아님)

Scene Objects의 물체 프레임이나 스크립트의 `header.frame_id`는 **`base_link`**여야 한다.

SRDF에 `virtual_joint(FixedBase, parent_frame="world")`가 있어서 플래닝 프레임이 `world`일 것
같지만 **아니다.** MoveIt은 fixed 타입 virtual joint로 모델 프레임을 만들지 않아서 루트 링크가
그대로 플래닝 프레임이 된다. `world`는 TF에는 있어도 **planning scene은 모른다.**

```
frame_id='world'     → [ERROR] moveit_planning_scene: Unknown frame: world  → 장애물 무시
frame_id='base_link' → /monitored_planning_scene에 정상 등록
```
(2026-08-02 실측. 에러는 move_group 터미널에만 뜨고 RViz는 조용해서 놓치기 쉽다.)

### 반영이 안 될 때

물체는 보이는데 경로가 그대로 통과하면 **`Publish`를 안 눌렀거나** planning scene 발행이
꺼진 것이다. 후자는 다음으로 확인한다:

```bash
ros2 param get /move_group publish_geometry_updates   # → True 여야 한다
ros2 topic echo /monitored_planning_scene --once      # 물체가 목록에 있어야 한다
```

`publish_geometry_updates`를 포함한 4개 파라미터는 `moveit.launch.py`가 켜준다.
**기본값으로 두면 RViz에 상자는 보이는데 경로는 통과하는** 상태가 된다 — 가장 헷갈리는 실패다.

### 씬이 매번 사라진다

RViz 씬은 재시작하면 없어진다. 같은 장애물 배치를 반복해서 쓰려면 저장이 필요한데,
MoveIt의 씬 저장은 warehouse(mongodb)에 의존한다. 지금은 안 붙어 있다.
매번 손으로 놓는 게 번거로워지면 그때 붙이거나, 장애물을 발행하는 작은 노드를 만든다.

---

## 6. cuMotion — 동적 장애물 회피 (OMPL 대체 플래너, 실기 검증됨)

[7절](#7-상태머신--graspgenx--실행-흐름)의 pick 경로는 OMPL로 계획한다. 이 ws는 그 대신
**cuMotion**(실행 중 재계획으로 움직이는 장애물도 피함)도 실기로 검증했다 — 계획만이고
GraspGenX/pick_fsm과는 아직 안 엮었다. **필요할 때만 켜는 옵션 경로**다.

전제: **Isaac ROS 컨테이너(GPU) + [터미널 1(로봇)·터미널 2(카메라)]가 3절 기준으로 떠 있어야
한다.** 터미널 3(MoveIt)은 **cuMotion 파이프라인을 추가로 등록**해서 다시 띄운다(OMPL도
그대로 살아있다 — RViz 드롭다운이나 `planning_pipeline` 파라미터로 고른다).

```bash
# [터미널 3 다시] MoveIt — cumotion:=true 를 추가한다 (OMPL 파이프라인도 그대로 유지됨)
ros2 launch m0609_rg2_moveit moveit.launch.py standalone:=false octomap:=true cumotion:=true
```

```bash
# 컨테이너 진입 (매 셸 첫 줄)
source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash
source /workspaces/cobot2_ws/install_container/setup.bash
export ROS_DOMAIN_ID=93

# [터미널 4] robot_segmenter — 로봇 몸을 depth에서 지운다. 빠뜨리면 로봇이 자기 몸을 장애물로 본다
cd /workspaces/isaac_ros-dev && ros2 run isaac_ros_cumotion robot_segmenter_node --ros-args \
  --params-file /workspaces/cobot2_ws/config/cumotion_segmenter.yaml

# [터미널 5] nvblox — esdf_mode:=3d 는 yaml 안에 있다. 리매핑만 명령줄에 남긴다
ros2 run nvblox_ros nvblox_node --ros-args \
  --params-file /workspaces/cobot2_ws/config/nvblox_realtime.yaml \
  -r camera_0/depth/image:=/cumotion/camera_1/world_depth \
  -r camera_0/depth/camera_info:=/camera/camera/aligned_depth_to_color/camera_info

# [터미널 6] cuMotion 플래너
cd /workspaces/isaac_ros-dev && ros2 run isaac_ros_cumotion cumotion_planner_node --ros-args \
  --params-file /workspaces/cobot2_ws/config/cumotion_planner.yaml
```

```bash
# 검증 — 로봇 안 움직인다 (OMPL vs cuMotion 계획시간 비교, plan_only 고정)
python3 scripts/bench_planning_time.py --repeat 10
```

⚠️ **위 T4~T6은 일부러 `config/testcommand.md`의 "명령어만" 블록과 다르다.** 그 블록은
`isaac_ros_cumotion_robot_description`의 **기본 XRDF 인자만** 주고 `--params-file`을
안 준다 — 즉 이 ws가 튜닝해 둔 작업영역·감시상자(`config/*.yaml`)가 **적용되지 않는다.**
여기는 그 튜닝값이 실제로 먹는 버전이다. 두 버전이 왜 갈렸는지·어느 쪽이 맞는지는
`src/PACKAGES.md`(cumotion 절, "config 파일" 소절)가 단일 출처다.

`planning_pipeline:=isaac_ros_cumotion`로 pick_fsm에 cuMotion을 쓰게 하는 법, 노드별
증상→원인 표, 실기 실측(계획시간·복셀 수·실패율)은 아래에 정리했다:

| 알고 싶은 것 | 문서 |
|---|---|
| 인터페이스·파라미터·배포·증상표 (안정된 레퍼런스) | `src/PACKAGES.md#cumotion` |
| 실행 명령·노드 지도·단계별 검증 명령 (T0~T12) | `config/testcommand.md` |
| 날짜별 실기 디버깅 로그(루프 결함, 그리퍼 자기충돌, 복셀 붕괴 조사) | `md/cumotion-experiment-log.md` |

🔴 **`robot_segmenter_node`([터미널 4]) 없이는 절대 쓰지 말 것** — 로봇이 자기 몸을
장애물로 보고 계획이 전부 실패한다. 🔴 **cuMotion은 MoveIt octomap을 안 본다** — OMPL과
장애물 출처가 다르다(OMPL은 octomap, cuMotion은 nvblox ESDF). 종료는 올린 순서의 반대로
(터미널 6→5→4, 그다음 컨테이너 자체를 닫으면 새로 만들어야 한다는 뜻이니 주의).

---

## 7. 상태머신 + GraspGenX — 실행 흐름

`pick_fsm`(상태머신) + `graspgenx_perception`(인식·grasp 계산)로 물체를 골라 집는 전체 경로다.
**여기는 "무엇을 어디서 띄우나"까지만 남긴다** — 인자·트러블슈팅·설계 근거는
`src/PACKAGES.md`가 단일 출처다. 전제: **터미널 1(로봇)·터미널 3(MoveIt)이 3절 기준으로 이미 떠
있다**(`standalone:=false` 필수). 카메라(터미널 2)는 인식을 실제로 쓸 때만 필요하다.

| 터미널 | 무엇 | 어디서 | 왜 거기서만 | 상세 |
|---|---|---|---|---|
| 7 | YOLO 탐지 (`yolo_seg_node`) | **컨테이너** `od_kimkh` | 호스트엔 `ultralytics` 없음(넣지 말 것 — `cv_bridge` 깨짐) | `src/PACKAGES.md#graspgenx_perception` |
| 8 | grasp 계산 (`grasp_bridge_node`, GPU) | **호스트** | GraspGenX 워커를 `uv`로 띄우는데 컨테이너엔 `uv` 없음 | 〃 |
| 9 | 상태머신 + 안전감시 | 호스트 | — | `src/PACKAGES.md#pick_fsm` §2 |
| 10 | (선택) 감시 UI | 호스트 | — | 〃 §9 |

```bash
# [터미널 7] 컨테이너 — 탐지. person(0) 넣지 말 것(yolo 경로엔 self-filter 없음)
scripts/graspx_container.sh run_bridge:=false device:=0 publish_overlay:=true \
  classes:='[39,41,44,46,47,49,64]'

# [터미널 8] 호스트 — 파지 계산. run_bridge:=true 필수(기본값 false라 안 주면 아무것도 안 뜬다)
export ROS_DOMAIN_ID=93   # 빠뜨리면 [터미널 7]이 안 보인다
ros2 launch graspgenx_perception graspx.launch.py run_yolo:=false run_bridge:=true \
  target_classes:=apple

# [터미널 9] 상태머신 (계획만, 기본/안전 — 실기 실행은 dry_run:=false 명시 필요)
ros2 launch pick_fsm pick_fsm.launch.py grasp_source:=legacy_trigger voice:=false target:=apple

# [터미널 10] (선택) 감시 UI
rqt --standalone pick_fsm
```

조작 명령(`/pick/start`·`/pick/approve`·`/pick/abort`·`/pick/reset`), `grasp_source` 세 값의
차이, GPU 없이 상태 전이만 보는 법(`grasp_source:=manual`), 집을 물체 클래스 바꾸는 법
(`classes`/`target_classes`), 안 될 때 진단 순서 — 전부 `src/PACKAGES.md`(pick_fsm·
graspgenx_perception 절)가 단일 출처다. 여기서 값을 다시 적지 않는다.

---

## 재캘리브 (카메라를 옮겼다면)

카메라 위치가 바뀌면 `base_link→camera_link` TF가 전부 무효다. 순서:

```bash
# 1. 데이터 수집 (체커보드를 그리퍼에 부착, 자세마다 회전을 30° 이상 여러 축으로 섞을 것)
cd corecode/Calibration_Tutorial && python3 data_recording.py

# 2. eye-to-hand 캘리브 → T_cam2base.npy 생성
python3 eye2hand_calibration.py

# 3. 패키지로 복사 (symlink-install이면 rebuild 불필요)
cd ~/cobot2_ws
cp corecode/Calibration_Tutorial/T_cam2base.npy \
   src/cobot_rg2/rg2/m0609_rg2_bringup/config/T_cam2base.npy

# 4. 값만 미리 확인하고 싶으면
ros2 run m0609_rg2_bringup calib_npy_to_tf.py \
   corecode/Calibration_Tutorial/T_cam2base.npy base_link camera_link
```

**static TF 명령을 복사해서 하드코딩하지 말 것.** npy가 갱신돼도 그 숫자는 안 따라온다.
`camera.launch.py`가 매 실행마다 npy에서 계산한다.

수집 단계의 제약(회전 다양성, 조명, 보드 부착)은 `md/context/constraints.md`에 정리돼 있다.

---

## 참고 문서

- `md/context/constraints.md` — 실기로 알아낸 제약. **코드보다 이걸 먼저 읽어라**
- `md/state.md` — 현재 진행 상황과 다음 할 일 + **기능별 검증 상태·결합점·알려진 함정**
  (README에서 이관, 2026-08-09)
- `CLAUDE.md` — 이 워크스페이스 작업 규칙
- **[`src/PACKAGES.md`](src/PACKAGES.md)** — `cobot_rg2`·`cumotion`·`graspgenx_perception`·
  `pick_fsm` 4개 패키지의 인터페이스·파라미터·빌드·검증 상태. **`classes`/`target_classes`로
  무엇을 집을지 고르는 법**과 개체 단위 선정 설계("다음 방향" 절)도 여기(graspgenx_perception 절)
- 날짜별 실기 디버깅 로그: [`md/cumotion-experiment-log.md`](md/cumotion-experiment-log.md),
  [`md/graspgenx-perception-notes.md`](md/graspgenx-perception-notes.md)
