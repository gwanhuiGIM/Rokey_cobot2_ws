# M0609 + RG2 + RealSense D435i — 실행·기능확인 가이드

팀원용. 이 문서 하나만 보고 **켜고, 동작을 확인하고, 어디까지 되는지 판단**할 수 있게 쓴다.

- 대상: `m0609_rg2_bringup`, `m0609_rg2_moveit` (둘 다 `src/cobot_rg2/rg2/`)
- 전제: **환경(카메라 위치·조명)이 바뀌지 않는다.** 카메라를 옮기면 캘리브부터 다시 해야 한다 → [재캘리브](#재캘리브-카메라를-옮겼다면)
- 최종 갱신: 2026-08-02

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

`ROS_DOMAIN_ID`가 팀원마다 다르면 서로 안 보인다. 같은 값으로 맞춰라 (`echo $ROS_DOMAIN_ID`).

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

### 런치 인자

| 런치 | 인자 | 기본값 | 설명 |
|---|---|---|---|
| `bringup` | `mode` | `virtual` | `real` \| `virtual` |
| | `host` | `127.0.0.10` | 실기 IP. **실기는 반드시 `192.168.1.100`** |
| | `port` | `12345` | |
| `camera` | `driver` | `true` | `false`면 TF만 발행 (realsense-viewer를 쓰는 중이거나 bag 재생 시) |
| `moveit` | `standalone` | `true` | **bringup 위에 얹을 땐 `false`.** `true`는 로봇 없이 MoveIt만 볼 때 |
| | `rviz` | `true` | MoveIt RViz spawn 여부 |
| | `octomap` | `true` | RealSense로 3D 장애물 감지. `false`면 RViz로 직접 놓은 장애물만 반영 → [알고리즘 디버깅](#8-시뮬레이션에서-장애물-놓고-회피-디버깅) |

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
로봇 옆 엉뚱한 곳에 90° 돌아가 떠 있으면 좌표 규약 문제다 → [알려진 함정](#7-알려진-함정) 참고.

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

## 5. 현재 상태 — 어디까지 되나

| 기능 | 상태 | 근거 |
|---|---|---|
| 로봇 bringup (virtual) | ✅ 검증됨 | 컨트롤러 3개 active 확인 (2026-08-02) |
| 로봇 bringup (real) | ✅ 검증됨 | 실기 연결 후 MoveIt Plan/Execute까지 확인 (2026-08-02) |
| 카메라 드라이버 (`reals` alias) | ✅ 검증됨 | `/camera/camera/...` 토픽 실측 (2026-08-01) |
| 카메라 드라이버 (`camera.launch.py`) | ⚠️ **미검증** | 이번에 새로 만든 런치. `ros2 launch -p` 드라이런만 했고 **실기 카메라로 기동한 적 없다**. 포인트클라우드 토픽 leaf 이름은 RealSense 관례에 따른 추론 |
| 캘리브 TF (`base_link→camera_link`) | ⚠️ **잠정** | 값은 나오나 **카메라 마운트 강성 미확보**. 정밀도 실측은 마운트 확정 후 |
| MoveIt 경로 계획 | ✅ 검증됨 | RRTConnect, 0.019 s |
| MoveIt 궤적 실행(Execute) | ✅ **실기 검증됨** | 실제 로봇으로 Plan → Execute 확인 (2026-08-02) |
| RViz 수동 장애물 회피 | ✅ 설정 완료 | `publish_geometry_updates` 등 4개 활성. [8절](#8-시뮬레이션에서-장애물-놓고-회피-디버깅) |
| **3D 장애물 감지 (octomap)** | ⚠️ **설정 완료 · 미검증** | 파라미터 주입까지 확인. **`moveit-ros-perception` 설치 후 실측 필요** |
| 그리퍼 MoveIt 제어 | ❌ 미지원 | RG2는 `/onrobot/sendCommand` 서비스로 직접 제어. MoveIt 컨트롤러 없음 |

### 3D 장애물 감지 (octomap) — 상태

배선은 끝났다. `config/sensors_3d.yaml`이 RealSense 포인트클라우드를 octomap에 물리고,
`moveit.launch.py`가 `octomap:=true`(기본)일 때 move_group에 주입한다.

**남은 것은 플러그인 패키지 설치 하나뿐이다:**
```bash
sudo apt install ros-humble-moveit-ros-perception
```

설치 전 상태에서 파라미터가 제대로 들어가는 것까지는 확인했다:
```
$ ros2 param get /move_group sensors
String values are: ['realsense_pointcloud']
$ ros2 param get /move_group realsense_pointcloud.sensor_plugin
String value is: occupancy_map_monitor/PointCloudOctomapUpdater
$ ros2 param get /move_group octomap_frame
String value is: base_link      # ⚠️ 이 출력은 실측 당시 값. 이후 'world'로 변경했다(미재확인)
```
설치 전 로그는 `Failed to load sensor: realsense_pointcloud` — 설정은 읽혔고 클래스만 없다는 뜻이다.
설치 후에는 이 줄이 사라져야 한다.

**설치 후 확인 방법:**
1. 세 런치 모두 켠다 (카메라 필수 — 클라우드가 없으면 octomap도 없다)
2. RViz → Displays → MotionPlanning → **Scene Geometry → Show Scene Geometry** 켜기
   → 카메라가 보는 물체가 **복셀 덩어리**로 나타나야 한다
3. 그 복셀을 통과하는 목표 자세를 잡고 Plan → 경로가 **돌아가야** 한다

**튜닝값은 `config/sensors_3d.yaml`에 `[튜닝]` 주석으로 표시해뒀다.**
자기 팔이 장애물로 잡히면 `padding_offset`을, 뒷벽까지 잡히면 `max_range`를 먼저 본다.

> 🚨 **설치·검증 전까지 이걸 장애물 회피로 믿지 말 것.** 지금 계획은 URDF 자기 충돌과
> RViz에 직접 놓은 물체만 피한다. 카메라가 보는 실물은 그대로 통과한다.

---

## 6. 결합점 — 한 곳만 바꾸면 조용히 깨지는 것들

**에러 없이 기능만 죽는다.** 건드리기 전에 짝을 확인하라.

| # | 값 | 나오는 곳 | 어긋나면 |
|---|---|---|---|
| ① | 네임스페이스 `dsr01` | `bringup.launch.py`(`namespace=`), `moveit_controllers.yaml`(컨트롤러 이름 `/dsr01/...`), `moveit.launch.py`(`-c /dsr01/controller_manager`) — **3곳** | **Plan은 되고 Execute만 ABORTED.** 실제로 겪은 버그다 |
| ② | 캘리브 `T_cam2base.npy` | `corecode/Calibration_Tutorial/`(생성) → `m0609_rg2_bringup/config/`(소비). 동기화는 **수동 `cp` 하나뿐** | 옛 값으로 TF가 발행된다. 340 mm 어긋난 전례 있음 |
| ③ | xacro 파일명 `m0609_with_rg2.urdf.xacro` | `bringup.launch.py`, `moveit.launch.py` 양쪽이 경로로 직접 읽는다 | moveit이 런타임에 깨진다 |
| ④ | 관절 이름 `joint_1..6` | SRDF, `dsr_controller2.yaml`의 JTC 설정, `moveit_controllers.yaml` | 궤적이 컨트롤러에 거부된다 |

---

## 7. 알려진 함정

**포인트클라우드가 로봇 옆에 90° 돌아가서 뜬다**
npy는 OpenCV **optical** 규약(z=전방), ROS `camera_link`는 REP-103 **body** 규약(x=전방)이다.
`calib_npy_to_tf.py`가 기본으로 보정한다. `inv(T)` 문제가 아니다 — **부호를 만지지 말 것.**
지문: 출력 RPY의 roll ≈ ±90°. 자세한 채점표는 `md/context/constraints.md`.

**`ros2 launch dsr_moveit_config_m0609 demo.launch.py`가 안 된다 — 이건 쓰는 게 아니다**
① `MoveItConfigsBuilder("m0609")`가 관례상 `m0609_moveit_config` 패키지를 찾는데 실제 이름은
`dsr_moveit_config_m0609`다 (업스트림 버그). ② 고쳐도 그 config의 URDF엔 **RG2가 없다.**
→ 이 ws에서는 `m0609_rg2_moveit`을 쓴다.

**Ctrl-C로 끌 때 segfault + 스택트레이스가 쏟아진다**
MoveIt 2.5.9의 알려진 종료 순서 버그(`class_loader` 언로드 타이밍). 기능에 영향 없다. 무시.

**RViz 창이 두 개 뜬다**
bringup RViz(관측: RobotModel/TF/PointCloud2)와 MoveIt RViz(조작: MotionPlanning)는 목적이 다르다.
다만 노드 이름이 둘 다 `rviz2`라 충돌 경고가 뜬다. 하나만 쓰려면 `moveit.launch.py rviz:=false`.

**realsense-viewer와 ROS 드라이버는 동시에 못 쓴다** (USB 장치를 독점한다)

**virtual 모드에서 에뮬레이터가 안 뜬다**
이전 run의 Docker 컨테이너가 `Exited`로 남은 경우다. bringup이 자동으로 지우지만, 수동은
`docker rm -f dsr01_emulator`.

---

## 8. 시뮬레이션에서 장애물 놓고 회피 디버깅

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
- `md/state.md` — 현재 진행 상황과 다음 할 일
- `CLAUDE.md` — 이 워크스페이스 작업 규칙
