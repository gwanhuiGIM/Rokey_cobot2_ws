# cuMotion / nvblox 브링업 명령서 (2026-08-05)

> 목적: Isaac ROS 3.2 컨테이너에서 cuMotion을 M0609+RG2로 띄우고, **OMPL 대비 계획 시간**을 잰다.
> 그 숫자가 [[ws/cobot2/plans/2026-08-05-foundationpose-graspgenx-pick]]의 (a) 계획시점 우회 /
> (b) 실행중 stop→재계획 중 무엇으로 갈지를 정한다. 추측 말고 여기서 측정한다.
>
> ⚠️ 아래 명령 중 **컨테이너 안에서 실제로 실행해 검증한 것은 하나도 없다.** 전부 소스를 읽고 구성한 것이다.
> 실패하면 그 자리에서 에러를 기록하고 이 문서를 고친다.

---

## 0. 호스트에 미리 배치해 둔 것 (2026-08-05 완료)

컨테이너는 **`~/cobot2_ws/isaac_ros-dev` 하나만** 마운트한다(`run_dev.sh:288`
`-v $ISAAC_ROS_DEV_DIR:/workspaces/isaac_ros-dev`). `~/cobot2_ws/src`는 **안 보인다.**
그래서 필요한 파일을 마운트 안쪽으로 복사해 뒀다.

| 호스트 경로 | 컨테이너 경로 | 용도 |
|---|---|---|
| `isaac_ros-dev/m0609/m0609_kinematics.urdf` | `/workspaces/isaac_ros-dev/m0609/m0609_kinematics.urdf` | **cuRobo용.** visual/collision 34개를 제거해 `package://` 0개 — 컨테이너에 `dsr_description2`가 없어도 파싱된다. 충돌 형상은 XRDF의 구에서 온다 |
| `isaac_ros-dev/m0609/m0609_with_rg2.urdf` | 〃 | 전체 URDF(메시 포함). 폴백용 |
| `isaac_ros_cumotion_robot_description/xrdf/m0609_rg2.xrdf` | 〃 | ⚠️ **cuMotion은 XRDF를 임의 경로가 아니라 이 패키지의 `xrdf/`에서 파일명으로 찾는다** (`update_kinematics.py:62-64`). 그래서 여기 복사했다 |

> **심링크 금지.** `~/cobot2_ws/src`를 가리키는 심링크는 컨테이너 안에서 대상 경로가 없어 깨진다. 반드시 복사한다.
>
> XRDF 정본은 `src/cobot_rg2/rg2/m0609_rg2_moveit/config/m0609_rg2.xrdf`다. 고치면 위 두 곳에 다시 복사한다.

---

## 1. 게이트 A — 컨테이너가 GPU를 보는가

컨테이너 진입 후:

```bash
nvidia-smi                       # RTX 4060이 나와야 한다
ls /workspaces/isaac_ros-dev/m0609/   # urdf 2개 + xrdf 가 보여야 한다
export ROS_DOMAIN_ID=93          # ⚠️ 컨테이너 안에서 매번. --network host라 밖 노드와 통신된다
ros2 node list                   # 호스트에서 bringup을 띄워 뒀다면 /dsr01/* 가 보인다
```

**통과 못 하면 여기서 멈춘다.** GPU가 안 보이면 아래는 전부 무의미하다.

---

## 2. 게이트 B — 빌드

### 2-0. 선행 3가지 (2026-08-05 실기에서 전부 걸렸다)

**① cuRobo는 git 서브모듈이다 — 따로 받아야 한다.**
호스트에서 `--depth 1 --branch v3.2-14`로 클론했기 때문에 서브모듈이 비어 있다.
`git submodule status` 앞에 `-`가 붙어 있으면 미초기화 상태다.

```bash
cd /workspaces/isaac_ros-dev/src/isaac_ros_cumotion
git submodule update --init --recursive     # curobo_core/curobo 를 받는다
```

**② `isaac_ros-dev/` 루트에 `COLCON_IGNORE`가 있다 → colcon이 아무 패키지도 못 본다.**
호스트 워크스페이스(`~/cobot2_ws`) 빌드가 Isaac 패키지를 집지 않게 하려고 둔 파일인데,
마운트로 컨테이너에도 그대로 보인다. **지우면 안 된다**(호스트 빌드가 오염된다).
대신 `--base-paths src`로 한 단계 아래에서 스캔한다.

**③ `src/isaac_ros_common.bak/`가 모든 패키지를 중복시킨다** → colcon이 중복 패키지명으로 실패.
**2026-08-05에 조사 후 삭제했다.** 조사 결과(다시 만들지 않기 위해 기록):
- 내용은 `isaac_ros_common` @ `v3.2-14`와 **동일한 커밋**(`fcf4d9e`), 고유 파일은 3개뿐이었다
- `docker/Dockerfile.{x86_64,aarch64}.lightninglink` — 커스터마이징이 **아니라 복사 사고**다.
  원본은 `Dockerfile.x86_64 -> Dockerfile.base` **심링크**인데, `.bak` 쪽은 "Dockerfile.base"라는
  **문자열이 든 15바이트 일반 파일**이었다(심링크 미보존 복사). 쓸모없다
- `isaac_ros_common/scripts/.isaac_ros_common-config` — **이것만 의미가 있었다.** 아래에 옮겨 적는다

### 컨테이너 이미지에 RealSense 레이어를 넣는 법 (지금은 안 들어가 있다)

`run_dev.sh:37`의 기본값은 `IMAGE_KEY=ros2_humble`이라 현재 이미지는 **`x86_64.ros2_humble`**,
즉 **컨테이너 안에 RealSense 드라이버가 없다**. 넣으려면 `run_dev.sh`와 같은 디렉토리에:

```bash
echo 'CONFIG_IMAGE_KEY=ros2_humble.realsense' \
  > src/isaac_ros_common/isaac_ros_common/scripts/.isaac_ros_common-config
```

(`run_dev.sh:27-28`이 이 파일을 source하고 `:40-41`이 `IMAGE_KEY`를 덮는다.
사용 가능한 레이어: `base`, `realsense`, `ros2_humble`.)

> **현재 구성에서는 필요 없다.** RealSense 드라이버는 **호스트**에서 돌고(`reals`),
> 컨테이너는 `--network host`로 `/camera/*` 토픽을 구독만 하면 된다. nvblox도 드라이버가
> 아니라 depth **토픽**을 먹는다. 컨테이너 안에서 드라이버를 직접 띄워야 할 때만 위 설정을 쓴다
> (예: 호스트가 없는 클라우드 GPU — `[[ws/cobot2/plans/2026-08-04-gpu-rental-checklist]]`).

### 2-1. 빌드

```bash
cd /workspaces/isaac_ros-dev
colcon build --symlink-install --base-paths src \
  --packages-up-to isaac_ros_cumotion_moveit isaac_ros_cumotion_robot_description
source install/setup.bash
```

실제로 빌드되는 것 8개(위상순, 2026-08-05 확인):
`isaac_ros_common` → `curobo_core` → `isaac_ros_cumotion_interfaces` →
`isaac_ros_cumotion_python_utils` → `isaac_ros_cumotion_robot_description` →
`nvblox_msgs` → `isaac_ros_cumotion` → `isaac_ros_cumotion_moveit`

`curobo_core`가 CUDA 커널을 컴파일하므로 **처음엔 오래 걸린다**(수십 분 각오).

```bash
python3 -c "import curobo; print('curobo OK', curobo.__file__)"
```

### 2-2. ⛔ `curobo_core` 빌드 실패 — `std::lerp` 충돌 (2026-08-05 발생·해결)

```
helper_math.h:1130: error: 'float lerp(float, float, float)' conflicts with a previous declaration
/usr/include/c++/11/cmath:1911: note: previous declaration 'constexpr float std::lerp(float, float, float)'
```

**원인은 CUDA가 아니라 C++ 표준이다.** C++20이 `std::lerp`를 추가했고 libstdc++의 `<math.h>`가
그것을 전역 네임스페이스로 주입한다. torch `cpp_extension`이 `-std=c++20`을 강제하므로
(cuRobo `setup.py`엔 `-std` 지정이 없다 — 확인함) cuRobo의 전역 `lerp`와 충돌한다.
cuRobo 커밋 `36ea382`는 2024-11-22자로 이 조합보다 오래됐다.

**해결:** 그 스칼라 `lerp`는 **cuRobo 전체에서 한 번도 호출되지 않는다**(grep 확인).
NVIDIA `helper_math.h` 샘플에서 딸려온 죽은 코드다. `#if __cplusplus < 202002L`로 감쌌다.
`float2/3/4` 오버로드는 인자 타입이 달라 충돌하지 않으므로 건드리지 않았다.

⚠️ **이 파일은 git 서브모듈(`curobo_core/curobo`)이라 재-init하면 패치가 사라진다.**
`patches/curobo-helper_math-cpp20-lerp.patch`에 저장해 뒀다. 날아가면:

```bash
cd /workspaces/isaac_ros-dev/src/isaac_ros_cumotion/curobo_core/curobo
git apply ~/cobot2_ws/patches/curobo-helper_math-cpp20-lerp.patch   # 호스트 경로 기준
```

**재빌드: `build/`를 지울 필요 없다.** (이전 판에 "복사되므로 지워야 한다"고 적었던 것은 **틀렸다**.)
`--symlink-install`이라 colcon은 소스를 복사하지 않고 **심링크**한다 — 2026-08-05 실측:

```
build/curobo_core/curobo/src/curobo
  -> /workspaces/isaac_ros-dev/src/isaac_ros_cumotion/curobo_core/curobo/src/curobo
```

즉 패치가 즉시 반영되고, ninja가 헤더 mtime 변화를 보고 알아서 재컴파일한다.
그냥 다시 돌리면 된다:

```bash
cd /workspaces/isaac_ros-dev
colcon build --symlink-install --base-paths src \
  --packages-up-to isaac_ros_cumotion_moveit isaac_ros_cumotion_robot_description
```

그래도 이상하면 그때 `rm -rf build/curobo_core install/curobo_core`로 초기화한다
(단 CUDA 커널을 처음부터 다시 컴파일하므로 수십 분을 다시 쓴다 — 먼저 그냥 재빌드해 볼 것).

> 대안(미시도): `-std=c++17`을 강제하는 방법도 있으나, torch가 c++20으로 빌드돼 있으면
> 헤더 호환이 깨질 수 있다. 죽은 코드 한 덩이를 막는 쪽이 blast radius가 작다.

> ⚠️ **`--base-paths src`를 빼먹으면** `Package '...' specified with --packages-up-to was not found`가
> 뜬다. 패키지가 없는 게 아니라 루트 `COLCON_IGNORE` 때문에 **스캔 자체를 안 한 것**이다.

### 2-3. ⛔ 런타임 실패 — `module 'warp' has no attribute 'torch'` (2026-08-05 발생)

게이트 D 첫 실행에서 `load_motion_gen()` 안에서 죽는다:

```
world_mesh.py:67  self._wp_device = wp.torch.device_from_torch(self.tensor_args.device)
warp/__init__.py:603 in __getattr__
AttributeError: module 'warp' has no attribute 'torch'
```

**2-2와 같은 종류의 문제다 — 의존성 버전 상한이 없어서 생긴 어긋남.**

- 컨테이너에 깔린 warp: **1.16.0** (실측)
- cuRobo 커밋: `36ea382` **2024-11-22**. 버전 게이트가 `1.2.1`까지밖에 모른다(`util/warp.py:60`)
- 원인: cuRobo `setup.cfg:53`이 `warp-lang>=0.9.0`으로 **상한을 안 걸었다.**
  cuRobo 자신의 dockerfile도 `pip3 install warp-lang`을 핀 없이 부른다
  (`docker/aarch64.dockerfile:137`) → pip이 최신을 끌어왔다. **14개 마이너 버전 드리프트.**

#### ❌ 안 되는 해결: `import warp.torch` 추가

처음에 `world_mesh.py`에 명시 임포트를 넣어 봤으나 **warp 1.16.0에는 `warp.torch` 모듈이
아예 없다** — `AttributeError`가 `ModuleNotFoundError`로 바뀔 뿐이다. 패치는 남겨 뒀지만
`try/except ImportError`로 삼키게 해서, 원래 코드와 같은 지점(67행)에서 죽도록 무해화했다
(`patches/curobo-warp-torch-import.patch`). **다운그레이드 후에는 이 임포트가 정상 동작한다.**

#### ✅ 해결: warp 다운그레이드

```bash
pip3 install 'warp-lang==1.5.0'
```

- 1.5.0은 cuRobo 커밋과 **동시대**(2024-12)이고 `>1.2.1`이라 `warp_support_kernel_key` 게이트가
  최신 경로를 탄다
- **colcon 재빌드 불필요.** warp 커널은 warp 자체 JIT 캐시(`~/.cache/warp`)라 첫 실행에서
  몇 분 걸릴 뿐, `curobolib`의 CUDA 확장(2-2에서 컴파일한 것)과는 무관하다
- 컨테이너 안에서 warp를 쓰는 건 **cuRobo뿐**이다 (`src/` 전수 grep: 다른 소비자는
  `GraspGenX/end2end/dynamic_playback.py` 하나인데 그건 호스트 `uv` 트랙이라 무관)
- ⚠️ **이미지 밖 변경이라 컨테이너를 새로 만들면 날아간다.** 재현 절차를 §0에 적어 둘 것

1.5.0에서도 깨지면 1.4.2로 한 칸 더 내린다. 그래도 안 되면 warp 최신 API 위치를 찾는다:

```bash
python3 -c "
import warp, pkgutil
print('warp', warp.config.version, warp.__file__)
print('submodules:', [m.name for m in pkgutil.iter_modules(warp.__path__)])
print('top-level device_from_torch:', hasattr(warp, 'device_from_torch'))
"
```

---

## 3. 게이트 C — cuRobo가 우리 XRDF를 읽는가 ⭐

**로봇도 nvblox도 필요 없다. 가장 값싸고 가장 중요한 검증이다.**
XRDF가 틀렸으면 여기서 죽고, 아래 단계를 아무리 해도 안 된다.

### 3-1. XRDF 파싱 (2026-08-05 통과 ✅)

```bash
python3 - <<'EOF'
from curobo.cuda_robot_model.util import load_robot_yaml
from curobo.types.file_path import ContentPath

cp = ContentPath(
    robot_xrdf_absolute_path='/workspaces/isaac_ros-dev/m0609/m0609_rg2.xrdf',
    robot_urdf_absolute_path='/workspaces/isaac_ros-dev/m0609/m0609_kinematics.urdf',
)
cfg = load_robot_yaml(cp)
print('XRDF 로드 OK')
k = cfg['robot_cfg']['kinematics']
print('  base   :', k.get('base_link'))
print('  ee     :', k.get('ee_link'))
print('  joints :', k.get('cspace', {}).get('joint_names'))
EOF
```

실기 결과 — `base_link` / `tool0` / `joint_1..6 + rg2_finger_joint`.

> **`rg2_finger_joint`가 7번째로 찍히는 건 정상이다. 계획 DOF가 7이 된 게 아니다.**
> `xrdf_utils.py:136`이 `all_joint_names = active_joints + lock_joints`로 합쳐서 찍기 때문이다.
> XRDF `cspace.joint_names`는 6개뿐이고, `rg2_finger_joint`는 `default_joint_positions`의
> `-0.558505`(gripper_open)로 **lock** 된다(`:126-128`). RG2의 나머지 5개 관절은 URDF mimic이라
> `get_controlled_joint_names()`에 애초에 안 들어오고, cuRobo가 mimic을
> 자체 처리한다(`urdf_kinematics_parser.py:166-202`). → **cuMotion은 6 DOF로 계획한다.**

### 3-2. 정기구학 + 구 검증 ← **지금 여기**

스크립트는 호스트에 두었다(바인드 마운트라 컨테이너에서 그대로 보인다):
`isaac_ros-dev/m0609/gate_c.py`

```bash
cd /workspaces/isaac_ros-dev && source install/setup.bash
python3 m0609/gate_c.py
```

**판정 (기대값은 스크립트가 직접 대조해 합/불을 찍는다):**

| 항목 | 기대 | 근거 |
|---|---|---|
| 계획 DOF | **6** | XRDF `cspace.joint_names` |
| 충돌 구 개수 | **75** | XRDF `geometry:` 절 실측 (base 10, link_1..6 = 4/5/4/6/5/4, RG2 37) |
| all-zeros `tool0` 위치 | **[0.0001, 0.0064, 1.0345] m** | ↓ |
| 자기충돌 무시쌍 | **34** | SRDF `disable_collisions` |

> EE 기대값은 `tf2_echo`가 아니라 **URDF 관절 origin을 직접 곱해 호스트에서 따로 계산한 값**이다
> (`joint_1..6` 전부 0일 때 체인이 수직으로 서서 z=1.0345 m). cuRobo와 무관한 기준점이라
> 대조에 쓸 수 있다. `tf2_echo base_link tool0`은 **현재 자세**를 주므로, 로봇이 전자세(all-zeros)에
> 있지 않으면 이 값과 안 맞는 게 당연하다 — 그걸로 판정하지 말 것.

**불합격일 때:**
- 구 개수 ≠ 75 → XRDF `geometry:` 절이 안 읽힌 것 (`collision.geometry` 이름 오타 의심)
- EE 오차 > 1 mm → URDF/XRDF의 base_link·tool0 지정이 어긋난 것
- 반지름이 이상하게 크면 `scripts/fit_spheres.py`를 다시 돌린다

---

## 4. 게이트 D — cuMotion 플래너 노드 (nvblox 없이) ← **지금 여기**

`read_esdf_world`가 **기본 False**라 nvblox 없이 MoveIt planning scene(=기존 octomap)을 쓴다.
**두 개를 동시에 켜지 않는다** — 실패 시 원인 분리가 안 된다.

### 4-0. ⛔ 먼저 컨테이너 안에서 이것부터 확인한다

```bash
echo "ROS_DOMAIN_ID=[$ROS_DOMAIN_ID]"
```

**비어 있으면 노드는 뜨지만 로봇을 못 본다.** `run_dev.sh:230`이 `-e ROS_DOMAIN_ID`로
**호스트 환경변수를 그대로 넘기는데**, 이 랩탑의 `~/.bashrc`는 `rdm` alias를 쳐야만
`ROS_DOMAIN_ID=93`을 설정한다. `rdm` 없이 연 터미널에서 `run_dev.sh`를 띄웠으면 컨테이너는
도메인 0이다. 컨테이너 재시작 없이 그 안에서 고칠 수 있다:

```bash
export ROS_DOMAIN_ID=93     # 이후에 띄우는 노드에만 적용된다
ros2 topic echo /joint_states --once   # 7개 관절이 나와야 정상
```

`/joint_states`는 호스트의 `joint_state_publisher`가
`/dsr01/joint_states` + `/gripper_joint_states`를 합쳐 내보내는 토픽이다
(`bringup.launch.py:175-179`). 플래너 노드의 기본값이 `/joint_states`라 **리맵이 필요 없다.**

### 4-1. 실행

```bash
cd /workspaces/isaac_ros-dev && source install/setup.bash
ros2 run isaac_ros_cumotion cumotion_planner_node --ros-args \
  -p robot:=m0609_rg2.xrdf \
  -p urdf_path:=/workspaces/isaac_ros-dev/m0609/m0609_kinematics.urdf \
  -p read_esdf_world:=False \
  -p publish_curobo_world_as_voxels:=True \
  -p voxel_size:=0.02 \
  -p publish_voxel_size:=0.02
```

파라미터 이름은 **소스에서 확인했다**(`cumotion_planner.py:62-95`). 주의할 점:

- `robot:=` 은 **파일명만** 준다(경로 아님, §0 참고)
- **`tool_frame`은 주지 않는다.** 안 주면 XRDF `tool_frames[0]`(=`tool0`)을 쓰는데
  (`:131-136`, `:330`), SRDF `manipulator` 그룹의 `tip_link`도 `tool0`이라 이미 일치한다.
  불일치하면 `:771`이 `relaunch node with tool_frame = ...` 로 알려준다
- `voxel_size`(계획용)와 `publish_voxel_size`(시각화용)는 **별개 파라미터**다. 둘 다 기본 0.05
- **뜨는 데 시간이 걸린다.** `load_motion_gen()` → `warmup()`이 액션 서버보다 먼저 돈다(`:260-261`).
  cuRobo 커널 워밍업이라 첫 실행은 수십 초 걸릴 수 있다 — 멈춘 게 아니다

### 4-2. 합격 판정 (2026-08-05 통과 ✅)

```bash
ros2 action list | grep cumotion      # /cumotion/move_group
```

실기 로그:
```
[INFO] warming up cuMotion, wait until ready
[INFO] cuMotion is ready for planning queries!     ← 워밍업 1.7초
```

- 액션 이름은 `cumotion/move_group`, 타입 `moveit_msgs/action/MoveGroup` (`:279`)
- `opt_base.py:298`의 sparse tensor UserWarning은 **무시해도 된다** (torch 내부 경고)
- 이 단계에서 로봇은 **움직이지 않는다.** move_group이 이 액션을 부르기 전까지는 대기만 한다

> **`/curobo/voxels`에 메시지가 안 오는 것은 정상이다.** 이 퍼블리시는 `execute_callback`
> 안에서만 일어난다(`:665 → :594 → :622`). **계획 요청이 와야 나온다.** 게다가
> `get_subscription_count() > 0`일 때만 계산한다(`:623`). 대기 중 `ros2 topic hz`로
> 판정하려던 이전 판의 기준은 **틀렸다** — 게이트 E에서 첫 계획을 돌린 뒤에 본다.

---

## 4-3. 🔴 여기서 발견한 것 — **cuMotion은 octomap을 아예 안 본다**

`read_esdf_world:=False`일 때 cuMotion이 세계를 받는 경로는 **한 곳뿐**이다:

```python
# cumotion_planner.py:662-665
scene = goal_handle.request.planning_options.planning_scene_diff
world_objects = scene.world.collision_objects        # ← collision_objects "만"
world_update_status = self.update_world_objects(world_objects)
```

`cumotion_planner.py` 전체에 **`octomap` 문자열이 0건**이다(grep 확인).
MoveIt 플러그인은 `getPlanningSceneMsg()`로 **전체** 씬을 보내주는데
(`cumotion_move_group_client.cpp:72,81`), 받는 쪽이 `world.octomap` 필드를 그냥 버린다.

### 이게 이 프로젝트에 갖는 의미

| | OMPL (현재) | cuMotion + `read_esdf_world:=False` | cuMotion + nvblox |
|---|---|---|---|
| 테이블·박스(CollisionObject) | 본다 | 본다 | 본다 |
| **사람 팔 (octomap 복셀)** | **본다** | **❌ 못 본다** | 본다 (ESDF) |

**우리 프로젝트의 목적(사람 팔 우회)에 직결된다.** RealSense가 만드는 사람 팔은
지금 octomap 복셀로만 존재하므로, 게이트 E에서 cuMotion으로 전환하면
**계획은 성공하는데 사람 팔을 통과하는 궤적이 나온다.**

⚠️ **가장 위험한 실패 방식이다 — 성공처럼 보인다.** 계획 시간은 빨라지고 에러도 안 나므로,
"cuMotion이 더 빠르다"는 결론만 남고 장애물을 안 봤다는 사실은 드러나지 않는다.

### 따라서 계획 수정

- **게이트 F(nvblox)는 선택이 아니라 필수다.** 이전 판이 "게이트 E 통과 후"의 부가 단계로
  적어 둔 것은 틀렸다. cuMotion이 미모델링 장애물을 보는 **유일한** 경로다
- 게이트 E는 **속도 비교 전용**으로만 쓴다(§7의 OMPL vs cuMotion 계획 시간).
  **이 단계에서 사람 팔을 넣고 실기를 돌리지 않는다**
- 게이트 E 중 실기 검증이 필요하면, 장애물을 **명시적 CollisionObject**로 넣어야 한다
  (계획서 `2026-08-05-foundationpose-graspgenx-pick.md` Phase 0-G의 ACM/CollisionObject 작업과
  같은 배선이다 — 중복 작업 아님)

---

## 5. 게이트 E — MoveIt 파이프라인에 붙이기

`ur.launch.py`가 보여주는 표준 방식은 **두 파이프라인 공존**이다:

```python
{'planning_pipelines': ['ompl', 'isaac_ros_cumotion']},
{'isaac_ros_cumotion': <isaac_ros_cumotion_moveit/config/isaac_ros_cumotion_planning.yaml 내용>}
```

→ **RViz MotionPlanning 패널의 플래너 드롭다운에서 OMPL ↔ cuMotion을 전환**할 수 있다.
같은 목표로 두 번 계획해 시간을 비교하는 게 이번 측정의 핵심이다.

`moveit.launch.py`의 `planning_pipelines` 딕셔너리(103-107행)에 추가하면 된다.

### ⚠️ 마운트 문제 — 지금 컨테이너로는 안 된다

`move_group`은 `m0609_rg2_moveit`(호스트 `~/cobot2_ws/src`)에 있는데 컨테이너에 마운트가 안 돼 있다.
플러그인(`isaac_ros_cumotion_moveit`)은 `moveit_msgs::action::MoveGroup`을 쓰는 **얇은 클라이언트**라
CUDA 의존이 없지만(`package.xml` 확인), 실행 구성은 둘 중 하나를 골라야 한다:

| 안 | 방법 | 평가 |
|---|---|---|
| **E-1 (권장)** | 컨테이너를 `~/cobot2_ws`까지 마운트해 다시 띄우고, 그 안에서 우리 패키지도 빌드해 move_group을 **컨테이너 안에서** 실행<br>`./run_dev.sh -a "-v $HOME/cobot2_ws:/workspaces/cobot2_ws"` | Isaac ROS 3.2 = Humble이라 우리 패키지가 빌드될 가능성이 높다. 구성이 단순 |
| E-2 | 호스트 move_group + 컨테이너 플래너 노드. `--network host`라 통신은 된다 | 플러그인 `.so`를 호스트에서 빌드해야 하는데 `isaac_ros_common` 빌드툴 의존이 걸린다. **미검증** |

컨테이너를 껐다 켜야 하므로, **게이트 C까지 끝낸 뒤에** E-1로 다시 띄우는 것을 권한다.

---

## 6. 게이트 F — nvblox 켜기 🔴 **선택 아님, 필수** (근거: §4-3)

cuMotion이 **사람 팔을 보는 유일한 경로**다. octomap은 cuMotion에 전달되지 않는다.

```bash
ros2 run isaac_ros_cumotion cumotion_planner_node --ros-args \
  -p robot:=m0609_rg2.xrdf \
  -p urdf_path:=/workspaces/isaac_ros-dev/m0609/m0609_kinematics.urdf \
  -p read_esdf_world:=True \
  -p esdf_service_name:=/nvblox_node/get_esdf_and_gradient \
  -p update_esdf_on_request:=True
```

nvblox는 별도로 띄운다(`isaac_ros_nvblox`, RealSense 입력).
연결은 **토픽이 아니라 서비스**다 — 계획 요청 시 pull 한다.

---

## 7. 측정 항목 (프로젝트 데이터가 되는 것)

| 항목 | 방법 | 왜 |
|---|---|---|
| **OMPL vs cuMotion 계획 시간** | 같은 목표로 드롭다운만 바꿔 각 10회 | (a)/(b) 결정의 근거 |
| 구가 로봇을 덮는가 | `/curobo/voxels` RViz 육안 | 덜 덮으면 부딪힌다 |
| octomap vs nvblox 갱신 지연 | 손을 넣었다 뺐다 | 계획서 Phase 2-1과 **같은 항목 — 중복 측정 말 것** |
| VRAM 피크 | `nvidia-smi --query-gpu=memory.used --format=csv -l 1` | 8 GB 안에서 순차 실행 가능한지 |

---

## 8. GraspGenX 실물 테스트 (컨테이너 **밖**, 별개 트랙)

컨테이너와 무관하다. 호스트에서 `uv`로 돈다.

```bash
# 1) 장면 캡처 (로봇 팔·사람을 작업공간 박스 밖으로 치우고)
cd ~/cobot2_ws
python3 scripts/capture_graspgenx_scene.py --ros-args -p scene:=00

# 2) grasp 생성
cd ~/cobot2_ws/isaac_ros-dev/src/GraspGenX
uv run python scripts/demo_scene_pc.py \
  --sample_data_dir ~/cobot2_ws/data/graspgenx_scene \
  --gripper_name onrobot_RG2 \
  --num_grasps 64
```

`--grasp_threshold` 기본 0.7. grasp가 0개면 0.3까지 내려 본다(계획서 §6 분기 E).

---
확신도: **추론** — 파라미터 이름·XRDF 조회 경로·플러그인 의존성·마운트 구성은 소스 확인(검증됨)이나,
**컨테이너 안에서 실행한 검증은 0건**이다. 특히 게이트 C의 `CudaRobotModelConfig.from_data_dict`
시그니처는 cuRobo 버전에 따라 다를 수 있다 — 실패하면 `help(CudaRobotModelConfig)`로 확인한다.
내가 채워넣은 가정: ① mesh-free URDF로 충분하다(충돌은 XRDF 구에서 온다) ② Isaac ROS 3.2 컨테이너에서
우리 패키지가 빌드된다 ③ `voxel_size`는 기존 octomap과 맞춰 0.02로 시작한다
확인 요청: **게이트 C(구 75개·EE 위치)가 통과합니까?** — 여기가 XRDF 초안의 합불 판정이다.
