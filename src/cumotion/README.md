<!-- meta
updated: 2026-08-07
status:  live (2026-08-07 GPU PC 실기 첫 실행 완료. 인프라 전 구간 OK, **회피는 미검증**.
         미해결 2건: INVALID_MOTION_PLAN(-2) 13%, IK_FAIL. 0~0-5 절이 전부다)
next:    0-4 절 "이어서 할 것" 1번부터 — RViz /curobo/voxels 육안 확인
owns:    MoveIt+cuMotion 스택의 파이썬 제어 · 실행 중 재계획 루프 설계 · 이 패키지의 배치/실행 위치
-->

# cumotion — 실행 중 재계획으로 동적 장애물을 회피한다

실행 명령·검증은 [[ws/cobot2/testcommand]], 파이프라인 파라미터는 `config/README.md`
(T4~T7 노드 yaml)가 단일 출처다. 여기는 **이 패키지 코드가 왜 이렇게 생겼는지**만 둔다.

| 파일 | 역할 |
|---|---|
| `cumotion/arm.py` | 라이브러리. 계획(MoveGroup 액션) + 실행(FollowJointTrajectory 액션) + 재계획 루프 |
| `cumotion/dynamic_avoid.py` | 실행 노드. `mode` 파라미터로 check/joint/pose/pingpong |
| `config/dynamic_avoid.yaml` | 파라미터 기본값 (주석이 본체다) |
| `launch/dynamic_avoid.launch.py` | 자주 바꾸는 것만 launch 인자로 노출 |

---

## 0. 🔴 2026-08-07 실기 첫 실행 — 루프가 로봇을 못 가게 한다

**M0609 실기 + T1~T7 전 구간에서 처음 돌렸다.** 인프라는 전부 붙었는데 **루프 설계에 결함이 드러났다.**
아래 두 줄이 이 패키지에서 가장 중요한 실측치다. 같은 목표(`[45,0,90,0,90,0]` deg), 같은 `vel:=0.15`,
장애물 없음:

| | 계획 | 결과 |
|---|---|---|
| `static:=true` (재계획 OFF) | **1회** | **7.7초 만에 도착** (최대오차 0.0098 rad) |
| `static:=false` (3 Hz 재계획) | 179회 (실패 23) | **60초 타임아웃, 도착 실패** · 궤적 교체 155회 |

### 왜 그런가 — 산수가 명확하다

궤적 하나의 길이가 **7.7초**인데 루프는 **0.33초마다** 갈아끼운다. 로봇은 매 궤적의 **앞 4%**만
실행하고 버린다. 그 앞 4%는 정지에서 출발하는 **가속 램프**라 거의 안 움직이고, cuMotion 은
시작 속도를 버리므로(4절) 다음 궤적도 **또 v=0 에서** 시작한다.

```
7.7초짜리 궤적 ─┬─ 앞 0.33초(가속 램프)만 실행 → 폐기
                ├─ 앞 0.33초(또 가속 램프)만 실행 → 폐기
                └─ … 155회 반복 = 제자리 기어감
```

직접 증거: 교체 155회 내내 `이음새`(교체 순간 실측 관절속도 최대성분)가 **0.000~0.037 rad/s**였다.
`vel 0.15`로 45° 이동이면 0.1~0.5 rad/s 는 나와야 한다 — **속도가 붙을 기회 자체가 없었다.**

### 그래서 바꾼 것: "무조건 교체" → "달라졌을 때만 교체"

장애물이 안 변했으면 새 궤적은 **기존 것과 같은 경로를 처음부터 다시 시작하는 것**뿐이다.
교체할수록 손해다. 그래서 계획은 3 Hz 로 계속 던지되(장애물 감시는 그대로 유지), 새 궤적이
현재 궤적의 남은 부분과 **유의미하게 다를 때만** 발행한다 — `swap_threshold_rad` (4절).

⚠️ `replan_hz` 를 낮추는 건 임시방편이다. 회피 반응도 같이 느려지고, 궤적이 7.7초라 1 Hz 로도
여전히 앞 13%만 실행한다.

### 같이 관측된 것

- `INVALID_MOTION_PLAN(-2)` **12.8%** (179회 중 23회). bench 로는 20~30%. 원인은 0-2 절.
- 복셀 **2500~2850** 사이에서 계속 변동 → nvblox 가 살아서 지도를 갱신 중.
  (2026-08-06 기록은 27,646개 — **10배 차이**, 원인 미확인)
- 계획시간 평균 **197 ms** — `testcommand.md` 실측 204 ms 와 일치
- `max_start_jump` 폐기 경고 **0회** → 인계 위치 예측 자체는 잘 맞았다

---

## 0-1. 수정 후 첫 실행 (같은 날, `swap_threshold_rad: 0.05`)

**`_same_path()` 는 동작한다. 다만 문턱이 너무 작다.**

```
궤적 교체 9회 (동일해서 생략 2회)      ← 11회 중 2회만 생략. 문턱 0.05 rad 가 작다
```
재계획마다 lookahead 지점이 달라 cuMotion 이 미묘하게 다른 경로를 낸다. 그 차이가 0.05 rad 를
넘어서 대부분 교체돼 버린다. **다음 시도는 0.10~0.15 부터.**

⚠️ 이 실행은 6.5초 만에 **다른 이유로** 중단돼서 문턱의 실제 효과(도착 시간)를 못 봤다.

## 0-2. 미해결 2건 — 계획 실패

### ① `INVALID_MOTION_PLAN(-2)` — MoveIt 이 궤적을 재검증해 거부

플래너가 낸 코드가 **아니다.** MoveIt 이 플래너가 준 궤적을 자기 planning scene 으로 다시
검증해서 거부한 것이다 (`planning_pipeline.h` 의 `check_solution_paths_`).

배제된 것 (실측):
- **옥토맵 아니다** — T7 을 `octomap:=false` 로 내려도 그대로 발생 (cuMotion 7/10 → 8/10)
- **목표 자세 아니다** — `/check_state_validity` 조회 결과 `valid=True`

남은 것: 같은 시작·목표인데 **산발적**으로 실패한다 → cuMotion 이 시드마다 다른 궤적을 내고
그중 일부의 **중간 지점**이 MoveIt 검사에 걸린다는 뜻.

🔴 유력 용의자 — **SRDF 와 XRDF 의 자기충돌 면제 목록이 다르다:**
```
XRDF (cuMotion): link_4 → [link_5, link_6, rg2_base_link]   ← 무시하고 계획
SRDF (MoveIt)  : link_4 → [link_3, link_5, link_6]          ← rg2_base_link 를 검사
```
`config/testcommand.md` 12절이 "XRDF `link_4 ↔ rg2_base_link` 자기충돌 검사를 꺼 뒀다 —
실기 모션 전 재검토 필수"로 이미 적어둔 그 항목이다. **미확정** — 실패한 궤적의 중간 자세를
실제로 검사해 봐야 한다.

### ② `IK_FAIL` — 목표 pose 의 IK 해가 전부 ESDF 와 충돌

4회 연속 실패로 `max_consecutive_failures` 에 걸려 감속 정지했다.

🔴 **여기서 코드 오류를 하나 잡았다.** T6 은 `IK_FAIL` 을 `NO_IK_SOLUTION(-31)` 로 반환하는데
우리에겐 `PLANNING_FAILED(-1)` 로 온다. **플러그인(`cumotion_interface.cpp`)이 실패 시
플래너의 진짜 error_code 를 덮어쓰고 `-1` 로 고정하기 때문이다.**

> **cuMotion 경로에서 `-1` 은 원인을 알려주지 않는다. 반드시 T6 로그의
> `Motion planning failed wih status:` 줄을 봐야 한다.** (`arm.py` 힌트에 반영해 둠)

🔴 **진짜 의심 지점 — T6·T5 가 이 ws 의 튜닝 yaml 로 안 돌고 있다:**
```
지금 (T6 로그의 ESDF req):        중심 (0,0,0),          2×2×2 m    ← 바닥 아래·로봇 뒤까지 포함
config/cumotion_planner.yaml:     중심 (0.35,0,0.325),   1.10×1.0×0.75 m
```
`testcommand.md` 의 T5·T6 명령이 `--params-file` 을 안 줘서 **이 ws 가 튜닝해 둔 yaml 두 개가
실제로는 한 번도 적용된 적이 없다.** T5 도 `workspace_bounds_type: unbounded` 로 돌고 있다.
감시 상자가 의도보다 훨씬 커서 바닥·로봇 뒤 공간까지 장애물로 들어온다.

## 0-3. 핵심 로그 발췌 (다른 PC 에서 분석용)

**A. 루프 결함 — `static:=true` (대조군)**
```
[dynamic_avoid] 🔴 static_mode=true — 재계획을 하지 않는다.
[dynamic_avoid] 목표(관절 deg): 45.0, 0.0, 90.0, 0.0, 90.0, 0.0
[dynamic_avoid] 목표 도착 (최대오차 0.0098 rad)                      ← 7.7초
[dynamic_avoid] 계획 1회 (실패 0) / 궤적 교체 0회 / 계획시간 평균 230 ms
```

**B. 루프 결함 — 3 Hz 재계획 (수정 전)**
```
[dynamic_avoid] 교체 #1   | 이음새 0.000 rad/s | 복셀 2741개
[dynamic_avoid] 교체 #2   | 이음새 0.014 rad/s | 복셀 2741개
   … (이음새가 끝까지 0.000~0.037 rad/s 를 벗어나지 않는다) …
[dynamic_avoid] 교체 #155 | 이음새 0.000 rad/s | 복셀 2697개
[dynamic_avoid] 타임아웃 60s — 정지
[dynamic_avoid] 계획 179회 (실패 23) / 궤적 교체 155회 / 계획시간 평균 197 ms, 최대 252 ms /
                이음새 최대 0.037 rad/s
```

**C. 수정 후 (`swap_threshold_rad: 0.05`) + IK_FAIL 중단**
```
[dynamic_avoid] 교체 #9 | 이음새 0.000 rad/s | 복셀 2663개
[dynamic_avoid] 계획 실패: PLANNING_FAILED(-1) …          ×4 연속
[dynamic_avoid] 계획 4회 연속 실패 — 감속 정지.
[dynamic_avoid] 계획 18회 (실패 6) / 궤적 교체 9회 (동일해서 생략 2회) /
                계획시간 평균 196 ms / 이음새 최대 0.020 rad/s
```

**D. 같은 시각 T6 (진짜 원인은 여기에만 있다)**
```
[cumotion_action_server] Planning with time_dilation_factor: 0.15
[cumotion_action_server] Calling ESDF service
[cumotion_action_server] ESDF req = Point(x=-1.0, y=-1.0, z=-1.0), Vector3(x=2.0, y=2.0, z=2.0)
                                    ↑ 🔴 튜닝 안 된 기본 그리드
[cumotion_action_server] Updated ESDF grid
[cumotion_action_server] Calculating goal pose from Joint target
[cumotion_action_server] Motion planning failed wih status: MotionGenStatus.IK_FAIL
```

**E. bench (`scripts/bench_planning_time.py --repeat 10`, plan_only)**
```
octomap:=true    ompl 10/10 (wall 98.7 ms)   cuMotion 7/10 (wall 199.9 ms)  실패 #2,#4,#5 = -2
octomap:=false   ompl 10/10 (wall 100.2 ms)  cuMotion 8/10 (wall 198.1 ms)  실패 #4,#5    = -2
```

## 0-4. 🔴 이어서 할 것 (우선순위 순)

1. **RViz `/curobo/voxels` 를 눈으로 본다.** 목표(45° 쪽) 근처가 복셀로 덮여 있는가?
   → 덮여 있으면 IK_FAIL 원인 확정. 그게 **실제 물체인지 로봇 자기 몸이 샌 것인지**도 거기서 갈린다.
2. **T6 을 튜닝 yaml 로 재기동**하고 `ESDF req` 가 `Point(-0.2,-0.5,-0.05), Vector3(1.1,1.0,0.75)`
   로 바뀌는지 확인:
   ```bash
   ros2 run isaac_ros_cumotion cumotion_planner_node --ros-args \
     --params-file /workspaces/cobot2_ws/config/cumotion_planner.yaml \
     -p robot:=m0609_rg2.xrdf \
     -p urdf_path:=/workspaces/isaac_ros-dev/m0609/m0609_kinematics.urdf
   ```
   T5 도 `config/nvblox_realtime.yaml` 로 같이 맞춘다 (voxel_size 0.05 는 양쪽 동일해야 한다 —
   다르면 `cumotion_planner.py:410` 에서 FATAL).
3. **`swap_threshold_rad` 를 0.10~0.15 로 올려** `mode:=joint` 도착 시간이 static 의 **7.7초**에
   근접하는지 본다. 그게 이 수정의 합격 기준이다.
4. `-2` 확정 — 실패한 궤적의 **중간 자세**를 `/check_state_validity` 에 넣어 어느 링크쌍이
   걸리는지 본다. `link_4 ↔ rg2_base_link` 가 나오면 SRDF/XRDF 정합 작업으로 넘어간다.
5. 그다음에야 `mode:=pingpong` + 실제 장애물 투입. **회피 자체는 아직 한 번도 검증 안 됐다.**

## 0-5. 아직 안 고친 것 (알고 남겨둔 것)

- **`do_check()` 가 계획을 1회만 던진다.** `-2` 실패율이 13% 라 **파이프라인이 멀쩡해도 check 가
  8번에 1번꼴로 실패**하고, 그때 `T4/T5/T6 중 하나가 문제다` 라는 엉뚱한 메시지가 찍힌다.
- **`/curobo/voxels` 구독을 첫 계획 뒤에 건다** → `mode:=check` 의 첫 계획 복셀을 못 본다
  (계획을 한 번 더 던져 우회 중). publisher 는 T6 시작 시점에 생기므로 앞당길 수 있다.

---

## 1. 🔴 왜 "루프"인가 — 이 패키지의 존재 이유

`cumotion_planner_node` 는 **계획 요청 1건당 ESDF 를 딱 1번** 읽는다
(`cumotion_planner.yaml` 의 `update_esdf_on_request` 주석, `cumotion_planner.py:621`).

> 궤적이 한 번 만들어지고 나면, 실행 중에 사람이 걸어 들어와도 cuMotion 은 모른다.

즉 **nvblox 지도를 실시간으로 만든 것만으로는 실시간 회피가 안 된다.** 지도는 재료일 뿐이고,
회피는 *이 노드가 계획을 계속 다시 시키고 실행 중인 궤적을 갈아끼울 때* 비로소 생긴다.
`config/README.md` 의 "아직 안 된 것" 마지막 항목("실행 중 동적 회피는 여전히 안 된다 …
그 다음 단계(실행 중 재계획 루프)의 전제 조건일 뿐")이 가리키는 게 정확히 이 패키지다.

```
                        ┌──────── 3 Hz 로 반복 ────────┐
현재/예측 상태 ──▶ plan() ──▶ nvblox ESDF pull ──▶ 새 궤적 ──▶ JTC 로 교체 발행 ──┘
                                                              (기존 goal 은 JTC 가 선점)
```

## 2. 🔴 이 노드는 nvblox 를 구독하지 않는다

가장 헷갈리는 지점이다. **장애물 데이터는 이 노드를 안 거친다.**

```
nvblox_node ──서비스── /nvblox_node/get_esdf_and_gradient
                           ▲  cumotion_planner_node 가 pull 한다
                           │  (cumotion_planner.yaml: read_esdf_world: true,
                           │   esdf_service_name, update_esdf_on_request: true)
                    cumotion_planner_node ── cuRobo 충돌월드
                           ▲ /cumotion/move_group
                    move_group (cuMotion 플러그인)
                           ▲ /move_action  ← 이 노드는 여기만 잡는다
                    dynamic_avoid
```

우리가 ESDF 를 받아서 플래너에 넘겨주는 구조가 **아니다.** `cumotion_planner_node` 가
자기 요청을 처리하는 도중에 nvblox 에 직접 서비스 콜을 날린다. 그래서:

> **`plan()` 호출 그 자체가 nvblox 에 ESDF 를 물어보는 트리거다.**
> "장애물을 다시 본다" = "`plan()` 을 다시 부른다" — 1절의 루프가 회피를 만드는 이유가 이것이다.

(직접 구독해서 MoveIt collision object 로 넘기는 방식은 `motion_planning/nvblox_bbox_bridge.py`
가 하는 **별개 접근**이다. OMPL/octomap 경로용이고 cuMotion 경로와 섞으면 안 된다.)

### 그래서 감시가 따로 필요하다

🔴 **nvblox 가 죽어도 계획은 성공한다.** 장애물이 없는 세상에서 계획할 뿐이다.
계획 성공/실패로는 절대 드러나지 않고, 로봇이 장애물을 통과한 뒤에야 안다.
`testcommand.md` 가 "성공처럼 보이는 실패"라 부르는 그것이다. 그래서 두 겹을 넣었다:

| 무엇 | 어떻게 | 걸리면 |
|---|---|---|
| ESDF 서비스 존재 (`check_obstacle_pipeline()`) | `esdf_service_name` 이 실제로 떠 있는지 | `require_obstacle_pipeline: true` 면 **이동을 거부**한다 |
| cuMotion 이 실제로 본 복셀 (`/curobo/voxels`) | 궤적 교체마다 복셀 수를 로그에 남긴다 | 0개면 "nvblox 는 살아 있어도 지도가 비었다" 경고 |

⚠️ 서비스 존재 확인은 nvblox 가 *떠 있다*는 것만 본다. `esdf_mode` 가 `2d` 면 nvblox 는
cuMotion 첫 요청에 FATAL 로 죽는데, 그건 첫 계획을 실제로 던져 봐야 드러난다 —
`mode:=check` 가 계획을 1회 던지는 이유다.

⚠️ `/curobo/voxels` 는 **계획 요청을 처리하는 중에만** 발행된다(`testcommand.md` 8절).
대기 중에 `topic hz` 로 확인하려 들면 안 나온다.

`pipeline_id:=ompl` 로 쓸 땐 nvblox 가 필요 없으므로 `require_obstacle_pipeline:=false` 로 내린다.

## 3. 전부 표준 ROS 2 인터페이스다

| 하는 일 | 인터페이스 |
|---|---|
| 계획 | `/move_action` — 액션 `moveit_msgs/action/MoveGroup` (`pipeline_id: isaac_ros_cumotion`, `plan_only: true`) |
| 실행 | `/dsr01/dsr_moveit_controller/follow_joint_trajectory` — 액션 `control_msgs/action/FollowJointTrajectory` |
| 상태 | `/joint_states` — 토픽 `sensor_msgs/msg/JointState` |
| 정지 | `/dsr01/motion/move_stop` — 서비스 `dsr_msgs2/srv/MoveStop` |

RViz MotionPlanning 패널이 쓰는 것과 같은 진입점이고, GUI 대신 이 노드가 클라이언트다.

### 🔴 왜 `moveit_py` 가 아니라 액션 클라이언트인가

`ARCHITECTURE.md` 2절이 권하는 `moveit_py` 는 **이 루프에는 못 쓴다.** 셋 다 치명적이다:

1. **`moveit_py.execute()` 가 MoveIt 실행 관리자를 탄다** → `allowed_start_tolerance`(0.01 rad)
   검사에 걸려 **움직이는 중의 궤적 교체가 매번 거부된다.** 이 패키지의 존재 이유가 그 교체다.
2. **실행 중 궤적을 선점 교체하는 API 가 없다.** plan→execute 순차 모델이라 표현 자체가 안 된다.
3. **프로세스 안에 RobotModel/PlanningScene 을 또 띄운다** → `move_group` 의 파라미터 일습
   (robot_description·SRDF·kinematics·planning_pipelines)을 이 노드에도 똑같이 먹여야 한다.
   액션 클라이언트는 이미 떠 있는 `move_group` 에 붙기만 하면 된다.

JTC 액션을 직접 부르면 ① 의 검사가 없고, 새 goal 이 오면 JTC 가 기존 goal 을 스스로 선점한다.
`plan_only: true` 로 궤적만 받아오는 이유가 이것이다.

⚠️ 컨트롤러 이름 앞의 `/dsr01/` 은 오타가 아니다. bringup 의 `controller_manager` 가
`dsr01` 네임스페이스에 있어서 액션도 그 밑에 뜬다.

## 4. 인계(handover) 타이밍 — 세 파라미터가 전부다

계획 1회에 wall **204 ms**(`testcommand.md` 9절 실측). 그동안 로봇은 계속 움직인다.
그래서 "지금 상태"로 계획하면 결과가 나올 땐 이미 그 지점을 지나쳐 있다 → 인계 시 점프.

| 파라미터 | 기본 | 의미 | 어긋나면 |
|---|---|---|---|
| `lookahead_s` | 0.35 s | **미래 시점**의 궤적 위 상태에서 계획을 시작 | 작으면 "새 궤적 시작점이 실측과 어긋남" 경고 후 폐기 |
| `handover_s` | 0.05 s | 새 궤적을 뒤로 밀어 JTC 가 보간해 올라타게 함 | 0 이면 즉시 점프, 크면 반응이 느려짐 |
| `replan_hz` | 3.0 Hz | 재계획 주파수 | 위로 올려도 **새 정보가 없다** (아래) |

🔴 **`lookahead_s > 계획시간 + handover_s`** 가 성립해야 루프가 돈다. 0.35 는 204 ms + 여유다.
`vel_scale` 을 올리면 같은 시간에 더 멀리 가므로 `lookahead_s` 도 같이 올려야 한다.
`mode:=check` 가 실측 계획시간과 비교해서 이 조건을 자동으로 경고해준다.

### 🔴 다만 lookahead 로 고쳐지는 건 **위치뿐**이다

**cuMotion 은 우리가 보낸 시작 velocity 를 버린다.** `cumotion_planner.py:675` 가
`CuJointState.from_position(position=, joint_names=)` 로만 시작상태를 만들어 velocity 가 0 으로
채워지고, `is_diff=False` 라 라이브 `/joint_states` 를 읽는 686~698 분기도 타지 않는다.

> 새 궤적의 **첫 점 velocity 는 언제나 0** 이다. 로봇이 달리는 중에 "정지 상태에서 출발하는"
> 궤적을 인계받으므로, 교체마다 속도 불연속이 남는다.

이건 튜닝 실패가 아니라 플래너의 성질이라 `lookahead_s` 를 아무리 키워도 안 없어진다.
할 수 있는 건 셋뿐이다 — **`handover_s` ↑ / `replan_hz` ↓ / `vel_scale` ↓.**
크기는 눈으로 재지 말고 교체 로그와 `summary()` 의 **`이음새 N rad/s`**(교체 순간의 실측
관절속도 최대성분 = 불연속의 크기) 로 본다. 0 에 가까울수록 매끄럽다.

⚠️ `start_pos` 를 아예 안 주면(`is_diff=True`) cuMotion 이 `/joint_states` 의 실제 velocity 를
읽는다(`:694-698`). 대신 lookahead 가 사라져 204 ms 뒤처진 상태로 계획하게 되므로,
이 루프에서는 그쪽 손해가 더 크다고 보고 현재 구조를 유지한다.

🔴 **3 Hz 위로 올리는 건 의미가 없다.** `robot_segmenter_node` 가 3.7 Hz 라
nvblox 지도 자체가 그 속도로만 갱신된다(`config/README.md` 병목 항목). GPU 부하만 늘어난다.

## 5. 안전 — 코드가 하는 것과 사람이 해야 하는 것

코드가 하는 것:
- **장애물 경로 gate** (`require_obstacle_pipeline`, 기본 true): ESDF 서비스가 없으면
  **이동을 거부**한다. nvblox 없이도 계획은 성공하므로 이게 없으면 통과한 뒤에야 안다 (2절)
- **시작점 점프 검사** (`max_start_jump`, 0.25 rad): 예측이 빗나간 궤적은 발행하지 않고 버린다
- **연속 실패 차단** (`max_consecutive_failures`, 4회): 감속 정지 후 종료
- **감속 정지** (`brake()`): goal 을 cancel 하면 JTC 가 그 자리를 홀드해 급정지가 된다.
  정상 종료 경로에서는 현재 속도에서 0 까지 등감속하는 짧은 궤적을 대신 쏜다
- **비상정지** (`emergency_stop()`): `/dsr01/motion/move_stop`, 기본 Soft stop

사람이 해야 하는 것:
- **`mode:=check` 를 먼저** 돌린다. 여기서 걸리는 게 실기에서 걸리는 것보다 싸다
- **첫 실행은 `vel:=0.15`**, 비상정지 버튼에 손을 올린 채로
- `pingpong_a_deg`/`pingpong_b_deg` 를 **`mode:=joint` 로 각각 따로 한 번씩** 가보고 눈으로 확인
- ⚠️ **루프가 도는 동안 `movej`/`movel` 을 부르지 말 것.** MoveIt 경로와 두산 네이티브 모션
  서비스가 **같은 DRFL TCP 연결 하나**를 공유한다(`ARCHITECTURE.md` 3절). 모션 모드가 충돌한다

## 6. 이 패키지를 어디에 두고 어디서 돌리나

### 결론

**GPU PC 호스트의 `~/cobot2_ws/src/cumotion/`.** 컨테이너 안에서 빌드·실행한다.

`testcommand.md` 3절의 기동 명령이 이미 그 디렉토리를 마운트하고 있어서 추가 설정이 없다:

```bash
./run_dev.sh -a "-v $HOME/cobot2_ws:/workspaces/cobot2_ws"
#                   └─ 호스트 ~/cobot2_ws  →  컨테이너 /workspaces/cobot2_ws
```

🔴 **컨테이너에서 돌릴 거면 마운트된 경로 안에 있어야 한다.** 도커는 마운트 안 된 호스트
디렉토리를 아예 못 본다. 다른 곳에 두려면 `-v` 를 하나 더 붙인다:

```bash
./run_dev.sh -a "-v $HOME/cobot2_ws:/workspaces/cobot2_ws -v $HOME/내경로:/workspaces/mypkg"
```

⚠️ `run_dev.sh` 는 컨테이너를 재사용하지 않고 **매번 새로 만든다**(`testcommand.md` 3절).
마운트는 띄울 때마다 붙여야 하고, 그래서 마운트를 늘릴수록 기동 명령이 길어진다.

### 왜 `~/cobot2_ws` 인가 — 기능이 아니라 관리 때문이다

이 패키지의 `esdf_service_name` · `base_frame` · `voxel_topic` 은 `config/` 의
`cumotion_planner.yaml` / `nvblox_realtime.yaml` 과 **짝을 맞춰야 하는 값**들이다
(어긋나면 에러 없이 조용히 장애물을 놓친다 — 2절). 같은 트리 안에 있어야 같이 고친다.

### 🔴 호스트에서 돌려도 된다

이 패키지는 **GPU 를 안 쓴다.** CUDA·curobo·moveit 코어 라이브러리 전부 무관하고,
필요한 건 메시지 패키지뿐이다(순수 액션 클라이언트라서 — 3절).

```bash
# 호스트에서 이 4개가 다 나오면 호스트 실행 가능
ros2 pkg list | grep -E "^(moveit_msgs|control_msgs|visualization_msgs|dsr_msgs2)$"
# moveit_msgs 가 없으면:  sudo apt install ros-humble-moveit-msgs
```

호스트 실행의 이점: 컨테이너를 새로 띄울 때마다 재빌드할 필요가 없고, 비상정지용
`dsr_msgs2` 가 호스트엔 확실히 있다(bringup 이 쓴다).

**어디서 돌리든 통신은 된다.** 이 노드는 `/move_action`(컨테이너)과 `/dsr01/...`(호스트)을
동시에 잡아야 하는데, T7 move_group 이 컨테이너에서 호스트 `controller_manager` 를 이미
그렇게 쓰고 있으니 검증된 경로다. 단 둘은 지킨다:

- `export ROS_DOMAIN_ID=93` — 호스트·컨테이너 양쪽 다
- ⚠️ **`RMW_IMPLEMENTATION` 을 건드리지 말 것.** cycloneddds 로 바꾸면 컨테이너↔호스트
  **서비스**가 안 붙는다(토픽만 됨). `check_obstacle_pipeline()` 도 서비스 조회라 같이 깨지고,
  그러면 "nvblox 가 없다"고 오판해서 이동을 거부한다.

### 빌드

```bash
# 컨테이너 T8
source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash
export ROS_DOMAIN_ID=93

cd /workspaces/cobot2_ws
colcon build --symlink-install --build-base build_container \
             --install-base install_container --packages-select cumotion
source install_container/setup.bash
```

⚠️ **`install_container` 를 따로 쓰는 이유가 있다.** 호스트와 컨테이너가 같은 `install/` 에
빌드하면 파이썬 경로·ABI 가 섞여 한쪽이 깨진다. 호스트에서도 빌드할 거면 호스트는
기본 `build/`·`install/` 를, 컨테이너는 `build_container/`·`install_container/` 를 쓴다.

`--symlink-install` 이면 파이썬 파일과 yaml 을 고쳐도 **재빌드 없이** 반영된다(노드 재시작만).
호스트에서 편집하면 마운트를 통해 컨테이너에 즉시 보인다.

### 🔴 0 에서 시작하는 전체 기동 순서 (2026-08-07 실기 관통 확인)

> **T1~T7 은 `config/testcommand.md` 의 발췌다.** 그쪽이 단일 출처이고, 어긋나면 그쪽이 이긴다.
> 여기 두는 이유는 T8(이 패키지)만 따로 보면 못 돌리기 때문이다. **T8 절은 여기가 주인이다.**

터미널 8개. **T2(실기 로봇)는 사람이 직접 띄운다.**

#### 호스트 터미널 — 매 터미널 첫 줄

```bash
cd ~/cobot2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
export ROS_DOMAIN_ID=93
```
🔴 **`ROS_DOMAIN_ID` 를 빠뜨리면 노드가 하나도 안 보인다.** 2026-08-07 에 T8 에서 실제로 겪었다 —
`/move_action 액션 서버 없음` 으로 나와서 T7 이 죽은 줄 알았는데 도메인이 0 이었던 것뿐이다.

```bash
# T1 카메라
ros2 launch m0609_rg2_bringup camera.launch.py depth_profile:=848x480x15 color_profile:=848x480x15
#   확인: ros2 topic hz /camera/camera/aligned_depth_to_color/image_raw   → 10~15 Hz
#   확인: ros2 node list | grep -c "camera/camera"                        → 1 (2면 depth 반토막)

# T2 실기 로봇  ← 사람이 띄운다
ros2 launch m0609_rg2_bringup bringup.launch.py mode:=real host:=192.168.1.100 rviz:=false
#   확인: ros2 topic echo /joint_states --once   → name/position/velocity 각 12개
#   🔴 velocity 가 비어 있으면 cuMotion 계획이 전부 실패한다
```

#### T3 — 컨테이너

```bash
export ROS_DOMAIN_ID=93          # ⚠️ run_dev.sh 가 -e 로 넘긴다. 먼저 해야 한다
cd ~/cobot2_ws/isaac_ros-dev/src/isaac_ros_common/scripts
./run_dev.sh -a "-v $HOME/cobot2_ws:/workspaces/cobot2_ws"
```

🔴 **`run_dev.sh` 는 `docker run -it --rm` 이다 — 그 터미널을 닫으면 컨테이너가 통째로 삭제된다.**
이미 떠 있으면 `docker exec -it isaac_ros_dev-x86_64-container bash` 로 들어간다(새로 안 만든다).

**새 컨테이너면 맨 처음 한 번:**
```bash
bash /workspaces/cobot2_ws/scripts/container_setup.sh    # warp 1.5.0 / numpy 1.26.4 / cv2 OK
```
🔴 **이걸 빠뜨리면 T4 는 `import cv2 → _ARRAY_API not found`, T6 은 `module 'warp' has no
attribute 'torch'` 로 죽는다.** 2026-08-07 에 둘 다 겪었다. 컨테이너를 새로 만들 때마다 매번이다.
(출력의 `🔴 패치가 없다` 줄은 git `dubious ownership` 오탐이니 무시 — curobo 패치 2개는 살아 있다)

#### 컨테이너 셸 — T4~T7 매 셸 첫 줄

```bash
source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash
source /workspaces/cobot2_ws/install_container/setup.bash
export ROS_DOMAIN_ID=93
```
⚠️ `RMW_IMPLEMENTATION` 은 건드리지 않는다 (6절).

T4~T7 명령은 `config/testcommand.md` 4~7절 그대로. 각 단계 확인:

| | 노드 | 확인 |
|---|---|---|
| T4 | `robot_segmenter_node` | `ros2 topic hz /cumotion/camera_1/world_depth` → 3~4 Hz **(T5 가 떠야 나온다 — 구독자가 있을 때만 발행한다)** |
| T5 | `nvblox_node` (`esdf_mode:=3d`) | `ros2 service list \| grep esdf` · `pgrep -f nvblox_node` |
| T6 | `cumotion_planner_node` | 로그에 `cuMotion is ready for planning queries!` (5~10초) |
| T7 | `moveit.launch.py standalone:=false octomap:=true cumotion:=true` | 로그 3줄: `ompl` / `isaac_ros_cumotion` 파이프라인 + `Configured and activated dsr_moveit_controller` |

#### RViz (T7 창, 재시작할 때마다)

- `Add → rviz_default_plugins/Marker` → Topic **`/curobo/voxels`** ← **MarkerArray 아님**
- `Trajectory` 디스플레이 → `Interrupt Display: **true**` (기본 false 면 궤적이 실제보다 뒤처져 보인다)
- 🚨 **MotionPlanning 패널의 Plan 버튼을 누르지 말 것** — `planner_busy` 로 T8 이 `FAILURE(99999)` 로 실패한다
- ⚠️ 보이는 궤적(`/display_planned_path`)은 **계획된 것**이지 실행 중인 것이 아니다.
  `max_start_jump` 로 폐기된 궤적도 거기 그려진다. 실행 실체는
  `ros2 topic echo /dsr01/dsr_moveit_controller/controller_state` (desired/actual/error)

#### T8 — 호스트 (이 패키지)

🔴 **T8 은 컨테이너가 아니라 호스트에서 돌린다.** GPU 를 안 쓰고, 비상정지용 `dsr_msgs2` 가
호스트에 확실히 있다(6절). 빌드도 호스트다:
```bash
colcon build --symlink-install --packages-select cumotion
```

### 실행

```bash
# ① 사전 점검 — 로봇 안 움직임 (plan_only 로 계획만 1회)
ros2 launch cumotion dynamic_avoid.launch.py mode:=check
#   → "cuMotion 이 장 본 장애물 복셀 N개" 가 나와야 한다. 0 개면 장애물을 못 보는 상태다
#   ⚠️ check 는 **제자리 계획**이라 RViz 에 볼 궤적이 없다. 궤적을 보려면:
#      python3 scripts/bench_planning_time.py --repeat 10   (plan_only 고정, 로봇 안 움직임)

# ② 관절 목표 1회 이동 (deg)
ros2 launch cumotion dynamic_avoid.launch.py mode:=joint \
    goal_joint_deg:="[0.0, 0.0, 90.0, 0.0, 90.0, 0.0]" vel:=0.15

# ③ 🔴 동적 회피 시연 — 왕복 중에 작업영역에 손/상자를 넣는다
ros2 launch cumotion dynamic_avoid.launch.py mode:=pingpong vel:=0.2

# ④ 대조군 — 재계획을 끈 같은 왕복. ③ 과의 차이가 유일한 증거다
ros2 launch cumotion dynamic_avoid.launch.py mode:=pingpong static:=true vel:=0.15

# ⑤ TCP 목표 (m, deg)
ros2 launch cumotion dynamic_avoid.launch.py mode:=pose \
    goal_pose:="[0.45, 0.0, 0.35, 180.0, 0.0, 0.0]" vel:=0.15

# ⑥ OMPL(octomap)로 같은 루프 — 플래너 비교용
ros2 launch cumotion dynamic_avoid.launch.py mode:=joint pipeline:=ompl vel:=0.15

# launch 없이 직접
ros2 run cumotion dynamic_avoid --ros-args \
    --params-file $(ros2 pkg prefix cumotion)/share/cumotion/config/dynamic_avoid.yaml \
    -p mode:=check
```

launch 인자에 없는 파라미터는 `config/dynamic_avoid.yaml` 을 고친다(주석이 본체다).
launch 인자가 yaml 을 덮어쓴다.

### 종료 — 올린 순서의 반대로

T7 → T6 → T5 → T4 → T2 → T1 각각 `Ctrl+C`. 컨테이너 셸은 `exit`.

```bash
ps -eo pid,user,cmd | grep -E "move_group|nvblox|cumotion|segmenter|realsense2_camera_node" | grep -v grep
nvidia-smi --query-gpu=memory.used --format=csv,noheader     # ~15 MiB 면 반납 완료
```

🔴 **`pkill -f` 를 쓰지 말 것.** 자기 명령줄에도 매칭돼 자기 셸을 먼저 죽이고, 공유 랩탑이라
남의 프로세스까지 걸린다. PID 로 죽인다 (`testcommand.md` 10절).

⚠️ `run_dev.sh` 로 띄운 컨테이너는 그 셸에서 나가면 **삭제된다**(`--rm`). 다음에 다시 띄우면
`container_setup.sh` 를 또 돌려야 한다.

## 7. 라이브러리로 쓰기

pick-and-place 같은 걸 짤 땐 노드를 쓰지 말고 `arm.py` 를 직접 import 한다.

```python
import rclpy
from cumotion.arm import ArmConfig, CumotionArm

rclpy.init()
cfg = ArmConfig()          # cfg 를 주면 ROS 파라미터를 선언하지 않는다
cfg.vel_scale = 0.2
arm = CumotionArm(cfg); arm.start_spin(); arm.wait_until_ready()

target = [0.0, 0.0, 1.57, 0.0, 1.57, 0.0]           # rad
arm.run_to_goal(arm.joint_goal(target), goal_positions=target, replan_hz=3.0)

print(arm.summary())       # 계획 N회 / 궤적 교체 M회 / 계획시간 평균·최대
```

그리퍼(RG2)는 MoveIt 밖이다 — `/onrobot/sendCommand`(`onrobot_rg_msgs/srv/SetCommand`)
서비스로 따로 부른다. 이 패키지에는 넣지 않았다.

## 8. 증상 → 어디를 볼 것인가

| 증상 | 원인 | 조치 |
|---|---|---|
| check 에서 "액션 서버 없음" | T7 move_group 미기동 | `ros2 action list \| grep move_action` |
| check 에서 "dsr_moveit_controller 없음" | 컨트롤러 spawn 실패 | T7 로그의 `Configured and activated dsr_moveit_controller` |
| `/joint_states 에 velocity 가 없다` 경고 | bringup 설정 | `publish_default_velocities: True` |
| `START_STATE_IN_COLLISION` 반복 | 로봇 몸이 nvblox 지도에 찍힘 | T4 `distance_threshold` ↑, T5 재시작 |
| `GOAL_IN_COLLISION` | 목표가 장애물 안 | 치워질 때까지 못 간다. 정상 동작이다 |
| "새 궤적 시작점이 실측과 어긋남" 반복 | 인계 예측 실패 | `lookahead` ↑ 또는 `vel` ↓ |
| 궤적 교체 순간 덜컹거림 | 인계 불연속 | `handover_s` 를 0.05 → 0.1 |
| `ESDF 서비스 없음` → 이동 거부 | T5 nvblox 미기동/사망 | `pgrep -f nvblox_node`. 죽었으면 `esdf_mode:=3d` 로 재기동 |
| `복셀 0개` 경고 | nvblox 는 살아 있는데 지도가 빔 | 카메라 FOV / `workspace_bounds_*` / T4 `world_depth` 발행 |
| `복셀 미수신` | `/curobo/voxels` 안 옴 | `publish_curobo_world_as_voxels: true` 확인. 대기 중엔 원래 안 온다 |
| 계획은 되는데 장애물을 통과 | T4/T5 누락 또는 `read_esdf_world:=False` | `testcommand.md` 4·5절. **2절의 감시 두 겹이 이걸 잡으라고 있다** |
| 궤적 교체가 0회 | `static_mode` 가 켜졌거나 목표가 너무 가까움 | 종료 시 찍히는 `summary()` 확인 |
| launch 가 `FileNotFoundError` | share 에 config 미설치 | `setup.py` 의 `data_files` 확인 후 재빌드 |
| 컨테이너에서 `Package 'cumotion' not found` | 마운트 밖에 뒀거나 `install_container` 미소스 | 6절 — `-v` 마운트 확인 후 재빌드 |
| 토픽은 보이는데 **서비스만** 안 붙는다 | `RMW_IMPLEMENTATION` 을 cyclonedds 로 바꿈 | 6절 — 지우고 기본값(fastrtps)으로 |
| 아무 노드도 안 보인다 | `ROS_DOMAIN_ID` 불일치 | 호스트·컨테이너 양쪽 `export ROS_DOMAIN_ID=93` |

## 9. 아직 안 된 것 / 검증 안 한 것

- ~~🔴 **GPU PC 실기에서 아직 한 번도 안 돌렸다.**~~
  **2026-08-07 실기 실행 완료.** `mode:=check` / `mode:=joint`(static 양쪽) 확인. 결과는 0절.
  아직 안 돌린 것: **`mode:=pingpong`, `mode:=pose`, `pipeline:=ompl`, 그리고 실제 장애물 투입.**
  0절의 두 실험은 **장애물 없이** 돌린 것이라 회피 자체는 여전히 미검증이다.
- ~~**재계획 시 시작 속도(velocity)를 cuMotion 이 실제로 반영하는지 미확인.**~~
  **확인됨 — 반영하지 않는다** (`cumotion_planner.py:675` 소스 + 실기 양쪽).
  ⚠️ **결과 예측이 틀렸었다.** "교체마다 덜컹인다"고 적어놨는데, 실측된 증상은 정반대다 —
  덜컹이지 않는 대신 **속도가 아예 안 붙어서 목표에 못 간다**(0절). `이음새` 최대 0.037 rad/s.
- ~~**JTC 가 새 goal 로 기존 goal 을 선점하는 동작에 의존한다.**~~
  **확인됨 — 선점 교체가 동작한다.** 2026-08-07 실기에서 궤적 교체 155회가 `cancel_execution()`
  없이 끊김 없이 이뤄졌다. `send_trajectory()` 앞에 취소를 넣을 필요가 없다.
- **`pingpong_a_deg`/`pingpong_b_deg` 는 안전 검증된 자세가 아니다.** 임의로 잡은 값이다.
  (A = `[45,0,90,0,90,0]` 만 `mode:=joint` 로 도달 확인됨. B 는 아직 안 가봤다)
- 🔴 **`swap_threshold_rad`(0절의 수정)를 실기에서 아직 튜닝 안 했다.** 기본값은 추정치다.
  너무 크면 장애물이 와도 교체를 안 하고(=회피 실패), 너무 작으면 0절의 기어감이 재현된다.
  `mode:=joint` 로 **도착 시간이 static 의 7.7초에 근접하는지**부터 확인하고 올린다.
- **최악 반응시간 미측정.** 파이프라인 지연 ~0.6 s + 재계획 주기 0.33 s + 인계 0.35 s
  ⇒ 장애물이 나타나고 궤적이 바뀌기까지 **1.3 s 내외**로 추정된다. 사람 손 속도에는 부족할 수
  있다. 실제로 재봐야 하고, 부족하면 `vel` 을 낮추는 것 말고는 이 코드가 할 수 있는 게 없다
  (진짜 해법은 세그멘터 3.7 Hz 병목을 푸는 것).
- **패키지 이름 `cumotion` 은 `isaac_ros_cumotion` 과 다른 것이다.** 파이썬 모듈명도
  `cumotion` 이라 헷갈릴 수 있다 — `from cumotion.arm import ...` 는 **이 패키지**다.
