<!-- meta
updated: 2026-08-08 (3차 개정 — 링크=휴대폰 핫스팟 확정, D435i 영상을 VLA PC 로 전송.
         §3-3 이 "이미지는 안 넘긴다" → "무엇을 어떻게 넘기나" 로 뒤집혔고 D3 가 강등됐다)
status:  live (미착수)
owns:    M0609_VLA_system ↔ cobot2_ws 통합 · 역할 경계 · 지시 채널 계약 · 물체 선정(target selection) 설계
-->

# VLA 통합 — 필요 부분 정리 (2026-08-08)

> **상대 저장소**: `~/M0609_VLA_system` (별도 repo. 이 세션 기준 `git log` 최신 `5a10649`)
> ⚠️ **그 repo 는 이 ws 밖이고, 앞으로 다른 PC 에서 돈다.** 여기 적힌 VLA 측 값은 전부
> 2026-08-08 에 그 repo 원문을 읽은 스냅샷이다 — 다음 세션에 `git log` 부터 확인한다.

---

## 0. 확정된 범위 (2026-08-08 사용자 지시)

**이게 이 문서의 전제다. 1차 개정의 결정 D1·D2 는 여기서 답이 났다.**

| | 담당 |
|---|---|
| **로봇 행동 (모션·IK·충돌회피·그리퍼)** | **우리.** `pick_fsm` + MoveIt + `graspgenx` **그대로 유지** |
| **파지 계산 (6D grasp)** | **우리.** 고정 eye-to-hand D435i + GraspGenX |
| **"어떤 물체를 집을지"** | **VLA** — 지시만 전달 |
| **"집은 물체를 어디에 놓을지"** | **VLA (나중)** — 아직 범위 밖 |
| VLA 쪽 웹캠(C270)·homography·LLM·GUI | **우리 역할 아님.** 아예 **다른 외부 PC** 에서 돈다 |
| **두 PC 를 잇는 링크** | **개인 휴대폰 핫스팟** (2026-08-08 확정) — 대역폭이 설계 제약이 된다 → §3-3 |
| **D435i 영상 전송** | **우리 → VLA PC.** 무엇을 보낼지·왜 압축본만 보내는지 → §3-3 |

> 🔑 **최우선 원칙: `pick_fsm` 에 물려 있는 의존성을 최대한 보존한다.**
> VLA 를 붙이려고 FSM·MoveIt·graspgenx 배선을 바꾸지 않는다. VLA 는 **입력 하나가 늘어나는
> 것**이지 실행 계층을 대체하는 것이 아니다.

### 이 확정으로 사라진 문제들 (1차 개정에서 크게 다뤘던 것)

| 1차의 쟁점 | 지금 |
|---|---|
| **D1** D435i 를 팔로 옮기나? | **소멸.** 고정 유지. `vla_wrist`(손목 전제)는 **우리가 안 쓴다** — 그쪽 PC 의 문제다 |
| **D2** 실행이 `vla_robot`(amovel)인가 `pick_fsm`(MoveIt)인가? | **`pick_fsm`(MoveIt) 확정** |
| `DR_init` 드라이버 경합 | **소멸.** `vla_robot` 이 다른 PC 에 있고 우리 로봇에 안 붙는다 |
| `vla_wrist` 를 우리 노드로 대체(1차 "C안") | **불필요.** `GraspRequest`/`GraspPlan` 계약을 구현할 이유가 없어졌다 → §4 |
| `~/.local` 오염 | **통합 이슈에서 빠진다.** VLA 를 이 PC 에 설치할 이유가 없다. 단 **이미 깔린 것은 남아 있다** → §6-2 |
| C270 웹캠을 우리 README 하드웨어 표에 넣을지 | **불필요.** 우리 역할 아님 |

### 남은 결정

| # | 질문 | 왜 막히나 |
|---|---|---|
| **D3** | VLA 가 `base_link` 기준 좌표를 줄 수 있나? | 🔻 **강등(3차).** `pixel` 경로면 필요 없다 → §3-3(c). C270 폴백에서만 살아 있다 |
| **D4** | **fingertip 180 mm vs 실측 218 mm** | **VLA 와 무관한 우리 내부 불일치**로 재분류됐다 → §6-1 |
| **D5** | **D435i 가 `480x320` 을 지원하나?** | 🔴 **지원 목록에 없어 보인다.** 없으면 `424x240`(현 기본) 또는 `640x480` 중 택 → §3-3(d). ⚠️ 해상도는 **octomap·nvblox 두 경로에 동시에** 걸린다 → §3-3(a-2) |

---

## 1. 역할 경계

```
┌─ 외부 PC (~/M0609_VLA_system) ───────┐      ┌─ 이 PC (cobot2_ws) ─────────────┐
│  고정 Webcam C270                    │      │  고정 D435i (eye-to-hand)        │
│    → YOLO-seg + table homography     │      │    → yolo_seg(컨테이너) + depth  │
│    → LLM 이 "무엇을" 판단             │      │  grasp_bridge_node → GraspGenX   │
│                                      │      │  pick_fsm → MoveIt → M0609+RG2   │
└──────────────┬───────────────────────┘      └───────────▲─────────────────────┘
               │                                          │
               └──── 지시 1개: "이 물체를 집어라" ──────────┘
                     (class + base XY, 나중에 place 목적지)
```

**넘어가는 것은 지시뿐이다.** 포즈도, 궤적도, 이미지도 넘어가지 않는다.
VLA 는 "무엇을", 우리는 "어떻게"를 전부 소유한다.

---

## 2. 지시 채널 — 커스텀 메시지를 만들지 않는다

### 왜 표준 타입인가 (외부 PC 라서 생기는 제약)

커스텀 msg 를 쓰면 **양쪽 PC 에 같은 인터페이스 패키지를 빌드·배포해야 한다.**
`pick_fsm_msgs` 를 외부 PC 에 설치시키는 순간 두 repo 가 버전으로 묶이고, 한쪽만 빌드가
갱신되면 **타입 해시가 어긋나 조용히 매칭이 끊긴다**(에러가 아니라 "토픽은 보이는데 데이터가
안 옴"으로 나타난다 — 이 ws 가 도메인/프로파일 문제로 이미 겪은 증상과 구분이 안 된다).

→ **`std_msgs/String`(JSON) 한 토픽.** 이 ws 에 이미 같은 패턴의 선례가 있다:
`/yolo_seg/classes` 가 `std_msgs/String` JSON 으로 클래스맵을 나른다
(`yolo_seg_node`, `graspgenx_perception/README.md` "클래스맵" 절). 그걸 재사용한다.

```
/vla/pick_command                        std_msgs/String (JSON)      VLA → 우리
/vla/pick_result                         std_msgs/String (JSON)      우리 → VLA  (성공/실패/사유)
/camera/camera/color/image_raw/compressed  sensor_msgs/CompressedImage 우리 → VLA  (§3-3)
```

```json
// /vla/pick_command — 지정 방식 2가지. pixel 이 있으면 pixel 을 쓴다
{"cmd": "pick", "class": "apple",
 "pixel": [312, 188], "pixel_wh": [424, 240],
 "request_id": "a17-3", "stamp_ns": 1754640000123456789}

// 폴백: VLA 가 자기 C270 만 볼 때 (base 좌표계 합의가 필요해진다 — §3-2)
{"cmd": "pick", "class": "apple", "base_xy": [0.42, -0.18], "request_id": "a17-3"}

// 나중에 place 가 붙을 때 — 필드만 늘린다. 토픽·타입은 그대로
{"cmd": "pick_and_place", "class": "apple", "pixel": [312, 188], "pixel_wh": [424, 240],
 "place": {"kind": "named", "value": "basket"}, "request_id": "a17-4"}
```

- 🟢 **`pixel` 을 기본 경로로 한다** — VLA 가 우리 D435i 컬러 프레임을 보게 되면서
  base 좌표계 합의(D3) 없이 물체를 가리킬 수 있게 됐다. 근거는 §3-3(c).
- ⚠️ **`pixel_wh` 는 생략 불가.** VLA 가 리사이즈한 프레임 위에서 찍었으면 좌표가 조용히
  어긋난다. 받는 쪽이 원본 해상도로 스케일링하고, 값이 없으면 **거부한다**.
- `base_xy` 는 폴백으로만 남긴다. 둘 다 있으면 `pixel` 우선, 둘 다 없으면 `class` 만으로
  선정(§5 폴백 정책).
- `request_id` 는 VLA 가 붙이고 우리가 **그대로 echo** 한다. 상관관계 추적용.
- `place` 는 **지금 채우지 않는다.** 필드를 미리 정의만 해 두고 `pick` 만 구현한다
  (YAGNI — 지금 `pick_fsm` 의 place 는 고정 관절값 `place_joints_deg` 하나다).
- ⚠️ **JSON 스키마 검증을 받는 쪽(우리)에서 한다.** 필드가 없거나 타입이 다르면 **거부하고
  `pick_result` 에 사유를 넣는다.** 조용히 기본값으로 진행하면 엉뚱한 물체를 집는다.

### `pick_fsm` 쪽 변경 최소화

| 무엇 | 어떻게 |
|---|---|
| 새 패키지 | **0개** |
| 새 msg/srv | **0개** |
| `pick_fsm` 코드 | 이상적으로 **0줄** — 지시를 받는 자리는 `grasp_bridge_node` 다(§5). FSM 은 지금처럼 `/grasp/compute` 를 부르고 결과를 받는다 |
| 대안 (지시를 FSM 이 받아야 할 때) | `pick_fsm` 의 기존 **`target` 파라미터**(클래스 이름)가 이미 있다. 좌표만 `grasp_bridge_node` 로 보내면 된다 |

> 지시를 **`grasp_bridge_node` 가 받는** 이유: 물체 선정은 **워커 호출 전에** 라벨을 걸러야
> 의미가 있다(§5). FSM 이 받아서 다시 내려보내면 경유지만 늘고 FSM 이 인식 개념을 알게 된다.

---

## 3. 외부 PC 경계에서 새로 생기는 것

### 3-1. 네트워크 — 여기서 가장 먼저 터진다

| 항목 | 우리 | VLA | 조치 |
|---|---|---|---|
| `ROS_DOMAIN_ID` | **93** (`src/pick_fsm/README.md` §2 단일 출처) | **지정 0건** → 기본 **0** (2026-08-08 `scripts/`·`src/`·`config/`·`.env` grep) | VLA 실행 셸에 `export ROS_DOMAIN_ID=93` |
| DDS 프로파일 | `fastdds_udp_only.xml` (SHM 비활성, UDP 전용) | 없음 | **PC 가 갈리면 SHM 은 어차피 못 쓴다** — 우리 프로파일을 그대로 쓰면 맞는다 |
| 방화벽·서브넷 | — | — | ⚠️ **미확인.** 같은 LAN 인지, 멀티캐스트가 통과하는지 안 봤다 |

> 💡 **증상 구분표가 이미 있다.** `graspgenx_perception/README.md` "데이터가 안 올 때" 절의
> **"도메인이 탐색을, 프로파일이 데이터를 결정한다"** (2026-08-07 A/B 실측)가 그대로 적용된다:
> 토픽 자체가 안 보이면 도메인, 토픽은 보이는데 데이터가 0 이면 프로파일/방화벽.

### 3-2. 🟡 공유 좌표계 (D3) — **§3-3 으로 우선순위가 내려갔다**

> 🔻 **2026-08-08 강등.** D435i 영상을 VLA PC 로 넘기기로 하면서, 지시를 **픽셀 좌표**로
> 보낼 수 있게 됐다(§3-3(c)). 그러면 캘리브가 **우리 것 하나뿐**이라 아래의 오차 합산 문제가
> 통째로 사라진다. 이 절은 **VLA 가 자기 C270 만 보는 폴백 경로**에만 적용된다.

두 시스템이 카메라를 **각자** 갖고 각자 캘리브한다. 공유하는 것은 **`base_link` 좌표 하나뿐**이다.

```
VLA: C270 픽셀 → table homography → base XY  (외부 PC 책임)
우리: D435i depth → T_cam2base → base XYZ     (이 PC 책임)
                        ↕
              이 둘이 맞아야 매칭이 성립
```

- VLA 는 이미 `SceneObject.position_base`(`geometry_msgs/Point`, m, base 프레임)를 낸다 —
  **그쪽 homography 가 우리 로봇 base 프레임으로 매핑되도록 이미 짜여 있다.** 원리적으로 가능.
- 허용오차는 VLA 자신이 정해 뒀다: `match_tolerance_m: 0.06`(`system.yaml:107`).
  **두 캘리브의 오차 합이 6 cm 를 넘으면 매칭이 통째로 실패한다.**
- ⚠️ 우리 캘리브의 현재 상태는 **잠정**이다 — AX=XB 병진잔차 **중앙값 40.1 mm**
  (`README.md` §5 상태표, 단일 출처 `md/state.md`). **여기서만 이미 예산의 3분의 2를 쓴다.**
- 카메라를 하나라도 건드리면 그쪽 캘리브만 무효가 되고, 증상은 **"엉뚱한 물체를 집는다"** 다.

→ **D3 확인 항목**: VLA 가 내는 `position_base` 가 **우리 `base_link` 와 같은 원점·축**인가.
이름이 같다고 같은 프레임이라는 보장이 없다. 물체 하나를 두 시스템이 각각 재서 대조하는 것이
유일한 확인 방법이다.

### 3-3. 🔴 D435i 영상을 VLA PC 로 보낸다 (2026-08-08 추가 — 1차·2차 개정의 전제를 뒤집는다)

> 2차 개정까지 이 절은 "**이미지는 경계를 넘지 않는다**" 였다. 사용자 지시로 **뒤집혔다.**
> D435i 영상을 VLA PC 로 전송한다. 그러면 대역폭이 통합의 **첫 번째** 장애물이 된다 —
> 특히 링크가 **개인 휴대폰 핫스팟**이기 때문이다.

#### (a) 지금 우리 ws 가 쓰는 카메라 토픽 — 전수

카메라 드라이버는 `realsense2_camera_node` 하나이고 이름/네임스페이스를 안 바꾸므로
접두사는 전부 **`/camera/camera/`** 다. 발행 옵션은 `camera.launch.py:88-97` 기준
`enable_color` `enable_depth` `align_depth.enable` `pointcloud.enable` `enable_sync` **전부 true**.

| 토픽 | 타입 | 우리 쪽 소비자 | 근거 |
|---|---|---|---|
| `/camera/camera/color/image_raw` | `sensor_msgs/Image` (rgb8) | `yolo_seg_node`(컨테이너), `capture_graspgenx_scene` | `yolo_seg_node.py:171`, `capture_graspgenx_scene.py:53`, `launch/graspx.launch.py:44` |
| `…/color/image_raw/compressed` | `sensor_msgs/CompressedImage` | `capture_graspgenx_scene` **폴백**(raw 우선) | `capture_graspgenx_scene.py:151` |
| `/camera/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` (16UC1) | `capture_graspgenx_scene` | `capture_graspgenx_scene.py:51` |
| `/camera/camera/aligned_depth_to_color/camera_info` | `sensor_msgs/CameraInfo` | `capture_graspgenx_scene` | `capture_graspgenx_scene.py:52` |
| `/camera/camera/color/camera_info` | `sensor_msgs/CameraInfo` | **`nvblox_node` 전용** — 다른 소비자 없음 | `plans/2026-08-05-cumotion-bringup.md:608` |
| `/camera/camera/depth/color/points` | `sensor_msgs/PointCloud2` | **MoveIt `move_group` octomap updater — 유일한 포인트클라우드 소비자** | `m0609_rg2_moveit/config/sensors_3d.yaml:38` |

**포인트클라우드를 쓰는 노드는 `move_group` 하나뿐이다.** `pick_fsm`·`graspgenx_perception`·
`cumotion` 중 `PointCloud2` 를 구독하는 코드는 **없다**(전 노드 `create_subscription` 전수 확인).

- `graspgenx` 는 포인트클라우드 토픽을 안 받고 **`aligned_depth_to_color` + `camera_info` 로
  직접 역투영해서** 점군을 만든다(`capture_graspgenx_scene.py`). 그래서 depth·info 가 짝이
  맞는지를 코드가 검사한다(`:544-553` — fx 가 ~640 vs ~900 으로 갈리는 사고 방지).
- `cumotion` **패키지 코드**가 보는 건 nvblox 가 낸 `visualization_msgs/Marker` 복셀이지
  원본 점군이 아니다(`cumotion/arm.py:416`). 카메라를 직접 먹는 건 **컨테이너 안의
  `nvblox_node`·`robot_segmenter_node`** 다 — 아래 (a-2).
- 파생 토픽 `/yolo_seg/{mask,labels,classes,overlay/compressed}` 는 **우리 내부**다.

#### (a-2) 🔴 nvblox/cuMotion 은 octomap 과 **같은 카메라 파라미터를 쓰지 않는다**

같은 D435i·같은 해상도이지만 **먹는 표현이 다르다.** 이걸 혼동하면 "장애물 설정을 바꿨는데
아무 변화가 없다" 가 나온다.

| | MoveIt octomap | nvblox → cuMotion |
|---|---|---|
| 입력 | `depth/color/points` (**PointCloud2**) | `aligned_depth_to_color/image_raw` + `…/camera_info` + `color/image_raw` + `color/camera_info` (**Image**) |
| 설정 위치 | `sensors_3d.yaml` | `nvblox_base.yaml` + CLI 리매핑 |
| 로봇 자기몸 제거 | `padding_offset`·`padding_scale` (**내장 self-filter**) | ❌ 없음 → **`robot_segmenter_node` 를 반드시 끼운다** (`distance_threshold:=0.15`) |
| 갱신 억제 | `max_update_rate`·`point_subsample` | nvblox 자체 설정 |

근거: 리매핑 4줄은 `plans/2026-08-05-cumotion-bringup.md:601-611`, 세그멘터 필수 사유·명령은
`context/constraints.md:365-395`(2026-08-06 실측 — 없으면 cuMotion 계획이 **전부**
`INVALID_START_STATE_WORLD_COLLISION` 로 실패한다). 세그멘터를 끼우면 nvblox 의 depth 입력만
`/cumotion/camera_1/world_depth` 로 바뀌고 **`camera_info`·color 는 원본 그대로**다.

**여기서 나오는 결론 3개:**
1. **`pointcloud.enable=true` 는 octomap 전용이다.** nvblox 는 점군을 안 본다 → cuMotion 으로
   완전히 넘어가면 끌 수 있고, 그러면 로컬 **~245 Mbps 발행이 통째로 사라진다.**
   지금은 못 끈다 — `cumotion/config/dynamic_avoid.yaml:92` 가 `planner:=ompl`(octomap) 경로를
   비교용으로 남겨두고 있다.
2. **해상도는 두 경로에 동시에 걸린다.** `align_depth` 가 공통 조상이라
   `depth_profile`/`color_profile` 을 바꾸면 octomap 도 nvblox 도 같이 바뀐다 →
   **D5(480x320)는 VLA 만의 문제가 아니라 cuMotion 경로도 흔든다.**
3. **로봇 필터링 파라미터가 두 군데에 따로 산다.** 한쪽을 튜닝해도 다른 쪽엔 영향이 0 이다.

#### (b) 🔴 대역폭 — 핫스팟에서 무엇이 통과하고 무엇이 못 하는가

현재 기본 프로파일은 `424x240x15`(depth·color 둘 다, `camera.launch.py:74-77`).
`align_depth.enable=true` 라 **depth 가 color 해상도로 리샘플**되므로 color 프로파일이 둘 다 지배한다.

| 스트림 | 424x240x15 (현 기본) | 480x320x15 (요청값) |
|---|---|---|
| `color/image_raw` (rgb8) | 305 KB/f → **36.6 Mbps** | 461 KB/f → **55 Mbps** |
| `aligned_depth…/image_raw` (16UC1) | 204 KB/f → **24.4 Mbps** | 307 KB/f → **37 Mbps** |
| `depth/color/points` (PointCloud2) | ~2.0 MB/f → **~245 Mbps** | ~3.1 MB/f → **~368 Mbps** |
| `color/image_raw/compressed` (JPEG q80) | ~10–15 KB/f → **~1.5 Mbps** | ~15–20 KB/f → **~2 Mbps** |
| `camera_info` | 수백 B → 무시 가능 | 무시 가능 |

> 산출: `W×H×바이트/픽셀×fps`. PointCloud2 는 `point_step` 을 16–20 B 로 잡았다 — ⚠️ **미검증**
> (`ros2 topic echo --field point_step /camera/camera/depth/color/points --once` 로 확인할 것).
> JPEG 는 우리 실측 압축비에서 환산했다: 848×480 bgr8 1.16 MB → q80 **36 KB**(33배,
> `yolo_seg_node.py:179-182`). 픽셀 수 비례로 축소한 값이라 **씬 복잡도에 따라 2배 이상 흔들린다.**

**결론 3줄:**
1. **포인트클라우드는 절대 보내지 않는다.** 245 Mbps 는 유선 기가비트도 버거운데 핫스팟은 논외다.
   소비자도 로컬 `move_group` 하나뿐이라 보낼 이유가 없다.
2. **raw 컬러도 보내지 않는다.** 480x320x15 raw 만으로 55 Mbps — 휴대폰 핫스팟 실효 대역
   (5 GHz 양호 시 20~50 Mbps, 2.4 GHz 면 그 절반)을 **혼자 다 먹거나 넘는다.**
3. → **`…/color/image_raw/compressed` 만 보낸다** (~1.5–2 Mbps). VLA 가 하는 일은
   "어떤 물체인지" 판단이고, 그건 JPEG 로 충분하다.

#### (c) 💡 영상이 넘어가면 **D3(좌표계 합의)가 사라진다**

이게 이번 변경의 진짜 이득이다. VLA 가 **우리 D435i 화면**을 보고 있다면, "이 물체"를
가리키는 가장 정확한 방법은 base XY 가 아니라 **그 이미지의 픽셀 `(u, v)`** 다.

```
지금까지의 전제 (§3-2)          영상을 넘긴 뒤
VLA: C270 → homography → base XY   VLA: 우리 컬러 프레임의 픽셀 (u,v)
우리: D435i → T_cam2base → base XYZ 우리: 같은 프레임의 depth 로 역투영
      ↕ 두 캘리브 오차가 합산               ↕ 캘리브가 하나뿐 — 합산할 게 없다
      6 cm 예산 중 40.1 mm 를 이미 소진      예산 문제 자체가 없다
```

- 픽셀 지정은 **§5 클릭 경로와 완전히 같은 입력**이다(`rqt_image_view` 가 내는
  `geometry_msgs/Point` 도 소스 이미지 픽셀 좌표다). 즉 **구현이 하나로 합쳐진다** —
  VLA 는 "사람 대신 클릭하는 클라이언트"가 된다.
- **카메라가 고정(eye-to-hand)이라 팔이 움직여도 픽셀 좌표가 유효하다.** 이건 손목 카메라면
  성립하지 않는 성질이다.
- ⚠️ 단, **VLA 가 보는 것은 압축·리사이즈된 프레임**일 수 있다. 픽셀 좌표를 보낼 때는
  **어떤 해상도 기준인지**를 같이 보내야 한다(아래 `pixel_wh`). 안 그러면 조용히 어긋난다.

→ **§2 지시 채널에 `pixel` 필드를 추가한다. `base_xy` 보다 이쪽을 우선한다.**
`base_xy` 는 VLA 가 자기 C270 만 보는 폴백 경로로 남긴다.

#### (d) 해상도 — ⚠️ `480x320` 은 D435i 프로파일에 없을 가능성이 높다

D435i 가 지원하는 값은 컬러 `320x180 / 320x240 / 424x240 / 640x360 / 640x480 / 848x480 / …`,
depth `424x240 / 480x270 / 640x360 / 640x480 / 848x480 / …` 다 — **`480x320` 은 어느 쪽에도
없다**(추론, 개인PC라 카메라가 없어 확인 못 함). 드라이버는 없는 프로파일을 요구하면
**기동 시 죽거나 임의 프로파일로 대체**한다.

```bash
# 실기(rokey)에서 먼저 확인할 것 — 카메라를 꽂고
rs-enumerate-devices | grep -i -A40 "Stream Profiles"
```

지원 목록에 없으면 후보는 **`424x240`(현 기본, 그대로 두면 아무것도 안 바꿔도 된다)** 또는
**`640x480`**. 올릴 때는 대역폭뿐 아니라 **`move_group` octomap updater 가 단일 스레드**라는
제약이 같이 걸린다(`camera.launch.py:52-58`) — 즉 **VLA 때문에 해상도를 올리면 우리 쪽
octomap 이 먼저 밀린다.** 올려야 하면 `sensors_3d.yaml` 의 `max_update_rate`·`point_subsample`
을 같이 본다.

#### (e) 핫스팟에서 DDS 가 걸리는 지점

| 위험 | 왜 | 조치 |
|---|---|---|
| 🔴 VLA 가 실수로 `depth/color/points` 를 구독 | 링크가 즉시 포화되고, **로컬 octomap 경로까지 같이 죽는다**(같은 발행 노드·같은 송신 경로) | 구독 화이트리스트를 §2 문서에 못박는다. 재발하면 `ros-humble-domain-bridge` 로 넘길 토픽만 명시 |
| 멀티캐스트 미전달 | 휴대폰 핫스팟은 멀티캐스트/AP 격리 동작이 기기마다 다르다. **탐색만 실패하고 에러는 없다** | `fastdds_udp_only.xml` 에 상대 IP 를 `initialPeersList` 유니캐스트로 명시 |
| 지연·지터 | DDS RELIABLE 은 재전송으로 버티다가 큐가 밀린다 | 영상은 **`SensorDataQoS`(BEST_EFFORT)** 로 받는다. 지시 JSON 만 RELIABLE |
| 핫스팟 IP 가 매번 바뀐다 | 유니캐스트 피어 목록이 무효화된다 | 접속할 때마다 `ip a` 로 확인. 고정이 필요하면 휴대폰 DHCP 예약 |

> 증상 구분은 §3-1 의 **"도메인이 탐색을, 프로파일이 데이터를 결정한다"** 를 그대로 쓴다.
> 여기에 한 줄 추가된다: **토픽도 보이고 데이터도 오는데 프레임이 뚝뚝 끊기면 대역폭이다.**

---

## 4. 폐기된 설계 — `vla_wrist` 대체안 (1차 개정 "C안")

**1차 개정은 `vla_wrist` 자리에 `grasp_bridge_node` 를 꽂아 `GraspRequest`/`GraspPlan` 을
말하게 하자고 적었다. §0 확정으로 그 전제가 사라졌다** — VLA 가 로봇을 움직이지 않으므로
`GraspPlan`(Doosan `posx`, mm, ZYZ deg, 접촉점)을 만들어 줄 이유가 없다.

기록으로만 남긴다 (그쪽 PC 에서 `vla_wrist` 를 살릴 때 필요할 수 있다):

- `GraspRequest`: `object_id`, `class_name`, `expected_base`(Point, m)
- `GraspPlan`: `target_posx[6]`(접촉점, mm + ZYZ deg), `pregrasp_posx[6]`, `confidence`,
  `approach_tilt_deg`, `candidate_count`, `cloud_points`
- `vla_wrist` 의 좌표 체인은 **손목 카메라 전제**다 — `posx(TCP) @ T_gripper2camera`
  (`wrist_geometry.py:157 camera_to_base_mm`), `expected_tcp_name: GripperDA_v1` 에 묶임.
  **우리 리그(고정 eye-to-hand)에서는 성립하지 않는다.**

**단, `GraspRequest` 의 필드 구성은 §2 JSON 이 그대로 물려받았다** — `class_name` +
`expected_base` 는 "어느 개체인가"를 카메라 경계 너머로 나르는 유일한 실용적 키다.

---

## 5. 물체 선정(target selection) — 우리 쪽 구현. **미구현**

> 📌 **선행 문서**: [[ws/cobot2/plans/2026-08-07-graspgenx-target-matching]] 이 "무슨 **종류**를
> 잡나"(`target_classes` 배선)까지를 소유한다. **이 절은 그 다음 단계인 "어느 개체를 잡나"만**
> 다룬다. 겹치는 값을 여기에 다시 적지 않는다.
>
> VLA 통합과 **독립적으로도 필요하다.** 사람이 클릭으로 고르는 경로가 먼저 있어야 VLA 가
> 틀렸을 때 무엇이 틀렸는지(지시가 틀렸나, 우리 매칭이 틀렸나) 분리할 수 있다.

### 지금 어디가 비어 있나

| 단계 | 수단 | 상태 |
|---|---|---|
| 무엇이 보이나 | `/yolo_seg/classes` (label·class·conf) | ✅ — **좌표가 없다** |
| 무슨 **종류**를 잡나 | `target_classes` (워커 호출 **전** 필터) | ✅ |
| **어느 개체**를 잡나 | `target='obj_2'` (워커 호출 **후**) | ❌ `obj_N` 은 프레임마다 바뀜 → 실사용 불가 |

### 선택 키는 id 가 아니라 **base XY 좌표**

- **VLA 와 우리는 카메라가 다르고 이제 PC 까지 다르다.** `apple_17` 같은 id 는 경계를 넘지
  못한다. 좌표는 넘는다(§3-2).
- **`/grasp/scene` 2단계 프로토콜(scene_id 핸들)은 만들지 않는다.** 그 설계가 필요했던
  이유는 `obj_N` 이 프레임 종속이라서인데, **좌표는 프레임 독립이라 핸들이 필요 없다.**
  (`graspgenx_perception/README.md` "다음 방향" 절의 옛 제안을 이걸로 대체한다.)
- 우리 카메라가 **고정**이므로 화면 클릭의 픽셀 좌표도 팔이 움직여도 유효하다. 다만 VLA
  경로가 어차피 base XY 로 들어오므로 **내부 표현은 base XY 하나로 통일**한다(로직 두 벌 방지).

### 입력 3개 → 내부 표현 1개

```
[클릭]  /yolo_seg/overlay_mouse_left  geometry_msgs/Point (픽셀)  ← rqt_image_view
[VLA]   /vla/pick_command             std_msgs/String (JSON)      ← §2
[수동]  /grasp/target                 geometry_msgs/PointStamped  ← CLI 디버깅용
                          ↓  전부 (class, base XY) 로 정규화
                     select_by_point()
```

**클릭 UI 는 만들 필요가 없다.** `rqt_image_view` 에 "publish click location" 체크박스가
이미 있고 `<이미지토픽>_mouse_left` 로 `geometry_msgs/Point`(원본 이미지 픽셀 좌표)를
발행한다 — `librqt_image_view.so` 심볼(`onMouseLeft`, `_mouse_left`,
`Publisher<geometry_msgs::msg::Point>`)로 2026-08-08 확인.

### 꽂는 자리 — `compute()` 의 `segment()` 직후 한 곳

`grasp_bridge_node.py:285` 바로 뒤. 여기면 `yolo`/`geometric` **두 경로 다** 커버되고,
워커 호출 전이라 GraspGenX 연산 자체가 1개로 줄어든다(진짜 병목은 워커 수십 초).

```python
# capture_graspgenx_scene.py 에 추가
def select_by_point(seg, label_map, X, Y, tx, ty, radius, margin):
    """지정 XY 에 가장 가까운 obj 하나만 남긴다 -> (seg, label_map, 진단)."""
    d = sorted((float(np.hypot(np.median(X[seg == v]) - tx, np.median(Y[seg == v]) - ty)), n, v)
               for n, v in label_map.items() if n.startswith('obj_'))
    if not d or d[0][0] > radius:
        return None, None, f'({tx:+.3f},{ty:+.3f}) 반경 {radius}m 안에 물체 없음'
    if len(d) > 1 and d[1][0] - d[0][0] < margin:
        # 잘못된 물체를 집는 것보다 안 집는 게 낫다 — VLA 의 refuse_ambiguous_match 와 같은 판단
        return None, None, f'모호: {d[0][1]} {d[0][0]:.3f}m vs {d[1][1]} {d[1][0]:.3f}m'
    out = seg.copy()
    for _, n, v in d[1:]:
        out[out == v] = 0        # 배경으로. 점군엔 남으므로 충돌 컨텍스트는 유지된다
    keep = {k: v for k, v in label_map.items() if not k.startswith('obj_') or v == d[0][2]}
    return out, keep, f'{d[0][1]} 선택 (지정점에서 {d[0][0]:.3f}m)'
```

- `X, Y` 는 `workspace_mask()` 가 이미 계산해 둔 base 프레임 좌표다.
- **`label_map` 의 `ground`/`table` 항목은 지우지 않는다** (`obj_` 만 거른다).
- 클릭 → base 변환: `to_base(depth, K, T_base_cam)[v, u]`. depth 구멍 방어로 5×5 median.
  ⚠️ 클릭 픽셀의 depth 가 0 이면 실패한다 — 그때만 최근접 centroid 로 폴백할지는 실기 판단.

### 파라미터 — VLA 와 같은 이름·같은 값

| 이름 | 값 | 근거 |
|---|---|---|
| `match_tolerance_m` | **0.06** | VLA `system.yaml:107` 과 동일. 두 시스템이 다른 허용오차를 쓰면 "VLA 는 지목했는데 우리가 못 찾는다"가 난다 |
| `refuse_ambiguous_match` | **true** | VLA `system.yaml:110` |
| 지시 TTL | 초안 **10 s** | ⚠️ VLA 의 `max_scene_age_s: 2.0`(`system.yaml:195`)은 **webcam 씬 신선도**이지 지시 TTL 이 아니다. **같은 개념이 아니므로 값을 맞추려 하지 말 것** |

### 무지정일 때 (폴백 정책 — 지금 코드에 안 적혀 있다)

현재 `select()` 는 **점수 최고**를 고른다(`grasp_bridge_node.py:132`). 이걸 **정책으로
명시**하고, 후보가 2개 이상이면 `지정 없음, 점수로 골랐다: obj_1 0.71 vs obj_3 0.68` 을
로그에 남긴다. 로그가 없으면 "왜 저걸 집었지"를 되짚을 수 없다.

### TTL 이 필요한 이유

지시를 소비하면 지운다. 안 지우면 10분 전 지시로 다음 픽이 나간다. 초과면 **"지정 만료"로
실패**시킨다 — 조용히 무지정 폴백으로 떨어지면 다른 물체를 집는다.

### 🟡 서비스 경로 — `/grasp/compute_grasp` 서버 부재

- `pick_fsm` 의 기본값 `grasp_source: compute_grasp` 가 부르는
  **`/grasp/compute_grasp`(`pick_fsm_msgs/ComputeGrasp`) 서버는 이 ws 어디에도 없다.**
  `grasp_bridge_node.py:151` 이 만드는 건 `/grasp/compute`(`std_srvs/Trigger`) 하나다.
  → 지금은 `grasp_source:=legacy_trigger` 를 **명시해야** 돈다.
- `ComputeGrasp.srv:8` 에 이미 **`string target`** 이 있다. 여기에
  `geometry_msgs/Point target_point` 를 얹으면 "클래스 + 좌표"가 한 요청에 들어가고
  TTL·경합이 사라진다.
- **다만 §0 의 "의존성 최대한 보존" 원칙상 이번 범위는 아니다.** 지금 도는 조합
  (`legacy_trigger`)을 건드리지 않는 것이 우선이다. 별도 작업으로 남긴다.

---

## 6. 우리 내부 정리 (VLA 와 무관하지만 통합 전에 걸린다)

### 6-1. 🔴 fingertip — 두 패키지가 다른 손끝 모델을 쓴다 (D4)

`md/context/constraints.md:900-906` 실측:

| | 값 |
|---|---|
| GraspGenX `config.json` `fingertip[2]` | **0.180 m** (모델 conditioning 값) |
| **실측** `rg2_base_link → 손끝`(닫힘) | **0.218 m** (= 240 − 22) |
| 개구 100 mm 일 때 | 0.177 m (힌지 구조, 비선형) |

`pick_fsm` 은 2026-08-07 에 `rg2.fingertip_from_rg2_base_m(width_m)` 로 갈아탔다
(`rg2.py:73`, `task_manager.py:480/521/595`). **`grasp_bridge_node.py:56` 의
`tcp_offset_m: 0.18` 은 아직 옛 상수다.**

지금은 `tcp_offset_m` 이 **로그·RViz 표시용**(`/grasp/best_tcp`)이라 모션에 직접 안 들어가지만,
**두 패키지가 같은 물리량에 다른 값을 쓰는 상태**다. `constraints.md:892` 가 같은 종류의
차이를 "MoveIt 이 손끝을 40 mm 더 깊이 밀어넣는다"로 경고해 뒀다.

→ **D4**: 180 은 GraspGenX 가 모델을 조건화한 값이라 바꾸면 grasp 품질이 흔들릴 수 있다.
계약값은 두고 **소비자에서만** 218 로 보정하는 쪽이 안전해 보이나 **실기 확인이 필요하다.**

### 6-2. 🔴 `~/.local` 오염 — 우리 `pytest` 가 깨져 있다

VLA 를 이 PC 에 설치할 이유는 없어졌지만 **이미 깔린 것은 남아 있다** (2026-08-08 실측):

```
torch 2.7.1 · torchvision 0.22.1 · ultralytics 8.4.76 · opencv-python 4.10.0
numpy 1.24.4 · anyio 4.13.0
```

`import cv2` 가 apt 4.5.4 가 아니라 `~/.local` 4.10.0 을 잡는다.
(apt numpy 1.21.5 는 이번에 재확인하지 않았다 — `graspgenx_perception/README.md:32` 의
2026-08-07 기록에서 옮긴 값.)

| 항목 | 결과 |
|---|---|
| `cv_bridge` + pip cv2 4.10 동시 로드 → 이미지 왕복 | **정상** (segfault 없음) |
| `pytest src/graspgenx_perception/test/*` | 🔴 **깨진다** — `ModuleNotFoundError: _pytest.scope` |
| `pytest -p no:anyio ...` | ✅ **PASS 24개** (0.44s) |

**우리가 할 것**: 테스트에 `-p no:anyio` 를 붙인다. 우리 노드는 계속 컨테이너(YOLO)와
`uv` venv(GraspGenX)를 쓰고 `~/.local` 의 torch 를 **쓰지 않는다.**
`~/.local` 을 실제로 지울지는 다른 팀원 작업에 영향을 줄 수 있어 **사용자 판단**이다.

### 6-3. 루트 `README.md` 잔여 항목

§0 확정으로 대부분 소멸했다. **남은 것 2개**:

| 항목 | 조치 |
|---|---|
| `README.md:198` **결합점 표** | **`ROS_DOMAIN_ID` 행 추가** — 이제 PC 가 둘이라 "한 곳만 어긋나면 조용히 깨지는" 대표 사례가 됐다 |
| `README.md:29` "전체 의존성은 `requirements.txt` 한 파일" | 지금 거짓은 아니다(VLA 를 여기 설치하지 않으므로). **단 §6-2 오염 사실을 어딘가 적어야 팀원이 pytest 실패를 재진단하지 않는다** → `md/state.md` 열린 이슈에 있음 |

**2026-08-08 에 이미 고친 것**: `README.md:68` 에 도메인 **93** 명시 + VLA 는 0 이라는 경고,
`README.md:189-190` 상태표의 죽은 포인터 갱신 + VLA 통합 행 신설.

---

## 7. 다음 세션 시작 절차

```bash
# 1. ~/.local 오염이 그대로인지 (§6-2)
ls -d ~/.local/lib/python3.10/site-packages/{torch,cv2,ultralytics,anyio} 2>/dev/null

# 2. 우리 테스트가 도는지
source /opt/ros/humble/setup.bash
python3 -m pytest -p no:anyio src/graspgenx_perception/test/test_yolo_seg.py \
                              src/graspgenx_perception/test/test_best_labels.py -q

# 3. VLA repo 스냅샷이 바뀌었는지 (지금은 로컬에도 사본이 있다. 곧 외부 PC 로 간다)
cd ~/M0609_VLA_system && git log --oneline -3     # 이 문서 기준: 5a10649

# 4. [실기 rokey 에서만] D435i 가 480x320 을 지원하나 (D5, §3-3(d))
rs-enumerate-devices | grep -i -A40 "Stream Profiles"

# 5. [실기 rokey 에서만] 포인트클라우드 실제 크기 — §3-3(b) 표의 245 Mbps 검산
ros2 topic echo --field point_step /camera/camera/depth/color/points --once
ros2 topic bw /camera/camera/depth/color/points
ros2 topic bw /camera/camera/color/image_raw/compressed

# 6. [핫스팟 연결 후] 실효 대역폭 — 위 값들과 대조한다
iperf3 -c <VLA_PC_IP> -t 10
```

**작업 순서 (권장)**:
1. **§5 선정 로직을 먼저 짠다** — VLA 없이 클릭만으로 끝까지 동작시킨다. 이게 되면
   VLA 는 "클릭 대신 JSON 을 쏘는 것"일 뿐이다. **`pixel` 경로가 확정되면서 클릭과
   VLA 지시가 같은 입력이 됐으므로**(§3-3(c)) 이 순서의 이득이 더 커졌다.
2. §2 지시 채널(JSON 구독 + 스키마 검증 + `pixel_wh` 스케일링)을 얹는다.
3. **그 다음에** 핫스팟·도메인·대역폭(§3-3). 여기서 D5(해상도)와 실효 대역폭을 먼저 재고
   보낼 토픽을 확정한다.

---

## 8. 미검증 — 다음 세션에서 확인할 것

| 항목 | 왜 미검증인가 |
|---|---|
| **VLA 를 실제로 띄워 본 적이 없다** | 소스·config·msg 정의만 읽었다 |
| 🔴 **D435i 가 `480x320` 을 지원하는가** | 지원 목록에 없어 보인다(§3-3(d)). 개인PC라 카메라가 없어 `rs-enumerate-devices` 를 못 돌렸다. **실기에서 이것부터 확인** |
| 🔴 **핫스팟 실효 대역폭** | 20~50 Mbps 는 일반론이다. 실측 안 함 — `iperf3` 로 재고 §3-3(b) 표와 대조 |
| `depth/color/points` 의 `point_step` | 16~20 B 로 가정하고 245 Mbps 를 산출했다. 실측 안 함 |
| JPEG 압축비 (424x240 기준) | 848x480 실측 33배를 픽셀 비례로 환산한 값이다. 씬 복잡도로 2배 이상 흔들린다 |
| 핫스팟에서 DDS 멀티캐스트 탐색이 되는가 | 기기마다 다르다. 안 되면 `initialPeersList` 유니캐스트 (§3-3(e)) |
| VLA 가 낼 `position_base` 가 우리 `base_link` 와 같은 프레임인가 (D3) | 🔻 **강등** — `pixel` 경로를 쓰면 불필요(§3-3(c)). C270 폴백 경로에서만 필요 |
| 두 PC 간 DDS 도달성(같은 LAN? 멀티캐스트? 방화벽?) | 🟢 **링크 확정: 개인 휴대폰 핫스팟**(2026-08-08). 도달성 자체는 아직 미실측 |
| 우리 캘리브 잔차 40.1 mm 가 `match_tolerance_m` 0.06 안에 드는가 | 예산의 3분의 2를 여기서 쓴다 (§3-2) |
| §5 선정 로직 | **코드 한 줄도 안 짰다.** 설계뿐 |
| **D4 (fingertip 180 vs 218)** | GraspGenX 를 218 로 조건화하면 grasp 품질이 어떻게 변하는지 모른다 |
| `standoff_m: 0.04`(VLA) vs `approach_offset_m: 0.10`(우리) | 역할이 같은지 확인 안 함. §0 확정으로 우선순위는 내려갔다 |
| apt numpy 1.21.5 | 이번 세션에 재확인 안 함 |

> 🔎 **이 문서는 2026-08-08 session-auditor 감사 후 2차 개정됐다.**
> 1차에서 잡힌 것: fingertip 0.18 을 "맞아 있음"으로 적은 것(→ §6-1), `max_scene_age_s`
> 의미 혼동(→ §5), 서비스 경로 누락(→ §5 말미), 선행 문서 경계 미표기(→ §5 머리말).
> 2차 개정 사유: **사용자가 §0 범위를 확정** — 1차의 D1·D2·C안·드라이버 경합·C270 이 폐기됐다.
