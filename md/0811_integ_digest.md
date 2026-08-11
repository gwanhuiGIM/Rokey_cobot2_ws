<!-- meta
updated: 2026-08-11
status:  digest — 이 날 VLA↔FSM 통합 세션에서 쌓인 지식 요약. 값의 정본은 각 코드/문서이고
         (아래 "참조") 이 파일은 "그날 무엇을 배웠나"를 한자리에 모은 것이다. 로그처럼
         불어나지 않는다 — 사실이 바뀌면 정본을 고치고 여기 요약도 덮어쓴다.
owns:    없음(파생 문서). 🟢=이 날 실기로 확인, 🔴=미검증
-->

# 0811 통합 digest — VLA 노드 ↔ pick_fsm

> **한 줄**: 외부 PC 의 VLA 가 "어느 **개체**를 집을지"를 픽셀로 지목하면, cobot2_ws 가 그
> 픽셀을 base XY 로 바꿔 같은 클래스 여러 개 중 하나만 골라 GraspGenX 로 넘기는 경로
> (`select_by_point`)를 구현하고 **실기로 PERCEIVE 까지 관통**시켰다.

관련 정본:
[[ws/cobot2/plans/2026-08-08-vla-integration]](설계·히스토리 단일 출처, §5 가 select_by_point 정본) ·
[[ws/cobot2/vla-bridge-contract]](외부 repo 와의 계약) ·
[`src/PACKAGES.md#voice_processing`](../src/PACKAGES.md) ·
[[ws/cobot2/context/constraints]](실기 상수)

---

## 1. 통합 아키텍처 — push(VLA) vs pull(FSM) 를 잇는 한 건짜리 래치

**역할 경계**: VLA(외부 PC, `~/M0609_VLA_system`)가 "무엇을", cobot2_ws 가 "어떻게"를 소유한다.
FSM 은 이미 `LISTENING` 상태에서 `/get_keyword`(Trigger)를 부르는 **음성 노드 자리**를 갖고
있어서, VLA 를 "사람 대신 말해주는 클라이언트"로 그 자리에 꽂으면 **FSM 을 안 고쳐도 된다.**

```
VLA PC ──/vla/pick_command(JSON)──▶ vla_command_node ──/get_keyword(Trigger)──▶ task_manager
   │  {class, pixel, pixel_wh,           │ (pixel_policy=select 일 때만)              │
   │   request_id, place, stamp_ns}      ├──/pick/target_pixel(JSON x,y,w,h)──▶ _on_target_pixel
   │                                     ├──/pick/place_location──────────────▶ _on_place_location
   ◀──/vla/pick_result(JSON)────────────┘                                         │
                                                              PERCEIVE 진입: _push_bridge()
                                                              SetParameters(pixel_x/y/w/h,
                                                              target_classes, seg_source)
                                                                            │
                                                            grasp_bridge_node.compute():
                                                            segment() → pixel_to_base()
                                                            → select_by_point() → GraspGenX
```

**왜 이렇게**: VLA 는 아무 때나 쏘고(push), FSM 은 `LISTENING` 에 와야 묻는다(pull). 그래서
`vla_command_node` 는 지시를 붙잡고 있다 FSM 이 물을 때 건네는 **한 건짜리 래치**다. 어긋나는
지점(TTL·"FSM 이 아직 듣나"·"이 사이클이 끝났나")을 코드가 명시적으로 방어한다.

---

## 2. 핵심 설계 판단 — 왜 픽셀이 아니라 base XY 로 매칭하나 🟢

VLA 와 cobot2_ws 는 **다른 프로세스·다른 프레임**에서 돈다. 그래서:

- VLA 는 **픽셀**로 지목한다(자기가 보는 이미지 좌표).
- cobot2_ws 는 그 픽셀을 **브리지에서 딱 한 번** base XY 로 바꾼다(카메라 TF 사용,
  `pixel_to_base()`). 이후 매칭은 전부 base XY 최근접이다.
- **`obj_N` 라벨 id 로 매칭하지 않는다** — 라벨은 캡처마다 재부여되어 프레임 의존적이다.

**오늘 이 판단이 옳았음을 실측이 증명했다**: 같은 픽셀 `(749,383)` 이 첫 캡처에선 `obj_1`,
재촬영 뒤엔 `obj_2` 로 **라벨이 뒤바뀌었는데도**, base XY 최근접(+0.23~0.25, +0.055 근처)이
매번 같은 물리 물체를 정확히 찾았다. 라벨 id 로 매칭했다면 재촬영에서 엉뚱한 걸 집었을 것이다.

---

## 3. 오늘 실기로 검증된 것 🟢 (domain 93, 실 D435i)

**전 구간 관통(PERCEIVE 까지)**: `vla_command_node`(pixel_policy=select) + `task_manager`
(voice:=false) + `grasp_bridge_node`(seg_source=geometric) 동시 기동.

| 단계 | 실측 결과 |
|---|---|
| VLA 흉내 `pixel:[749,383], pixel_wh:[1280,720]` | `/pick/target_pixel = {x:749,y:383,w:1280,h:720}` |
| task_manager 수신 | `개체 선정 좌표 수신: (749,383) / 기준 1280x720` |
| PERCEIVE push | `브리지 설정: target_classes='(전부)', seg_source=geometric, pixel=(749,383)` |
| 첫 캡처 | obj_1 선정(base +0.227,+0.056, 지정점 0.022m) → **collision 0/157** → 후보 0 → 재촬영 |
| 재촬영(자동 재시도 1/2) | **obj_2 선정(base +0.247,+0.054, 지정점 0.007m)** → collision 9/239 → 성공 |
| GraspGenX | `score=0.701, 손끝=(+0.235,+0.036,+0.025), 폭=39.9mm` |
| SCENE_PREP | MoveIt 없음 → 10s 타임아웃 → **안전하게 ABORT→SAFE_STOP** |

- 실물체 6~9개 씬에서 지정 픽셀 개체를 **7mm 오차**로 선정.
- 실패 경로도 검증: 배경 픽셀 `(5,5)` → `base=(-4.521,-3.974)` → `반경 0.060m 안에 물체 없음`
  으로 **워커 호출 전 조기 거부**(GPU 낭비 없음).

**카메라 실측**(오늘, color 1280x720 → aligned_depth 도 1280x720 추종):
- K = `fx909.53 fy909.20 cx659.54 cy370.20`
- TF `base_link→camera_color_optical_frame` Translation `[1.264, -0.053, 0.760]`
- `table_z` 자동추정 `-0.0099 ~ -0.0142 m` (캡처마다 흔들린다)

---

## 4. 로보틱스 지식 (통합하며 확인/재확인)

- **GraspGenX grasp 원점 = 그리퍼 base, TCP 는 +Z 로 0.18m**(RG2 fingertip). grasp 를 물체
  위치로 읽으면 접근이 기울수록 옆으로 벗어난다 — 그래서 TCP 를 따로 계산·발행한다. 🟢
- **테이블도 장애물이다**: collision 점군엔 seg 와 무관하게 유효 depth 가 전부 들어간다
  (`collision_threshold=0.02m`). 납작한 물체를 위에서 잡으면 손끝이 테이블 2cm 안에 들어와
  **정상 grasp 까지 전멸**할 수 있다 — 0개가 나오면 여기부터 의심. 🟢(오늘 첫 캡처 0/157)
- **depth 노이즈로 grasp 후보가 요동친다**: 같은 물체·같은 자리에서 collision-free 비율이
  0%↔4%(0/157 → 9/239) 로 흔들렸다. **재촬영 한 번**으로 살아났다 — `_perceive_failed` 의
  자동 재촬영 재시도가 이걸 흡수한다(설계대로 동작 🟢).
- **pixel_to_base 의 depth 구멍 방어**: 지정 픽셀이 구멍(depth=0)이면 이웃 5×5 median 으로
  메운다. 전부 구멍이면 None(선정 실패). 🟢(단위테스트)
- **모호성 거부**: 지정점 반경 `match_tolerance_m`(0.06m, VLA `system.yaml` 과 맞춤) 밖이면
  거부, 2등 후보가 `ambiguity_margin_m`(0.02m, **초안값 🔴**) 안으로 붙으면 "모호"로 거부.
  틀린 물체를 집는 것보다 안 집는 쪽이 안전하다는 판단(`refuse_ambiguous_match`).
- **nan 센티널**: "지정 없음"은 `pixel_x/y/w/h = nan` 으로 표현(이 ws 의 `table_z`/`class_dims`
  관례와 동일). 지정 없는 보통 pick 은 nan 이 그대로 와서 기존 "점수 최고" 동작과 같다. 🟢

---

## 5. ROS / DDS / 인프라 함정

- **도메인 격리**: 이 랩탑은 여러 계정·세션이 **같은 domain 93**을 공유한다. 단독 노드
  테스트는 빈 도메인(예: 77)으로 격리했고, 실카메라가 필요할 땐 93 을 공유했다(사용자가
  "다른 세션은 cumotion 비전이라 안 겹친다" 확인). 🟢
- **QoS 매칭**: `/pick/target_pixel`·`/pick/place_location` 은 `TRANSIENT_LOCAL depth=1`
  (늦게 붙어도 마지막 값 수신). 발행자가 이보다 얕은 durability 면 **조용히 매칭 안 됨.**
  `vla_command_node` 의 `PLACE_QOS` 와 `task_manager` 의 `TARGET_QOS` 는 **글자 그대로 같아야**
  한다. 🟢
- **TTL 은 받은 시각 기준**: 지시는 다른 PC 에서 핫스팟 건너 온다 — 시계가 안 맞으므로 송신
  `stamp_ns` 로 나이를 재지 않는다(에코만). 🟢(설계)
- **`/get_keyword` 중복**: 마이크 노드(`get_keyword`)와 `vla_command_node` 둘 다 이 서비스를
  제공 → **동시에 띄우면 어느 쪽이 답할지 모름.** 하나만 띄운다.
- **`ros2 launch ... target:=`(빈 값)은 문법 오류**로 거부된다. 빈 타겟은 인자를 아예 빼고
  파라미터 기본값(자동)에 맡긴다. 🟢(오늘 밟음)
- **SetParameters float 은 DOUBLE 타입 명시**(`float_param`) — 이 ws 는 int/float 혼동으로
  노드가 죽은 이력 다수. 🟢
- **launch 미선언 인자는 경고 없이 무시**된다(`dry_run:=`, `target_classes:=` 등). 🟢

---

## 6. 안전 — 오늘 겪은 것과 규칙

- **승인 게이트**: `dry_run` 은 2026-08-09 제거됨. 남은 소프트 안전장치는 `require_approval`
  하나이고, VLA/음성은 `/pick/approve` 를 **절대 부르지 않는다**(`BLOCKED_CMDS` — 코드 경로
  자체가 없다). 최종 안전장치는 물리 비상정지 버튼.
- 🔴→규칙 **`/dsr01/*` 저수준 드라이버가 이미 떠 있을 수 있다**: 오늘 task_manager 를 띄울 때
  다른 세션이 올려둔 `/dsr01/controller_manager` 등이 살아 있었다(내가 안 올림). `SAFE_STOP→
  HOME` 전이가 관절이동을 시도하는 상태라 잠깐 놀랐으나, **실제 모션은 안 나갔다** — `_move()`
  가 `/move_action`(MoveIt) 준비를 요구하는데 그게 없어서(오늘 `moveit.launch.py` 미기동)
  SCENE_PREP 타임아웃으로 안전하게 멈췄다. **교훈: task_manager 기동 전에 `/dsr01/*` 와
  `/move_action` 존재를 먼저 확인한다**(공유 랩탑이라 다른 세션이 로봇을 살려둔 채 작업 중일
  수 있다). → `constraints.md` 승격 후보.

---

## 7. cross-review 지적과 대응 (HIGH 1건)

- **HIGH(수정 완료 🟢)**: 단발성 픽셀 override(`_pixel_override`)가 성공 경로
  (`_push_bridge`)에서만 삭제되고 `_pushed` 는 매 전이에서 리셋 → **소비 전에 SPEAK_FAIL/
  ABORT 로 빠지면 좌표가 다음 사이클로 새서** 클래스만 지시한 다음 pick 이 엉뚱한 개체를
  고른다. 수정: `_to()` 에서 SPEAK_FAIL/ABORT 진입 시 override 를 지운다(`_st_idle` 시작점
  클리어는 vla 의 "픽셀→start" legit 픽셀을 race 로 지우므로 피함). build PASS, test 113/113.
  - 오늘 라이브 관통은 성공 경로만 탔어서 이 누수는 안 보였다 — **정적 리뷰가 잡았다.**
- note 2건(미수정, 판단 후 플래그): centroid median 이 depth 구멍에 이론상 당겨질 수 있음
  (오늘 7mm 정확도라 실질 영향 미미) / 1회성 픽셀 토픽의 `TRANSIENT_LOCAL` latch 가 노드
  재시작 시 stale 을 줄 수 있음(QoS 계약이라 안 바꿈).

---

## 8. 남은 것 🔴

- **MoveIt 이후 전 구간**(SCENE_PREP → PLAN → 실제 파지·place)은 오늘 안 돌렸다. `moveit.
  launch.py` + 로봇 bringup 을 다 띄운 진짜 pick 한 사이클 미검증.
- **실제 VLA PC 연결** 미검증 — 오늘은 `ros2 topic pub` 로 흉내. 핫스팟·도메인·DDS 도달성
  전부 미실측.
- **같은 클래스 다중 개체 씬**에서의 모호 거부(`ambiguity_margin_m=0.02` 초안값) 실기 튜닝.
- `base_xy` 경로는 여전히 검증만 하고 선정에 안 씀(캘리브가 `match_tolerance_m` 예산 밖).
- `select_by_point` 을 SPEAK_FAIL/ABORT 로 빠뜨리는 **누수 실패경로 자체의 실기 재현**은 안 함
  (코드 경로로만 확인 후 수정).
