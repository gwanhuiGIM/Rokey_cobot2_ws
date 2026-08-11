<!-- meta
updated: 2026-08-11 (§12 신설 — `/vla/pick_status` 상태 미러링 채널. cobot2_ws 쪽
         publisher(`vla_command_node`) 구현 완료, 빌드·단위테스트 34건 PASS. 🔴 실기 미검증.
         VLA 쪽 subscriber/UI 는 그쪽 세션이 구현. 이전: §9 select_by_point() 구현,
         §10 rqt↔VLA cmd 대응표, §11 /pick/retry_place 신설)
status:  live — 값이 바뀌면 이 문서를 덮어쓴다 (append 하지 않는다, 히스토리는 안 남긴다)
owns:    cobot2_ws 가 관리. 정본은 항상 `src/voice_processing/voice_processing/vla_command_node.py`
         (`parse_command()`) 코드다 — 이 문서는 그 요약이라 어긋나면 코드가 이긴다.
         "왜 이렇게 됐는지"(히스토리·근거)는 여기 없다 → `md/plans/2026-08-08-vla-integration.md`
         (그 문서는 계속 불어나도 되는 로그, 이 문서는 계약만 담는 요약이라 안 불어나야 한다)

**단일 사본 원칙 (2026-08-10)**: 이 파일은 cobot2_ws 안에 **이 경로 하나만** 존재한다.
`~/M0609_VLA_system`(같은 GitHub remote `0730_cobo2_personal`의 다른 clone, 브랜치
`vla_integ` — 별개 repo 아님, `git remote -v` 로 2026-08-10 확인됨)에는 **사본을 두지
않는다** — 두 곳에 복사해두면 한쪽만 갱신되고 어긋난다(실제로 2026-08-10 그 clone이
초기화되며 사본이 통째로 날아간 적 있음). 그쪽 세션은 이 파일을 절대경로
`~/cobot2_ws/md/vla-bridge-contract.md` 로 직접 읽는다 — 같은 머신(`rokey`)에서 두 clone이
같이 돌아가는 동안은(2026-08-10 확정, `M0609_VLA_system/2026-08-10-two-pc-fallback.md`)
이 방식이 duplication 없이 가장 단순하다. 나중에 물리적으로 PC 를 분리하면(같은 문서 트리거
조건) 이 파일을 그때 가서 다시 복사/동기화하는 방법을 정한다 — 지금은 안 한다.
-->

# cobot2_ws 브리지 계약 — `vla_pick_bridge`가 지금 맞춰야 하는 것

이 문서를 읽는 쪽(`M0609_VLA_system` clone, 브랜치 `vla_integ`)이 만들 `vla_pick_bridge`가
상대할 것은 cobot2_ws의 `vla_command_node` 하나뿐이다. 그 노드가 받아들이는 것/거부하는
것/부르지 않는 것을 여기 적는다.

## 1. 채널

```
저쪽 → /vla/pick_command (std_msgs/String, JSON) → vla_command_node
저쪽 ← /vla/pick_result  (std_msgs/String, JSON) ← vla_command_node
```

커스텀 msg 없음. `vla_interfaces`를 cobot2_ws에 가져오지 않는다 — 두 clone이 빌드 버전으로
묶이는 걸 피하기 위한 의도된 설계다.

## 2. `/vla/pick_command` 스키마

```json
{"cmd": "pick", "class": "apple", "place": "basket",
 "request_id": "a17-3", "stamp_ns": 1754640000123456789}
```

| 필드 | 규칙 |
|---|---|
| `cmd` | `pick` \| `pick_and_place`(같은 뜻으로 처리) \| `start` \| `abort` \| `reset`. 그 외 값은 거부 |
| `class` | **필수.** 공백 불가 — 있으면 거부(FSM이 응답을 공백으로 쪼개 첫 단어만 쓰기 때문). 여러 개는 콤마(`apple,orange`). `class_name`으로 보내도 받는다(SceneObject 필드명) |
| `place` | **선택.** `basket` \| `table` \| `discard` 중 하나만 허용, 그 외 값은 거부. 안 보내면 cobot2_ws 파라미터 기본값(`basket`)이 그대로 쓰인다 — §5 참고 |
| `request_id` | 그대로 echo. **결과 판정은 반드시 이걸로 대조** — 핫스팟류 연결 끊김 시 결과를 놓칠 수 있다(QoS VOLATILE) |
| `stamp_ns` | 에코만 됨. TTL은 cobot2_ws가 **수신 시각 기준**으로 계산하므로 두 PC 시계 동기화 불필요 |
| `pixel` + `pixel_wh` | **`pixel_policy=select`(신설, 2026-08-11)이면 실제로 개체 선정에 쓰인다** — `select_by_point()`가 cobot2_ws 쪽에 구현됐다(§8/§9, `pixel_x/pixel_y/pixel_w/pixel_h`가 `vla_command_node` → `/pick/target_pixel` → `task_manager` → `grasp_bridge_node` 파라미터로 흐른다). `pixel_policy=warn`(기본)이면 여전히 클래스만으로 진행 + `ignored:["pixel"]`, `reject`면 거부. `pixel`만 보내고 `pixel_wh` 안 보내면 무조건 거부. `pixel_wh`가 지금 depth 프레임 해상도와 다르면(스케일링 추측 안 함) `select`에서 거부. VLA 쪽은 2026-08-11부터 실제로 보낸다 — `SceneObject` bbox 중심 픽셀 + 그 프레임의 `image_width/height`. 두 카메라가 같은 물리 D435i라 재투영 없이 그대로 의미가 있다(§9). 🔴 **cobot2_ws 쪽은 코드·빌드·순수함수 단위테스트까지만 됐다 — 실기 미검증.** `pixel_policy`를 `select`로 올리는 시점(파라미터/launch)은 VLA 쪽 결정 사항 |
| `base_xy` | 무시됨(`ignored`로 회신), 검증도 없음 |
| `approve` 관련 필드 | **없다.** `cmd:"approve"`는 코드 경로 자체가 없어 무조건 거부됨 — §4 참고, 이 필드 자체를 스키마에서 빼는 게 맞다 |

## 3. `/vla/pick_result` 스키마

```json
{"request_id":"a17-3","accepted":true,"result":"succeeded",
 "reason":"...","ignored":[],"stamp_ns":null,"state":"HOME"}
```

`result` ∈ `rejected | accepted | succeeded | failed | superseded`. 성공 판정은 `RELEASE`
**진입이 아니라 그 다음 `HOME` 도달**이다 — `RELEASE → ABORT`도 허용된 전이라 `RELEASE` 만
보고 성공 처리하면 뒤따르는 실패를 놓친다.

## 4. 🔴 승인은 이 브리지가 절대 손대지 않는다 — cobot2_ws가 로컬로 처리한다 (2026-08-10)

**그립 승인 UX를 다시 설계할 필요가 없다.** cobot2_ws 쪽에서 graspgenx 판단 화면을 사람이
직접 보고, **버튼 또는 음성**으로 로컬에서 승인한다(`rqt_panel`의 '승인' 버튼 +
`approve_listener_node`의 음성 명령 — 둘 다 `/pick/approve`를 호출). 이 브리지는:

- `/pick/approve`를 **호출하지 않는다** (`vla_command_node`가 `cmd:"approve"`를 코드
  레벨로 거부하므로 보내도 무의미하다)
- LLM 툴(`agent/tools.py`)에 **승인 관련 툴을 추가하지 않는다** — `ask_clarification`이
  승인 대역을 겸하게 만들 필요 없음. `RobotState.status`에 `waiting_approval`류 값을
  새로 만들지 여부는 여전히 열려 있는 질문(UX상 LLM이 "지금 사람 승인 대기 중"을 알면
  좋다는 점은 남아있음)이지만, **승인 자체를 자동화하는 경로는 만들지 않는다**는 게
  유일한 하드 제약이다.

## 5. `place` 값 대응표 (cobot2_ws 쪽, 참고용)

| 값 | 뜻 | 실기 teach 상태 |
|---|---|---|
| `basket` | 장바구니 | ✅ teach 완료 |
| `table` | 작업테이블 지정 자리 | 🔴 **UNVERIFIED** — home 관절값을 임시로 복사해 둔 자리표시자. 실기에서 안전한 자세로 다시 잡기 전까지 이 값을 실제로 쓰지 말 것 |
| `discard` | 테이블 밖 폐기 | 🔴 **UNVERIFIED** — 위와 동일 |

## 6. `class` 허용 목록 불일치 — 지금 그대로 붙이면 일부 거부된다

cobot2_ws는 `allowed_classes`로 들어온 클래스를 검사해서 밖의 이름을 거부한다(정확한 원인
메시지와 함께). 2026-08-10 기준 실측:

| | 목록 |
|---|---|
| cobot2_ws `config/objects.yaml` (`detect`) | `bottle, cup, spoon, banana, apple, orange, mouse` |
| 저쪽 `system.yaml` (`target_classes`, webcam+wrist 공통) | `apple, banana, orange, cup, bottle, wine glass, book` |

**겹침 5개**(`apple, banana, orange, cup, bottle`) — 이것만 지금 바로 통과한다.
**`wine glass`, `book`은 지금 상태로 브리지를 붙이면 즉시 거부된다.** cobot2_ws의
`yolo11n-seg.pt`가 COCO 80종을 아니까(`wine glass`/`book` 둘 다 COCO 클래스) `detect`에
추가하는 건 가능 — 필요하면 cobot2_ws 쪽에 요청할 것(`config/objects.yaml` 한 줄 추가 +
`yolo_seg_node` 재기동).

## 7. LLM 툴 스키마 — 지금 상태 (2026-08-11)

- `pick_and_place` 스키마는 `place`(`basket`/`table`/`discard`) 필수 인자를 받는다 —
  `RobotAction.place` → JSON `place`로 그대로 옮겨간다. `table`/`discard`는 §5의
  UNVERIFIED 상태라 VLA 쪽 `vla_pick_bridge_node`의 `allow_unverified_place`(기본
  `false`)가 로컬에서 막는다 — teach 끝나면 VLA 쪽에서 그 파라미터만 뒤집는다, cobot2_ws
  쪽 조치 불필요.
- `pick_and_hold`/`release` 툴은 스키마에 그대로 남아 있다 — FSM엔 매핑할 데가 없어서
  ("들고만 대기"·"현재 위치에 놓기" 개념이 cobot2_ws 쪽에 없음) 브리지가 로컬에서
  거부하지만, `enable_robot`(VLA 단독 모드, cobot2_ws 없이 도는 경로)에서는 실제로
  쓰는 기능이라 스키마에서 빼지 않기로 함 — cobot2_ws 쪽 조치 불필요.

## 8. cobot2_ws 쪽 구현 상태 (2026-08-11 갱신)

- ✅ **`select_by_point()` 구현 완료** — §9 제안(마스크 point-in 대신 base XY 최근접
  매칭, 아래 참고)을 그대로 코드로 옮겼다. `pixel`을 보내도 선정에 안 쓰이던 상태는
  해소됐다 — `pixel_policy=select`로 올리면 쓰인다(§2 표).
  - 새 파일 없음. 손댄 곳 4개: `graspgenx_perception/capture_graspgenx_scene.py`
    (`pixel_to_base()`·`select_by_point()` 순수함수 신설), `grasp_bridge_node.py`
    (`compute()`에 배선 — `segment()` 직후·워커 호출 전, class 필터와 같은 자리),
    `voice_processing/vla_command_node.py`(`pixel_policy` 값에 `select` 추가,
    `/pick/target_pixel` 발행 신설), `pick_fsm/task_manager.py`(`/pick/target_pixel`
    구독 + `_push_bridge()`가 `target_classes`와 같이 `pixel_x/y/w/h` 파라미터를 민다).
  - **채널**: `vla_command_node` → `/pick/target_pixel`(String JSON
    `{"x":,"y":,"w":,"h":}`, place와 같은 TRANSIENT_LOCAL QoS) → `task_manager`
    → (다음 PERCEIVE 진입 때 1회, `target_classes`와 같은 SetParameters 호출)
    → `grasp_bridge_node`의 `pixel_x/pixel_y/pixel_w/pixel_h` 파라미터.
    ⚠️ **place와 달리 단발성이다** — `task_manager`가 다음 PERCEIVE에 실어 보내는
    즉시 지운다. 안 그러면 클래스만 지시한 다음 pick이 이전 프레임 좌표를 재사용해
    엉뚱한 물체를 가리킨다(place_location은 반대로 "다음 값이 올 때까지 유지"가 맞는
    의미라 계속 남아 있다 — 둘의 성격이 다르다).
  - **매칭 로직**은 §9 제안과 한 가지 다르다: 마스크 point-in-polygon이 아니라
    **base XY 최근접 centroid 매칭**이다(`md/plans/2026-08-08-vla-integration.md`
    §5가 이 문서보다 먼저 있던 정본 설계라 그쪽을 따랐다) — `match_tolerance_m`
    (기본 0.06, VLA `system.yaml`과 값을 맞춤) 안에 후보가 없으면 거부, 2등과의
    거리차가 `ambiguity_margin_m`(기본 0.02) 미만이면 모호로 보고 **역시 거부**한다
    (`refuse_ambiguous_match`, 기본 true — false면 거리만으로 1등을 쓴다).
    실측 튜닝값 아님(UNVERIFIED) — 실기에서 다시 잡을 값이다.
  - 🔴 **실기 미검증.** `colcon build --packages-select graspgenx_perception
    voice_processing pick_fsm` PASS, 순수함수 단위테스트 PASS(`test_select_by_point.py`
    7건 + `test_vla_command.py` pixel_policy=select 케이스 포함 34건 전체) — PERCEIVE
    한 사이클을 실기로 관통시켜 본 적은 없다. 특히 미검증인 것: ① `pixel_to_base()`의
    5×5 median depth-hole 방어가 실제 D435i 노이즈에서 충분한지 ② `match_tolerance_m`/
    `ambiguity_margin_m` 값 자체(초안값, VLA `system.yaml`의 `match_tolerance_m`만
    맞췄고 margin은 이 ws가 임의로 정함) ③ `/pick/target_pixel`의 단발성 소비 타이밍이
    실제 LISTENING→PERCEIVE 전이 사이에서 안전하게 맞물리는지.

## 9. `select_by_point()` 설계 제안 원문 (VLA 쪽 작성, 2026-08-11 — §8이 실제 구현이다)

**전제가 하나 바뀌었다**: `pixel`을 처음 스키마에 넣을 때는 VLA 쪽 카메라와 cobot2_ws
쪽 카메라가 다른 물리 장치라고 알려져 있었다(그래서 좌표 재투영 없인 무의미해 미구현
상태로 남겨뒀을 가능성). 그런데 **둘 다 같은 물리 D435i를 보는 게 2026-08-10 확정됐고**,
VLA 쪽 `vla_perception`도 2026-08-11부터 `cv2.VideoCapture` 직접 오픈 대신 그 카메라의
ROS 이미지 토픽을 구독하도록 바뀌었다 — 즉 지금 `pixel`은 **cobot2_ws의 PERCEIVE 세그멘테이션과
같은 좌표계**에서 나온다. 재투영이 필요 없다는 뜻이라, `select_by_point()`를 지금
구현하는 게 전보다 훨씬 쉬워졌을 것으로 판단해 제안한다.

**언제 쓰나**: 같은 class의 후보가 씬에 2개 이상이고, VLA 쪽에서 사용자가
`ask_clarification`으로 "1번"/"2번"을 골랐을 때. 그 선택은 `object_id`(예:
`apple_17`)로 남는데, `object_id`는 경계를 안 넘으므로(§2) 그 개체의 픽셀 중심을
`pixel`/`pixel_wh`로 대신 보낸다 — 지금 `vla_pick_bridge_node`가 이미 그렇게 채워서
보내고 있다(§2 표, 검증됨).

**제안하는 로직** (PERCEIVE 진입 후, 후보 생성 뒤 · PLAN 진입 전 어딘가):

```
1. pixel + pixel_wh가 있으면:
   a. class로 필터링된 후보들의 세그멘테이션 마스크 중 pixel이 안에 들어가는
      후보가 있으면 그걸 선택한다 (point-in-mask -- bbox가 아니라 마스크로:
      물체끼리 bbox는 겹쳐도 마스크는 안 겹치는 경우가 많다).
   b. 정확히 들어가는 후보가 없으면(두 프로세스가 서로 다른 시점에 촬영한
      프레임이라 그 사이 물체가 살짝 움직였을 수 있음) pixel에서 가장 가까운
      마스크 중심(centroid)의 후보를 쓰되, 거리가 임계값(예: 화면 대각선의 5%)을
      넘으면 선택하지 않는다 -- 틀린 후보를 자신 있게 집는 것보다 거부하고
      VLA가 다시 확인하게 하는 편이 안전하다.
   c. pixel_wh가 지금 PERCEIVE의 실제 해상도와 다르면 거부한다(스케일링을
      추측하지 않는다 -- 지금 pixel_policy=reject가 이미 이 방향으로 설계돼 있음).
2. pixel이 없으면 기존 동작 그대로(class만으로 고름).
3. 같은 class 후보가 1개뿐이면 pixel 유무와 무관하게 그 하나를 쓴다 -- 애매함
   자체가 없으므로 매칭 로직을 안 태운다.
```

**같이 필요한 것**: `pixel_policy`에 `warn`/`reject` 외에 실제로 선정에 쓰는 값(예:
`select`)을 추가하거나, `pixel`이 있으면 자동으로 이 로직을 타게 하고 `warn`/`reject`는
"구현 전 fallback" 의미로 재정의.

**VLA 쪽이 이미 해둔 것 / 검증한 것**: `vla_pick_bridge_node`가 `SceneObject`의 bbox
중심을 `pixel`로, `SceneSnapshot.image_width/height`를 `pixel_wh`로 실어 보낸다
(`bridge/pick_bridge.py`의 `bbox_center()`). cobot2_ws의 `vla_command_node`를 실제로
띄워서 왕복 확인함(2026-08-11) — `pixel_policy=warn` 상태로 `ignored:["pixel"]`
경고가 그대로 나오는 것까지 재현됨. `select_by_point()`가 구현되면 VLA 쪽 코드 변경은
불필요하다 — 이미 필요한 값을 다 보내고 있다.

## 10. rqt 패널 버튼 ↔ VLA `cmd` 대응표 — **VLA 쪽이 명시적으로 확인할 것** (2026-08-11)

cobot2_ws 쪽 사람이 로컬 rqt 패널(`rqt --standalone pick_fsm`)에서 누르는 버튼과, 그
버튼을 대신할 수 있는 `/vla/pick_command`의 `cmd` 값을 **1:1로** 적는다. 매핑 자체는
**코드 레벨로 이미 끝났다**(빌드 PASS + 노드 단독 실측, §8과 같은 수준) — VLA 쪽이 이
표만 보고 자기 쪽 버튼/툴 스키마를 cobot2_ws 채널에 그대로 물릴 수 있다.

| rqt 패널 버튼 | `/vla/pick_command` 대응 | 실기로 대체 가능한가 |
|---|---|---|
| 시작 | `{"cmd":"start"}` | ✅ 노드 단독 실측(왕복) — 🔴 FSM+로봇 연결한 사이클은 미검증 |
| 중단(ABORT) | `{"cmd":"abort","reason":"..."}` | 〃 |
| 리셋 | `{"cmd":"reset"}` — SAFE_STOP 에서만 먹히고 성공하면 승인 없이 HOME 까지 실제로 움직인다 | 〃 |
| 타겟 선택(콤보 + 적용) | `{"cmd":"pick","class":"..."}` (+ `pixel_policy:=select`면 `pixel`/`pixel_wh`로 개체까지 지정) | ✅ class 만 — 🔴 pixel 개체지정은 PERCEIVE 까지만 관통, 실기 완주 미검증(§8) |
| **승인** | **없음 — 절대 대체 불가.** `cmd:"approve"`는 코드 경로 자체가 없어 무조건 거부(§4) | ❌ 의도된 설계. 사람이 로컬(rqt 또는 `approve_listener_node` 음성)로 직접 눌러야 한다 |
| 속도(vel/acc) 조절 | 없음 | ❌ 이 채널 스키마에 없음. `SetParameters`로 `/task_manager`를 직접 불러야 한다(rqt 전용 기능) |
| 놓을 위치 선택 | `pick_command`의 `place`(§2) | ✅ — 단 `table`/`discard`는 §5 UNVERIFIED |
| **놓기 재시도**(2026-08-11 신설, `PLACE_RETRY`) | **없음** | ❌ `/pick/retry_place`(Trigger) 서비스 전용 — 이 브리지 스키마에 없다. VLA 가 이 경로를 쓰려면 스키마에 `cmd:"retry_place"`(+선택 `place`)를 추가 요청해야 한다(cobot2_ws 쪽 미구현, §11 참고) |
| 그리퍼 파워사이클 · 안전모드(backdrive) · 즉시정지(`/safety/stop`) | 없음 | ❌ 이 채널 스키마에 없음. 하드웨어 안전 조작이라 의도적으로 로컬 전용 |

**결론**: pick 사이클의 "시작→타겟→(승인 제외)→중단→리셋"은 이미 텍스트/음성(JSON)으로
rqt 없이 완전히 대체 가능하게 짜여 있다. **승인·속도조절·그리퍼복구·안전모드·(신설)
놓기재시도는 이 채널에 없고, rqt 패널 또는 사람의 직접 서비스콜이 계속 필요하다** — 이
경계는 의도된 설계(승인=안전장치, 나머지=하드웨어 직접조작)이지 미구현이 아니다.

## 11. `/pick/retry_place` — cobot2_ws 쪽 신설, VLA 스키마엔 아직 없음 (2026-08-11)

그립까지 성공한 뒤 **놓을 위치로의 모션 계획이 실패**하는 경우(장애물·도달범위 등)가
실기에서 관측됐다. 기존엔 이 실패가 곧장 `ABORT→SAFE_STOP`으로 빠져서 물체를 문 채 정지 —
복구하려면 `/pick/reset`(→HOME→IDLE) 뒤 처음부터 다시 잡아야 했다(물체를 이미 들고 있는데
재인식부터 다시 하는 건 위험 — `_st_idle`의 desync 경고 참고).

cobot2_ws 는 `PLACE` 실패 시 `ABORT` 대신 새 상태 `PLACE_RETRY`로 보내도록 고쳤다 —
물체를 문 채(그리퍼 안 열림) 정지하고, `/pick/place_location`으로 다른 위치를 새로 골라
`/pick/retry_place`(Trigger)를 부르면 **재인식 없이** 그 위치로 다시 계획·이동을 시도한다.
상세는 `src/PACKAGES.md#pick_fsm` §1 상태 다이어그램·`rqt_panel.py`.

**VLA 쪽 조치 불필요** — 이 경로는 지금 로컬(rqt 패널)에만 있다. VLA 가 이 재시도까지
원격으로 하고 싶다면 `/vla/pick_command`에 `cmd:"retry_place"`(+선택 `place`)를 추가하는
스키마 확장이 필요한데, 이건 cobot2_ws 쪽 판단 사항이 아니라 **VLA 쪽이 필요하다고
명시적으로 요청해야** 붙인다(§10 표 참고 — 지금은 일부러 안 붙였다: 놓기 실패는 물체를
문 채인 민감한 순간이라 첫 버전은 로컬 승인 없이 원격에서 못 건드리게 막아둔 것).

## 12. `/vla/pick_status` — rqt 패널 상태 미러링 (2026-08-11 신설)

rqt 패널(`rqt --standalone pick_fsm`)이 보여주는 상태를 VLA UI 에서도 같이 띄우기 위한
**읽기 전용 상태 스트림**이다. 명령/결과와 별개의 세 번째 채널 — VLA 는 이걸 구독만 하고,
여기로는 아무것도 안 보낸다(제어는 여전히 §1 의 `/vla/pick_command` 로만).

```
cobot2_ws(vla_command_node) → /vla/pick_status (std_msgs/String, JSON) → VLA UI
```

| 필드 | 뜻 |
|---|---|
| `fsm` | `/pick/state` 원문(State enum 이름). **정상플로우 진행이 곧 이 값의 전이**다: `PERCEIVE→SCENE_PREP→PLAN→WAIT_APPROVAL→STOW→APPROACH→…→RELEASE→HOME` |
| `robot` | `/pick/robot_state_text` 원문(예: `STANDBY`/`MOVING`/`SAFE_OFF`). 표시용 이름 |
| `robot_code` | `/pick/robot_state_code`(Int8) 원본 정수 |
| `target` | `/pick/target_active` — FSM 이 **실제로 쓰는** 타겟(로컬 rqt 조작과의 desync 까지 반영). 자동이면 빈 문자열 |
| `place` | `/pick/place_location_active` — 지금 쓰는 놓을 위치 |
| `request_id` | 지금 latch 된 명령의 id — VLA 가 자기가 보낸 명령과 대조용. 대기 명령 없으면 `""` |
| `waiting_approval` | `fsm == "WAIT_APPROVAL"`. **사람 승인 대기 표시용**(§4 가 열어둔 신호) — 승인 자체는 여전히 로컬 사람 몫, 이 채널로 승인 못 한다 |
| `unsafe` | `robot_code ∈ {3,5,6,9,10}`(안전정지류). VLA 가 name 테이블 없이 바로 쓸 수 있게 미리 계산해 보낸다 |
| `stamp_ns` | 발행 시각(wall-clock ns) — **정보용** |

**QoS**: `COMMAND_QOS`(RELIABLE/VOLATILE/depth 10, result 와 동일). on-change + **1 Hz 하트비트**.
- 🔴 **staleness 판정은 `stamp_ns` 차이가 아니라 "N 초간 수신 없음"으로 하라** — 두 PC
  시계 동기화를 가정하지 않는다(TTL 을 수신 시각으로 재는 §2 와 같은 원칙). 하트비트가
  1 s 마다 오므로, 예를 들어 3 s 이상 끊기면 "cobot2_ws 응답 없음"으로 표시하면 된다.
- VOLATILE 이라 늦게 붙은 구독자는 직전 값을 못 받지만 1 s 안에 하트비트로 현재 상태를 받는다.
  (place 처럼 TRANSIENT_LOCAL 로 latch 하지 않는 이유: 핫스팟 링크에서 그 조합의 블로킹
  위험 — §2 `request_id` 항의 배경과 같다.)

**구현 상태**: cobot2_ws 쪽 publisher(`vla_command_node`) 완료 — `colcon build
--packages-select voice_processing` PASS, 단위테스트 34건 PASS. 🔴 **실기 미검증**(노드
단독으로 토픽이 나가는지까지는 미확인). **VLA 쪽 subscriber/UI 는 그쪽 세션이 구현** —
cobot2_ws 조치 불필요.
