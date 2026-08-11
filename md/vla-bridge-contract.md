<!-- meta
updated: 2026-08-11 (VLA 쪽 세션이 §2/§7/§9 갱신 -- place 완료, pixel 실제 전송 시작,
         select_by_point() 제안 추가. cobot2_ws 쪽 검토/구현은 아직)
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
| `pixel` + `pixel_wh` | **검증만 되고 선정에 안 쓰인다** (cobot2_ws 쪽 `select_by_point()` 미구현, §8/§9). `pixel_policy=warn`(기본)이면 클래스만으로 진행 + 결과에 `ignored:["pixel"]`, `pixel_policy=reject`면 거부. `pixel`만 보내고 `pixel_wh` 안 보내면 무조건 거부. **VLA 쪽은 2026-08-11부터 실제로 보낸다** — `SceneObject` bbox 중심 픽셀 + 그 프레임의 `image_width/height`. 두 카메라가 같은 물리 D435i라 재투영 없이 그대로 의미가 있다(§9) |
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

## 8. cobot2_ws 쪽에 아직 없는 것 (기다리지 않아도 됨)

- `select_by_point()` — `pixel` 좌표로 특정 개체를 찍어 집는 기능. 아직 미구현이라
  `pixel`을 보내도 선정에 안 쓰인다(§2 표). 되묻기("1번")가 실제로 그 개체를 집어야 하는지
  여부는 cobot2_ws 쪽 결정 사항 — 필요 없으면(씬에 같은 클래스 1개만 두는 데모로 범위를
  좁히면) 지금 그대로 접합 가능하다. 구현할 거면 §9 참고.

## 9. 🟡 제안 — `select_by_point()` 설계 (VLA 쪽 작성, 2026-08-11, cobot2_ws 검토 대기)

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
