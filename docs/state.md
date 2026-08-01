---
updated: 2026-07-31
---

# cobot2_ws — 현재 상태

> 이 파일은 **현재 상태로 덮어쓴다.** 로그로 쌓지 않는다.

## 지금 하던 일

**Claude Code ↔ Obsidian ↔ NotebookLM 학습 환경 구축.** 목표는 코딩 → 기초지식 연결,
면접 준비, 로보틱스 도메인 확장. 코드 작업이 아니라 학습 인프라 작업이다.

### NotebookLM — 업로드·1차 질의 완료 (2026-07-31)

노트북 **`cobot2_ws — 강의자료 ↔ 내 코드`**
(`cd7cc3d9-757c-4e1d-9633-8402d3514693`) — 소스 7개 업로드 완료, 질문 3개 실행 완료.
결과는 [[ws/cobot2/math/notebooklm-대조|notebooklm-대조]].

| 올린 것 | 상태 |
|---|---|
| `docs/math/coordinate-transform.md`, `docs/math/없는-것.md` | ✅ |
| `docs/협동로봇2_강의자료.pdf`, `docs/협동2_AI_강의자료.pdf` | ✅ |
| `src/pick_and_place_voice/README.md`, `src/cobot_rg2/README.md` | ✅ (README는 15개가 아니라 **3개**뿐이고, `src/usb_cam/README.md`는 upstream 문서라 뺐다) |
| `CLAUDE.md` | ✅ (실기 검증 사실이 이론 대조에 쓸모 있어 추가) |
| **교재·매뉴얼** — Modern Robotics(무료 PDF), Doosan DRL 매뉴얼, RG2 매뉴얼 | ❌ **사용자가 받아와야 함** |

**1차 질의로 확인된 것: 교재 없이는 절반이 막힌다.**
강의자료 2개에는 기구학·궤적·힘제어 이론 챕터가 아예 없다(Vision/YOLO/캘리브/픽앤플레이스 응용만).
면접 질문 10개 중 5개가 "이 저장소에 근거 없음"으로 나왔고, 그 5개가 `없는-것.md`의
"NotebookLM에 물을 것" 목록과 정확히 일치한다. → **Modern Robotics PDF만 넣으면 바로 재질의 가능.**

## 이번에 만든 것 (전역 — 모든 ws에 적용됨)

- `~/.claude/agents/cross-review.md` — 백지 상태 코드 재검토. `~/.claude/CLAUDE.md` 8절이 트리거
- `~/.claude/CLAUDE.md` **9절 세션 관리** 신설 — 작업 단위 끝나면 `/clear` 제안
  (근거: 52세션 실측, 8시간+ 19%, 최장 126.8h, `Agent` 호출은 2회뿐이라 서브에이전트 규칙은 안 넣음)
- `~/.claude/scripts/prompts_to_vault.py` + `Stop` 훅 — 모든 ws의 프롬프트를
  `~/vault/claude-log/<ws>/<날짜> <세션제목>.md`로 자동 추출. `claude-log/`는 **매 실행 전체 재생성**되니
  거기 손으로 쓴 건 날아간다
- NotebookLM MCP 등록 (`~/.claude.json`, `uv tool` 격리 설치, 인증 완료)

## 미해결

- [ ] **교재 PDF 확보** — Modern Robotics / DRL 매뉴얼 / RG2 매뉴얼
- [ ] `~/.local/lib/python3.10/site-packages/`에 pydantic v2 잔해
      (`pydantic_core` 2.46.4, `pydantic_settings` 2.14.2 — import 실패 상태).
      빌드는 정상(`generate_parameter_library_py` OK, pydantic 1.10.26).
      치우려면 `pip uninstall pydantic_settings pydantic_core` — **`pydantic` 본체는 건드리지 말 것**
- [ ] `docs/context/constraints.md` 비어 있음. 캘리브 실기 하면 채울 것
- [ ] cobot_rg2 스택 실기 미검증 (CLAUDE.md 6절)
- [ ] `voice_processing/resource/.env` 없어서 빌드 시 `--packages-skip voice_processing` 필요

## 알아둘 것

`src/pick_and_place_text/resource/T_gripper2camera.npy`는 **교육자료 제공 파일**이다.
`corecode/Calibration_Tutorial/eye2hand_calibration.py`로 직접 뽑은 게 아니다(그 스크립트는
`T_cam2base.npy`로 저장한다). 직접 캘리브하면 `docs/math/coordinate-transform.md` §4·§6③을 고칠 것.

## 관련
- [[ws/cobot2/math/coordinate-transform|축 1 — 좌표 변환]]
- [[ws/cobot2/math/없는-것|축 2 — 없는 것]]
- [[ws/cobot2/math/notebooklm-대조|NotebookLM 대조 결과 1차]]
