# CLAUDE.md — cobot2_ws

> 공통 규칙(빌드 게이트·셸·금지 규칙·패키지 완성 정의·응답 계약·문서 규칙)은 `~/.claude/CLAUDE.md`에 있다. 여기엔 이 ws에서만 참인 것만 적는다.
> **주의**: 팀 공유 랩탑에서는 계정마다 `~/.claude/CLAUDE.md`가 따로 있고 새 계정엔 비어 있을 수 있다. 이 참조가 깨져 있으면 공통 규칙이 실제로 적용되지 않으니 작업 전 확인한다. (`kimkh` 계정은 채워져 있음 — 2026-08-01 확인. 깨져 있으면 파일 존재 여부부터 직접 재확인할 것.)

## 1. 현재 상태
> git 상태(브랜치·remote·push 방식·커밋 이력)는 [[ws/cobot2/state]] "계정/환경"이 단일 출처다. 여기서 값을 다시 적지 않는다.

- `.claude/agents/{cross-review,devils-advocate,ros-verifier}.md`, `.claude/commands/{debug,dump,quiz}.md`도 커밋됨(2026-08-01) — cobot1_ws에서 검증되어 전역(`~/.claude/`)으로 승격됐던 것을 프로젝트-로컬로 복사. 전역 설정 없는 다른 PC/계정에서도 `git pull`만으로 동일하게 동작.
- `src/`에 패키지 9개 존재: `cobot_rg2`, `object_detection`, `od_msg`, `pick_and_place_text`, `pick_and_place_voice`, `robot_control`, `rokey`, `usb_cam`, `voice_processing`. `build/ install/ log/`는 이미 생성돼 있고 `.gitignore`에 정상적으로 제외됨.
- `.claude/settings.json` + `.claude/hooks/{guard.sh,format.sh}`는 이미 repo에 커밋되어 동작 중 (rm -rf 방지, opencv-python/numpy2/pydantic2 설치 차단, 실기 모션 명령 차단, build 산출물 커밋 차단, 저장 시 ruff 포맷).
- **commit/push 단위는 이 `cobot2_ws` repo 하나다.** 다른 ws나 홈 디렉토리 전역에 영향을 주는 git 작업은 하지 않는다.

## 2. 환경
- ROS 2 Humble / Ubuntu 22.04 / Python 3.10
- **팀 공유 랩탑**(hostname `rokey`)이며 팀원마다 OS 계정을 분리해서 쓴다 (`kimkh`, `jjh`, `rokey`, `buildfarm` 등 확인됨). `kimkh` 계정에서는 `~/cobot1_ws`가 실제로 접근 가능함(2026-08-01 확인 — 이전에 "접근 불가"로 적혀 있던 건 오기였다). 다만 다른 계정의 홈 디렉토리는 여전히 권한상 접근 불가할 수 있으니 계정이 다르면 재확인한다.
- 하드웨어: **M0609 로봇(네임스페이스 `dsr01`, IP `192.168.1.100`) + OnRobot RG2 그리퍼 + RealSense D435i 카메라 — 2026-08-02 실기 확인 완료**(`bringup.launch.py mode:=real` 연결 후 MoveIt Plan/Execute 성공). 근거·상세는 [[ws/cobot2/context/constraints]]. C270은 아직 실기 미확인.

## 3. cobot1_ws에서 가져올 것 / 가져오지 말 것
`kimkh` 계정에서 `~/cobot1_ws`는 접근 가능하다 (2026-08-01 확인).
- **가져온다**: `.claude/` 설정(hooks·agents·commands)처럼 하드웨어 무관한 것. cobot1_ws는 이미 이런 것들을 전역(`~/.claude/`)으로 승격시켜뒀으므로, 프로젝트별 이식이 필요하면 전역에서 복사해 오면 된다(1절 참고). cobot1_ws의 `.claude/commands/archive/*`는 사용자가 스스로 정리하며 archive/삭제한 것들이라 복원 대상이 아니다.
- `~/cobot1_ws/CLAUDE.md` 3절(실기 검증 사실)은 **같은 하드웨어를 쓸 때만** 유효하다. cobot1_ws는 카메라가 OAK-D-Pro, cobot2_ws는 RealSense(추정)로 다르므로 카메라 의존 코드(`cup_detect` 등)는 그대로 복사하지 말고 재작성한다. 그리퍼(RG2/RG6)가 다르면 힘 기반 노드도 재보정 필요.
- **가져오지 않는다**: cobot1_ws의 `src/` ROS 코드를 복사해 오기 전에 네임스페이스·토픽·툴 무게 프리셋 의존성을 확인한다. 특히 힘 기반 노드는 그리퍼 자중 보정에 의존한다.

## 4. 이 ws에서만 참인 규칙 (실패에서 승격 — 상세 근거는 [[ws/cobot2/context/constraints]])
- **yaml만 고쳐도 `colcon build`를 다시 돌린다.** `--symlink-install`이어도 `config/*.yaml`은 심볼릭 링크가 아니라 복사본이라 소스 수정이 반영되지 않는다 (`.py`는 반영돼서 착각하기 쉽다). — 2026-08-08
- **실수형 파라미터·리스트에는 예외 없이 소수점을 붙인다 (`0` ✗ → `0.0` ✓).** rcl YAML 파서는 리스트 안 int/float 혼합을 거부하고, 스칼라도 INTEGER로 읽혀 `declare_parameter`(DOUBLE)와 타입이 어긋나면 노드가 죽는다. PyYAML은 통과시키므로 파이썬 검증으로는 안 걸린다. — 2026-08-08
- **`pick_fsm.yaml`의 `home_joints_deg`/`place_joints_deg`를 `robot_control.py`의 JReady/BUCKET_POS에 "맞추지" 않는다.** 다른 게 정상이다. — 2026-08-08
- **컨테이너 안 노드를 `docker exec`로 직접 띄우지 않는다 → `scripts/graspx_container.sh`.** `docker exec`엔 `--sig-proxy`가 없어 호스트 Ctrl-C가 전달되지 않고, 재실행마다 인스턴스가 쌓인다(실측 9개, `/yolo_seg/mask` publisher 9). — 2026-08-08

## 5. 채워야 할 항목
- [x] 하드웨어 (로봇 모델, 네임스페이스, 그리퍼, 센서) — 2026-08-02 실기 확인 완료 (2절 참고)
- [ ] 패키지 지도 — 패키지 9개는 존재하나 각각의 역할·완성도 설명은 아직 없음
- [x] 이 ws에서 실기로 확인한 사실 — **`md/context/constraints.md`** 가 단일 출처(1000줄+ 축적됨)
- [ ] 1절 패키지 목록이 낡았다 — `src/`에 `pick_fsm`, `graspgenx_perception`, `yolo_seg_grasp` 등이 늘어 있음
- ⚠️ **문서 디렉토리는 `docs/`가 아니라 `md/`다.** `.gitignore:72`가 `docs/`를 통째로 무시한다(PDF 서고용). 공통 규칙(`~/.claude/CLAUDE.md` 5절)이 말하는 `docs/state.md`·`docs/plans/`·`docs/decisions/`는 이 ws에서 각각 `md/state.md`·`md/plans/`·`md/decisions/`로 읽는다
- [ ] 검증 절차 (`scripts/verify.sh`를 쓸지 — cobot1_ws의 스크립트는 이 계정에서 접근 불가하므로 필요하면 사용자가 직접 옮겨야 함)

## 6. 작업 범위 제약
- 이 계정(`kimkh`)의 홈 디렉토리·이 워크스페이스는 자유롭게 수정 가능. 단 `/opt/ros/*` 등 이 랩탑을 공유하는 다른 계정들이 의존하는 시스템 전역 자원은 건드리지 않는다 (예: `sudo apt`로 ROS 패키지 재설치/제거, `/opt/ros` 하위 파일 수정 금지).
