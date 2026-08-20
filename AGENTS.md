# AGENTS.md — cobot2_ws

> 공통 규칙(빌드 게이트·셸·금지 규칙·패키지 완성 정의·응답 계약·문서 규칙)은 `~/.codex/AGENTS.md`에 있다. 여기엔 이 ws에서만 참인 것만 적는다.
> Claude/Codex 멀티에이전트 위임·리뷰 절차는 `~/vault/ai/MULTI_AGENT_POLICY.md`(여러 ws 공유 단일 출처)를 따른다.
> **주의**: 팀 공유 랩탑에서는 계정마다 `~/.codex/AGENTS.md`가 따로 있고 새 계정엔 비어 있을 수 있다. 이 참조가 깨져 있으면 공통 규칙이 실제로 적용되지 않으니 작업 전 확인한다. (`kimkh` 계정은 채워져 있음 — 2026-08-01 확인. 깨져 있으면 파일 존재 여부부터 직접 재확인할 것.)

## 1. 현재 상태
> git 상태(브랜치·remote·push 방식·커밋 이력)는 [[ws/cobot2/state]] "계정/환경"이 단일 출처다. 여기서 값을 다시 적지 않는다.

- `.codex/agents/{cross-review,devils-advocate,ros-verifier}.md`, `.codex/commands/{debug,dump,quiz}.md`도 커밋됨(2026-08-01) — cobot1_ws에서 검증되어 전역(`~/.codex/`)으로 승격됐던 것을 프로젝트-로컬로 복사. 전역 설정 없는 다른 PC/계정에서도 `git pull`만으로 동일하게 동작.
- `src/`에 패키지 7개 존재 (2026-08-08 정리 — `md/plans/2026-08-08-ws-cleanup.md` 근거): `cobot_rg2`(로봇/그리퍼/카메라 bringup+moveit), `cumotion`(GPU 대체 planning pipeline, 옵션), `graspgenx_perception`(인식·grasp 계산, pick_fsm이 부르는 주 경로), `object_detection`(코드 없음 — graspgenx가 쓰는 YOLO 가중치 share 경로 전용), `pick_fsm`(상태머신, task_manager+robot_safety_node), `pick_fsm_msgs`(ComputeGrasp 인터페이스), `voice_processing`(`/get_keyword` 제공 노드 2개 — `get_keyword`(마이크) · `vla_command_node`(외부 VLA), 둘 다 pick_fsm의 지시 입력 층. **`COLCON_IGNORE`는 2026-08-09 해제됨** — `setup.py`가 `glob('resource/.env')`로 고쳐져 `.env` 없어도 빌드가 안 깨진다. 상세는 `src/PACKAGES.md#voice_processing`). `pick_and_place_text/pick_and_place_voice/robot_control/rokey/od_msg/usb_cam/webcam_perception`은 pick_fsm 경로에서 참조 0건 확인 후 삭제. `build/ install/ log/`는 이미 생성돼 있고 `.gitignore`에 정상적으로 제외됨.
- `.codex/settings.json` + `.codex/hooks/{guard.sh,format.sh}`는 이미 repo에 커밋되어 동작 중 (rm -rf 방지, opencv-python/numpy2/pydantic2 설치 차단, 실기 모션 명령 차단, build 산출물 커밋 차단, 저장 시 ruff 포맷).
- **commit/push 단위는 이 `cobot2_ws` repo 하나다.** 다른 ws나 홈 디렉토리 전역에 영향을 주는 git 작업은 하지 않는다.

## 2. 환경
- ROS 2 Humble / Ubuntu 22.04 / Python 3.10
- **팀 공유 랩탑**(hostname `rokey`)이며 팀원마다 OS 계정을 분리해서 쓴다 (`kimkh`, `jjh`, `rokey`, `buildfarm` 등 확인됨). `kimkh` 계정에서는 `~/cobot1_ws`가 실제로 접근 가능함(2026-08-01 확인 — 이전에 "접근 불가"로 적혀 있던 건 오기였다). 다만 다른 계정의 홈 디렉토리는 여전히 권한상 접근 불가할 수 있으니 계정이 다르면 재확인한다.
- 하드웨어: **M0609 로봇(네임스페이스 `dsr01`, IP `192.168.1.100`) + OnRobot RG2 그리퍼 + RealSense D435i 카메라 1대(eye-to-hand — 작업대 옆 고정, 팔에 붙어 있지 않다) — 2026-08-02 실기 확인 완료**(`bringup.launch.py mode:=real` 연결 후 MoveIt Plan/Execute 성공). 근거·상세는 [[ws/cobot2/context/constraints]].
  - **pick_fsm·graspgenx 가 쓰는 카메라는 이 D435i 한 대뿐이고 고정이다.** "손목/eye-in-hand RealSense" 는 존재하지 않는다 (2026-08-08 사용자 정정 — graspgenx README 가 그렇게 적고 있었다. 출처는 별도 repo `~/M0609_VLA_system`). 팔에 다는 안은 C270이고 **아직 실기 미확인·미착수**다 — D435i 와 무관하다.

## 3. cobot1_ws에서 가져올 것 / 가져오지 말 것
`kimkh` 계정에서 `~/cobot1_ws`는 접근 가능하다 (2026-08-01 확인).
- **가져온다**: `.codex/` 설정(hooks·agents·commands)처럼 하드웨어 무관한 것. cobot1_ws는 이미 이런 것들을 전역(`~/.codex/`)으로 승격시켜뒀으므로, 프로젝트별 이식이 필요하면 전역에서 복사해 오면 된다(1절 참고). cobot1_ws의 `.codex/commands/archive/*`는 사용자가 스스로 정리하며 archive/삭제한 것들이라 복원 대상이 아니다.
- `~/cobot1_ws/AGENTS.md` 3절(실기 검증 사실)은 **같은 하드웨어를 쓸 때만** 유효하다. cobot1_ws는 카메라가 OAK-D-Pro, cobot2_ws는 RealSense(추정)로 다르므로 카메라 의존 코드(`cup_detect` 등)는 그대로 복사하지 말고 재작성한다. 그리퍼(RG2/RG6)가 다르면 힘 기반 노드도 재보정 필요.
- **가져오지 않는다**: cobot1_ws의 `src/` ROS 코드를 복사해 오기 전에 네임스페이스·토픽·툴 무게 프리셋 의존성을 확인한다. 특히 힘 기반 노드는 그리퍼 자중 보정에 의존한다.

## 4. 이 ws에서만 참인 규칙 (실패에서 승격 — 상세 근거는 [[ws/cobot2/context/constraints]])
- **yaml만 고쳐도 `colcon build`를 다시 돌린다 — 단 `ament_python` 패키지에서만 참이다.** `pick_fsm`·`cumotion`·`graspgenx_perception`은 `--symlink-install`이어도 share가 `build/<pkg>/config/`를 가리켜서 src 수정이 안 넘어간다 (`.py`는 반영돼서 착각하기 쉽다). **`ament_cmake` + `install(DIRECTORY config)`인 `m0609_rg2_bringup`·`m0609_rg2_moveit`은 반대로 share가 src로의 심볼릭 링크라 즉시 반영된다** — 여기에 이 규칙을 적용하면 없는 문제를 쫓는다(2026-08-08 `ls -l` 실측으로 정정). — 2026-08-08
- **실수형 파라미터·리스트에는 예외 없이 소수점을 붙인다 (`0` ✗ → `0.0` ✓).** rcl YAML 파서는 리스트 안 int/float 혼합을 거부하고, 스칼라도 INTEGER로 읽혀 `declare_parameter`(DOUBLE)와 타입이 어긋나면 노드가 죽는다. PyYAML은 통과시키므로 파이썬 검증으로는 안 걸린다. — 2026-08-08
- **`pick_fsm.yaml`의 `home_joints_deg`/`place_joints_deg`를 `robot_control.py`의 JReady/BUCKET_POS에 "맞추지" 않는다.** 다른 게 정상이다. — 2026-08-08
- **컨테이너 안 노드를 `docker exec`로 직접 띄우지 않는다 → `scripts/graspx_container.sh`.** `docker exec`엔 `--sig-proxy`가 없어 호스트 Ctrl-C가 전달되지 않고, **재실행 1회당 인스턴스가 +1** 된다(실측 10개까지, `/yolo_seg/mask` publisher 10). — 2026-08-08
- **`ament_python` 패키지에서 `resource/`에 dangling symlink가 남으면 `colcon build`가 조용히 계속 깨진다.** `voice_processing`의 `build/voice_processing/resource/.env`가 예전엔 실재했다가 `src`에서 지워진 뒤에도 symlink-install 잔재로 남아, `setup.py`의 `glob('resource/.env')`가 그걸 주워 매 빌드 실패시켰다(2026-08-09 실측). `.py` 코드 문제가 아닌데 빌드가 안 되면 `build/<pkg>/resource/`에 깨진 심볼릭 링크부터 `ls -la`로 확인하고, 있으면 `rm -rf build/<pkg> install/<pkg>` 후 재빌드한다(src는 안 건드림). — 2026-08-09
- **하드웨어·네트워크 사실을 말하기 전에 `hostname`을 먼저 확인한다.** 이 ws는 머신 두 대에서 열린다: `rokey`(실기, i7-13620H + RTX 4060, 로봇 연결)와 `kimkh-17U70N-GA70K`(개인PC, i7-10510U, **GPU 없음**, `192.168.1.x` 도달 불가). `lspci`/`nvidia-smi`/`ping` 결과만으로 저장소 기록을 "거짓"이라 판정하지 말 것 — **머신이 다를 가능성을 먼저 배제한다.** 서브에이전트는 자기가 어느 머신에 있는지 모른 채 자신 있게 틀린 결론을 내므로 그 보고에도 같은 검사를 적용한다. — 2026-08-08

## 5. 채워야 할 항목
- [x] 하드웨어 (로봇 모델, 네임스페이스, 그리퍼, 센서) — 2026-08-02 실기 확인 완료 (2절 참고)
- [x] 패키지 지도 — 1절 참고 (2026-08-08). 각 패키지 역할·완성도까지의 상세 설명은 아직 없음
- [x] 이 ws에서 실기로 확인한 사실 — **`md/context/constraints.md`** 가 단일 출처(1000줄+ 축적됨)
- ⚠️ **문서 디렉토리는 `docs/`가 아니라 `md/`다.** `.gitignore:72`가 `docs/`를 통째로 무시한다(PDF 서고용). 공통 규칙(`~/.codex/AGENTS.md` 5절)이 말하는 `docs/state.md`·`docs/plans/`·`docs/decisions/`는 이 ws에서 각각 `md/state.md`·`md/plans/`·`md/decisions/`로 읽는다
- [ ] 검증 절차 (`scripts/verify.sh`를 쓸지 — cobot1_ws의 스크립트는 이 계정에서 접근 불가하므로 필요하면 사용자가 직접 옮겨야 함)

## 6. 작업 범위 제약
- 이 계정(`kimkh`)의 홈 디렉토리·이 워크스페이스는 자유롭게 수정 가능. 단 `/opt/ros/*` 등 이 랩탑을 공유하는 다른 계정들이 의존하는 시스템 전역 자원은 건드리지 않는다 (예: `sudo apt`로 ROS 패키지 재설치/제거, `/opt/ros` 하위 파일 수정 금지).
