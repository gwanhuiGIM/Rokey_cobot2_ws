# CLAUDE.md — cobot2_ws

> 공통 규칙(빌드 게이트·셸·금지 규칙·패키지 완성 정의·응답 계약·문서 규칙)은 `~/.claude/CLAUDE.md`에 있다. 여기엔 이 ws에서만 참인 것만 적는다.
> **주의**: 팀 공유 랩탑에서는 계정마다 `~/.claude/CLAUDE.md`가 따로 있고 새 계정엔 비어 있을 수 있다 (5절 참고). 이 참조가 깨져 있으면 공통 규칙이 실제로 적용되지 않으니 작업 전 확인한다.

## 1. 현재 상태 (2026-08-01)
- **더 이상 빈 워크스페이스가 아님**: git 초기화 완료, `origin`은 개인 fork `gwanhuiGIM/0730_cobo2_personal.git`, 현재 브랜치 `init_sett`(origin과 동기화, `main`보다 3커밋 앞섬 — 아직 main에 머지 안 됨).
- `src/`에 패키지 9개 존재: `cobot_rg2`, `object_detection`, `od_msg`, `pick_and_place_text`, `pick_and_place_voice`, `robot_control`, `rokey`, `usb_cam`, `voice_processing`. `build/ install/ log/`는 이미 생성돼 있고 `.gitignore`에 정상적으로 제외됨.
- `.claude/settings.json` + `.claude/hooks/{guard.sh,format.sh}`는 이미 repo에 커밋되어 동작 중 (rm -rf 방지, opencv-python/numpy2/pydantic2 설치 차단, 실기 모션 명령 차단, build 산출물 커밋 차단, 저장 시 ruff 포맷).
- **commit/push 단위는 이 `cobot2_ws` repo 하나다.** 다른 ws나 홈 디렉토리 전역에 영향을 주는 git 작업은 하지 않는다.

## 2. 환경
- ROS 2 Humble / Ubuntu 22.04 / Python 3.10
- **팀 공유 랩탑**(hostname `rokey`)이며 팀원마다 OS 계정을 분리해서 쓴다 (`kimkh`, `jjh`, `rokey`, `buildfarm` 등 확인됨). 다른 계정의 홈 디렉토리는 권한상 접근 불가 — **`~/cobot1_ws`는 이 계정(`kimkh`)에서 존재하지 않거나 접근할 수 없다.** 3절의 "가져올 것" 지침은 해당 파일에 실제로 접근 가능할 때만 적용된다.
- 하드웨어: `~/.bashrc`의 alias로 미루어 **M0609 로봇(네임스페이스 `dsr01`, IP `192.168.1.100`) + OnRobot RG2 그리퍼 + RealSense 카메라**로 추정됨. 단, 이는 alias 추론이지 실기 확인이 아니므로 실기 코드를 실행하기 전에는 사용자에게 재확인한다.

## 3. cobot1_ws에서 가져올 것 / 가져오지 말 것
- 이 계정에서는 `~/cobot1_ws`에 접근할 수 없으므로 아래 지침은 **cobot1_ws 파일을 직접 열람할 수 있는 상황(사용자가 내용을 붙여넣거나 접근 가능한 계정)에서만** 적용한다.
- **가져온다**: `~/cobot1_ws/CLAUDE.md` 3절(실기 검증 사실)은 **같은 하드웨어를 쓸 때만** 유효하다. 로봇/그리퍼가 다르면 그 사실들은 무효이므로 복사하지 말고 다시 실측한다.
- **가져오지 않는다**: cobot1_ws의 `src/` 코드를 복사해 오기 전에 네임스페이스·토픽·툴 무게 프리셋 의존성을 확인한다. 특히 힘 기반 노드는 그리퍼 자중 보정에 의존한다.

## 4. 채워야 할 항목
- [~] 하드웨어 (로봇 모델, 네임스페이스, 그리퍼, 센서) — alias 기반 추정만 있음 (2절 참고), 실기 확인 필요
- [ ] 패키지 지도 — 패키지 9개는 존재하나 각각의 역할·완성도 설명은 아직 없음
- [ ] 이 ws에서 실기로 확인한 사실
- [ ] 검증 절차 (`scripts/verify.sh`를 쓸지 — cobot1_ws의 스크립트는 이 계정에서 접근 불가하므로 필요하면 사용자가 직접 옮겨야 함)

## 5. 이 계정(`kimkh`)의 Claude 작업공간 설정 상태 (2026-08-01)
- git 커밋 author 정보(`user.name`/`user.email`)가 이 계정에 아직 설정되지 않음. 기존 커밋은 전부 다른 팀원(`gwanhuiGIM`) 이름으로 되어 있음 — 새 커밋을 만들기 전에 이 계정용 identity를 (repo-local 범위로) 설정해야 한다.
- push 인증 수단이 없음: SSH 키 없음, `gh` CLI 미설치, credential helper 미설정. push 전에 반드시 인증 수단을 설정한다.
- `~/.claude/CLAUDE.md`(계정 전역 공통 규칙)가 비어 있어 이 파일 상단의 참조가 현재 깨져 있다. 복원하거나 새로 작성하기 전까지 "공통 규칙"은 이 저장소의 `.claude/hooks/*`에서 유추 가능한 것 외에는 적용되지 않는다고 가정한다.
- **작업 범위**: 이 계정(`kimkh`)의 홈 디렉토리·이 워크스페이스는 자유롭게 수정 가능. 단 `/opt/ros/*` 등 이 랩탑을 공유하는 다른 계정들이 의존하는 시스템 전역 자원은 건드리지 않는다 (예: `sudo apt`로 ROS 패키지 재설치/제거, `/opt/ros` 하위 파일 수정 금지).
