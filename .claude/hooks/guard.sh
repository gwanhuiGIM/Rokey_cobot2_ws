#!/usr/bin/env bash
# PreToolUse(Bash) 훅. stdin으로 JSON을 받고, exit 2로 도구 실행을 차단한다.
INPUT=$(cat)
CMD=$(printf '%s' "$INPUT" | python3 -c "import sys,json; print(json.load(sys.stdin).get('tool_input',{}).get('command',''))" 2>/dev/null)

block() { echo "BLOCKED by harness: $1" >&2; exit 2; }

# 1) 워크스페이스 파괴 방지
echo "$CMD" | grep -qE 'rm[[:space:]]+-[a-zA-Z]*r[a-zA-Z]*f?[[:space:]]+(/|~|\$HOME|.*src/?[[:space:]]*$)' \
  && block "src/ 또는 홈 디렉토리에 대한 재귀 삭제는 금지됩니다."

# 2) 알려진 의존성 충돌 유발 설치 차단
echo "$CMD" | grep -qE 'pip[0-9]*[[:space:]]+install.*(opencv-python|numpy[=><]*2|pydantic[[:space:]=><]*2)' \
  && block "CLAUDE.md 금지 규칙: opencv-python / numpy>=2 / pydantic v2 설치는 ROS 2 Humble을 깨뜨립니다. apt 패키지 또는 격리된 venv를 사용하세요."

# 3) 실기 모션/그리퍼 명령 차단 (사람이 직접 실행)
# ~/.claude/CLAUDE.md 0절: movej/movel/서보/그리퍼 명령은 사람 승인 없이 실행 금지.
# dsr_msgs2 motion 서비스 전체(move_j/move_l 외 movehome/movecircle/movespiral/...)와
# 그리퍼 서비스(Robotiq2F*, OnRobot RG)도 포함해야 실제로 그 원칙을 지킨다.
echo "$CMD" | grep -qiE 'ros2[[:space:]]+(service[[:space:]]+call|action[[:space:]]+send_goal|topic[[:space:]]+pub).*(move[a-z]*|servo|jog|altermotion|robotiq|onrobot|gripper|dsr_msgs2/srv/(motion|gripper)|onrobot_rg_msgs)' \
  && block "실기 모션/그리퍼 명령은 에이전트가 실행할 수 없습니다. 사람이 직접 실행하세요."

# 4) 빌드 산출물 커밋 차단
echo "$CMD" | grep -qE 'git[[:space:]]+add.*(build/|install/|log/)' \
  && block "build/ install/ log/ 는 커밋하지 않습니다."

exit 0
