#!/usr/bin/env bash
# 컨테이너(od_kimkh) 안에서 graspx 파이프라인을 띄운다. 인자는 launch 로 그대로 넘어간다.
#
#   scripts/graspx_container.sh run_bridge:=false device:=0 publish_overlay:=true classes:='[46,47,49]'
#
# 왜 스크립트인가 — `docker exec` 에는 `--sig-proxy` 가 없다 (docker 29.7.0 확인).
# `-t` 없이 띄우면 컨테이너 안에 제어 터미널이 없어서 호스트 Ctrl-C 는 호스트 쪽 docker
# 클라이언트만 죽이고, 컨테이너 안 `ros2 launch` 는 containerd-shim 밑에 고아로 남는다.
# README 가 2026-08-08 까지 "-it 없이 띄우면 Ctrl-C 로 끝낼 수 있다"고 정반대로 적어 뒀고,
# 그 결과 yolo_seg_node 가 9개까지 쌓여 /yolo_seg/mask 에 publisher 가 9개 붙었다(실측).
# ROS 2 는 같은 이름 노드 중복을 막지 않으므로 소비자는 어느 인스턴스의 마스크인지 모른다.
#
# 여기서 세 겹으로 막는다:
#   1) 띄우기 전에 컨테이너 안 잔존 인스턴스를 정리한다 (재시작 의미론)
#   2) `-t` 로 pty 를 붙여 Ctrl-C 가 컨테이너 안까지 전달되게 한다
#   3) trap 으로 이 스크립트가 어떻게 끝나든 컨테이너 쪽을 다시 정리한다
#      — 2)가 안 먹는 환경에서도 3)이 받아낸다
# 마지막 방어선은 yolo_seg_node 자신의 flock 이다 (yolo_seg_node.py:acquire_singleton).
set -uo pipefail

CONTAINER=${GRASPX_CONTAINER:-od_kimkh}
WS=${GRASPX_WS:-/home/kimkh/cobot2_ws}
# 이 컨테이너에서 도는 graspgenx_perception 프로세스 전체가 정리 대상이다.
# 호스트에서 도는 grasp_bridge_node 는 영향을 받지 않는다 (docker exec 범위 밖).
PATTERN='graspgenx_perception'

cleanup() { docker exec "$CONTAINER" pkill -f "$PATTERN" >/dev/null 2>&1 || true; }

docker start "$CONTAINER" >/dev/null || exit 1
cleanup
trap cleanup EXIT INT TERM

# 호출자에 tty 가 없으면(스크립트·CI) `-t` 는 "input device is not a TTY" 로 실패한다.
TTY_FLAGS=(-i)
[ -t 0 ] && TTY_FLAGS=(-i -t)

# printf %q 로 감싼다. classes:='[46,47]' 처럼 원격 bash -lc 문자열 안에서 다시 파싱되는
# 인자가 있어서, 따옴표 없이 넘기면 대괄호가 글롭으로 먹힌다.
ARGS=$(printf '%q ' "$@")

docker exec "${TTY_FLAGS[@]}" \
  -e FASTRTPS_DEFAULT_PROFILES_FILE="$WS/fastdds_udp_only.xml" \
  "$CONTAINER" bash -lc "
    source /opt/ros/humble/setup.bash
    source $WS/install/setup.bash
    exec ros2 launch graspgenx_perception graspx.launch.py $ARGS"
