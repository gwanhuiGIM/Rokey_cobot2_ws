<!-- meta
updated: 2026-08-06 12:00
status:  live
owns:    nvblox 워크스페이스·udev·Docker 컨테이너 셋업 (§1~§5만. §6 이후는 cumotion-bringup.md 소유)
-->

# Phase 1: Isaac ROS + nvblox 셋업 가이드 (D435i, RTX 4050/4060 노트북)

> ⏸ **보류 — GPU는 있으나(RTX 4060 Laptop 8GB) 도커 경로가 막혀 있다.**
> `kimkh`가 docker 그룹 비멤버(멤버는 `rokey`) + `nvidia-container-toolkit` 미설치 — 이 둘이 풀려야 §4 이후를 진행할 수 있다.
> 충돌 회피는 이미 Octomap이 담당 중이라 nvblox는 여전히 **시각화 전용·우선순위 낮음**이다.
> **nvblox 실행 절차 본체(§6 이후)는 이 문서가 아니라 [[ws/cobot2/plans/2026-08-05-cumotion-bringup]] §6이 단일 출처다** — 가장 최신이고 실패 이력(`std::lerp`, `warp.torch`)까지 있다.
> 이 문서는 §1~§5(워크스페이스·udev·Docker 컨테이너 셋업)만 유효하다. 문서 지도: [[ws/cobot2/README]]

목표: 로봇 없이, D435i만으로 nvblox의 실시간 3D 재구성을 눈으로 확인한다.

---

## 0. 사전 확인

```bash
nvidia-smi   # GPU 인식 확인
lsb_release -a   # Ubuntu 22.04 확인
docker --version   # Docker 설치 여부 확인
```

Docker와 nvidia-container-toolkit이 없다면 먼저 설치해야 합니다
(Isaac ROS는 Docker 기반 개발환경을 강력히 권장합니다).

```bash
# nvidia-container-toolkit 설치 (없는 경우)
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

---

## 1. 워크스페이스 및 isaac_ros_common 클론

```bash
mkdir -p ~/workspaces/isaac_ros-dev/src
cd ~/workspaces/isaac_ros-dev/src
export ISAAC_ROS_WS=~/workspaces/isaac_ros-dev

git clone -b release-4.4 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git isaac_ros_common
git clone -b release-4.4 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox.git isaac_ros_nvblox
```

## 2. RealSense 지원을 위한 컨테이너 설정

Docker 컨테이너를 빌드하기 전에, RealSense용 레이어를 포함하도록 설정합니다.

```bash
cd ${ISAAC_ROS_WS}/src/isaac_ros_common/scripts
touch .isaac_ros_common-config
echo CONFIG_IMAGE_KEY=ros2_humble.realsense > .isaac_ros_common-config
```

## 3. RealSense udev 규칙 등록 (호스트 측)

카메라를 뽑아둔 상태에서 진행합니다.

```bash
wget https://raw.githubusercontent.com/realsenseai/librealsense/v2.56.3/config/99-realsense-libusb.rules
sudo mv 99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

> 주의: 이전에 apt로 설치한 `librealsense2-dkms`와 버전(v2.56.3)이 다를 수 있습니다.
> 충돌이 의심되면 `dpkg -l | grep realsense`로 현재 버전을 먼저 확인하세요.

## 4. Docker 컨테이너 실행

```bash
cd ${ISAAC_ROS_WS}/src/isaac_ros_common
./scripts/run_dev.sh ${ISAAC_ROS_WS}
```

컨테이너 안에 들어가면 카메라를 연결하고 확인합니다.

```bash
# 컨테이너 내부에서
realsense-viewer
```

## 5. 워크스페이스 빌드

```bash
# 컨테이너 내부에서
cd ${ISAAC_ROS_WS}
colcon build --symlink-install --packages-up-to-regex realsense*
colcon build --symlink-install --packages-up-to isaac_ros_nvblox
source install/setup.bash
```

## 6. nvblox 실행 · 검증 · 다음 단계 — 여기서부터는 이 문서가 아니라 다른 문서를 본다

> 이 문서가 작성된 뒤(release-4.4 가정, rosbag 대신 라이브 카메라 가정) 실제 환경이 갈렸다:
> `release-3.2` 고정, 카메라 없이 rosbag 재생 기반 검증. 그 최신 상태·확정 명령어·지뢰 목록은
> **[[ws/cobot2/plans/2026-08-05-cumotion-bringup]] §6이 단일 출처다.**
> 검증 체크리스트·ESDF→PlanningScene 다음 단계도 그 문서 이후 절에서 다룬다.

## 참고 문서

- RealSense 셋업: https://nvidia-isaac-ros.github.io/getting_started/sensors/realsense_setup.html
- nvblox 개요: https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/isaac_ros_nvblox/index.html
- 트러블슈팅(RealSense): https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/isaac_ros_nvblox/troubleshooting/troubleshooting_nvblox_realsense.html
