# Phase 1: Isaac ROS + nvblox 셋업 가이드 (D435i, RTX 4050/4060 노트북)

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

## 6. nvblox 실시간 실행 (D435i 라이브 데이터)

```bash
ros2 launch nvblox_examples_bringup realsense_example.launch.py
```

RViz가 뜨면서 3D 메시가 실시간으로 쌓이는 걸 확인할 수 있습니다.
카메라를 들고 방 안을 천천히 훑으면 재구성되는 과정이 보입니다.

### (선택) 사람 인식 모드로 실행

```bash
ros2 launch nvblox_examples_bringup realsense_example.launch.py \
  mode:=people_segmentation \
  people_segmentation:=peoplesemsegnet_shuffleseg
```

---

## 검증 체크리스트

- [ ] RViz에서 3D 메시가 실시간으로 갱신되는가
- [ ] `ros2 topic hz /nvblox_node/esdf_slice` (또는 유사 토픽)로 갱신 주기 확인
- [ ] 이전 `vision_latency_bench.py`로 측정했던 지연시간과 비교
- [ ] 손이나 물체를 카메라 앞에 넣었을 때 재구성에 실시간으로 반영되는지 확인

---

## 다음 단계 (Phase 2 예고)

여기서 나온 ESDF(Euclidean Signed Distance Field) 출력을 MoveIt의
플래닝 씬 충돌 객체로 변환하는 커스텀 노드를 작성하는 게 다음 단계입니다.
nvblox 토픽 구조(특히 ESDF slice, mesh 메시지 타입)를 이 단계에서
확인해두면 Phase 2 설계가 수월해집니다.

## 참고 문서

- RealSense 셋업: https://nvidia-isaac-ros.github.io/getting_started/sensors/realsense_setup.html
- nvblox 개요: https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/isaac_ros_nvblox/index.html
- 트러블슈팅(RealSense): https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/isaac_ros_nvblox/troubleshooting/troubleshooting_nvblox_realsense.html
