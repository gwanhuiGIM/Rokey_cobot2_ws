# GPU 대여 준비/작업 체크리스트

**작성일:** 2026-08-04
**성격:** 부가 문서. 본 계획은 [[ws/cobot2/M0609_perception_motion_sprint_plan]] §6-2, [[ws/cobot2/plans/2026-08-03-gpu-dependent-candidates]]에 있음. 이 문서는 "GPU를 어떻게 확보하느냐"의 실행 절차만 다룬다 — 무엇을 테스트할지는 위 두 문서가 기준.
**배경:** 팀 실물 GPU(RTX 4070)를 팀원들과 공유 중이라 자리를 못 잡을 때, 대여 GPU로 FoundationPose/GraspGenX 소프트웨어 스택(빌드·노드 구동·알고리즘 플로우)을 먼저 검증해두는 용도. 최종 시연은 여전히 로컬 RTX 4070에서 진행.

---

## 0. 이 방식으로 확인되는 것 / 안 되는 것 (매번 되새길 것)

✅ 패키지 빌드, 노드 기동, 인터페이스(토픽/메시지) 정합성, 알고리즘 플로우
❌ 정확도 DoD(실측 좌표 오차), 실기 그립/모션, self-occlusion 같은 동적 장면, 4070 실물 성능

원격 GPU와는 **실시간 통신을 하지 않는다.** rosbag 파일을 통째로 복사해 원격 머신 안에서 혼자 재생·처리하는 방식이다 (네트워크가 개입하는 지점은 파일 전송 시점뿐).

---

## 1. 대여 전 준비물

- [ ] **rosbag 파일 확보** — 기존 계획에 이미 있는 녹화 지점(Day3 P1 플래너 튜닝, Day5 통합 테스트) 재활용, 없으면 D435i로 짧게 새로 녹화
  ```bash
  ros2 bag record -o gpu_test_bag /camera/camera/depth/image_rect_raw /camera/camera/depth/camera_info /camera/camera/color/image_raw
  ```
- [ ] **정확도 검증까지 하고 싶으면** bag 촬영 시점에 물체의 실제 좌표를 줄자/CAD로 재서 따로 기록해둘 것 (안 해두면 이 세션에서 정확도 DoD는 확인 불가)
- [ ] **GraspGenX 저장소 실재 확인** — GPU 없이 지금 브라우저로 URL 확인 가능, Day0 항목 그대로
- [ ] SSH 키 페어 준비 (`ssh-keygen`), 서비스 콘솔에 퍼블릭 키 등록
- [ ] 서비스 계정 + 결제수단 등록 (사용자 직접 — 대행 불가)
- [ ] 예산 한도 인지: 세션당 대략 $2~10 예상(서비스·GPU·시간에 따라 변동, 대여 직전 콘솔에서 현재가 재확인)

---

## 2. 서비스 선택 순서 (비용 낮은 순으로 시도, 막히면 다음)

| 순서 | 서비스 | 이유 |
|---|---|---|
| 1 | **Lightning AI 무료 티어** | 신용카드 불필요, 월 15 크레딧(~80 GPU-시간, 저사양 기준). 먼저 시도 |
| 2 | **Lambda Cloud** | 진짜 VM(루트 SSH), nested docker 문제 없음. Lightning/RunPod에서 docker 막히면 이쪽 |
| 3 | **Vast.ai / RunPod** | 저렴하나 인스턴스가 컨테이너일 수 있어 nested docker 확인 필수 |

## 3. 대여 시작 직후 — 가장 먼저 할 것 (모든 서비스 공통)

```bash
nvidia-smi
docker run --rm --gpus all nvidia/cuda:12.2.0-base-ubuntu22.04 nvidia-smi
```
두 번째 명령이 실패하면 (컨테이너 안에서 docker 중첩 불가) → **Docker/Isaac ROS 워크플로우는 이 인스턴스에서 불가**, ROS 없이 순수 PyTorch(YOLO 등)만 가능하거나 서비스를 바꿔야 함.

**✅ 검증됨 (2026-08-04, Lightning AI 무료 티어, Tesla T4):** 두 명령 모두 통과. `nvidia-smi`에서 Tesla T4 15360MiB 인식, `docker run --gpus all nvidia/cuda:12.2.0-base-ubuntu22.04 nvidia-smi`도 컨테이너 안에서 정상적으로 같은 GPU를 잡음 — nested docker 문제 없음. Isaac ROS Day4 P0 블록(`run_dev.sh` 포함) 그대로 진행 가능.
**주의:** T4는 Turing 아키텍처(4070=Ada Lovelace와 세대 다름) — 빌드·노드 구동·알고리즘 플로우 검증엔 문제 없으나, TensorRT 엔진은 4070 실물에서 반드시 재빌드해야 함(§0 원칙 그대로).

---

## 4. 대여 중 작업 순서

**✅ 진행 상황 (2026-08-04):** bag 4종(`obstacle1`, `hand`, `robot_moving`, `apple`) 업로드 완료, SSH 접속 완료, `isaac_ros_common`/`isaac_ros_pose_estimation` 클론 완료. 다음은 §4-2(`run_dev.sh`)부터.

**⚠️ bag 재생 시 주의 — `ros2 bag info`로 4종 전부 실측 확인함:**
- 컬러가 **compressed로만** 녹화돼 있음(`/camera/camera/color/image_raw/compressed`), Day4 P0가 기대하는 raw `/camera/camera/color/image_raw`는 bag에 없음 → 재생 시 압축 해제 노드를 반드시 같이 띄울 것(아래 3번)
- depth(`/camera/camera/depth/image_rect_raw`)는 raw로 존재, 그대로 사용 가능
- `/camera/camera/extrinsics/depth_to_color`(`realsense2_camera_msgs/msg/Extrinsics`) 토픽 존재 — apt `ros-humble-realsense2-camera`(4.58.2, 2026-08-01 설치 확인)가 의존성으로 `realsense2_camera_msgs`도 깔아주므로 추가 조치 불필요
- 헤드리스 인스턴스라 RViz2 시각화 불가 — pose는 `ros2 topic echo`로 텍스트 확인

1. bag 파일 전송 — **완료 (2026-08-04)**
   ```bash
   scp -i <key> -r rosbag/bag_0803calibed/* <user>@<instance-ip>:~/
   ```
2. Docker: **켜야 함.** Isaac ROS는 컨테이너 안에서 빌드/실행하는 게 표준 워크플로우(`run_dev.sh`가 dev 컨테이너를 띄움) — §3에서 nested docker는 이미 검증 통과했으니 그대로 진행
   ```bash
   cd ${ISAAC_ROS_WS}/src/isaac_ros_common
   ./scripts/run_dev.sh ${ISAAC_ROS_WS}
   # 컨테이너 내부
   rosdep install -i -r --from-paths src --rosdistro humble -y
   colcon build --symlink-install --packages-up-to isaac_ros_foundationpose
   source install/setup.bash
   ```
3. 라이브 카메라 대신 재생으로 대체 (컬러 압축 해제 노드 병행 필수)
   ```bash
   ros2 bag play ~/d435i_0803_2149_apple --loop &
   ros2 run image_transport republish compressed raw \
     --ros-args -r in/compressed:=/camera/camera/color/image_raw/compressed \
                -r out:=/camera/camera/color/image_raw &
   ```
4. FoundationPose 노드 launch, 출력 pose 로그 확인/기록
   ```bash
   ros2 launch isaac_ros_foundationpose isaac_ros_foundationpose.launch.py \
     input_depth_topic:=/camera/camera/depth/image_rect_raw \
     input_rgb_topic:=/camera/camera/color/image_raw
   ```
5. GraspGenX 블록 이어서 실행 (저장소 실재 시), 실행 가능한 그립 후보 로그 확인
6. 위 2~4를 나머지 3개 bag(`obstacle1`, `hand`, `robot_moving`)에도 반복
7. **결과물 회수** — 로그, 스크린샷, 빌드 산출물 중 필요한 것만 로컬로 scp
   ```bash
   scp -i <key> <user>@<instance-ip>:~/results/* ./
   ```

---

## 5. 종료 체크리스트 (과금 방지, 필수)

- [ ] 결과물 로컬로 다 받았는지 확인 (persistent storage 안 붙였으면 종료 즉시 디스크 삭제됨)
- [ ] 인스턴스 **Terminate/Stop** — 콘솔에서 직접 확인, 켜둔 채로 방치하면 계속 과금
- [ ] 이번 세션에서 확인된 사실(빌드 성공 여부, release-3.2 호환성, GraspGenX 실제 CLI명 등)을 `M0609_perception_motion_sprint_plan.md`의 해당 Day4 블록 또는 `md/context/constraints.md`에 반영 — 다음 세션이 이 정보를 기억하지 못하므로

---
확신도: 추론(근거 있으나 미확인) — 서비스별 nested docker 지원 여부, 정확한 시간당 가격은 검색 시점(2026-08-04) 정보이며 실제로 대여해 돌려본 적 없음
내가 채워넣은 가정: (1) rosbag 파일이 GB 단위라 업로드는 최초 1회면 충분하다고 가정 (2) 세션당 예산 $2~10을 문제없다고 가정 (3) Lightning AI Studio도 RunPod처럼 컨테이너 기반일 가능성이 있다고 보수적으로 가정
확인 요청: 어느 서비스로 먼저 시도할지 정해지면(Lightning AI 무료 티어 추천) 이 문서의 §3 확인 명령 결과를 알려주면 다음 단계를 구체화하겠다
