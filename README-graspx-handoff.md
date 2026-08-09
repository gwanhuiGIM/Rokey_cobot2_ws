# yolo_seg + GraspGenX 성능개선 테스트 — 전달용 가이드

대상: 별도 GPU PC를 가진 팀원. **M0609 + RG2 + RealSense D435i(eye-to-hand) 하드웨어는 동일**
전제. 목표는 로봇을 움직이는 게 아니라 **인식(YOLO 세그) → grasp 계산(GraspGenX)** 두 단계의
정확도/속도를 튜닝하는 것 — `pick_fsm`(상태머신)·로봇 모션은 이 테스트 범위 밖이다.

의존성 목록은 `requirements-graspx-handoff.txt`(이 파일과 같은 위치). 여기는 절차·범위만 다룬다.

## 0. 필요한 것만 — 안 필요한 것

| 필요 | 안 필요 |
|---|---|
| `src/graspgenx_perception/`, `src/cobot_rg2/`(카메라·로봇 bringup용), `config/objects.yaml`, `scripts/graspx_container.sh` | `pick_fsm`, `voice_processing`, `cumotion`, `isaac_ros-dev/`(GraspGenX만 별도로 clone) |
| GraspGenX 저장소(아래 1-B) — **이 ws에 안 딸려있다, 직접 clone** | 로봇 실기 연결 (씬 재생/저장된 데이터로도 튜닝 가능 — 4절) |

이 워크스페이스(`cobot2_ws`) 전체를 git clone/전송해도 되고, 위 표의 "필요" 항목만 골라
복사해도 된다 — `graspgenx_perception`은 `pick_fsm_msgs`(옵션 의존, 없으면 `/grasp/compute_grasp`
서비스만 빠지고 `/grasp/compute`는 그대로 동작) 말고는 이 ws의 다른 패키지에 안 물려 있다.

## 1. 설치

### A. 호스트 (ROS 2 Humble)
```bash
sudo apt install ros-humble-desktop python3-opencv python3-numpy python3-yaml \
  ros-humble-cv-bridge ros-humble-tf2-ros \
  ros-humble-realsense2-camera ros-humble-realsense2-description
```
pip로 opencv-python/numpy>=2를 깔지 말 것 — `cv_bridge`가 깨진다. 상세는
`requirements-graspx-handoff.txt` [A].

### B. GraspGenX (grasp 계산 — GPU, uv로 호스트 파이썬과 격리)
```bash
git clone https://github.com/NVlabs/GraspGenX.git ~/GraspGenX && cd ~/GraspGenX && uv sync
```
버전 고정은 그 repo의 `pyproject.toml`/`uv.lock`이 정본이다. `graspgen_worker.py`
(`src/graspgenx_perception/graspgenx_perception/graspgen_worker.py`)를 이 venv 안에서
`uv run python graspgen_worker.py --gripper onrobot_RG2`로 실행할 수 있어야 한다 — 즉
`~/GraspGenX`가 아니어도 되지만, `grasp_bridge_node`의 `graspgenx_dir`/`worker_script`
파라미터가 이 경로를 가리키게 맞춘다.

### C. YOLO 세그 컨테이너 (GPU, ultralytics — 호스트에 절대 안 깐다)
이 랩탑은 사전 빌드된 도커 이미지(`od_kimkh`)를 쓰는데, 그 이미지 자체는 여기 없다
(로컬 빌드 산물). 새 GPU PC에서는:
1. `nvcr.io/nvidia/pytorch:25.03-py3` 같은 CUDA 베이스 이미지로 컨테이너를 하나 만들고
2. 그 안에서 `requirements-graspx-handoff.txt` [C]를 `pip install`
3. 워크스페이스를 컨테이너에 **같은 절대경로**로 바인드 마운트한다(예: `-v ~/cobot2_ws:/home/<user>/cobot2_ws`) —
   `config/objects.yaml`을 재빌드 없이 호스트/컨테이너가 같이 보게 하려면 경로가 같아야 한다
   (`scripts/graspx_container.sh` 참고).
4. 컨테이너 안에서 `colcon build --symlink-install --packages-select graspgenx_perception`
   (컨테이너 전용 install 트리를 따로 둔다).

가중치(`yolo11n-seg.pt`)는 `.gitignore`로 커밋 안 된다 — 컨테이너 안에서 `ultralytics`가
첫 실행에 자동으로 받는다(호스트엔 받을 방법이 없다, `ultralytics` 자체가 없어서).

## 2. 빌드
```bash
cd ~/cobot2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select graspgenx_perception m0609_rg2_bringup
```

## 3. 실행 — 최소 루프 (로봇 없이, 카메라 + 컨테이너 + 호스트 브리지만)
```bash
# 공통
export ROS_DOMAIN_ID=93   # 팀 규약 — 안 맞으면 노드끼리 안 보인다

# [터미널 1] 카메라만 (로봇 bringup 없이도 됨 — TF는 grasp 계산에 필요하니 아래 참고)
ros2 launch m0609_rg2_bringup camera.launch.py

# [터미널 2] 컨테이너 — YOLO 세그
scripts/graspx_container.sh run_bridge:=false device:=0 publish_overlay:=true

# [터미널 3] 호스트 — grasp 계산 (GraspGenX 워커를 uv로 자식 프로세스로 띄움)
ros2 launch graspgenx_perception graspx.launch.py run_yolo:=false run_bridge:=true

# 호출
ros2 service call /grasp/compute std_srvs/srv/Trigger {}
```
⚠️ `world→base_link→camera_link` TF가 없으면 grasp pose가 나와도 좌표가 안 맞는다 —
로봇을 안 켜도 `m0609_rg2_bringup bringup.launch.py mode:=virtual`로 TF만 띄우거나,
직접 캘리브 결과(`T_cam2base.npy`)로 static TF를 발행한다. 상세: 루트 `README.md` 2절·재캘리브 절.

무엇을 탐지/집을지는 **`config/objects.yaml` 하나가 정본**이다(`detect`: YOLO 탐지 대상,
`dimensions`: 클래스별 실측 반경/높이). 고쳤으면 컨테이너 줄([터미널 2])을 다시 실행해야 한다
(`__init__`에서 1회만 읽는다).

## 4. 성능개선 테스트가 건드릴 지점

| 레이어 | 파일/파라미터 | 뭘 튜닝하나 |
|---|---|---|
| YOLO 세그 | `yolo_seg_node` 파라미터 `conf`/`min_pixels`/`model_path` | 오탐/누락, 다른 가중치로 교체 |
| 테이블 필터링 | `capture_graspgenx_scene.py`의 `obj_min_h`/`yolo_table_ring_m`/`class_dims` | 마스크 오염(테이블면 섞임) 제거 정확도 |
| GraspGenX 추론 | GraspGenX repo `scripts/batch_inference_scene.py`, `tests/test_inference_perf.py` | latency/throughput 벤치 (torch.compile on/off 비교 등 — GraspGenX 자체 도구) |
| grasp 선택 | `grasp_bridge_node.py`의 `class_payload`/필터 순서 | grasp 후보 랭킹 품질 |

저장된 씬(`data/graspgenx_scene/<scene>/`의 rgb/depth/seg/meta 4파일)만으로도 카메라 없이
GraspGenX 쪽 반복 튜닝이 된다 — `capture_graspgenx_scene.py`가 매 `/grasp/compute` 호출마다
씬을 저장하므로, 한 번 실기로 캡처해서 팀원에게 그 폴더만 넘기는 것도 방법이다(더 가볍다).

## 5. 참고 문서 (이 문서가 다루지 않는 것)

| 필요한 것 | 문서 |
|---|---|
| 전체 파이프라인(로봇 포함) 실행법, 재캘리브 | 워크스페이스 루트 `README.md` |
| 인터페이스·파라미터 전체표, 검증 상태 | `src/PACKAGES.md#graspgenx_perception` |
| 날짜별 버그·설계 이력 | `md/graspgenx-perception-notes.md` |
| GraspGenX 자체 사용법(그리퍼 설정, 데모 스크립트) | GraspGenX repo `README.md` |

⚠️ **미검증**: 이 문서 자체는 이 랩탑(`rokey`) 기준 절차를 새 GPU PC용으로 다시 적은 것이라,
새 PC에서 실제로 이 순서대로 돌려본 적은 없다. 컨테이너 빌드(1-C)가 특히 그렇다 — 이 랩탑은
기존 이미지를 재사용할 뿐 처음부터 빌드해본 적이 없어서, 정확한 `pip install` 순서/누락
패키지는 팀원이 실행하며 채워야 할 가능성이 있다.
