# corecode — 교육용 튜토리얼 코드

ROS 2 패키지가 **아니다.** `colcon build` 대상이 아니고 각 디렉토리에서 `python3 <파일>`로 직접 돌린다.
`src/` 아래의 실제 패키지(`pick_and_place_voice`, `object_detection` 등)에 들어갈 코드의 원형이자 학습 자료다.

각 파일 상단 docstring에 실행법·입출력·주의사항이 있다. 여기서는 전체 흐름과 파일 간 관계만 다룬다.

---

## 전체 파이프라인

```
[음성]  wakeup_word ──▶ STT ──▶ keyword_extraction ──▶ (도구, 목적지)
                                                            │
[비전]  yolo_train ──▶ best.pt ──▶ yolo_eval ──▶ 픽셀 좌표    │
                                                    │        │
[변환]  data_recording ──▶ handeye_calibration ──▶ T_gripper2camera.npy
                                                    │
                                                    ▼
                                          verify ──▶ 로봇 픽앤플레이스
```

세 갈래가 독립적으로 학습된 뒤 마지막에 합쳐진다. **순서대로 하나씩 돌려보고 다음으로 넘어가는 구성**이다.

---

## 1. DRL_Tutorial — 두산 로봇 기본 제어

| 파일 | 내용 |
|---|---|
| `rokey_study.ipynb` | movej / movel / movePeriodic / force control / RG2 그리퍼. 셀 단위로 실행하며 배우는 자료 |

먼저 로봇을 띄운다:
```bash
# 가상
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=virtual host:=127.0.0.1 port:=12345 model:=m0609
# 실기
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=real host:=192.168.1.100 port:=12345 model:=m0609
```
가상 모드는 DRCF 에뮬레이터(Docker)가 떠 있어야 모션 서비스가 응답한다.

> 노트북 안의 `sys.path.append('~/ros2_ws/install/common2/lib/common2/imp')`는 이 ws 경로가 아니다. 여기서 돌리려면 `~/cobot2_ws/...`로 고쳐야 한다.

---

## 2. Calibration_Tutorial — 핸드아이 캘리브레이션

**목적:** 카메라가 본 픽셀을 로봇 베이스 좌표(mm)로 바꾸는 변환행렬을 구한다. 이게 없으면 "보이는 것을 집는" 동작이 불가능하다.

| 파일 | 역할 | 실행 |
|---|---|---|
| `data_recording.py` | 체커보드 이미지 + 로봇 자세 수집 | 1단계 |
| `handeye_calibration.py` | **eye-in-hand** (카메라가 그리퍼에) → `T_gripper2camera.npy` | 2단계 |
| `eye2hand_calibration.py` | **eye-to-hand** (카메라 고정) → `T_cam2base.npy` | 2단계 (택1) |
| `verify.py` | 클릭한 물체를 실제로 집어 검증 ⚠️ 실기 이동 | 3단계 |
| `realsense.py` | RealSense ROS 토픽 구독 모듈 (라이브러리) | — |
| `onrobot.py` | RG2/RG6 Modbus TCP 드라이버 (라이브러리) | — |
| `modbus.ipynb` | 그리퍼 개폐 대화형 테스트 | — |

### 실행 순서
```bash
cd corecode/Calibration_Tutorial
python3 data_recording.py        # 자세 바꿔가며 'q'로 15~20장, Ctrl+C로 종료
python3 handeye_calibration.py   # → T_gripper2camera.npy
python3 verify.py                # ⚠️ 로봇이 움직인다. 주변 정리 + E-stop 준비
```

### 두 캘리브레이션 중 무엇을 쓰나
카메라가 **그리퍼에 달려 같이 움직이면** `handeye_calibration.py`,
**삼각대 등에 고정돼 있으면** `eye2hand_calibration.py`. 둘 다 돌릴 이유는 없다.

### 설정값
- 체커보드: 내부 코너 `(8, 6)`, 한 칸 `25mm` — 보드가 다르면 두 파일 모두 수정
- 회전 규약: ZYZ 오일러 (두산 posx)
- 단위: 전 구간 mm
- 그리퍼: `192.168.1.1:502` (툴 체인저 IP, 컨트롤러 IP와 다름)

> **알려진 함정:** `find_checkerboard_pose()` 안의 `objp` 계산이 `square_size` 대신 `25`로 하드코딩돼 있다. 칸 크기가 25mm가 아닌 보드를 쓰면 `square_size`만 고쳐서는 반영되지 않는다.

---

## 3. OD_Tutorial — 객체 인식 (YOLO)

`YOLO_SIMPLE/`로 감을 잡고 `YOLO/`로 실제 학습을 한다.

| 디렉토리 | 파일 | 내용 |
|---|---|---|
| `YOLO_SIMPLE/` | `eval.py` | 사전학습 COCO 모델로 바로 추론. **환경 확인용 첫 관문** |
| | `train.py` | 5줄짜리 최소 학습 예제 (데이터셋은 별도 준비) |
| `YOLO/` | `data_download.ipynb` | Roboflow에서 공구 데이터셋 다운로드 |
| | `yolo_train.py` | 커스텀 학습 → `runs/detect/yolo_custom/weights/best.pt` |
| | `yolo_eval.py` | 학습 결과로 추론 |
| | `custom_config.yaml` | epochs 100 / imgsz 640 / batch 16 |

```bash
cd corecode/OD_Tutorial/YOLO_SIMPLE && python3 eval.py     # 환경 확인
cd ../YOLO && python3 yolo_train.py                        # 커스텀 학습
python3 yolo_eval.py
```

> **이 호스트에는 NVIDIA GPU가 없다** (Intel CometLake-U 내장 그래픽). YOLO 학습은 CPU로 돌아가며 100 epoch는 매우 오래 걸린다. 수업 중에 확인만 할 목적이면 `custom_config.yaml`의 `epochs`를 먼저 줄인다.

> ⚠️ `data_download.ipynb`에 Roboflow API 키가 하드코딩돼 있다. 공개 저장소에 올리기 전에 환경변수로 빼야 한다.

---

## 4. VoiceProcessing — 음성 명령

| 파일 | 단계 | 내용 |
|---|---|---|
| `mic_test.py` | 0 | 마이크 파형 확인. **음성이 안 되면 여기부터** |
| `MicController.py` | — | 마이크 입력 래퍼 (라이브러리) |
| `wakeup_word.py` | 1 | "hello rokey" 웨이크업 감지 (openWakeWord + tflite) |
| `STT.py` | 2 | 음성 → 텍스트 (OpenAI Whisper API) |
| `keyword_extraction.py` | 3 | 문장 → (도구, 목적지) (gpt-4o + LangChain) |

```bash
cd corecode/VoiceProcessing
python3 mic_test.py            # 파형이 움직이는지 먼저 확인
python3 wakeup_word.py         # "hello rokey"
python3 STT.py                 # 5초 녹음 → 텍스트
python3 keyword_extraction.py  # 예시 문장으로 파싱 확인
```

### API 키
`STT.py`와 `keyword_extraction.py`는 **같은 디렉토리의 `.env`** 에서 `OPENAI_API_KEY`를 읽는다.
```
OPENAI_API_KEY=sk-...
```
`.env`는 커밋하지 않는다.

### 인식 어휘
`keyword_extraction.py`의 프롬프트에 하드코딩돼 있다: `hammer, screwdriver, wrench, pos1, pos2, pos3`.
물체를 추가하려면 **프롬프트와 YOLO 클래스 이름을 함께** 고쳐야 한다.

### 샘플레이트
마이크는 48kHz로 열리고 openWakeWord는 16kHz를 요구한다. `wakeup_word.py`가 resample로 맞춘다.
마이크를 바꿔 48kHz가 안 되면 이 변환도 같이 손봐야 한다.

---

## 사전 준비

```bash
# ROS (apt로 설치. venv 쓰지 않는다)
source /opt/ros/humble/setup.bash

# 카메라
ros2 launch realsense2_camera rs_align_depth_launch.py \
  depth_module.depth_profile:=848x480x30 rgb_camera.color_profile:=1280x720x30 \
  align_depth.enable:=true
```

`realsense2_camera`는 apt 바이너리 설치(`/opt/ros/humble/`)라 이 워크스페이스를 빌드할 필요가 없다.

---

## 검증 상태

| 항목 | 상태 |
|---|---|
| 각 스크립트 코드 읽기 | 확인함 |
| GPU 부재 (CPU 학습) | 확인함 (`lspci`) |
| `square_size` 하드코딩 | 코드에서 확인함 |
| **실제 실행 / 실기 동작** | **미검증** — 이 README는 코드를 읽고 쓴 것이며 돌려보고 쓴 것이 아니다 |

실기로 돌려서 위 내용과 다른 점을 발견하면 `docs/context/constraints.md`에 적는다.
