# yolo_seg

YOLO 인스턴스 세그멘테이션을 ROS 토픽에 붙인다. 컬러 이미지를 구독해 **인스턴스 라벨맵**과
**이진 마스크**를 발행한다.

원본 실험 스크립트(`yoloseg.py`)에서 두 가지를 바꿨다:

- **pyrealsense2 로 카메라를 직접 열지 않는다.** RealSense 는 한 프로세스만 잡을 수 있고
  이 워크스페이스는 `realsense2_camera` 가 이미 물고 있다(graspx 가 정렬 depth 를 쓴다).
  직접 열면 둘 중 하나가 죽으므로 컬러 **토픽을 구독**한다.
- **`show=True` 대신 overlay 토픽.** GUI 창은 컨테이너 X11 에 묶이고 헤드리스에서 죽는다.
  `publish_overlay:=true` 로 켜는 이미지 토픽으로 뺐다.

## 실행 환경 — 도커 컨테이너 전용

`ultralytics`/`torch` 는 **호스트 시스템 파이썬에 없다.** 설치하면 numpy 가 1.21.5 →
1.26 으로 올라가 apt `cv_bridge` 를 덮는다(`~/.claude/CLAUDE.md` §3). GPU 컨테이너에서만 돈다.

```bash
docker exec -it od_kimkh bash
source /opt/ros/humble/setup.bash && source /home/kimkh/cobot2_ws/install/setup.bash \
  && export FASTRTPS_DEFAULT_PROFILES_FILE=/home/kimkh/cobot2_ws/fastdds_udp_only.xml
ros2 run yolo_seg yolo_seg_node
```

`FASTRTPS_DEFAULT_PROFILES_FILE` 은 **호스트 터미널에도** 걸어야 한다. rqt 를 띄우는
터미널도 포함이다. 없으면 토픽 탐색은 되는데 데이터가 안 흐른다(FastDDS 공유메모리가
컨테이너 경계를 못 넘는다 — `fastdds_udp_only.xml` 주석 참고).

오버레이까지 보려면 (컨테이너에서 노드, **호스트에서 rqt**):
```bash
# 컨테이너
ros2 run yolo_seg yolo_seg_node --ros-args -p publish_overlay:=true
# 호스트 — 위 export 를 이 터미널에도 건 뒤
ros2 run rqt_image_view rqt_image_view
#   토픽 드롭다운에서 /yolo_seg/overlay 를 고르고 transport 를 compressed 로 둔다
```

## 토픽

| 방향 | 토픽 | 타입 | 설명 |
|---|---|---|---|
| sub | `/camera/camera/color/image_raw` | `sensor_msgs/Image` (bgr8) | BEST_EFFORT, **depth=1** |
| pub | `/yolo_seg/labels` | `sensor_msgs/Image` (mono8) | 인스턴스 라벨맵. `obj_1`→101, `obj_2`→102 … |
| pub | `/yolo_seg/mask` | `sensor_msgs/Image` (mono8) | 전경 이진 마스크 0/255 |
| pub | `/yolo_seg/overlay/compressed` | `sensor_msgs/CompressedImage` (jpeg) | `publish_overlay:=true` 일 때. **기본** |
| pub | `/yolo_seg/overlay` | `sensor_msgs/Image` (bgr8) | `overlay_compressed:=false` 일 때만 |

**오버레이가 왜 JPEG 이 기본인가**: 848x480 bgr8 = 1.16MB 다. UDP 전용 경로로는 이 크기를
15Hz 로 못 보낸다 — 컨테이너 안에서조차 3.75Hz 까지 떨어졌다(실측). 같은 화면이 JPEG q80 이면
**44KB**(26배)라 12Hz 무손실로 나간다. rqt 로 볼 때 raw 를 고르면 그림이 안 뜨거나 끊긴다.

라벨 규약(`LABEL_OBJ_BASE=100`, `MAX_OBJECTS=155`)은 `scripts/capture_graspgenx_scene.py`
와 맞췄다. GraspGenX 로더가 `obj_` 접두어 라벨만 보기 때문이다. 상한이 갈리면 조용히
어긋나므로 테스트(`test_max_objects_matches_capture_script`)가 두 파일을 대조한다.

**겹침 처리**: 입력이 신뢰도 내림차순이라 **역순으로 칠해 고신뢰가 최상위**에 온다.
겹침으로 픽셀이 0개가 되거나 `min_pixels` 미만으로 깎인 인스턴스는 버리고 남은 것에
101부터 **연속으로** 다시 매긴다 — 라벨이 비면(`101,103,…`) 소비자의 "obj_N = N번째 물체"
가정이 깨진다.

## 파라미터

| 이름 | 기본값 | 설명 |
|---|---|---|
| `model_path` | `''` | 비우면 `object_detection` share 의 `resource/yolo11n-seg.pt` |
| `image_topic` | `/camera/camera/color/image_raw` | 구독할 컬러 토픽. depth=1 이라 추론이 느리면 묵은 프레임 대신 최신만 본다 |
| `mask_topic` / `label_topic` / `overlay_topic` | `/yolo_seg/{mask,labels,overlay}` | 발행 토픽 |
| `publish_overlay` | `false` | 오버레이 발행 여부 |
| `conf` | `0.25` | 신뢰도 임계 |
| `device` | `'0'` | `'0'`=첫 GPU, `'cpu'` |
| `classes` | `[]` | COCO 클래스 인덱스 필터. 비우면 전체. 예: `-p classes:="[1,16]"` |
| `max_objects` | `155` | 라벨맵이 uint8 이라 `100+156` 은 0 으로 랩어라운드한다. 더 크게 줘도 155로 잘린다 |
| `min_pixels` | `0` | 이보다 작은 인스턴스는 버린다(겹침으로 깎인 것 포함) |
| `overlay_compressed` | `true` | `false` 면 raw Image 로 발행 (대역폭 26배) |
| `overlay_jpeg_quality` | `80` | JPEG 품질 |

숫자·문자열 파라미터는 `dynamic_typing=True` 로 선언했다. launch/CLI 값은 YAML 로 파싱돼
`device:=0` 이 STRING 선언에 INTEGER 로 들어오고 `InvalidParameterTypeException` 이 난다.

`classes` 는 `dynamic_typing=True` 로 선언했다. 빈 리스트를 그냥 넘기면 rclpy 가 타입을
`BYTE_ARRAY` 로 추론해 정수 목록을 못 넣는다(`InvalidParameterTypeException`).
`scripts/capture_graspgenx_scene.py:111` 이 같은 이유로 같은 우회를 쓴다.

## 호스트 rqt 에 이미지가 안 보일 때

노드가 떠 있는데 아무것도 안 보이면 위에서부터 확인한다. 노드는 5초마다
`5초간 <토픽> 를 한 장도 못 받았다` 경고를 찍으므로 **먼저 노드 로그를 본다.**

| 확인 | 명령 | 정상 |
|---|---|---|
| 1. 입력이 들어오는가 | 노드 로그에 watchdog 경고가 없는가 | 경고 없음 |
| 2. 오버레이가 켜져 있는가 | `-p publish_overlay:=true` 를 줬는가 | 안 주면 `/yolo_seg/overlay` **토픽 자체가 없다** |
| 3. 호스트에 프로파일이 걸렸는가 | rqt 터미널에서 `echo $FASTRTPS_DEFAULT_PROFILES_FILE` | 빈 값이면 데이터가 안 온다 |
| 4. 호스트에 데이터가 오는가 | `ros2 topic hz /yolo_seg/overlay` | 값이 나온다. 토픽은 보이는데 hz 가 0 이면 3번 |

실측 수신율(호스트, 194×259 프로브 5Hz): `labels` 4.995Hz, `overlay` 5.002Hz — 무손실.
호스트→컨테이너 방향 640×480 bgr8(921KB) 10Hz 도 10.003Hz 무손실이다.

## graspx 와 함께 띄우기

```bash
ros2 launch yolo_seg graspx.launch.py
```

`scripts/grasp_bridge_node.py` 와 `scripts/capture_graspgenx_scene.py` 를 이 패키지의 실행
파일로 심어뒀다 — 파일은 `scripts/` 에 그대로 두고(기존 테스트가 거기를 import 한다) 경로만
`yolo_seg` 로 통일했다.

```bash
ros2 run yolo_seg grasp_bridge_node.py
ros2 run yolo_seg capture_graspgenx_scene.py
```

**두 노드는 같은 머신에서 못 돈다.** `yolo_seg_node` 는 ultralytics 때문에 **컨테이너 전용**
이고, `grasp_bridge_node` 는 GraspGenX 워커를 `uv` 로 띄우는데 **컨테이너에 uv 가 없다**.
런치 인자로 반씩 나눠 띄운다:

```bash
# 컨테이너 (od_kimkh)
ros2 launch yolo_seg graspx.launch.py run_bridge:=false
# 호스트
ros2 launch yolo_seg graspx.launch.py run_yolo:=false
ros2 service call /grasp/compute std_srvs/srv/Trigger
```

카메라와 로봇 bringup 은 이 런치에 넣지 않았다 — bringup 은 실기 모션이라 사람이 직접
실행해야 하고, 카메라는 다른 파이프라인과 공유한다.

| 런치 인자 | 기본값 | 설명 |
|---|---|---|
| `seg_source` | `geometric` | `geometric` 또는 `yolo` |
| `run_yolo` / `run_bridge` | `true` | 어느 쪽을 띄울지 |
| `image_topic` | `/camera/camera/color/image_raw` | |
| `publish_overlay` | `true` | |
| `device` / `conf` / `min_pixels` | `0` / `0.25` / `300` | |
| `out_dir` | `''` | 씬 4파일을 남길 경로. 비우면 임시 디렉토리 |

## 기하 세그 vs YOLO-seg

`capture_graspgenx_scene.py` 의 `seg_source` 파라미터로 고른다. 라벨 규약(101,102,…)이
같아 변환이 없다.

| | 기하 (`geometric`) | YOLO (`yolo`) |
|---|---|---|
| 입력 | **depth** | **RGB** |
| 방식 | base 프레임 작업공간 박스 + 테이블면 높이 + `connectedComponents` | 학습된 클래스의 인스턴스 마스크 |
| 속도(848×480) | **38.8 ms** (CPU) | **5.2 ms** (RTX 4060) / 63.6 ms (CPU) |
| 클래스 제한 | **없음** — 박스 안에 있으면 뭐든 잡는다 | 학습한 것만 |
| 붙어 있는 물체 | **하나로 뭉친다** | 분리한다 |
| 로봇 팔 | **물체로 잡힌다** (self-filter 없음) | 클래스에 없으면 무시된다 |
| 실기 씬 10 | `obj_1`~`obj_4` 채택 | `person`/`cell phone`/`sink` — **오검출** |
| **실기 `/grasp/compute`** | **성공** (score 0.703, 46개) | **0개 통과** (충돌 필터 전멸) |

**속도는 YOLO 가 7.5배 빠르지만 지금은 기하가 정답이다.** 현재 가중치가 COCO 80종이라
이 워크스페이스의 공구 5종을 모른다. 공구 seg 데이터셋으로 재학습하기 전에는 `geometric` 을
쓴다. 어느 쪽이든 병목은 GraspGenX 추론이라 세그 38.8ms 는 실질적으로 문제가 아니다.

## 가중치

`yolo11n-seg.pt` 는 `object_detection/resource/` 에 있고 **`.gitignore` 의 `*.pt` 로 커밋되지
않는다.** 다른 PC 에서 `git pull` 만 하면 파일이 없어 노드가 뜨자마자 죽는다:

```bash
docker exec -it od_kimkh bash -lc \
  'cd /home/kimkh/cobot2_ws/src/object_detection/resource && python3 -c "from ultralytics import YOLO; YOLO(\"yolo11n-seg.pt\")"'
```

노드는 시작 시 두 번 막는다:

- 파일이 없으면 `FileNotFoundError`. ultralytics 는 basename 이 공식 에셋명이면 없는 경로를
  받아도 **조용히 네트워크에서 받아오므로**, 존재 확인을 노드가 먼저 한다.
- `model.task != 'segment'` 면 `RuntimeError`. 이 워크스페이스의 `yolov8n_tools_0122.pt` 는
  `task: detect` 라 **마스크를 못 낸다** — 여기 쓸 수 없다.

## 검증 결과

| 항목 | 상태 |
|---|---|
| `colcon build --symlink-install --packages-select yolo_seg` | **PASS** |
| `pytest src/yolo_seg/test/test_yolo_seg.py` (호스트, ultralytics 없이) | **PASS** 10개 |
| 컨테이너 통합 — 194×259 입력 → `labels` 194×259, `mask` `[0,255]` | **검증됨** |
| `classes:="[1,16]"` 필터 — 4개 검출이 2개(`[0,101,102]`)로 | **검증됨** |
| 입력 없을 때 5초 watchdog 경고 · SIGTERM 시 스택트레이스 없이 종료 | **검증됨** |
| 호스트 수신율 `labels` 4.995Hz / `overlay` 5.002Hz (프로브 5Hz) | **검증됨** — 무손실 |
| **실기 카메라** 848×480 → `overlay/compressed` 11.7~12.3Hz, 44KB/프레임 | **검증됨** |
| 컨테이너가 호스트 카메라 raw 를 받는 속도 12.3Hz / compressed 15.0Hz | **검증됨** |
| `ros2 launch yolo_seg graspx.launch.py run_bridge:=false` (컨테이너) | **검증됨** |
| `... run_yolo:=false` (호스트) — bridge 기동 | **검증됨** |
| `seg_source=yolo` 경로 (`segment_from_labels`) | **검증됨** — 라벨 압축·박스·반경 크롭·실패 메시지 |
| 기존 graspx 테스트 2종 회귀 | **PASS** (`test_capture_graspgenx_scene`, `test_grasp_bridge`) |
| 기본 가중치 자동 해석(`object_detection` share) | **검증됨** (노드 로그) |
| GPU 추론 `device=0` | **검증됨** (RTX 4060) |
| **`/grasp/compute` 실기 호출** (`geometric`) | **성공** — obj_1 score=0.703, 후보 46개 |
| **`/grasp/compute` 실기 호출** (`yolo`) | **실패** — 후보는 나오나 충돌 필터 0/29·0/28 통과 |

`retina_masks=True` 가 없으면 `masks.data` 가 letterbox 된 모델 해상도로 나온다
(194×259 입력 → 480×640 마스크, ultralytics 8.4.113 실측). 그 상태로 라벨맵에 인덱싱하면
`IndexError` 다 — `build_label_map()` 이 먼저 잡아 `retina_masks` 를 지목하는 메시지를 낸다.

## graspx 에 YOLO 를 쓰기 전에

배선은 끝났다(`seg_source:=yolo`). 남은 건 하나다.

1. **클래스 불일치.** `yolo11n-seg.pt` 는 COCO 80종이라 이 프로젝트의 공구 5종
   (drill/hammer/pliers/screwdriver/wrench)이 없다. 실기 캡처
   `data/graspgenx_scene/10/rgb.png` 에 돌리면 `['person','cell phone','sink']` 로 오검출한다.
   공구 seg 데이터셋 재학습이 필요하다.
`seg_source=yolo` 는 **작업공간 박스를 안 본다** — 기하 경로가 해주던 "박스 밖은 무시"가
사라지므로, 로봇 팔이나 배경이 학습 클래스면 그대로 물체로 들어간다.

## 수동 통합 확인

```bash
ros2 run yolo_seg yolo_seg_node --ros-args -p image_topic:=/yolo_seg_probe/image &
python3 src/yolo_seg/test/manual_roundtrip.py corecode/OD_Tutorial/YOLO_SIMPLE/sample2.jpg
```

프로브는 기본적으로 `/yolo_seg_probe/image` 로 쏜다. 실제 카메라 토픽에 주입하면
`realsense2_camera` 를 쓰는 다른 소비자에게도 합성 프레임이 간다.
