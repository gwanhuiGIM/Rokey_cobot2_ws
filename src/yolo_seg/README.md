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

오버레이까지 보려면:
```bash
ros2 run yolo_seg yolo_seg_node --ros-args -p publish_overlay:=true
ros2 run rqt_image_view rqt_image_view /yolo_seg/overlay
```

## 토픽

| 방향 | 토픽 | 타입 | 설명 |
|---|---|---|---|
| sub | `/camera/camera/color/image_raw` | `sensor_msgs/Image` (bgr8) | BEST_EFFORT, **depth=1** |
| pub | `/yolo_seg/labels` | `sensor_msgs/Image` (mono8) | 인스턴스 라벨맵. `obj_1`→101, `obj_2`→102 … |
| pub | `/yolo_seg/mask` | `sensor_msgs/Image` (mono8) | 전경 이진 마스크 0/255 |
| pub | `/yolo_seg/overlay` | `sensor_msgs/Image` (bgr8) | `publish_overlay:=true` 일 때만 |

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
| 기본 가중치 자동 해석(`object_detection` share) | **검증됨** (노드 로그) |
| GPU 추론 `device=0` | **검증됨** (RTX 4060) |
| RealSense 실기 스트림 연결 | **미검증** — 합성 퍼블리셔로만 확인했다 |
| 추론 지연·처리율 | **미측정** |
| graspx `seg.png` 연결 | **미구현** — §아래 |

`retina_masks=True` 가 없으면 `masks.data` 가 letterbox 된 모델 해상도로 나온다
(194×259 입력 → 480×640 마스크, ultralytics 8.4.113 실측). 그 상태로 라벨맵에 인덱싱하면
`IndexError` 다 — `build_label_map()` 이 먼저 잡아 `retina_masks` 를 지목하는 메시지를 낸다.

## graspx 에 아직 못 붙이는 이유

1. **클래스 불일치.** `yolo11n-seg.pt` 는 COCO 80종이라 이 프로젝트의 공구 5종
   (drill/hammer/pliers/screwdriver/wrench)이 없다. 실기 캡처
   `data/graspgenx_scene/10/rgb.png` 에 돌리면 `['person','cell phone','sink']` 로 오검출한다.
   공구 seg 데이터셋 재학습이 필요하다.
2. **어댑터 부재.** `scripts/capture_graspgenx_scene.py` 의 `segment()` 는 작업공간 박스 +
   `connectedComponents` 기하 방식이고, 이 노드의 `/yolo_seg/labels` 를 받아 `seg.png` 로
   쓰는 경로가 없다. `md/detect_graspx.md` §7-8 항목.

라벨 규약을 맞춰둔 것은 그 어댑터가 생겼을 때 변환이 필요 없게 하기 위해서다.

## 수동 통합 확인

```bash
ros2 run yolo_seg yolo_seg_node --ros-args -p image_topic:=/yolo_seg_probe/image &
python3 src/yolo_seg/test/manual_roundtrip.py corecode/OD_Tutorial/YOLO_SIMPLE/sample2.jpg
```

프로브는 기본적으로 `/yolo_seg_probe/image` 로 쏜다. 실제 카메라 토픽에 주입하면
`realsense2_camera` 를 쓰는 다른 소비자에게도 합성 프레임이 간다.
