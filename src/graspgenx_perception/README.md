# graspgenx_perception

(구 `yolo_seg` — GraspGenX 파이프라인에 결합하며 패키지명을 바꿨다. 노드명·토픽(`/yolo_seg/*`)은
그대로다.)

YOLO 인스턴스 세그멘테이션을 ROS 토픽에 붙인다. 컬러 이미지를 구독해 **인스턴스 라벨맵**과
**이진 마스크**를 발행한다.

원본 실험 스크립트(`yoloseg.py`)에서 두 가지를 바꿨다:

- **pyrealsense2 로 카메라를 직접 열지 않는다.** RealSense 는 한 프로세스만 잡을 수 있고
  이 워크스페이스는 `realsense2_camera` 가 이미 물고 있다(graspx 가 정렬 depth 를 쓴다).
  직접 열면 둘 중 하나가 죽으므로 컬러 **토픽을 구독**한다.
- **`show=True` 대신 overlay 토픽.** GUI 창은 컨테이너 X11 에 묶이고 헤드리스에서 죽는다.
  `publish_overlay:=true` 로 켜는 이미지 토픽으로 뺐다.

## 실행 환경 — 컨테이너 전용이다 (2026-08-07 재확인)

**이 노드는 호스트에서 돌지 않는다.** 호스트 시스템 파이썬에 `ultralytics`/`torch` 가 없기
때문이다. 넣지도 말 것 — torch 가 numpy 를 끌어올려 apt `cv_bridge` 를 깬다
(`~/.claude/CLAUDE.md` §3).

2026-08-07 이 PC를 직접 측정한 상태다. **README 이전 버전(2026-08-06)의 서술은 세 항목이
뒤집혔으므로 그대로 믿지 말 것** — 그날은 GPU도 가중치도 컨테이너도 없는 상태였다.

| 확인 | 2026-08-06 (옛 README) | **2026-08-07 실측** |
|---|---|---|
| 호스트 GPU | 없음 | **RTX 4060 Laptop** (driver 595.84, CUDA 13.2) |
| `od_kimkh` 컨테이너 | 없음 | **있음** (`object_detection_backup_20260806:latest`) |
| 그 컨테이너 GPU 패스스루 | 미설정 | **설정됨** — 컨테이너 안 `torch.cuda.is_available()` **True** |
| 컨테이너 파이썬 | — | torch 2.13.0+cu130 / ultralytics 8.4.113 / numpy 1.26.4 |
| 호스트 `~/.local` 오염 | torch·ultralytics·anyio 있었음 | **정리됨** — `pymodbus` 하나뿐. numpy 는 apt 1.21.5 |
| 가중치 `yolo11n-seg.pt` | 없음 | **있음** (`src/object_detection/resource/`, 6.2MB) |
| 로봇·카메라 | 미연결 | **연결됨** — `/camera/camera` 848×480, `dsr01` 컨트롤러 기동 중 |

`~/.local` 이 정리된 덕분에 **`pytest` 를 그냥 돌려도 된다.** 옛 README 의 `-p no:anyio`
우회는 더 이상 필요 없다(anyio 가 사라졌다).

## 빠른 실행

⚠️ **`ROS_DOMAIN_ID` 가 호스트와 컨테이너에서 같아야 한다.** 이 ws 의 규약은 **93** 이다
(`src/pick_fsm/README.md` §2 실행 절이 단일 출처). 컨테이너 이미지에는 이미 `ROS_DOMAIN_ID=93`
이 `Config.Env` 로 박혀 있으므로 **컨테이너에서는 아무것도 export 하지 않아도 맞는다.**

틀리기 쉬운 쪽은 **호스트**다. 호스트 셸은 기본이 도메인 0 이라 bringup·카메라를 `export
ROS_DOMAIN_ID=93` 없이 띄우면 컨테이너와 갈라진다. 2026-08-07 이 세션에서 실제로 그 상태였고,
증상은 "컨테이너에서 `ros2 topic list` 에 카메라 토픽이 **0개**" 였다.

```bash
docker start od_kimkh && docker exec -it od_kimkh bash
# --- 컨테이너 안 (도메인은 이미 93) ---
source /opt/ros/humble/setup.bash && source /home/kimkh/cobot2_ws/install/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/kimkh/cobot2_ws/fastdds_udp_only.xml
ros2 run graspgenx_perception yolo_seg_node --ros-args -p publish_overlay:=true -p device:=0
```

오버레이를 보려면 **호스트** 터미널에서:

```bash
export ROS_DOMAIN_ID=93
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/kimkh/cobot2_ws/fastdds_udp_only.xml
ros2 run rqt_image_view rqt_image_view
#   토픽 드롭다운에서 /yolo_seg/overlay 를 고르고 transport 를 compressed 로 둔다
```

> 도메인 93 에서 **컨테이너 → 호스트 방향 데이터가 지금 안 흐른다.** 아래 "🔴 미해결" 참고 —
> rqt 오버레이도 이 문제에 걸린다.

## 데이터가 안 올 때 — 위에서부터

노드가 떠 있는데 아무것도 안 나오면 이 순서로 본다. 노드는 5초마다
`5초간 <토픽> 를 한 장도 못 받았다` 경고를 찍으므로 **먼저 노드 로그를 본다.**

| # | 확인 | 명령 | 정상 |
|---|---|---|---|
| **1** | **도메인이 양쪽에서 같은가** | 호스트·컨테이너 각각 `echo $ROS_DOMAIN_ID` | **둘 다 93.** 다르면 **토픽이 아예 안 보인다** |
| 2 | 입력이 들어오는가 | 노드 로그에 watchdog 경고가 없는가 | 경고 없음 |
| 3 | 오버레이가 켜져 있는가 | `-p publish_overlay:=true` 를 줬는가 | 안 주면 `/yolo_seg/overlay` **토픽 자체가 없다** |
| 4 | 양쪽에 프로파일이 걸렸는가 | `echo $FASTRTPS_DEFAULT_PROFILES_FILE` | 빈 값이면 **토픽은 보이는데 데이터가 0** |
| 5 | 데이터가 오는가 | `ros2 topic hz /yolo_seg/labels` | 카메라 fps 와 같은 값 |

**1번과 4번은 증상이 다르다.** 이걸 구분하면 진단이 빨라진다 (2026-08-07 A/B 실측. 그날
호스트 스택이 도메인 0 에 떠 있었으므로 "맞음"이 0 이었다 — 규약대로면 93 이다):

| 도메인 일치 | 프로파일 | `ros2 topic list` 의 camera 토픽 | `ros2 topic hz` |
|---|---|---|---|
| 불일치 | 있음 | **0개** | — |
| 불일치 | 없음 | **0개** | — |
| **일치** | 없음 | 47개 | **데이터 안 옴** |
| **일치** | **있음** | 47개 | **14.06 Hz** ✅ |

즉 **도메인이 탐색을, 프로파일이 데이터를 결정한다.**

`ROS_DOMAIN_ID=93` 은 컨테이너 이미지의 `Config.Env` 에 박혀 있어 `docker exec` 마다 상속된다
(`.bashrc` 에는 없다). 이 값이 ws 규약과 같으므로 **컨테이너 쪽은 그대로 두고, 호스트 셸에서
`export ROS_DOMAIN_ID=93` 을 빠뜨리지 않는 것**이 맞는 운용이다.

`FASTRTPS_DEFAULT_PROFILES_FILE` 이 필요한 이유는 `fastdds_udp_only.xml` 주석에 있다 —
FastDDS 공유메모리가 컨테이너 경계를 못 넘는다.

## 🔴 미해결 — 컨테이너 → 호스트 데이터가 안 흐른다 (2026-08-07)

**`yolo_seg_node` 를 컨테이너에서 띄우면 호스트의 어떤 소비자도 `/yolo_seg/*` 를 못 받는다.**
`capture_graspgenx_scene.py`(호스트)가 라벨맵을 소비하므로 **`seg_source:=yolo` 경로가 지금
막혀 있다.** rqt 오버레이도 마찬가지다.

측정한 것 (20바이트 `std_msgs/String` 프로브까지 내려가서 확인):

| 방향 | 결과 |
|---|---|
| **호스트 → 컨테이너** | **5.000 Hz — 정상** |
| **컨테이너 → 호스트** | **0건** |

- 도메인 **0 / 77 / 93 셋 다** 같은 결과다 — 도메인 경합이 아니다.
- FastDDS 프로파일 **있으나 없으나** 같다 — SHM/UDP 선택 문제가 아니다.
- 메시지 크기 무관 — 53KB(`overlay/compressed`)도 407KB(`labels`)도 20B(String)도 전부 0건.
- **탐색은 양방향으로 된다.** 호스트에서 `ros2 topic info -v /yolo_seg/labels` 가 퍼블리셔
  GID·QoS 까지 다 보여준다. 데이터만 안 온다.
- **같은 날 오전에는 이 방향이 14.095 Hz 로 됐다** (도메인 0, 호스트 스택도 0). 그 뒤 호스트
  스택이 93 으로 재기동됐고 컨테이너도 `docker stop`/`start` 를 거쳤다. 그 사이에 무엇이
  바뀌었는지는 특정하지 못했다.

재현:

```bash
docker exec od_kimkh bash -c 'source /opt/ros/humble/setup.bash; \
  timeout 25 ros2 topic pub -r 5 /probe std_msgs/String "{data: hi}"' &
export ROS_DOMAIN_ID=93 && ros2 topic hz /probe      # 0건
```

원인 미특정이므로 **추측으로 고치지 말 것**(`~/.claude/CLAUDE.md` §7). 전용 `/debug` 세션이
필요하다. 유력 가설 순서: (1) `net=host` 컨테이너와 호스트가 participant 포트를 나눠 갖는
과정에서 컨테이너 퍼블리셔가 도달 불가한 unicast locator 를 광고, (2) `docker stop` 이 남긴
root 소유 `/dev/shm/fastrtps_*` 잔재(현재 5개), (3) 호스트 스택 재기동 스크립트가 바꾼 환경변수.

**우회**: `seg_source:=geometric` 은 호스트 안에서만 도는 경로라 이 문제와 무관하다.
지금 pick_fsm 파이프라인의 기본값이 `geometric` 이므로 **현재 파이프라인은 이 버그에 걸리지
않는다.**

## 이 PC에서 지금 테스트 가능한가

**2026-08-07 이 세션에서 직접 빌드·실행해 확인했다.** 실기 모션 명령은 하나도 실행하지 않았다
(이 노드는 카메라를 구독하고 마스크를 발행할 뿐 로봇을 움직이지 않는다).

| 하고 싶은 것 | 지금 이 PC에서 | 근거 |
|---|---|---|
| `colcon build --packages-select graspgenx_perception` | **가능** | 실행함 — **PASS** (0.87s) |
| 순수 함수 유닛테스트 | **가능** | `pytest` 그냥 실행 → **10 passed**. 우회 플래그 불필요 |
| 호스트에서 `yolo_seg_node` 실행 | **불가** | 호스트에 `ultralytics`/`torch` 없음 (`_load_model()` 에서 ImportError) |
| 컨테이너에서 GPU 추론 | **가능 — 확인함** | `od_kimkh`, `torch.cuda.is_available()` True, RTX 4060 |
| **실제 카메라 → GPU 추론 → 토픽 발행** | **가능 — 확인함** | 848×480 라이브 입력, watchdog 경고 0건, 아래 수신율 참고 |
| CPU 추론(`device:=cpu`) | **가능** | 컨테이너 내 실측 47.8 ms/frame |
| `/grasp/compute` 등 graspx 연동 | **이번에 재확인 안 함** | 이전 세션 값만 있다 — "검증 결과" 표 참고 |

### 실측 성능 (2026-08-07, 라이브 카메라 848×480)

추론 20회 median, warmup 3회 제외:

| device | median | min / max |
|---|---|---|
| `0` (RTX 4060 Laptop) | **5.5 ms** | 5.2 / 6.0 |
| `cpu` (컨테이너) | **47.8 ms** | 46.3 / 72.8 |

수신율 — 컨테이너 내부와 호스트에서 **같은 시간창에** 동시 측정:

| 측정 위치 | `labels` | `overlay/compressed` |
|---|---|---|
| 컨테이너 내부 | 14.075 Hz | 14.091 Hz |
| 호스트 | 14.095 Hz | 14.036 Hz |

같은 창에서 카메라 원본이 12.6~14.3 Hz 였다. **경계를 넘으며 잃는 프레임이 없다.**
오버레이는 약 53 KB/프레임(543 KB/s).

> 측정 함정: 이전 실행의 노드가 안 죽은 채로 새로 띄우면 퍼블리셔가 둘이 되어 `hz` 가 **두 배**로
> 나온다(28~30 Hz). 재실행 전 `pkill -f yolo_seg_node` 로 확인할 것 — 이번 세션에서 실제로 한 번
> 속았고, 그 값을 오버레이 손실로 오해할 뻔했다.

## 토픽

| 방향 | 토픽 | 타입 | 설명 |
|---|---|---|---|
| sub | `/camera/camera/color/image_raw` | `sensor_msgs/Image` (**rgb8**) | BEST_EFFORT, **depth=1** |
| pub | `/yolo_seg/labels` | `sensor_msgs/Image` (mono8) | 인스턴스 라벨맵. `obj_1`→101, `obj_2`→102 … |
| pub | `/yolo_seg/mask` | `sensor_msgs/Image` (mono8) | 전경 이진 마스크 0/255 |
| pub | `/yolo_seg/overlay/compressed` | `sensor_msgs/CompressedImage` (jpeg) | `publish_overlay:=true` 일 때. **기본** |
| pub | `/yolo_seg/overlay` | `sensor_msgs/Image` (bgr8) | `overlay_compressed:=false` 일 때만 |

입력 토픽의 **와이어 인코딩은 `rgb8` 이다**(2026-08-07 실측 — 옛 README 는 `bgr8` 로 잘못 적혀
있었다). 노드는 `imgmsg_to_cv2(desired_encoding='bgr8')` 로 받으므로 cv_bridge 가 변환해 준다.
직접 `imgmsg_to_cv2(msg)` 로 받아 쓰는 코드를 새로 짜면 **채널이 뒤집힌다.**

**오버레이가 왜 JPEG 이 기본인가**: 848×480 bgr8 = 1.16MB 다. UDP 전용 경로로는 이 크기를
15Hz 로 못 보낸다 — 컨테이너 안에서조차 3.75Hz 까지 떨어졌다(실측). 같은 화면이 JPEG q80 이면
**53KB**(22배)라 카메라 fps 그대로 무손실로 나간다. rqt 로 볼 때 raw 를 고르면 그림이 안 뜨거나
끊긴다.

라벨 규약(`LABEL_OBJ_BASE=100`, `MAX_OBJECTS=155`)은 `graspgenx_perception/capture_graspgenx_scene.py`
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
| `overlay_compressed` | `true` | `false` 면 raw Image 로 발행 (대역폭 22배) |
| `overlay_jpeg_quality` | `80` | JPEG 품질 |

숫자·문자열 파라미터는 `dynamic_typing=True` 로 선언했다. launch/CLI 값은 YAML 로 파싱돼
`device:=0` 이 STRING 선언에 INTEGER 로 들어오고 `InvalidParameterTypeException` 이 난다.

`classes` 도 같은 이유다. 빈 리스트를 그냥 넘기면 rclpy 가 타입을 `BYTE_ARRAY` 로 추론해
정수 목록을 못 넣는다. `capture_graspgenx_scene.py:111` 이 같은 우회를 쓴다.

## graspx 와 함께 띄우기

```bash
ros2 launch graspgenx_perception graspx.launch.py
```

```bash
ros2 run graspgenx_perception grasp_bridge_node
ros2 run graspgenx_perception capture_graspgenx_scene
```

> **2026-08-07 파일 위치를 통일했다.** 이전에는 소스가 워크스페이스 `scripts/` 에 있고
> `setup.py` 가 `scripts=[...]` 로 바깥 경로를 심었다 — 한 기능의 파일이 두 디렉토리에
> 흩어져 편집·grep 이 번거로웠다. 지금은 전부 패키지 안에 있다.
>
> | 이전 | 지금 |
> |---|---|
> | `graspgenx_perception/capture_graspgenx_scene.py` | `graspgenx_perception/capture_graspgenx_scene.py` |
> | `graspgenx_perception/grasp_bridge_node.py` | `graspgenx_perception/grasp_bridge_node.py` |
> | `scripts/graspgen_worker.py` | `graspgenx_perception/graspgen_worker.py` |
> | `scripts/test_capture_graspgenx_scene.py` | `test/manual_capture_scene.py` |
> | `scripts/test_grasp_bridge.py` | `test/manual_grasp_bridge.py` |
> | `scripts/test_scene_roundtrip.py` | `test/manual_scene_roundtrip.py` |
>
> **실행 파일 이름에서 `.py` 가 빠졌다** (`scripts=[...]` → `console_scripts`).
> 옛 이름으로 `ros2 run` 하면 실행 파일을 못 찾는다.
>
> 테스트 3종을 `test_*` 가 아니라 `manual_*` 로 바꾼 이유: 셋 다 pytest 함수가 없는
> **스크립트형**이라(최상위 `assert`, `sys.argv`, `rclpy.init()`, `import graspgenx`)
> `test/` 에 `test_` 이름으로 두면 `colcon test` 가 이들을 실행하다 깨진다. 이 패키지에
> 이미 있던 `test/manual_roundtrip.py` 와 같은 규칙이다.
>
> `graspgen_worker.py` 는 패키지 안에 있지만 **`console_scripts` 진입점이 아니다** —
> rclpy 가 아니라 GraspGenX venv 에서 `uv run python <경로>` 로 도는 별도 프로세스다.
> `grasp_bridge_node` 의 `worker_script` 파라미터가 형제 파일로 자동 해석한다.
>
> `--symlink-install` 로 빌드하면 `build/.../graspgenx_perception` 이 `src/` 를 가리키는
> 심볼릭 링크라 **파이썬 파일 편집은 재빌드 없이 즉시 반영된다**(2026-08-07 확인).

**두 노드는 같은 머신에서 못 돈다.** `yolo_seg_node` 는 ultralytics 때문에 **컨테이너 전용**
이고, `grasp_bridge_node` 는 GraspGenX 워커를 `uv` 로 띄우는데 **컨테이너에 uv 가 없다**.
런치 인자로 반씩 나눠 띄운다:

```bash
# 컨테이너 (od_kimkh — ROS_DOMAIN_ID=0 export 를 잊지 말 것)
ros2 launch graspgenx_perception graspx.launch.py run_bridge:=false
# 호스트
ros2 launch graspgenx_perception graspx.launch.py run_yolo:=false
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
| `out_dir` | `''` | 씬 4파일 저장 경로. 비우면 `<repo>/data/graspgenx_scene` (2026-08-07부터 항상 영구 저장, 임시 디렉토리 아님 — 아래 "라이브 경로" 절) |

## pick_fsm 과의 연결 — 지금 작동하는가

전체 사슬. **머신이 셋으로 갈린다**(컨테이너 / 호스트), 도메인은 **전부 93**이어야 한다.

```
카메라 /camera/camera/{color,aligned_depth_to_color}/image_raw   [호스트]
   │                                    │
   │ (seg_source=yolo 일 때만)           │ (seg_source=geometric — 기본)
   ▼                                    │
yolo_seg_node  [컨테이너·GPU]            │
   │ /yolo_seg/labels                   │
   ▼                                    ▼
capture_graspgenx_scene.py ◀────────────┘        [호스트]
   │ 씬 4파일
   ▼
grasp_bridge_node.py ──uv──▶ GraspGenX 워커       [호스트]
   │ /grasp/compute (Trigger) · /grasp/best (PoseStamped, base_link, GraspGenX 원시 grasp 프레임)
   │   ⚠️ tool0 목표가 아니다 — FSM 이 to_gripper_base() 로 rg2_base_link 목표로 바꾼다
   ▼
task_manager (pick_fsm) ──▶ MoveIt ──▶ 로봇
```

### ⛔ 기본값 그대로 띄우면 연결이 안 된다

`pick_fsm` 의 `grasp_source` 기본값이 **`compute_grasp`** 인데
(`config/pick_fsm.yaml:66`, `launch/pick_fsm.launch.py:43`), 그 경로가 부르는
**`/grasp/compute_grasp` (`pick_fsm_msgs/ComputeGrasp`) 서버는 이 워크스페이스 어디에도
구현이 없다.** `grasp_bridge_node.py:141` 이 만드는 건 `/grasp/compute` (`std_srvs/Trigger`)
하나뿐이다. `pick_fsm/README.md` §3 도 이 계약을 "**아직 없음** — 정본 계약"으로 적어 두었다.

→ **`grasp_source:=legacy_trigger` 를 명시해야 한다.**

```bash
ros2 launch pick_fsm pick_fsm.launch.py grasp_source:=legacy_trigger
```

`legacy_trigger` 는 폭 정보를 못 받으므로 `default_width_m`(0.06 m, UNVERIFIED)로 잡는다.

### 세 경로의 현재 상태

| 경로 | 인식 소스 | 지금 |
|---|---|---|
| `seg_source=geometric` + `grasp_source=legacy_trigger` | depth (호스트 전용) | **유일하게 도는 조합.** 이 패키지의 `yolo_seg_node` 는 **아예 안 쓰인다** |
| `seg_source=yolo` | `/yolo_seg/labels` (컨테이너) | **막혀 있다** — 컨테이너→호스트 전송 문제(위 🔴) + COCO 클래스 불일치 |
| `grasp_source=compute_grasp` | — | **서버 없음.** 기본값이라 그대로 띄우면 여기서 걸린다 |

즉 **`graspgenx_perception` 의 `yolo_seg_node` 는 현재 pick_fsm 파이프라인에 실질적으로
연결돼 있지 않다.** 기본 경로가 depth 기반 기하 세그라서 라벨맵을 아무도 구독하지 않는다.
이 패키지에서 pick_fsm 이 실제로 쓰는 것은 `setup.py` 가 심어 둔
`capture_graspgenx_scene.py` / `grasp_bridge_node.py` 두 실행 파일이다.

### 확인한 전제 (2026-08-07, 도메인 93)

| 항목 | 상태 |
|---|---|
| `color` 848×480 / `aligned_depth_to_color` 848×480 | **일치** — `segment_from_labels` 의 shape 검사 통과 조건 |
| TF `base_link → camera_color_optical_frame` | **있음** (`camera_calib_tf`, xyz `[1.237, -0.237, 0.784]`) |
| 호스트에 `uv` | **있음** (`~/.local/bin/uv`) — 브리지가 GraspGenX 워커를 이걸로 띄운다 |
| `pick_fsm` 안전 기본값 `dry_run:=true` + `require_approval:=true` | **그대로** — 실기 모션이 안 나간다 |

## 기하 세그 vs YOLO-seg

`capture_graspgenx_scene.py` 의 `seg_source` 파라미터로 고른다. 라벨 규약(101,102,…)이
같아 변환이 없다.

| | 기하 (`geometric`) | YOLO (`yolo`) |
|---|---|---|
| 입력 | **depth** | **RGB** |
| 방식 | base 프레임 작업공간 박스 + 테이블면 높이 + `connectedComponents` | 학습된 클래스의 인스턴스 마스크 |
| 속도(848×480) | 38.8 ms (CPU, 이전 세션) | **5.5 ms** (RTX 4060 Laptop) / 47.8 ms (CPU) — 2026-08-07 실측 |
| 클래스 제한 | **없음** — 박스 안에 있으면 뭐든 잡는다 | 학습한 것만 |
| 붙어 있는 물체 | **하나로 뭉친다** | 분리한다 |
| 로봇 팔 | **물체로 잡힌다** (self-filter 없음) | 클래스에 없으면 무시된다 |
| 실기 씬 10 | `obj_1`~`obj_4` 채택 | `person`/`cell phone`/`sink` — **오검출** |
| 라이브 씬 (2026-08-07) | — | `apple`/`cup`/`person` — **여전히 COCO 클래스** |
| **실기 `/grasp/compute`** | **성공** (score 0.703, 46개) | **0개 통과** (충돌 필터 전멸) |

**속도는 YOLO 가 7배 빠르지만 지금도 기하가 정답이다.** 현재 가중치가 COCO 80종이라
이 워크스페이스의 공구 5종을 모른다. 2026-08-07 라이브 카메라에 그대로 돌려도 검출 클래스가
`apple`/`cup`/`person` 이었다 — 실제 테이블 위 물체가 무엇이든 COCO 로 억지 매핑된다.
공구 seg 데이터셋으로 재학습하기 전에는 `geometric` 을 쓴다. 어느 쪽이든 병목은 GraspGenX
추론이라 세그 38.8ms 는 실질적으로 문제가 아니다.

## 가중치

`yolo11n-seg.pt` 는 `object_detection/resource/` 에 있고 **`.gitignore` 의 `*.pt` 로 커밋되지
않는다.** 이 PC에는 **지금 있다**(6.2MB, 2026-08-07 확인). 다른 PC 에서 `git pull` 만 하면
파일이 없어 노드가 뜨자마자 죽는다:

```bash
docker exec -it od_kimkh bash -lc \
  'cd /home/kimkh/cobot2_ws/src/object_detection/resource && python3 -c "from ultralytics import YOLO; YOLO(\"yolo11n-seg.pt\")"'
```

호스트에는 ultralytics 가 없으므로 **호스트에서 받는 방법은 없다.** 컨테이너에서 받아 바인드
마운트된 워크스페이스에 두는 게 유일한 경로다(`/home/kimkh/cobot2_ws` 가 컨테이너에 그대로
마운트돼 있다).

노드는 시작 시 두 번 막는다:

- 파일이 없으면 `FileNotFoundError`. ultralytics 는 basename 이 공식 에셋명이면 없는 경로를
  받아도 **조용히 네트워크에서 받아오므로**, 존재 확인을 노드가 먼저 한다.
- `model.task != 'segment'` 면 `RuntimeError`. 이 워크스페이스의 `yolov8n_tools_0122.pt` 는
  `task: detect` 라 **마스크를 못 낸다** — 여기 쓸 수 없다.

## 검증 결과

**2026-08-07 이 세션에서 재확인한 것:**

| 항목 | 상태 |
|---|---|
| `colcon build --symlink-install --packages-select graspgenx_perception` | **PASS** (0.87s) |
| `pytest src/graspgenx_perception/test/test_yolo_seg.py` (호스트, 우회 없음) | **PASS** 10개 |
| 컨테이너 GPU 가용성 (`torch.cuda.is_available()`) | **True** — RTX 4060 Laptop, torch 2.13.0+cu130 |
| 컨테이너에서 기본 가중치 자동 해석 + 로드 (`classes=80`) | **검증됨** (노드 로그) |
| **실기 카메라 → GPU 추론 → `labels`/`mask`/`overlay` 발행** | **검증됨** — watchdog 경고 0, ERROR 0 |
| 호스트 수신율 `labels` 14.095Hz / `overlay` 14.036Hz | **검증됨** — 무손실 |
| 컨테이너 수신율 `labels` 14.075Hz / `overlay` 14.091Hz | **검증됨** |
| 오버레이 대역폭 543 KB/s (≈53KB/프레임) | **검증됨** |
| 추론 속도 GPU 5.5ms / CPU 47.8ms (848×480 median×20) | **검증됨** |
| 도메인 일치/불일치 × 프로파일 유무 4조합 A/B | **검증됨** — 위 표 |
| 입력 와이어 인코딩이 `rgb8` | **검증됨** |
| 도메인 93(ws 규약)에서 컨테이너가 카메라 47토픽 수신 · 추론 정상 | **검증됨** — 컨테이너 내부 10.3Hz, watchdog 0 |
| **컨테이너 → 호스트 데이터 전송** | **🔴 실패** — 0건. 도메인 0/77/93, 프로파일 유무, 20B~407KB 전부. 위 "미해결" |
| 호스트 → 컨테이너 데이터 전송 | **정상** — 5.000Hz |
| `color`/`aligned_depth_to_color` 해상도 일치 (848×480) | **검증됨** |
| TF `base_link → camera_color_optical_frame` 존재 | **검증됨** |
| `/grasp/compute_grasp` 서버 부재 (pick_fsm 기본값이 이걸 부른다) | **확인됨** — 소스 전수 grep, 구현 없음 |

**이전 세션 값 — 이번에 재확인하지 않았다:**

| 항목 | 상태 |
|---|---|
| 컨테이너 통합 — 194×259 입력 → `labels` 194×259, `mask` `[0,255]` | 검증됨 (이전) |
| `classes:="[1,16]"` 필터 — 4개 검출이 2개(`[0,101,102]`)로 | 검증됨 (이전) |
| 입력 없을 때 5초 watchdog 경고 · SIGTERM 시 스택트레이스 없이 종료 | 검증됨 (이전) |
| `graspx.launch.py run_bridge:=false` / `run_yolo:=false` | 검증됨 (개명 전 `yolo_seg` 기준) |
| `seg_source=yolo` 경로 (`segment_from_labels`) | 검증됨 (이전) |
| 기존 graspx 테스트 2종 회귀 (`test_capture_graspgenx_scene`, `test_grasp_bridge`) | PASS (이전) |
| **`/grasp/compute` 실기 호출** (`geometric`) | 성공 — obj_1 score=0.703, 후보 46개 |
| **`/grasp/compute` 실기 호출** (`yolo`) | 실패 — 후보는 나오나 충돌 필터 0/29·0/28 통과 |

`retina_masks=True` 가 없으면 `masks.data` 가 letterbox 된 모델 해상도로 나온다
(194×259 입력 → 480×640 마스크, ultralytics 8.4.113 실측). 그 상태로 라벨맵에 인덱싱하면
`IndexError` 다 — `build_label_map()` 이 먼저 잡아 `retina_masks` 를 지목하는 메시지를 낸다.

### 🔴→✅ 라이브 경로(`/grasp/compute`)가 판단 근거를 안 남기던 문제 — 2026-08-07 수정

**증상(발견 당시)**: `capture_graspgenx_scene` 단독 실행은 `out_dir`가 비어 있으면
`<repo>/data/graspgenx_scene/<scene>/`에 영구 저장하는데, `grasp_bridge_node.compute()`는
같은 `out_dir=''` 기본값에서 `tempfile.TemporaryDirectory()`를 쓰고 워커 호출 직후 `finally`
블록에서 **즉시 지웠다.** `/grasp/compute`를 실기로 불러도 GraspGenX가 뭘 보고 판단했는지
(rgb.png/seg.png/meta_data.json)를 나중에 열어볼 방법이 없었다 — 이 워크스페이스 어디에도
`data/graspgenx_scene/`가 존재하지 않았던 이유이기도 하다(`find` 전수조사 0건).

**수정**: `grasp_bridge_node.compute()`에서 임시 디렉토리 분기를 없애고 항상
`capture_graspgenx_scene.default_out_dir()`(비었으면 이 값, 지정하면 그 경로) 아래에
영구 저장한다. `scene` 파라미터를 안 바꾸면(기본값 `00`) 호출마다 타임스탬프
(`YYYYmmdd_HHMMSS`) 하위 디렉토리를 새로 만들어 이전 호출 기록을 덮어쓰지 않는다.
고정된 씬 이름이 필요하면(재현 테스트 등) `scene` 파라미터를 명시하면 그 이름을 그대로 쓴다.

**수정 중에 딸려나온 두 번째 버그(환경 감지)**: `default_out_dir()`가 `os.path.abspath(__file__)`
로 "패키지 루트"를 계산했는데, 이 워크스페이스의 기본 빌드 방식(`--symlink-install`)에서는
`install/setup.bash`가 PYTHONPATH에 `build/graspgenx_perception/graspgenx_perception/`를
먼저 얹고, 그 안의 각 파일은 `src/`를 가리키는 **심볼릭 링크**다. `abspath`는 링크를 풀지
않으므로 계산된 경로가 `build/graspgenx_perception/data/graspgenx_scene`가 됐다 — `colcon
build`/`rm -rf build`로 지워지는 산출물 디렉토리다. `python3 -c` 로 직접 import 해
`__file__`이 `build/...`로 잡히는 것과, `realpath`로 풀면 `src/graspgenx_perception/...`가
되는 것을 확인하고 `abspath` → `realpath`로 고쳤다(같은 파일, `default_out_dir()`).

```bash
# 재확인 (2026-08-07, colcon build PASS 후):
python3 -c "from graspgenx_perception.capture_graspgenx_scene import default_out_dir; print(default_out_dir())"
# -> /home/kimkh/cobot2_ws/src/graspgenx_perception/data/graspgenx_scene  (수정 전엔 build/ 밑)
```

`.gitignore:38`의 `data/graspgenx_scene/`가 대상이라 어느 경로든 커밋되지는 않는다.
회귀 확인: `pytest test_yolo_seg.py`(10개) + `manual_grasp_bridge.py` + `pick_fsm`
`pytest`(26개) 전부 PASS(2026-08-07).

**cross-review 로 추가 발견/수정된 것 (2026-08-07)**: (1) 씬 디렉토리명을 초 단위
타임스탬프로 잡아 빠른 재시도가 충돌·덮어쓰기 할 수 있었다 — `%f`(마이크로초)를 붙여 수정.
(2) `write_scene()` 이 파일 4개 중 일부만 쓰다 실패하면 반쪽짜리 씬이 영구히 남는다 —
실패 시 `shutil.rmtree` 로 통째로 지우도록 수정. 정리 정책(오래된 씬 자동 삭제)과
`scene='00'`(기본값과 같은 문자열이라 "명시"해도 구분 불가) 은 낮은 우선순위로 보고
그대로 뒀다 — 필요해지면 추가.

### yolo 세그 — 최신 한 프레임 대신 최근 n 장 중 탐지 최선을 쓴다 (2026-08-07)

**요청 배경**: grasp 연산(GPU 워커, 수 초~수십 초)에 비하면 카메라 프레임 몇 장을 더 보는
시간은 무시할 만하다 — 탐지 정확도를 그 여유로 사도 되는지 확인하고 반영.

`SceneCapture` 가 `/yolo_seg/labels` 최신 한 장(`self.yolo_labels`)만 쓰던 것을, depth 처럼
최근 프레임을 버퍼(`self.yolo_labels_history`, 상한은 depth 와 같은 `MAX_DEPTH_BUFFER`)에
쌓아두고 `best_labels()`로 그중 물체 픽셀(라벨 > 100)이 가장 많은 프레임을 골라 쓰도록 바꿨다.
`capture_graspgenx_scene.run()`은 depth 를 모으는 `frames`(기본 10)장 동안 쌓인 라벨을,
`grasp_bridge_node.compute()`는 호출마다 지운 뒤 새로 쌓인 라벨 전부를 후보로 본다.

- ponytail: 라벨맵은 픽셀별 정수 클래스ID라 depth처럼 중앙값을 낼 수 없다(서로 다른 프레임을
  섞으면 의미 없는 값) — "픽셀 수 최대인 프레임 통째로 채택"이 가장 싼 대리 지표다.
- geometric 경로(기본값)는 원래 depth 만 쓰므로 이 변경과 무관하다.
- 회귀: `pytest test_best_labels.py`(3개, 신규) 통과 확인(2026-08-07).
- ⚠️ **미검증**: 실제 카메라로 "흔들린 프레임 하나 때문에 탐지가 비었다가 다른 프레임에서
  살아나는" 상황을 재현해서 개선을 실측하지는 않았다 — 논리상 개선이지 관측한 적은 없다.

## graspx 에 YOLO 를 쓰기 전에

배선은 끝났다(`seg_source:=yolo`). 남은 건 하나다.

1. **클래스 불일치.** `yolo11n-seg.pt` 는 COCO 80종이라 이 프로젝트의 공구 5종
   (drill/hammer/pliers/screwdriver/wrench)이 없다. 실기 캡처
   `data/graspgenx_scene/10/rgb.png` 에서는 `['person','cell phone','sink']`,
   2026-08-07 라이브 카메라에서는 `['apple','cup','person']` 로 오검출한다.
   공구 seg 데이터셋 재학습이 필요하다.

2. **컨테이너 → 호스트 전송이 막혀 있다** (위 "🔴 미해결"). 라벨맵이 호스트의
   `capture_graspgenx_scene.py` 까지 도달하지 못하므로, 클래스 문제를 풀어도 이게 먼저 풀려야
   한다.

> 옛 README 는 "`seg_source=yolo` 는 작업공간 박스를 안 본다"고 적었는데 **사실이 아니다.**
> `segment_from_labels()` 는 기하 경로와 **같은** `workspace_mask()` 박스와 반경 크롭
> (`obj_radius_m`)을 적용한다(`capture_graspgenx_scene.py:290-308`). 역할 분담은
> "YOLO 가 어느 물체인지, 기하가 닿을 수 있는 곳인지"다. 박스가 없으면 COCO 의 `dining table`
> 같은 라벨이 화면 대부분을 덮어 GraspGenX 가 죽는다 — 2026-08-06 에 67,879 px 라벨로 41.7GB
> 할당을 시도한 사고가 그래서 코드에 박스가 들어간 이유다.

## 수동 통합 확인

```bash
ros2 run graspgenx_perception yolo_seg_node --ros-args -p image_topic:=/yolo_seg_probe/image &
python3 src/graspgenx_perception/test/manual_roundtrip.py corecode/OD_Tutorial/YOLO_SIMPLE/sample2.jpg
```

프로브는 기본적으로 `/yolo_seg_probe/image` 로 쏜다. 실제 카메라 토픽에 주입하면
`realsense2_camera` 를 쓰는 다른 소비자에게도 합성 프레임이 간다.
