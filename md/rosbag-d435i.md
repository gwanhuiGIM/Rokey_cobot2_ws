# D435i rosbag 녹화 — 무엇을 왜 담는가 (2026-08-03)

**녹화·재생 명령어의 유일한 출처는 이 문서다.** `state.md`·`Personal_0801`에는 두지 않는다.

2026-08-03에 못 쓰는 bag 4.8GB가 나왔다(§4). **원인은 사본이 갈라진 게 아니라, 세 문서가
입을 모아 틀린 런치(`rs_align_depth_launch.py`)를 지시하고 있었던 것이다** — 그 런치엔
`camera_calib_tf`가 없다는 사실을 아무도 몰랐기 때문이다. 사본이 여러 개면 이런 오류를
한 번에 고칠 수 없다. 그래서 출처를 하나로 모은다.

- 재녹화하러 왔다 → **[§A 재녹화 절차](#a-재녹화-절차)**
- 찍어둔 bag을 돌리러 왔다 → **§5 재생 절차**
- 왜 이 토픽 조합인지 / bag으로 뭘 개발하는지 → §1, §3

---

## A. 재녹화 절차

### A-0. 시작 전 — 카메라를 옮겼는가?

이게 순서를 가른다.

- **안 옮겼다** → 현재 `config/T_cam2base.npy`가 유효할 **가능성이 높다**. 그대로 §A-1로.
  bag의 `/tf_static`에 `base_link→camera_link`가 박히므로 재생 때 보충이 필요 없다.
  (2026-08-03 확인: `driver:=false`가 내는 **평행이동** (1.148174, 0.640096, 0.677658)이
  `d435i_0803_1640` bag의 값과 일치 → npy가 그때 이후 안 바뀌었다.
  ⚠️ **회전은 비교하지 않았다** — 덤프 스크립트가 translation만 읽었다. 캘리브 오차는
  회전에서 더 크게 나므로, 이 확인은 "npy 파일이 안 바뀌었다"는 것이지
  "캘리브가 정확하다"는 뜻이 아니다.)
- **옮겼다/옮길 거다** → npy가 거짓이 된다. 그 상태로 녹화하면 **bag에 가짜 캘리브가 박혀**
  §4의 `rosbag_modified`보다 더 나쁘다(없는 것보다 틀린 게 나쁘다). **재캘리브를 먼저** 하고
  npy를 갈아끼운 뒤 녹화한다.

> `state.md`의 "rosbag → 캘리브 순서" 규칙은 **npy가 없던 시절**의 것이다. 지금은 유효한 npy가
> 있으므로, 카메라를 안 건드리는 한 그 순서 문제가 발생하지 않는다.

카메라 고정 상태를 마스킹테이프로 표시하고 사진을 찍어둔다(재현용).
`realsense-viewer`가 떠 있으면 **닫는다** — USB를 독점해 ROS 노드를 죽이는데,
증상이 "TF 프레임 없음"으로 나와 오진을 유발한다.

**세 터미널의 `ROS_DOMAIN_ID`가 같아야 한다.** `.bashrc`의 `rdm` alias는 93으로 바꾸고
런치들은 도메인을 지정하지 않아 0에서 뜬다(`md/context/constraints.md`). 하나만 `rdm`을
치면 **`ros2 bag record`가 토픽을 하나도 못 보고 빈 bag을 만든다 — 에러 없이.**
녹화 터미널에서 `echo $ROS_DOMAIN_ID`로 확인한다.

### A-1. 터미널 1 — 로봇 bringup (`/tf`용)

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash && \
  ros2 launch m0609_rg2_bringup bringup.launch.py mode:=real host:=192.168.1.100
```
⚠️ 미검증 (이 세션에서 실행 안 함 — 출처는 `camera.launch.py` docstring)

**왜 필요한가**: 로봇 링크 TF(`world→base_link→…→tool0`)를 `robot_state_publisher`가 발행한다.
없으면 §4의 `1703`처럼 로봇이 없는 bag이 되어 **Octomap self-filter를 못 한다**
(로봇 팔 자체가 장애물로 잡힌다). 로봇을 움직이지 않아도 bringup은 띄운다.

### A-2. 터미널 2 — 카메라 (`reals`가 아니라 인자를 준다)

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash && \
  ros2 launch m0609_rg2_bringup camera.launch.py \
    depth_profile:=848x480x15 color_profile:=848x480x15
```

- **공식 `rs_align_depth_launch.py`를 쓰지 않는다.** 그 런치엔 `camera_calib_tf`가 없어
  `base_link→camera_link`가 bag에 안 들어간다 — §4에서 4.8GB를 버리게 만든 원인이다.
- **`reals`(인자 없음)도 쓰지 않는다.** 기본 424x240x15는 **Octomap 실시간 운용용** 저해상도다.
  녹화는 나중에 오프라인으로 돌리니 해상도를 아끼면 손해다.
- **왜 848x480x15인가**: 해상도는 YOLO 검출·depth 품질에 직결되지만 **fps는 정적 장면에서
  가치가 없다.** 30fps 대비 대역폭 절반이라 녹화 중 프레임 드랍 위험이 준다
  (이 랩탑은 `ros2_control_node`가 상시 200%대 — `md/context/constraints.md`).
- **color를 depth와 같이 848x480으로 맞추는 이유**: `align_depth`는 depth를 **컬러 해상도로
  리샘플**한다. color가 1280x720이면 aligned depth만 55MB/s로 뛴다.

⚠️ 이 런치의 한계 두 가지 (`camera.launch.py` 확인):
- `pointcloud.enable`이 `True`로 **하드코딩**돼 있다. 녹화하지도 않을 포인트클라우드를
  계속 만든다. **그래도 끄지 않는다** — 이유는 §A-5.
- `initial_reset` 인자가 없다. USB 인식이 꼬이면 케이블을 뽑았다 꽂는다.

### A-3. 터미널 3 — 녹화

```bash
ros2 bag record --compression-mode file --compression-format zstd \
  -o d435i_$(date +%m%d_%H%M)\
  /camera/camera/depth/image_rect_raw \
  /camera/camera/depth/camera_info \
  /camera/camera/aligned_depth_to_color/image_raw \
  /camera/camera/aligned_depth_to_color/camera_info \
  /camera/camera/color/image_raw/compressed \
  /camera/camera/color/camera_info \
  /camera/camera/extrinsics/depth_to_color \
  /camera/camera/gyro/sample \
  /camera/camera/accel/sample \
  /tf /tf_static
```

`<장면이름>`을 직접 넣는다(`empty`, `obstacle1`, `obstacle3`, `hand`, `robot_moving`).
타임스탬프만으로는 나중에 어느 게 뭔지 모른다 — §4의 bag들이 그 상태다.

**장면 구성**(명령어보다 이게 중요하다): **60초 × 5장면**이 10분 연속 1개보다 낫다.
§3의 용도 4개가 각각 다른 장면을 요구한다.

| 장면 | 쓰는 곳 |
|---|---|
| 빈 테이블 | Octomap 바닥 제거·노이즈 기준선 |
| 장애물 1개 | 검출 최소 케이스, 파지점 정확도 |
| 장애물 여러 개 | Octomap 해상도·플래너 회피 |
| 사람 손 진입 | 동적 장애물, Octomap 잔상(decay) 튜닝 |
| 로봇 동작 중 | self-filter 검증 — **`/tf`가 실제로 변하는 유일한 장면** |

### A-4. 녹화 직후 검증 (실기 떠나기 전에 한다)

```bash
ros2 bag info d435i_0803_XXXX_<장면이름>
```
확인할 것 — **하나라도 틀리면 그 자리에서 다시 찍는다. 나중엔 못 고친다.**

1. **토픽 11개**가 다 잡혔는가 (`ros2 bag record`는 없는 토픽을 에러 없이 건너뛴다)
2. `aligned_depth_to_color/image_raw` Count ÷ Duration ≈ **15** 인가 (드랍되면 내려간다).
   **`depth/image_rect_raw`로 재지 말 것** — `1640` bag에서 2720/62.7 = **43 Hz**로 나왔다.
   depth raw는 프로파일 fps와 다른 속도로 나가며 그 이유는 아직 규명 안 됐다(2026-08-03 미해결).
   컬러·정렬 depth는 15로 맞았다.
3. `/tf` count > 0 인가 (bringup이 떠 있었는지)
4. **`base_link→camera_link`가 실제로 들어갔는가** — 이게 §4에서 4.8GB를 버린 항목이다.
   **메시지 개수로 판정하면 안 된다**: `1647`은 `/tf_static` Count가 3인데도 그 변환이 없었다
   (robot_state_publisher가 로봇 프레임만 여러 번 발행했기 때문). 반드시 내용을 본다:

```bash
# 터미널 1 — camera.launch.py는 띄우지 않는다(띄우면 bag이 아니라 그게 잡힌다)
ros2 bag play <bag> --topics /tf_static -l
# 터미널 2
ros2 run tf2_ros tf2_echo base_link camera_link      # ⚠️ 미검증
```
변환이 안 나오면 그 bag은 로봇 좌표계에 못 올린다. **그 자리에서 다시 찍는다.**

### A-5. 왜 포인트클라우드를 끄지 않는가 (검토했고 안 하기로 함)

`camera.launch.py`에 `pointcloud` 인자를 추가해 녹화 중 끄는 방안을 검토했다. **하지 않는다.**

- 포인트클라우드는 **호스트에서 depth로부터 계산하는 파생물이라 USB 대역폭을 안 쓴다.**
  끄면 절약되는 건 CPU뿐이다.
- 그런데 **CPU는 병목이 아니라고 이 ws가 이미 측정해뒀다** (`md/context/constraints.md`,
  2026-08-01): `align_depth+enable_rgbd+pointcloud` 동시 실행 중 load average 0.5~0.6,
  realsense 노드 CPU 18.8%. 프레임 드롭은 **USB/드라이버 쪽으로 추정**됐고 CPU는 배제됐다.
- 측정으로 배제된 원인을 잡겠다고 런치 인자를 늘리는 건 근거 없는 복잡도다.

**대신 §A-4 검증 2번(`aligned_depth` ≈ 15 Hz)으로 판단한다.** 그 값이 내려가면 그때 원인을 찾는다.

⚠️ 08-01 측정에 없던 변수: 이번엔 **로봇 bringup(`ros2_control_node` 상시 200%대)이 같이 뜬다.**
그래도 선제 대응이 아니라 실측 수치로 판단한다.

---

## 0. 네임스페이스 `/camera/camera/...`는 런치와 무관하다 (2026-08-03 실측)

`alias reals="ros2 launch m0609_rg2_bringup camera.launch.py"`로 띄우고 `ros2 node list` 실행
→ **`/camera/camera`**. 토픽도 전부 `/camera/camera/...`.

이 ws의 [camera.launch.py](src/cobot_rg2/rg2/m0609_rg2_bringup/launch/camera.launch.py)는
`Node(...)`에 `namespace=`도 `name=`도 안 주는데도 두 겹이 나온다 —
**`realsense2_camera_node` 자체의 기본값이 name=`camera`, namespace=`camera`**이기 때문이다.
공식 `rs_launch.py`가 같은 값을 명시(`rs_launch.py:23-24`)할 뿐, 이름을 만들어내는 게 아니다.
즉 **어느 런치로 띄워도 토픽 경로는 같다.** state.md의 2026-08-01 실측과 일치.

그래도 녹화 직전엔 한 번 본다:
```bash
source /opt/ros/humble/setup.bash && ros2 topic list | grep -i camera
```
`ros2 bag record`는 **없는 토픽을 에러 없이 조용히 건너뛴다.** 오타 하나면 그 스트림만 빠진
bag이 나오고, 그 사실은 실기 세션이 끝난 뒤에야 드러난다.
녹화 시작 시 터미널 출력이나 `ros2 bag info`로 **토픽 11개가 다 잡혔는지** 눈으로 확인한다.

### 같은 세션에서 함께 실측한 것 (2026-08-03, `reals` 기본 인자 424x240x15)
- `color/image_raw/compressed` **15.5 Hz**, `aligned_depth_to_color/image_raw` **15.0 Hz** — 프로파일대로 나온다.
- **IMU가 기본으로 켜져 있다**: `gyro/sample`이 **199 Hz**로 실제 발행 중. `accel/sample`도 존재.
- `infra1`/`infra2`(IR 스테레오 원본), `aligned_depth_to_infra1`도 발행된다.
- `camera_calib_tf` 노드가 떠 있음 → `config/T_cam2base.npy`가 존재하고 `base_link→camera_link` TF가 나간다.

## 1. 토픽별 — 언제 필요한가

| 토픽 | 없으면 못 하는 것 | 뺄 수 있나 |
|---|---|---|
| `depth/image_rect_raw` | depth 광학계 원본. depth intrinsic 기준 포인트클라우드 재생성, depth 필터(hole filling·temporal) 오프라인 튜닝 | color 정렬만 쓸 거면 뺀다 (용량 절반) |
| `depth/camera_info` | 위 depth를 3D로 못 푼다 (fx·fy·cx·cy) | 위와 세트. 절대 따로 빼지 않는다 |
| `aligned_depth_to_color/image_raw` | **픽셀(u,v) → 3D 좌표 매핑.** YOLO 박스 중심에서 깊이 꺼내는 pick&place의 핵심 | 인지-매니퓰레이션 하려면 필수 |
| `aligned_depth_to_color/camera_info` | 위 정렬 depth의 intrinsic. **color 프로파일을 따른다**(state.md 실측) | 세트 |
| `color/image_raw/compressed` | 시각 확인, YOLO 학습/추론 입력, 라벨링 | 필수. raw 대신 compressed인 이유는 §2 |
| `color/camera_info` | compressed 컬러를 3D와 엮을 수 없다 | 필수 |
| `extrinsics/depth_to_color` | 두 광학계 사이 R,t. 정렬을 **직접 다시 계산**할 때만 | 정렬 토픽을 그대로 쓰면 사실상 안 쓴다. 1회 latched라 용량 0 → 그냥 담아둔다 |
| `gyro/sample` | 마운트 충격 포렌식 — "34초에 누가 건드림 → 이후 프레임 신뢰 불가"를 사후에 짚는다 | 담는다 (§1-1) |
| `accel/sample` | 정지 시 중력 벡터 → **hand-eye 캘리브 독립 검증** | 담는다 (§1-1) |
| `/tf_static` | `camera_link` ↔ optical frame 관계. **없으면 포인트클라우드가 로봇 좌표계에 안 붙는다** | 필수 |
| `/tf` | 로봇 관절이 움직이는 장면의 재생·Octomap 누적 | 카메라만 고정으로 볼 거면 뺄 수 있으나 용량이 작다 |

**의도적으로 뺀 것**
- `depth/color/points` — depth image + camera_info로 언제든 재생성되는 파생물인데 ~390MB/s.
  bag에 넣는 순간 디스크가 먼저 죽는다. `depth_image_proc`으로 재생 시 만든다.
- `color/image_raw` (raw) — compressed와 중복. 30fps 848x480 raw만 37MB/s.
- `*/theora`, `*/compressedDepth` — 재생 시 디코드 실패가 잦다.
- `*/metadata` — 프레임 타임스탬프·노출 등 디버깅용. 재생 파이프라인이 안 쓴다.
- `infra1`/`infra2` — IR 스테레오 원본. depth 알고리즘 자체를 뜯을 게 아니면 불필요.

### 1-1. IMU는 왜 담는가 (eye-to-hand인데도)

IMU의 통상 용도인 **VIO/SLAM은 여기서 완전히 무의미하다** — 카메라가 안 움직인다.
그럼에도 담는 이유는 둘이다.

1. **중력 벡터로 hand-eye 캘리브를 독립 검증.** 정지 상태 accel은 카메라 optical frame 기준
   중력 방향을 준다. `T_cam2base.npy`로 `base_link`에 옮기면 `-Z`가 나와야 한다. 안 나오면
   캘리브 회전이 틀린 것이다. **체커보드와 완전히 독립한 측정**이라 "캘리브가 틀렸나 코드가
   틀렸나"를 가른다. 한계: 중력축 둘레의 **yaw는 관측 불가** — 3 DOF 중 2개만 검증된다.
   (⚠️ 원리는 확실하나 이 ws에서 실행해본 적 없다. D435i accel의 바이어스가 실용 정밀도를
   내는지는 미검증.)
2. **마운트 충격 포렌식.** eye-to-hand의 가장 흔한 조용한 실패는 "누가 카메라를 건드려
   캘리브가 무효화됐는데 아무도 모름"이다. 200 Hz gyro가 있으면 bag 재생 중 충격 시점을
   짚어 이후 프레임을 버릴 수 있다. 없으면 데이터가 왜 안 맞는지 영영 모른다.

**비용/재수집 비대칭이 결정적이다.** `sensor_msgs/Imu` 200 Hz는 수십 KB/s로 영상 50MB/s 옆에서
반올림 오차인데, **안 담으면 실기를 다시 잡아야 한다.** 평소라면 YAGNI로 자를 항목이지만
비용 0 + 재수집 비쌈 조합에서는 담는 쪽이 맞다.

`unite_imu_method` 기본이 `0`(None)이라 gyro/accel이 **따로** 나온다(`rs_launch.py:69`).
합본 `/camera/camera/imu`를 원하면 `unite_imu_method:=2`로 띄워야 하지만, 위 두 용도는
원본 스트림으로 충분하다.

**IMU가 켜져 있는 이유**: `rs_launch.py:63-64`는 `enable_gyro`/`enable_accel`을 명시적으로
`false`로 **끈다.** 이 ws의 `camera.launch.py`는 그 인자를 안 주므로 **노드 자체 기본값(켬)**이
살아난다 — §0의 네임스페이스와 완전히 같은 구조다. **공식 런치의 기본값을 드라이버의
기본값으로 착각하지 말 것.**

## 2. 옵션이 맞는 이유

- `--compression-mode file` — 파일 단위로 닫힐 때 압축. `message` 모드는 메시지마다 압축해
  녹화 중 CPU를 계속 먹는다. 이 랩탑은 i7-10510U 15W에 `ros2_control_node`가 상시 200%대라
  (`md/context/constraints.md`) **녹화 중 CPU를 쓰면 프레임이 드랍된다.** `file`이 정답.
- `--compression-format zstd` — Humble rosbag2가 기본 제공. depth 16UC1은 압축이 잘 먹는다.
- `-o d435i_$(date +%m%d_%H%M)` — 같은 이름으로 두 번 녹화하면 rosbag2가 그냥 실패한다.
  타임스탬프가 그걸 막는다. 다만 **장면 이름이 안 들어간다** — 녹화 후 폴더명 뒤에
  `_empty`, `_obstacle3`, `_hand` 처럼 붙여두는 편이 나중에 훨씬 낫다.

**녹화 중 하면 안 되는 것**: 카메라→base 임시 static TF를 띄우는 것. `/tf_static`에 가짜
캘리브 값이 박히면 나중에 진짜 값과 충돌하고, bag만 봐서는 어느 쪽이 진짜인지 알 수 없다.

## 3. 이 bag으로 개발할 것

녹화의 목적은 "데이터 수집"이 아니라 **실기 없이 반복 가능한 입력을 확보하는 것**이다.
아래 넷 다 실기 점유 없이 랩탑에서 돌아간다.

1. **Octomap 파라미터 튜닝** ([[ws/cobot2/plans/2026-08-03-octomap-integration]])
   `resolution`, `max_range`, `point_subsample`을 바꿔가며 `ros2 bag play`로 같은 장면을 반복 입력한다.
   실기에서는 매번 물체를 똑같이 놓을 수 없어 비교가 성립하지 않는다. bag이면 성립한다.
   → **"장애물 1개 / 여러 개 / 사람 손 진입" 장면이 따로 필요한 이유가 이것이다.**

2. **hand-eye 캘리브 검증**
   `T_cam2base.npy`를 갈아끼우고 같은 bag을 재생해 포인트클라우드가 로봇 모델과 겹치는지 본다.
   캘리브가 틀렸는지 코드가 틀렸는지를 실기 없이 분리할 수 있는 유일한 방법.

3. **YOLO 학습/평가 데이터**
   `color/image_raw/compressed`를 프레임으로 뽑아 라벨링. 정렬 depth가 같이 있으므로
   **박스 중심의 3D 좌표를 정답으로 붙일 수 있다** — 검출 정확도와 파지점 정확도를 따로 잴 수 있다.

4. **pick&place 회귀 테스트**
   인지 파이프라인(YOLO → 3D 좌표 → 목표 pose)만 bag 입력으로 돌려 결과 좌표를 고정 기대값과 비교.
   모션은 빼고 인지만 검증하므로 로봇도 사람 승인도 필요 없다.

**따라서 장면 구성이 명령어보다 중요하다**: 60초 × 5장면(빈 테이블 / 장애물 1 / 장애물 다수 /
사람 손 진입 / 로봇 동작 중)이 10분 연속 1개보다 낫다. 위 4개 용도가 각각 다른 장면을 필요로 한다.

## 4. 녹화된 bag 실사 (2026-08-03, `ros2 bag info` + tf_static 덤프로 확인)

| bag | 크기 | 길이 | `base_link→camera_link` | 로봇 `/tf` | IMU | fps |
|---|---|---|---|---|---|---|
| `rosbag_reals_launched/d435i_0803_1640` | 159 MB | 63 s | **✅ 있음** | ✅ 628 | ✅ | 15 |
| `rosbag_reals_launched/d435i_0803_1639` | — | — | **미확인 (한 번도 안 열어봄)** | — | — | — |
| `rosbag_modified/d435i_0803_1643` | — | — | **❌ 없음** | ✅ | ❌ | 30 |
| `rosbag_modified/d435i_0803_1647` | 1.9 GB | 136 s | **❌ 없음** | ✅ 1365 | ❌ | 30 |
| `rosbag_modified/d435i_0803_1703` | 1.3 GB | 101 s | **❌ 없음** | ❌ 없음 | ❌ | 30 |

**핵심: 4.8GB짜리 `rosbag_modified` 3개는 그대로 재생하면 포인트클라우드가 로봇 좌표계에
안 붙는다.** tf_static에 카메라 내부 프레임(`camera_link→camera_depth_frame` 등)만 있고
`camera_link`를 로봇 트리에 매다는 변환이 없어 **TF 체인이 끊긴 고아 프레임**이 된다.
`1703`은 로봇 `/tf`조차 없어 self-filter도 불가능하다.

원인은 런치 차이다. `rosbag_modified`는 공식 `rs_align_depth_launch.py`로 띄웠고, 그 런치엔
`camera_calib_tf`가 없다(+`enable_gyro/accel=false`라 IMU도 빠졌다 — §1-1 참고).
**159MB짜리 `reals` bag이 4.8GB짜리보다 쓸모가 많다.** 용량이 데이터 가치가 아니다.

### 살릴 수 있다 — 재생할 때 TF만 보충하면 된다
bag에 **가짜 캘리브 값이 안 들어갔기 때문에** 충돌 없이 덧붙일 수 있다(§2 "녹화 중 하면 안 되는 것").
`camera.launch.py`의 `driver:=false`가 정확히 이 용도다 — 드라이버 없이 TF만 발행한다.

## 5. 재생 절차

**순서를 지킨다. `/tf_static`은 latched지만 소비 노드가 먼저 떠 있는 편이 안전하다.**

```bash
# 터미널 1 — TF 보충 (rosbag_modified 재생 시 필수, reals bag엔 불필요하나 띄워도 무해)
source /opt/ros/humble/setup.bash && source install/setup.bash && \
  ros2 launch m0609_rg2_bringup camera.launch.py driver:=false

# 터미널 2 — 소비 노드 (Octomap, 인지 등)를 먼저 띄운다. use_sim_time 필수
#   ros2 launch ... use_sim_time:=true

# 터미널 3 — 재생
source /opt/ros/humble/setup.bash && \
  ros2 bag play rosbag_reals_launched/d435i_0803_1640 --clock -l
```

검증된 것 (2026-08-03 실행):
- `driver:=false`가 `base_link→camera_link` = **(1.148174, 0.640096, 0.677658)**,
  quat (-0.17872839, 0.0008063, 0.94975769, -0.2569355)를 발행한다.
- 이 중 **평행이동만** `d435i_0803_1640` bag의 tf_static과 대조했고 일치했다.
  회전은 대조하지 않았다(덤프 스크립트가 translation만 읽음). 따라서 이 확인이 말해주는 것은
  "녹화 이후 npy가 안 바뀌었다"까지다.

주의:
- `--clock` ↔ 소비 노드 `use_sim_time:=true`는 **한 짝이다.** 안 맞추면 TF가
  "extrapolation into the future"로 계속 터진다.
- **실기 카메라 드라이버가 떠 있으면 먼저 끈다.** 같은 토픽에 두 소스가 겹쳐 조용히 섞인다.
- compressed 컬러를 raw로 받는 노드에 물리려면 `image_transport republish`가 필요하다.
- `-l`(loop) 없이는 재생이 끝나면 latched TF도 사라진다. 파라미터를 만지며 반복할 땐 `-l`.

### 포인트클라우드는 재생 시 만든다
녹화에서 뺐으므로(§1) 필요하면 `depth_image_proc`으로 복원한다:
```bash
ros2 run depth_image_proc point_cloud_xyz_node --ros-args \
  -r image_rect:=/camera/camera/depth/image_rect_raw \
  -r camera_info:=/camera/camera/depth/camera_info \
  -r points:=/camera/camera/depth/points_xyz \
  -p use_sim_time:=true          # ⚠️ 미검증 (state.md 절차와 동일, bag 입력으로는 미실행)
```

### bag이 zstd라 도구로 직접 열 땐 주의
⚠️ **`ros2 bag play`는 `.db3`를 bag 폴더 안에 압축 해제하고 지우지 않는다.**
2026-08-03 실측: `d435i_0803_1703`을 재생하자 폴더에 5.2GB `.db3`가 남았다(원본 `.zstd` 1.4GB는 그대로).
`rosbag_modified/`가 4.8 → 9.7GB가 됐다. **재생 후 `.db3`를 지운다.** `/` 여유가 16GB뿐이다.

`rosbag2_py.SequentialReader`는 **압축 bag을 못 연다** (`sqlite3` 플러그인이 `.db3.zstd`를
그냥 "not a database"로 거절한다 — 2026-08-03 실측). `ros2 bag play`/`info`는 알아서 푼다.
스크립트로 파싱해야 하면 `zstd -d`로 풀고 `metadata.yaml`의 `.db3.zstd` 파일명과
`compression_format`/`compression_mode`를 지운 사본을 만들어 연다.

## 6. 저장

`*.db3`/`*.mcap`은 이미 gitignore. **USB로 옮긴다. git에 올리지 않는다.**
`rosbag_modified/`가 4.8GB다 — `/` 파티션 여유가 16GB뿐이니(2026-08-03) 압축 해제 작업은
한 번에 하나씩 하고 지운다.
