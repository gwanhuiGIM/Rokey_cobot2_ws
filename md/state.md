# 세션 상태

> 현재 상태로 덮어쓴다. 로그처럼 쌓지 않는다.

**최종 갱신:** 2026-08-05

## 🟢 지금 어디까지 왔나 (2026-08-05) — GraspGenX 실기 파이프라인 관통

**명령 한 번에 촬영→세그멘테이션→GPU 추론→grasp 선택까지 돈다. 로봇은 아직 안 움직인다.**

```
ros2 service call /grasp/compute std_srvs/srv/Trigger
  [grasp_bridge_node.py] 시스템 파이썬·rclpy          [graspgen_worker.py] uv venv·GPU
    depth 10프레임 중앙값 → 세그멘테이션 → 씬 4파일 ──파이프──▶ GraspGenX (모델 1회 로드)
    ◀── grasps(base_link) ── 점수·도달·접근축 필터 ──▶ /grasp/best, /grasp/best_tcp, /grasp/candidates
```

| 실측 결과 (사과) | 값 |
|---|---|
| 손끝(TCP) vs 자로 잰 사과 위치 | **1.1 cm** |
| 세그멘테이션 표면중심 vs 실측 | **0.1 cm** |
| 접근축 `R[2,2]` | −1.00 (수직) |
| grasp 후보 | 12~32개 (씬에 따라) |

- 파일: `scripts/{capture_graspgenx_scene,graspgen_worker,grasp_bridge_node}.py` + 테스트 3종.
  **아직 ROS 패키지가 아니다**(`scripts/` 단독 실행). 동작이 굳으면 패키지로 올린다.
- 계약·튜닝값·함정은 [[ws/cobot2/context/constraints]] "GraspGenX 관련"이 단일 출처다.
- ⛔ **실기 모션 전 블로커 1개**: `tool0` 플랜지면 → RG2 손끝 **실측 거리**.
  URDF는 190 mm(어댑터 오프셋 0), 매뉴얼은 220 mm + 브라켓 10 mm.
  차이가 실재하면 MoveIt이 손끝을 **40 mm 더 깊이** 민다. 줄자 한 번이면 갈린다.

> 📁 **문서 전체 지도는 [[ws/cobot2/README]]에 있다.** 어느 문서가 무엇의 단일 출처인지 거기서 본다.

## 계정/환경
- 공유 랩탑(`rokey`)의 `kimkh` 계정, `cobot2_ws`.
- git: remote 이름 `personal` = `https://github.com/gwanhuiGIM/0730_cobo2_personal` (**HTTPS**), 브랜치 `init_sett`(main보다 앞섬, 아직 미머지).
- git identity는 **repo-local**로 설정됨(`user.name=kimkh`, `user.email=wook9980@gmail.com`). repo-local이라 **다른 PC에서 clone하면 다시 설정해야 한다.**
- push는 **VS Code Source Control**로 한다(터미널 git 아님). 정상 동작 확인 — 커밋 `6a78c78`까지 push 완료, 워킹트리 clean.
- `~/.claude/CLAUDE.md`(전역 공통 규칙) 복원 완료.

## 두 PC 체제 — ⚠️ **hostname·계정으로는 구분되지 않는다** (2026-08-05 정정)

두 PC 모두 hostname `rokey`, 계정 `kimkh`, `/home` 구성까지 같다. **`nvidia-smi` 유무로 구분한다.**

- **GPU PC (지금 이 머신)**: **RTX 4060 Laptop 8GB**, 드라이버 595.84. 로봇(`192.168.1.100`)·D435i **직결**.
  ⚠️ VRAM이 계획서들이 가정한 12GB가 아니라 **8GB**다 — `--num_grasps`를 64로 시작한다.
  ⚠️ `kimkh`가 **docker 그룹에 없고**(멤버는 `rokey`) `nvidia-container-toolkit`도 미설치 →
  Isaac ROS 컨테이너 경로(`run_dev.sh`)는 지금 못 쓴다. GraspGenX만 하면 docker 불필요.
- **개인PC(노트북)**: NVIDIA GPU 없음. rosbag으로 개발.
- 상세 하드웨어 실측표는 [[ws/cobot2/context/constraints]] "🔴 이 랩탑 하드웨어".
- 역할 분담·시뮬레이션 범위·동기화 방법은 [[ws/cobot2/plans/2026-08-01-pc-role-split]] 참조.

### GPU PC 최초 세팅 절차
```bash
git clone https://github.com/gwanhuiGIM/0730_cobo2_personal.git cobot2_ws
cd cobot2_ws && git checkout init_sett          # 기본 브랜치가 main이라 필수
git config user.name "kimkh" && git config user.email "wook9980@gmail.com"
./scripts/setup_isaac_ros.sh                    # isaac_ros-dev/ 복원 (release-3.2)
code .                                          # 이후 VS Code Source Control로 커밋/푸시
```

## Isaac ROS 소스 상태
- `isaac_ros-dev/`는 **커밋하지 않는다**(.gitignore). `scripts/setup_isaac_ros.sh`로 재현한다 — 136MB·수천 파일이라 VS Code Source Control이 마비되고, `**/.git/` 규칙 때문에 pull한 쪽에서 태그·히스토리를 알 수 없게 되기 때문.
- 개인PC에 `release-3.2`로 클론 완료: `isaac_ros_common`(`scripts/run_dev.sh` 존재 확인), `isaac_ros_nvblox` `v3.2-14`(submodule `nvblox_core` 포함).
- **release-4.x 금지** — `run_dev.sh`가 Isaac ROS CLI로 이전되어 사라졌고 사실상 Jazzy 중심. 4.4로 받았다가 막혀서 3.2로 재클론한 이력 있음(2026-08-01).
- `.isaac_ros_common-config` = `CONFIG_IMAGE_KEY=ros2_humble.realsense`
- ⚠️ **`setup_isaac_ros.sh`는 GraspGenX를 클론하지 않는다.** 새 PC에서는 따로 받아야 한다:
  `git clone https://github.com/NVlabs/GraspGenX.git isaac_ros-dev/src/GraspGenX` → `uv sync` (venv, torch).
  체크포인트(HF `adithyamurali/GraspGenXModel`)는 **1.7 GB LFS**이고, 매체 복사로 옮기면
  **크기가 맞아도 내용이 0으로 채워질 수 있다** — 반드시 `sha256sum` 검증(해시는 constraints.md).
- `isaac_ros-dev/`에 `COLCON_IGNORE` 있음. **없으면 루트 `colcon build`가 CUDA 없이 nvblox를 빌드하려 든다**
  (colcon은 `base_paths=['.']`로 repo 전체를 훑는다).
- `realsense-ros` 클론 **불필요** — apt `ros-humble-realsense2-camera 4.58.2` 설치됨(GPU PC도 동일하다는 사용자 진술, 미검증).

## 열려 있는 이슈
- **GPU PC에 GPU 있음 — 사용자 확인 (2026-08-02).** 스프린트 Day4~5(nvblox·FoundationPose·GraspGenX) 재작성 리스크는 해소됐다.
  단 **`docker info | grep -i runtime`으로 nvidia-docker 런타임이 잡히는지는 아직 미확인** — nvblox 컨테이너 빌드 전에 확인할 것. GPU 모델·VRAM도 미확인(FoundationPose VRAM 요구량 판단에 필요).
- 하드웨어(M0609 + RG2 + D435i + C270)는 `.bashrc` alias 추론이며 실기로 재확인되지 않음.
- **D435i depth rosbag 미확보** — 이게 있어야 개인PC에서 실기 없이 Octomap·플래너·상태머신 개발 가능. 절차는 아래 "출근 후 D435i 세션" 참조.
- **카메라 마운트 강성 미확보** — 견고한 고정이 아직 어려움. 캘리브는 **잠정(provisional)**으로 취급하고, Day4 인식 정확도 실측 검증은 마운트 확정 후로 미룬다. 개발용 TF로는 잠정값으로 충분하다.
- **Day4 인식 방식 변경(2026-08-02)**: ray-plane intersection → **FoundationPose**(6D pose), 하드코딩 그립 → **GraspGenX**. 평면 가정이 사라지는 건 이득이지만 **GPU 의존이 커졌다** — Day4는 GPU PC 전용이 된다. `GraspGenX` 저장소는 실재 미확인(`NVlabs/GraspGen`은 확인됨).
- **캘리브 방식 변경(2026-08-02)**: `easy_handeye2` → `corecode/Calibration_Tutorial`(`eye2hand_calibration.py` / `handeye_calibration.py`). 두 알고리즘 모두 합성 데이터로 정답 복원 확인(오차 ~1e-13). npy(mm) → static TF(m) 변환은 `src/cobot_rg2/rg2/m0609_rg2_bringup/scripts/calib_npy_to_tf.py`. ~~미해결: `data_recording.py`가 `set_tcp` 후의 `posx`를 기록하므로 결과의 부모 프레임이 flange가 아니라 TCP다~~ → **해소(2026-08-03).** eye-to-hand에서는 판↔그리퍼 변환 G가 AX=XB 유도에서 소거되므로, "그리퍼"를 flange로 잡든 TCP로 잡든 **`T_cam2base` 결과는 같다**(X는 base·camera 쪽 변환이라 팔 쪽 기준 프레임 선택과 무관). `--selfcheck`가 이걸 합성 데이터로 확인한다. 현재 `RECORD_IN_FLANGE_FRAME = False`(TCP 기준)로 두어도 문제 없다.
- **Day1.5 압축 경로 신설** — 캘리브·인식 전부 생략하고 "장애물 놓으면 궤적이 바뀐다"만 보여주는 시연용 경로. GPU 불필요, 개인PC 가능. 임시 static TF를 쓰므로 **rosbag 녹화와 절대 겹치면 안 된다.**
- **좌표 규약 버그 발견·수정 (2026-08-02)** — npy는 OpenCV **optical** 규약인데 ROS `camera_link`(body 규약)로 발행해 클라우드가 로봇 옆으로 90° 튀었다. `src/cobot_rg2/rg2/m0609_rg2_bringup/scripts/calib_npy_to_tf.py`가 이제 기본 보정한다. **`inv(T)` 문제가 아니었다** — 규약을 먼저 의심할 것. 채점표는 [[ws/cobot2/context/constraints]].
- **보류: `sensors_3d.yaml`의 `max_range: 1.5`** (2026-08-03 사용자 결정). 카메라~base_link
  실측 거리가 이 값을 넘어섰고(주석은 아직 낡은 "약 1.48 m" 기준이라 값과 근거가 어긋나 있다),
  그만큼 로봇 베이스 주변 점이 잘린다. **재캘리브로 거리가 확정된 뒤에 정한다** — 지금 맞춰봐야
  또 바뀐다. 재캘리브 직후 `max_range`와 주석을 같이 고칠 것.
- **캘리브 품질: 현행값은 자체 진단 불합격 (2026-08-03, `data/` 34장)**. AX=XB 병진잔차
  중앙값 **40.1 mm**, 31쌍 중 **21쌍**이 30 mm 초과. 자세를 34장으로 늘렸는데 26장이던
  직전 수집(23.5 mm)보다 **나빠졌다** — 장수가 아니라 자세 품질 문제다.
  **다음 캘리브는 이 40.1 mm와 비교한다.** `eye2hand_calibration.py`가 매 실행 찍는다.
  **자세 재수집은 미룬다 (2026-08-03 사용자 결정)** — 현행값으로 개발을 진행하고,
  마운트 강성이 확보되는 시점에 재수집한다. 그 전까지 인식 정확도 실측은 하지 않는다.
  - RMS 재투영오차는 공장 내부파라미터를 쓰는 동안 **결과 품질이 아니다**(참고값).
  - **LOO 수치를 재현성으로 읽지 말 것** — 리스트 순서만 섞어도 같은 크기로 움직이고,
    계통오차에는 눈이 멀다. 잔차 큰 쌍을 한꺼번에 빼면 80 mm 옮겨간 반면 LOO는 3.6 mm였다.
    (2026-08-03 cross-review에서 반증. 그 전 세션에 내가 "LOO 1.5 mm = 양호"로 적은 건 오판)
- ✅ **`max_range` 1.5 → 2.0으로 되돌림 (2026-08-03, 사용자 결정).** 1.5는 카메라~base 거리 **1.684 m**보다
  작아 베이스 부근을 잘라내고 있었다. 원래 목적이던 CPU 절감은 체감되지 않았고, `max_range`는 애초에
  CPU 손잡이가 아니다(비용은 거리가 아니라 점 개수에 비례 → `point_subsample`·카메라 프로파일이 실효 손잡이).
  **아직 실기 미검증**: 2.0에서 ① 뒷벽이 장애물로 잡히지 않는지 ② 장애물 경계가 또렷해지는지
  ③ `move_group` CPU가 견디는지 — 다음 실기에서 확인. 경위: [[ws/cobot2/errors-log]] §7
- **octomap_rviz_plugins 미설치** — `/octomap_binary`·`/octomap_full`을 RViz에서 볼 수 없다. 당장은 `/octomap_point_cloud_centers`(PointCloud2)로 우회 중. `topic_tools`도 미설치(throttle 불가 → 카메라 프로파일에서 줄인다).

## 다음 할 일 (순서 고정 — 위가 막히면 아래로 내려가지 않는다)
> 이 절이 "다음에 뭐 하지"의 단일 출처다. 끝난 항목은 지우고 Day 진행 절로 옮긴다.
> 08-03 상세 계획: [[ws/cobot2/plans/2026-08-03-octomap-integration]]

> ✅ 1~3번(플러그인 설치 / 프로파일 축소 / self-filter / 장애물 회피)은 **2026-08-03에 끝났다.**
> 결과·채택값·검증 상태는 [[ws/cobot2/review_moveit]]로 옮겼다. 아래는 남은 것만이다.

0. ⛔ **`tool0` 플랜지면 → RG2 손끝 거리 실측 (줄자)** — 실기 모션의 전제.
   URDF 190 mm vs 매뉴얼 220+10 mm. 차이가 있으면 **`tcp_offset_m`이 아니라
   `onrobot_rg2.xacro`의 `origin xyz`에 어댑터 두께를 넣는다.** 근거는 constraints.md.
0-b. **물체 이름 지정** — 지금 선택 정책은 "점수 최고"라 컵·노이즈를 집을 수 있다.
   실제로 2026-08-05에 558 px 노이즈를 고르고 사과는 후보 0이었던 사례가 있다.
   `-p min_pixels:=1000`이 임시 방편이고, 정본은 YOLO-seg + 음성 타겟팅
   ([[ws/cobot2/plans/2026-08-05-foundationpose-graspgenx-pick]] §1).
   ⚠️ **FoundationPose는 마스크를 만들어주지 않는다** — 마스크를 **입력으로 요구**하고 CAD 메시도 필요하다.
   ROI/마스크 출처는 여전히 검출기(YOLO-seg 또는 RT-DETR)다.
0-c. **RG2 개구 폭(`rgwd`) 계산** — grasp에서 폭을 뽑아 그리퍼 명령으로. 아직 미구현.

1. **README 4절 체크1~3 명령 실측** — `tf2_echo base_link camera_link`, `topic hz .../points`,
   `topic echo /dsr01/joint_states`는 **아직 한 번도 실행 안 됨**(README에 ⚠️ 미검증 표기해 둠).
   실기 켠 김에 돌려서 기대값과 대조하고 경고를 지운다.
2. **D435i depth rosbag 녹화** — 개인PC에서 실기 없이 개발하려면 필수.
   ⚠️ **명령의 단일 출처는 [[ws/cobot2/rosbag-d435i]] §A다.** 아래 "출근 후 D435i 세션"에는 순서만 남겼다.
3. **캘리브 오차 정량 측정** — 알려진 좌표의 물체로 cm 단위. `padding_offset` 해석과 cuRobo 비교의 전제.
4. **OMPL 플래너 성공률·계획시간 로그** — 스프린트 Day3 P1. cuRobo 비교의 기준선.

**상시 실행** — ⚠️ **MoveIt octomap 경로에는 더 이상 필요 없다(2026-08-02).**
`sensors_3d.yaml`이 RealSense가 직접 발행하는 `/camera/camera/depth/color/points`를 쓰므로
`depth_image_proc`를 거치지 않는다. 아래 노드는 **RViz 육안 확인·nav2용 별도 클라우드**일 때만 띄운다.
(이 노드가 죽어도 MoveIt 충돌회피는 안 멈춘다 — 예전 설명이 틀렸다.)
```bash
export ROS_DOMAIN_ID=93
ros2 run depth_image_proc point_cloud_xyz_node --ros-args \
  -r image_rect:=/camera/camera/depth/image_rect_raw \
  -r camera_info:=/camera/camera/depth/camera_info \
  -r points:=/camera/camera/depth/points_xyz
```
카메라를 재기동하면 이 노드도 같이 죽는다. `ros2 topic list | grep points_xyz`로 먼저 확인할 것.

## Day2 진행 (2026-08-02)
- ✅ 캘리브 결과 → static TF 연결 성공: `base_link → camera_link`, `tf2_echo base_link camera_depth_optical_frame` 정상.
  (이 줄에 처음 적혀 있던 993.4 mm는 좌표 규약 버그 수정 **전**의 값이라 폐기.)
  ⚠️ **여기 적혀 있던 `[1.148, 0.640, 0.678]` (약 1.48 m)는 2026-08-03 재캘리브로 폐기.**
  값의 출처는 `T_cam2base.npy` 하나뿐이다 — **거리 수치를 문서에 베껴 적지 말 것.**
  읽는 법과 사고 이력은 [[ws/cobot2/context/constraints]].
- ✅ **`base_0`는 TF에 존재하지 않는 프레임임을 확인** — `base_link`가 맞다. 계획서 전체를 `base_0`→`base_link`, `link6`→`link_6`으로 수정 완료. 근거·표는 [[ws/cobot2/context/constraints]].
- ✅ **좌표 규약 버그 수정 후 육안 검증 통과** — 클라우드 속 로봇 팔이 모델에 정확히 포개짐. 캘리브 잠정값 사용 가능.
- ✅ **`octomap_server`는 이 파이프라인에서 불필요하다고 결론.** MoveIt은 `/octomap_binary`를 구독하지 않고
  `move_group` 내부에서 octree를 직접 만든다. 둘 다 돌리면 CPU 이중 소모 — 정식 경로는 `sensors_3d.yaml`이다.
- ✅ **`m0609_rg2_moveit/config/sensors_3d.yaml` 작성 완료** + `moveit.launch.py`에서 주입(`octomap:=true` 기본).
  실제 채택값: **`octomap_frame: base_link`**, **`octomap_resolution: 0.02`**(계획은 `0.03`),
  토픽 `/camera/camera/depth/color/points`(계획은 `/depth/points_xyz`), 센서명 `realsense_pointcloud`.
  `ros2 param get /move_group`으로 주입까지 확인. ⛔ **다만 `moveit-ros-perception` 미설치라 플러그인 로드는 실패 중**(위 1번).
  ※ 한때 `world`로 적혀 있었으나 **틀렸다**(2026-08-02 실측): SRDF에 `virtual_joint(fixed, parent_frame="world")`가
  있어도 MoveIt은 fixed 타입으로는 모델 프레임을 만들지 않아 플래닝 프레임이 루트 링크(`base_link`)로 남는다.
  `frame_id='world'`로 CollisionObject를 발행하면 `Unknown frame: world` 에러와 함께 **조용히 무시**된다.
  RViz Scene Objects도 같은 규칙. 경위는 [[ws/cobot2/context/constraints]].
- ✅ **캘리브 결과를 launch가 npy에서 직접 계산** — `m0609_rg2_bringup/config/T_cam2base.npy` →
  `camera.launch.py`가 매 실행 `calib_npy_to_tf.py`로 static TF 생성. **하드코딩된 `static_transform_publisher`
  명령을 다시 만들지 말 것** (낡은 값으로 340 mm 어긋난 이력 있음).
- ✅ **런치 3분할 확정**: `bringup`(로봇 전용) / `camera`(RealSense + 캘리브 TF) / `moveit`(move_group + JTC spawner + RViz).
  `bringup_camera.launch.py`는 **eye-in-hand 전용**(URDF가 camera_link를 tool0에 붙임) — 현재 리그와 섞으면 TF가 깨진다.
- ✅ **MoveIt 실기 Plan·Execute 성공** — Execute ABORTED의 원인은 두 개였다: bringup이 `dsr_moveit_controller`를
  안 띄움 + 네임스페이스 불일치. `moveit.launch.py`에 spawner 추가 + `moveit_controllers.yaml`의 컨트롤러 이름을
  절대경로 `/dsr01/dsr_moveit_controller`로. **`dsr_controller2`와 동시 active 가능**(인터페이스를 claim하지 않는 서비스 래퍼).
- ✅ **팀원용 통합 `README.md` 작성**(ws 루트) — 3터미널 실행 절차·인자표·기능확인 체크1~3·알려진 함정.
- ⚠️ **로봇 명령 경로가 두 개 살아 있다**: `dsr_controller2`(서비스 movej/movel → DRFL) 와
  `dsr_moveit_controller` (JTC → `Drfl.servoj_rt`/`Drfl.amovej`, `dsr_hw_interface2.cpp:494-503`).
  **동시에 명령하지 말 것.**
- ⚠️ `realsense-viewer`가 USB를 독점해 ROS 카메라 노드를 죽인다 — 증상이 "TF 프레임 없음"으로 나와 오진 유발. 뷰어 먼저 닫을 것.

## 출근 후 D435i 세션 (순서 고정)

**rosbag → 캘리브 순서로 한다.** 캘리브가 그날 제일 잘 깨지는 단계라, 먼저 하다 실패하면 빈손이 된다.
캘리브 결과는 static TF 6개라서 bag 재생 시 `static_transform_publisher`로 나중에 얹을 수 있다 — depth 데이터 자체는 캘리브와 무관하게 유효하다.

1. 카메라를 최종 위치에 **최대한 고정** + 마스킹테이프 표시 + 사진 (재현용)
2. **rosbag 녹화** ← 여기서 개인PC 작업이 풀린다
3. **eye-to-hand 캘리브** — 1~3 사이에 카메라를 건드리지 않는다 (건드리면 bag과 짝이 안 맞음)
4. 결과 `T_cam2base.npy`는 **사본을 만들지 않는다** — 정본은 `corecode/Calibration_Tutorial/`이고
   `m0609_rg2_bringup/config/`는 symlink다. (~~`config/handeye/`에 복사해 커밋~~ 은 폐기된 지시다.)

> ⚠️ **녹화 런치·토픽 목록·재생 절차는 여기 두지 않는다. 단일 출처는 [[ws/cobot2/rosbag-d435i]] §A다.**
> 예전 이 자리에 있던 `rs_align_depth_launch.py` 기반 명령은 **폐기됐다** — 그 런치엔 `camera_calib_tf`가
> 없어 `base_link→camera_link` 없는 bag 4.8GB가 나왔다. 현행은 `camera.launch.py`(= `reals` alias)다.
> 녹화 중 **임시 static TF를 띄우지 말 것**(bag의 `/tf_static`에 가짜 값이 박힌다)만 여기 남긴다.

## 문서 위치 규칙
- 작업 문서는 **`md/` 한 곳만** 쓴다 (커밋됨). `docs/`는 PDF 서고 전용이며 ignore.
- 2026-08-01: `docs/`에 있던 `state.md`·`context/constraints.md` 낡은 사본 삭제. ignore된 위치에 문서가 있으면 git이 갱신 누락을 잡아주지 못한다.

## 이 ws에서 확인된 사실 (실측)
- **doosan-robot2 launch의 `model` 기본값이 `m1013`** — M0609 쓸 때마다 `model:=m0609` 명시 필요. `dsr_bringup2_{rviz,gazebo,mujoco,moveit}.launch.py` 모두 해당.
- 시뮬 경로 3종 존재: virtual 모드(DRCF 에뮬레이터, `install_emulator.sh` 선행 필요), Gazebo(`dsr_gazebo2`), MuJoCo(`dsr_mujoco`).
- RealSense D435I 도메인/지터 이슈는 [[ws/cobot2/context/constraints]]에 기록.
- **D435i 토픽 네임스페이스는 `/camera/camera/...`** (2026-08-01 `ros2 topic list` 실측). 계획서 초안의 `/d435i/...`는 오기다. 토픽 이름은 **런치 명령이 정하지 마운트 방식(eye-in-hand/eye-to-hand)이 정하지 않는다** — 마운트를 바꿔도 이름은 그대로고 TF 부모 프레임만 바뀐다.
- `align_depth.enable:=true`일 때 `aligned_depth_to_color`의 해상도는 **depth가 아니라 color 프로파일을 따른다.** 대역폭 계산 시 주의.
