# 세션 상태

> 현재 상태로 덮어쓴다. 로그처럼 쌓지 않는다.

**최종 갱신:** 2026-08-03

## 계정/환경
- 공유 랩탑(`rokey`)의 `kimkh` 계정, `cobot2_ws`.
- git: remote 이름 `personal` = `https://github.com/gwanhuiGIM/0730_cobo2_personal` (**HTTPS**), 브랜치 `init_sett`(main보다 앞섬, 아직 미머지).
- git identity는 **repo-local**로 설정됨(`user.name=kimkh`, `user.email=wook9980@gmail.com`). repo-local이라 **다른 PC에서 clone하면 다시 설정해야 한다.**
- push는 **VS Code Source Control**로 한다(터미널 git 아님). 정상 동작 확인 — 커밋 `6a78c78`까지 push 완료, 워킹트리 clean.
- `~/.claude/CLAUDE.md`(전역 공통 규칙) 복원 완료.

## 두 PC 체제 (2026-08-01 확정)
- **개인PC(이 노트북)**: NVIDIA GPU **없음**(`nvidia-smi` 없음, docker 기본 런타임 `runc`). nvblox 빌드/실행 불가.
- **GPU PC**: 별도 머신. nvblox 빌드·실행 전용. GPU/nvidia-docker 유무는 **아직 미확인** — Day0 최우선 확인 항목.
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
- `realsense-ros` 클론 **불필요** — apt `ros-humble-realsense2-camera 4.58.2` 설치됨(GPU PC도 동일하다는 사용자 진술, 미검증).

## 열려 있는 이슈
- ⛔ **캘리브 수집이 막혀 있다 (2026-08-03)**: `data_recording.py`의 `set_tool('Tool Weight')`가 이름이 맞는데도
  `-1`을 돌려줘 `SystemExit`. 등록명은 실기로 확인됨(`Tool Weight` / `GripperDA_v1`). 가설·배제 순서는
  [[ws/cobot2/context/constraints]] "수집 시 주의" 절. **다음 실기 세션의 1번 항목.**
- **`data_recording.py` 알려진 버그 2건(미수정)**: `:48` `DR_init.dsr__id` 오타(언더바 누락 → 로그의 `_robot_id =`가
  빈 값. `_srv_name_prefix`가 어차피 `''`라 동작엔 무해), `:134` `cap.release()`가 `USE_REALSENSE_TOPIC=True`일 때
  `None`이라 ESC 종료 시 크래시(저장은 끝난 뒤라 데이터는 안 날아감).
- **캘리브 사본 480 mm 사고 → symlink로 해결 (2026-08-03)**. `config/T_cam2base.npy`가 이제 `corecode/` 정본을
  가리키는 symlink다. 재캘리브 후 `cp` 불필요. 경위·남은 함정(merge-install)은 [[ws/cobot2/context/constraints]].
- **미스테이징 변경분(리뷰 지적 있음)**: `sensors_3d.yaml`의 `max_range: 1.5`(주석은 2.5 — 로봇 뒤쪽이 잘린다),
  `padding_offset: 0.1`(링크 주변 10 cm 삭제 → 집으려는 물체까지 지워질 수 있다). 실측 후 확정할 것.
- **GPU PC에 GPU 있음 — 사용자 확인 (2026-08-02).** 스프린트 Day4~5(nvblox·FoundationPose·GraspGenX) 재작성 리스크는 해소됐다.
  단 **`docker info | grep -i runtime`으로 nvidia-docker 런타임이 잡히는지는 아직 미확인** — nvblox 컨테이너 빌드 전에 확인할 것. GPU 모델·VRAM도 미확인(FoundationPose VRAM 요구량 판단에 필요).
- 하드웨어(M0609 + RG2 + D435i + C270)는 `.bashrc` alias 추론이며 실기로 재확인되지 않음.
- **D435i depth rosbag 미확보** — 이게 있어야 개인PC에서 실기 없이 Octomap·플래너·상태머신 개발 가능. 절차는 아래 "출근 후 D435i 세션" 참조.
- **카메라 마운트 강성 미확보** — 견고한 고정이 아직 어려움. 캘리브는 **잠정(provisional)**으로 취급하고, Day4 인식 정확도 실측 검증은 마운트 확정 후로 미룬다. 개발용 TF로는 잠정값으로 충분하다.
- **Day4 인식 방식 변경(2026-08-02)**: ray-plane intersection → **FoundationPose**(6D pose), 하드코딩 그립 → **GraspGenX**. 평면 가정이 사라지는 건 이득이지만 **GPU 의존이 커졌다** — Day4는 GPU PC 전용이 된다. `GraspGenX` 저장소는 실재 미확인(`NVlabs/GraspGen`은 확인됨).
- **캘리브 방식 변경(2026-08-02)**: `easy_handeye2` → `corecode/Calibration_Tutorial`(`eye2hand_calibration.py` / `handeye_calibration.py`). 두 알고리즘 모두 합성 데이터로 정답 복원 확인(오차 ~1e-13). npy(mm) → static TF(m) 변환은 `src/cobot_rg2/rg2/m0609_rg2_bringup/scripts/calib_npy_to_tf.py`. ~~미해결: 부모 프레임이 flange냐 TCP냐~~ → **해결(2026-08-03)**: `RECORD_IN_FLANGE_FRAME` 플래그로 노출, 기본 `False`(TCP 기준). eye-to-hand에서는 판↔그리퍼 변환이 AX=XB에서 소거되어 **어느 쪽이든 결과가 같다**(`eye2hand_calibration.py --selfcheck`로 확인). `False`인 이유는 정확도가 아니라 이후 pick 코드와 프레임을 맞추기 위해서다.
- **Day1.5 압축 경로 신설** — 캘리브·인식 전부 생략하고 "장애물 놓으면 궤적이 바뀐다"만 보여주는 시연용 경로. GPU 불필요, 개인PC 가능. 임시 static TF를 쓰므로 **rosbag 녹화와 절대 겹치면 안 된다.**
- **좌표 규약 버그 발견·수정 (2026-08-02)** — npy는 OpenCV **optical** 규약인데 ROS `camera_link`(body 규약)로 발행해 클라우드가 로봇 옆으로 90° 튀었다. `src/cobot_rg2/rg2/m0609_rg2_bringup/scripts/calib_npy_to_tf.py`가 이제 기본 보정한다. **`inv(T)` 문제가 아니었다** — 규약을 먼저 의심할 것. 채점표는 [[ws/cobot2/context/constraints]].
- **octomap_rviz_plugins 미설치** — `/octomap_binary`·`/octomap_full`을 RViz에서 볼 수 없다. 당장은 `/octomap_point_cloud_centers`(PointCloud2)로 우회 중. `topic_tools`도 미설치(throttle 불가 → 카메라 프로파일에서 줄인다).

## 다음 할 일 (순서 고정 — 위가 막히면 아래로 내려가지 않는다)
> 이 절이 "다음에 뭐 하지"의 단일 출처다. 끝난 항목은 지우고 Day 진행 절로 옮긴다.
> 08-03 상세 계획: [[ws/cobot2/plans/2026-08-03-octomap-integration]]

1. **`sudo apt install ros-humble-moveit-ros-perception`** — ⛔ **현재 미설치**(확인함). `PointCloudOctomapUpdater`
   플러그인이 이 패키지에 있다. 없으면 `sensors_3d.yaml`을 채워도 플러그인 로드 실패로 **조용히** octomap이 안 생긴다.
1.5. **depth 프로파일을 `424x240x15`로 낮추고 켠다** — i7-10510U 15W·GPU 없음·`ros2_control_node` 204%에
   848x480x30(12.2 M point/s)은 안 돌아간다(어제 `octomap_server`에서 확인). `camera.launch.py`에 인자 추가.
2. **`/moveit/filtered_cloud`로 self-filter 검증** ← **진짜 관문.** 로봇 팔이 지워졌는지 RViz로 눈으로 본다.
   남아 있으면 로봇이 자기 몸을 장애물로 보고 한 발짝도 못 움직인다. `padding_offset`을 키운다.
   (1번 설치 전에는 플러그인이 없어 이 토픽 자체가 안 나온다.)
3. 장애물 놓고 궤적이 바뀌는지 확인 (fake execution 먼저).
4. **README 4절 체크1~3 명령 실측** — `tf2_echo base_link camera_link`, `topic hz .../points`,
   `topic echo /dsr01/joint_states`는 **아직 한 번도 실행 안 됨**(README에 ⚠️ 미검증 표기해 둠).
   실기 켠 김에 돌려서 기대값과 대조하고 경고를 지운다.
5. **D435i depth rosbag 녹화** — 개인PC에서 실기 없이 개발하려면 필수. 절차는 아래 "출근 후 D435i 세션".

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
  **유효값은 `Translation: [1.148, 0.640, 0.678]` (약 1.48 m)** — 사용자 확인(2026-08-02).
  (이 줄에 처음 적혀 있던 993.4 mm는 좌표 규약 버그 수정 **전**의 값이라 폐기.)
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
  `dsr_moveit_controller`(JTC → `Drfl.servoj_rt`/`Drfl.amovej`, `dsr_hw_interface2.cpp:494-503`).
  **동시에 명령하지 말 것.**
- ⚠️ `realsense-viewer`가 USB를 독점해 ROS 카메라 노드를 죽인다 — 증상이 "TF 프레임 없음"으로 나와 오진 유발. 뷰어 먼저 닫을 것.

## D435i rosbag 세션

**명령어는 여기 두지 않는다 → [[ws/cobot2/rosbag-d435i]] §A(재녹화)·§5(재생)가 유일한 출처다.**
2026-08-03에 못 쓰는 bag 4.8GB가 나왔다. 원인은 이 절과 `md/Personal_0801`이 둘 다
`rs_align_depth_launch.py`를 지시했고, **그 런치엔 `camera_calib_tf`가 없다는 걸 몰랐던** 것이다.
사본이 여러 곳이면 이런 오류를 한 번에 못 고친다. 명령을 여기 다시 붙여넣지 말 것.

이 절에는 **명령이 아닌 것**만 남긴다:
- 카메라를 최종 위치에 **최대한 고정** + 마스킹테이프 표시 + 사진 (재현용)
- **카메라를 옮겼으면 재캘리브가 녹화보다 먼저다.** 낡은 `T_cam2base.npy` 상태로 녹화하면
  bag에 가짜 `base_link→camera_link`가 박혀 **없는 것보다 나쁘다**.
  ("rosbag → 캘리브" 순서는 npy가 없던 시절 규칙이다. 지금은 유효한 npy가 있다.)
- 캘리브 산출물 `T_cam2base.npy` → `config/handeye/`에 `_provisional` 붙여 복사·커밋.
  `src/cobot_rg2/rg2/m0609_rg2_bringup/scripts/calib_npy_to_tf.py`로 static TF 인자 생성
- bag은 USB로 이동. git 금지(`*.db3`/`*.mcap` 이미 ignore)

## 문서 위치 규칙
- 작업 문서는 **`md/` 한 곳만** 쓴다 (커밋됨). `docs/`는 PDF 서고 전용이며 ignore.
- 2026-08-01: `docs/`에 있던 `state.md`·`context/constraints.md` 낡은 사본 삭제. ignore된 위치에 문서가 있으면 git이 갱신 누락을 잡아주지 못한다.

## 이 ws에서 확인된 사실 (실측)
- **doosan-robot2 launch의 `model` 기본값이 `m1013`** — M0609 쓸 때마다 `model:=m0609` 명시 필요. `dsr_bringup2_{rviz,gazebo,mujoco,moveit}.launch.py` 모두 해당.
- 시뮬 경로 3종 존재: virtual 모드(DRCF 에뮬레이터, `install_emulator.sh` 선행 필요), Gazebo(`dsr_gazebo2`), MuJoCo(`dsr_mujoco`).
- RealSense D435I 도메인/지터 이슈는 [[ws/cobot2/context/constraints]]에 기록.
- **D435i 토픽 네임스페이스는 `/camera/camera/...`** (2026-08-01 `ros2 topic list` 실측). 계획서 초안의 `/d435i/...`는 오기다. **런치와 무관하다** — `realsense2_camera_node` 자체 기본값이 name=`camera`+namespace=`camera`라, ws의 `camera.launch.py`처럼 `name=`/`namespace=`를 안 줘도 두 겹이 된다 (2026-08-03 `ros2 node list` → `/camera/camera` 재확인).
- **bag의 쓸모는 용량이 아니라 tf_static이 정한다** (2026-08-03). 공식 `rs_align_depth_launch.py`로 찍은 `rosbag_modified/` 3개(4.8GB)는 `base_link→camera_link`가 없어 포인트클라우드를 로봇 좌표계에 못 올린다 — 그 런치엔 `camera_calib_tf`가 없기 때문. `reals`(ws 런치)로 찍은 159MB짜리가 더 쓸모 있다. 재생 시 `camera.launch.py driver:=false`로 TF만 보충하면 살릴 수 있다. 상세는 [[ws/cobot2/rosbag-d435i]] §4-5
- **D435i IMU가 기본으로 켜져 있다** — `gyro/sample` 199 Hz 실발행(2026-08-03 `ros2 topic hz`). 드라이버 기본이 꺼짐일 거라는 추정은 틀렸다. `infra1`/`infra2`도 발행된다.
- **토픽 이름은 마운트 방식(eye-in-hand/eye-to-hand)과 무관하다** — 마운트를 바꿔도 이름은 그대로고 TF 부모 프레임만 바뀐다. (2026-08-01에 "런치가 정한다"고 적었으나 위 항목대로 **런치와도 무관**하다. 런치가 정하는 건 이름이 아니라 **어떤 TF·스트림을 켜느냐**다 — 그게 4.8GB 사고의 실제 축이었다.)
- `align_depth.enable:=true`일 때 `aligned_depth_to_color`의 해상도는 **depth가 아니라 color 프로파일을 따른다.** 대역폭 계산 시 주의.
