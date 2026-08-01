# 세션 상태

> 현재 상태로 덮어쓴다. 로그처럼 쌓지 않는다.

**최종 갱신:** 2026-08-01

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
- **GPU PC에서 `nvidia-smi`, `docker info | grep -i runtime` 미확인** — 실패하면 스프린트 계획 전체 재작성 필요.
- 하드웨어(M0609 + RG2 + D435i + C270)는 `.bashrc` alias 추론이며 실기로 재확인되지 않음.
- **D435i depth rosbag 미확보** — 이게 있어야 개인PC에서 실기 없이 Octomap·플래너·상태머신 개발 가능. 절차는 아래 "출근 후 D435i 세션" 참조.
- **카메라 마운트 강성 미확보** — 견고한 고정이 아직 어려움. 캘리브는 **잠정(provisional)**으로 취급하고, Day4 ray-plane 실측 정확도 검증은 마운트 확정 후로 미룬다. 개발용 TF로는 잠정값으로 충분하다.
- `dsr_moveit_config_m0609/config/sensors_3d.yaml`이 `sensors: []`로 비어 있음 — 채워야 MoveIt Octomap 연동됨.

## 출근 후 D435i 세션 (순서 고정)

**rosbag → 캘리브 순서로 한다.** 캘리브가 그날 제일 잘 깨지는 단계라, 먼저 하다 실패하면 빈손이 된다.
캘리브 결과는 static TF 6개라서 bag 재생 시 `static_transform_publisher`로 나중에 얹을 수 있다 — depth 데이터 자체는 캘리브와 무관하게 유효하다.

1. 카메라를 최종 위치에 **최대한 고정** + 마스킹테이프 표시 + 사진 (재현용)
2. `ros2 topic list`로 토픽 이름 재확인 (아래는 2026-08-01 확인분)
3. **rosbag 녹화** ← 여기서 개인PC 작업이 풀린다
4. **eye-to-hand 캘리브** — 3~4 사이에 카메라를 건드리지 않는다 (건드리면 bag과 짝이 안 맞음)
5. `~/.ros/easy_handeye2/*.calib` → `config/handeye/`에 복사해 커밋 (파일명에 `_provisional`)

### 녹화용 런치 (운용 설정과 다름 — 의도적)
```bash
ros2 launch realsense2_camera rs_align_depth_launch.py \
  depth_module.depth_profile:=848x480x30 \
  rgb_camera.color_profile:=848x480x30 \
  initial_reset:=true \
  align_depth.enable:=true \
  pointcloud.enable:=false \
  enable_rgbd:=false
```
- **color를 848x480으로 낮추는 이유**: `align_depth`는 depth를 **컬러 해상도로 리샘플**한다. color가 1280x720이면 aligned depth가 55MB/s(3.3GB/분)까지 뛴다. 맞추면 24MB/s. Octomap·플래너 개발에 720p depth는 과잉.
- `pointcloud.enable:=false`: `/depth/color/points`는 ~390MB/s이고 depth+camera_info로 언제든 재생성되는 파생물이다.
- `enable_rgbd:=false`: 개별 토픽을 다 녹화하므로 합본 메시지는 중복.
- `initial_reset:=true`: USB 인식 꼬임 방지 — 유지.

### 녹화 명령
```bash
ros2 bag record --compression-mode file --compression-format zstd \
  -o d435i_$(date +%m%d_%H%M) \
  /camera/camera/depth/image_rect_raw \
  /camera/camera/depth/camera_info \
  /camera/camera/aligned_depth_to_color/image_raw \
  /camera/camera/aligned_depth_to_color/camera_info \
  /camera/camera/color/image_raw/compressed \
  /camera/camera/color/camera_info \
  /camera/camera/extrinsics/depth_to_color \
  /tf /tf_static
```
- 제외: `depth/color/points`(파생물·초대용량), `*/theora`·`*/compressedDepth`(재생 시 디코드 실패 잦음), `*/metadata`, color raw
- `color/camera_info` 필수 — 없으면 compressed 컬러를 못 쓴다
- 약 50MB/s → zstd 후 1.5~2GB/분. **60초 × 5장면**(빈 테이블 / 장애물 1개 / 장애물 여러 개 / 사람 손 진입 / 로봇 동작 중)이 10분 연속 1개보다 낫다
- 녹화 중 **카메라→base 임시 static TF를 띄우지 말 것** — bag의 `/tf_static`에 가짜 값이 박히면 나중에 진짜 캘리브 값과 충돌한다
- USB로 이동. git 금지(`*.db3`/`*.mcap` 이미 ignore)

## 문서 위치 규칙
- 작업 문서는 **`md/` 한 곳만** 쓴다 (커밋됨). `docs/`는 PDF 서고 전용이며 ignore.
- 2026-08-01: `docs/`에 있던 `state.md`·`context/constraints.md` 낡은 사본 삭제. ignore된 위치에 문서가 있으면 git이 갱신 누락을 잡아주지 못한다.

## 이 ws에서 확인된 사실 (실측)
- **doosan-robot2 launch의 `model` 기본값이 `m1013`** — M0609 쓸 때마다 `model:=m0609` 명시 필요. `dsr_bringup2_{rviz,gazebo,mujoco,moveit}.launch.py` 모두 해당.
- 시뮬 경로 3종 존재: virtual 모드(DRCF 에뮬레이터, `install_emulator.sh` 선행 필요), Gazebo(`dsr_gazebo2`), MuJoCo(`dsr_mujoco`).
- RealSense D435I 도메인/지터 이슈는 [[ws/cobot2/context/constraints]]에 기록.
- **D435i 토픽 네임스페이스는 `/camera/camera/...`** (2026-08-01 `ros2 topic list` 실측). 계획서 초안의 `/d435i/...`는 오기다. 토픽 이름은 **런치 명령이 정하지 마운트 방식(eye-in-hand/eye-to-hand)이 정하지 않는다** — 마운트를 바꿔도 이름은 그대로고 TF 부모 프레임만 바뀐다.
- `align_depth.enable:=true`일 때 `aligned_depth_to_color`의 해상도는 **depth가 아니라 color 프로파일을 따른다.** 대역폭 계산 시 주의.
