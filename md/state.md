<!-- meta
updated: 2026-08-07 11:05
status:  live
owns:    지금 상태 · 다음 할 일 · 열려 있는 이슈
-->

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

- 파일: `src/graspgenx_perception/graspgenx_perception/{capture_graspgenx_scene,graspgen_worker,grasp_bridge_node}.py`
  + 수동 검증 3종(`test/manual_{capture_scene,grasp_bridge,scene_roundtrip}.py`).
  **2026-08-07 부터 ROS 패키지다** — 이전에는 워크스페이스 `scripts/` 단독 실행이었고
  `setup.py` 가 바깥 경로를 `scripts=[...]` 로 심었다. 지금은 소스가 전부 패키지 안에 있고
  진입점이 `console_scripts` 라 **실행 파일 이름에서 `.py` 가 빠졌다**:
  `ros2 run graspgenx_perception {grasp_bridge_node,capture_graspgenx_scene}`.
  이전↔현재 경로 대조표는 `src/graspgenx_perception/README.md` "graspx 와 함께 띄우기" 절에 있다
  (vault 밖이라 위키링크가 아니라 저장소 경로로 적는다).
- 계약·튜닝값·함정은 [[ws/cobot2/detect_graspx]]가 단일 출처다(출력 규약 §3, 폭 계산·1/10mm 함수 §5, 상류 버그 §6). 실측 사실(체크포인트 sha256, VRAM 측정치)만 [[ws/cobot2/context/constraints]] "GraspGenX 관련"에 있다.
- ✅ **`tool0` → RG2 손끝 실측 완료 (2026-08-07)**: 닫힘 240 mm = 브라켓+퀵커넥터 22 mm +
  그리퍼 자체 218 mm. 개구 70/100 mm 에서 손끝이 17/41 mm 후퇴(힌지 회전, 비선형).
  실측 근거·두 오차 성분 분리는 [[ws/cobot2/context/constraints]] "GraspGenX 관련"이 단일 출처.
  - 폭에 따른 가변 성분은 `pick_fsm/pick_fsm/rg2.py`의 `fingertip_length_m(width_m)`로 배선
    완료 — `tcp_offset_m` 상수(0.18) 대체. `colcon test --packages-select pick_fsm` PASS(21/21).
  - ✅ **고정 오프셋(브라켓 22 mm)도 xacro에 반영함(2026-08-07, 사용자 승인 후 진행)**.
    `onrobot_rg2.xacro`의 `has_bracket` 분기 값을 0.004→0.022로, `m0609_with_rg2.urdf.xacro`가
    그 스위치를 켜도록 수정. 축은 추측 없이 FK 재계산으로 확인(rg2_base_link 접근축 +Z가
    tool0 프레임의 +X로 매핑됨). `colcon build` PASS. **미검증**: 실기 RViz 육안 확인,
    self-collision 경계 재검토 — 로봇 세션 필요.

> 📁 **문서 전체 지도는 [[ws/cobot2/README]]에 있다.** 어느 문서가 무엇의 단일 출처인지 거기서 본다.

## 🟢 cuMotion + nvblox 실기 파이프라인 전 구간 관통 (2026-08-06) — 로봇은 아직 안 움직였다

> 📌 **실행 명령어·노드 지도·단계별 검증은 `config/testcommand.md`가 단일 출처다.**
> 여기서 명령을 다시 적지 않는다.
> ⚠️ 이 문서는 vault(`md/`) 밖에 있어 **위키링크로 못 건다** — 아래 "열려 있는 이슈" 참고.

```
카메라 ─▶ robot_segmenter ─▶ nvblox(esdf_mode:=3d) ─(서비스)─▶ cumotion_planner ─▶ move_group
         (로봇 몸 지움)                                        (read_esdf_world:=True)
```

실기 실측 (로봇+카메라+nvblox 전부 살아 있는 상태, 관절목표, 각 10회):

| | server 중앙값 | 성공 |
|---|---|---|
| OMPL | 42.4 ms | 10/10 |
| cuMotion | 110.6 ms | **10/10** |

cuMotion이 쥔 장애물 복셀 **27,646개**(`/curobo/voxels`) — nvblox 세계가 실제로 실렸다.

🔴 **이 숫자로 cuMotion을 판정하지 말 것.** 관절공간 목표는 OMPL(RRTConnect)에 가장 유리하다.
(a)/(b) 결정 근거는 **장애물이 궤적을 실제로 막는 씬**에서의 재측정이다.

**하루에 걸린 함정 6개(전부 [[ws/cobot2/context/constraints]]에 기록):**
`/joint_states` velocity 필수 · XRDF 구 과대추정 6쌍 · nvblox `esdf_mode` 기본값 2d가 프로세스를
죽임 · **`robot_segmenter_node` 없으면 로봇이 자기 몸을 장애물로 봄** · 이미지 numpy 2.2.6이 cv2를
깸 · `run_dev.sh`가 컨테이너를 새로 만들어 pip 설치 유실(→ `scripts/container_setup.sh`).
공통점: **OMPL은 멀쩡한데 cuMotion만 죽는다.**

- ⛔ **미해결**: ① 장애물이 궤적을 실제로 바꾸는지 미검증(계획 성공까지만) ② XRDF에서
  `link_4 ↔ rg2_base_link` 자기충돌 검사를 껐다 — 실기 모션 전 재검토 필수 ③ 세그멘터 3.7 Hz 병목
  ④ 그리퍼 Modbus 연결 실패 ⑤ depth가 요청 15 Hz 대비 9.65 Hz.

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
  ✅ **Isaac ROS 컨테이너 경로 열림(2026-08-06)** — `kimkh`가 docker 그룹 멤버이고 컨테이너에서
  `nvidia-smi`가 RTX 4060을 본다. 기동: `./run_dev.sh -a "-v $HOME/cobot2_ws:/workspaces/cobot2_ws"`.
  ⚠️ **컨테이너를 새로 만들면 `pip3 install 'warp-lang==1.5.0'`을 다시 해야 한다**(이미지 밖 변경).
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
- 🟡 **`testcommand.md`가 vault 밖에 있다** (2026-08-07). 저장소 루트 → `config/testcommand.md`로
  옮겨졌는데, 이 문서는 **md/ 문서 형식으로 쓰여 있다** — meta 헤더(`status: live`,
  `owns: 실행 명령어 · 노드 지도 · 단계별 검증 명령`)가 있고 본문이
  `[[ws/cobot2/context/constraints]]`·`[[ws/cobot2/plans/2026-08-05-cumotion-bringup]]`을 건다.
  `md/` 밖에 있으면 **세 가지가 동시에 깨진다**: ① 이 문서를 `ws/cobot2/testcommand` 위키링크로
  못 부른다(위 "cuMotion" 절이 그래서 저장소 경로로 바뀌었다) ② 이 문서가 내보내는 위키링크 2개가
  Obsidian에서 안 걸린다 ③ `doc_check.sh`가 md/만 훑으므로 이 문서는 **검사 대상에서 빠진다**
  (meta·문서 지도·live 방치 전부). 정본 해결은 `md/testcommand.md`로 옮기고 `md/README.md`
  문서 지도에 등재하는 것. 옮기지 않기로 정했다면 위 세 가지를 감수하는 결정임을 여기 적어둔다.
- 🔴 **세 계정이 동시 로그인해 같은 ROS 도메인(93)과 같은 GPU를 쓴다** (2026-08-06 실측).
  `joonwon`이 띄운 `move_group`이 내 `ros2 node list`에 그대로 나오고, `kill`은 uid가 달라 실패한다.
  **`ps`에 `user`를 넣지 않으면 남의 프로세스를 내 것으로 착각한다.**
  도메인 분리가 협의된 적이 없다 — 정하지 않으면 이 충돌은 계속된다.
  절차·판별법은 [[ws/cobot2/context/constraints]] "세 계정이 동시에 로그인해…"가 단일 출처.
- ~~GPU PC 도커 경로가 막혀 있다~~ ⤴ **해소.** `kimkh`는 이제 docker 그룹 멤버이고
  (`id`로 확인, 2026-08-06) 컨테이너 안에서 `nvidia-smi`가 RTX 4060을 본다.
  GPU 사양의 단일 출처는 [[ws/cobot2/context/constraints]](RTX 4060 Laptop 8GB).
- ~~D435i depth rosbag 미확보~~ ⤴ **해소(2026-08-04).** `rosbag/bag_0803calibed/` 4개 검증 통과.
  남은 건 장면 2개(빈 테이블 / 장애물 여러 개) 추가 뿐이다 → [[ws/cobot2/rosbag-d435i]] "다시 찍어야 하는 것".
- **카메라 마운트 강성 미확보** — 견고한 고정이 아직 어려움. 캘리브는 **잠정(provisional)**으로 취급하고, Day4 인식 정확도 실측 검증은 마운트 확정 후로 미룬다. 개발용 TF로는 잠정값으로 충분하다.
- **Day4 인식 방식 변경(2026-08-02)**: ray-plane intersection → **FoundationPose**(6D pose), 하드코딩 그립 → **GraspGenX**. 평면 가정이 사라지는 건 이득이지만 **GPU 의존이 커졌다** — Day4는 GPU PC 전용이 된다.
- **C270**은 아직 실기 등록·실행 이력이 없다(`.bashrc` alias만 존재). M0609·RG2·D435i는 실기 확인 완료 → [[ws/cobot2/context/constraints]].
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
   **2026-08-07 코드 감사로 재확인**: 여전히 배선 안 됨(6개 근거·남은 일 3개는
   [[ws/cobot2/plans/2026-08-07-graspgenx-target-matching]]).
   ⚠️ **FoundationPose는 마스크를 만들어주지 않는다** — 마스크를 **입력으로 요구**하고 CAD 메시도 필요하다.
   ROI/마스크 출처는 여전히 검출기(YOLO-seg 또는 RT-DETR)다.
0-c. **RG2 개구 폭(`rgwd`) 계산** — grasp에서 폭을 뽑아 그리퍼 명령으로.
   ~~아직 미구현~~ → **정정(2026-08-06 코드 감사)**: 알고리즘은 이미 있다
   (`corecode/GraspSelection/grasp_selector.py`, 442줄, 테스트 통과). 못 한 건 **배선**이다 —
   `grasp_bridge_node.py`가 이 파일을 안 쓰고 폭 계산 없는 자체 `select()`를 따로 쓴다.
   남은 일(import 교체 · `/grasp/best` width_m 추가 · `OnRobotRGOutput` 퍼블리시)은
   [[ws/cobot2/detect_graspx]] §7-10이 단일 출처.
0-d. **GraspMoE 분기별 점수 캘리브레이션 확인 — 로봇 없이 데이터만으로 된다 (2026-08-07 추가)**
   판별기가 확산 분기(`diff`)와 기하 분기(`obb`)를 같은 척도로 채점한다는 보장이 없다.
   (판별기는 데이터셋 음성·on-policy 음성으로도 학습되지만 — 소스 확인 — OBB 격자 후보가
   그 분포에 있는지는 미확인.)
   **최소 실험**: 순위 1·2위가 `obb`, 3·4위가 `diff`인 **순위 역전** 프레임을 골라, 상위 `obb`
   후보를 `corecode/GraspSelection/grasp_selector.py`의 재충돌 필터에 통과시킨다. 점수 0.9+인데
   충돌로 탈락하면 → **그 분기 점수를 못 믿는다는 증거**다. `branch_tags`는 이미 출력에 있다.
   근거·배경은 [[ws/cobot2/2026-08-07-nvblox-curobo-digest]] §9-2(E).

1. **README 4절 체크1~3 명령 실측** — `tf2_echo base_link camera_link`, `topic hz .../points`,
   `topic echo /dsr01/joint_states`는 **아직 한 번도 실행 안 됨**(README에 ⚠️ 미검증 표기해 둠).
   실기 켠 김에 돌려서 기대값과 대조하고 경고를 지운다.
2. **D435i depth rosbag 장면 2개 추가 녹화**(빈 테이블 / 장애물 여러 개) — 기존 4개(2026-08-04)는 검증 통과.
   ⚠️ **녹화 절차의 단일 출처는 [[ws/cobot2/rosbag-d435i]] §6, 재생 절차는 §3이다.** 아래 "출근 후 D435i 세션"에는 순서만 남겼다.
3. **캘리브 오차 정량 측정** — 알려진 좌표의 물체로 cm 단위. `padding_offset` 해석과 cuRobo 비교의 전제.
   - 🔴 **여기에 오차예산 판정이 걸려 있다 (2026-08-07 추가).** 계통오차는 TSDF 가중평균·베이즈
     갱신으로 **원리적으로 제거되지 않는다**(무작위만 상쇄된다). 합성 규칙은 무작위=RSS,
     계통·양자화=선형 합. 대입하면 `필요 마진 ≳ 40 mm(캘리브) + 25 mm(voxel/2) = 65 mm`인데
     현재 cuRobo `activation_distance`는 **25 mm**다 → **마진 부족 가능성.**
     ⚠️ **계산상의 결론이지 실측이 아니다.** 전제 2개가 미검증: ① 40 mm가 정말 bias인지
     (무작위 성분과 분리 안 됨) ② `activation_distance`가 유일한 마진인지(XRDF 구가 이미
     부풀려져 6쌍이 겹쳤으므로 실효 마진은 더 클 수 있다).
     **이 측정이 그 계산의 입력이다.** 근거·유도는 [[ws/cobot2/2026-08-07-nvblox-curobo-digest]] §9-1.
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

## 실기 안전 경고 (상시 유효)
> 완료된 작업 경위는 [[ws/cobot2/review_moveit]] §0으로 이관했다.
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

> ⚠️ **녹화 런치·토픽 목록·재생 절차는 여기 두지 않는다. 단일 출처는 [[ws/cobot2/rosbag-d435i]] §6(재녹화 절차), 재생은 §3이다.**
> 예전 이 자리에 있던 `rs_align_depth_launch.py` 기반 명령은 **폐기됐다** — 그 런치엔 `camera_calib_tf`가
> 없어 `base_link→camera_link` 없는 bag 4.8GB가 나왔다. 현행은 `camera.launch.py`(= `reals` alias)다.
> 녹화 중 **임시 static TF를 띄우지 말 것**(bag의 `/tf_static`에 가짜 값이 박힌다)만 여기 남긴다.

> 문서 위치 규칙은 [[ws/cobot2/README]] "문서를 쓸 때"가 단일 출처다(중복 삭제).
> 실기 실측 사실(doosan-robot2 launch 기본값, D435i 토픽 네임스페이스 등)은 [[ws/cobot2/context/constraints]]로 이관했다.
