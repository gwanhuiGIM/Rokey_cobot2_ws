# CLAUDE.md — cobot2_ws

> 공통 규칙(빌드 게이트·셸·금지 규칙·패키지 완성 정의·응답 계약·문서 규칙)은 `~/.claude/CLAUDE.md`에 있다. 여기엔 이 ws에서만 참인 것만 적는다.

## 1. 현재 상태 (2026-07-30)
- git 초기화됨, `.gitignore` 있음. `src/`에 rokey 교육용 패키지(object_detection, od_msg, pick_and_place_*, robot_control, voice_processing, rokey) + `cobot_rg2` 스택.
- `cobot_rg2`는 cobot1_ws에서 `cp -a`로 옮겨온 **중첩 git 저장소**(origin: `github.com/ahnisinc/cobot_rg2`). cobot1_ws와 동일하게 outer repo에서 gitlink로 잡힌다. 이 안의 파일을 고칠 때는 `src/cobot_rg2`에서 별도로 커밋한다.
  - 옮겨올 때 이미 있던 로컬 수정: `dsr_example` 2개, `OnRobotRGControllerServer.py`, `m0609_rg2_bringup/rviz/default.rviz` — upstream과 다르니 `git diff`로 확인하고 지우지 않는다.
- 빌드: `colcon build --symlink-install --packages-skip voice_processing` → 34개 PASS (2026-07-30).
  - `voice_processing`은 `resource/.env`(gitignore된 API 키 파일)가 없어서 FAIL한다. 이 실패가 colcon 전체를 Abort시켜 `m0609_rg2_bringup`까지 빌드가 안 되므로, `.env`를 만들기 전까지는 `--packages-skip voice_processing`을 쓴다.

## 2. 환경
- ROS 2 Humble / Ubuntu 22.04 / Python 3.10 (cobot1_ws와 동일 호스트)
- 하드웨어 (cobot_rg2 스택 기준, **실기 미검증**): Doosan M0609 + OnRobot RG2 + RealSense.
  - 로봇 IP `192.168.1.100`, 그리퍼 컴퓨트박스 `192.168.1.1` — cobot1_ws README 값이므로 이 ws의 실기에 붙이기 전 확인한다.
  - real 모드는 `sudo sysctl -w net.ipv4.ip_unprivileged_port_start=0`, virtual 모드 motion service는 DRCF 에뮬레이터(Docker) 필요. 상세는 `src/cobot_rg2/README.md`.

## 3. cobot1_ws에서 가져올 것 / 가져오지 말 것
- **가져온다**: `~/cobot1_ws/CLAUDE.md` 3절(실기 검증 사실)은 **같은 하드웨어를 쓸 때만** 유효하다. 로봇/그리퍼가 다르면 그 사실들은 무효이므로 복사하지 말고 다시 실측한다.
- **가져오지 않는다**: cobot1_ws의 `src/` 코드를 복사해 오기 전에 네임스페이스·토픽·툴 무게 프리셋 의존성을 확인한다. 특히 힘 기반 노드는 그리퍼 자중 보정에 의존한다.

## 4. 패키지 지도
- `src/cobot_rg2/rg2/m0609_rg2_bringup` — 커스텀 통합 브링업. `bringup.launch.py`(그리퍼) / `bringup_camera.launch.py`(+RealSense). 인자: `mode`(virtual|real), `host`, `port`.
- `src/cobot_rg2/rg2/m0609_rg2_moveit` — MoveIt 설정
- `src/cobot_rg2/doosan-robot2`, `src/cobot_rg2/onrobot-ros2` — 외부 패키지, read-only 취급
- `src/{object_detection,od_msg,pick_and_place_text,pick_and_place_voice,robot_control,voice_processing,rokey}` — rokey 교육용 패키지
- `src/usb_cam` — 외부 패키지(vendored, git 미추적). `launch/camera.launch.py`의 `CAMERAS` 리스트에 `CameraConfig` 추가하면 노드가 자동 생성된다. config: `params_1.yaml`(=`/dev/video0`), `params_2.yaml`(=`/dev/video2`).

## 5. 실기로 확인한 사실 (usb_cam, 2026-07-30)
- USB 웹캠 2대: `/dev/video0` = LG HD WebCam(usb-...-7), `/dev/video2` = Logitech C270(usb-...-6). 둘 다 MJPG/YUYV 640x480 지원. `/dev/video1`, `/dev/video3`은 같은 카메라의 metadata 노드다.
- **프레임레이트가 30이 아니라 ~15 Hz인 원인 = 자동노출 (대역폭 아님).** ROS 없이 `v4l2-ctl --stream-mmap`로 실측: 자동노출(`auto_exposure=3`, Aperture Priority) 상태에서 MJPG 640x480 = **14 fps**, YUYV 640x480 = **10 fps**. `v4l2-ctl -c auto_exposure=1 -c exposure_time_absolute=100`으로 수동 고정하면 같은 조건에서 **30.2 fps**. 어두운 실내라 노출시간이 길어져 드라이버가 프레임을 버리는 것. → **30 Hz가 필요하면 조명을 밝게 하거나 노출을 수동 고정한다.**
- **`params_*.yaml`의 `autoexposure` / `exposure` / `autofocus` / `auto_white_balance` 파라미터는 이 커널에서 무시된다.** `usb_cam`은 `popen("v4l2-ctl -c <name>=<v>")`로 컨트롤을 쓰는데(`src/usb_cam.cpp:701`), 쓰는 이름이 구버전(`exposure_auto`, `focus_auto`, `white_balance_temperature_auto`)이다. 커널 5.x+는 `auto_exposure`, `focus_automatic_continuous`, `white_balance_automatic`으로 개명 → 로그에 `unknown control '...'`만 찍히고 **조용히 실패**한다. 노출을 바꾸려면 노드 실행 전에 `v4l2-ctl`로 직접 설정한다.
  - 실제로 먹히는 yaml 파라미터: `brightness`, `contrast`, `saturation`, `sharpness`(이름이 안 바뀜), `video_device`, `image_width/height`, `pixel_format`, `framerate`, `io_method`, `frame_id`, `camera_name`, `camera_info_url`.
  - LG HD WebCam에는 `focus_*`, `gain` 컨트롤 자체가 없다(고정초점). `v4l2-ctl -d /dev/video0 -l`로 확인.
- **`throw char*` abort의 원인 = C270(`/dev/video2`)이 USB에서 간헐적으로 빠진다.** 로그에 `Device specified is not available ... /dev/video2` + `Available V4L2 devices are: /dev/video0, /dev/video1`만 남고 `/dev/video2`가 사라진다. 코드 버그가 아니라 **케이블/허브/전원 문제**. 이때 `/dev/video0`(다른 카메라)까지 select timeout(`src/usb_cam.cpp:643`)으로 동반 abort한다. → 재현되면 C270 케이블·포트를 먼저 의심한다.
- `/dev/videoN` 번호는 재연결 시 바뀐다. 고정하려면 `/dev/v4l/by-id/usb-Generic_LG_HD_WebCam_200901010001-video-index0` 같은 by-id 심볼릭 링크를 `video_device`에 쓴다.
- **compressed 토픽은 기본으로 이미 켜져 있다.** `image_transport`가 `image_raw` 옆에 자동 광고하고, `compressed_image_transport`도 설치되어 있다. launch의 remapping 때문에 이름이 `<name>/image_compressed`(표준 `image_raw/compressed`가 아님)다. 실측: raw 15.0 MB/s vs compressed 0.75 MB/s (**약 20배 절감**), 둘 다 ~16 Hz. 구독자가 없으면 인코딩을 안 하므로 CPU 비용은 0.
- camera2의 `[test_camera2] does not match test_camera` 경고: `camera_info.yaml`을 두 대가 공유 → **camera2 intrinsics는 틀린 값**이다.

## 6. 채워야 할 항목
- [ ] cobot_rg2 스택 실기 확인 (아직 없음 — 이 ws에서 실기로 돌린 적 없다)
- [ ] `voice_processing/resource/.env`
- [ ] 검증 절차 (`scripts/verify.sh`를 쓸지 — 필요하면 `~/cobot1_ws/scripts/verify.sh`를 복사)
