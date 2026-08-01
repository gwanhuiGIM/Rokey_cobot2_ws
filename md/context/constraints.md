# 실기/현장 제약 (실측 사실만 기록. 추측 금지)

## RealSense D435I (2026-08-01)
- alias `reals` (`rs_align_depth_launch.py` + `align_depth.enable:=true enable_rgbd:=true pointcloud.enable:=true`, depth 848x480x30 / color 1280x720x30)는 **ROS_DOMAIN_ID를 지정하지 않으므로 기본값(비어있음 → 0)에서 뜬다.**
- `.bashrc`의 `rdm` alias는 `ROS_DOMAIN_ID=93`을 설정한다. 카메라(도메인 0)와 rqt/다른 터미널(도메인 93)이 섞이면 rqt에 토픽이 전혀 안 보인다 — 실측으로 확인(도메인 93에서 `ros2 topic list`는 `/parameter_events`, `/rosout`만 나옴).
- CPU는 병목이 아님: `align_depth+enable_rgbd+pointcloud` 동시 실행 중에도 `load average 0.5~0.6`, realsense 노드 CPU 18.8%. 그런데도 `/camera/camera/color/image_raw`, `/rgbd`, `/depth/color/points`의 `ros2 topic hz`가 최소 간격(~0.031~0.033s, 30fps대)은 정상이면서 가끔 0.2~0.3s까지 벌어져 누적 평균이 계속 떨어짐 — USB/드라이버 쪽 간헐적 프레임 드롭으로 추정(미검증, CPU 원인은 배제됨).

## 카메라·로봇 하드웨어 (미검증 — alias 추론일 뿐)
- `.bashrc` alias(`br`, `hom`)로 미루어 로봇은 M0609(네임스페이스 `/dsr01`, IP `192.168.1.100`), 그리퍼는 OnRobot RG2로 보이나 실기로 확인된 사실 아님.
