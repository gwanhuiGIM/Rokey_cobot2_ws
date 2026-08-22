# cobot2_ws

Doosan M0609 + OnRobot RG2 + RealSense D435i 로 하는 vision-guided pick 실험 워크스페이스. 개인 개발 진행용이라 **파이프라인이 끝까지 이어져서 검증된 상태가 아니다** — 아래 "미완/공백" 참고.

## 구성 (src/)

| 패키지 | 역할 |
|---|---|
| `cobot_rg2` | 로봇+그리퍼+카메라 bringup, MoveIt 설정 (doosan-robot2/onrobot-ros2는 외부, read-only) |
| `cumotion` | 실행 중 재계획으로 동적 장애물 회피 (옵션 경로, GPU 컨테이너 필요) |
| `graspgenx_perception` | YOLO 인식 + GraspGenX 파지 계산 |
| `pick_fsm` | 음성/타겟 지시로 도는 pick 상태머신 |
| `voice_processing` | 음성/VLA 지시 입력 → `/get_keyword` |
| `object_detection`, `depth_downsample_cpp`, `gripper_virtual_cpp`, `planned_tcp_path_cpp`, `robot_safety_cpp`, `pick_fsm_msgs` | 위 패키지들이 쓰는 보조/메시지 패키지 |
| `cumotion` 은 실제 코드는 이 ws에 있지만 **공유 랩탑의 다른 사람 소유 패키지도 `src/`에 섞여 있을 수 있다** — 손대기 전에 항상 확인 |

## 전체 흐름 (의도)

```
[음성/VLA 지시] → voice_processing → pick_fsm(상태머신)
                                         ├─ graspgenx_perception (인식+파지계산)
                                         └─ cobot_rg2 (bringup) + MoveIt(OMPL 또는 cumotion) → 실기 이동
```

## 실행

각 패키지의 실행 명령·파라미터·인터페이스는 `src/PACKAGES.md`(단일 참조 문서)에 있다. 날짜별 실기 디버깅 로그·의사결정은 `md/`, 세션 상태는 `docs/state.md`.

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
colcon build --symlink-install
```

## 미완/공백 (개인 개발 진행 중이라 명시)

- 전체 파이프라인(음성 지시 → 인식 → 파지 → 실기 이동)을 처음부터 끝까지 한 번에 실기로 통과시킨 기록이 없다 — 구간별로는 검증됐어도 이어붙인 end-to-end 검증은 아직이다.
- `cumotion`(동적 회피 경로)은 `docs/state.md`, `src/PACKAGES.md`의 "아직 안 된 것" 절 참고 — 실기로 안 돌려본 튜닝값 다수, 실행 중 동적 회피 자체가 미완성 단계.
- git remote/브랜치 이력에 사고가 한 번 있었다(공유 remote 오염) — 현재 `Rokey_cobot2_ws` remote로 정리됐지만 과거 브랜치 잔재가 로컬에 남아있을 수 있다. 자세한 경위는 `docs/state.md`.
- 상세 실행 절차의 상당수 항목이 "⚠️ 미검증" 표시가 붙어 있다 — `src/PACKAGES.md` 참고 시 표시를 그대로 신뢰할 것(검증됐다고 단정하지 않는다).
