# pick_and_place_voice

음성 명령("Hello Rokey" → "hammer를 pos1에 놔줘")으로 Doosan M0609 + OnRobot RG2가
YOLO로 찾은 공구를 집어 오는 rokey 교육용 패키지.

하나의 ament_python 패키지 안에 파이썬 모듈 3개(`robot_control`, `object_detection`,
`voice_processing`)가 들어 있다. 같은 이름의 **독립 ROS 패키지**(`src/robot_control`,
`src/object_detection`, `src/voice_processing`)가 이 ws에 따로 있으므로 혼동 주의.

## 노드 구성

```
get_keyword (Trigger 서버)        robot_control (클라이언트, 메인 루프)
   마이크 → 웨이크워드 → STT           │ 1) /get_keyword 호출 → "hammer wrench"
   → GPT-4o 키워드 추출                │ 2) 단어마다 /get_3d_position 호출
        ▲ /get_keyword               │ 3) 카메라 좌표 → 베이스 좌표 변환
        └──────────────────────────────┤ 4) movel → 그리퍼 close → open
                                      ▼ /get_3d_position
                          object_detection (SrvDepthPosition 서버)
                             RealSense 구독 → YOLO → 픽셀+depth → 카메라 좌표
```

## 실행

빌드:
```bash
source /opt/ros/humble/setup.bash && \
colcon build --symlink-install --packages-select pick_and_place_voice
```

터미널 4개 (매번 source를 같은 줄에 붙인다):
```bash
# 1. 로봇 + 그리퍼 + RealSense 브링업
source /opt/ros/humble/setup.bash && source install/setup.bash && \
ros2 launch m0609_rg2_bringup bringup_camera.launch.py mode:=real host:=192.168.1.100

# 2. 객체 탐지 서버
source /opt/ros/humble/setup.bash && source install/setup.bash && \
ros2 run pick_and_place_voice object_detection

# 3. 음성 키워드 서버
source /opt/ros/humble/setup.bash && source install/setup.bash && \
ros2 run pick_and_place_voice get_keyword

# 4. 로봇 제어 (★ 실기 동작. 사람이 지켜보는 상태에서만 실행)
source /opt/ros/humble/setup.bash && source install/setup.bash && \
ros2 run pick_and_place_voice robot_control
```

음성 없이 키워드 서버만 따로 두드려 볼 때:
```bash
ros2 service call /get_keyword std_srvs/srv/Trigger "{}"
```

## 서비스 / 토픽

| 이름 | 타입 | 방향 | 노드 |
|---|---|---|---|
| `/get_keyword` | `std_srvs/srv/Trigger` | 서버 | get_keyword |
| | | 클라이언트 | robot_control |
| `/get_3d_position` | `od_msg/srv/SrvDepthPosition` | 서버 | object_detection |
| | | 클라이언트 | robot_control |
| `/camera/camera/color/image_raw` | `sensor_msgs/Image` | 구독 | object_detection |
| `/camera/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | 구독 | object_detection |
| `/camera/camera/color/camera_info` | `sensor_msgs/CameraInfo` | 구독 | object_detection |

`SrvDepthPosition`: `string target` → `float64[] depth_position` (카메라 좌표계 x,y,z mm).
검출 실패 시 `[0,0,0]`을 돌려주고 robot_control이 이를 "타깃 없음"으로 처리한다.

`/get_keyword` 응답의 `message`는 공백으로 구분된 공구 이름 문자열(`"hammer wrench"`).
LLM은 `도구 / 목적지` 형식으로 답하지만 현재 코드는 **도구만 사용하고 목적지는 버린다**
([get_keyword.py:135](voice_processing/get_keyword.py#L135)).

## 파라미터 (전부 소스 상수 — `declare_parameter()` 미사용)

| 상수 | 값 | 위치 | 비고 |
|---|---|---|---|
| `ROBOT_ID` / `ROBOT_MODEL` | `dsr01` / `m0609` | robot_control.py:18 | |
| `VELOCITY`, `ACC` | 60, 60 | robot_control.py:20 | |
| `TOOLCHARGER_IP:PORT` | `192.168.1.1:502` | robot_control.py:24 | 그리퍼 컴퓨트박스 |
| `DEPTH_OFFSET` | **-5.0 mm** | robot_control.py:26 | 하드웨어 보정값. 지우지 말 것 |
| `MIN_DEPTH` | **2.0 mm** | robot_control.py:27 | 하한 클램프 |
| `BUCKET_POS`, `JHOME_POS` | 관절값 | robot_control.py:21 | 현재 주석 처리된 경로에서만 쓰임 |
| `device_index` | 10 | get_keyword.py:110 | 마이크. `MicController`가 현재 이 값을 stream에 넘기지 않음(주석 처리) |
| 웨이크워드 threshold | 0.1 | wakeup_word.py:26 | |
| STT / LLM | `whisper-1` / `gpt-4o` | stt.py:29, get_keyword.py:51 | |

## 필요한 리소스 (★ 3개 중 3개가 저장소에 없다)

`share/pick_and_place_voice/resource/` 에 있어야 실행된다:

| 파일 | 현재 상태 | 없으면 |
|---|---|---|
| `.env` (`OPENAI_API_KEY=...`) | **없음** (gitignore) | get_keyword가 인증 실패 |
| `yolov8n_tools_0122.pt` | **없음** | object_detection이 import 시점에 실패 |
| `T_gripper2camera.npy` | **없음** (`src/pick_and_place_text/resource/`에는 있음) | robot_control이 좌표 변환에서 실패 |
| `hello_rokey_8332_32.tflite` | 있음 | |
| `class_name_tool.json` | 있음 (drill/hammer/pliers/screwdriver/wrench) | |

`T_gripper2camera.npy`는 캘리브레이션 산출물이라 **다른 패키지에서 복사해 오면 안 된다** —
이 ws의 실제 카메라 장착 상태로 다시 캘리브레이션(`corecode/Calibration_Tutorial/`)한다.

LLM 프롬프트의 도구 리스트(`hammer, screwdriver, wrench`)와 YOLO 클래스
(`drill, hammer, pliers, screwdriver, wrench`)가 **서로 다르다**. `drill`/`pliers`는
음성으로 지시해도 LLM이 걸러낸다.

## 검증 결과 (2026-07-30)

| 항목 | 결과 |
|---|---|
| `colcon build --symlink-install --packages-select pick_and_place_voice` | **PASS** (setuptools deprecation 경고만) |
| 노드 실행 | **미검증** — 위 리소스 3개가 없어 실행 불가 |
| 실기 동작 (로봇/그리퍼/카메라) | **미검증** — 이 ws에서 실기로 돌린 적 없음 |
| 마이크 `device_index=10` | **미검증** — 호스트마다 다르다. `python3 -m sounddevice`로 확인 |
| 로봇 IP `192.168.1.100`, 그리퍼 `192.168.1.1` | **미검증** — cobot1_ws README에서 온 값 |

`.env`가 없어도 이 패키지는 빌드된다(`glob.glob('resource/.env')`가 빈 리스트라 무시됨).
`.env` 부재로 빌드가 깨지는 건 별개 패키지인 `src/voice_processing` 쪽이다.

## 알려진 문제

- `package.xml`에 런타임 의존성이 하나도 없다 — `rclpy`, `std_srvs`, `sensor_msgs`,
  `od_msg`, `cv_bridge`가 `<exec_depend>`로 빠져 있어 `rosdep`이 무의미하다.
- `robot_control.main()`은 `while rclpy.ok(): node.robot_control()` — 예외 없이 무한
  반복하고, `robot_control()` 안에서 `spin_until_future_complete`로 블로킹한다
  (공통 규칙의 "블로킹 서비스 호출 금지" 위반). 교육용 원본 코드라 그대로 두었다.
- `get_keyword()`의 웨이크워드 대기가 `while not ...: pass` busy-wait이라 코어 하나를
  100% 태운다.
- `pick_and_place_target()` 안에 물건을 버킷으로 옮기는 코드가 주석 처리돼 있다
  ([robot_control.py:159-173](robot_control/robot_control.py#L159-L173)). 현재는 집은
  자리에서 바로 놓는다.
