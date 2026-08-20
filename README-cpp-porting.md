# C++ 포팅 가이드 (학습용)

이 ws의 Python(rclpy) 노드 중 일부를 C++(rclcpp)로 옮겨보는 학습 트랙의 색인이다.
**실기 검증 목적이 아니라 언어/API 학습 목적**이고, 포팅한 노드는 원본과 별개의 새
패키지로 존재한다 — 기존 Python 소스는 어떤 것도 수정하지 않는다.

⚠️ 아래 "실기 미검증" 표시가 있는 항목은 `colcon build` PASS까지만 확인했고, 실제
`dsr_controller2`/로봇/카메라가 붙은 상태에서 동작시켜본 적이 없다. 이 문서의 장단점
서술도 코드를 읽고 판단한 **추론**이지, 두 버전을 실측 비교(latency 등)한 결과가 아니다.

## 왜 이 순서인가 — 후보 선정 기준

1. **외부 IO/네트워크/서브프로세스 의존이 적을 것** — C++ 로 바꿔도 이득이 대부분
   "OS 콜을 감싸는 래퍼가 하나 더 생긴다"에 그치는 노드는 후보에서 제외했다.
2. **PyTorch·STT 등 Python 전용 생태계에 묶여 있지 않을 것** — 있으면 C++ 포팅이
   아니라 "완전히 다른 스택(LibTorch/PortAudio 등)으로 재작성"이 된다.
3. **한 노드가 200줄 안팎일 것** — 학습 세션 하나에서 구조 전체를 눈으로 따라갈 수
   있는 크기. 이보다 크면 포팅이 아니라 며칠짜리 프로젝트가 된다.

## 포팅 후보와 판단 근거

| 노드 (줄 수) | 원본 위치 | C++로 바꾸면 살아나는 것 | 상태 |
|---|---|---|---|
| `robot_safety_node.py` (203) | `src/pick_fsm/pick_fsm/robot_safety_node.py` | 비동기 서비스 클라이언트(콜백 등록 vs 수동 폴링), 서비스 필드 타입 불일치를 컴파일 타임에 잡아주는 사례 | ✅ 포팅 완료 → [`src/robot_safety_cpp/`](src/robot_safety_cpp/) |
| `planned_tcp_path_node.py` (126) | `src/pick_fsm/pick_fsm/planned_tcp_path_node.py` | ~~Eigen~~ → 정정: 행렬 연산은 없다. 실제 핵심은 **fan-out/fan-in 비동기 패턴**(waypoint마다 `/compute_fk` 요청을 동시에 N개 보내고, 세대 번호로 stale 응답을 버리며, pending 카운터가 0이 되면 합쳐서 발행) | ✅ 포팅 완료 → [`src/planned_tcp_path_cpp/`](src/planned_tcp_path_cpp/) |
| `depth_downsample_node.py` (133) | `src/cobot_rg2/rg2/m0609_rg2_bringup/scripts/depth_downsample_node.py` | ~~PCL~~ → 정정: 포인트클라우드가 아니라 **depth 이미지**를 `cv2.resize`(INTER_NEAREST)로 다운샘플하고 카메라 intrinsics(K/P 행렬)를 스케일한다. C++에서는 `cv_bridge`+OpenCV C++ API(`cv::resize`, `cv::Mat`)로 옮기는 것이 자연스럽다 — PCL이 아니라 **OpenCV 네이티브 사용**과 이미지 처리 성능 비교가 학습 포인트 | ✅ 포팅 완료 → [`src/depth_downsample_cpp/`](src/depth_downsample_cpp/) |
| `gripper_virtual_node.py` (110) | `src/cobot_rg2/rg2/m0609_rg2_bringup/scripts/gripper_virtual_node.py` | `MultiThreadedExecutor` + `ReentrantCallbackGroup` 위에서 타이머 콜백과 서비스 콜백이 **실제로 다른 스레드에서 동시 실행**되는 구조. 서비스 콜백이 목표 도달까지 블로킹 폴링하는 동안 타이머 콜백이 별도 스레드에서 값을 계속 갱신해줘야 한다 — `std::mutex`+`std::lock_guard` 스코프를 좁게 잡는 이유를 몸으로 배우기 좋음 | ✅ 포팅 완료 → [`src/gripper_virtual_cpp/`](src/gripper_virtual_cpp/) |

## 후보에서 제외한 것과 이유

| 노드 (줄 수) | 제외 이유 |
|---|---|
| `yolo_seg_node.py` (458) | PyTorch 의존. C++로 가려면 LibTorch/TensorRT로 스택 자체를 바꿔야 해서 "포팅"이 아니라 별도 프로젝트가 된다 |
| `grasp_bridge_node.py` (562) | `subprocess`/파일시스템 오케스트레이션이 핵심 로직이라, C++로 바꿔도 결국 `<cstdlib>`/`<filesystem>` 래퍼가 하나 더 생기는 정도 — 언어 특성이 드러나는 지점이 적다 |
| `approve_listener_node.py` (233) | `pyaudio`/STT 라이브러리 의존. IO-bound(마이크 입력 대기)라 C++의 성능 이점이 드러날 자리가 없고, PortAudio로 바꾸는 순간 별도 스택 학습이 된다 |
| `vla_command_node.py` (896) | 외부 VLA API 호출이 핵심 — 네트워크 IO-bound, 줄 수도 커서 "학습용 포팅" 범위를 넘어선다 |
| `task_manager.py` (1388) | pick_fsm의 핵심 상태머신. 이론적으로는 포팅 가능하지만 `pick_fsm` 패키지 전체를 `ament_python`→`ament_cmake`로 옮겨야 하고 분량도 커서, 개별 노드 학습이 아니라 패키지 마이그레이션 프로젝트가 된다 |

## 포팅한 파일들의 위치

```
cobot2_ws/
├── README-cpp-porting.md          ← 이 파일 (색인)
├── src/
│   ├── pick_fsm/                  ← 원본 Python 노드들 (수정 안 함)
│   │   └── pick_fsm/{robot_safety_node.py, planned_tcp_path_node.py}
│   ├── cobot_rg2/rg2/m0609_rg2_bringup/scripts/
│   │   └── {depth_downsample_node.py, gripper_virtual_node.py}   ← 원본 (수정 안 함)
│   ├── robot_safety_cpp/          ← C++ 포팅 #1 (완료)
│   │   ├── README.md              ← rclpy→rclcpp 상세 설명, 검증 상태 표
│   │   ├── package.xml / CMakeLists.txt
│   │   └── src/robot_safety_node.cpp
│   ├── planned_tcp_path_cpp/      ← C++ 포팅 #2 (완료, Codex 위임)
│   │   ├── README.md
│   │   ├── package.xml / CMakeLists.txt
│   │   └── src/planned_tcp_path_node.cpp
│   ├── depth_downsample_cpp/      ← C++ 포팅 #3 (완료, Codex 위임)
│   │   ├── README.md
│   │   ├── package.xml / CMakeLists.txt
│   │   └── src/depth_downsample_node.cpp
│   └── gripper_virtual_cpp/       ← C++ 포팅 #4 (완료, Codex 위임)
│       ├── README.md
│       ├── package.xml / CMakeLists.txt
│       └── src/gripper_virtual_node.cpp
```

`planned_tcp_path_cpp`/`depth_downsample_cpp`는 `~/vault/ai/MULTI_AGENT_POLICY.md` 절차대로
Task Owner를 Codex(`gpt-5.6-sol`, medium effort)로 위임해 구현했고, Claude(이 세션)가 diff
기준 독립 리뷰 + clean rebuild(`build/install` 삭제 후 재빌드)로 재검증했다. 원본 Python
무변경도 `git diff` 결과로 확인함(같은 turn 도구 출력 — "검증됨").

새 노드를 포팅할 때마다 `src/<노드이름>_cpp/`로 별도 `ament_cmake` 패키지를 만들고,
이 표의 "상태" 칸과 위 트리를 갱신하는 방식으로 진행한다.

## 공통 규칙

- 원본 Python 소스는 절대 수정하지 않는다 — 대조 기준으로 남겨둔다.
- 포팅한 패키지의 `README.md`에는 반드시 **검증 상태**(빌드만 확인 vs 실기 확인)를
  표로 구분해 적는다. "실기에서 이렇게 동작한다"고 단정하지 않는다.
- 실기(로봇) 명령이 걸린 노드는 빌드까지만 하고 실행은 사용자가 한다
  (`~/.claude/CLAUDE.md` 0절 — 실기 안전 규칙, 이 문서로 예외를 만들지 않는다).
