# planned_tcp_path_cpp

`pick_fsm/pick_fsm/planned_tcp_path_node.py`를 C++(`rclcpp`)로 옮긴 **학습용** 패키지다.
원본 Python 파일은 수정하지 않았으며, 같은 ROS interface를 쓰는 독립 노드다. 두 버전을
동시에 실행하면 같은 Marker를 서로 덮어쓸 수 있으므로 하나만 실행한다.

## 검증 상태

MoveIt 서버나 실제 로봇에 연결하지 않았다. 확인한 범위와 남은 범위는 다음과 같다.

| 확인함 | 확인 안 함 |
|---|---|
| 설치된 Humble 헤더에서 `GetPositionFK` request/response 필드와 `SUCCESS` 상수 확인 | `/compute_fk`가 실제로 동시 요청 N개를 처리하는 runtime 동작 |
| Python 원본과 토픽·파라미터·Marker 값 및 generation/fan-in 로직 대조 | MoveIt 실행 중 Marker가 RViz에 표시되는지 |
| `colcon build --symlink-install --packages-select planned_tcp_path_cpp` | simulation 검증 |
| `ros2 pkg executables planned_tcp_path_cpp`에서 실행 파일 등록 확인 | 실제 로봇/실기 검증 |

즉 보장 범위는 **interface를 유지한 코드가 컴파일되고 executable로 등록되는 것**까지다.
simulation과 hardware 동작은 미검증이다.

## 실행

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run planned_tcp_path_cpp planned_tcp_path_node
```

파라미터를 바꾸는 예:

```bash
ros2 run planned_tcp_path_cpp planned_tcp_path_node --ros-args \
  -p base_frame:=base_link -p tip_link:=tool0 -p downsample:=3
```

## ROS interface

| 종류 | 이름 | 타입 / 기본값 |
|---|---|---|
| 구독 | `/move_group/display_planned_path` | `moveit_msgs/msg/DisplayTrajectory` |
| client | `/compute_fk` | `moveit_msgs/srv/GetPositionFK` |
| 발행 | `/pick/planned_tcp_path` | `visualization_msgs/msg/Marker` (`LINE_STRIP`) |
| parameter | `base_frame` | `base_link` |
| parameter | `tip_link` | `tool0` |
| parameter | `downsample` | `1` (최솟값 1로 clamp) |

Marker는 원본과 같이 `ns="planned_tcp_path"`, `id=0`이다. 새 plan은 이전 line을 같은
`ns/id`로 덮어쓴다. 성공한 FK point가 2개 미만이면 Marker를 발행하지 않는다.

## fan-out / fan-in 흐름

1. 첫 번째 `RobotTrajectory`의 joint waypoint를 `downsample` 간격으로 고르고 마지막 점을
   항상 포함한다.
2. 선택한 waypoint마다 `/compute_fk` 요청을 기다리지 않고 연속 전송한다(fan-out).
3. 모든 response callback은 하나의 `BatchState`를 공유해 자기 index의 결과를 기록하고
   `pending`을 1씩 줄인다.
4. 현재 generation의 마지막 response가 `pending == 0`을 만들면, 성공한 point를 원래
   waypoint 순서대로 모아 Marker 한 개를 발행한다(fan-in).
5. 그 사이 새 plan이 들어오면 `generation`이 증가한다. 이전 generation callback은 결과를
   버리므로 서로 다른 plan의 point가 섞이지 않는다.

## rclpy → rclcpp에서 달라진 지점

### closure factory → callback lambda + 공유 batch

Python 원본은 `_make_fk_done()`이 callback closure를 만들어 `gen`, `i`, `results`, `pending`을
capture한다. `pending`은 closure 안에서 값을 바꿀 수 있도록 원소 하나짜리 list다.

C++에서는 `BatchState`를 `std::shared_ptr`로 만들고 모든 lambda가 함께 capture한다. 그래서
`onPlan()`이 return한 뒤에도 마지막 FK callback까지 결과 vector와 counter가 살아 있다.
`std::optional<geometry_msgs::msg::Point>`는 Python의 `None`/Point 슬롯에 대응한다.

### Future callback 등록 방식

Python 원본도 polling하지 않고 `call_async()`가 돌려준 Future에 `add_done_callback()`을
등록한다. rclcpp에서는 `async_send_request(request, callback)`에 callback을 같은 호출로
전달한다. API 모양은 다르지만 둘 다 응답을 기다리는 blocking spin이나 수동 polling이 없다.

### fan-in counter와 thread safety

원본 executable은 기본 `rclpy.spin()`의 single-threaded 실행을 전제로 closure의 list counter를
그대로 감소시킨다. C++ executable도 `rclcpp::spin()`이라 기본은 single-threaded지만, 학습 후
`MultiThreadedExecutor`로 옮겨도 깨지지 않도록 `generation`, 결과 기록, `pending`, 최종 publish를
하나의 mutex로 보호했다. 특히 generation 검사와 publish 사이에 새 plan이 끼어드는
check-then-act race를 막기 위해 최종 publish까지 같은 critical section에서 수행한다.

## 미포함 범위

- launch 파일
- 다른 `pick_fsm` 모듈과의 연동 변경
- simulation 또는 hardware 실행
- `/compute_fk` timeout/retry 정책 추가(원본에도 없음)
