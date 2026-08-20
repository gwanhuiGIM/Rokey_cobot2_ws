# gripper_virtual_cpp

`cobot_rg2/rg2/m0609_rg2_bringup/scripts/gripper_virtual_node.py`를 C++(`rclcpp` +
`std::thread`/`std::mutex`)로 옮긴 **학습용** 패키지다. 원본 Python 파일은 수정하지 않았으며,
DRCF emulator mode에서 실제 OnRobot driver와 같은 ROS interface를 흉내 내 RViz의 gripper
joint를 움직이는 독립 node다. Python/C++ 버전 또는 실제 driver를 동시에 실행하면 같은
service 이름이 충돌할 수 있으므로 하나만 실행한다.

## 검증 상태

실제 gripper와 DRCF emulator에서 실행하지 않았다. 이번 범위는 package build와 executable
등록 확인까지다.

| 확인함 | 확인 안 함 |
|---|---|
| 설치 생성된 `SetCommand` C++ header에서 `command`/`success`/`message` 필드 확인 | DRCF emulator + RViz에서 joint animation 확인 |
| Python 원본과 topic/service, 상수, node clock timer, callback group, mutex scope, polling 구조 대조 | 실제 OnRobot gripper 또는 driver와 연동 |
| `colcon build --symlink-install --packages-select gripper_virtual_cpp` | service 응답 latency 및 50 Hz runtime timing 측정 |
| `ros2 pkg executables gripper_virtual_cpp`에서 executable 등록 확인 | 종료 중 진행 중인 blocking service callback의 runtime 거동 |

따라서 보장 범위는 **원본 interface와 concurrency 구조를 유지한 코드가 컴파일되고 executable로
등록되는 것**까지다. simulation과 hardware 동작은 미검증이다.

## 실행

> 아래 `ros2 run`으로 node를 실제 실행하는 검증은 이번 범위에서 하지 않았다.

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run gripper_virtual_cpp gripper_virtual_node
```

## ROS interface와 상수

| 구분 | 이름 | 타입 / 값 |
|---|---|---|
| publisher | `/gripper_joint_states` | `sensor_msgs/msg/JointState`, depth 10 |
| service | `/onrobot/sendCommand` | `onrobot_rg_msgs/srv/SetCommand` |
| joint name | `rg2_finger_joint` | position 단위 rad |
| `GRIPPER_OPEN` | - | `-0.558505` rad |
| `GRIPPER_CLOSED` | - | `0.785398` rad |
| `PUBLISH_RATE` | - | `1.0 / 50.0` s (50 Hz) |
| `SPEED` | - | `1.0` rad/s |
| `DONE_TOL` | - | `0.01` rad |

`command="c"`는 closed, `command="o"`는 open target을 설정한다. 그 밖의 문자열은 `double`로
parse한 뒤 `[GRIPPER_OPEN, GRIPPER_CLOSED]`로 clamp한다. parse가 실패하면 `success=false`와
error message를 즉시 반환한다.

## 왜 이 node만 MultiThreadedExecutor를 유지하는가

첫 포팅 묶음의 `robot_safety_cpp`, `planned_tcp_path_cpp`, `depth_downsample_cpp`는 callback 안에서
다른 callback의 진행을 기다리는 구조가 아니어서 기본 `SingleThreadedExecutor`로도 각자의 핵심
동작이 진행된다. 반면 이 node의 service callback은 목표에 도달할 때까지 polling loop에서
blocking한다. 목표를 향해 `position`을 움직이는 코드는 50 Hz timer callback에만 있다.

따라서 `SingleThreadedExecutor`이면 다음 순환 대기가 생긴다.

1. service callback이 유일한 executor thread를 점유하고 `position` 변화를 기다린다.
2. timer callback은 그 thread를 얻지 못해 `position`을 갱신하지 못한다.
3. `position`이 바뀌지 않으므로 service callback도 끝나지 않는다.

이 포팅은 Python 원본과 같이 `Reentrant` callback group과 `MultiThreadedExecutor`를 함께 쓴다.
`MultiThreadedExecutor`만 쓰고 기본 `MutuallyExclusive` callback group에 두 callback을 넣어도
동시 실행이 허용되지 않으므로 충분하지 않다.

## rclpy → rclcpp에서 대응하는 지점

### `threading.Lock` → `std::mutex` + `std::lock_guard`

Python의 `with self._lock:` scope는 C++의 중괄호 scope 안 `std::lock_guard<std::mutex>`로
옮겼다. `lock_guard`는 scope를 벗어날 때 자동으로 unlock한다(RAII). `_position`과 `_target`을
읽거나 쓰는 짧은 구간만 보호하며, ROS publish와 polling sleep 중에는 lock을 들고 있지 않는다.
lock scope를 sleep까지 넓히면 timer가 같은 mutex를 얻지 못해 animation이 멈춘다.

### node clock timer

Python `Node.create_timer()`는 node의 ROS clock을 사용한다. C++도 `create_wall_timer()`가 아니라
`rclcpp::create_timer(..., get_clock(), ...)`를 써서 같은 clock semantics를 유지한다.
`use_sim_time:=true`이면 `/clock` 진행에 맞춰 두 버전의 50 Hz animation도 함께 진행한다.

### `time.sleep()` → `std::this_thread::sleep_for()`

service callback은 원본과 같이 `PUBLISH_RATE`마다 mutex를 잠깐 잡아 현재 position을 확인한다.
Python `time.sleep(PUBLISH_RATE)`는
`std::this_thread::sleep_for(std::chrono::duration<double>(PUBLISH_RATE))`로 대응한다. sleep은
service를 처리하는 worker thread 하나만 재우며, 다른 executor worker의 timer callback은 계속
실행될 수 있다.

### `float()` 예외 → `std::stod()` 예외

Python `float()`의 문법과 C++ `std::stod()` 문법은 완전히 같지 않다. 그래서 먼저 regex로
ASCII Python-style decimal/`inf`/`nan` 형태를 검사하고, Python이 허용하는 숫자 사이 underscore를
제거한 뒤 `std::stod()`로 변환한다. ASCII 주변 whitespace와 underscore는 허용하고 hex float와
trailing garbage는 거부한다. `std::stod()`의 `std::invalid_argument`/`std::out_of_range`는
`std::exception`으로 받아 실패 response로 바꾼다.

## 원본에서 그대로 유지한 한계

- service callback에는 timeout이나 cancel path가 없다.
- 종료 여부를 polling 조건에 추가하지 않았다. 종료 중 callback이 어떤 상태로 정리되는지는
  runtime 미검증이다.
- `Reentrant`이므로 여러 service 요청이 동시에 들어올 수 있다. 뒤 요청이 target을 바꾸면 앞
  요청의 local target 대기가 길어지거나 끝나지 않을 수 있다.
- Python `float()`가 허용하는 Unicode 숫자와 Unicode whitespace까지 재현하지는 않는다.
  numeric command의 parser parity 범위는 위에 적은 ASCII 문법이다.

이 항목들은 이번 학습 포팅에서 개선하지 않았다. polling과 lock scope를 포함한 원본 동시성
구조를 1:1로 재현하는 것이 목적이며, 실기 배포 설계로 승인했다는 뜻이 아니다.

## 미포함 범위

- launch 파일
- `grip_test.py` 등 client 연동 변경
- DRCF emulator/RViz runtime 검증
- 실제 gripper hardware 검증
- blocking polling 구조의 timeout/cancel/action 기반 개선
