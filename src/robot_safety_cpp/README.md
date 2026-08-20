# robot_safety_cpp

`pick_fsm/pick_fsm/robot_safety_node.py`를 C++(rclcpp)로 옮겨보는 **학습용** 패키지다.
원본 Python 파일은 건드리지 않았고, 이 패키지는 그것과 별개로 동작하는 독립 노드다
(같은 서비스/토픽 이름을 쓰므로 **둘을 동시에 띄우면 충돌한다** — 학습용으로 하나씩만 실행할 것).

## ⚠️ 검증 상태

**실기(dsr_controller2 + 실제 로봇)로 실행해본 적이 없다.** 확인한 것과 안 한 것을 구분한다:

| 확인함 | 확인 안 함 |
|---|---|
| `colcon build` 통과 (같은 turn에 실행, PASS) | `dsr_controller2` 가 떠 있는 상태에서 서비스 호출이 실제로 로봇을 멈추는지 |
| 서비스 필드 타입(`int8`/`int32`)을 `.srv` 파일 grep 으로 확인 | 비동기 콜백이 스핀 중 실제로 몇 ms 안에 도는지(타이밍) |
| Python 원본과 로직 구조가 1:1 대응하는지 육안 대조 | backdrive 진입/해제 시퀀스가 C++ 버전에서도 안전하게 동작하는지 |

즉 **"컴파일되고 구조가 맞는다"까지만 보장**한다. 로봇에 붙여서 쓰려면 사람이 비상정지
버튼을 손 닿는 곳에 두고 저속·저위험 자세에서 먼저 검증해야 한다 — 이건 Python 원본
주석에도 이미 적혀 있던 경고이고, C++ 로 옮겼다고 사라지는 위험이 아니다.

## 실행

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run robot_safety_cpp robot_safety_node --ros-args -p robot_ns:=dsr01 -p poll_hz:=2.0
```

Python 버전과 파라미터 이름·기본값이 동일하다(`robot_ns="dsr01"`, `poll_hz=2.0`).

## 토픽 · 서비스 (Python 원본과 동일)

발행:
- `/pick/robot_state_code` (`std_msgs/Int8`)
- `/pick/robot_state_text` (`std_msgs/String`)

서비스 (전부 `std_srvs/Trigger`, fire-and-forget — 아래 "설계 노트" 참고):
- `/safety/stop`
- `/safety/enter_backdrive`
- `/safety/exit_backdrive`

## rclpy → rclcpp, 이 코드에서 바뀐 지점들

### 1. 비동기 서비스 호출: 수동 폴링 → 콜백 등록

Python 원본은 타이머 콜백 안에서 `spin_until_future_complete()`를 부르면 재진입으로
엉킨다는 rclpy 함정 때문에, `_poll_fut` 멤버에 Future를 저장해두고 매 tick마다
"없으면 보내고, 있으면 `done()`만 확인"하는 수동 폴링을 했다.

```python
def _poll(self):
    if self._poll_fut is None:
        ...
        self._poll_fut = self.cli_state.call_async(GetRobotState.Request())
        return
    if not self._poll_fut.done():
        return
    res = self._poll_fut.result()
    ...
```

rclcpp의 `async_send_request(request, callback)`은 콜백을 바로 등록할 수 있어서 이 수동
폴링이 필요 없다. executor가 응답을 받으면 알아서 콜백을 실행해준다 — 콜백 안에서
또 다른 블로킹 spin 계열 함수만 안 부르면 재진입 문제 자체가 안 생기는 구조다.
이 포팅에서는 "이전 요청 완료 전에 또 보내지 않는다"는 목적만 `in_flight_` 불리언
플래그로 남겼다.

**배우는 것**: 같은 문제(비동기 응답 대기)라도 언어/라이브러리가 제공하는 동시성 원시
자료형이 다르면 해법의 모양이 통째로 달라진다. rclpy Future의 폴링 스타일은 사실
"콜백을 못 걸어서"가 아니라 원본 작성 당시 관례였을 수 있다 — rclpy도 `add_done_callback`을
지원한다. 즉 이 차이는 언어 강제가 아니라 **코드 작성자가 어느 스타일을 골랐는가**의
차이일 수도 있다는 점도 같이 알아두면 좋다.

### 2. 제네릭 함수: 덕 타이핑 → 템플릿

Python `_fire()`는 서비스 타입이 뭐든 `client.call_async(request)`만 부르면 그만이다
(덕 타이핑 — 타입을 안 따지고 그냥 메서드가 있으면 부른다). C++는 `rclcpp::Client<T>`,
`T::Request`가 서비스 타입마다 별개 클래스라, 함수 하나로 재사용하려면 `template<typename
ServiceT>`가 필요하다. 이 패키지의 `fire<ServiceT>()`가 그 대응물이다.

**배우는 것**: 템플릿은 "일반화된 코드"라는 점에서 결과는 덕 타이핑과 비슷해 보이지만,
컴파일 타임에 타입별로 코드가 각각 찍혀 나온다(템플릿 인스턴스화) — 런타임 비용이 없는
대신 에러 메시지가 덜 친절하다는 트레이드오프가 있다.

### 3. 메시지/서비스 요청 생성: 생성자 인자 → 필드 대입

```python
req = SetSafetyMode.Request()
req.safety_mode = SAFETY_MODE_BACKDRIVE
req.safety_event = SAFETY_MODE_EVENT_ENTER
```

```cpp
auto req = std::make_shared<dsr_msgs2::srv::SetSafetyMode::Request>();
req->safety_mode = kSafetyModeBackdrive;
req->safety_event = kSafetyModeEventEnter;
```

모양은 비슷해 보이지만 C++ 쪽은 `std::make_shared`로 힙에 만들고 `shared_ptr`로 들고
다닌다는 차이가 있다 — rclcpp API가 요청 객체를 `shared_ptr`로 요구하기 때문(콜백이
비동기로 나중에 실행되므로, 그 시점까지 요청 객체가 살아있어야 해서 공유 소유권이 필요).

### 4. 타입 불일치 함정을 컴파일 타임에 잡아준 사례

`GetRobotState`/`SetSafetyMode`/`SetRobotControl`의 필드는 전부 `int8`인데
`MoveStop.stop_mode`만 `int32`다(`.srv` 파일이 필드마다 타입을 따로 고른다). Python은
동적 타입이라 이 차이를 신경 안 써도 실행이 되지만, C++은 `kDrHold`를 `int32_t`로
따로 선언해야 했다 — 안 그러면 컴파일러가 좁히기 변환(narrowing) 경고를 낸다. 이게
"C++이 컴파일 타임에 실수를 잡아준다"의 실물 예시다.

## 설계 노트: 왜 Trigger 서비스가 "성공"을 반환해도 로봇이 안 멈췄을 수 있는가

`/safety/stop` 등은 **fire-and-forget** 계약이다. 서비스 응답의 `success=true`는
"드라이버에 정지 명령을 보냈다"는 뜻이지 "로봇이 실제로 멈췄다"는 뜻이 아니다. 실제
결과는 `fire()` 콜백의 로그와 `/pick/robot_state_text` 토픽으로 나중에 드러난다.
이건 Python 원본이 이미 그렇게 설계했던 이유(`task_manager.py`의 `/pick/start`와 같은
계약, 서비스 콜백 안에서 블로킹하면 재진입 함정)를 그대로 물려받은 것이지, C++ 포팅
과정에서 새로 만든 설계가 아니다.

## 포팅 안 한 것

- Python 원본의 `UNSAFE_STATES` 상수 — `robot_safety_node.py` 안에서는 안 쓰이고
  `task_manager.py`가 import해서 쓰는 값이라, 이 노드 단독 포팅 범위 밖이라 판단해 뺐다.
- `CONTROL_RECOVERY_BACKDRIVE`(값 6) 관련 경고 주석 — 코드에서 그 값을 안 쓰므로 옮길
  로직이 없다. 그 값을 왜 피해야 하는지는 원본 Python 파일 주석 참고.

## 다음 학습 스텝 (제안, 아직 안 함)

- 콜백 그룹(`rclcpp::CallbackGroup`)을 명시적으로 나눠서 poll()과 서비스 콜백이 서로
  기다리지 않는지 실험 — 지금은 기본 그룹 하나로도 fire-and-forget이라 문제가 없지만,
  MultiThreadedExecutor로 바꿔서 동시성 버그를 일부러 만들어보는 것도 학습 소재가 된다.
- `launch` 파일 작성 — 이 패키지엔 아직 없다. Python 노드처럼 `robot_ns`/`poll_hz`를
  launch argument로 넘기는 예제를 만들어보면 launch XML/Python 문법도 같이 익힐 수 있다.
