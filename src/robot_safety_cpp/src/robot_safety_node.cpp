// robot_safety_node.py 의 C++ 포팅 (학습용).
// 원본: pick_fsm/pick_fsm/robot_safety_node.py — 이 파일은 원본을 참고만 하고 절대 수정하지 않는다.
//
// ⚠️ 이 코드는 실기(dsr_controller2 드라이버 + 실제 로봇)로 검증되지 않았다. 서비스 이름·필드
// 타입은 dsr_msgs2 .srv 파일을 grep으로 직접 확인해 맞췄지만("검증됨"은 실행 결과에만 씀 —
// ~/.claude/CLAUDE.md 6-1절), 실제로 dsr_controller2 가 떠 있는 상태에서 호출해본 적은 없다.
// 학습 목적의 "구조가 맞는지" 코드이지, "실기에서 이렇게 동작한다"를 보장하는 코드가 아니다.
//
// ── Python(rclpy) vs C++(rclcpp) 핵심 차이, 이 파일에서 만날 것들 ──────────────
// 1. 콜백을 넘기는 방법: rclpy 는 함수/람다를 그냥 값으로 넘기면 되지만(동적 타입), rclcpp 는
//    콜백 시그니처가 템플릿 타입(SharedFuture<ServiceT>)에 정확히 맞아야 컴파일된다.
// 2. 소유권: rclpy 는 GC 가 알아서 치우지만, rclcpp 는 rclcpp::Node/Publisher/Client 등이 전부
//    std::shared_ptr 로 관리된다 — "언제 죽는지"를 사람이 신경써야 한다.
// 3. 메시지 필드: Python 은 dsr_msgs2.msg.Xxx(field=value) 로 생성자에서 바로 채우지만, C++ 은
//    기본 생성 후 필드에 하나씩 대입한다(아래 코드 참고).
// ────────────────────────────────────────────────────────────────────────────

#include <array>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "dsr_msgs2/srv/get_robot_state.hpp"
#include "dsr_msgs2/srv/move_stop.hpp"
#include "dsr_msgs2/srv/set_robot_control.hpp"
#include "dsr_msgs2/srv/set_safety_mode.hpp"
#include "rclcpp/create_timer.hpp"

// Python 의 ROBOT_STATE_NAMES 딕셔너리 + 상수들. 모듈 전역 대신 익명 네임스페이스에 둔다 —
// C++ 에는 "모듈 전역"이 없고, 이 파일 밖에서 안 보이게 하려면 static/익명 네임스페이스가
// Python 의 "이 파일 안에서만 쓰는 상수"에 대응한다.
namespace
{
const std::unordered_map<int8_t, std::string> kRobotStateNames{
  {0, "INITIALIZING"}, {1, "STANDBY"}, {2, "MOVING"}, {3, "SAFE_OFF"}, {4, "TEACHING"},
  {5, "SAFE_STOP"}, {6, "EMERGENCY_STOP"}, {7, "HOMMING"}, {8, "RECOVERY"},
  {9, "SAFE_STOP2"}, {10, "SAFE_OFF2"}, {15, "NOT_READY"},
};

constexpr int8_t kSafetyModeAutonomous = 1;
constexpr int8_t kSafetyModeBackdrive = 3;
constexpr int8_t kSafetyModeEventEnter = 0;

constexpr int8_t kResetSafetStop = 2;
constexpr int8_t kResetSafetOff = 3;
constexpr std::array<std::pair<int8_t, const char *>, 2> kResets{{
  {kResetSafetStop, "RESET_SAFET_STOP"},
  {kResetSafetOff, "RESET_SAFET_OFF"},
}};

// MoveStop.srv 의 stop_mode 는 int32 다(GetRobotState/SetSafetyMode/SetRobotControl 은 int8) —
// .srv 정의가 필드마다 타입을 따로 고르기 때문에 이런 불일치가 생긴다. Python 은 동적 타입이라
// 신경 안 써도 되지만, C++ 은 대입할 때 타입이 안 맞으면 컴파일러가 경고/에러를 낸다.
constexpr int32_t kDrHold = 3;
}  // namespace

class RobotSafetyNode : public rclcpp::Node
{
public:
  RobotSafetyNode()
  : Node("robot_safety_node")
  {
    // declare_parameter 는 rclpy 와 동일한 개념이지만, C++ 은 템플릿 인자로 타입을 명시한다
    // (rclpy 는 기본값의 파이썬 타입을 보고 런타임에 추론한다).
    declare_parameter<std::string>("robot_ns", "dsr01");
    declare_parameter<double>("poll_hz", 2.0);
    const auto ns = get_parameter("robot_ns").as_string();
    const auto prefix = ns.empty() ? "" : "/" + ns;

    // create_client<ServiceT>(name) — Python 의 create_client(SrvType, name) 과 같은 역할이지만
    // 서비스 타입이 <> 안 템플릿 인자로 들어간다. rclpy 는 콜백 그룹을 인자로 넘겨야 재진입을
    // 제어할 수 있었지만(ReentrantCallbackGroup), 여기서는 스핀 중 블로킹 호출을 아예 안 하므로
    // (아래 fire() 참고) 기본 콜백 그룹으로도 문제가 없다.
    cli_state_ = create_client<dsr_msgs2::srv::GetRobotState>(
      prefix + "/system/get_robot_state");
    cli_control_ = create_client<dsr_msgs2::srv::SetRobotControl>(
      prefix + "/system/set_robot_control");
    cli_safety_mode_ = create_client<dsr_msgs2::srv::SetSafetyMode>(
      prefix + "/system/set_safety_mode");
    cli_stop_ = create_client<dsr_msgs2::srv::MoveStop>(
      prefix + "/motion/move_stop");

    pub_code_ = create_publisher<std_msgs::msg::Int8>("/pick/robot_state_code", 10);
    pub_text_ = create_publisher<std_msgs::msg::String>("/pick/robot_state_text", 10);

    const double hz = get_parameter("poll_hz").as_double();
    // rclpy Node.create_timer()는 node clock을 사용한다. get_clock()을 넘기는 create_timer로
    // 옮겨 use_sim_time에서 /clock이 멈추면 상태 polling도 함께 멈추는 원본 semantics를 유지한다.
    timer_ = rclcpp::create_timer(
      this,
      get_clock(),
      rclcpp::Duration::from_seconds(1.0 / hz),
      std::bind(&RobotSafetyNode::poll, this));

    // create_service 콜백은 rclpy 처럼 (request, response) -> response 형태가 아니라
    // (request, response) -> void 다. response 는 참조로 미리 만들어져 들어오고, 콜백은 그
    // 내용만 채운다 — 리턴값이 없다는 게 처음 보면 헷갈리는 포인트.
    srv_stop_ = create_service<std_srvs::srv::Trigger>(
      "/safety/stop",
      std::bind(
        &RobotSafetyNode::srvStop, this,
        std::placeholders::_1, std::placeholders::_2));
    srv_enter_backdrive_ = create_service<std_srvs::srv::Trigger>(
      "/safety/enter_backdrive",
      std::bind(
        &RobotSafetyNode::srvEnterBackdrive, this,
        std::placeholders::_1, std::placeholders::_2));
    srv_exit_backdrive_ = create_service<std_srvs::srv::Trigger>(
      "/safety/exit_backdrive",
      std::bind(
        &RobotSafetyNode::srvExitBackdrive, this,
        std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(
      get_logger(),
      "준비됨(C++ 포팅, 미검증) — 로봇 네임스페이스 '%s' 기준 Doosan 안전 서비스에 연결 시도",
      ns.empty() ? "(없음)" : ns.c_str());
  }

private:
  // ── 상태 폴링 ────────────────────────────────────────────
  // Python 원본은 _poll_fut 멤버에 Future 를 저장해두고 매 tick 마다 "아직 없으면 보내고,
  // 있으면 done() 인지만 확인" 하는 수동 폴링을 했다 — 이유는 파일 주석에 있듯 "타이머 콜백
  // 안에서 spin_until_future_complete() 를 부르면 재진입으로 엉킨다"는 rclpy 함정을 피하려는
  // 것이었다.
  //
  // rclcpp 의 async_send_request(request, callback) 은 콜백을 직접 등록할 수 있어서 저 수동
  // 폴링이 필요 없다 — executor 가 응답을 받으면 알아서 콜백을 불러준다(콜백 안에서 또
  // spin 계열 블로킹 함수만 안 부르면 재진입 문제도 원천적으로 없다). 대신 "이전 요청이 아직
  // 안 끝났는데 새 요청을 또 보내는" 것만 막아주면 되므로, 그 목적만 in_flight_ 플래그
  // 하나로 대체했다.
  void poll()
  {
    if (in_flight_) {
      return;  // 이전 GetRobotState 응답을 아직 기다리는 중 — 이번 tick 은 건너뛴다.
    }
    if (!cli_state_->service_is_ready()) {
      return;  // dsr_controller2 가 아직 안 떴을 수 있다. 에러로 안 취급하고 다음 tick 재시도.
    }
    in_flight_ = true;

    auto request = std::make_shared<dsr_msgs2::srv::GetRobotState::Request>();
    // GetRobotState::Request 는 필드가 없다(.srv 의 --- 위쪽이 비어있음) — 빈 요청도
    // 반드시 shared_ptr 로 만들어야 한다(rclcpp 클라이언트 API 의 요구사항).

    cli_state_->async_send_request(
      request,
      // 캡처 리스트에 this 를 넣어 멤버(pub_code_ 등)에 접근한다. 이 람다는 executor 스레드가
      // 응답을 받는 시점에 나중에 호출된다 — poll() 함수가 이미 리턴한 뒤라는 뜻.
      [this](rclcpp::Client<dsr_msgs2::srv::GetRobotState>::SharedFuture future) {
        in_flight_ = false;
        auto res = future.get();
        if (!res || !res->success) {
          return;
        }
        const int8_t code = res->robot_state;

        std_msgs::msg::Int8 code_msg;
        code_msg.data = code;
        pub_code_->publish(code_msg);

        std_msgs::msg::String text_msg;
        const auto it = kRobotStateNames.find(code);
        text_msg.data = (it != kRobotStateNames.end())
          ? it->second
          : ("UNKNOWN(" + std::to_string(code) + ")");
        pub_text_->publish(text_msg);
      });
  }

  // ── fire-and-forget 안전 명령 ────────────────────────────
  // Python 의 _fire() 를 그대로 옮기되, 서비스 타입이 4종류(MoveStop/SetSafetyMode/
  // SetRobotControl/...)라 함수 하나로 재사용하려면 템플릿이 필요하다. rclpy 는 덕 타이핑이라
  // 그냥 함수 하나로 다 받았지만, C++ 은 타입마다 rclcpp::Client<T>, T::Request 가 달라서
  // 템플릿 없이는 함수 시그니처를 못 쓴다 — "제네릭 프로그래밍"을 실감하는 지점.
  template<typename ServiceT>
  void fire(
    typename rclcpp::Client<ServiceT>::SharedPtr client,
    typename ServiceT::Request::SharedPtr request,
    const std::string & name,
    const std::shared_ptr<std_srvs::srv::Trigger::Response> & response)
  {
    if (!client->service_is_ready()) {
      response->success = false;
      response->message = name + " 서비스 없음 (dsr_controller2 미기동?)";
      return;
    }
    client->async_send_request(
      request,
      [this, name](typename rclcpp::Client<ServiceT>::SharedFuture future) {
        auto res = future.get();
        if (!res) {
          RCLCPP_ERROR(get_logger(), "%s: 응답 없음", name.c_str());
        } else if (!res->success) {
          RCLCPP_ERROR(get_logger(), "%s: 드라이버가 실패 응답", name.c_str());
        } else {
          RCLCPP_INFO(get_logger(), "%s: 완료", name.c_str());
        }
      });
    // Trigger 서비스 자체의 응답은 "요청을 보냈다"는 뜻이지 "로봇이 멈췄다"는 뜻이 아니다.
    // 실제 결과는 위 콜백의 로그와 poll() 이 발행하는 /pick/robot_state_text 로 나중에
    // 드러난다 — task_manager 의 /pick/start 와 같은 계약(원본 주석 그대로).
    response->success = true;
    response->message = name + " 요청 전송함 — 결과는 로그·상태 토픽에서 확인";
  }

  void srvStop(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>/*req(미사용)*/,
    const std::shared_ptr<std_srvs::srv::Trigger::Response> & res)
  {
    auto req = std::make_shared<dsr_msgs2::srv::MoveStop::Request>();
    req->stop_mode = kDrHold;
    fire<dsr_msgs2::srv::MoveStop>(cli_stop_, req, "move_stop(HOLD)", res);
  }

  void srvEnterBackdrive(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>/*req(미사용)*/,
    const std::shared_ptr<std_srvs::srv::Trigger::Response> & res)
  {
    auto req = std::make_shared<dsr_msgs2::srv::SetSafetyMode::Request>();
    req->safety_mode = kSafetyModeBackdrive;
    req->safety_event = kSafetyModeEventEnter;
    fire<dsr_msgs2::srv::SetSafetyMode>(
      cli_safety_mode_, req, "set_safety_mode(BACKDRIVE)", res);
  }

  // exit_backdrive 는 fire() 로 재사용하지 않는다 — 응답이 온 뒤 추가로 set_robot_control 을
  // 두 번 더 쏘는 후속 로직이 있어서(원본 Python 도 이 서비스만 별도로 풀어 썼다).
  void srvExitBackdrive(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>/*req(미사용)*/,
    const std::shared_ptr<std_srvs::srv::Trigger::Response> & res)
  {
    auto req = std::make_shared<dsr_msgs2::srv::SetSafetyMode::Request>();
    req->safety_mode = kSafetyModeAutonomous;
    req->safety_event = kSafetyModeEventEnter;

    if (!cli_safety_mode_->service_is_ready()) {
      res->success = false;
      res->message = "set_safety_mode 서비스 없음";
      return;
    }

    cli_safety_mode_->async_send_request(
      req,
      [this](rclcpp::Client<dsr_msgs2::srv::SetSafetyMode>::SharedFuture future) {
        auto r = future.get();
        if (r && r->success) {
          RCLCPP_INFO(get_logger(), "set_safety_mode(AUTONOMOUS): 완료");
        } else {
          RCLCPP_ERROR(get_logger(), "set_safety_mode(AUTONOMOUS): 실패");
        }

        // SAFE_STOP/SAFE_OFF 로 남아 있을 수 있으니 리셋도 시도한다. 드라이버가 이미
        // 자동으로 했을 수도 있다(dsr_controller2.cpp 의 OnMonitoringStateCB) — 이건 그
        // 보험이지 유일한 경로가 아니다. 실패해도 조용히 넘어간다: 애초에 그 상태가
        // 아니면 드라이버가 거부하는 게 정상이다(원본 주석 그대로).
        if (!cli_control_->service_is_ready()) {
          return;
        }
        // reset 목록은 항상 두 개이므로 고정 크기 kResets를 순회한다. 구조적 바인딩
        // (auto & [value, label])은 Python의 (값, 라벨) tuple unpacking에 대응한다.
        for (const auto & [value, label] : kResets) {
          auto creq = std::make_shared<dsr_msgs2::srv::SetRobotControl::Request>();
          creq->robot_control = value;
          cli_control_->async_send_request(
            creq,
            [this, label](rclcpp::Client<dsr_msgs2::srv::SetRobotControl>::SharedFuture f) {
              auto cres = f.get();
              RCLCPP_INFO(
                get_logger(), "set_robot_control(%s): %s",
                label,
                (cres && cres->success) ? "완료" : "실패/불필요");
            });
        }
      });

    res->success = true;
    res->message = "exit_backdrive 요청 전송함 — 결과는 로그·상태 토픽에서 확인";
  }

  bool in_flight_ = false;

  rclcpp::Client<dsr_msgs2::srv::GetRobotState>::SharedPtr cli_state_;
  rclcpp::Client<dsr_msgs2::srv::SetRobotControl>::SharedPtr cli_control_;
  rclcpp::Client<dsr_msgs2::srv::SetSafetyMode>::SharedPtr cli_safety_mode_;
  rclcpp::Client<dsr_msgs2::srv::MoveStop>::SharedPtr cli_stop_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pub_code_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_text_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_stop_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_enter_backdrive_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_exit_backdrive_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // std::make_shared<Node>() — Python 의 RobotSafetyNode() 인스턴스화 + rclpy.spin(node) 를
  // 합친 자리. rclcpp::spin() 은 인자로 shared_ptr<Node> 를 받는다(원시 포인터 아님) —
  // spin 이 도는 동안 노드가 살아있어야 하므로 소유권을 공유해야 한다는 rclcpp 의 설계.
  rclcpp::spin(std::make_shared<RobotSafetyNode>());
  rclcpp::shutdown();
  return 0;
}
