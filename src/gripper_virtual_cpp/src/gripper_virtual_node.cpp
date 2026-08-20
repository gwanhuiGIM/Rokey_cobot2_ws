// DRCF emulator mode에서 RViz gripper joint를 움직이는 가상 node.
//
// 원본(참고만 하며 수정하지 않음):
//   cobot_rg2/rg2/m0609_rg2_bringup/scripts/gripper_virtual_node.py
//
// 이 포팅의 학습 주제는 "blocking service callback과 timer callback의 실제 동시 실행"이다.
// service callback은 목표 도달까지 polling하며 자기 worker thread를 점유한다. 그동안 timer가
// 다른 worker thread에서 position을 갱신해야 하므로 Reentrant callback group과
// MultiThreadedExecutor가 둘 다 필요하다. SingleThreadedExecutor에서는 service callback이
// 유일한 executor thread를 막아 timer가 실행되지 못하고, 결과적으로 service가 끝나지 않는다.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <regex>
#include <stdexcept>
#include <string>
#include <thread>

#include "onrobot_rg_msgs/srv/set_command.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace
{
constexpr double kGripperOpen = -0.558505;   // rad, rg2_finger_joint URDF lower limit
constexpr double kGripperClosed = 0.785398;  // rad, rg2_finger_joint URDF upper limit
constexpr double kPublishRate = 1.0 / 50.0;  // s, real OnRobot driver와 같은 50 Hz
constexpr double kSpeed = 1.0;               // rad/s animation speed
constexpr double kDoneTolerance = 0.01;      // rad, target 도달 판정 허용오차
}  // namespace

class GripperVirtualNode : public rclcpp::Node
{
public:
  GripperVirtualNode()
  : Node("gripper_virtual_node")
  {
    // rclpy ReentrantCallbackGroup의 rclcpp 대응. 같은 group 안의 service와 timer가 서로
    // 배타적으로 serialize되지 않으므로 MultiThreadedExecutor worker 두 개에서 동시에
    // 실행될 수 있다. 기본 MutuallyExclusive group을 쓰면 executor thread가 여러 개여도
    // 이 두 callback은 동시에 실행되지 않아 blocking service가 끝나지 않는다.
    callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      "/gripper_joint_states", 10);

    // rclpy Node.create_timer()는 node clock을 사용한다. create_wall_timer()로 옮기면
    // use_sim_time에서 /clock이 멈춰도 C++ animation만 진행하는 차이가 생기므로,
    // get_clock()을 받는 rclcpp::create_timer()로 원본 clock semantics까지 유지한다.
    publish_timer_ = rclcpp::create_timer(
      this,
      get_clock(),
      rclcpp::Duration::from_seconds(kPublishRate),
      std::bind(&GripperVirtualNode::publishCallback, this),
      callback_group_);

    command_service_ = create_service<onrobot_rg_msgs::srv::SetCommand>(
      "/onrobot/sendCommand",
      std::bind(
        &GripperVirtualNode::sendCommandCallback, this,
        std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default,
      callback_group_);

    RCLCPP_INFO(get_logger(), "GripperVirtualNode ready - /onrobot/sendCommand");
  }

private:
  void sendCommandCallback(
    const onrobot_rg_msgs::srv::SetCommand::Request::SharedPtr request,
    onrobot_rg_msgs::srv::SetCommand::Response::SharedPtr response)
  {
    double target;
    if (request->command == "c") {
      target = kGripperClosed;
    } else if (request->command == "o") {
      target = kGripperOpen;
    } else {
      // Python float()의 일반적인 ASCII 문법은 주변 whitespace와 숫자 사이의 단일
      // underscore를 허용하지만 C++ std::stod는 underscore를 모르고 Python이 거부하는
      // hex float는 수락한다. 먼저 ASCII decimal/inf/nan 문법을 regex로 확인하고
      // underscore만 제거한 뒤 stod에 넘긴다. Unicode 숫자/공백 parity는 이 범위에 없다.
      try {
        static const std::regex python_float_pattern(
          R"(^\s*[+-]?(?:(?:(?:[0-9](?:_?[0-9])*)(?:\.(?:[0-9](?:_?[0-9])*)?)?|\.(?:[0-9](?:_?[0-9])*))(?:[eE][+-]?(?:[0-9](?:_?[0-9])*))?|inf(?:inity)?|nan)\s*$)",
          std::regex_constants::icase);
        if (!std::regex_match(request->command, python_float_pattern)) {
          throw std::invalid_argument("not a Python float literal");
        }

        std::string normalized_command = request->command;
        normalized_command.erase(
          std::remove(normalized_command.begin(), normalized_command.end(), '_'),
          normalized_command.end());
        target = std::stod(normalized_command);
      } catch (const std::exception &) {
        response->success = false;
        response->message = "Unknown command: '" + request->command + "'";
        return;
      }

      // Python max(OPEN, min(CLOSED, target))와 같은 순서로 clamp한다.
      target = std::max(kGripperOpen, std::min(kGripperClosed, target));
    }

    {
      // Python `with self._lock:`과 std::mutex + std::lock_guard의 직접 대응이다.
      // lock_guard는 이 scope가 끝날 때 RAII로 unlock한다. 아래 polling sleep까지 lock을
      // 잡고 있으면 timer가 position을 갱신할 수 없어 deadlock이 되므로 scope를 넓히지 않는다.
      std::lock_guard<std::mutex> lock(state_mutex_);
      target_ = target;
    }

    // 원본과 동일한 blocking polling loop. 이 callback을 맡은 worker thread 하나는 여기서
    // 목표 도달까지 점유된다. Reentrant group + MultiThreadedExecutor의 다른 worker가
    // publishCallback()을 계속 실행해 position_을 움직이는 것이 이 구조의 전제다.
    while (true) {
      {
        // 읽는 순간에만 lock을 잡고 sleep 중에는 반드시 놓는다.
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (std::abs(position_ - target) < kDoneTolerance) {
          break;
        }
      }

      // Python time.sleep(PUBLISH_RATE)의 C++ 대응. wall-clock duration 동안 현재 service
      // worker만 재우며, executor의 다른 worker thread와 timer callback은 계속 실행된다.
      std::this_thread::sleep_for(std::chrono::duration<double>(kPublishRate));
    }

    response->success = true;
    response->message = "";
  }

  void publishCallback()
  {
    double position;
    {
      // position_과 target_ 계산 전체를 하나의 critical section으로 묶어 service의 target
      // 갱신과 race하지 않게 한다. publish는 공유 상태를 쓰지 않으므로 lock 밖에서 수행한다.
      std::lock_guard<std::mutex> lock(state_mutex_);
      const double difference = target_ - position_;
      const double step = kSpeed * kPublishRate;

      if (std::abs(difference) <= step) {
        position_ = target_;
      } else {
        position_ += step * (difference > 0.0 ? 1.0 : -1.0);
      }
      position = position_;
    }

    sensor_msgs::msg::JointState message;
    message.header.stamp = now();
    message.name = {"rg2_finger_joint"};
    message.position = {position};
    joint_state_pub_->publish(message);
  }

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::Service<onrobot_rg_msgs::srv::SetCommand>::SharedPtr command_service_;

  // Python의 _position, _target, threading.Lock에 각각 대응한다. callback group이 실제
  // 동시 실행을 허용하므로 이 mutex는 설명용 장식이 아니라 data race를 막는 필수 요소다.
  std::mutex state_mutex_;
  double position_{0.0};
  double target_{0.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GripperVirtualNode>();

  // 이 node는 SingleThreadedExecutor로 바꾸면 안 된다. service callback의 polling이 끝나려면
  // timer callback이 다른 thread에서 position_을 갱신해야 한다. 기본 thread 수를 사용하는
  // MultiThreadedExecutor가 그 병행 실행을 제공한다.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  executor.remove_node(node);
  node.reset();
  // SIGINT handler가 이미 context를 shutdown했을 수 있으므로 Python 원본의 rclpy.ok() guard와
  // 같은 의도로 중복 shutdown을 피한다.
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
