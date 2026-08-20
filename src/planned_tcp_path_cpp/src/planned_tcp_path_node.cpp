// move_group이 계획한 joint trajectory를 /compute_fk로 풀어 TCP LINE_STRIP을 그린다.
//
// 원본(참고만 하며 수정하지 않음):
//   pick_fsm/pick_fsm/planned_tcp_path_node.py
//
// 이 포팅의 학습 주제는 fan-out/fan-in이다.
//   fan-out: 선택한 waypoint N개에 대해 FK 요청 N개를 기다리지 않고 연속 전송한다.
//   fan-in : 응답마다 같은 BatchState의 결과 슬롯과 pending을 갱신하고, 마지막 응답이
//            도착한 콜백 하나가 결과를 waypoint 순서대로 Marker로 발행한다.
//
// rclcpp::spin(node)는 기본적으로 SingleThreadedExecutor를 사용하므로 이 실행 파일만 보면
// 콜백이 동시에 실행되지 않는다. 그러나 이 노드를 나중에 component나
// MultiThreadedExecutor로 옮길 가능성까지 고려해 shared state를 mutex로 보호한다. 즉 이 코드는
// single-threaded executor 가정에 정확성을 의존하지 않는다.

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "moveit_msgs/msg/display_trajectory.hpp"
#include "moveit_msgs/msg/move_it_error_codes.hpp"
#include "moveit_msgs/srv/get_position_fk.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

class PlannedTcpPathNode : public rclcpp::Node
{
public:
  PlannedTcpPathNode()
  : Node("planned_tcp_path_node")
  {
    // rclpy는 기본값으로 parameter type을 추론하지만, rclcpp에서는 template 인자로 명시한다.
    declare_parameter<std::string>("base_frame", "base_link");
    declare_parameter<std::string>("tip_link", "tool0");
    declare_parameter<int64_t>("downsample", 1);

    base_frame_ = get_parameter("base_frame").as_string();
    tip_link_ = get_parameter("tip_link").as_string();
    downsample_ = std::max<int64_t>(1, get_parameter("downsample").as_int());

    fk_client_ = create_client<moveit_msgs::srv::GetPositionFK>("/compute_fk");
    plan_sub_ = create_subscription<moveit_msgs::msg::DisplayTrajectory>(
      "/move_group/display_planned_path", 10,
      [this](moveit_msgs::msg::DisplayTrajectory::ConstSharedPtr msg) {
        onPlan(*msg);
      });
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "/pick/planned_tcp_path", 10);

    RCLCPP_INFO(
      get_logger(), "planned_tcp_path_node 시작: tip_link=%s, base_frame=%s",
      tip_link_.c_str(), base_frame_.c_str());
  }

private:
  // Python 원본은 results와 pending(list로 감싼 int)을 closure에 capture한다. C++에서는
  // 요청 콜백 전부가 shared_ptr<BatchState>를 capture해 같은 batch의 수명과 상태를 공유한다.
  // optional<Point>는 Python의 None 또는 Point인 결과 슬롯을 1:1로 표현한다.
  struct BatchState
  {
    explicit BatchState(std::size_t size)
    : results(size), pending(size)
    {
    }

    std::vector<std::optional<geometry_msgs::msg::Point>> results;
    std::size_t pending;
  };

  void onPlan(const moveit_msgs::msg::DisplayTrajectory & msg)
  {
    // Python 원본과 동일하게 첫 trajectory만 사용하며, 빈 trajectory/points는 아무 것도 하지
    // 않는다. 빈 plan은 generation도 올리지 않으므로 진행 중인 유효 plan을 무효화하지 않는다.
    if (msg.trajectory.empty() || msg.trajectory.front().joint_trajectory.points.empty()) {
      return;
    }

    const auto & joint_trajectory = msg.trajectory.front().joint_trajectory;
    const auto & trajectory_points = joint_trajectory.points;

    // slice [::downsample]에 대응하는 index 목록. 마지막 waypoint가 sample에 걸리지 않아도
    // 반드시 추가한다. index를 쓰면 Python의 객체 identity 비교 대신 의도가 직접 드러난다.
    std::vector<std::size_t> selected_indices;
    const auto step = static_cast<std::size_t>(downsample_);
    for (std::size_t index = 0; index < trajectory_points.size(); index += step) {
      selected_indices.push_back(index);
    }
    const std::size_t last_index = trajectory_points.size() - 1;
    if (selected_indices.back() != last_index) {
      selected_indices.push_back(last_index);
    }

    uint64_t generation;
    {
      // 새 유효 plan의 generation 할당을 FK 응답 콜백의 검사와 같은 mutex 아래 둔다.
      // 따라서 old callback의 "검사 직후" 새 plan이 끼어들어 old Marker가 나중에 발행되는
      // check-then-act race도 생기지 않는다.
      std::lock_guard<std::mutex> lock(state_mutex_);
      generation = ++generation_;
    }

    auto batch = std::make_shared<BatchState>(selected_indices.size());

    // 각 요청을 기다리지 않고 전부 전송한다(fan-out). Python의 Future.add_done_callback과 달리
    // rclcpp는 async_send_request(request, callback) 한 호출에서 callback을 바로 등록한다.
    // 둘 다 polling은 하지 않으며, executor가 service response를 받을 때 callback을 실행한다.
    for (std::size_t result_index = 0; result_index < selected_indices.size(); ++result_index) {
      const auto & point = trajectory_points[selected_indices[result_index]];
      auto request = std::make_shared<moveit_msgs::srv::GetPositionFK::Request>();
      request->header.frame_id = base_frame_;
      request->fk_link_names = {tip_link_};
      request->robot_state.joint_state.name = joint_trajectory.joint_names;
      request->robot_state.joint_state.position = point.positions;

      fk_client_->async_send_request(
        request,
        [this, generation, result_index, batch](
          rclcpp::Client<moveit_msgs::srv::GetPositionFK>::SharedFuture future) {
          onFkDone(generation, result_index, batch, future);
        });
    }
  }

  void onFkDone(
    uint64_t generation,
    std::size_t result_index,
    const std::shared_ptr<BatchState> & batch,
    rclcpp::Client<moveit_msgs::srv::GetPositionFK>::SharedFuture future)
  {
    // generation 검사, result 기록, pending 감소, 마지막 publish까지 같은 critical section이다.
    // MultiThreadedExecutor에서 FK 응답 N개와 새 plan callback이 섞여도 다음 불변식이 유지된다.
    //   1) old generation 결과는 현재 batch에 들어오지 않는다.
    //   2) pending==0을 관찰하고 publish하는 callback은 정확히 하나다.
    //   3) 새 plan이 generation을 올린 뒤 old generation Marker가 발행되지 않는다.
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (generation != generation_) {
      return;
    }

    const auto response = future.get();
    if (
      response &&
      response->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS &&
      !response->pose_stamped.empty())
    {
      batch->results[result_index] = response->pose_stamped.front().pose.position;
    }

    --batch->pending;
    if (batch->pending == 0) {
      publishLine(batch->results);
    }
  }

  void publishLine(const std::vector<std::optional<geometry_msgs::msg::Point>> & results)
  {
    std::vector<geometry_msgs::msg::Point> points;
    points.reserve(results.size());
    for (const auto & result : results) {
      if (result.has_value()) {
        points.push_back(*result);
      }
    }

    // 원본의 방어 로직: 성공한 FK point가 0개 또는 1개면 LINE_STRIP을 발행하지 않는다.
    if (points.size() < 2) {
      RCLCPP_WARN(
        get_logger(), "FK 성공한 waypoint가 %zu개뿐 — 라인을 그리지 않음", points.size());
      return;
    }

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = base_frame_;
    marker.header.stamp = now();
    marker.ns = "planned_tcp_path";
    marker.id = 0;  // 고정 id: 같은 ns/id의 이전 line을 새 plan마다 덮어쓴다.
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.008;
    marker.color.r = 0.1F;
    marker.color.g = 0.9F;
    marker.color.b = 0.2F;
    marker.color.a = 0.9F;
    marker.points = std::move(points);
    marker.lifetime.sec = 0;  // 0: 다음 덮어쓰기 전까지 계속 표시한다.
    marker_pub_->publish(marker);
  }

  std::string base_frame_;
  std::string tip_link_;
  int64_t downsample_{1};

  rclcpp::Client<moveit_msgs::srv::GetPositionFK>::SharedPtr fk_client_;
  rclcpp::Subscription<moveit_msgs::msg::DisplayTrajectory>::SharedPtr plan_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  std::mutex state_mutex_;
  uint64_t generation_{0};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannedTcpPathNode>());
  rclcpp::shutdown();
  return 0;
}
