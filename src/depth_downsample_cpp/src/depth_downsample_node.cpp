// depth_downsample_node.py의 C++ 포팅(학습용).
// 원본: cobot_rg2/rg2/m0609_rg2_bringup/scripts/depth_downsample_node.py
// 원본 Python 파일은 참고만 하며 이 패키지에서 수정하지 않는다.
//
// 이 노드는 point cloud/PCL을 다루지 않는다. sensor_msgs/Image의 2D depth pixels를 OpenCV
// cv::Mat으로 보고 리사이즈한 뒤, 그 새 해상도에 맞게 CameraInfo intrinsics를 고친다.

#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include <opencv2/imgproc.hpp>

#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

class DepthDownsampleNode : public rclcpp::Node
{
public:
  DepthDownsampleNode()
  : Node("depth_downsample_node")
  {
    // rclpy에서는 p(name, default).value로 Parameter를 꺼냈다. rclcpp에서는
    // declare_parameter<T>()가 곧바로 T 값을 돌려준다. ROS integer parameter의 C++ 표현은
    // int64_t이므로 target_width/height도 그 타입으로 선언한다.
    const auto in_image_topic = declare_parameter<std::string>(
      "in_image_topic", "/camera/camera/aligned_depth_to_color/image_raw");
    const auto in_info_topic = declare_parameter<std::string>(
      "in_info_topic", "/camera/camera/aligned_depth_to_color/camera_info");
    const auto out_image_topic = declare_parameter<std::string>(
      "out_image_topic", "/cumotion/depth_1/image");
    const auto out_info_topic = declare_parameter<std::string>(
      "out_info_topic", "/cumotion/depth_1/camera_info");
    target_width_ = declare_parameter<int64_t>("target_width", 424);
    target_height_ = declare_parameter<int64_t>("target_height", 240);

    // Python qos_profile_sensor_data와 대응한다: KeepLast(5), BestEffort, Volatile.
    const auto sensor_qos = rclcpp::SensorDataQoS();
    image_pub_ = create_publisher<sensor_msgs::msg::Image>(out_image_topic, sensor_qos);
    info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(out_info_topic, sensor_qos);

    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      in_info_topic, sensor_qos,
      std::bind(&DepthDownsampleNode::onInfo, this, std::placeholders::_1));
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      in_image_topic, sensor_qos,
      std::bind(&DepthDownsampleNode::onDepth, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "%s -> %s (%ldx%ld, INTER_NEAREST)",
      in_image_topic.c_str(), out_image_topic.c_str(), target_width_, target_height_);
  }

private:
  void onInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
  {
    // Python의 self._latest_info = msg와 같은 역할. 기본 rclcpp::spin()은 SingleThreadedExecutor라
    // 두 subscription callback이 동시에 이 포인터를 바꾸지 않는다. MultiThreadedExecutor로
    // 바꾼다면 이 공유 상태에는 mutex 등 별도 동기화가 필요하다.
    latest_info_ = msg;
  }

  void onDepth(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    // 원본 구현의 실제 순서도 이렇다: CameraInfo가 먼저 오지 않았으면 image를 변환하거나
    // 발행하지 않는다. K/P를 올바르게 스케일할 기준이 없으므로 첫 프레임들은 의도적으로 버린다.
    if (!latest_info_) {
      return;
    }

    // Python: bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    // C++: 빈 encoding의 toCvCopy()가 원본 encoding을 유지한다(설치된 cv_bridge 헤더로 확인).
    // toCvCopy는 새 cv::Mat data를 만든다. cv::Mat은 numpy ndarray처럼 shape/stride를 갖지만,
    // numpy의 depth.shape[:2]나 reshape/flatten 대신 rows/cols와 고정 길이 배열 접근을 쓴다.
    const auto depth_cv = cv_bridge::toCvCopy(msg, "");
    const int source_height = depth_cv->image.rows;
    const int source_width = depth_cv->image.cols;

    cv::Mat resized;
    if (source_width == target_width_ && source_height == target_height_) {
      // Python의 resized = depth와 동등하다. cv::Mat 대입은 pixel buffer를 복사하지 않는
      // shallow header copy다. 아래 toImageMsg()가 ROS 출력용 pixel data를 별도로 복사한다.
      resized = depth_cv->image;
    } else {
      // Depth edge에서 foreground/background 값을 섞어 존재하지 않는 깊이를 만들지 않도록
      // 반드시 nearest-neighbor를 쓴다. INTER_LINEAR 등으로 바꾸면 원본의 물리적 invariant가 깨진다.
      cv::resize(
        depth_cv->image, resized,
        cv::Size(static_cast<int>(target_width_), static_cast<int>(target_height_)),
        0.0, 0.0, cv::INTER_NEAREST);
    }

    // Python: bridge.cv2_to_imgmsg(resized, encoding=msg.encoding); out.header = msg.header
    // C++ CvImage는 (header, encoding, cv::Mat)을 묶고 toImageMsg()로 새 ROS Image를 만든다.
    // encoding을 고정 문자열로 바꾸지 않고 입력 msg의 문자열 그대로 보존한다.
    cv_bridge::CvImage out_cv(msg->header, msg->encoding, resized);
    image_pub_->publish(*out_cv.toImageMsg());

    const double sx = static_cast<double>(target_width_) / source_width;
    const double sy = static_cast<double>(target_height_) / source_height;

    // numpy에서는 np.array(info.k).reshape(3, 3) 후 k[0, 0]처럼 2D로 접근했다.
    // ROS CameraInfo::k는 C++에서 std::array<double, 9>인 row-major 1D 배열이다. Eigen을
    // 쓰지 않아도 k[0], k[4], k[2], k[5]가 각각 (0,0), (1,1), (0,2), (1,2)에 대응한다.
    // 복사 후 필요한 네 원소만 바꾸므로 원본 latest_info_ 메시지는 절대 수정하지 않는다.
    sensor_msgs::msg::CameraInfo out_info;
    out_info.header = msg->header;
    out_info.width = static_cast<uint32_t>(target_width_);
    out_info.height = static_cast<uint32_t>(target_height_);
    out_info.distortion_model = latest_info_->distortion_model;
    out_info.d = latest_info_->d;  // D는 스케일하지 않는다.
    out_info.k = latest_info_->k;
    out_info.k[0] *= sx;  // fx = K(0,0)
    out_info.k[4] *= sy;  // fy = K(1,1)
    out_info.k[2] *= sx;  // cx = K(0,2)
    out_info.k[5] *= sy;  // cy = K(1,2)

    // P는 3x4 row-major 배열(원소 12개)이다. numpy의 reshape(3, 4)/flatten()을 하지 않고
    // 같은 row-major index를 직접 쓴다. P의 fx/fy/cx/cy 자리도 K와 같은 축소비를 적용한다.
    out_info.p = latest_info_->p;
    out_info.p[0] *= sx;   // P(0,0): fx
    out_info.p[5] *= sy;   // P(1,1): fy
    out_info.p[2] *= sx;   // P(0,2): cx
    out_info.p[6] *= sy;   // P(1,2): cy
    out_info.r = latest_info_->r;  // R도 스케일하지 않는다.

    info_pub_->publish(out_info);
  }

  int64_t target_width_{};
  int64_t target_height_{};

  sensor_msgs::msg::CameraInfo::ConstSharedPtr latest_info_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DepthDownsampleNode>());
  rclcpp::shutdown();
  return 0;
}
