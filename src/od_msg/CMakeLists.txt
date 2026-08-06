cmake_minimum_required(VERSION 3.8)
project(od_msg)

# ament 및 rosidl 설정
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# 메시지/서비스 인터페이스 생성
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SrvDepthPosition.srv"
  "srv/SrvSegmentation.srv"
  "msg/SegmentationResult.msg"
  DEPENDENCIES sensor_msgs std_msgs
)

# 런타임 의존성 내보내기
ament_export_dependencies(rosidl_default_runtime)

ament_package()
