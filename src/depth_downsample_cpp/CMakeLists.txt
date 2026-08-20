cmake_minimum_required(VERSION 3.8)
project(depth_downsample_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)

add_executable(depth_downsample_node src/depth_downsample_node.cpp)
ament_target_dependencies(depth_downsample_node rclcpp sensor_msgs cv_bridge)
target_link_libraries(depth_downsample_node ${OpenCV_LIBS})

install(TARGETS depth_downsample_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
