cmake_minimum_required(VERSION 3.8)
project(gripper_virtual_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(onrobot_rg_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(gripper_virtual_node src/gripper_virtual_node.cpp)
ament_target_dependencies(gripper_virtual_node
  onrobot_rg_msgs
  rclcpp
  sensor_msgs)

install(TARGETS gripper_virtual_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
