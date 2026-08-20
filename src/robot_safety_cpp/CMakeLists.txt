cmake_minimum_required(VERSION 3.8)
project(robot_safety_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(dsr_msgs2 REQUIRED)

add_executable(robot_safety_node src/robot_safety_node.cpp)
ament_target_dependencies(robot_safety_node
  rclcpp std_msgs std_srvs dsr_msgs2)

install(TARGETS robot_safety_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
