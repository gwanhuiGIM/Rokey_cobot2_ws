cmake_minimum_required(VERSION 3.8)
project(planned_tcp_path_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(planned_tcp_path_node src/planned_tcp_path_node.cpp)
ament_target_dependencies(planned_tcp_path_node
  geometry_msgs
  moveit_msgs
  rclcpp
  visualization_msgs)

install(TARGETS planned_tcp_path_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
