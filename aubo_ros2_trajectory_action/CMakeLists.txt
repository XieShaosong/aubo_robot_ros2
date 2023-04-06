cmake_minimum_required(VERSION 3.5)
project(aubo_ros2_trajectory_action)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_executable(aubo_ros2_trajectory_action src/aubo_ros2_trajectory_action.cpp src/main.cpp)
ament_target_dependencies(aubo_ros2_trajectory_action
  rclcpp
  rclcpp_action control_msgs trajectory_msgs)

target_include_directories(aubo_ros2_trajectory_action
  PRIVATE
  include
)

install(TARGETS
  aubo_ros2_trajectory_action
  DESTINATION lib/${PROJECT_NAME})

ament_package()
