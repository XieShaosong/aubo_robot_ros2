cmake_minimum_required(VERSION 3.8)
project(aubo_ros2_common)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AuboJointStates.msg"
  "msg/AuboTcpPose.msg"
  "msg/AuboArmEvent.msg"
  "msg/AuboArmStates.msg"
  "srv/AuboArmControl.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

install(DIRECTORY
  ${CMAKE_CURRENT_SOURCE_DIR}/include/

  DESTINATION include/${PROJECT_NAME}
)

ament_package()
