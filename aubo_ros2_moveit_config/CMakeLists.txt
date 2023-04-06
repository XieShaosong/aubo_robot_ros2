cmake_minimum_required(VERSION 3.5)
project(aubo_ros2_moveit_config)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

install(
  DIRECTORY config srdf rviz launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
