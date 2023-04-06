cmake_minimum_required(VERSION 3.5)
project(aubo_ros2_driver)

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
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(aubo_ros2_common REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(aubo_ros2_driver src/aubo_sdk/aubo_sdk.cpp src/aubo_sdk/aubo_thread.cpp src/aubo_ros2_driver.cpp src/main.cpp)

target_include_directories(aubo_ros2_driver
  PRIVATE
  include
)

target_link_libraries(aubo_ros2_driver
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/lib64/libotgLib.a 
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/lib64/aubocontroller/libauborobotcontroller.so)

ament_target_dependencies(aubo_ros2_driver
  rclcpp
  control_msgs trajectory_msgs aubo_ros2_common sensor_msgs)

install(DIRECTORY
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/lib64/

  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  aubo_ros2_driver
  DESTINATION lib/${PROJECT_NAME})

ament_package()
