// Copyright 2023 Xie Shaosong
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUBO_DRIVER_H_
#define AUBO_DRIVER_H_

#include <string>
#include <thread>

#include "aubo_ros2_common/threadSafeQueue.h"

#include "AuboRobotMetaType.h"
#include "serviceinterface.h"
#include "robot_state.h"
#include "robotiomatetype.h"
#include "auboroboterror.h"
#include "auborobotevent.h"

#include "aubo_ros2_metaType.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>

#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "aubo_ros2_common/msg/aubo_arm_event.hpp"
#include "aubo_ros2_common/msg/aubo_arm_states.hpp"
#include "aubo_ros2_common/msg/aubo_joint_states.hpp"
#include "aubo_ros2_common/msg/aubo_tcp_pose.hpp"

#include "aubo_ros2_common/srv/aubo_arm_control.hpp"

using namespace std;

namespace aubo_ros2_driver
{

class AuboRos2Driver : public rclcpp::Node
{
public:
  AuboRos2Driver();

  bool start();

private:
  bool connectArmController();
  bool jointMove(std::vector<double> &target_joints, std::vector<double> &max_vel, std::vector<double> &max_acc);
  void handleArmStopped();

  void moveitControllerThread();
  bool checkReachTarget();

private:
  rclcpp::TimerBase::SharedPtr states_pub_timer_;
  rclcpp::Publisher<aubo_ros2_common::msg::AuboJointStates>::SharedPtr joint_states_pub_;
  rclcpp::Publisher<aubo_ros2_common::msg::AuboTcpPose>::SharedPtr tcp_pose_pub_;
  rclcpp::Publisher<aubo_ros2_common::msg::AuboArmStates>::SharedPtr arm_states_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr sensor_joint_states_pub_;
  rclcpp::Publisher<control_msgs::action::FollowJointTrajectory_Feedback>::SharedPtr fjt_feedback_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr moveit_execution_pub_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_control_sub_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr moveit_controller_sub_;

  rclcpp::Service<aubo_ros2_common::srv::AuboArmControl>::SharedPtr arm_control_srv_;

  void intervalStatesCallback();

  void armControlServiceCallback(const std::shared_ptr<aubo_ros2_common::srv::AuboArmControl::Request> request,
                                 std::shared_ptr<aubo_ros2_common::srv::AuboArmControl::Response> response);

  void moveitControllerCallback(const trajectory_msgs::msg::JointTrajectoryPoint::ConstSharedPtr msg);

  void robotControlCallback(const std_msgs::msg::String::ConstSharedPtr msg);

private:
  ServiceInterface robot_send_service_;
  ServiceInterface robot_receive_service_;

  bool real_robot_exist_;
  bool controller_connected_flag_;
  int control_option_;

  RobotState robot_state_;
  
  aubo_robot_namespace::ToolKinematicsParam tcp_;

  double target_joints_[6];
  double actual_joints_[6];

  bool start_move_;

  ThreadSafeQueue<PlanningState> moveit_controller_queue_;

  std::thread *moveit_controller_thread_;

  int rib_buffer_size_;

  ArmStopped arm_stopped_;
};

}

#endif
