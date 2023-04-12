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

#ifndef AUBO_JOINT_TRAJECTORY_ACTION_H
#define AUBO_JOINT_TRAJECTORY_ACTION_H

#include <memory>
#include <queue>
#include <thread>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>

#include "std_msgs/msg/string.hpp"

namespace aubo_ros2_trajectory_action
{

class JointTrajectoryAction : public rclcpp::Node
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFjt = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

  JointTrajectoryAction(std::string controller_name);

private:
  rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr moveit_controller_pub_;
  rclcpp::Subscription<control_msgs::action::FollowJointTrajectory_Feedback>::SharedPtr fjt_feedback_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr moveit_execution_sub_;

  bool has_active_goal_;
  trajectory_msgs::msg::JointTrajectory current_trajectory_;
  std::shared_ptr<GoalHandleFjt> active_goal_;

  void fjtFeedbackCallback(const control_msgs::action::FollowJointTrajectory_Feedback::ConstSharedPtr msg);
  void moveitExecutionCallback(const std_msgs::msg::String::ConstSharedPtr msg);

  rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const FollowJointTrajectory::Goal> goal);
  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandleFjt> goal_handle);
  void handleAccept(const std::shared_ptr<GoalHandleFjt> goal_handle);

  void abortActiveGoal();

  void calculateMotionTrajectory();
  double toSec(const builtin_interfaces::msg::Duration &duration);

  bool isSimilar(std::vector<std::string> lhs, std::vector<std::string> rhs);

  trajectory_msgs::msg::JointTrajectory remapTrajectoryByJointName(trajectory_msgs::msg::JointTrajectory &trajectory);

  bool checkReachTarget(const control_msgs::action::FollowJointTrajectory_Feedback::ConstSharedPtr feedback, const trajectory_msgs::msg::JointTrajectory &traj);

  std::vector<std::string> joint_names;
};

}

#endif
