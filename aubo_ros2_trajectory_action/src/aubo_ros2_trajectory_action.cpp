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

#include "aubo_ros2_trajectory_action.h"

using namespace aubo_ros2_trajectory_action;

JointTrajectoryAction::JointTrajectoryAction(std::string controller_name):Node("aubo_ros2_trajectory_action")
{
  using namespace std::placeholders;
  this->action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
    this, controller_name,
    std::bind(&JointTrajectoryAction::handleGoal, this, _1, _2),
    std::bind(&JointTrajectoryAction::handleCancel, this, _1),
    std::bind(&JointTrajectoryAction::handleAccept, this, _1));

  moveit_controller_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectoryPoint>("/aubo_robot/moveit_controller", 2000);
  fjt_feedback_sub_ = this->create_subscription<control_msgs::action::FollowJointTrajectory_Feedback>(
    "/aubo_robot/fjt_feedback", 10, std::bind(&JointTrajectoryAction::fjtFeedbackCallback, this, _1));
  moveit_execution_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/aubo_robot/moveit_execution", 10, std::bind(&JointTrajectoryAction::moveitExecutionCallback, this, _1));
}

void JointTrajectoryAction::fjtFeedbackCallback(const control_msgs::action::FollowJointTrajectory_Feedback::ConstSharedPtr msg)
{
  if (!has_active_goal_ || current_trajectory_.points.empty())
    return;

  if (checkReachTarget(msg, current_trajectory_))
  {
    RCLCPP_INFO(this->get_logger(), "reach target");
    auto result = std::make_shared<FollowJointTrajectory::Result>();
    result->SUCCESSFUL;
    active_goal_->succeed(result);
    has_active_goal_ = false; 
  }
}

void JointTrajectoryAction::moveitExecutionCallback(const std_msgs::msg::String::ConstSharedPtr msg)
{
  if (msg->data == "stop")
  {
    RCLCPP_INFO(this->get_logger(), "moveit execution stopped");

    if (has_active_goal_)
      abortActiveGoal();
  }
}

rclcpp_action::GoalResponse JointTrajectoryAction::handleGoal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const FollowJointTrajectory::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received new goal");

  if (goal->trajectory.points.empty())
  {
    RCLCPP_INFO(this->get_logger(), "Joint trajectory action failed on empty trajectory");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (has_active_goal_)
  {
    RCLCPP_INFO(this->get_logger(), "Received new goal, canceling current goal");
    abortActiveGoal();
  }

  (void)uuid;

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse JointTrajectoryAction::handleCancel(const std::shared_ptr<GoalHandleFjt> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "cancel goal");

  has_active_goal_ = false;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void JointTrajectoryAction::handleAccept(const std::shared_ptr<GoalHandleFjt> goal_handle)
{
  active_goal_ = std::move(goal_handle);
  current_trajectory_ = goal_handle->get_goal()->trajectory;
  has_active_goal_ = true;

  std::thread calculateThread(std::bind(&JointTrajectoryAction::calculateMotionTrajectory, this));
  calculateThread.detach();

  return;
}

void JointTrajectoryAction::abortActiveGoal()
{
  RCLCPP_INFO(this->get_logger(), "Marks the active goal as aborted");
  auto result = std::make_shared<FollowJointTrajectory::Result>();
  result->error_string = "aborted";
  active_goal_->abort(result);
  has_active_goal_ = false;
}

void JointTrajectoryAction::calculateMotionTrajectory()
{
  RCLCPP_INFO(this->get_logger(), "calculate trajectory and publish motion trajectory point");

  std::queue<trajectory_msgs::msg::JointTrajectoryPoint> motion_buffer, plan_motion_buffer;

  //sometimes current_trajectory.joint_names order: foreArm_joint, shoulder_joint, upperArm_joint, wrist1_joint, wrist2_joint, wrist3_joint
  //will change order: shoulder_joint, upperArm_joint, foreArm_joint, wrist1_joint, wrist2_joint, wrist3_joint

  trajectory_msgs::msg::JointTrajectory remap_traj = remapTrajectoryByJointName(current_trajectory_);

  for(uint64_t i = 0; i < remap_traj.points.size(); i++)
  {
    trajectory_msgs::msg::JointTrajectoryPoint point = remap_traj.points[i];
    motion_buffer.push(point);
  }

  RCLCPP_INFO(this->get_logger(), "trajectory: %ld time from start: %f", remap_traj.points.size(), toSec(remap_traj.points[remap_traj.points.size()-1].time_from_start));

  double move_duration, T, T2, T3, T4, T5, tt, ti, t1, t2, t3, t4, t5;
  double a1[6], a2[6], a3[6], a4[6], a5[6], h[6];

  trajectory_msgs::msg::JointTrajectoryPoint last_goal_point, current_goal_point, intermediate_goal_point;
  intermediate_goal_point.positions.resize(6);
  intermediate_goal_point.velocities.resize(6);
  intermediate_goal_point.accelerations.resize(6);

  last_goal_point = motion_buffer.front();

  int motion_size = motion_buffer.size();

  for (int i = 0; i < motion_size - 1; i++)
  {
    motion_buffer.pop();
    current_goal_point = motion_buffer.front();

    T = toSec(current_goal_point.time_from_start) - toSec(last_goal_point.time_from_start);
    memcpy(a1, &last_goal_point.velocities[0], 6 * sizeof(double));
    T2 = T * T;
    T3 = T2 * T;
    T4 = T3 * T;
    T5 = T4 * T;
    for (int j = 0; j < 6; j++)
    {
      a2[j] = 0.5 * last_goal_point.accelerations[j];
      h[j] = current_goal_point.positions[j] - last_goal_point.positions[j];
      a3[j] = 0.5 / T3 * (20 * h[j] - (8 * current_goal_point.velocities[j] + 12 * last_goal_point.velocities[j]) * T - (3 * last_goal_point.accelerations[j] - current_goal_point.accelerations[j]) * T2);
      a4[j] = 0.5 / T4 * (-30 * h[j] + (14 * current_goal_point.velocities[j] + 16 * last_goal_point.velocities[j]) * T + (3 * last_goal_point.accelerations[j] - 2 * current_goal_point.accelerations[j]) * T2);
      a5[j] = 0.5 / T5 * (12 * h[j] - 6 * (current_goal_point.velocities[j] + last_goal_point.velocities[j]) * T + (current_goal_point.accelerations[j] - last_goal_point.accelerations[j]) * T2);
    }
    tt = toSec(last_goal_point.time_from_start);
    ti = tt;
    while (tt < toSec(current_goal_point.time_from_start))
    {
      t1 = tt - ti;
      t2 = t1 * t1;
      t3 = t2 * t1;
      t4 = t3 * t1;
      t5 = t4 * t1;
      for (int j = 0; j < 6; j++)
      {
        intermediate_goal_point.positions[j] = last_goal_point.positions[j] + a1[j] * t1 + a2[j] * t2 + a3[j] * t3 + a4[j] * t4 + a5[j] * t5;
        intermediate_goal_point.velocities[j] = a1[j] + 2 * a2[j] * t1 + 3 * a3[j] * t2 + 4 * a4[j] * t3 + 5 * a5[j] * t4;
        intermediate_goal_point.accelerations[j] = 2 * a2[j] + 6 * a3[j] * t1 + 12 * a4[j] * t2 + 20 * a5[j] * t3;
      }
      tt += 1.0 / 200;

      plan_motion_buffer.push(intermediate_goal_point);

      if (has_active_goal_)
        moveit_controller_pub_->publish(intermediate_goal_point);  
    }
    move_duration = toSec(current_goal_point.time_from_start) - tt;
    plan_motion_buffer.push(current_goal_point);
    last_goal_point = current_goal_point;
    if (has_active_goal_)
      moveit_controller_pub_->publish(current_goal_point);
  }


  RCLCPP_INFO(this->get_logger(), "send trajectory %ld trajectory point finished", plan_motion_buffer.size());
}

double JointTrajectoryAction::toSec(const builtin_interfaces::msg::Duration &duration)
{
  return (double)duration.sec + 1e-9*(double)duration.nanosec;
}

bool JointTrajectoryAction::checkReachTarget(const control_msgs::action::FollowJointTrajectory_Feedback::ConstSharedPtr feedback, const trajectory_msgs::msg::JointTrajectory &traj)
{
  bool ret = false;
  int last_point = traj.points.size() - 1;
  ret = true;
  
  for (int i = 0; i < 6; i++)
  {
    if(abs(feedback->actual.positions[i] - traj.points[last_point].positions[i]) > fabs(0.001))
    {
      ret = false;
      break;
    }
  }

  return ret;
}

trajectory_msgs::msg::JointTrajectory JointTrajectoryAction::remapTrajectoryByJointName(trajectory_msgs::msg::JointTrajectory &trajectory)
{
  std::string joint_name_[6] = {"shoulder_joint","upperArm_joint","foreArm_joint","wrist1_joint","wrist2_joint","wrist3_joint"};
  std::vector<int> mapping;

  mapping.resize(6, 6);
	for (uint16_t i = 0; i < trajectory.joint_names.size(); i++) {
		for (int j = 0; j < 6; j++) {
			if (trajectory.joint_names[i] == joint_name_[j])
				mapping[j] = i;
		}
	}

  trajectory_msgs::msg::JointTrajectory new_traj;
  for (int i = 0; i < 6; i++)
    new_traj.joint_names.push_back(joint_name_[i]);

  for (uint16_t i = 0; i < trajectory.points.size(); i++)
  {
    trajectory_msgs::msg::JointTrajectoryPoint new_point;
    for(int j = 0; j < 6; j++)
    {
      new_point.positions.push_back(trajectory.points[i].positions[mapping[j]]);
      new_point.velocities.push_back(trajectory.points[i].velocities[mapping[j]]);
      new_point.accelerations.push_back(trajectory.points[i].accelerations[mapping[j]]);
    }
    new_point.time_from_start = trajectory.points[i].time_from_start;
    new_traj.points.push_back(new_point);
  }

  return new_traj;
}
