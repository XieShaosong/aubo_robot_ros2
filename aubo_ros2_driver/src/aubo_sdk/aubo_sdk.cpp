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

#include "aubo_ros2_driver.h"

using namespace aubo_ros2_driver;

bool AuboRos2Driver::connectArmController()
{
  int ret1 = aubo_robot_namespace::InterfaceCallSuccCode;
  int ret2 = aubo_robot_namespace::InterfaceCallSuccCode;

  string server_host = "192.168.29.2";

  //log in
  int max_link_times = 5;
  int count = 0;
  do
  {
    count ++;
    ret1 = robot_send_service_.robotServiceLogin(server_host.c_str(), 8899, "aubo", "123456");
  } while (ret1 != aubo_robot_namespace::InterfaceCallSuccCode && count < max_link_times);
  
  if (ret1 == aubo_robot_namespace::InterfaceCallSuccCode)
  {
    ret2 = robot_receive_service_.robotServiceLogin(server_host.c_str(), 8899, "aubo", "123456");
    controller_connected_flag_ = true;
    RCLCPP_INFO(this->get_logger(), "login success.");

    ret2 = robot_receive_service_.robotServiceGetIsRealRobotExist(real_robot_exist_);
    if (ret2 == aubo_robot_namespace::InterfaceCallSuccCode)
    {
      if (real_robot_exist_)
        RCLCPP_INFO(this->get_logger(), "real robot exist.");
      else
        RCLCPP_INFO(this->get_logger(), "real robot does not exist.");
    }
  }
  else
  {
    controller_connected_flag_ = false;
    RCLCPP_INFO(this->get_logger(), "login failed.");
    return false;
  }
  
  return true;
}

bool AuboRos2Driver::jointMove(std::vector<double> &target_joints, std::vector<double> &max_vel, std::vector<double> &max_acc)
{
  int ret = aubo_robot_namespace::InterfaceCallSuccCode;
  bool result = false;

  ret = robot_send_service_.robotServiceLeaveTcp2CanbusMode();
  if (ret == aubo_robot_namespace::InterfaceCallSuccCode)
  {
    control_option_ = aubo_ros2_driver::AuboAPI;
  }
  else
    return false;
  
  ret = robot_send_service_.robotServiceInitGlobalMoveProfile();

  aubo_robot_namespace::JointVelcAccParam jointMaxAcc;
  aubo_robot_namespace::JointVelcAccParam jointMaxVelc;

  for (int i = 0; i < 6; i++)
  {
    jointMaxAcc.jointPara[i] = max_acc[i];
    jointMaxVelc.jointPara[i] = max_vel[i];
  }

  ret = robot_send_service_.robotServiceSetGlobalMoveJointMaxAcc(jointMaxAcc);
  ret = robot_send_service_.robotServiceSetGlobalMoveJointMaxVelc(jointMaxVelc);

  double joints[6];
  for (int i = 0; i < 6; i++)
  {
    joints[i] = target_joints[i];
  }

  ret = robot_send_service_.robotServiceJointMove(joints, false);

  if (ret == aubo_robot_namespace::InterfaceCallSuccCode)
  {
    result = true;
    RCLCPP_INFO(this->get_logger(), "joint move success.");
  }
  else
  {
    result = false;
    RCLCPP_INFO(this->get_logger(), "joint move failed. errCode: %d", ret);
  }

  ret = robot_send_service_.robotServiceEnterTcp2CanbusMode();
  if (ret == aubo_robot_namespace::InterfaceCallSuccCode)
  {
    control_option_ = aubo_ros2_driver::RosMoveIt;
  }
  
  return result;
}

 void AuboRos2Driver::handleArmStopped()
 {
  robot_send_service_.robotMoveFastStop();
  if(moveit_controller_queue_.size() > 0)
  {
    std_msgs::msg::String msg;
    msg.data = "stop";
    moveit_execution_pub_->publish(msg);
  }

  while (moveit_controller_queue_.size() > 0)
  {
    moveit_controller_queue_.clear();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  RCLCPP_INFO(this->get_logger(), "handle arm stopped");
}
