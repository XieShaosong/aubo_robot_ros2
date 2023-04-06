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

void AuboRos2Driver::moveitControllerThread()
{
  rclcpp::Rate loop_rate(UPDATE_RATE_);

  while (rclcpp::ok())
  {
    if(moveit_controller_queue_.size() > 0 && !start_move_)
      start_move_ = true;
    
    if (start_move_ && rib_buffer_size_ < MINIMUM_BUFFER_SIZE)
    {
      if (moveit_controller_queue_.size() > 0)
      {
        PlanningState ps;
        moveit_controller_queue_.pop(ps);
        robot_send_service_.robotServiceSetRobotPosData2Canbus(ps.joint_pos_);
      }
      else
        start_move_ = false;
    }

    loop_rate.sleep();
  }
  
}

bool AuboRos2Driver::checkReachTarget()
{
  bool ret = true;
  for (int i = 0; i < ARM_DOF; i++)
  {
    if(fabs(target_joints_[i] - actual_joints_[i]) > 0.001)
    {
      ret = false;
      break;
    }
  }
  return ret;
}
