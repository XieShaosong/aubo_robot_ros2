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
