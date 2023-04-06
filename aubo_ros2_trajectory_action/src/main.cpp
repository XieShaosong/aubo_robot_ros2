#include "aubo_ros2_trajectory_action.h"

using namespace aubo_ros2_trajectory_action;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::string controller_name = "aubo_i5_controller/follow_joint_trajectory";
  auto action_server = std::make_shared<JointTrajectoryAction>(controller_name);

  rclcpp::spin(action_server);

  rclcpp::shutdown();
  return 0;
}
