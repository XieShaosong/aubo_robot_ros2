#include "aubo_ros2_driver.h"

using namespace aubo_ros2_driver;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
  auto aubo_driver = std::make_shared<AuboRos2Driver>();
  bool ret = aubo_driver->start();
  if(!ret)
  {
    return 0;
  }

  rclcpp::spin(aubo_driver);
  rclcpp::shutdown();
  return 0;
}