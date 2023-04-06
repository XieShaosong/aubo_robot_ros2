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