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

AuboRos2Driver::AuboRos2Driver():Node("aubo_ros2_driver")
{
  using namespace std::placeholders;
  controller_connected_flag_ = false;
  real_robot_exist_ = false;

  this->declare_parameter<std::string>("robot_ip", "127.0.0.1");

  tcp_.toolInEndOrientation.w = 1;
  tcp_.toolInEndOrientation.x = 0;
  tcp_.toolInEndOrientation.y = 0;
  tcp_.toolInEndOrientation.z = 0;
  tcp_.toolInEndPosition.x = 0;
  tcp_.toolInEndPosition.y = 0;
  tcp_.toolInEndPosition.z = 0;

  joint_states_pub_ = this->create_publisher<aubo_ros2_common::msg::AuboJointStates>("/aubo_robot/joint_states", 10);
  tcp_pose_pub_ = this->create_publisher<aubo_ros2_common::msg::AuboTcpPose>("/aubo_robot/tcp_pose", 10);
  arm_states_pub_ = this->create_publisher<aubo_ros2_common::msg::AuboArmStates>("/aubo_robot/arm_states", 10);

  sensor_joint_states_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  fjt_feedback_pub_ = this->create_publisher<control_msgs::action::FollowJointTrajectory_Feedback>("/aubo_robot/fjt_feedback", 10);
  moveit_execution_pub_ = this->create_publisher<std_msgs::msg::String>("/aubo_robot/moveit_execution", 10);

  moveit_controller_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectoryPoint>(
    "/aubo_robot/moveit_controller", 5000, std::bind(&AuboRos2Driver::moveitControllerCallback, this, _1));

  robot_control_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/aubo_robot/robot_control", 10, std::bind(&AuboRos2Driver::robotControlCallback, this, _1));

  arm_control_srv_ = this->create_service<aubo_ros2_common::srv::AuboArmControl>("/aubo_robot/arm_control", std::bind(&AuboRos2Driver::armControlServiceCallback, this, _1, _2));

  states_pub_timer_ = create_wall_timer(100ms, std::bind(&AuboRos2Driver::intervalStatesCallback, this));
}

bool AuboRos2Driver::start()
{
  RCLCPP_INFO(this->get_logger(), "start the driver.");

  //1.connect arm controller
  bool ret = connectArmController();

  if (!ret)
  {
    RCLCPP_INFO(this->get_logger(), "connect arm failed.");
    return false;
  }

  //2.switches to ros-controller
  int ret1 = robot_send_service_.robotServiceEnterTcp2CanbusMode();
  if (ret1 == aubo_robot_namespace::InterfaceCallSuccCode)
  {
    RCLCPP_INFO(this->get_logger(), "Switches to ros-controller successfully");
    control_option_ = aubo_ros2_driver::RosMoveIt;
  }
  else if (ret1 == aubo_robot_namespace::ErrCode_ResponseReturnError)
  {
    ret1 = robot_send_service_.robotServiceLeaveTcp2CanbusMode();
    ret1 = robot_send_service_.robotServiceEnterTcp2CanbusMode();
    if (ret1 == aubo_robot_namespace::InterfaceCallSuccCode)
    {
      RCLCPP_INFO(this->get_logger(), "Switches to ros-controller successfully");
      control_option_ = aubo_ros2_driver::RosMoveIt;
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Failed to switch to ros-controller, the robot is still controlled by the robot controller!");
      control_option_ = aubo_ros2_driver::AuboAPI;
    }
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "Failed to switch to ros-controller, the robot is still controlled by the robot controller!");
    control_option_ = aubo_ros2_driver::AuboAPI;
  }

  moveit_controller_thread_ = new std::thread(std::bind(&AuboRos2Driver::moveitControllerThread, this));
  moveit_controller_thread_->detach();

  RCLCPP_INFO(this->get_logger(), "aubo driver started.");
  return true;
}

void AuboRos2Driver::intervalStatesCallback()
{
  if (controller_connected_flag_ && real_robot_exist_)
  {
    int ret = robot_receive_service_.robotServiceGetCurrentWaypointInfo(robot_state_.wayPoint_);
    if (ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
      for (int i = 0; i < 6; i++)
        actual_joints_[i] = robot_state_.wayPoint_.jointpos[i];
    }

    // get joint states
    robot_receive_service_.robotServiceGetRobotJointStatus(robot_state_.joint_status_, 6);
    aubo_ros2_common::msg::AuboJointStates joint_states;
    for (int i = 0; i < 6; i++)
    {
      joint_states.actual_current.push_back(robot_state_.joint_status_[i].jointCurrentI);
      joint_states.target_current.push_back(robot_state_.joint_status_[i].jointTagCurrentI);
      joint_states.actual_position.push_back(robot_state_.joint_status_[i].jointPosJ);
      joint_states.target_position.push_back(robot_state_.joint_status_[i].jointTagPosJ);
    }
    joint_states_pub_->publish(joint_states);

    // get tcp pose
    aubo_robot_namespace::Pos actual_toolEndPositionOnBase;
    aubo_robot_namespace::Ori actual_toolEndOrientationOnBase;
    robot_receive_service_.baseToBaseAdditionalTool(robot_state_.wayPoint_.cartPos.position, robot_state_.wayPoint_.orientation, tcp_, actual_toolEndPositionOnBase, actual_toolEndOrientationOnBase);
    aubo_ros2_common::msg::AuboTcpPose tcp_pose;
    tcp_pose.actual_pose.position.x = actual_toolEndPositionOnBase.x;
    tcp_pose.actual_pose.position.y = actual_toolEndPositionOnBase.y;
    tcp_pose.actual_pose.position.z = actual_toolEndPositionOnBase.z;
    tcp_pose.actual_pose.orientation.w = actual_toolEndOrientationOnBase.w;
    tcp_pose.actual_pose.orientation.x = actual_toolEndOrientationOnBase.x;
    tcp_pose.actual_pose.orientation.y = actual_toolEndOrientationOnBase.y;
    tcp_pose.actual_pose.orientation.z = actual_toolEndOrientationOnBase.z;
    tcp_pose_pub_->publish(tcp_pose);

    // get arm states
    aubo_ros2_common::msg::AuboArmStates arm_states;

    robot_receive_service_.robotServiceGetRobotDiagnosisInfo(robot_state_.robot_diagnosis_info_);
    rib_buffer_size_ = robot_state_.robot_diagnosis_info_.macTargetPosDataSize;
    arm_states.control_option = control_option_;
    int collision_level = 0;
    robot_receive_service_.robotServiceGetRobotCollisionCurrentService(collision_level);
    arm_states.collision_level = collision_level;
    arm_states.arm_power_status = robot_state_.robot_diagnosis_info_.armPowerStatus;
    arm_states.collision_stopped = robot_state_.robot_diagnosis_info_.robotCollision;
    arm_states.emergency_stopped = robot_state_.robot_diagnosis_info_.softEmergency;
    arm_states.singularity_stopped = robot_state_.robot_diagnosis_info_.singularityOverSpeedAlarm;
    arm_states.protective_stopped = false;
    arm_states.reached_target = true;
    bool is_timeout;
    robot_receive_service_.robotServiceGetConnectStatus(is_timeout);
    arm_states.is_timeout = is_timeout;
    arm_states.in_motion = start_move_;
    arm_states_pub_->publish(arm_states);

    if (arm_states.emergency_stopped || arm_states.collision_stopped || arm_stopped_.protective_stopped || arm_states.singularity_stopped)
    {
      handleArmStopped();
    }

    //pub sensor joint states
    sensor_msgs::msg::JointState sensor_joint_states;
    sensor_joint_states.header.stamp = this->now();
    sensor_joint_states.name.resize(ARM_DOF);
    sensor_joint_states.position.resize(ARM_DOF);
    for (int i = 0; i < 6; i++)
    {
      sensor_joint_states.name[i] = joint_name_[i];
      sensor_joint_states.position[i] = robot_state_.joint_status_[i].jointPosJ;
    }
    sensor_joint_states_pub_->publish(sensor_joint_states);

    //pub fjt joint feedback
    control_msgs::action::FollowJointTrajectory_Feedback joint_feedback;
    joint_feedback.header.stamp = this->now();
    for (int i = 0; i < 6; i++)
    {
      joint_feedback.joint_names.push_back(joint_name_[i]);
      joint_feedback.actual.positions.push_back(robot_state_.joint_status_[i].jointPosJ);
    }
    fjt_feedback_pub_->publish(joint_feedback);
  }
}

void AuboRos2Driver::robotControlCallback(const std_msgs::msg::String::ConstSharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "receive cmd: %s", msg->data.c_str());
  int ret = aubo_robot_namespace::InterfaceCallSuccCode;

  if (msg->data == "powerOn")
  {
    int collision_level = 7;
    aubo_robot_namespace::ToolDynamicsParam toolDynamicsParam;
    toolDynamicsParam.positionX = 0.0;
    toolDynamicsParam.positionY = 0.0;
    toolDynamicsParam.positionZ = 0.0;
    toolDynamicsParam.payload = 1.0;

    aubo_robot_namespace::ROBOT_SERVICE_STATE result;
    ret = robot_send_service_.rootServiceRobotStartup(toolDynamicsParam /**工具动力学参数**/,
                                                      collision_level /*碰撞等级*/,
                                                      true /*是否允许读取位姿　默认为true*/,
                                                      true,    /*保留默认为true */
                                                      1000,    /*保留默认为1000 */
                                                      result); /*机械臂初始化*/

    if (ret == aubo_robot_namespace::InterfaceCallSuccCode)
      RCLCPP_INFO(this->get_logger(), "powerOn success.");
    else
      RCLCPP_INFO(this->get_logger(), "powerOn failed.");
  }
  else if (msg->data == "powerOff")
  {
    ret = robot_send_service_.robotServiceRobotShutdown(true);

    if (ret == aubo_robot_namespace::InterfaceCallSuccCode)
      RCLCPP_INFO(this->get_logger(), "powerOff success.");
    else
      RCLCPP_INFO(this->get_logger(), "powerOff failed.");
  }
}

void AuboRos2Driver::armControlServiceCallback(const std::shared_ptr<aubo_ros2_common::srv::AuboArmControl::Request> request,
                                               std::shared_ptr<aubo_ros2_common::srv::AuboArmControl::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "request cmd: %s", request->cmd.c_str());

  int ret = aubo_robot_namespace::InterfaceCallSuccCode;

  if (request->cmd == "powerOn")
  {
    int collision_level = 7;
    aubo_robot_namespace::ToolDynamicsParam toolDynamicsParam;
    toolDynamicsParam.positionX = 0.0;
    toolDynamicsParam.positionY = 0.0;
    toolDynamicsParam.positionZ = 0.0;
    toolDynamicsParam.payload = 1.0;

    aubo_robot_namespace::ROBOT_SERVICE_STATE result;
    ret = robot_send_service_.rootServiceRobotStartup(toolDynamicsParam /**工具动力学参数**/,
                                                      collision_level /*碰撞等级*/,
                                                      true /*是否允许读取位姿　默认为true*/,
                                                      true,    /*保留默认为true */
                                                      1000,    /*保留默认为1000 */
                                                      result); /*机械臂初始化*/

    if (ret == aubo_robot_namespace::InterfaceCallSuccCode)
      RCLCPP_INFO(this->get_logger(), "powerOn success.");
    else
      RCLCPP_INFO(this->get_logger(), "powerOn failed.");
  }
  else if (request->cmd == "powerOff")
  {
    ret = robot_send_service_.robotServiceRobotShutdown(true);

    if (ret == aubo_robot_namespace::InterfaceCallSuccCode)
      RCLCPP_INFO(this->get_logger(), "powerOff success.");
    else
      RCLCPP_INFO(this->get_logger(), "powerOff failed.");
  }
  else if (request->cmd == "stop")
  {
    handleArmStopped();
  }
  else if (request->cmd == "collision recover")
  {
    ret = robot_send_service_.robotServiceCollisionRecover();

    if (ret == aubo_robot_namespace::InterfaceCallSuccCode)
      RCLCPP_INFO(this->get_logger(), "collision recover success.");
    else
      RCLCPP_INFO(this->get_logger(), "collision recover failed.");
  }
  else if (request->cmd == "clear singularityOverSpeedAlarm")
  {
    ret = robot_send_service_.rootServiceRobotControl(aubo_robot_namespace::RobotControlCommand::ClearSingularityOverSpeedAlarm);

    if (ret == aubo_robot_namespace::InterfaceCallSuccCode)
      RCLCPP_INFO(this->get_logger(), "clear singularityOverSpeedAlarm success.");
    else
      RCLCPP_INFO(this->get_logger(), "clear singularityOverSpeedAlarm failed.");
  }
  else if (request->cmd == "setTcp")
  {
    tcp_.toolInEndPosition.x = request->tcp.position.x;
    tcp_.toolInEndPosition.y = request->tcp.position.y;
    tcp_.toolInEndPosition.z = request->tcp.position.z;
    tcp_.toolInEndOrientation.w = request->tcp.orientation.w;
    tcp_.toolInEndOrientation.x = request->tcp.orientation.x;
    tcp_.toolInEndOrientation.y = request->tcp.orientation.y;
    tcp_.toolInEndOrientation.z = request->tcp.orientation.z;

    ret = robot_send_service_.robotServiceSetRobotTool(tcp_);
    if (ret == aubo_robot_namespace::InterfaceCallSuccCode)
      RCLCPP_INFO(this->get_logger(), "set robot tool success.");
    else
      RCLCPP_INFO(this->get_logger(), "set robot tool failed.");

    ret = robot_send_service_.robotServiceSetToolKinematicsParam(tcp_);

    if (ret == aubo_robot_namespace::InterfaceCallSuccCode)
      RCLCPP_INFO(this->get_logger(), "set tcp kinematics success.");
    else
      RCLCPP_INFO(this->get_logger(), "set tcp kinematics failed.");
  }
  else if (request->cmd == "setPayload")
  {
    aubo_robot_namespace::ToolDynamicsParam toolDynamicsParam;
    toolDynamicsParam.positionX = 0.0;
    toolDynamicsParam.positionY = 0.0;
    toolDynamicsParam.positionZ = 0.0;
    toolDynamicsParam.payload = request->payload;
    ret = robot_send_service_.robotServiceGetToolDynamicsParam(toolDynamicsParam);

    if (ret == aubo_robot_namespace::InterfaceCallSuccCode)
      RCLCPP_INFO(this->get_logger(), "set payload success.");
    else
      RCLCPP_INFO(this->get_logger(), "set payload failed.");
  }
  else if (request->cmd == "jointMove")
  {
    start_move_ = true;

    RCLCPP_INFO(this->get_logger(), "set payload failed.");
    std::vector<double> maxAcc, maxVel;
    if (request->velocities.size() != 6 || request->accelerations.size() != 6)
    {
      for (int i = 0; i < 6; i++)
      {
        maxAcc.push_back(MAX_JOINT_ACC);
        maxVel.push_back(MAX_JOINT_VEL);
      }
    }
    else 
    {
      for (int i = 0; i < 6; i++)
      {
        maxAcc.push_back(request->accelerations[i]);
        maxVel.push_back(request->velocities[i]);
      }
    }

    RCLCPP_INFO(this->get_logger(), "set payload failed.");
    bool result = jointMove(request->joints, maxVel, maxAcc);

    start_move_ = false;

    if (result)
      ret = aubo_robot_namespace::InterfaceCallSuccCode;
    else
      ret = aubo_robot_namespace::ErrCode_Failed;
  }
  else if (request->cmd == "setCollisionLevel")
  {
    ret = robot_send_service_.robotServiceSetRobotCollisionClass(request->collision_level);
    if (ret == aubo_robot_namespace::InterfaceCallSuccCode)
      RCLCPP_INFO(this->get_logger(), "set collision level %d success.", request->collision_level);
    else
      RCLCPP_INFO(this->get_logger(), "set collision level failed.");
  }

  if (ret == aubo_robot_namespace::InterfaceCallSuccCode)
  {
    response->result = true;
    response->result_code = ret;
  }
  else
  {
    response->result = false;
    response->result_code = ret;
  }
}

void AuboRos2Driver::moveitControllerCallback(const trajectory_msgs::msg::JointTrajectoryPoint::ConstSharedPtr msg)
{
  PlanningState ps;
  for(int i = 0; i < 6; i++)
  {
    ps.joint_pos_[i] = msg->positions[i];
    ps.joint_vel_[i] = msg->velocities[i];
    ps.joint_acc_[i] = msg->accelerations[i];
  }

  moveit_controller_queue_.push(ps);
}
