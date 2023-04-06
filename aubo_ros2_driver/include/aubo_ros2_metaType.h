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

#ifndef AUBO_ROS2_METATYPE_H_
#define AUBO_ROS2_METATYPE_H_

#include "AuboRobotMetaType.h"
#include "math.h"

const int ARM_DOF = 6;
const double MAX_JOINT_ACC = 10.0/180.0*M_PI;
const double MAX_JOINT_VEL = 10.0/180.0*M_PI;
const int UPDATE_RATE_ = 500;
const int MINIMUM_BUFFER_SIZE = 300;
const std::string joint_name_[ARM_DOF] = {"shoulder_joint","upperArm_joint","foreArm_joint","wrist1_joint","wrist2_joint","wrist3_joint"};

namespace aubo_ros2_driver
{
  struct PlanningState
  {
    double joint_vel_[ARM_DOF];
    double joint_acc_[ARM_DOF];
    double joint_pos_[ARM_DOF];
  };

  enum ROBOT_CONTROLLER_MODE
  {
    ROBOT_CONTROLLER = 0,
    ROS_CONTROLLER
  };

  enum ControlOption
  {
    AuboAPI = 0,
    RosMoveIt
  };

  enum ControMode
  {
    Teach = 0,
    SendTargetGoal,
    SynchronizeWithRealRobot
  };

  struct ArmStopped
  {
    bool user_stopped;
    bool collision_stopped;
    bool emergency_stopped;
    bool protective_stopped;
    bool singularity_stopped;
  };

  struct RobotState
  {
    aubo_robot_namespace::JointStatus joint_status_[ARM_DOF];
    aubo_robot_namespace::wayPoint_S wayPoint_;
    aubo_robot_namespace::RobotDiagnosis robot_diagnosis_info_;
    bool IsRealRobotExist;
    bool isRobotControllerConnected;
    ROBOT_CONTROLLER_MODE robot_controller_;
    aubo_robot_namespace::RobotState state_;
    aubo_robot_namespace::RobotErrorCode code_;
  };
}

#endif
