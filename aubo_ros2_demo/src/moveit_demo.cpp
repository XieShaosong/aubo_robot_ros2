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

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <control_msgs/action/follow_joint_trajectory.hpp>

typedef control_msgs::action::FollowJointTrajectory FollowJointTrajectory;
typedef rclcpp_action::ClientGoalHandle<FollowJointTrajectory> GoalHandleFjt;

class DemoActionClient
{
public:
  DemoActionClient(const rclcpp::Node::SharedPtr& node)
  : node_(node)
  {
    this->client_ptr_ = rclcpp_action::create_client<FollowJointTrajectory>(node_->get_node_base_interface(),
                                                                            node_->get_node_graph_interface(),
                                                                            node_->get_node_logging_interface(),
                                                                            node_->get_node_waitables_interface(),"aubo_i5_controller/follow_joint_trajectory");
  }

  void planAndSendGoal()
  {
    static const std::string  PLANNING_GROUP  = "manipulator_i5";
    moveit::planning_interface::MoveGroupInterface move_group(node_, PLANNING_GROUP );
    move_group.setPoseReferenceFrame("base_link");

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP );

    move_group.setMaxAccelerationScalingFactor(1);
    move_group.setMaxVelocityScalingFactor(1);

    std::vector<double> zero_position;
    zero_position.push_back(M_PI*(0.0)/180.0);
    zero_position.push_back(M_PI*(0.0)/180.0);
    zero_position.push_back(M_PI*(90.0)/180.0);
    zero_position.push_back(M_PI*(0.0)/180.0);
    zero_position.push_back(M_PI*(90.0)/180.0);
    zero_position.push_back(M_PI*(0.0)/180.0);
    move_group.setJointValueTarget(zero_position);
    move_group.setPlannerId("RRTConnect");

    move_group.plan(plan);

    trajectory_msgs::msg::JointTrajectory trajectory;
    trajectory = plan.trajectory_.joint_trajectory;
    control_msgs::action::FollowJointTrajectory_Goal goal;
    goal.trajectory = trajectory;

    client_ptr_->wait_for_action_server();
    client_ptr_->async_send_goal(goal);
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr client_ptr_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("moveit_demo", node_options);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]()
              { executor.spin(); })
      .detach();

  DemoActionClient demo(node);
  demo.planAndSendGoal();

  rclcpp::shutdown();
  return 0;
}
