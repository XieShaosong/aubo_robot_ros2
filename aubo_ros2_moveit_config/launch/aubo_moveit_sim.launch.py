# This launch file is based on
# https://github.com/ros-planning/moveit_resources/blob/galactic/panda_moveit_config/launch/demo.launch.py

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
import xacro

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def generate_launch_description():
    # Declare command-line arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "db",
            default_value="false",
            description="Database flag",
        )
    )

    db_config = LaunchConfiguration("db")

    # planning_context
    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory("aubo_ros2_description"),
            "urdf",
            "aubo_i5.urdf.xacro",
        )
    )
    robot_description = {"robot_description": robot_description_config.toxml()}

    # MoveIt configuration
    robot_description_semantic_config = load_file(
        "aubo_ros2_moveit_config", "srdf/aubo_i5.srdf"
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }

    kinematics_yaml = load_yaml("aubo_ros2_moveit_config", "config/kinematics.yaml")

    # Planning Configuration
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization "
            "default_planner_request_adapters/ResolveConstraintFrames "
            "default_planner_request_adapters/FixWorkspaceBounds "
            "default_planner_request_adapters/FixStartStateBounds "
            "default_planner_request_adapters/FixStartStateCollision "
            "default_planner_request_adapters/FixStartStatePathConstraints",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml("aubo_ros2_moveit_config", "config/ompl_planning.yaml")
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # Trajectory Execution Configuration
    moveit_simple_controllers_yaml = load_yaml(
        "aubo_ros2_moveit_config", "config/moveit_controllers.yaml"
    )
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    joint_limits_yaml = {
        "robot_description_planning": load_yaml(
            "aubo_ros2_moveit_config", "config/joint_limits.yaml"
        )
    }

    joint_names_yaml = {
        "joint_name" : load_yaml(
          "aubo_ros2_moveit_config", "config/joint_names.yaml"
        )
    }

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            joint_limits_yaml,
            joint_names_yaml,
        ],
    )

    # RViz with MoveIt configuration
    rviz_base = os.path.join(
        get_package_share_directory("aubo_ros2_moveit_config"), "rviz"
    )
    rviz_config_file = os.path.join(rviz_base, "moveit.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
            joint_limits_yaml,
        ],
    )

    #aubo ros2 tarjectory action
    aubo_trajectory_action_node = Node(
        package="aubo_ros2_trajectory_action",
        executable="aubo_ros2_trajectory_action",
        output="screen",
        parameters=[joint_names_yaml],
    )

    #joint state publisher
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
    )

    # Publish TF
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Warehouse mongodb server
    mongodb_server_node = Node(
        package="warehouse_ros_mongo",
        executable="mongo_wrapper_ros.py",
        parameters=[
            {"warehouse_port": 33829},
            {"warehouse_host": "localhost"},
            {"warehouse_plugin": "warehouse_ros_mongo::MongoDatabaseConnection"},
        ],
        output="screen",
        condition=IfCondition(db_config),
    )

    nodes = [
        joint_state_publisher_node,
        aubo_trajectory_action_node,
        move_group_node,
        robot_state_publisher_node,
        rviz_node,
        mongodb_server_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
