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

    joint_limits_yaml = {
        "robot_description_planning": load_yaml(
            "aubo_ros2_moveit_config", "config/joint_limits.yaml"
        )
    }

    # Start the actual move_group node/action server
    move_group_demo = Node(
        package="aubo_ros2_demo",
        executable="aubo_ros2_moveit_demo",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            joint_limits_yaml,
        ],
    )

    nodes = [
        move_group_demo,
    ]

    return LaunchDescription(declared_arguments + nodes)