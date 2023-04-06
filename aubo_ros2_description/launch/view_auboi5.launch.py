
import os
import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    pkg_share = FindPackageShare("aubo_ros2_description")
    default_rviz_config_path = PathJoinSubstitution(
        [pkg_share, "rviz", "view_rizon.rviz"]
    )
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("aubo_ros2_description"), "urdf", "aubo_i5.urdf.xacro"]
            ),
            " ",
            "name:=",
            "aubo_i5",
        ]
    )

    # Robot state publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description_content}],
    )

    # package_path = get_package_share_directory("aubo_ros2_description")
    # joint_state_publisher_node = Node(
    #     package="joint_state_publisher",
    #     executable="joint_state_publisher",
    #     name="joint_state_publisher",
    #     parameters=[package_path + "/config/initial_positions.yaml"],
    #     # condition=UnlessCondition(LaunchConfiguration("gui")),
    # )

    # joint_state_publisher_gui_node = Node(
    #     package="joint_state_publisher_gui",
    #     executable="joint_state_publisher_gui",
    #     name="joint_state_publisher_gui",
    #     condition=IfCondition(LaunchConfiguration("gui")),
    # )

    # RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="aubo_type",
                default_value="aubo_i5",
                description="Type of the aubo robot.",
                choices=["aubo_i3", "aubo_i5", "aubo_i10"],
            ),
            DeclareLaunchArgument(
                name="gui",
                default_value="False",
                description="Flag to enable joint_state_publisher_gui",
            ),
            DeclareLaunchArgument(
                name="rvizconfig",
                default_value=default_rviz_config_path,
                description="Absolute path to rviz config file",
            ),
            robot_state_publisher_node,
            # joint_state_publisher_node,
            # joint_state_publisher_gui_node,
            rviz_node,
        ]
    )
