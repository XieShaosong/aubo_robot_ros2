# Aubo Robot ROS2

## Compatibility

| **Supported OS**          | **Supported ROS2 distribution**                         |
|---------------------------|---------------------------------------------------------|
| Ubuntu 20.04              | [Foxy Fitzroy](https://docs.ros.org/en/foxy/index.html) |

|**Recommend AUBO Teachpendant Version**    |**Recommend AUBO Hardware Version**  |
|-------------------------------------------|-------------------------------------|
|V4.5.48                                    |V3.4.38                              |

## Attention
Due to the update of aubo, the execution efficiency of the robotServiceSetRobotPosData2Canbus(double jointAngle [aubo_robot_namespace: ARM-DOF]) API has decreased. The new version may cause the robotic arm to shake during trajectory execution. It is recommended to use the recommended version to run this project.

## Getting Started

This project was developed for ROS2 Foxy on Ubuntu 20.04. Other versions of Ubuntu and ROS2 may work, but are not officially supported.

1. Install [ROS2 Foxy via Debian Packages](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)

2. Install `colcon` and additional ROS packages:

    ```bash
    sudo apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep2 \
    libeigen3-dev \
    ros-foxy-xacro \
    ros-foxy-tinyxml2-vendor \
    ros-foxy-ros2-control \
    ros-foxy-realtime-tools \
    ros-foxy-control-toolbox \
    ros-foxy-moveit \
    ros-foxy-ros2-controllers \
    ros-foxy-test-msgs \
    ros-foxy-joint-state-publisher \
    ros-foxy-joint-state-publisher-gui \
    ros-foxy-robot-state-publisher \
    ros-foxy-rviz2
    ```

3. Setup workspace:

    ```bash
    mkdir -p ~/aubo_ros2_ws/src
    cd ~/aubo_ros2_ws/src
    git clone https://github.com/XieShaosong/aubo_robot_ros2.git
    cd aubo_robot_ros2/
    ```

4. Install dependencies:

    ```bash
    cd ~/aubo_ros2_ws
    rosdep update
    rosdep install --from-paths src --ignore-src --rosdistro foxy -r -y
    ```

5. Build and source the workspace:

    ```bash
    cd ~/aubo_ros2_ws
    source /opt/ros/foxy/setup.bash
    colcon build --symlink-install
    source install/setup.bash
    ```

**NOTE**: Remember to source the setup file and the workspace whenever a new terminal is opened:

```bash
source /opt/ros/foxy/setup.bash
source ~/aubo_ros2_ws/install/setup.bash
```

## Usage

### Using MoveIt

You can also run the MoveIt example and use the `MotionPlanning` plugin in RViZ to start planning:

```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/aubo_ros2_ws/install/aubo_ros2_driver/lib/aubo_ros2_driver/aubocontroller
ros2 launch aubo_ros2_moveit_config aubo_moveit.launch.py robot_ip:=[robot ip]
ros2 topic pub --once /aubo_robot/robot_control std_msgs/msg/String 'data: powerOn'
```

Test sim:
```bash
ros2 launch  aubo_ros2_moveit_config aubo_moveit_sim.launch.py
```
