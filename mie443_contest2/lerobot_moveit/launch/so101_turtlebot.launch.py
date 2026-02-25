"""
SO101 TurtleBot4 Launch File (Distributed Setup - Robot Side)

This launch file runs on the TurtleBot4 where the SO-ARM101 is physically connected.
It starts:
- Robot state publisher (URDF)
- SO101 Bridge node (connects to real hardware via lerobot)

Run the companion launch file (so101_laptop.launch.py) on your laptop for MoveIt + RViz.

Usage:
    ros2 launch lerobot_moveit so101_turtlebot.launch.py
    ros2 launch lerobot_moveit so101_turtlebot.launch.py port:=/dev/ttyACM1

Requirements:
    - SO-ARM101 connected via USB
    - lerobot Python package installed
    - Same ROS_DOMAIN_ID as laptop (e.g., export ROS_DOMAIN_ID=0)
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Declare launch arguments
    port_arg = DeclareLaunchArgument(
        name="port",
        default_value="/dev/ttyACM0",
        description="Serial port for SO-ARM101",
    )

    camera_index_arg = DeclareLaunchArgument(
        name="camera_index",
        default_value="0",
        description="Camera index for wrist camera (0=/dev/video0, 1=/dev/video1, etc.)",
    )

    port = LaunchConfiguration("port")
    camera_index = LaunchConfiguration("camera_index")

    # Get package directories
    lerobot_description_dir = get_package_share_directory("lerobot_description")

    # URDF path
    so101_urdf_path = os.path.join(lerobot_description_dir, "urdf", "so101.urdf.xacro")

    # Robot description
    robot_description = ParameterValue(
        Command(["xacro ", so101_urdf_path]),
        value_type=str,
    )

    # Robot state publisher - publishes TF transforms from joint states
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace="arm",
        output="screen",
        parameters=[{"robot_description": robot_description}],
        remappings=[("joint_states", "/joint_states")],
    )

    # SO101 Bridge node - connects to real hardware
    so101_bridge_node = Node(
        package="lerobot_controller",
        executable="so101_bridge.py",
        name="so101_bridge",
        output="screen",
        parameters=[
            {"port": port},
            {"camera_index": camera_index},
            {"joint_state_rate": 50.0},
        ],
    )

    # Static TF to mount arm root on TurtleBot base_link.
    mount_x = "0.028441"
    mount_y = "0.0"
    mount_z = "0.168"
    mount_roll = "0.0"
    mount_pitch = "0.0"
    mount_yaw = "0.0"

    arm_mount_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="arm_mount_tf_publisher",
        output="screen",
        arguments=[
            "--x",
            mount_x,
            "--y",
            mount_y,
            "--z",
            mount_z,
            "--roll",
            mount_roll,
            "--pitch",
            mount_pitch,
            "--yaw",
            mount_yaw,
            "--frame-id",
            "base_link",
            "--child-frame-id",
            "arm_mount",
        ],
    )

    return LaunchDescription(
        [
            port_arg,
            camera_index_arg,
            robot_state_publisher_node,
            so101_bridge_node,
            arm_mount_tf_node,
        ]
    )
