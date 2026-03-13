"""
SO101 Real Hardware Launch File

This launch file starts all necessary nodes for controlling the real SO-ARM101:
- Robot state publisher (URDF)
- SO101 Bridge node (connects to real hardware via lerobot)
- MoveIt move_group
- RViz for visualization

Usage:
    ros2 launch lerobot_moveit so101_real.launch.py
    ros2 launch lerobot_moveit so101_real.launch.py port:=/dev/ttyACM1
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Declare launch arguments
    port_arg = DeclareLaunchArgument(
        name="port",
        default_value="/dev/ttyACM0",
        description="Serial port for SO-ARM101"
    )
    
    camera_index_arg = DeclareLaunchArgument(
        name="camera_index",
        default_value="0",
        description="Camera index for wrist camera (0=/dev/video0, 1=/dev/video1, etc.)"
    )

    port = LaunchConfiguration("port")
    camera_index = LaunchConfiguration("camera_index")

    # Get package directories
    lerobot_description_dir = get_package_share_directory("lerobot_description")
    lerobot_moveit_dir = get_package_share_directory("lerobot_moveit")

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

    # MoveIt configuration
    moveit_config = (
        MoveItConfigsBuilder("so101", package_name="lerobot_moveit")
        .robot_description(file_path=so101_urdf_path)
        .robot_description_semantic(file_path="config/so101.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    # MoveIt move_group node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": False},
            {"publish_robot_description_semantic": True},
        ],
        arguments=["--ros-args", "--log-level", "info"],
        remappings=[
            ("/robot_description", "/arm/robot_description"),
            ("/robot_description_semantic", "/arm/robot_description_semantic"),
        ],
    )

    # RViz
    rviz_config_path = os.path.join(lerobot_moveit_dir, "config", "moveit.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    return LaunchDescription([
        port_arg,
        camera_index_arg,
        robot_state_publisher_node,
        so101_bridge_node,
        move_group_node,
        rviz_node,
    ])

