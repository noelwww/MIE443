"""
SO101 Laptop Launch File

This launch file runs on your laptop for MoveIt motion planning and RViz visualization.
The SO101 Bridge should already be running on the TurtleBot4.

It starts:
- MoveIt move_group (motion planning)
- RViz for visualization
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Get package directories
    lerobot_description_dir = get_package_share_directory("lerobot_description")
    lerobot_moveit_dir = get_package_share_directory("lerobot_moveit")

    # URDF path
    so101_urdf_path = os.path.join(lerobot_description_dir, "urdf", "so101.urdf.xacro")

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
        move_group_node,
        rviz_node,
    ])

