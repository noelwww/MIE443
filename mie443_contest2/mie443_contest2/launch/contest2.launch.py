"""
Contest 2 All-in-One Launch File
================================
Launches:
  1. AprilTag detector (remapped to OAK-D camera topics)
  2. YOLO object detector (Python node)
  3. Contest2 main node (C++)

Usage:
  # Normal contest mode:
  ros2 launch mie443_contest2 contest2.launch.py

  # Debug mode (interactive hardware testing):
  ros2 launch mie443_contest2 contest2.launch.py debug:=true

Prerequisites (must already be running):
  - TurtleBot4 base (undocked)
  - AMCL localization:  ros2 launch turtlebot4_navigation localization.launch.py map:=<path>.yaml
  - Nav2 navigation:    ros2 launch turtlebot4_navigation nav2.launch.py
  - OAK-D camera:       ros2 launch depthai_ros_driver camera.launch.py (or equivalent)
  - SO-ARM101 bridge:   ros2 launch lerobot_moveit so101_turtlebot.launch.py  (on TurtleBot)
  - MoveIt2:            ros2 launch lerobot_moveit so101_laptop.launch.py     (on Laptop)
  - Image capture:      ros2 run mie443_contest2 image_capture_server         (on TurtleBot)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():

    # ---- Launch Arguments ----
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Run in debug mode (interactive hardware testing)'
    )

    tag_size_arg = DeclareLaunchArgument(
        'tag_size',
        default_value='0.048',
        description='Physical AprilTag size in meters'
    )

    # ---- 1. AprilTag Detection Node ----
    # Remaps default topics to the OAK-D camera topics on the TurtleBot4.
    # This replaces the need to manually run camera_36h11.launch.yml.
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_oakd',
        remappings=[
            ('image_rect', '/oakd/rgb/preview/image_raw'),
            ('camera_info', '/oakd/rgb/preview/camera_info'),
        ],
        parameters=[{
            'family': '36h11',
            'size': LaunchConfiguration('tag_size'),
            'max_hamming': 0,
            'detector.threads': 2,
            'detector.quad_decimate': 2.0,
            'detector.refine_edges': True,
        }],
        output='screen',
    )

    # ---- 2. YOLO Object Detector Node (Python) ----
    yolo_node = Node(
        package='mie443_contest2',
        executable='yolo_detector',
        name='yolo_detector',
        output='screen',
    )

    # ---- 3a. Contest2 Node — Normal Mode ----
    contest2_normal = Node(
        package='mie443_contest2',
        executable='contest2',
        name='contest2',
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('debug'), "' == 'false'"])
        ),
    )

    # ---- 3b. Contest2 Node — Debug Mode (with --debug flag) ----
    contest2_debug = Node(
        package='mie443_contest2',
        executable='contest2',
        name='contest2',
        output='screen',
        arguments=['--debug'],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('debug'), "' == 'true'"])
        ),
    )

    return LaunchDescription([
        debug_arg,
        tag_size_arg,
        apriltag_node,
        yolo_node,
        contest2_normal,
        contest2_debug,
    ])
