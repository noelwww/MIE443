"""
Launch apriltag_ros node with OAK-D camera on TurtleBot4.

Run on the LAPTOP (same machine as contest2):
  ros2 launch mie443_contest2 apriltag_oakd.launch.py

The OAK-D camera must be streaming first (the image_capture_server or
/oakd/start_camera service triggers this).
"""

from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    # Path to our apriltag config
    config_file = os.path.join(
        os.path.dirname(os.path.realpath(__file__)),
        '..', 'config', 'apriltag_config.yaml'
    )

    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag',
        namespace='',
        remappings=[
            # Map apriltag_ros inputs to the OAK-D camera topics
            ('image_rect', '/oakd/rgb/preview/image_raw'),
            ('camera_info', '/oakd/rgb/preview/camera_info'),
        ],
        parameters=[config_file],
        output='screen',
        arguments=['--ros-args', '--log-level', 'apriltag:=warn'],
    )

    return LaunchDescription([
        apriltag_node,
    ])
