#!/usr/bin/env python3
"""
SO101 Teleoperation Follower Node

This node runs on the TURTLEBOT where the follower arm is connected.
It subscribes to the leader joint states topic and sends commands to the
SO101 follower arm.

Usage:
    ros2 run lerobot_controller teleop_follower.py --ros-args -p port:=/dev/ttyACM0

Make sure ROS_DOMAIN_ID is the same on both laptop and TurtleBot for communication.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# LeRobot imports
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig


class TeleopFollowerNode(Node):
    """ROS2 node that receives leader joint states and controls SO101 follower arm."""

    # Joint names (matching the lerobot naming convention)
    JOINT_NAMES = [
        "shoulder_pan",
        "shoulder_lift",
        "elbow_flex",
        "wrist_flex",
        "wrist_roll",
        "gripper",
    ]

    def __init__(self):
        super().__init__('teleop_follower')

        # Declare parameters
        self.declare_parameter('port', '/dev/ttyACM0')

        port = self.get_parameter('port').get_parameter_value().string_value

        self.get_logger().info(f"Initializing SO101 Follower on port: {port}")

        # Initialize follower arm (no camera for teleoperation)
        self.follower_config = SO101FollowerConfig(
            port=port,
            id='follower',
        )
        self.follower = SO101Follower(self.follower_config)
        
        try:
            self.follower.connect()
            self.get_logger().info("Connected to SO101 Follower arm")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to follower arm: {e}")
            raise

        # Calibrate if needed
        if not self.follower.is_calibrated:
            self.get_logger().info("Calibrating follower arm...")
            self.follower.calibrate()
            self.get_logger().info("Calibration complete")

        # Subscriber for leader joint states
        self.joint_sub = self.create_subscription(
            JointState,
            'teleop/leader_joints',
            self.leader_callback,
            10
        )

        # Track last received time for timeout detection
        self.last_received_time = None
        self.timeout_sec = 1.0  # Consider leader disconnected after 1 second

        self.get_logger().info("Subscribing to 'teleop/leader_joints'")
        self.get_logger().info("Follower node ready! Waiting for leader commands...")

    def leader_callback(self, msg: JointState):
        """Process incoming leader joint states and send to follower."""
        try:
            self.last_received_time = self.get_clock().now()
            
            # Build action dict for lerobot
            # Format: {"shoulder_pan.pos": value, "shoulder_lift.pos": value, ...}
            action = {}
            
            for i, joint_name in enumerate(msg.name):
                if i < len(msg.position):
                    key = f"{joint_name}.pos"
                    action[key] = msg.position[i]
            
            # Send action to follower arm
            self.follower.send_action(action)
            
        except Exception as e:
            self.get_logger().error(f"Error sending to follower arm: {e}")

    def destroy_node(self):
        """Clean up when node is destroyed."""
        self.get_logger().info("Shutting down follower node...")
        try:
            self.follower.disconnect()
            self.get_logger().info("Disconnected from follower arm")
        except Exception as e:
            self.get_logger().warn(f"Error during disconnect: {e}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = TeleopFollowerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
