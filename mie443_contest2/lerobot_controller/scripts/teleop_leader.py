#!/usr/bin/env python3
"""
SO101 Teleoperation Leader Node

This node runs on the LAPTOP where the leader arm is connected.
It reads joint positions from the SO101 leader arm and publishes them
to a ROS2 topic for the follower node to receive.

Usage:
    ros2 run lerobot_controller teleop_leader.py --ros-args -p port:=/dev/ttyACM0

Make sure ROS_DOMAIN_ID is the same on both laptop and TurtleBot for communication.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# LeRobot imports
from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig


class TeleopLeaderNode(Node):
    """ROS2 node that reads from SO101 leader arm and publishes joint states."""

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
        super().__init__('teleop_leader')

        # Declare parameters
        self.declare_parameter('port', '/dev/ttyACM1')
        self.declare_parameter('publish_rate', 50.0)  # Hz

        port = self.get_parameter('port').get_parameter_value().string_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        self.get_logger().info(f"Initializing SO101 Leader on port: {port}")

        # Initialize leader arm
        self.leader_config = SO101LeaderConfig(
            port=port,
            id='leader',
        )
        self.leader = SO101Leader(self.leader_config)
        
        try:
            self.leader.connect()
            self.get_logger().info("Connected to SO101 Leader arm")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to leader arm: {e}")
            raise

        # Calibrate if needed
        if not self.leader.is_calibrated:
            self.get_logger().info("Calibrating leader arm...")
            self.leader.calibrate()
            self.get_logger().info("Calibration complete")

        # Publisher for leader joint states
        self.joint_pub = self.create_publisher(
            JointState,
            'teleop/leader_joints',
            10
        )

        # Timer to read and publish at the specified rate
        period = 1.0 / publish_rate
        self.timer = self.create_timer(period, self.publish_leader_state)

        self.get_logger().info(f"Publishing leader joint states to 'teleop/leader_joints' at {publish_rate} Hz")
        self.get_logger().info("Leader node ready! Move the leader arm to control the follower.")

    def publish_leader_state(self):
        """Read leader arm positions and publish them."""
        try:
            # get_action() returns dict like {"shoulder_pan.pos": value, ...}
            action = self.leader.get_action()
            
            # Create JointState message
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'leader'
            
            # Extract positions in order
            positions = []
            for joint_name in self.JOINT_NAMES:
                key = f"{joint_name}.pos"
                if key in action:
                    positions.append(float(action[key]))
                else:
                    self.get_logger().warn(f"Joint {key} not found in action, using 0.0")
                    positions.append(0.0)
            
            msg.name = self.JOINT_NAMES
            msg.position = positions
            msg.velocity = []
            msg.effort = []
            
            self.joint_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"Error reading leader arm: {e}")

    def destroy_node(self):
        """Clean up when node is destroyed."""
        self.get_logger().info("Shutting down leader node...")
        try:
            self.leader.disconnect()
            self.get_logger().info("Disconnected from leader arm")
        except Exception as e:
            self.get_logger().warn(f"Error during disconnect: {e}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = TeleopLeaderNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
