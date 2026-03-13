#!/usr/bin/env python3
"""
SO101 Bridge Node

This node bridges ROS 2 (MoveIt) with the real SO-ARM101 hardware via the lerobot Python API.
It provides:
- Action servers for arm and gripper trajectory execution (matching MoveIt's expected interface)
- Joint state publishing for feedback to MoveIt

Joint name mapping (URDF → lerobot):
    "1" → "shoulder_pan.pos"
    "2" → "shoulder_lift.pos"
    "3" → "elbow_flex.pos"
    "4" → "wrist_flex.pos"
    "5" → "wrist_roll.pos"
    "6" → "gripper.pos"

Unit conversions (linear interpolation):
    - Joints 1-5: lerobot (-100 to +100) ↔ URDF limits (radians)
    - Joint 6 (gripper): lerobot (0 to 100) ↔ URDF limits (radians)
    
    Formula: radians = URDF_min + ((lerobot_val - LEROBOT_min) / LEROBOT_range) × URDF_range
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState, CompressedImage
from mie443_contest2.srv import CaptureImage

import time
import threading
import math
import cv2
import numpy as np

# LeRobot imports
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig

class So101Bridge(Node):
    """Bridge node connecting ROS 2/MoveIt to real SO-ARM101 hardware via lerobot."""

    # Joint name mapping: URDF name → lerobot feature name
    JOINT_MAP = {
        "1": "shoulder_pan.pos",
        "2": "shoulder_lift.pos",
        "3": "elbow_flex.pos",
        "4": "wrist_flex.pos",
        "5": "wrist_roll.pos",
        "6": "gripper.pos",
    }

    # URDF joint limits (in radians) - from so101_ros2_control.xacro
    URDF_LIMITS = {
        "1": {"min": -1.91986, "max": 1.91986},   # ±110°
        "2": {"min": -1.74533, "max": 1.74533},   # ±100°
        "3": {"min": -1.74533, "max": 1.5708},    # -100° to +90°
        "4": {"min": -1.65806, "max": 1.65806},   # ±95°
        "5": {"min": -2.79253, "max": 2.79253},   # ±160°
        "6": {"min": -0.174533, "max": 1.74533},  # gripper
    }

    # LeRobot value ranges
    # Joints 1-5: -100 to +100
    # Joint 6 (gripper): 0 to 100
    LEROBOT_LIMITS = {
        "1": {"min": -100.0, "max": 100.0},
        "2": {"min": -100.0, "max": 100.0},
        "3": {"min": -100.0, "max": 100.0},
        "4": {"min": -100.0, "max": 100.0},
        "5": {"min": -100.0, "max": 100.0},
        "6": {"min": 0.0, "max": 100.0},  # gripper
    }

    # Arm joints (for arm_controller)
    ARM_JOINTS = ["1", "2", "3", "4", "5"]
    
    # Gripper joints (for gripper_controller)
    GRIPPER_JOINTS = ["6"]

    def __init__(self):
        super().__init__('so101_bridge')

        # Declare parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('camera_index', 0) 
        self.declare_parameter('joint_state_rate', 50.0)  # Hz

        port = self.get_parameter('port').get_parameter_value().string_value
        camera_index = self.get_parameter('camera_index').get_parameter_value().integer_value
        joint_state_rate = self.get_parameter('joint_state_rate').get_parameter_value().double_value

        self.get_logger().info(f"Initializing SO101 Bridge on port: {port}, camera index: {camera_index}")

        # Camera configuration for wrist camera
        camera_config = {
            "wrist": OpenCVCameraConfig(
                index_or_path=camera_index,
                fps=30,
                width=640,
                height=480,
            )
        }
        
        # Initialize lerobot robot
        self.config = SO101FollowerConfig(
            port=port,
            id='follower',
            cameras=camera_config,
        )
        self.robot = SO101Follower(self.config)
        self.robot.connect(calibrate=False)
        if self.robot.calibration:
            self.get_logger().info("Writing calibration to motors from stored calibration file...")
            self.robot.bus.write_calibration(self.robot.calibration)
            self.get_logger().info("Calibration applied")
        else:
            self.get_logger().warn("No calibration file found - arm may need manual calibration")
        self.get_logger().info("Connected to SO-ARM101 hardware")
                
        self.robot_lock = threading.Lock()

        # Current joint positions in RADIANS
        self.current_positions = {joint: 0.0 for joint in self.JOINT_MAP.keys()}

        self.callback_group = ReentrantCallbackGroup()

        # Action server for arm trajectory
        self.arm_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            'arm_controller/follow_joint_trajectory',
            execute_callback=self.execute_arm_trajectory,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group
        )

        # Action server for gripper trajectory
        self.gripper_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            'gripper_controller/follow_joint_trajectory',
            execute_callback=self.execute_gripper_trajectory,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group
        )

        # Joint state publisher
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)

        # Timer for publishing joint states
        period = 1.0 / joint_state_rate
        self.joint_state_timer = self.create_timer(period, self.publish_joint_states)

        # Wrist camera capture service
        self.capture_service = self.create_service(
            CaptureImage,
            'wrist_camera/capture_image',
            self.capture_image_callback
        )

        self.get_logger().info("SO101 Bridge initialized successfully!")
        self.get_logger().info("  - Arm action server: arm_controller/follow_joint_trajectory")
        self.get_logger().info("  - Gripper action server: gripper_controller/follow_joint_trajectory")
        self.get_logger().info("  - Publishing joint states to: /joint_states")
        self.get_logger().info("  - Wrist camera service: wrist_camera/capture_image")
        self.get_logger().info("  - Using linear interpolation for unit conversion")

    # ==================== Unit Conversion Functions ====================
    
    def lerobot_to_radians(self, joint_name, lerobot_value):
        """Convert lerobot value to radians using linear interpolation.
        
        Formula: radians = URDF_min + ((lerobot - LEROBOT_min) / LEROBOT_range) × URDF_range
        
        Args:
            joint_name: URDF joint name ("1" through "6")
            lerobot_value: Value from lerobot (-100 to +100 for joints, 0-100 for gripper)
            
        Returns:
            Value in radians, clamped to URDF limits
        """
        urdf = self.URDF_LIMITS[joint_name]
        lerobot = self.LEROBOT_LIMITS[joint_name]
        
        urdf_range = urdf["max"] - urdf["min"]
        lerobot_range = lerobot["max"] - lerobot["min"]
        
        # Linear interpolation
        radians = urdf["min"] + ((lerobot_value - lerobot["min"]) / lerobot_range) * urdf_range
        
        # Clamp to URDF limits for safety
        radians = max(urdf["min"], min(urdf["max"], radians))
        
        return radians
    
    def radians_to_lerobot(self, joint_name, radians):
        """Convert radians to lerobot value using linear interpolation.
        
        Formula: lerobot = LEROBOT_min + ((radians - URDF_min) / URDF_range) × LEROBOT_range
        
        Args:
            joint_name: URDF joint name ("1" through "6")
            radians: Value in radians
            
        Returns:
            Value for lerobot, clamped to lerobot limits
        """
        urdf = self.URDF_LIMITS[joint_name]
        lerobot = self.LEROBOT_LIMITS[joint_name]
        
        urdf_range = urdf["max"] - urdf["min"]
        lerobot_range = lerobot["max"] - lerobot["min"]
        
        # Clamp input radians to URDF limits first
        radians_clamped = max(urdf["min"], min(urdf["max"], radians))
        
        if radians != radians_clamped:
            self.get_logger().warn(
                f"Joint {joint_name}: commanded {radians:.4f} rad, clamped to {radians_clamped:.4f} rad "
                f"(limits: [{urdf['min']:.4f}, {urdf['max']:.4f}])"
            )
        
        # Linear interpolation
        lerobot_value = lerobot["min"] + ((radians_clamped - urdf["min"]) / urdf_range) * lerobot_range
        
        # Clamp to lerobot limits for safety
        lerobot_value = max(lerobot["min"], min(lerobot["max"], lerobot_value))
        
        return lerobot_value

    # ==================== Callbacks ====================

    def goal_callback(self, goal_request):
        """Accept all goals."""
        self.get_logger().debug("Received goal request")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept all cancel requests."""
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    # ==================== Wrist Camera Capture ====================

    def capture_image_callback(self, request, response):
        """Service callback to capture a single image from the wrist camera.
        
        Returns a JPEG compressed image.
        """
        self.get_logger().info("Wrist camera capture requested")
        
        try:
            # Read camera directly - no need for robot lock, camera has its own thread
            if "wrist" not in self.robot.cameras:
                response.success = False
                response.message = f"No wrist camera configured. Available cameras: {list(self.robot.cameras.keys())}"
                self.get_logger().error(response.message)
                return response
            
            # Use async_read() to get the latest frame from camera
            image = self.robot.cameras["wrist"].async_read()
            
            # Convert to numpy array if needed
            if hasattr(image, 'numpy'):
                image = image.numpy()
            
            # Ensure it's a proper numpy array
            image = np.asarray(image, dtype=np.uint8)
            
            # If image is RGB, convert to BGR for OpenCV
            if len(image.shape) == 3 and image.shape[2] == 3:
                # Assuming lerobot returns RGB, convert to BGR for JPEG encoding
                image_bgr = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            else:
                image_bgr = image
            
            # Resize to smaller resolution for faster network transfer
            image_bgr = cv2.resize(image_bgr, (320, 240), interpolation=cv2.INTER_AREA)
            
            # Compress to JPEG with higher compression (lower quality) for faster network transfer
            compression_params = [cv2.IMWRITE_JPEG_QUALITY, 50]
            success, compressed_data = cv2.imencode('.jpg', image_bgr, compression_params)
            
            if not success:
                response.success = False
                response.message = "Failed to compress image to JPEG"
                self.get_logger().error(response.message)
                return response
            
            # Create CompressedImage message
            response.image = CompressedImage()
            response.image.header.stamp = self.get_clock().now().to_msg()
            response.image.header.frame_id = "wrist_camera"
            response.image.format = "jpeg"
            response.image.data = compressed_data.tobytes()
            
            response.success = True
            response.message = f"Wrist camera image captured ({len(response.image.data)} bytes)"
            self.get_logger().info(response.message)
            
        except Exception as e:
            response.success = False
            response.message = f"Error capturing wrist camera image: {str(e)}"
            self.get_logger().error(response.message)
        
        return response

    # ==================== Joint State Reading/Publishing ====================

    def read_joint_positions(self):
        """Read current joint positions from hardware and convert to radians.
        
        Uses bus.sync_read() directly for fast motor-only reading (no camera).
        """
        with self.robot_lock:
            # Read motor positions directly from bus - fast, no camera involved
            motor_positions = self.robot.bus.sync_read("Present_Position")
            
            # motor_positions is dict like {"shoulder_pan": value, "shoulder_lift": value, ...}
            for urdf_name, lerobot_name in self.JOINT_MAP.items():
                # lerobot_name is like "shoulder_pan.pos", motor key is "shoulder_pan"
                motor_key = lerobot_name.replace(".pos", "")
                if motor_key in motor_positions:
                    lerobot_value = float(motor_positions[motor_key])
                    self.current_positions[urdf_name] = self.lerobot_to_radians(urdf_name, lerobot_value)

    def publish_joint_states(self):
        """Publish current joint states in radians."""
        self.read_joint_positions()

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.JOINT_MAP.keys())
        msg.position = [self.current_positions[j] for j in msg.name]
        msg.velocity = []
        msg.effort = []

        self.joint_state_pub.publish(msg)

    # ==================== Action Execution ====================

    def send_positions_to_robot(self, positions_dict):
        """Send joint positions to the robot hardware.
        
        Args:
            positions_dict: Dict mapping URDF joint names to position values (RADIANS)
        """
        # Convert from radians to lerobot units
        action = {}
        for urdf_name, ros_value in positions_dict.items():
            lerobot_name = self.JOINT_MAP.get(urdf_name)
            if lerobot_name:
                lerobot_value = self.radians_to_lerobot(urdf_name, ros_value)
                action[lerobot_name] = lerobot_value
                self.get_logger().debug(
                    f"Joint {urdf_name}: {ros_value:.4f} rad → {lerobot_value:.2f} lerobot"
                )

        with self.robot_lock:
            try:
                self.robot.send_action(action)
            except Exception as e:
                self.get_logger().error(f"Failed to send action: {e}")
                raise

    async def execute_arm_trajectory(self, goal_handle):
        """Execute arm trajectory."""
        self.get_logger().info("Executing arm trajectory")
        return await self._execute_trajectory(goal_handle, self.ARM_JOINTS)

    async def execute_gripper_trajectory(self, goal_handle):
        """Execute gripper trajectory."""
        self.get_logger().info("Executing gripper trajectory")
        return await self._execute_trajectory(goal_handle, self.GRIPPER_JOINTS)

    async def _execute_trajectory(self, goal_handle, expected_joints):
        """Execute a joint trajectory.
        
        Args:
            goal_handle: Action goal handle
            expected_joints: List of joint names this action handles
        """
        trajectory = goal_handle.request.trajectory
        
        # Log trajectory info
        self.get_logger().info(f"Trajectory has {len(trajectory.points)} points for joints: {trajectory.joint_names}")
        
        # Validate joints
        for joint in trajectory.joint_names:
            if joint not in expected_joints:
                self.get_logger().warn(f"Unexpected joint in trajectory: {joint}")

        # Execute each trajectory point
        for i, point in enumerate(trajectory.points):
            # Check for cancellation
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info("Trajectory canceled")
                return FollowJointTrajectory.Result()

            # Build positions dict for this point (in radians from MoveIt)
            positions = {}
            for j, joint_name in enumerate(trajectory.joint_names):
                if j < len(point.positions):
                    positions[joint_name] = point.positions[j]

            # Log the command being sent
            pos_str = ", ".join([f"{k}:{v:.3f}" for k, v in positions.items()])
            self.get_logger().info(f"Point {i+1}/{len(trajectory.points)}: {pos_str}")

            # Send to robot (conversion happens inside)
            try:
                self.send_positions_to_robot(positions)
            except Exception as e:
                self.get_logger().error(f"Failed to execute trajectory point {i}: {e}")
                goal_handle.abort()
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                return result

            # Wait for the appropriate time
            if i < len(trajectory.points) - 1:
                # Calculate time until next point
                current_time = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
                next_time = (trajectory.points[i + 1].time_from_start.sec + 
                            trajectory.points[i + 1].time_from_start.nanosec * 1e-9)
                wait_time = next_time - current_time
                
                if wait_time > 0:
                    time.sleep(wait_time)
            else:
                # Last point - give some time for the robot to reach the position
                time.sleep(0.1)

        # Success
        goal_handle.succeed()
        self.get_logger().info("Trajectory execution completed successfully")
        
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result

    def destroy_node(self):
        """Clean up when node is destroyed."""
        self.get_logger().info("Shutting down SO101 Bridge...")
        try:
            with self.robot_lock:
                self.robot.disconnect()
            self.get_logger().info("Disconnected from robot")
        except Exception as e:
            self.get_logger().warn(f"Error during disconnect: {e}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = So101Bridge()

    # Use multi-threaded executor for concurrent action handling
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
