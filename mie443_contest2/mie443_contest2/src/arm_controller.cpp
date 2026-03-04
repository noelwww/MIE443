#include "mie443_contest2/arm_controller.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

ArmController::ArmController(std::shared_ptr<rclcpp::Node> node) : node_(node) {
	RCLCPP_INFO(node_->get_logger(), "Initializing ArmController...");

	try {
		// ── Create a dedicated node for MoveIt ──
		// The main "contest2" node is already spun by a MultiThreadedExecutor.
		// MoveGroupInterface creates its own internal callback group (auto_add=false)
		// and its own internal executor to handle action-client callbacks.
		//
		// HOWEVER, DDS graph-change events (needed for action-server *discovery*)
		// are delivered on the node's DEFAULT callback group.  If no executor is
		// spinning that default group, wait_for_action_server() will silently
		// time out and plan() later reports "action client/server not ready".
		//
		// Solution:
		// 1. Create a separate node (arm_moveit) so we don't conflict with the
		//    main contest2 MultiThreadedExecutor.
		// 2. Spin it on its OWN SingleThreadedExecutor in a background thread
		//    so DDS discovery events get processed.
		// 3. MoveGroupInterface's internal executor handles its own callback
		//    group (auto_add=false, so our executor won't touch it).  No conflict.
		RCLCPP_INFO(node_->get_logger(), "Creating dedicated MoveIt node...");
		moveit_node_ = std::make_shared<rclcpp::Node>("arm_moveit");

		// Copy the robot description parameters from the main node
		moveit_node_->declare_parameter("robot_description",
			node_->get_parameter("robot_description").as_string());
		moveit_node_->declare_parameter("robot_description_semantic",
			node_->get_parameter("robot_description_semantic").as_string());
		moveit_node_->declare_parameter(
			"robot_description_kinematics.arm.kinematics_solver",
			node_->get_parameter("robot_description_kinematics.arm.kinematics_solver").as_string());
		moveit_node_->declare_parameter(
			"robot_description_kinematics.arm.kinematics_solver_search_resolution",
			node_->get_parameter("robot_description_kinematics.arm.kinematics_solver_search_resolution").as_double());
		moveit_node_->declare_parameter(
			"robot_description_kinematics.arm.kinematics_solver_timeout",
			node_->get_parameter("robot_description_kinematics.arm.kinematics_solver_timeout").as_double());
		moveit_node_->declare_parameter(
			"robot_description_kinematics.arm.position_only_ik",
			node_->get_parameter("robot_description_kinematics.arm.position_only_ik").as_bool());

		RCLCPP_INFO(node_->get_logger(), "Robot parameters copied to dedicated MoveIt node.");

		// ── Spin the dedicated node for DDS discovery ──
		moveit_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
		moveit_executor_->add_node(moveit_node_);
		moveit_spinner_ = std::thread([this]() { moveit_executor_->spin(); });
		RCLCPP_INFO(node_->get_logger(), "MoveIt node executor spinning (DDS discovery active).");

		// Brief pause to let DDS discover existing nodes/endpoints
		std::this_thread::sleep_for(std::chrono::seconds(2));

		// ── Connect to move_group with a finite timeout ──
		auto timeout = rclcpp::Duration::from_seconds(30.0);

		RCLCPP_INFO(node_->get_logger(), "Connecting to move_group action server (timeout: 30s)...");
		arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
			moveit_node_, "arm", std::shared_ptr<tf2_ros::Buffer>(), timeout);
		RCLCPP_INFO(node_->get_logger(), "arm MoveGroupInterface created.");

		gripper_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
			moveit_node_, "gripper", std::shared_ptr<tf2_ros::Buffer>(), timeout);
		RCLCPP_INFO(node_->get_logger(), "gripper MoveGroupInterface created.");

		// Set planning parameters for better motion planning
		arm_group_->setPlanningTime(5.0);  // Allow up to 5 seconds for planning
		arm_group_->setNumPlanningAttempts(10);  // Try multiple times to find a plan
		arm_group_->setMaxVelocityScalingFactor(0.5);  // Limit velocity to 50% of max
		arm_group_->setMaxAccelerationScalingFactor(0.5);  // Limit acceleration to 50% of max

		gripper_group_->setPlanningTime(3.0);
		gripper_group_->setNumPlanningAttempts(5);

		RCLCPP_INFO(node_->get_logger(), "ArmController initialized successfully");
		RCLCPP_INFO(node_->get_logger(), "Arm planning frame: %s", arm_group_->getPlanningFrame().c_str());
		RCLCPP_INFO(node_->get_logger(), "Arm end effector link: %s", arm_group_->getEndEffectorLink().c_str());
	} catch (const std::exception& e) {
		RCLCPP_ERROR(node_->get_logger(), "Failed to initialize ArmController: %s", e.what());
		RCLCPP_ERROR(node_->get_logger(), "Make sure the MoveIt move_group node is running:");
		RCLCPP_ERROR(node_->get_logger(), "  ros2 launch lerobot_moveit so101_laptop.launch.py");
		throw;
	}
}

ArmController::~ArmController() {
	if (moveit_executor_) {
		moveit_executor_->cancel();
	}
	if (moveit_spinner_.joinable()) {
		moveit_spinner_.join();
	}
}

bool ArmController::moveToCartesianPose(double x, double y, double z, 
                                         double roll, double pitch, double yaw) {
	RCLCPP_INFO(node_->get_logger(), 
	            "Planning arm motion to Cartesian pose: (%.3f, %.3f, %.3f) with RPY: (%.3f, %.3f, %.3f)",
	            x, y, z, roll, pitch, yaw);

	// Convert roll, pitch, yaw to quaternion
	tf2::Quaternion q;
	q.setRPY(roll, pitch, yaw);
	return moveToCartesianPose(x, y, z, q.x(), q.y(), q.z(), q.w());
}

bool ArmController::moveToCartesianPose(double x, double y, double z,
                                         double qx, double qy, double qz, double qw) {
	RCLCPP_INFO(node_->get_logger(), 
	            "Planning arm motion to Cartesian pose: (%.3f, %.3f, %.3f) with quaternion: (%.3f, %.3f, %.3f, %.3f)",
	            x, y, z, qx, qy, qz, qw);

	try {
		// Create target pose
		geometry_msgs::msg::Pose target_pose;
		target_pose.position.x = x;
		target_pose.position.y = y;
		target_pose.position.z = z;
		target_pose.orientation.x = qx;
		target_pose.orientation.y = qy;
		target_pose.orientation.z = qz;
		target_pose.orientation.w = qw;

		// Set the target pose
		arm_group_->setPoseTarget(target_pose);

		// Plan the motion
		RCLCPP_INFO(node_->get_logger(), "Planning trajectory...");
		moveit::planning_interface::MoveGroupInterface::Plan plan;
		auto result = arm_group_->plan(plan);
		bool success = (result == moveit::core::MoveItErrorCode::SUCCESS);

		if (!success) {
			RCLCPP_ERROR(node_->get_logger(), "Planning failed (MoveIt error code: %d).", result.val);
			RCLCPP_ERROR(node_->get_logger(),
				"If error is FAILURE (-1): move_group action server is not connected. "
				"Kill zombies (killall -9 move_group) and relaunch.");
			RCLCPP_ERROR(node_->get_logger(),
				"If error is PLANNING_FAILED: pose may be unreachable (5-DOF limitation).");
			return false;
		}

		RCLCPP_INFO(node_->get_logger(), "Plan found! Executing motion...");

		// Execute the planned motion
		success = (arm_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);

		if (success) {
			RCLCPP_INFO(node_->get_logger(), "Motion executed successfully!");
			return true;
		} else {
			RCLCPP_ERROR(node_->get_logger(), "Motion execution failed!");
			return false;
		}
	} catch (const std::exception& e) {
		RCLCPP_ERROR(node_->get_logger(), "Exception during arm motion: %s", e.what());
		return false;
	}
}

bool ArmController::moveGripper(double position) {
	RCLCPP_INFO(node_->get_logger(), "Moving gripper to position: %.3f", position);

	try {
		// Set joint value target for the gripper
		std::vector<double> joint_values = {position};
		gripper_group_->setJointValueTarget(joint_values);

		// Plan and execute
		moveit::planning_interface::MoveGroupInterface::Plan plan;
		bool success = (gripper_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

		if (!success) {
			RCLCPP_ERROR(node_->get_logger(), "Gripper planning failed!");
			return false;
		}

		success = (gripper_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);

		if (success) {
			RCLCPP_INFO(node_->get_logger(), "Gripper moved successfully!");
			return true;
		} else {
			RCLCPP_ERROR(node_->get_logger(), "Gripper execution failed!");
			return false;
		}
	} catch (const std::exception& e) {
		RCLCPP_ERROR(node_->get_logger(), "Exception during gripper motion: %s", e.what());
		return false;
	}
}

bool ArmController::openGripper() {
	RCLCPP_INFO(node_->get_logger(), "Opening gripper...");
	return moveGripper(0.8);  // Open position
}

bool ArmController::closeGripper() {
	RCLCPP_INFO(node_->get_logger(), "Closing gripper...");
	return moveGripper(0.0);  // Closed position
}

