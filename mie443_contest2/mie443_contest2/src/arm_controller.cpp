#include "mie443_contest2/arm_controller.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

ArmController::ArmController(std::shared_ptr<rclcpp::Node> node) : node_(node) {
	RCLCPP_INFO(node_->get_logger(), "Initializing ArmController...");

	try {
		// Initialize MoveIt MoveGroup interfaces for arm and gripper
		// These planning groups are defined in the so101.srdf file
		arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "arm");
		gripper_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "gripper");

		// Set planning parameters for better motion planning
		arm_group_->setPlanningTime(5.0);  // Allow up to 5 seconds for planning
		arm_group_->setNumPlanningAttempts(10);  // Try multiple times to find a plan
		arm_group_->setMaxVelocityScalingFactor(0.5);  // Limit velocity to 50% of max
		arm_group_->setMaxAccelerationScalingFactor(0.5);  // Limit acceleration to 50% of max

		gripper_group_->setPlanningTime(3.0);
		gripper_group_->setNumPlanningAttempts(5);

		RCLCPP_INFO(node_->get_logger(), "ArmController initialized successfully");
		RCLCPP_INFO(node_->get_logger(), "Arm planning frame: %s", arm_group_->getPlanningFrame().c_str());			// world
		RCLCPP_INFO(node_->get_logger(), "Arm end effector link: %s", arm_group_->getEndEffectorLink().c_str());	// gripper
	} catch (const std::exception& e) {
		RCLCPP_ERROR(node_->get_logger(), "Failed to initialize ArmController: %s", e.what());
		RCLCPP_ERROR(node_->get_logger(), "Make sure MoveIt move_group node is running!");
		throw;
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
		bool success = (arm_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

		if (!success) {
			RCLCPP_ERROR(node_->get_logger(), "Planning failed! Pose may be unreachable (5-DOF limitation)");
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

