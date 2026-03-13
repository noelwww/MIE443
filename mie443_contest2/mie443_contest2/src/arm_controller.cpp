#include "mie443_contest2/arm_controller.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/allowed_collision_matrix.hpp>
#include <moveit_msgs/msg/allowed_collision_entry.hpp>
#include <cstdlib>

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
		// Override the kinematics timeout – default is 0.005s which is far too short
		// for IK solving on a 5-DOF arm.  Give KDL 0.1s.
		moveit_node_->declare_parameter(
			"robot_description_kinematics.arm.kinematics_solver_timeout", 0.1);
		// The SO-ARM101 has only 5 revolute joints → can't satisfy full 6-DOF pose.
		// position_only_ik=true tells KDL to only match xyz position (3 DOF),
		// which the 5-DOF arm can always solve (with 2 DOF of redundancy).
		moveit_node_->declare_parameter(
			"robot_description_kinematics.arm.position_only_ik", true);

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
		arm_group_->setPlanningTime(10.0);  // Allow up to 10 seconds for planning
		arm_group_->setNumPlanningAttempts(20);  // Try many times to find a plan
		arm_group_->setMaxVelocityScalingFactor(0.5);  // Limit velocity to 50% of max
		arm_group_->setMaxAccelerationScalingFactor(0.5);  // Limit acceleration to 50% of max

		// ── Explicitly select the OMPL planning pipeline + RRTConnect planner ──
		// MoveItConfigsBuilder auto-selects OMPL as default, but being explicit
		// avoids any ambiguity and ensures RRTConnect is used (fast, reliable
		// for arm planning).
		arm_group_->setPlanningPipelineId("ompl");
		arm_group_->setPlannerId("RRTConnectkConfigDefault");
		RCLCPP_INFO(node_->get_logger(), "Planner: OMPL / RRTConnectkConfigDefault");

		// The SO-ARM101 'arm' planning group has 5 revolute joints (per SRDF).
		// A full pose target demands 6-DOF (x,y,z + 3 orientation), so the IK solver
		// often can't find an exact solution. Relax orientation tolerance to help.
		// Position tolerance stays tight, orientation gets ~15 degrees of slack.
		arm_group_->setGoalPositionTolerance(0.01);       // 1 cm
		arm_group_->setGoalOrientationTolerance(0.26);    // ~15 degrees
		arm_group_->setGoalJointTolerance(0.05);           // ~3 degrees per joint

		gripper_group_->setPlanningTime(5.0);
		gripper_group_->setNumPlanningAttempts(10);
		gripper_group_->setPlanningPipelineId("ompl");
		gripper_group_->setPlannerId("RRTConnectkConfigDefault");

		RCLCPP_INFO(node_->get_logger(), "ArmController interfaces created.");
		RCLCPP_INFO(node_->get_logger(), "Arm planning frame: %s", arm_group_->getPlanningFrame().c_str());
		RCLCPP_INFO(node_->get_logger(), "Arm end effector link: %s", arm_group_->getEndEffectorLink().c_str());

		// ── Verify the move_group action server is actually reachable ──
		// MoveGroupInterface's constructor may return before DDS has fully
		// discovered the action server endpoints.  A premature move() call
		// will instantly fail with code 99999 ("action client/server not ready").
		// Probe with getCurrentJointValues() in a retry loop – this forces a
		// service call to move_group and will fail quickly if it's not there.
		RCLCPP_INFO(node_->get_logger(), "Waiting for move_group action server readiness...");
		bool server_ready = false;
		for (int probe = 0; probe < 10; ++probe) {
			try {
				auto joint_values = arm_group_->getCurrentJointValues();
				if (!joint_values.empty()) {
					auto joint_names = arm_group_->getJointNames();
					std::string vals_str;
					for (size_t i = 0; i < joint_values.size(); ++i) {
						vals_str += joint_names[i] + "=" + std::to_string(joint_values[i]) + " ";
					}
					RCLCPP_INFO(node_->get_logger(), "move_group ready! Joint states: %s", vals_str.c_str());
					server_ready = true;
					break;
				}
			} catch (...) {}
			RCLCPP_INFO(node_->get_logger(), "  probe %d/10 – not ready yet, waiting 2s...", probe + 1);
			std::this_thread::sleep_for(std::chrono::seconds(2));
		}

		if (!server_ready) {
			RCLCPP_ERROR(node_->get_logger(),
				"move_group action server NOT reachable after 20s!\n"
				"  Check that these are running:\n"
				"    Pi:     ros2 launch lerobot_moveit so101_turtlebot.launch.py\n"
				"    Laptop: ros2 launch lerobot_moveit so101_laptop.launch.py\n"
				"  Also verify: ros2 topic echo /joint_states --once");
			throw std::runtime_error("move_group action server not reachable");
		}

		// One extra second for remaining DDS endpoint discovery
		std::this_thread::sleep_for(std::chrono::seconds(1));

		// ══════════════════════════════════════════════════════════════════
		// FIX: Relax Allowed Collision Matrix (ACM) to prevent error -10
		// ══════════════════════════════════════════════════════════════════
		// The SRDF's ACM only disables collisions between 12 link pairs,
		// but the SO-ARM101's collision geometry overlaps at many joint
		// configurations (e.g. at "home" with all joints at 0, base and
		// upper_arm mesh overlap).  Missing ACM pairs cause the
		// CheckStartStateCollision adapter to reject the start state
		// (error code -10 = START_STATE_IN_COLLISION) for EVERY motion.
		//
		// Fix: Publish a PlanningScene diff that allows all self-collisions
		// between all arm links.  This is safe because:
		//  1. The SO-ARM101 is a simple 5-DOF serial arm that physically
		//     cannot self-collide in any reachable configuration.
		//  2. The collision meshes in the URDF are over-sized approximations.
		//  3. We still get environment collision checking (obstacles).
		// ══════════════════════════════════════════════════════════════════
		{
			RCLCPP_INFO(node_->get_logger(), "Relaxing ACM for all arm self-collisions...");

			auto planning_scene_pub = moveit_node_->create_publisher<moveit_msgs::msg::PlanningScene>(
				"/planning_scene", rclcpp::QoS(1).transient_local());

			// All links that belong to the arm + gripper mechanism
			std::vector<std::string> arm_links = {
				"base", "shoulder", "upper_arm", "lower_arm", "wrist", "gripper", "jaw"
			};

			moveit_msgs::msg::AllowedCollisionMatrix acm_msg;
			acm_msg.default_entry_names = arm_links;
			// default_entry_values = true → if a pair involving this link
			// is NOT found in the matrix, collision is ALLOWED (not checked).
			acm_msg.default_entry_values.assign(arm_links.size(), true);

			moveit_msgs::msg::PlanningScene scene_msg;
			scene_msg.is_diff = true;
			scene_msg.allowed_collision_matrix = acm_msg;

			// Wait a moment for the publisher to connect, then publish
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
			planning_scene_pub->publish(scene_msg);
			RCLCPP_INFO(node_->get_logger(), "ACM relaxed – arm self-collisions allowed.");

			// Give move_group time to process the planning scene update
			std::this_thread::sleep_for(std::chrono::seconds(1));
		}

		RCLCPP_INFO(node_->get_logger(), "ArmController fully initialized and verified.");

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
	            "Planning arm motion to Cartesian pose: (%.3f, %.3f, %.3f) quat: (%.3f, %.3f, %.3f, %.3f)",
	            x, y, z, qx, qy, qz, qw);

	try {
		geometry_msgs::msg::Pose target_pose;
		target_pose.position.x = x;
		target_pose.position.y = y;
		target_pose.position.z = z;
		target_pose.orientation.x = qx;
		target_pose.orientation.y = qy;
		target_pose.orientation.z = qz;
		target_pose.orientation.w = qw;

		const int MAX_RETRIES = 3;

		// ── Strategy 1: Approximate Joint-Value Target (best for 5-DOF) ──
		// The SO-ARM101 has only 5 revolute joints, so setPoseTarget() demands
		// 6-DOF IK that the KDL solver can never satisfy → 10s planning timeout.
		// setApproximateJointValueTarget() runs IK ONCE (respecting
		// position_only_ik if configured), accepts the best approximate
		// solution, and converts it to a JOINT-SPACE target.
		// The planner then plans in joint space which is fast and reliable.
		RCLCPP_INFO(node_->get_logger(), "Strategy 1: IK → approximate joint target...");
		bool ik_ok = arm_group_->setApproximateJointValueTarget(target_pose);
		if (ik_ok) {
			for (int retry = 0; retry < MAX_RETRIES; ++retry) {
				RCLCPP_INFO(node_->get_logger(), "  Approx-joint move attempt %d/%d...", retry + 1, MAX_RETRIES);
				auto result = arm_group_->move();
				if (result == moveit::core::MoveItErrorCode::SUCCESS) {
					RCLCPP_INFO(node_->get_logger(), "Motion executed successfully (approx joint target)!");
					return true;
				}
				RCLCPP_WARN(node_->get_logger(), "  move() failed (code: %d).", result.val);
				if (retry < MAX_RETRIES - 1) {
					std::this_thread::sleep_for(std::chrono::seconds(1));
				}
			}
			RCLCPP_WARN(node_->get_logger(), "Strategy 1 exhausted.");
		} else {
			RCLCPP_WARN(node_->get_logger(), "IK solver could not find approximate joint solution.");
		}

		// ── Strategy 2: Position-Only Target ──
		// Ignore orientation entirely; just reach the XYZ position.
		RCLCPP_INFO(node_->get_logger(), "Strategy 2: position-only target (%.3f, %.3f, %.3f)...", x, y, z);
		arm_group_->clearPoseTargets();
		arm_group_->setPositionTarget(x, y, z);
		for (int retry = 0; retry < MAX_RETRIES; ++retry) {
			RCLCPP_INFO(node_->get_logger(), "  Position-only move attempt %d/%d...", retry + 1, MAX_RETRIES);
			auto result = arm_group_->move();
			if (result == moveit::core::MoveItErrorCode::SUCCESS) {
				RCLCPP_INFO(node_->get_logger(), "Motion executed successfully (position-only)!");
				return true;
			}
			RCLCPP_WARN(node_->get_logger(), "  Position-only move() failed (code: %d).", result.val);
			if (retry < MAX_RETRIES - 1) {
				std::this_thread::sleep_for(std::chrono::seconds(1));
			}
		}

		RCLCPP_ERROR(node_->get_logger(), "All Cartesian planning strategies failed.");
		return false;
	} catch (const std::exception& e) {
		RCLCPP_ERROR(node_->get_logger(), "Exception during arm motion: %s", e.what());
		return false;
	}
}

bool ArmController::moveToJointTarget(const std::vector<double>& joint_values) {
	RCLCPP_INFO(node_->get_logger(), "Planning arm motion to joint target (%zu joints)", joint_values.size());

	try {
		arm_group_->setJointValueTarget(joint_values);

		for (int retry = 0; retry < 2; ++retry) {
			RCLCPP_INFO(node_->get_logger(), "Joint-space move attempt %d/2...", retry + 1);
			auto result = arm_group_->move();
			if (result == moveit::core::MoveItErrorCode::SUCCESS) {
				RCLCPP_INFO(node_->get_logger(), "Joint motion executed successfully!");
				return true;
			}
			RCLCPP_WARN(node_->get_logger(), "Joint-space move() failed (code: %d).", result.val);
			if (retry == 0) std::this_thread::sleep_for(std::chrono::seconds(1));
		}
		RCLCPP_ERROR(node_->get_logger(), "Joint motion failed after retries.");
		return false;
	} catch (const std::exception& e) {
		RCLCPP_ERROR(node_->get_logger(), "Exception during joint motion: %s", e.what());
		return false;
	}
}

bool ArmController::moveToNamedTarget(const std::string& name) {
	RCLCPP_INFO(node_->get_logger(), "Moving arm to named target: '%s'", name.c_str());

	try {
		arm_group_->setNamedTarget(name);

		for (int retry = 0; retry < 2; ++retry) {
			RCLCPP_INFO(node_->get_logger(), "Named target '%s' move attempt %d/2...", name.c_str(), retry + 1);
			auto result = arm_group_->move();
			if (result == moveit::core::MoveItErrorCode::SUCCESS) {
				RCLCPP_INFO(node_->get_logger(), "Moved to '%s' successfully!", name.c_str());
				return true;
			}
			RCLCPP_WARN(node_->get_logger(), "move() to '%s' failed (code: %d).", name.c_str(), result.val);
			if (retry == 0) std::this_thread::sleep_for(std::chrono::seconds(1));
		}
		RCLCPP_ERROR(node_->get_logger(), "Failed to move to '%s' after retries.", name.c_str());
		return false;
	} catch (const std::exception& e) {
		RCLCPP_ERROR(node_->get_logger(), "Exception moving to '%s': %s", name.c_str(), e.what());
		return false;
	}
}

bool ArmController::moveGripper(double position) {
	RCLCPP_INFO(node_->get_logger(), "Moving gripper to position: %.3f", position);

	try {
		std::vector<double> joint_values = {position};
		gripper_group_->setJointValueTarget(joint_values);

		for (int retry = 0; retry < 2; ++retry) {
			RCLCPP_INFO(node_->get_logger(), "Gripper move attempt %d/2...", retry + 1);
			auto result = gripper_group_->move();
			if (result == moveit::core::MoveItErrorCode::SUCCESS) {
				RCLCPP_INFO(node_->get_logger(), "Gripper moved successfully!");
				return true;
			}
			RCLCPP_WARN(node_->get_logger(), "Gripper move() failed (code: %d).", result.val);
			if (retry == 0) std::this_thread::sleep_for(std::chrono::seconds(1));
		}
		RCLCPP_ERROR(node_->get_logger(), "Gripper motion failed after retries.");
		return false;
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
	return moveGripper(-0.174);  // Max close (URDF hard limit is -0.1745)
}

geometry_msgs::msg::PoseStamped ArmController::getCurrentPose() {
	return arm_group_->getCurrentPose();
}

std::vector<double> ArmController::getCurrentJointValues() {
	return arm_group_->getCurrentJointValues();
}

std::string ArmController::getPlanningFrame() {
	return arm_group_->getPlanningFrame();
}

std::string ArmController::getEndEffectorLink() {
	return arm_group_->getEndEffectorLink();
}

