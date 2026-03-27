#include "mie443_contest2/navigation.h"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

// =============================================================================
// Nav2 Tuning Parameters — edit these to tune navigation behavior
// =============================================================================
namespace NavParams {
    double XY_GOAL_TOLERANCE      = 0.25;   // meters — how close robot must be to goal
    double YAW_GOAL_TOLERANCE     = 0.15;   // radians — heading accuracy at goal
    double INFLATION_RADIUS       = 0.18;   // meters — obstacle inflation in costmaps
    double COST_SCALING_FACTOR    = 4.0;    // how steeply cost drops from obstacles
    double PROGRESS_RADIUS        = 0.10;   // meters — min movement to avoid "stuck"
    double PROGRESS_TIME          = 30.0;   // seconds — time window for progress check
    double TF_TOLERANCE           = 2.0;    // seconds — transform staleness tolerance
    double FOLLOW_PATH_TF_TOL    = 2.0;    // seconds — FollowPath transform tolerance
    double COLLISION_TIME_BEFORE  = 0.3;    // seconds — stop-before-collision threshold
    double COLLISION_SIM_STEP     = 0.01;   // seconds — collision simulation time step
    double COLLISION_SOURCE_TIMEOUT = 0.0;   // seconds — 0 disables staleness check (DDS drops cause growing drift)
    double COLLISION_STOP_PUB_TIMEOUT = 2.0; // seconds — how long stop cmd persists after source goes invalid
}

Navigation::Navigation(std::shared_ptr<rclcpp::Node> node) : node_(node) {
	// Set up action client for Nav2
	action_client_ = rclcpp_action::create_client<NavigateToPose>(node_, "navigate_to_pose");

	// Wait for action server to be available
	RCLCPP_INFO(node_->get_logger(), "Waiting for navigate_to_pose action server...");
	if (!action_client_->wait_for_action_server(std::chrono::seconds(10))) {
		RCLCPP_ERROR(node_->get_logger(), 
			"navigate_to_pose action server NOT available after 10s! "
			"Check that Nav2 is running: ros2 launch turtlebot4_navigation nav2.launch.py");
	} else {
		RCLCPP_INFO(node_->get_logger(), "Navigation action client initialized");
	}

	// Set up Spin action client (part of Nav2 behavior server)
	spin_client_ = rclcpp_action::create_client<nav2_msgs::action::Spin>(node_, "spin");
	if (!spin_client_->wait_for_action_server(std::chrono::seconds(5))) {
		RCLCPP_WARN(node_->get_logger(), 
			"Spin action server not available after 5s. "
			"In-place rotation will fall back to moveToGoal.");
	} else {
		RCLCPP_INFO(node_->get_logger(), "Spin action client initialized");
	}

	// Set up BackUp action client (part of Nav2 behavior server)
	backup_client_ = rclcpp_action::create_client<nav2_msgs::action::BackUp>(node_, "backup");
	if (!backup_client_->wait_for_action_server(std::chrono::seconds(5))) {
		RCLCPP_WARN(node_->get_logger(),
			"BackUp action server not available after 5s. "
			"Backup recovery will not work.");
	} else {
		RCLCPP_INFO(node_->get_logger(), "BackUp action client initialized");
	}
}

bool Navigation::moveToGoal(double xGoal, double yGoal, double phiGoal, double timeout_sec) {
	// Check if action server is available
	if (!action_client_->action_server_is_ready()) {
		RCLCPP_ERROR(node_->get_logger(), 
			"navigate_to_pose action server not ready! "
			"Is Nav2 running? Try: ros2 action list");
		return false;
	}

	// Create quaternion from yaw angle using tf2
	tf2::Quaternion q;
	q.setRPY(0, 0, phiGoal);

	// Set up goal message
	auto goal_msg = NavigateToPose::Goal();
	goal_msg.pose.header.frame_id = "map";
	goal_msg.pose.header.stamp = node_->now();
	goal_msg.pose.pose.position.x = xGoal;
	goal_msg.pose.pose.position.y = yGoal;
	goal_msg.pose.pose.position.z = 0.0;
	goal_msg.pose.pose.orientation = tf2::toMsg(q);

	RCLCPP_INFO(node_->get_logger(), 
		"Sending Nav2 goal: x=%.2f, y=%.2f, yaw=%.2f rad (%.1f deg) in frame '%s'", 
		xGoal, yGoal, phiGoal, phiGoal * 180.0 / M_PI, 
		goal_msg.pose.header.frame_id.c_str());

	// Send goal with callbacks
	auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

	// Goal response callback
	send_goal_options.goal_response_callback =
		[this](const GoalHandleNavigateToPose::SharedPtr & goal_handle) {
			if (!goal_handle) {
				RCLCPP_ERROR(node_->get_logger(), 
					"Goal REJECTED by Nav2! Possible causes:\n"
					"  - Robot is not localized (set 2D Pose Estimate in RViz)\n"
					"  - Goal is outside the map bounds\n"
					"  - Goal is inside an obstacle in the costmap\n"
					"  - Nav2 planner is not configured or crashed");
			} else {
				RCLCPP_INFO(node_->get_logger(), "Goal accepted by Nav2 (ID: %s)", 
					rclcpp_action::to_string(goal_handle->get_goal_id()).c_str());
			}
		};

	// Feedback callback — shows real-time navigation progress
	send_goal_options.feedback_callback =
		[this](GoalHandleNavigateToPose::SharedPtr,
		       const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
			static int feedback_count = 0;
			feedback_count++;
			// Log every 10th feedback to avoid spam (~1Hz from Nav2)
			if (feedback_count % 10 == 0) {
				double remaining_dist = feedback->distance_remaining;
				auto eta = feedback->estimated_time_remaining;
				RCLCPP_INFO(node_->get_logger(), 
					"Nav2 progress: %.2fm remaining, ETA: %ds, at (%.2f, %.2f)",
					remaining_dist,
					(int)eta.sec,
					feedback->current_pose.pose.position.x,
					feedback->current_pose.pose.position.y);
			}
		};

	// Result callback (informational only, we also poll below)
	send_goal_options.result_callback =
		[this](const GoalHandleNavigateToPose::WrappedResult & wrapped_result) {
			switch (wrapped_result.code) {
				case rclcpp_action::ResultCode::SUCCEEDED:
					RCLCPP_INFO(node_->get_logger(), "Nav2 result: SUCCEEDED");
					break;
				case rclcpp_action::ResultCode::ABORTED:
					RCLCPP_ERROR(node_->get_logger(), 
						"Nav2 result: ABORTED (error_code=%u, msg='%s')\n"
						"  Possible causes:\n"
						"  - No valid path found (goal may be unreachable/blocked)\n"
						"  - Robot got stuck and recovery behaviors failed\n"
						"  - TF transform error (clock skew between Pi and laptop)\n"
						"  - Progress checker timeout (robot moving too slowly)",
						wrapped_result.result->error_code,
						wrapped_result.result->error_msg.c_str());
					break;
				case rclcpp_action::ResultCode::CANCELED:
					RCLCPP_WARN(node_->get_logger(), "Nav2 result: CANCELED");
					break;
				default:
					RCLCPP_ERROR(node_->get_logger(), "Nav2 result: UNKNOWN code %d", 
						(int)wrapped_result.code);
					break;
			}
		};

	auto goal_handle_future = action_client_->async_send_goal(goal_msg, send_goal_options);

	// Wait for goal to be accepted
	if (goal_handle_future.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
		RCLCPP_ERROR(node_->get_logger(), 
			"Timeout waiting for goal acceptance (10s). Nav2 may be overloaded or not running.");
		return false;
	}

	auto goal_handle = goal_handle_future.get();
	if (!goal_handle) {
		RCLCPP_ERROR(node_->get_logger(), "Goal was rejected — see above for details.");
		return false;
	}

	// Wait for result
	auto result_future = action_client_->async_get_result(goal_handle);
	int timeout_int = std::max(5, static_cast<int>(std::ceil(timeout_sec)));

	if (result_future.wait_for(std::chrono::seconds(timeout_int)) != std::future_status::ready) {
		RCLCPP_WARN(node_->get_logger(), 
			"Navigation timeout (%ds)! Cancelling goal.", timeout_int);
		action_client_->async_cancel_goal(goal_handle);
		return false;
	}

	auto result = result_future.get();

	switch (result.code) {
		case rclcpp_action::ResultCode::SUCCEEDED:
			RCLCPP_INFO(node_->get_logger(), "Goal reached successfully!");
			return true;
		case rclcpp_action::ResultCode::ABORTED:
			RCLCPP_ERROR(node_->get_logger(), 
				"Goal ABORTED for (%.2f, %.2f). error_code=%u msg='%s'", 
				xGoal, yGoal,
				result.result->error_code,
				result.result->error_msg.c_str());
			return false;
		case rclcpp_action::ResultCode::CANCELED:
			RCLCPP_WARN(node_->get_logger(), "Goal was canceled.");
			return false;
		default:
			RCLCPP_ERROR(node_->get_logger(), "Unknown result code: %d", (int)result.code);
			return false;
	}
}

bool Navigation::spin(double angle_rad, double timeout_sec) {
	if (!spin_client_ || !spin_client_->action_server_is_ready()) {
		RCLCPP_WARN(node_->get_logger(), 
			"Spin action server not ready. Cannot spin %.1f deg.",
			angle_rad * 180.0 / M_PI);
		return false;
	}

	RCLCPP_INFO(node_->get_logger(), "Spinning %.1f degrees in place...",
		angle_rad * 180.0 / M_PI);

	auto goal_msg = nav2_msgs::action::Spin::Goal();
	goal_msg.target_yaw = static_cast<float>(angle_rad);
	goal_msg.time_allowance.sec = static_cast<int32_t>(timeout_sec);
	goal_msg.time_allowance.nanosec = 0;

	auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::Spin>::SendGoalOptions();

	auto goal_handle_future = spin_client_->async_send_goal(goal_msg, send_goal_options);

	if (goal_handle_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
		RCLCPP_ERROR(node_->get_logger(), "Spin goal acceptance timeout (5s).");
		return false;
	}

	auto goal_handle = goal_handle_future.get();
	if (!goal_handle) {
		RCLCPP_ERROR(node_->get_logger(), "Spin goal was rejected by Nav2.");
		return false;
	}

	auto result_future = spin_client_->async_get_result(goal_handle);
	auto wait_duration = std::chrono::seconds(static_cast<int>(timeout_sec) + 5);

	if (result_future.wait_for(wait_duration) != std::future_status::ready) {
		RCLCPP_ERROR(node_->get_logger(), "Spin timeout (%.0fs). Cancelling.", timeout_sec);
		spin_client_->async_cancel_goal(goal_handle);
		return false;
	}

	auto result = result_future.get();
	if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
		RCLCPP_INFO(node_->get_logger(), "Spin completed successfully.");
		return true;
	}

	RCLCPP_WARN(node_->get_logger(), "Spin finished with code %d.", (int)result.code);
	return false;
}

bool Navigation::backup(double distance_m, double speed, double timeout_sec) {
	if (!backup_client_ || !backup_client_->action_server_is_ready()) {
		RCLCPP_WARN(node_->get_logger(),
			"BackUp action server not ready. Cannot back up %.2fm.", distance_m);
		return false;
	}

	RCLCPP_INFO(node_->get_logger(), "Backing up %.2fm at %.2f m/s...",
		distance_m, speed);

	auto goal_msg = nav2_msgs::action::BackUp::Goal();
	goal_msg.target.x = -distance_m;  // negative X = backward in base_link
	goal_msg.target.y = 0.0;
	goal_msg.target.z = 0.0;
	goal_msg.speed = static_cast<float>(speed);
	goal_msg.time_allowance.sec = static_cast<int32_t>(timeout_sec);
	goal_msg.time_allowance.nanosec = 0;

	auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::BackUp>::SendGoalOptions();

	auto goal_handle_future = backup_client_->async_send_goal(goal_msg, send_goal_options);

	if (goal_handle_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
		RCLCPP_ERROR(node_->get_logger(), "BackUp goal acceptance timeout (5s).");
		return false;
	}

	auto goal_handle = goal_handle_future.get();
	if (!goal_handle) {
		RCLCPP_ERROR(node_->get_logger(), "BackUp goal was rejected by Nav2.");
		return false;
	}

	auto result_future = backup_client_->async_get_result(goal_handle);
	auto wait_duration = std::chrono::seconds(static_cast<int>(timeout_sec) + 5);

	if (result_future.wait_for(wait_duration) != std::future_status::ready) {
		RCLCPP_ERROR(node_->get_logger(), "BackUp timeout (%.0fs). Cancelling.", timeout_sec);
		backup_client_->async_cancel_goal(goal_handle);
		return false;
	}

	auto result = result_future.get();
	if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
		RCLCPP_INFO(node_->get_logger(), "BackUp completed successfully.");
		return true;
	}

	RCLCPP_WARN(node_->get_logger(), "BackUp finished with code %d (error: %u, msg: '%s').",
		(int)result.code, result.result->error_code, result.result->error_msg.c_str());
	return false;
}

bool Navigation::waitUntilReady(double timeout_sec) {
	RCLCPP_INFO(node_->get_logger(),
		"Waiting up to %.0fs for Nav2 global costmap to be ready...", timeout_sec);

	std::atomic<bool> costmap_received{false};
	auto costmap_sub = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
		"/global_costmap/costmap",
		rclcpp::QoS(1).transient_local().reliable(),
		[&costmap_received, this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
			if (!costmap_received.load()) {
				RCLCPP_INFO(node_->get_logger(),
					"Global costmap received (%ux%u, resolution %.3fm). Nav2 is ready!",
					msg->info.width, msg->info.height, msg->info.resolution);
				costmap_received.store(true);
			}
		});

	auto wait_start = std::chrono::steady_clock::now();
	while (rclcpp::ok() && !costmap_received.load()) {
		auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
			std::chrono::steady_clock::now() - wait_start).count();
		if (elapsed > static_cast<long>(timeout_sec)) {
			RCLCPP_WARN(node_->get_logger(),
				"Global costmap not received after %.0fs! Navigation may fail.\n"
				"  Check: ros2 topic hz /global_costmap/costmap", timeout_sec);
			return false;
		}
		if (elapsed > 0 && elapsed % 5 == 0) {
			static long last_log = -1;
			if (elapsed != last_log) {
				RCLCPP_INFO(node_->get_logger(), "Still waiting for global costmap... (%lds)", elapsed);
				last_log = elapsed;
			}
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}

	// =========================================================================
	// CRITICAL: Tune Nav2 parameters at runtime to prevent ABORT errors.
	//
	// Problem 1 — Progress checker is too strict:
	//   Default requires 0.5m movement in 10s. On this robot with clock skew
	//   the controller runs slowly and gets killed. Fix: 0.10m in 30s.
	//
	// Problem 2 — Transform tolerance is too tight:
	//   Pi clock is ~1-2s ahead of laptop. Default TF tolerance is 0.1-0.2s.
	//   This causes TF_ERROR (error_code 102) on controller/costmaps/collision.
	//   Fix: increase to 2.0s everywhere.
	//
	// Problem 3 — Wrong param name:
	//   TurtleBot4 nav2.yaml uses "general_goal_checker" (not "goal_checker").
	// =========================================================================
	if (costmap_received.load()) {
		// --- Controller server ---
		try {
			auto param_client = std::make_shared<rclcpp::SyncParametersClient>(
				node_, "/controller_server");
			if (param_client->wait_for_service(std::chrono::seconds(5))) {
				param_client->set_parameters({
					rclcpp::Parameter("progress_checker.required_movement_radius", NavParams::PROGRESS_RADIUS),
					rclcpp::Parameter("progress_checker.movement_time_allowance", NavParams::PROGRESS_TIME),
					rclcpp::Parameter("general_goal_checker.xy_goal_tolerance", NavParams::XY_GOAL_TOLERANCE),
					rclcpp::Parameter("general_goal_checker.yaw_goal_tolerance", NavParams::YAW_GOAL_TOLERANCE),
					rclcpp::Parameter("FollowPath.transform_tolerance", NavParams::FOLLOW_PATH_TF_TOL)
				});
				RCLCPP_INFO(node_->get_logger(),
					"Nav2 tuned: progress=%.2fm/%.0fs, goal=%.2fm/%.2frad, TF=%.1fs",
					NavParams::PROGRESS_RADIUS, NavParams::PROGRESS_TIME,
					NavParams::XY_GOAL_TOLERANCE, NavParams::YAW_GOAL_TOLERANCE,
					NavParams::FOLLOW_PATH_TF_TOL);
			} else {
				RCLCPP_WARN(node_->get_logger(),
					"Could not reach /controller_server to set params.");
			}
		} catch (const std::exception& e) {
			RCLCPP_WARN(node_->get_logger(),
				"Failed to set controller_server params: %s", e.what());
		}

		// --- Local costmap ---
		try {
			auto local_client = std::make_shared<rclcpp::SyncParametersClient>(
				node_, "/local_costmap/local_costmap");
			if (local_client->wait_for_service(std::chrono::seconds(3))) {
				local_client->set_parameters({
					rclcpp::Parameter("transform_tolerance", NavParams::TF_TOLERANCE),
					rclcpp::Parameter("inflation_layer.inflation_radius", NavParams::INFLATION_RADIUS),
					rclcpp::Parameter("inflation_layer.cost_scaling_factor", NavParams::COST_SCALING_FACTOR)
				});
				RCLCPP_INFO(node_->get_logger(),
					"Local costmap: TF tol=%.1fs, inflation=%.2fm, cost_scaling=%.1f",
					NavParams::TF_TOLERANCE, NavParams::INFLATION_RADIUS, NavParams::COST_SCALING_FACTOR);
			}
		} catch (const std::exception& e) {
			RCLCPP_WARN(node_->get_logger(),
				"Failed to set local costmap params: %s", e.what());
		}

		// --- Global costmap ---
		try {
			auto global_client = std::make_shared<rclcpp::SyncParametersClient>(
				node_, "/global_costmap/global_costmap");
			if (global_client->wait_for_service(std::chrono::seconds(3))) {
				global_client->set_parameters({
					rclcpp::Parameter("transform_tolerance", NavParams::TF_TOLERANCE),
					rclcpp::Parameter("inflation_layer.inflation_radius", NavParams::INFLATION_RADIUS),
					rclcpp::Parameter("inflation_layer.cost_scaling_factor", NavParams::COST_SCALING_FACTOR)
				});
				RCLCPP_INFO(node_->get_logger(),
					"Global costmap: TF tol=%.1fs, inflation=%.2fm, cost_scaling=%.1f",
					NavParams::TF_TOLERANCE, NavParams::INFLATION_RADIUS, NavParams::COST_SCALING_FACTOR);
			}
		} catch (const std::exception& e) {
			RCLCPP_WARN(node_->get_logger(),
				"Failed to set global costmap params: %s", e.what());
		}

		// --- Collision monitor ---
		// Reduce approach threshold so robot can get close to bins for AprilTag
		// detection and object drop. Default 1.2s blocks robot at ~0.21m.
		try {
			auto cm_client = std::make_shared<rclcpp::SyncParametersClient>(
				node_, "/collision_monitor");
			if (cm_client->wait_for_service(std::chrono::seconds(3))) {
				cm_client->set_parameters({
					rclcpp::Parameter("transform_tolerance", NavParams::TF_TOLERANCE),
					rclcpp::Parameter("source_timeout", NavParams::COLLISION_SOURCE_TIMEOUT),
					rclcpp::Parameter("stop_pub_timeout", NavParams::COLLISION_STOP_PUB_TIMEOUT),
					rclcpp::Parameter("FootprintApproach.time_before_collision", NavParams::COLLISION_TIME_BEFORE),
					rclcpp::Parameter("FootprintApproach.simulation_time_step", NavParams::COLLISION_SIM_STEP)
				});
				RCLCPP_INFO(node_->get_logger(),
					"Collision monitor: TF=%.1fs, src_timeout=%.1fs, stop_pub=%.1fs, approach=%.1fs",
					NavParams::TF_TOLERANCE, NavParams::COLLISION_SOURCE_TIMEOUT,
					NavParams::COLLISION_STOP_PUB_TIMEOUT, NavParams::COLLISION_TIME_BEFORE);
			}
		} catch (const std::exception& e) {
			RCLCPP_WARN(node_->get_logger(),
				"Failed to set collision_monitor params: %s", e.what());
		}
	}

	return costmap_received.load();
}
