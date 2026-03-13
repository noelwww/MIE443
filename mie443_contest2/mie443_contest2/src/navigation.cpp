#include "mie443_contest2/navigation.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

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
				// ETA from estimated_time_remaining
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
						"Nav2 result: ABORTED. Possible causes:\n"
						"  - No valid path found (goal may be unreachable/blocked)\n"
						"  - Robot got stuck and recovery behaviors failed\n"
						"  - Costmap shows obstacle at goal location\n"
						"  - Controller failed to follow the planned path\n"
						"  Check Nav2 logs: ros2 topic echo /bt_navigator/transition_event");
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
				"Goal ABORTED for (%.2f, %.2f). Path planning or execution failed.", 
				xGoal, yGoal);
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
			// Log every 5 seconds so user knows we're still waiting
			static long last_log = -1;
			if (elapsed != last_log) {
				RCLCPP_INFO(node_->get_logger(), "Still waiting for global costmap... (%lds)", elapsed);
				last_log = elapsed;
			}
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}

	// Set Nav2 goal checker tolerance to 0.1m (default is 0.25m)
	if (costmap_received.load()) {
		try {
			auto param_client = std::make_shared<rclcpp::SyncParametersClient>(
				node_, "/controller_server");
			if (param_client->wait_for_service(std::chrono::seconds(5))) {
				param_client->set_parameters({
					rclcpp::Parameter("goal_checker.xy_goal_tolerance", 0.1),
					rclcpp::Parameter("goal_checker.yaw_goal_tolerance", 0.15)
				});
				RCLCPP_INFO(node_->get_logger(),
					"Nav2 goal tolerance set: xy=0.1m, yaw=0.15rad (8.6deg)");
			} else {
				RCLCPP_WARN(node_->get_logger(),
					"Could not reach /controller_server to set goal tolerance.");
			}
		} catch (const std::exception& e) {
			RCLCPP_WARN(node_->get_logger(),
				"Failed to set goal tolerance: %s", e.what());
		}
	}

	// Subscription goes out of scope and is destroyed (no longer needed)
	return costmap_received.load();
}
