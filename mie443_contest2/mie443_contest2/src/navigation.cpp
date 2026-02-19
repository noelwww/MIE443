#include "mie443_contest2/navigation.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

Navigation::Navigation(std::shared_ptr<rclcpp::Node> node) : node_(node) {
	// Set up action client for Nav2
	action_client_ = rclcpp_action::create_client<NavigateToPose>(node_, "navigate_to_pose");

	// Wait for action server to be available
	if (!action_client_->wait_for_action_server(std::chrono::seconds(10))) {
		RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting");
	} else {
		RCLCPP_INFO(node_->get_logger(), "Navigation action client initialized");
	}
}

bool Navigation::moveToGoal(double xGoal, double yGoal, double phiGoal) {
	// Check if action server is available
	if (!action_client_->action_server_is_ready()) {
		RCLCPP_ERROR(node_->get_logger(), "Action server not available");
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

	RCLCPP_INFO(node_->get_logger(), "Sending goal location (%.2f, %.2f, %.2f)...", xGoal, yGoal, phiGoal);

	// Send goal
	auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

	// Set up response callback (optional, for logging)
	send_goal_options.goal_response_callback =
		[this](const GoalHandleNavigateToPose::SharedPtr & goal_handle) {
			if (!goal_handle) {
				RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
			} else {
				RCLCPP_INFO(node_->get_logger(), "Goal accepted by server, waiting for result");
			}
		};

	auto goal_handle_future = action_client_->async_send_goal(goal_msg, send_goal_options);

	// Wait for goal to be accepted
	if (rclcpp::spin_until_future_complete(node_, goal_handle_future) !=
		rclcpp::FutureReturnCode::SUCCESS) {
		RCLCPP_ERROR(node_->get_logger(), "Failed to send goal");
		return false;
	}

	auto goal_handle = goal_handle_future.get();
	if (!goal_handle) {
		RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
		return false;
	}

	// Wait for result
	auto result_future = action_client_->async_get_result(goal_handle);

	if (rclcpp::spin_until_future_complete(node_, result_future) !=
		rclcpp::FutureReturnCode::SUCCESS) {
		RCLCPP_ERROR(node_->get_logger(), "Failed to get result");
		return false;
	}

	auto result = result_future.get();

	switch (result.code) {
		case rclcpp_action::ResultCode::SUCCEEDED:
			RCLCPP_INFO(node_->get_logger(), "Goal reached successfully");
			return true;
		case rclcpp_action::ResultCode::ABORTED:
			RCLCPP_ERROR(node_->get_logger(), "Goal was aborted");
			return false;
		case rclcpp_action::ResultCode::CANCELED:
			RCLCPP_ERROR(node_->get_logger(), "Goal was canceled");
			return false;
		default:
			RCLCPP_ERROR(node_->get_logger(), "Unknown result code");
			return false;
	}
}
