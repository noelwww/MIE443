#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

class Navigation {
	public:
		Navigation(std::shared_ptr<rclcpp::Node> node);
		bool moveToGoal(double xGoal, double yGoal, double phiGoal);

	private:
		std::shared_ptr<rclcpp::Node> node_;
		rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;
};
