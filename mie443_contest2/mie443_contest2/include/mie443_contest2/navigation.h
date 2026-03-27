#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/action/spin.hpp>
#include <nav2_msgs/action/back_up.hpp>

class Navigation {
public:
	Navigation(std::shared_ptr<rclcpp::Node> node);
	bool moveToGoal(double xGoal, double yGoal, double phiGoal, double timeout_sec = 60.0);
	bool spin(double angle_rad, double timeout_sec = 15.0);
	/// Back up a given distance (meters, positive = backward). Uses Nav2 BackUp behavior.
	bool backup(double distance_m = 0.3, double speed = 0.1, double timeout_sec = 10.0);
	/// Wait for Nav2 global costmap to be ready (indicates planning will work)
	bool waitUntilReady(double timeout_sec = 30.0);

private:
	std::shared_ptr<rclcpp::Node> node_;
	rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;
	rclcpp_action::Client<nav2_msgs::action::Spin>::SharedPtr spin_client_;
	rclcpp_action::Client<nav2_msgs::action::BackUp>::SharedPtr backup_client_;
};
