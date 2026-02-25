#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <geometry_msgs/msg/pose.hpp>


class ArmController {
public:
	ArmController(std::shared_ptr<rclcpp::Node> node);

	bool moveToCartesianPose(double x, double y, double z, 
	                         double roll, double pitch, double yaw);

	bool moveToCartesianPose(double x, double y, double z,
	                         double qx, double qy, double qz, double qw);

	bool moveGripper(double position);

	bool openGripper();

	bool closeGripper();

private:
	std::shared_ptr<rclcpp::Node> node_;
	std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
	std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;
};

