#include "mie443_contest2/robot_pose.h"
#include "tf2/utils.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

RobotPose::RobotPose(double x, double y, double phi) {
	this->x = x;
	this->y = y;
	this->phi = phi;
}

void RobotPose::poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
	// Extract yaw from quaternion using tf2
	phi = tf2::getYaw(msg->pose.pose.orientation);
	x = msg->pose.pose.position.x;
	y = msg->pose.pose.position.y;
}
