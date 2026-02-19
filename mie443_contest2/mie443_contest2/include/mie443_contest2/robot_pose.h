#pragma once

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

class RobotPose {
	public:
		double x;
		double y;
		double phi;
	public:
		RobotPose(double x, double y, double phi);
		void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
};
