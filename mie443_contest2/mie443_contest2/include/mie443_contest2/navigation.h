#pragma once

#include <memory>
#include <mutex>
#include <vector>
#include <utility>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/action/spin.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

class Navigation {
public:
	struct Pose2D {
		double x = 0.0;
		double y = 0.0;
		double yaw = 0.0;
	};

	struct Viewpoint {
		double x;
		double y;
		double yaw;   // Facing toward the obstacle centroid
	};

	Navigation(std::shared_ptr<rclcpp::Node> node);
	bool moveToGoal(double xGoal, double yGoal, double phiGoal, double timeout_sec = 60.0);
	bool spin(double angle_rad, double timeout_sec = 15.0);
	/// Wait for Nav2 global costmap to be ready (indicates planning will work)
	bool waitUntilReady(double timeout_sec = 30.0);

	/// Get the latest AMCL pose. Returns false if no pose received yet.
	bool getLatestAmclPose(Pose2D& pose_out);

	/// Find the centroid of the nearest obstacle (bin) near the robot using costmap.
	/// Returns true if an obstacle cluster was found.
	/// Falls back to (fallback_x, fallback_y) if costmap is unavailable.
	bool findNearestObstacle(double robot_x, double robot_y,
	                         double& obs_x, double& obs_y,
	                         double search_radius = 1.5,
	                         double fallback_x = 0.0, double fallback_y = 0.0,
	                         double max_bin_area = 0.5,
	                         int cost_threshold = 80);

	/// Generate orbit viewpoints around an obstacle centroid.
	/// Each viewpoint faces toward (obs_x, obs_y) and is verified to be in free space.
	/// Sorted by angular distance from current robot heading, then by proximity.
	std::vector<Viewpoint> generateViewpoints(double obs_x, double obs_y,
	                                          double robot_x, double robot_y,
	                                          double orbit_radius = 0.55,
	                                          int num_candidates = 12,
	                                          double robot_radius = 0.22,
	                                          double fallback_radius_ratio = 0.7);

private:
	bool sendGoalAndWait(double xGoal, double yGoal, double phiGoal, double timeout_sec);
	double normalizeAngle(double angle_rad) const;
	void amclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
	void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

	/// Check if a world-coordinate point is in free space on the costmap.
	/// robot_radius inflates the check to ensure the robot footprint fits.
	bool isInFreeSpace(double wx, double wy, double robot_radius = 0.22, int cost_threshold = 80);

	std::shared_ptr<rclcpp::Node> node_;
	rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;
	rclcpp_action::Client<nav2_msgs::action::Spin>::SharedPtr spin_client_;

	// AMCL pose tracking
	rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
	std::mutex amcl_pose_mutex_;
	Pose2D latest_amcl_pose_;
	bool has_amcl_pose_{false};

	// Costmap for obstacle detection
	rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
	std::mutex costmap_mutex_;
	std::vector<int8_t> costmap_data_;
	double costmap_resolution_{0.0};
	double costmap_origin_x_{0.0};
	double costmap_origin_y_{0.0};
	uint32_t costmap_width_{0};
	uint32_t costmap_height_{0};
	bool has_costmap_{false};
};
