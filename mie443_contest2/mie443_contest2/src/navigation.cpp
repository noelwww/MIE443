#include "mie443_contest2/navigation.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <algorithm>
#include <queue>

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

	// Track AMCL pose so we can evaluate and refine final goal accuracy.
	auto amcl_qos = rclcpp::QoS(rclcpp::KeepLast(5)).transient_local().reliable();
	amcl_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
		"/amcl_pose",
		amcl_qos,
		std::bind(&Navigation::amclPoseCallback, this, std::placeholders::_1));

	// Subscribe to global costmap for obstacle detection (used in costmap orbit)
	auto costmap_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
	costmap_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
		"/global_costmap/costmap",
		costmap_qos,
		std::bind(&Navigation::costmapCallback, this, std::placeholders::_1));
	RCLCPP_INFO(node_->get_logger(), "Subscribed to /global_costmap/costmap for orbit planning.");
}

bool Navigation::moveToGoal(double xGoal, double yGoal, double phiGoal, double timeout_sec) {
	if (!sendGoalAndWait(xGoal, yGoal, phiGoal, timeout_sec)) {
		return false;
	}

	// Brief settle time — let AMCL update after the robot physically stops
	std::this_thread::sleep_for(std::chrono::milliseconds(500));

	// Log final pose error for diagnostics (no corrective re-send)
	Pose2D final_pose{};
	if (getLatestAmclPose(final_pose)) {
		double final_dx = xGoal - final_pose.x;
		double final_dy = yGoal - final_pose.y;
		double final_xy = std::sqrt(final_dx * final_dx + final_dy * final_dy);
		double final_yaw = std::abs(normalizeAngle(phiGoal - final_pose.yaw));
		RCLCPP_INFO(node_->get_logger(),
			"Final stop pose: target=(%.3f, %.3f, %.1fdeg), actual=(%.3f, %.3f, %.1fdeg), error=(%.3fm, %.2fdeg)",
			xGoal, yGoal, phiGoal * 180.0 / M_PI,
			final_pose.x, final_pose.y, final_pose.yaw * 180.0 / M_PI,
			final_xy, final_yaw * 180.0 / M_PI);
	}

	return true;
}

bool Navigation::sendGoalAndWait(double xGoal, double yGoal, double phiGoal, double timeout_sec) {
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

void Navigation::amclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
	std::lock_guard<std::mutex> lock(amcl_pose_mutex_);
	latest_amcl_pose_.x = msg->pose.pose.position.x;
	latest_amcl_pose_.y = msg->pose.pose.position.y;
	latest_amcl_pose_.yaw = tf2::getYaw(msg->pose.pose.orientation);
	has_amcl_pose_ = true;
}

bool Navigation::getLatestAmclPose(Pose2D& pose_out) {
	std::lock_guard<std::mutex> lock(amcl_pose_mutex_);
	if (!has_amcl_pose_) {
		return false;
	}
	pose_out = latest_amcl_pose_;
	return true;
}

double Navigation::normalizeAngle(double angle_rad) const {
	while (angle_rad > M_PI) angle_rad -= 2.0 * M_PI;
	while (angle_rad < -M_PI) angle_rad += 2.0 * M_PI;
	return angle_rad;
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
					rclcpp::Parameter("goal_checker.xy_goal_tolerance", 0.15),
					rclcpp::Parameter("goal_checker.yaw_goal_tolerance", 0.20)
				});
				RCLCPP_INFO(node_->get_logger(),
					"Nav2 goal tolerance set: xy=0.15m, yaw=0.20rad (11.5deg)");
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

// =============================================================================
// Costmap callback — stores latest global costmap for obstacle detection
// =============================================================================
void Navigation::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
	std::lock_guard<std::mutex> lock(costmap_mutex_);
	costmap_data_.assign(msg->data.begin(), msg->data.end());
	costmap_resolution_ = msg->info.resolution;
	costmap_origin_x_ = msg->info.origin.position.x;
	costmap_origin_y_ = msg->info.origin.position.y;
	costmap_width_ = msg->info.width;
	costmap_height_ = msg->info.height;
	if (!has_costmap_) {
		RCLCPP_INFO(node_->get_logger(),
			"Costmap stored for orbit planning (%ux%u, res=%.3fm).",
			costmap_width_, costmap_height_, costmap_resolution_);
		has_costmap_ = true;
	}
}

// =============================================================================
// isInFreeSpace — checks if a world point is traversable on the costmap
// =============================================================================
bool Navigation::isInFreeSpace(double wx, double wy, double robot_radius, int cost_threshold) {
	std::lock_guard<std::mutex> lock(costmap_mutex_);
	if (!has_costmap_ || costmap_resolution_ <= 0.0) return false;

	int gx = static_cast<int>((wx - costmap_origin_x_) / costmap_resolution_);
	int gy = static_cast<int>((wy - costmap_origin_y_) / costmap_resolution_);

	// Check a square footprint around the point
	int pad = static_cast<int>(std::ceil(robot_radius / costmap_resolution_));

	for (int dy = -pad; dy <= pad; ++dy) {
		for (int dx = -pad; dx <= pad; ++dx) {
			int cx = gx + dx;
			int cy = gy + dy;
			if (cx < 0 || cx >= (int)costmap_width_ || cy < 0 || cy >= (int)costmap_height_) {
				return false;  // Out of map = not safe
			}
			int idx = cy * (int)costmap_width_ + cx;
			int8_t cost = costmap_data_[idx];
			// cost >= threshold includes inscribed/lethal; -1 = unknown
			if (cost < 0 || cost >= cost_threshold) {
				return false;
			}
		}
	}
	return true;
}

// =============================================================================
// findNearestObstacle — BFS flood-fill to find the nearest obstacle cluster
//   Uses cost==100 (raw lethal) for clustering to ignore inflation.
//   Clusters < 0.5 m² are considered bins; larger are walls.
//   For wall clusters, returns the point nearest the robot as centroid.
// =============================================================================
bool Navigation::findNearestObstacle(double robot_x, double robot_y,
                                     double& obs_x, double& obs_y,
                                     double search_radius,
                                     double fallback_x, double fallback_y,
                                     double max_bin_area, int cost_threshold) {
	std::lock_guard<std::mutex> lock(costmap_mutex_);
	if (!has_costmap_ || costmap_resolution_ <= 0.0) {
		RCLCPP_WARN(node_->get_logger(),
			"findNearestObstacle: No costmap available. Using fallback (%.2f, %.2f).",
			fallback_x, fallback_y);
		obs_x = fallback_x;
		obs_y = fallback_y;
		return false;
	}

	int w = static_cast<int>(costmap_width_);
	int h = static_cast<int>(costmap_height_);
	int robot_gx = static_cast<int>((robot_x - costmap_origin_x_) / costmap_resolution_);
	int robot_gy = static_cast<int>((robot_y - costmap_origin_y_) / costmap_resolution_);
	int search_cells = static_cast<int>(search_radius / costmap_resolution_);

	// Bounds for the search window
	int x_min = std::max(0, robot_gx - search_cells);
	int x_max = std::min(w - 1, robot_gx + search_cells);
	int y_min = std::max(0, robot_gy - search_cells);
	int y_max = std::min(h - 1, robot_gy + search_cells);

	double search_r2 = search_radius * search_radius;

	// Mark which cells are lethal (cost == 100) and within search radius
	// Using a flat visited array for BFS
	int window_w = x_max - x_min + 1;
	int window_h = y_max - y_min + 1;
	std::vector<bool> lethal(window_w * window_h, false);
	std::vector<bool> visited(window_w * window_h, false);

	for (int gy = y_min; gy <= y_max; ++gy) {
		for (int gx = x_min; gx <= x_max; ++gx) {
			double wx = costmap_origin_x_ + (gx + 0.5) * costmap_resolution_;
			double wy = costmap_origin_y_ + (gy + 0.5) * costmap_resolution_;
			double dx = wx - robot_x;
			double dy = wy - robot_y;
			if (dx * dx + dy * dy > search_r2) continue;

			int idx = gy * w + gx;
			if (costmap_data_[idx] >= cost_threshold) {  // cost_threshold=80 includes inscribed+lethal
				int li = (gy - y_min) * window_w + (gx - x_min);
				lethal[li] = true;
			}
		}
	}

	// BFS flood-fill to identify connected clusters of lethal cells
	struct Cluster {
		double cx = 0.0, cy = 0.0;  // centroid in world coords
		int count = 0;
		double nearest_dist2 = 1e9; // distance² of nearest cell to robot
		double nearest_x = 0.0, nearest_y = 0.0;
	};
	std::vector<Cluster> clusters;
	const int dx4[4] = {1, -1, 0, 0};
	const int dy4[4] = {0, 0, 1, -1};

	for (int gy = y_min; gy <= y_max; ++gy) {
		for (int gx = x_min; gx <= x_max; ++gx) {
			int li = (gy - y_min) * window_w + (gx - x_min);
			if (!lethal[li] || visited[li]) continue;

			// BFS from this cell
			Cluster cluster;
			std::queue<std::pair<int, int>> q;
			q.push({gx, gy});
			visited[li] = true;

			while (!q.empty()) {
				auto [cx, cy] = q.front();
				q.pop();

				double wx = costmap_origin_x_ + (cx + 0.5) * costmap_resolution_;
				double wy = costmap_origin_y_ + (cy + 0.5) * costmap_resolution_;
				cluster.cx += wx;
				cluster.cy += wy;
				cluster.count++;

				double d2 = (wx - robot_x) * (wx - robot_x) + (wy - robot_y) * (wy - robot_y);
				if (d2 < cluster.nearest_dist2) {
					cluster.nearest_dist2 = d2;
					cluster.nearest_x = wx;
					cluster.nearest_y = wy;
				}

				for (int d = 0; d < 4; ++d) {
					int nx = cx + dx4[d];
					int ny = cy + dy4[d];
					if (nx < x_min || nx > x_max || ny < y_min || ny > y_max) continue;
					int nli = (ny - y_min) * window_w + (nx - x_min);
					if (!lethal[nli] || visited[nli]) continue;
					visited[nli] = true;
					q.push({nx, ny});
				}
			}

			if (cluster.count > 0) {
				cluster.cx /= cluster.count;
				cluster.cy /= cluster.count;
				clusters.push_back(cluster);
			}
		}
	}

	if (clusters.empty()) {
		RCLCPP_WARN(node_->get_logger(),
			"findNearestObstacle: No lethal cells within %.1fm. Using fallback (%.2f, %.2f).",
			search_radius, fallback_x, fallback_y);
		obs_x = fallback_x;
		obs_y = fallback_y;
		return false;
	}

	// Classify clusters by area: small = bin, large = wall
	double cell_area = costmap_resolution_ * costmap_resolution_;

	// Find nearest bin-sized cluster
	double best_dist2 = 1e9;
	int best_idx = -1;
	for (size_t i = 0; i < clusters.size(); ++i) {
		double area = clusters[i].count * cell_area;
		double d2 = (clusters[i].cx - robot_x) * (clusters[i].cx - robot_x) +
		            (clusters[i].cy - robot_y) * (clusters[i].cy - robot_y);
		if (area <= max_bin_area && d2 < best_dist2) {
			best_dist2 = d2;
			best_idx = static_cast<int>(i);
		}
	}

	if (best_idx >= 0) {
		obs_x = clusters[best_idx].cx;
		obs_y = clusters[best_idx].cy;
		double area = clusters[best_idx].count * cell_area;
		RCLCPP_INFO(node_->get_logger(),
			"findNearestObstacle: Bin cluster at (%.2f, %.2f), area=%.3fm², %d cells, dist=%.2fm.",
			obs_x, obs_y, area, clusters[best_idx].count, std::sqrt(best_dist2));
		return true;
	}

	// All clusters are wall-sized — use the nearest point of the nearest cluster
	best_dist2 = 1e9;
	best_idx = -1;
	for (size_t i = 0; i < clusters.size(); ++i) {
		if (clusters[i].nearest_dist2 < best_dist2) {
			best_dist2 = clusters[i].nearest_dist2;
			best_idx = static_cast<int>(i);
		}
	}

	if (best_idx >= 0) {
		// For a wall, orbit around the nearest point rather than the full centroid
		obs_x = clusters[best_idx].nearest_x;
		obs_y = clusters[best_idx].nearest_y;
		double area = clusters[best_idx].count * cell_area;
		RCLCPP_INFO(node_->get_logger(),
			"findNearestObstacle: Wall cluster (%.3fm², %d cells). Using nearest point (%.2f, %.2f).",
			area, clusters[best_idx].count, obs_x, obs_y);
		return true;
	}

	// Should not reach here but just in case
	obs_x = fallback_x;
	obs_y = fallback_y;
	return false;
}

// =============================================================================
// generateViewpoints — orbit points around an obstacle centroid, sorted by
//   proximity to the robot.  Each point faces the obstacle.
// =============================================================================
std::vector<Navigation::Viewpoint> Navigation::generateViewpoints(
		double obs_x, double obs_y,
		double robot_x, double robot_y,
		double orbit_radius, int num_candidates,
		double robot_radius, double fallback_radius_ratio) {

	std::vector<Viewpoint> viewpoints;
	viewpoints.reserve(num_candidates);

	for (int i = 0; i < num_candidates; ++i) {
		double theta = 2.0 * M_PI * i / num_candidates;
		double vx = obs_x + orbit_radius * std::cos(theta);
		double vy = obs_y + orbit_radius * std::sin(theta);
		// Yaw faces toward the obstacle centroid
		double yaw = std::atan2(obs_y - vy, obs_x - vx);

		if (isInFreeSpace(vx, vy, robot_radius)) {
			viewpoints.push_back({vx, vy, yaw});
		}
	}

	if (viewpoints.empty()) {
		RCLCPP_WARN(node_->get_logger(),
			"generateViewpoints: All %d candidates blocked! Trying with smaller radius...",
			num_candidates);
		// Retry with reduced radius as fallback
		double smaller_r = orbit_radius * fallback_radius_ratio;
		for (int i = 0; i < num_candidates; ++i) {
			double theta = 2.0 * M_PI * i / num_candidates;
			double vx = obs_x + smaller_r * std::cos(theta);
			double vy = obs_y + smaller_r * std::sin(theta);
			double yaw = std::atan2(obs_y - vy, obs_x - vx);
			if (isInFreeSpace(vx, vy, robot_radius)) {
				viewpoints.push_back({vx, vy, yaw});
			}
		}
	}

	// Sort by distance from robot (nearest first)
	std::sort(viewpoints.begin(), viewpoints.end(),
		[robot_x, robot_y](const Viewpoint& a, const Viewpoint& b) {
			double da = (a.x - robot_x) * (a.x - robot_x) + (a.y - robot_y) * (a.y - robot_y);
			double db = (b.x - robot_x) * (b.x - robot_x) + (b.y - robot_y) * (b.y - robot_y);
			return da < db;
		});

	RCLCPP_INFO(node_->get_logger(),
		"generateViewpoints: %zu valid viewpoints around (%.2f, %.2f) at r=%.2fm.",
		viewpoints.size(), obs_x, obs_y, orbit_radius);

	return viewpoints;
}
