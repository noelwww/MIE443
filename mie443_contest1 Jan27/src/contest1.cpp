#include <chrono>
#include <memory>
#include <cmath>
#include <map>
#include <vector>
#include <algorithm>
#include <queue>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "irobot_create_msgs/msg/hazard_detection_vector.hpp"
#include "irobot_create_msgs/srv/reset_pose.hpp"
#include "tf2/utils.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;

// --- Structs for A* and Grid Logic ---
struct Point {
    int x, y; // Grid coordinates
    bool operator==(const Point& other) const { return x == other.x && y == other.y; }
    bool operator<(const Point& other) const { return x < other.x || (x == other.x && y < other.y); }
};

struct WorldPoint {
    double x, y; // World coordinates (meters)
};

// Renamed to PathNode to avoid conflict with rclcpp::Node
struct PathNode {
    Point p;
    double g, h;
    std::shared_ptr<PathNode> parent;
    double f() const { return g + h; }
};

struct NodeCompare {
    bool operator()(const std::shared_ptr<PathNode>& a, const std::shared_ptr<PathNode>& b) {
        return a->f() > b->f(); // Min-heap priority queue
    }
};

class Contest1Node : public rclcpp::Node
{
public:
    Contest1Node()
        : Node("contest1_node")
    {
        // 1. Initialize Publishers & Subscribers
        vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);

        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),
            std::bind(&Contest1Node::laserCallback, this, std::placeholders::_1));

        hazard_sub_ = this->create_subscription<irobot_create_msgs::msg::HazardDetectionVector>(
            "/hazard_detection", rclcpp::SensorDataQoS(),
            std::bind(&Contest1Node::hazardCallback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", rclcpp::SensorDataQoS(),
            std::bind(&Contest1Node::odomCallback, this, std::placeholders::_1));

        // Using Transient Local to ensure we get the map even if published before we started
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", rclcpp::QoS(1).transient_local(),
            std::bind(&Contest1Node::mapCallback, this, std::placeholders::_1));

        // 2. Initialize TF Listener (Critical for accurate Map coordinates)
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 3. Reset Pose Service (From your original code)
        auto reset_client = this->create_client<irobot_create_msgs::srv::ResetPose>("/reset_pose");
        // Don't block startup if service fails, just try async
        if (reset_client->wait_for_service(1s)) {
            auto request = std::make_shared<irobot_create_msgs::srv::ResetPose::Request>();
            reset_client->async_send_request(request);
            RCLCPP_INFO(this->get_logger(), "Sent request to reset odometry to zero.");
        }

        // 4. Timer (Updated for Fast Simulation)
        // Using create_timer (syncs with Sim Time) running at 100Hz
        timer_ = this->create_timer(
            10ms, std::bind(&Contest1Node::controlLoop, this));

        // 5. Initialize Variables
        start_time_ = this->now();
        // Initialize bump_timer_ with the correct clock source (past time)
        bump_timer_ = this->now() - rclcpp::Duration(10, 0);

        angular_ = 0.0;
        linear_ = 0.0;
        
        // Initialize bumpers map
        bumpers_["bump_front_left"] = false;
        bumpers_["bump_front_center"] = false;
        bumpers_["bump_front_right"] = false;
        bumpers_["bump_left"] = false;
        bumpers_["bump_right"] = false;

        pos_x_ = 0.0;
        pos_y_ = 0.0;
        yaw_ = 0.0;
        min_laser_dist_ = 10.0;
        map_received_ = false;
        is_planning_ = false;
        
        RCLCPP_INFO(this->get_logger(), "Contest 1 Frontier Exploration Node Initialized (100Hz Mode).");
    }

private:
    // --- Callbacks ---

    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        last_scan_ = *scan; // Store full scan for safety logic later
        
        double min_dist = 10.0;
        for (float r : scan->ranges) {
            if (std::isfinite(r) && r > scan->range_min && r < scan->range_max) {
                if (r < min_dist) min_dist = r;
            }
        }
        min_laser_dist_ = min_dist;
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
    {
        // Fallback: If TF fails, we use this raw odom
        raw_odom_x_ = odom->pose.pose.position.x;
        raw_odom_y_ = odom->pose.pose.position.y;
        raw_odom_yaw_ = tf2::getYaw(odom->pose.pose.orientation);
    }

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map) 
    { 
        std::lock_guard<std::mutex> lock(map_mutex_);
        current_map_ = *map; 
        map_received_ = true;
    }

    void hazardCallback(const irobot_create_msgs::msg::HazardDetectionVector::SharedPtr hazard_vector)
    {
        for (auto& [key, val] : bumpers_) val = false;
        for (const auto& detection : hazard_vector->detections) {
            if (detection.type == irobot_create_msgs::msg::HazardDetection::BUMP) {
                bumpers_[detection.header.frame_id] = true;
                bump_timer_ = this->now();
            }
        }
    }

    // --- Main Control Loop ---

    void controlLoop()
    {
        // 1. Update Robot Pose using TF (Map Frame)
        bool tf_success = false;
        try {
            geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform(
                "map", "base_link", tf2::TimePointZero);
            pos_x_ = t.transform.translation.x;
            pos_y_ = t.transform.translation.y;
            yaw_ = tf2::getYaw(t.transform.rotation);
            tf_success = true;
        } catch (const tf2::TransformException & ex) {
            // TF not ready yet, fallback to odom
            pos_x_ = raw_odom_x_;
            pos_y_ = raw_odom_y_;
            yaw_ = raw_odom_yaw_;
        }

        // 2. Time Limit Check (480s)
        double seconds_elapsed = (this->now() - start_time_).seconds();
        if (seconds_elapsed >= 480.0) {
            stopRobot();
            rclcpp::shutdown();
            return;
        }

        // 3. Hazard Recovery
        bool any_bumper_pressed = false;
        for (const auto& [key, val] : bumpers_) {
            if (val) any_bumper_pressed = true;
        }

        if (any_bumper_pressed || (this->now() - bump_timer_).seconds() < 2.0) {
            // Back up and turn
            publishVelocity(-0.2, 1.0); // Aggressive recovery
            path_.clear(); 
            is_planning_ = false;
            return;
        }

        // 4. Wait for Map
        if (!map_received_) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for Map...");
            publishVelocity(0.0, 0.4); 
            return;
        }

        // 5. Exploration Logic
        if (!path_.empty()) {
            WorldPoint goal = path_.back();
            double dist_to_goal = std::hypot(goal.x - pos_x_, goal.y - pos_y_);
            
            // Tight Goal Tolerance (0.15m) to force robot to drive all the way there
            if (dist_to_goal < 0.15) { 
                RCLCPP_INFO(this->get_logger(), "Goal reached. Rescanning.");
                path_.clear(); 
                publishVelocity(0.0, 0.0);
            } else {
                executePurePursuit();
            }
        } 
        else {
            if (!is_planning_) {
                planToFrontier();
            }
        }
    }

    // --- Frontier & Planning Algorithms ---

    void planToFrontier() {
        is_planning_ = true;
        
        nav_msgs::msg::OccupancyGrid map_copy;
        {
            std::lock_guard<std::mutex> lock(map_mutex_);
            map_copy = current_map_;
        }

        // A. Find Frontiers
        std::vector<Point> frontiers = findFrontiers(map_copy);
        if (frontiers.empty()) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "No frontiers found. Rotating.");
            publishVelocity(0.0, 0.5); 
            is_planning_ = false;
            return;
        }

        // B. Cluster Frontiers
        std::vector<std::vector<Point>> groups = clusterFrontiers(frontiers);
        
        // C. Find Path
        Point robot_grid = worldToGrid(pos_x_, pos_y_, map_copy);
        bool found_path = false;

        // Sort by size (big frontiers first)
        std::sort(groups.begin(), groups.end(), [](const std::vector<Point>& a, const std::vector<Point>& b) {
            return a.size() > b.size();
        });

        int checks = 0;
        for (const auto& group : groups) {
            if (checks++ > 5) break; // Check top 5 groups to find one suitable
            if (group.size() < 5) continue; // Noise filter

            // Get valid point in the group (avoids centroid-in-wall issue)
            Point centroid = getValidCentroid(group, map_copy);
            
            // Distance Filter: Ignore targets too close to the robot (< 0.5m)
            // This prevents the "stuck in local loop" behavior
            double dist_sq = std::pow(centroid.x - robot_grid.x, 2) + std::pow(centroid.y - robot_grid.y, 2);
            double min_dist_pixels = 0.5 / map_copy.info.resolution;
            
            if (dist_sq < min_dist_pixels * min_dist_pixels) {
                RCLCPP_INFO(this->get_logger(), "Target too close (skipping). Size: %zu", group.size());
                continue;
            }

            RCLCPP_INFO(this->get_logger(), "Checking group with size: %zu", group.size());
            
            std::vector<WorldPoint> new_path = runAStar(map_copy, robot_grid, centroid);
            if (!new_path.empty()) {
                path_ = new_path;
                found_path = true;
                break;
            }
        }

        if (!found_path) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "No path to frontiers.");
            publishVelocity(0.0, 0.5); // Spin faster
        } else {
            RCLCPP_INFO(this->get_logger(), "Path found with %zu points.", path_.size());
        }

        is_planning_ = false;
    }

    std::vector<Point> findFrontiers(const nav_msgs::msg::OccupancyGrid& map) {
        std::vector<Point> frontiers;
        int width = map.info.width;
        int height = map.info.height;

        for (int y = 1; y < height - 1; ++y) {
            for (int x = 1; x < width - 1; ++x) {
                int i = y * width + x;
                if (map.data[i] == 0) { // Free
                    
                    // Obstacle Padding: Don't mark as frontier if touching a wall
                    bool too_close_to_obstacle = false;
                    int check_radius = 2; // Check 2-cell radius
                    for (int dy = -check_radius; dy <= check_radius; ++dy) {
                        for (int dx = -check_radius; dx <= check_radius; ++dx) {
                            int ni = (y+dy) * width + (x+dx);
                            // >50 is obstacle
                            if (ni >= 0 && ni < width*height && map.data[ni] > 50) {
                                too_close_to_obstacle = true;
                                break;
                            }
                        }
                        if (too_close_to_obstacle) break;
                    }
                    if (too_close_to_obstacle) continue;

                    // Standard Frontier Check (Free adjacent to Unknown)
                    int neighbors[] = {i+1, i-1, i+width, i-width};
                    for (int n : neighbors) {
                        if (map.data[n] == -1) { // Unknown
                            frontiers.push_back({x, y});
                            break;
                        }
                    }
                }
            }
        }
        return frontiers;
    }

    std::vector<std::vector<Point>> clusterFrontiers(const std::vector<Point>& frontiers) {
        std::vector<std::vector<Point>> groups;
        std::map<Point, bool> visited;
        for (const auto& p : frontiers) visited[p] = false;

        for (const auto& p : frontiers) {
            if (visited[p]) continue;

            std::vector<Point> group;
            std::queue<Point> q;
            q.push(p);
            visited[p] = true;

            while(!q.empty()) {
                Point curr = q.front(); q.pop();
                group.push_back(curr);

                for (const auto& other : frontiers) {
                    if (!visited[other]) {
                        if (std::abs(curr.x - other.x) <= 1 && std::abs(curr.y - other.y) <= 1) {
                            visited[other] = true;
                            q.push(other);
                        }
                    }
                }
            }
            groups.push_back(group);
        }
        return groups;
    }

    Point getValidCentroid(const std::vector<Point>& group, const nav_msgs::msg::OccupancyGrid& map) {
        long sum_x = 0, sum_y = 0;
        for (const auto& p : group) {
            sum_x += p.x;
            sum_y += p.y;
        }
        
        Point avg;
        avg.x = static_cast<int>(sum_x / group.size());
        avg.y = static_cast<int>(sum_y / group.size());
        
        // Find the point in the group closest to the geometric average
        // This ensures the target is actually inside the white space
        double min_dist = 1e9;
        Point best_p = group[0];
        
        for (const auto& p : group) {
            double d = std::hypot(p.x - avg.x, p.y - avg.y);
            if (d < min_dist) {
                min_dist = d;
                best_p = p;
            }
        }
        return best_p;
    }

    std::vector<WorldPoint> runAStar(const nav_msgs::msg::OccupancyGrid& map, Point start, Point goal) {
        std::priority_queue<std::shared_ptr<PathNode>, std::vector<std::shared_ptr<PathNode>>, NodeCompare> open_set;
        std::vector<bool> closed_set(map.info.width * map.info.height, false);

        auto start_node = std::make_shared<PathNode>();
        start_node->p = start;
        start_node->g = 0;
        start_node->h = std::hypot(start.x - goal.x, start.y - goal.y);
        open_set.push(start_node);

        int expansions = 0;
        while (!open_set.empty()) {
            auto current = open_set.top();
            open_set.pop();

            if (current->p == goal || expansions > 4000) {
                return reconstructPath(current, map);
            }

            int idx = current->p.y * map.info.width + current->p.x;
            if (idx < 0 || idx >= (int)closed_set.size() || closed_set[idx]) continue;
            closed_set[idx] = true;
            expansions++;

            int dx[] = {0, 0, 1, -1};
            int dy[] = {1, -1, 0, 0};

            for (int i = 0; i < 4; ++i) {
                Point next_p = {current->p.x + dx[i], current->p.y + dy[i]};
                
                if (next_p.x < 0 || next_p.x >= (int)map.info.width || next_p.y < 0 || next_p.y >= (int)map.info.height) continue;

                int next_idx = next_p.y * map.info.width + next_p.x;
                // Treat Unknown (-1) as obstacle for A* planning
                if (map.data[next_idx] > 50 || map.data[next_idx] == -1) continue; 

                auto next_node = std::make_shared<PathNode>();
                next_node->p = next_p;
                next_node->g = current->g + 1.0;
                next_node->h = std::hypot(next_p.x - goal.x, next_p.y - goal.y);
                next_node->parent = current;
                open_set.push(next_node);
            }
        }
        return {}; 
    }

    std::vector<WorldPoint> reconstructPath(std::shared_ptr<PathNode> node, const nav_msgs::msg::OccupancyGrid& map) {
        std::vector<WorldPoint> path;
        while (node) {
            path.push_back(gridToWorld(node->p, map));
            node = node->parent;
        }
        std::reverse(path.begin(), path.end());
        return path;
    }

    // --- Control Logic (Pure Pursuit) ---

    void executePurePursuit() {
        if (path_.empty()) return;

        double lookahead_dist = 0.5;
        
        // 1. Find the point on the path closest to the robot
        size_t closest_idx = 0;
        double min_dist_to_path = 1e9;
        
        for (size_t i = 0; i < path_.size(); ++i) {
            double d = std::hypot(path_[i].x - pos_x_, path_[i].y - pos_y_);
            if (d < min_dist_to_path) {
                min_dist_to_path = d;
                closest_idx = i;
            }
        }
        
        // 2. Look ahead from the closest point
        WorldPoint target = path_.back();
        for (size_t i = closest_idx; i < path_.size(); ++i) {
            double d = std::hypot(path_[i].x - pos_x_, path_[i].y - pos_y_);
            if (d > lookahead_dist) {
                target = path_[i];
                break;
            }
        }

        // 3. Calculate Heading Error
        double error_y = target.y - pos_y_;
        double error_x = target.x - pos_x_;
        double desired_heading = std::atan2(error_y, error_x);
        double heading_error = desired_heading - yaw_;

        while (heading_error > M_PI) heading_error -= 2*M_PI;
        while (heading_error < -M_PI) heading_error += 2*M_PI;

        // Speed Boost (0.5 m/s)
        double lin = 0.5; 
        double ang = 2.0 * heading_error; 

        // Removed safety slowdown for simulation as requested
        
        // Turn in place if heading error is large
        if (std::abs(heading_error) > 0.6) {
            lin = 0.0;
        }

        publishVelocity(lin, ang);
    }

    void publishVelocity(double x, double z) {
        geometry_msgs::msg::TwistStamped vel;
        vel.header.stamp = this->now();
        vel.twist.linear.x = std::clamp(x, -0.2, 0.5); // Max speed 0.5
        vel.twist.angular.z = std::clamp(z, -2.0, 2.0);
        vel_pub_->publish(vel);
    }

    void stopRobot() { publishVelocity(0, 0); }

    Point worldToGrid(double x, double y, const nav_msgs::msg::OccupancyGrid& map) {
        Point p;
        p.x = static_cast<int>((x - map.info.origin.position.x) / map.info.resolution);
        p.y = static_cast<int>((y - map.info.origin.position.y) / map.info.resolution);
        p.x = std::clamp(p.x, 0, (int)map.info.width - 1);
        p.y = std::clamp(p.y, 0, (int)map.info.height - 1);
        return p;
    }

    WorldPoint gridToWorld(Point p, const nav_msgs::msg::OccupancyGrid& map) {
        WorldPoint wp;
        wp.x = p.x * map.info.resolution + map.info.origin.position.x;
        wp.y = p.y * map.info.resolution + map.info.origin.position.y;
        return wp;
    }

    // Members
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<irobot_create_msgs::msg::HazardDetectionVector>::SharedPtr hazard_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // TF Listener
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    sensor_msgs::msg::LaserScan last_scan_;
    nav_msgs::msg::OccupancyGrid current_map_;
    std::vector<WorldPoint> path_;
    std::mutex map_mutex_;

    rclcpp::Time start_time_;
    rclcpp::Time bump_timer_;
    
    double pos_x_, pos_y_, yaw_;
    double raw_odom_x_, raw_odom_y_, raw_odom_yaw_;
    double min_laser_dist_; 
    
    float angular_, linear_;
    bool map_received_;
    bool is_planning_;

    std::map<std::string, bool> bumpers_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Contest1Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}