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

// --- Structs ---
struct Point {
    int x, y;
    bool operator==(const Point& other) const { return x == other.x && y == other.y; }
    bool operator<(const Point& other) const { return x < other.x || (x == other.x && y < other.y); }
};

struct WorldPoint {
    double x, y;
};

struct PathNode {
    Point p;
    double g, h;
    std::shared_ptr<PathNode> parent;
    double f() const { return g + h; }
};

struct NodeCompare {
    bool operator()(const std::shared_ptr<PathNode>& a, const std::shared_ptr<PathNode>& b) {
        return a->f() > b->f();
    }
};

class Contest1Node : public rclcpp::Node
{
public:
    Contest1Node() : Node("contest1_node")
    {
        // 1. Pub/Sub
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
            
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", rclcpp::QoS(1).transient_local(),
            std::bind(&Contest1Node::mapCallback, this, std::placeholders::_1));

        // 2. TF Listener
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 3. Reset Pose Service
        auto reset_client = this->create_client<irobot_create_msgs::srv::ResetPose>("/reset_pose");
        if (reset_client->wait_for_service(1s)) {
            auto request = std::make_shared<irobot_create_msgs::srv::ResetPose::Request>();
            reset_client->async_send_request(request);
            RCLCPP_INFO(this->get_logger(), "Sent request to reset odometry.");
        }

        // 4. Timer & Variables
        timer_ = this->create_timer(10ms, std::bind(&Contest1Node::controlLoop, this));
        start_time_ = this->now();
        bump_timer_ = this->now() - rclcpp::Duration(10, 0);
        last_plan_time_ = this->now(); 

        pos_x_ = 0.0; pos_y_ = 0.0; yaw_ = 0.0;
        map_received_ = false; is_planning_ = false;

        RCLCPP_INFO(this->get_logger(), "Contest 1 Node Started (Variable Speed Version).");
    }

private:
    // --- Constants for Robot Physical Size ---
    const double ROBOT_RADIUS_METERS = 0.16; // ~16cm radius (Turtlebot4)
    const double INFLATION_RADIUS = 0.20;    // 20cm Safety bubble for hidden obstacles

    // --- Data Structures ---
    std::vector<WorldPoint> hidden_obstacles_; // Stores "Bumper Map" hits

    // --- Callbacks ---
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
        last_scan_ = *scan;
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom) {
        raw_odom_x_ = odom->pose.pose.position.x;
        raw_odom_y_ = odom->pose.pose.position.y;
        raw_odom_yaw_ = tf2::getYaw(odom->pose.pose.orientation);
    }

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map) { 
        std::lock_guard<std::mutex> lock(map_mutex_);
        current_map_ = *map; 
        map_received_ = true;
    }

    void hazardCallback(const irobot_create_msgs::msg::HazardDetectionVector::SharedPtr hazard_vector)
    {
        for (const auto& detection : hazard_vector->detections) {
            if (detection.type == irobot_create_msgs::msg::HazardDetection::BUMP) {
                bumpers_[detection.header.frame_id] = true;
                bump_timer_ = this->now();

                // --- BUMPER MAPPER LOGIC ---
                // Store the world location of the object we just hit
                double angle_offset = 0.0;
                if (detection.header.frame_id.find("left") != std::string::npos) angle_offset = 0.5; // ~30 deg
                if (detection.header.frame_id.find("right") != std::string::npos) angle_offset = -0.5;
                
                WorldPoint impact_pt;
                // Calculate impact point: RobotPos + (Radius + 5cm) in direction of bump
                impact_pt.x = pos_x_ + (ROBOT_RADIUS_METERS + 0.05) * std::cos(yaw_ + angle_offset);
                impact_pt.y = pos_y_ + (ROBOT_RADIUS_METERS + 0.05) * std::sin(yaw_ + angle_offset);
                
                hidden_obstacles_.push_back(impact_pt);
                RCLCPP_WARN(this->get_logger(), "Bump! Hidden Obstacle marked at: %.2f, %.2f", impact_pt.x, impact_pt.y);
            }
        }
    }

    // --- Main Loop ---
    void controlLoop()
    {
        // Update Pose (TF with Odom Fallback)
        try {
            geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
            pos_x_ = t.transform.translation.x;
            pos_y_ = t.transform.translation.y;
            yaw_ = tf2::getYaw(t.transform.rotation);
        } catch (const tf2::TransformException & ex) {
            pos_x_ = raw_odom_x_; pos_y_ = raw_odom_y_; yaw_ = raw_odom_yaw_;
        }

        // Time Limit
        if ((this->now() - start_time_).seconds() >= 480.0) {
            stopRobot(); rclcpp::shutdown(); return;
        }

        // Recovery Mode (Back up)
        if ((this->now() - bump_timer_).seconds() < 2.0) {
            publishVelocity(-0.15, 0.0); // Reduced backup speed for safety
            path_.clear(); is_planning_ = false;
            return;
        }

        if (!map_received_) {
            publishVelocity(0.0, 0.4); return;
        }

        // --- PERIODIC RE-PLANNING ---
        // Force a new plan every 3 seconds to account for map updates or drift
        if (!path_.empty() && (this->now() - last_plan_time_).seconds() > 3.0) {
             path_.clear(); 
        }

        if (!path_.empty()) {
            WorldPoint goal = path_.back();
            // Tight tolerance to ensure we reach the frontier
            if (std::hypot(goal.x - pos_x_, goal.y - pos_y_) < 0.15) { 
                RCLCPP_INFO(this->get_logger(), "Goal reached. Rescanning.");
                path_.clear(); 
                publishVelocity(0.0, 0.0);
            } else {
                executePurePursuit();
            }
        } else {
            if (!is_planning_) planToFrontier();
        }
    }

    // --- Planning ---
    void planToFrontier() {
        is_planning_ = true;
        
        nav_msgs::msg::OccupancyGrid map_copy;
        {
            std::lock_guard<std::mutex> lock(map_mutex_);
            map_copy = current_map_;
        }

        // --- INJECT BUMPER HITS INTO MAP ---
        int inflation_cells = INFLATION_RADIUS / map_copy.info.resolution;
        for (const auto& obs : hidden_obstacles_) {
            Point center = worldToGrid(obs.x, obs.y, map_copy);
            // Draw a square wall around the hit point
            for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
                for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
                    int nx = center.x + dx; int ny = center.y + dy;
                    if (nx >= 0 && nx < (int)map_copy.info.width && ny >= 0 && ny < (int)map_copy.info.height) {
                        map_copy.data[ny * map_copy.info.width + nx] = 100; // Mark as WALL
                    }
                }
            }
        }

        std::vector<Point> frontiers = findFrontiers(map_copy);
        if (frontiers.empty()) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Map Complete. Spinning to verify.");
            publishVelocity(0.0, 0.5); 
            is_planning_ = false;
            return;
        }

        std::vector<std::vector<Point>> groups = clusterFrontiers(frontiers);
        
        // Sort by size
        std::sort(groups.begin(), groups.end(), [](const std::vector<Point>& a, const std::vector<Point>& b) {
            return a.size() > b.size();
        });

        Point robot_grid = worldToGrid(pos_x_, pos_y_, map_copy);
        bool found_path = false;

        // --- TWO-PASS TARGET SELECTION ---
        // Pass 0: Strict (ignore close targets). Pass 1: Desperate (allow them).
        for (int pass = 0; pass < 2; ++pass) {
            if (found_path) break;

            for (const auto& group : groups) {
                if (group.size() < 5) continue; // Noise filter

                Point centroid = getValidCentroid(group, map_copy);
                
                double dist_sq = std::pow(centroid.x - robot_grid.x, 2) + std::pow(centroid.y - robot_grid.y, 2);
                double min_dist_pixels = 0.5 / map_copy.info.resolution;
                
                if (pass == 0 && dist_sq < min_dist_pixels * min_dist_pixels) continue;

                std::vector<WorldPoint> new_path = runAStar(map_copy, robot_grid, centroid);
                if (!new_path.empty()) {
                    path_ = new_path;
                    found_path = true;
                    last_plan_time_ = this->now();
                    break;
                }
            }
        }

        if (!found_path) {
            RCLCPP_WARN(this->get_logger(), "Frontiers exist but are unreachable. Spinning.");
            publishVelocity(0.0, 0.5);
        }
        is_planning_ = false;
    }

    std::vector<WorldPoint> runAStar(const nav_msgs::msg::OccupancyGrid& map, Point start, Point goal) {
        std::priority_queue<std::shared_ptr<PathNode>, std::vector<std::shared_ptr<PathNode>>, NodeCompare> open_set;
        std::vector<bool> closed_set(map.info.width * map.info.height, false);

        auto start_node = std::make_shared<PathNode>();
        start_node->p = start; start_node->g = 0; start_node->h = std::hypot(start.x - goal.x, start.y - goal.y);
        open_set.push(start_node);

        int expansions = 0;
        int robot_radius_cells = std::ceil(ROBOT_RADIUS_METERS / map.info.resolution);

        while (!open_set.empty()) {
            auto current = open_set.top(); open_set.pop();

            if (current->p == goal || expansions > 5000) return reconstructPath(current, map);

            int idx = current->p.y * map.info.width + current->p.x;
            if (closed_set[idx]) continue;
            closed_set[idx] = true;
            expansions++;

            int dx[] = {0, 0, 1, -1}; int dy[] = {1, -1, 0, 0};

            for (int i = 0; i < 4; ++i) {
                Point next_p = {current->p.x + dx[i], current->p.y + dy[i]};
                
                // --- COLLISION CHECK WITH ROBOT RADIUS ---
                // Instead of checking 1 pixel, check if the ROBOT fits here
                if (isColliding(next_p, map, robot_radius_cells)) continue; 

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

    bool isColliding(Point p, const nav_msgs::msg::OccupancyGrid& map, int radius) {
        if (p.x < 0 || p.x >= (int)map.info.width || p.y < 0 || p.y >= (int)map.info.height) return true;
        for (int dy = -radius; dy <= radius; ++dy) {
            for (int dx = -radius; dx <= radius; ++dx) {
                int nx = p.x + dx; int ny = p.y + dy;
                if (nx < 0 || nx >= (int)map.info.width || ny < 0 || ny >= (int)map.info.height) return true; 
                int idx = ny * map.info.width + nx;
                if (map.data[idx] > 50 || map.data[idx] == -1) return true;
            }
        }
        return false;
    }
    
    // --- Unchanged Helper Functions ---
    std::vector<Point> findFrontiers(const nav_msgs::msg::OccupancyGrid& map) {
         std::vector<Point> frontiers;
         int width = map.info.width;
         int height = map.info.height;
         for (int y = 1; y < height - 1; ++y) {
             for (int x = 1; x < width - 1; ++x) {
                 int i = y * width + x;
                 if (map.data[i] == 0) { 
                     bool too_close = false;
                     // Safety padding to avoid marking walls as frontiers
                     for (int dy = -2; dy <= 2; ++dy) {
                        for (int dx = -2; dx <= 2; ++dx) {
                             int ni = (y+dy)*width + (x+dx);
                             if (ni >= 0 && ni < width*height && map.data[ni] > 50) { too_close=true; break; }
                        }
                     }
                     if (too_close) continue;
                     int neighbors[] = {i+1, i-1, i+width, i-width};
                     for (int n : neighbors) {
                         if (map.data[n] == -1) { frontiers.push_back({x, y}); break; }
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
            q.push(p); visited[p] = true;
            while(!q.empty()) {
                Point curr = q.front(); q.pop();
                group.push_back(curr);
                for (const auto& other : frontiers) {
                    if (!visited[other] && std::abs(curr.x - other.x) <= 1 && std::abs(curr.y - other.y) <= 1) {
                        visited[other] = true; q.push(other);
                    }
                }
            }
            groups.push_back(group);
        }
        return groups;
    }

    Point getValidCentroid(const std::vector<Point>& group, const nav_msgs::msg::OccupancyGrid& map) {
         long sum_x = 0, sum_y = 0;
         for (const auto& p : group) { sum_x += p.x; sum_y += p.y; }
         Point avg = {static_cast<int>(sum_x/group.size()), static_cast<int>(sum_y/group.size())};
         double min_dist = 1e9;
         Point best_p = group[0];
         for (const auto& p : group) {
             double d = std::hypot(p.x - avg.x, p.y - avg.y);
             if (d < min_dist) { min_dist = d; best_p = p; }
         }
         return best_p;
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
    
    void executePurePursuit() {
        if (path_.empty()) return;
        double lookahead = 0.5;
        size_t closest_idx = 0; double min_dist = 1e9;
        for(size_t i=0; i<path_.size(); ++i) {
            double d = std::hypot(path_[i].x - pos_x_, path_[i].y - pos_y_);
            if(d < min_dist) { min_dist=d; closest_idx=i; }
        }
        WorldPoint target = path_.back();
        for(size_t i=closest_idx; i<path_.size(); ++i) {
            if(std::hypot(path_[i].x - pos_x_, path_[i].y - pos_y_) > lookahead) {
                target = path_[i]; break;
            }
        }
        double err_y = target.y - pos_y_;
        double err_x = target.x - pos_x_;
        double head_err = std::atan2(err_y, err_x) - yaw_;
        while(head_err > M_PI) head_err -= 2*M_PI;
        while(head_err < -M_PI) head_err += 2*M_PI;
        
        // --- NEW: DYNAMIC SPEED SCALING ---
        double speed_limit = 0.25; // Default Max Speed

        // 1. Find nearest obstacle distance from LiDAR
        double min_lidar_dist = 10.0;
        if (!last_scan_.ranges.empty()) {
            for (float r : last_scan_.ranges) {
                // Check if valid range and ignore noise (< 0.05)
                if (std::isfinite(r) && r > 0.05 && r < min_lidar_dist) {
                    min_lidar_dist = r;
                }
            }
        }

        // 2. Scale Speed based on Distance
        if (min_lidar_dist < 0.3) {
            speed_limit = 0.1; // Danger Zone: Creep mode
        } else if (min_lidar_dist < 0.6) {
            // Caution Zone: Linearly scale from 0.1 to 0.25
            double ratio = (min_lidar_dist - 0.3) / 0.3;
            speed_limit = 0.1 + (ratio * 0.15); 
        }
        
        double lin = speed_limit;
        double ang = 2.0 * head_err;
        if(std::abs(head_err) > 0.6) lin = 0.0;
        publishVelocity(lin, ang);
    }
    
    void publishVelocity(double x, double z) {
        geometry_msgs::msg::TwistStamped vel;
        vel.header.stamp = this->now();
        // Clamping to max speed to be safe
        vel.twist.linear.x = std::clamp(x, -0.25, 0.25);
        vel.twist.angular.z = std::clamp(z, -2.0, 2.0);
        vel_pub_->publish(vel);
    }
    
    void stopRobot() { publishVelocity(0,0); }
    
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

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    sensor_msgs::msg::LaserScan last_scan_;
    nav_msgs::msg::OccupancyGrid current_map_;
    std::vector<WorldPoint> path_;
    std::mutex map_mutex_;

    rclcpp::Time start_time_;
    rclcpp::Time bump_timer_;
    rclcpp::Time last_plan_time_; 

    double pos_x_, pos_y_, yaw_;
    double raw_odom_x_, raw_odom_y_, raw_odom_yaw_;
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