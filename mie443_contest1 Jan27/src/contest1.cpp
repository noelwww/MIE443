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
    Contest1Node()
        : Node("contest1_node")
    {
        // 1. Initialize Publishers & Subscribers for velocity commands
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

        // 3. Reset Pose
        auto reset_client = this->create_client<irobot_create_msgs::srv::ResetPose>("/reset_pose");
        if (reset_client->wait_for_service(1s)) {
            auto request = std::make_shared<irobot_create_msgs::srv::ResetPose::Request>();
            reset_client->async_send_request(request);
            RCLCPP_INFO(this->get_logger(), "Requesting odometry reset...");
        }

        // 4. Timer for control loop
        timer_ = this->create_timer(
            10ms, std::bind(&Contest1Node::controlLoop, this));

        // 5. Initialize Variables
        start_time_ = this->now();
        bump_timer_ = this->now() - rclcpp::Duration(10, 0);
        
        scan_timer_ = this->now();
        scanning_in_progress_ = false;

        pos_x_ = 0.0; pos_y_ = 0.0; yaw_ = 0.0;
        map_received_ = false;
        is_planning_ = false;
        
        bumpers_["bump_front_left"] = false;
        bumpers_["bump_front_center"] = false;
        bumpers_["bump_front_right"] = false;
        bumpers_["bump_left"] = false;
        bumpers_["bump_right"] = false;
        
        RCLCPP_INFO(this->get_logger(), "Contest 1 Node: Static Scan Mode Enabled.");
    }

private:
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

    void hazardCallback(const irobot_create_msgs::msg::HazardDetectionVector::SharedPtr hazard_vector) {
        for (auto& [key, val] : bumpers_) val = false;
        for (const auto& detection : hazard_vector->detections) {
            if (detection.type == irobot_create_msgs::msg::HazardDetection::BUMP) {
                bumpers_[detection.header.frame_id] = true;
            }
        }
    }

    // --- Main Control Loop ---

    void controlLoop()
    {
        // 1. Get Robot current location
        try {
            geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform(
                "map", "base_link", tf2::TimePointZero);
            pos_x_ = t.transform.translation.x;
            pos_y_ = t.transform.translation.y;
            yaw_ = tf2::getYaw(t.transform.rotation);
        } catch (const tf2::TransformException & ex) {
            pos_x_ = raw_odom_x_;
            pos_y_ = raw_odom_y_;
            yaw_ = raw_odom_yaw_;
        }

        // 2. Time Check to shutdown if it exceeds the contest limit 
        if ((this->now() - start_time_).seconds() >= 480.0) {
            stopRobot();
            rclcpp::shutdown();
            return;
        }

        // 3. Smart Bumper Logic
        bool is_bumping = false;
        for (const auto& [key, val] : bumpers_) {
            if (val) is_bumping = true;
        }

        if (is_bumping) {
            double hazard_dist = 0.25;
            WorldPoint hazard_pt;
            hazard_pt.x = pos_x_ + hazard_dist * std::cos(yaw_);
            hazard_pt.y = pos_y_ + hazard_dist * std::sin(yaw_);
            virtual_obstacles_.push_back(hazard_pt);  //Record the location of the bump, store it as virtual obstacle so that it will not go there any more 
            
            bump_timer_ = this->now();
            path_.clear(); 
            is_planning_ = false;
            scanning_in_progress_ = false; 
            
            RCLCPP_WARN(this->get_logger(), "Bump detected. Recording obstacle.");
        }

        // --- BACKUP LOGIC (Short Blind Backup) ---
        if ((this->now() - bump_timer_).seconds() < 1.5) {
            publishVelocity(-0.15, 0.0); //Backup in the opposite direction for 1.5 seconds
            return;
        }
        // ----------------------------------------

        // 4. Wait for Map
        if (!map_received_) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for Map...");
            publishVelocity(0.0, 0.4); 
            return;
        }

        // 5. Exploration Pipeline
        if (!path_.empty()) {
            WorldPoint goal = path_.back();
            double dist_to_goal = std::hypot(goal.x - pos_x_, goal.y - pos_y_);
            
            // Goal Logic
            if (dist_to_goal < 0.10) { //A buffer here to make sure that the robot robot doesn't have to accurately aim for the exact destination coordinate
                
                if (!scanning_in_progress_) {
                    scanning_in_progress_ = true;
                    scan_timer_ = this->now();
                    RCLCPP_INFO(this->get_logger(), "Goal reached. Pausing to scan...");
                }

                if ((this->now() - scan_timer_).seconds() < 2.0) {
                    // UPDATED: Stand STILL (0,0) while LIDAR scans, to get completed date 
                    publishVelocity(0.0, 0.0);
                    return; 
                } else {
                    RCLCPP_INFO(this->get_logger(), "Scan complete. Finding new target.");
                    scanning_in_progress_ = false;
                    path_.clear(); 
                    publishVelocity(0.0, 0.0);
                }
            } else {
                scanning_in_progress_ = false; 
                executePurePursuit();
            }
        } 
        else {
            if (!is_planning_) {
                planToFrontier(); //Plan to the next frontier
            }
        }
    }

    // --- Frontier & Planning ---

    void planToFrontier() {
        is_planning_ = true;
        
        nav_msgs::msg::OccupancyGrid map_copy;
        {
            std::lock_guard<std::mutex> lock(map_mutex_);
            map_copy = current_map_;
        }

        // Inject Virtual Obstacles
        for (const auto& obs : virtual_obstacles_) {
            Point p = worldToGrid(obs.x, obs.y, map_copy);
            int width = map_copy.info.width;
            for(int dy=-1; dy<=1; dy++) {
                for(int dx=-1; dx<=1; dx++) {
                    int idx = (p.y + dy) * width + (p.x + dx);
                    if (idx >= 0 && idx < (int)map_copy.data.size()) map_copy.data[idx] = 100;
                }
            }
        }

        // A. Find Frontiers
        std::vector<Point> frontiers = findFrontiers(map_copy);
        if (frontiers.empty()) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "No frontiers found. Rotating."); //Marks the full exploration of the map 
            publishVelocity(0.0, 0.3); 
            is_planning_ = false;
            return;
        }

        // B. Cluster
        std::vector<std::vector<Point>> groups = clusterFrontiers(frontiers); //group the frontiers into several chunks of clusters
        
        // C. Find Path
        Point robot_grid = worldToGrid(pos_x_, pos_y_, map_copy); 
        bool found_path = false;

        std::sort(groups.begin(), groups.end(), [](const std::vector<Point>& a, const std::vector<Point>& b) {
            return a.size() > b.size(); //Select the largest cluster
        });

        int checks = 0;
        for (const auto& group : groups) {
            if (checks++ > 3) break; 
            if (group.size() < 3) continue; 

            Point centroid = getValidCentroid(group, map_copy);
            
            // Distance Filter: To filter out the centroid that is too close to the robot. This helps prevent the robot from roaming in the same region
            double dist_sq = std::pow(centroid.x - robot_grid.x, 2) + std::pow(centroid.y - robot_grid.y, 2);
            double min_dist_px = 0.25 / map_copy.info.resolution;
            
            if (dist_sq < min_dist_px * min_dist_px) continue;

            // Run A* for path planning
            std::vector<WorldPoint> new_path = runAStar(map_copy, robot_grid, centroid);
            if (!new_path.empty()) {
                path_ = new_path;
                found_path = true;
                break;
            }
        }

        if (!found_path) { //The path is to dangerous to reach. For example, the channel is too narrow
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Frontiers visible but unreachable.");
            publishVelocity(0.0, 0.3); 
        }

        is_planning_ = false;
    }

    // --- A* Components ---

    double getProximityPenalty(Point p, const nav_msgs::msg::OccupancyGrid& map) {
        
        double hard_radius = 3.0; // 3 pixels = 15cm
        double soft_radius = 10.0; // 10 pixels = 50cm.
        
        double min_dist = 100.0;
        int width = map.info.width;
        int check_limit = (int)soft_radius;
        
        for (int dy = -check_limit; dy <= check_limit; ++dy) {
            for (int dx = -check_limit; dx <= check_limit; ++dx) {
                if (dx*dx + dy*dy > check_limit*check_limit) continue;

                int nx = p.x + dx;
                int ny = p.y + dy;
                
                if (nx < 0 || nx >= width || ny < 0 || ny >= (int)map.info.height) continue;
                
                int idx = ny * width + nx;
                
                if (map.data[idx] > 50) {
                    double dist = std::hypot(dx, dy);
                    if (dist < min_dist) min_dist = dist;
                    if (min_dist < hard_radius) return -1.0; 
                }
            }
        }

        if (min_dist <= hard_radius) return -1.0;

        if (min_dist < soft_radius) {
            return std::pow(soft_radius - min_dist, 2) * 5.0; 
        }

        return 0.0;
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

            if (current->p == goal || expansions > 7000) { 
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
                
                if (map.data[next_idx] > 50 || map.data[next_idx] == -1) continue; 

                double prox_penalty = getProximityPenalty(next_p, map);
                
                if (prox_penalty < 0.0) continue; 

                double move_cost = 1.0;
                
                auto next_node = std::make_shared<PathNode>();
                next_node->p = next_p;
                next_node->g = current->g + move_cost + prox_penalty; 
                next_node->h = std::hypot(next_p.x - goal.x, next_p.y - goal.y);
                next_node->parent = current;
                open_set.push(next_node);
            }
        }
        return {}; 
    }

    std::vector<Point> findFrontiers(const nav_msgs::msg::OccupancyGrid& map) {
        std::vector<Point> frontiers;
        int width = map.info.width;
        int height = map.info.height;

        for (int y = 1; y < height - 1; ++y) {
            for (int x = 1; x < width - 1; ++x) {
                int i = y * width + x;
                if (map.data[i] == 0) { // Free
                    
                    bool lethal = false;
                    for (int dy = -1; dy <= 1; ++dy) {
                        for (int dx = -1; dx <= 1; ++dx) {
                            int ni = (y+dy) * width + (x+dx);
                            if (ni >= 0 && ni < width*height && map.data[ni] > 90) {
                                lethal = true; break;
                            }
                        }
                    }
                    if (lethal) continue;

                    int neighbors[] = {i+1, i-1, i+width, i-width};
                    for (int n : neighbors) {
                        if (map.data[n] == -1) { 
                            frontiers.push_back({x, y});
                            break;
                        }
                    }
                }
            }
        }
        return frontiers;
    }

    // --- Helpers ---

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
        for (const auto& p : group) { sum_x += p.x; sum_y += p.y; }
        Point avg = {static_cast<int>(sum_x / group.size()), static_cast<int>(sum_y / group.size())};
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

    // --- Control Logic ---

    void executePurePursuit() {
        if (path_.empty()) return;
        double lookahead_dist = 0.35;
        
        size_t closest_idx = 0;
        double min_dist = 1e9;
        for (size_t i = 0; i < path_.size(); ++i) {
            double d = std::hypot(path_[i].x - pos_x_, path_[i].y - pos_y_);
            if (d < min_dist) { min_dist = d; closest_idx = i; }
        }
        
        WorldPoint target = path_.back();
        for (size_t i = closest_idx; i < path_.size(); ++i) {
            double d = std::hypot(path_[i].x - pos_x_, path_[i].y - pos_y_);
            if (d > lookahead_dist) { target = path_[i]; break; }
        }
        
        double error_y = target.y - pos_y_;
        double error_x = target.x - pos_x_;
        double desired_heading = std::atan2(error_y, error_x);
        double heading_error = desired_heading - yaw_;
        while (heading_error > M_PI) heading_error -= 2*M_PI;
        while (heading_error < -M_PI) heading_error += 2*M_PI;

        double lin = 0.15;
        double ang = 0.6 * heading_error; 
        
        if (std::abs(heading_error) > 0.6) lin = 0.0;
        publishVelocity(lin, ang);
    }

    void publishVelocity(double x, double z) {
        geometry_msgs::msg::TwistStamped vel;
        vel.header.stamp = this->now();
        vel.twist.linear.x = std::clamp(x, -0.15, 0.15); 
        vel.twist.angular.z = std::clamp(z, -1.0, 1.0);
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

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    sensor_msgs::msg::LaserScan last_scan_;
    nav_msgs::msg::OccupancyGrid current_map_;
    std::vector<WorldPoint> path_;
    std::mutex map_mutex_;

    rclcpp::Time start_time_;
    rclcpp::Time bump_timer_;
    rclcpp::Time scan_timer_;
    bool scanning_in_progress_;
    
    double pos_x_, pos_y_, yaw_;
    double raw_odom_x_, raw_odom_y_, raw_odom_yaw_;
    
    bool map_received_;
    bool is_planning_;

    std::map<std::string, bool> bumpers_;
    std::vector<WorldPoint> virtual_obstacles_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Contest1Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}