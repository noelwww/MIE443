#include <chrono>
#include <memory>
#include <cmath>
#include <map>
#include <vector>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "irobot_create_msgs/msg/hazard_detection_vector.hpp"
#include "tf2/utils.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

inline double rad2deg(double rad) { return rad * 180.0 / M_PI; }
inline double deg2rad(double deg) { return deg * M_PI / 180.0; }

class Contest1Node : public rclcpp::Node
{
public:
    Contest1Node()
        : Node("contest1_node")
    {
        // Initialize publisher for velocity commands
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

        // Timer for main control loop at 10 Hz
        timer_ = this->create_wall_timer(
            100ms, std::bind(&Contest1Node::controlLoop, this));

        // Initialize variables
        start_time_ = this->now();
        angular_ = 0.0;
        linear_ = 0.0;
        bumpers_["bump_front_left"] = false;
        bumpers_["bump_front_center"] = false;
        bumpers_["bump_front_right"] = false;
        bumpers_["bump_left"] = false;
        bumpers_["bump_right"] = false;
        pos_x_ = 0.0;
        pos_y_ = 0.0;
        yaw_ = 0.0;
        min_laser_dist_ = 10.0;

        RCLCPP_INFO(this->get_logger(), "Contest 1 node initialized. Running for 480 seconds.");
    }

private:
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        double min_dist = 10.0;
        // Check the front 30 degrees (approx indices 0-15 and the last 15)
        // Adjust logic if your lidar has different indexing
        size_t range = 15;
        
        // Check first 15 degrees
        for (size_t i = 0; i < range && i < scan->ranges.size(); i++) {
            if (scan->ranges[i] < min_dist && scan->ranges[i] > scan->range_min) {
                min_dist = scan->ranges[i];
            }
        }
        
        // Check last 15 degrees (wrapping around)
        for (size_t i = scan->ranges.size() - range; i < scan->ranges.size(); i++) {
            if (scan->ranges[i] < min_dist && scan->ranges[i] > scan->range_min) {
                min_dist = scan->ranges[i];
            }
        }
        min_laser_dist_ = min_dist;
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
    {
        // implement your code here
        // Extract position
        pos_x_ = odom->pose.pose.position.x;
        pos_y_ = odom->pose.pose.position.y;

        // Extract yaw from quaternion using tf2
        yaw_ = tf2::getYaw(odom->pose.pose.orientation);

        RCLCPP_INFO(this->get_logger(), "Position: (%.2f, %.2f), Orientation: %f rad or %f deg", pos_x_, pos_y_, yaw_, rad2deg(yaw_));
    }

    void hazardCallback(const irobot_create_msgs::msg::HazardDetectionVector::SharedPtr hazard_vector)
    {
        // Reset all bumpers to released (false) state first
        for (auto& [key, val] : bumpers_) {
            val = false;
        }

        // Update bumper states based on current detections
        for (const auto& detection : hazard_vector->detections) {
            // HazardDetection types include: BUMP, CLIFF, STALL, etc.
            // Type 1 (BUMP) is what we are looking for here
            if (detection.type == irobot_create_msgs::msg::HazardDetection::BUMP) {
                bumpers_[detection.header.frame_id] = true;
                RCLCPP_INFO(this->get_logger(), "Bumper pressed: %s", 
                            detection.header.frame_id.c_str());
            }
        }
    }

    void controlLoop()
    {
        // 1. Check Time Limit
        auto current_time = this->now();
        if ((current_time - start_time_).seconds() >= 480.0) {
            RCLCPP_INFO(this->get_logger(), "Time limit reached.");
                        
            // Manually stop the robot
            geometry_msgs::msg::TwistStamped vel;
            vel.header.stamp = this->now();
            vel.twist.linear.x = 0.0;
            vel.twist.angular.z = 0.0;
            vel_pub_->publish(vel);
            
            rclcpp::shutdown();
            return;
        }

        // 2. Update Sensors
        bool any_bumper_pressed = false;
        for (const auto& [key, val] : bumpers_) {
            if (val) { any_bumper_pressed = true; break; }
        }
        bool obstacle_ahead = (min_laser_dist_ < 0.5);

        // 3. PRIORITY CHECK: Emergency Stop
        // If bumper hit OR obstacle seen, we STOP regardless of state
        if (any_bumper_pressed || obstacle_ahead) 
        {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                "EMERGENCY STOP! Laser: %.2f", min_laser_dist_);
            linear_ = 0.0;
            angular_ = 0.0;
            
            // Optional: Change state to 'Stop' so it doesn't try to move again
            // state_ = 2; 
        }
        // 4. NORMAL NAVIGATION (State Machine)
        else 
        {
            // STATE 0: Move Forward
            if (state_ == 0) 
            {
                if (pos_x_ < 0.5) {
                    linear_ = 0.1;
                    angular_ = 0.0;
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Moving Forward... x=%.2f", pos_x_);
                } else {
                    // Goal reached! Switch to Turn State
                    state_ = 1;
                    linear_ = 0.0;
                    angular_ = 0.0;
                    RCLCPP_INFO(this->get_logger(), "0.5m Reached! Switching to Turn.");
                }
            }
            // STATE 1: Turn Right (-90 Degrees)
            else if (state_ == 1) 
            {
                // Turn until Yaw is roughly -1.57 radians (-90 deg)
                if (yaw_ > -1.50) {
                    linear_ = 0.0;
                    angular_ = -0.3; // Negative = Right Turn
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Turning Right... Yaw=%.2f", yaw_);
                } else {
                    // Turn complete! Switch to Finished State
                    state_ = 2;
                    linear_ = 0.0;
                    angular_ = 0.0;
                    RCLCPP_INFO(this->get_logger(), "Turn Complete! Stopping.");
                }
            }
            // STATE 2: Finished
            else if (state_ == 2) 
            {
                linear_ = 0.0;
                angular_ = 0.0;
                // Uncomment below if you want to kill the node immediately
                // rclcpp::shutdown(); 
            }
    }

    // 5. Publish Command
    geometry_msgs::msg::TwistStamped vel;
    vel.header.stamp = this->now();
    vel.twist.linear.x = linear_;
    vel.twist.angular.z = angular_;
    vel_pub_->publish(vel);
    }

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<irobot_create_msgs::msg::HazardDetectionVector>::SharedPtr hazard_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Time start_time_;
    float angular_;
    float linear_;
    double pos_x_;
    double pos_y_;
    double yaw_;
    double min_laser_dist_; // Minimum distance to an obstacle in front
    int state_ = 0; // 0 = Forward, 1 = Turn, 2 = Stop
    std::map<std::string, bool> bumpers_; // Map to store bumper states
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Contest1Node>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
