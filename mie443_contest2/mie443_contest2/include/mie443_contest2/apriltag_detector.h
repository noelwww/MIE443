#pragma once

#include <memory>
#include <string>
#include <vector>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>


class AprilTagDetector {
public:
    AprilTagDetector(std::shared_ptr<rclcpp::Node> node,
                     const std::string& tag_frame_prefix = "tag",
                     const std::string& reference_frame = "base_link");

    bool isTagVisible(int tag_id, int timeout_ms = 100);

    std::optional<geometry_msgs::msg::Pose> getTagPose(int tag_id, int timeout_ms = 100);

    std::optional<geometry_msgs::msg::TransformStamped> getTagTransform(int tag_id, int timeout_ms = 100);

    std::vector<int> getVisibleTags(const std::vector<int>& candidate_ids, int timeout_ms = 50);

    void setReferenceFrame(const std::string& frame);

    std::string getReferenceFrame() const;

    void setTagFramePrefix(const std::string& prefix);

private:
    std::string getTagFrameName(int tag_id) const;

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::string tag_frame_prefix_;
    std::string reference_frame_;
};

