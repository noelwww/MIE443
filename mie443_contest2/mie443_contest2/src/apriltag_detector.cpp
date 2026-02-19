#include "mie443_contest2/apriltag_detector.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

AprilTagDetector::AprilTagDetector(std::shared_ptr<rclcpp::Node> node,
                                   const std::string& tag_frame_prefix,
                                   const std::string& reference_frame)
    : node_(node)
    , tag_frame_prefix_(tag_frame_prefix)
    , reference_frame_(reference_frame)
{
    // Create TF2 buffer and listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(node_->get_logger(), 
                "AprilTagDetector initialized (prefix: '%s', reference: '%s')",
                tag_frame_prefix_.c_str(), reference_frame_.c_str());
}

std::string AprilTagDetector::getTagFrameName(int tag_id) const {
    return tag_frame_prefix_ + std::to_string(tag_id);
}

bool AprilTagDetector::isTagVisible(int tag_id, int timeout_ms) {
    (void)timeout_ms;  // Not used - we check staleness instead
    std::string tag_frame = getTagFrameName(tag_id);
    
    try {
        // Get latest cached transform (non-blocking)
        auto transform = tf_buffer_->lookupTransform(
            reference_frame_,
            tag_frame,
            tf2::TimePointZero
        );
        
        // Check if transform is recent (within 500ms)
        auto transform_age = node_->get_clock()->now() - transform.header.stamp;
        if (transform_age.seconds() > 0.5) {
            return false;  // Transform is stale - tag no longer visible
        }
        
        return true;
    } catch (const tf2::TransformException& ex) {
        // Transform not available - tag never seen
        return false;
    }
}

std::optional<geometry_msgs::msg::Pose> AprilTagDetector::getTagPose(int tag_id, int timeout_ms) {
    auto transform_opt = getTagTransform(tag_id, timeout_ms);
    
    if (!transform_opt.has_value()) {
        return std::nullopt;
    }
    
    // Convert TransformStamped to Pose
    geometry_msgs::msg::Pose pose;
    pose.position.x = transform_opt->transform.translation.x;
    pose.position.y = transform_opt->transform.translation.y;
    pose.position.z = transform_opt->transform.translation.z;
    pose.orientation = transform_opt->transform.rotation;
    
    return pose;
}

std::optional<geometry_msgs::msg::TransformStamped> AprilTagDetector::getTagTransform(int tag_id, int timeout_ms) {
    (void)timeout_ms;  // Not used - we check staleness instead
    std::string tag_frame = getTagFrameName(tag_id);
    
    try {
        // Get latest cached transform (non-blocking)
        geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
            reference_frame_,
            tag_frame,
            tf2::TimePointZero
        );
        
        // Check if transform is recent (within 500ms)
        auto transform_age = node_->get_clock()->now() - transform.header.stamp;
        if (transform_age.seconds() > 0.5) {
            RCLCPP_DEBUG(node_->get_logger(), 
                         "Transform for tag %d is stale (%.2f seconds old)",
                         tag_id, transform_age.seconds());
            return std::nullopt;  // Transform is stale - tag no longer visible
        }
        
        return transform;
    } catch (const tf2::TransformException& ex) {
        RCLCPP_DEBUG(node_->get_logger(), 
                     "Could not get transform for tag %d (%s -> %s): %s",
                     tag_id, reference_frame_.c_str(), tag_frame.c_str(), ex.what());
        return std::nullopt;
    }
}

std::vector<int> AprilTagDetector::getVisibleTags(const std::vector<int>& candidate_ids, int timeout_ms) {
    std::vector<int> visible_tags;
    
    for (int tag_id : candidate_ids) {
        if (isTagVisible(tag_id, timeout_ms)) {
            visible_tags.push_back(tag_id);
        }
    }
    
    return visible_tags;
}

void AprilTagDetector::setReferenceFrame(const std::string& frame) {
    reference_frame_ = frame;
    RCLCPP_INFO(node_->get_logger(), "AprilTagDetector reference frame set to: %s", frame.c_str());
}

std::string AprilTagDetector::getReferenceFrame() const {
    return reference_frame_;
}

void AprilTagDetector::setTagFramePrefix(const std::string& prefix) {
    tag_frame_prefix_ = prefix;
    RCLCPP_INFO(node_->get_logger(), "AprilTagDetector tag frame prefix set to: %s", prefix.c_str());
}

