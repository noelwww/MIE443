#pragma once

#include <rclcpp/rclcpp.hpp>
#include <mie443_contest2/srv/capture_image.hpp>
#include <mie443_contest2/srv/detect_object.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <string>

/**
 * Camera source selection for YOLO detection
 */
enum class CameraSource {
    OAKD,   // OAK-D camera on TurtleBot4
    WRIST   // Wrist camera on SO-ARM101
};


class YoloInterface {
private:
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Client<mie443_contest2::srv::CaptureImage>::SharedPtr capture_client_oakd_;
    rclcpp::Client<mie443_contest2::srv::CaptureImage>::SharedPtr capture_client_wrist_;
    rclcpp::Client<mie443_contest2::srv::DetectObject>::SharedPtr detect_client_;

    // Latest detection results
    std::string latest_class_name_;
    float latest_confidence_;
    bool has_detection_;

public:
    YoloInterface(std::shared_ptr<rclcpp::Node> node);

    bool captureAndDetect(CameraSource camera, bool save_image = false);

    std::string getObjectName(CameraSource camera, bool save_image = false);

    float getConfidence();

    std::string getLatestClassName();

    bool hasDetection();

    void clearDetection();
};
