#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include "mie443_contest2/srv/capture_image.hpp"

/*
    This runs on the robot and captures images from the oakd lite camera.

    It creates a service that can be called to capture an image.
*/

class ImageCaptureServer : public rclcpp::Node {
public:
    ImageCaptureServer() : Node("image_capture_server") {
        // Subscribe to camera feed to maintain latest image
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/oakd/rgb/preview/image_raw",
            10,
            std::bind(&ImageCaptureServer::imageCallback, this, std::placeholders::_1)
        );

        // Create service server
        service_ = this->create_service<mie443_contest2::srv::CaptureImage>(
            "oakd_camera/capture_image",
            std::bind(&ImageCaptureServer::captureImageCallback, this,
                     std::placeholders::_1, std::placeholders::_2)
        );

        RCLCPP_INFO(this->get_logger(), "Image Capture Service Server started on /oakd_camera/capture_image");
        RCLCPP_INFO(this->get_logger(), "Subscribing to /oakd/rgb/preview/image_raw");
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        latest_image_ = msg;
    }

    void captureImageCallback(
        const std::shared_ptr<mie443_contest2::srv::CaptureImage::Request> request,
        std::shared_ptr<mie443_contest2::srv::CaptureImage::Response> response)
    {
        (void)request; // Unused parameter

        if (!latest_image_) {
            response->success = false;
            response->message = "No image available from camera";
            RCLCPP_WARN(this->get_logger(), "Capture requested but no image available");
            return;
        }

        try {
            // Convert ROS Image to OpenCV
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(latest_image_, sensor_msgs::image_encodings::BGR8);

            // Compress to JPEG (quality 85 for good balance)
            std::vector<int> compression_params;
            compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
            compression_params.push_back(85);

            std::vector<uchar> compressed_data;
            cv::imencode(".jpg", cv_ptr->image, compressed_data, compression_params);

            // Create CompressedImage message
            response->image.header = latest_image_->header;
            response->image.format = "jpeg";
            response->image.data = compressed_data;

            response->success = true;
            response->message = "Image captured successfully (" +
                               std::to_string(compressed_data.size()) + " bytes)";

            RCLCPP_INFO(this->get_logger(), "Image captured and compressed: %zu bytes",
                       compressed_data.size());

        } catch (const std::exception& e) {
            response->success = false;
            response->message = std::string("Error compressing image: ") + e.what();
            RCLCPP_ERROR(this->get_logger(), "Failed to compress image: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Service<mie443_contest2::srv::CaptureImage>::SharedPtr service_;
    sensor_msgs::msg::Image::SharedPtr latest_image_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageCaptureServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}