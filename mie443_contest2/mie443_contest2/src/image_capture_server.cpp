#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_srvs/srv/trigger.hpp>
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
        // Separate callback groups so the subscription can deliver frames
        // while the service callback is waiting for a fresh image.
        sub_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        srv_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        // Subscribe to camera feed to maintain latest image
        rclcpp::SubscriptionOptions sub_opts;
        sub_opts.callback_group = sub_cb_group_;
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/oakd/rgb/preview/image_raw",
            10,
            std::bind(&ImageCaptureServer::imageCallback, this, std::placeholders::_1),
            sub_opts
        );

        // Create service server (in its own callback group)
        service_ = this->create_service<mie443_contest2::srv::CaptureImage>(
            "oakd_camera/capture_image",
            std::bind(&ImageCaptureServer::captureImageCallback, this,
                     std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default,
            srv_cb_group_
        );

        RCLCPP_INFO(this->get_logger(), "Image Capture Service Server started on /oakd_camera/capture_image");
        RCLCPP_INFO(this->get_logger(), "Subscribing to /oakd/rgb/preview/image_raw");

        // Client to start the OAK-D camera (it auto-stops after inactivity)
        start_camera_client_ = this->create_client<std_srvs::srv::Trigger>("/oakd/start_camera");
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

        // Ensure the OAK-D camera is streaming (it auto-stops after inactivity).
        // Clear the cached image so we get a FRESH frame, not a stale one.
        latest_image_ = nullptr;

        // Try up to 3 times to start the camera and get a frame.
        // Each attempt calls /oakd/start_camera and waits progressively longer.
        const int MAX_CAMERA_ATTEMPTS = 3;
        const double WAIT_SECONDS[] = {3.0, 5.0, 8.0};

        for (int attempt = 0; attempt < MAX_CAMERA_ATTEMPTS && !latest_image_; ++attempt) {
            if (attempt > 0) {
                RCLCPP_WARN(this->get_logger(),
                    "Camera frame not received (attempt %d/%d). Restarting camera...",
                    attempt, MAX_CAMERA_ATTEMPTS);
                latest_image_ = nullptr;
            }

            // Call /oakd/start_camera to wake it up (or restart it)
            if (start_camera_client_->wait_for_service(std::chrono::seconds(2))) {
                auto trigger = std::make_shared<std_srvs::srv::Trigger::Request>();
                auto future = start_camera_client_->async_send_request(trigger);
                // Wait for the start_camera call to complete (best effort)
                future.wait_for(std::chrono::seconds(3));
            } else if (attempt == 0) {
                RCLCPP_WARN(this->get_logger(),
                    "/oakd/start_camera service not available — camera may already be running");
            }

            // Wait for a fresh frame with increasing timeout per attempt
            double timeout = WAIT_SECONDS[attempt];
            auto start = this->now();
            while (!latest_image_ && (this->now() - start).seconds() < timeout) {
                rclcpp::sleep_for(std::chrono::milliseconds(50));
            }
        }

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

    rclcpp::CallbackGroup::SharedPtr sub_cb_group_;
    rclcpp::CallbackGroup::SharedPtr srv_cb_group_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Service<mie443_contest2::srv::CaptureImage>::SharedPtr service_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_camera_client_;
    sensor_msgs::msg::Image::SharedPtr latest_image_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageCaptureServer>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}