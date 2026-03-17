#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <atomic>
#include <mutex>
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

        // Subscribe to camera feed to maintain latest image.
        createImageSubscription();

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
        stop_camera_client_ = this->create_client<std_srvs::srv::Trigger>("/oakd/stop_camera");
    }

private:
    void createImageSubscription() {
        // Recreating the subscription can recover from rare transport-side stalls.
        image_sub_.reset();

        rclcpp::SubscriptionOptions sub_opts;
        sub_opts.callback_group = sub_cb_group_;
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/oakd/rgb/preview/image_raw",
            qos,
            std::bind(&ImageCaptureServer::imageCallback, this, std::placeholders::_1),
            sub_opts
        );
    }

    void clearLatestImage() {
        std::lock_guard<std::mutex> lock(image_mutex_);
        latest_image_.reset();
    }

    sensor_msgs::msg::Image::SharedPtr getLatestImage() {
        std::lock_guard<std::mutex> lock(image_mutex_);
        return latest_image_;
    }

    bool callTrigger(
        const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr& client,
        const std::string& service_name,
        int wait_s = 2,
        int call_timeout_s = 3)
    {
        if (!client->wait_for_service(std::chrono::seconds(wait_s))) {
            RCLCPP_WARN(this->get_logger(), "%s not available", service_name.c_str());
            return false;
        }

        auto trigger = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future = client->async_send_request(trigger);
        if (future.wait_for(std::chrono::seconds(call_timeout_s)) != std::future_status::ready) {
            RCLCPP_WARN(this->get_logger(), "%s call timed out", service_name.c_str());
            return false;
        }

        auto resp = future.get();
        if (!resp->success) {
            RCLCPP_WARN(this->get_logger(), "%s returned failure: %s",
                service_name.c_str(), resp->message.c_str());
        }
        return resp->success;
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        {
            std::lock_guard<std::mutex> lock(image_mutex_);
            latest_image_ = msg;
        }
        frame_counter_++;
        last_frame_steady_ns_.store(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count());
    }

    void captureImageCallback(
        const std::shared_ptr<mie443_contest2::srv::CaptureImage::Request> request,
        std::shared_ptr<mie443_contest2::srv::CaptureImage::Response> response)
    {
        (void)request; // Unused parameter

        // ── PHASE 1: Start camera & flush stale DDS-buffered frames ──
        // The OAK-D auto-stops after inactivity. When restarted, DDS may
        // deliver old buffered frames instantly. We discard ALL frames that
        // arrive in the first 1.0s after restart, then wait for a truly
        // fresh frame.
        clearLatestImage();
        uint64_t pre_flush_count = frame_counter_.load();

        // Try up to 3 times to start the camera.
        const int MAX_CAMERA_ATTEMPTS = 3;

        for (int attempt = 0; attempt < MAX_CAMERA_ATTEMPTS; ++attempt) {
            if (attempt > 0) {
                RCLCPP_WARN(this->get_logger(),
                    "Camera frame not received (attempt %d/%d). Restarting camera...",
                    attempt, MAX_CAMERA_ATTEMPTS);

                // If the stream silently dies, recreating subscription often helps.
                createImageSubscription();
            }

            if (attempt > 0) {
                // Force-stop before restart if service exists (best effort).
                callTrigger(stop_camera_client_, "/oakd/stop_camera", 1, 2);
                rclcpp::sleep_for(std::chrono::milliseconds(150));
            }

            // Wake or restart camera.
            bool start_ok = callTrigger(start_camera_client_, "/oakd/start_camera", 2, 4);
            if (!start_ok && attempt == 0) {
                RCLCPP_WARN(this->get_logger(),
                    "Could not confirm /oakd/start_camera success. Continuing with frame wait.");
            }

            // ── Flush: discard frames for 1.0s to purge stale DDS buffers ──
            RCLCPP_INFO(this->get_logger(), "Flushing stale frames for 1.0s...");
            auto flush_start = this->now();
            while ((this->now() - flush_start).seconds() < 1.0) {
                clearLatestImage();  // keep discarding
                rclcpp::sleep_for(std::chrono::milliseconds(50));
            }
            clearLatestImage();  // final discard

            // ── PHASE 2: Wait for a genuinely fresh frame (up to 3s) ──
            RCLCPP_INFO(this->get_logger(), "Waiting for fresh frame...");
            auto wait_start = this->now();
            double wait_timeout = (attempt == 0) ? 3.0 : 5.0;
            while (!getLatestImage() && (this->now() - wait_start).seconds() < wait_timeout) {
                rclcpp::sleep_for(std::chrono::milliseconds(50));
            }

            if (getLatestImage()) {
                uint64_t post_count = frame_counter_.load();
                RCLCPP_INFO(this->get_logger(),
                    "Got fresh frame (%lu new frames arrived since request)",
                    post_count - pre_flush_count);
                break;
            }
        }

        auto image = getLatestImage();
        if (!image) {
            int64_t last_ns = last_frame_steady_ns_.load();
            double sec_since_last = -1.0;
            if (last_ns > 0) {
                int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()).count();
                sec_since_last = static_cast<double>(now_ns - last_ns) / 1e9;
            }
            response->success = false;
            response->message = "No image available from camera";
            if (sec_since_last >= 0.0) {
                RCLCPP_WARN(this->get_logger(),
                    "Capture requested but no image available (last frame %.2fs ago)",
                    sec_since_last);
            } else {
                RCLCPP_WARN(this->get_logger(), "Capture requested but no image available");
            }
            return;
        }

        try {
            // Convert ROS Image to OpenCV
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);

            // Compress to JPEG (quality 85 for good balance)
            std::vector<int> compression_params;
            compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
            compression_params.push_back(85);

            std::vector<uchar> compressed_data;
            cv::imencode(".jpg", cv_ptr->image, compressed_data, compression_params);

            // Create CompressedImage message
            response->image.header = image->header;
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
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_camera_client_;
    std::mutex image_mutex_;
    sensor_msgs::msg::Image::SharedPtr latest_image_;
    std::atomic<uint64_t> frame_counter_{0};
    std::atomic<int64_t> last_frame_steady_ns_{0};
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
