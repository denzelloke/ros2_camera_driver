// Example: Fake Camera Node using CameraFeedWrapper
// This demonstrates how to use the wrapper with test patterns
// Later, replace this with your real IMX678 camera capture code!

#include <h264_camera_driver_ros2/core/CameraFeedWrapper_ros2.h>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

/**
 * @brief A simple fake camera that generates test patterns
 *
 * This inherits from CameraFeedWrapper and implements captureAndPublishOnce()
 * to generate and publish test images.
 */
class FakeCameraNode : public CameraFeedWrapper {
public:
    FakeCameraNode(std::shared_ptr<rclcpp::Node> node,
                   const std::string& frame_id,
                   const std::string& topic_prefix,
                   int width = 640,
                   int height = 480)
        : CameraFeedWrapper(node, frame_id, topic_prefix)
        , width_(width)
        , height_(height)
        , frame_count_(0)
    {
        RCLCPP_INFO(node_->get_logger(), "Fake Camera Node created (%dx%d)", width_, height_);
        // NO DMA buffer needed for fake camera!
    }

    void captureAndPublishOnce() override {
        cv::Mat test_image = generateTestPattern();

        // Publish directly - no DMA buffer conversion
        publishCvImage(test_image, "bgr8");

        // Compress and publish
        std::vector<uint8_t> jpeg_buffer;
        std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 80};
        if (cv::imencode(".jpg", test_image, jpeg_buffer, params)) {
            compressed_image_msg_.header.stamp = node_->now();
            compressed_image_msg_.data = jpeg_buffer;
            pub_compressed_image_->publish(compressed_image_msg_);
        }

        frame_count_++;
    }

private:
    cv::Mat generateTestPattern() {
        cv::Mat img(height_, width_, CV_8UC3);

        for (int y = 0; y < height_; y++) {
            for (int x = 0; x < width_; x++) {
                int val = (x + y + frame_count_ * 2) % 255;
                img.at<cv::Vec3b>(y, x) = cv::Vec3b(
                    val,
                    (val + 85) % 255,
                    (val + 170) % 255
                );
            }
        }

        // Skip putText - can cause issues
        return img;
    }

    int width_;
    int height_;
    int frame_count_;
    // Removed dma_buf_fd_ - not needed!
};

// ========================================
// Main Function
// ========================================

int main(int argc, char** argv) {
    // Initialize ROS2
    rclcpp::init(argc, argv);

    // Create the ROS2 node
    auto node = std::make_shared<rclcpp::Node>("fake_camera_node");

    RCLCPP_INFO(node->get_logger(), "========================================");
    RCLCPP_INFO(node->get_logger(), "Starting Fake Camera Node");
    RCLCPP_INFO(node->get_logger(), "========================================");

    // Get parameters (you can set these with ros2 run or launch files)
    node->declare_parameter("frame_id", "camera");
    node->declare_parameter("topic_prefix", "/camera");
    node->declare_parameter("width", 640);
    node->declare_parameter("height", 480);
    node->declare_parameter("fps", 30);

    std::string frame_id = node->get_parameter("frame_id").as_string();
    std::string topic_prefix = node->get_parameter("topic_prefix").as_string();
    int width = node->get_parameter("width").as_int();
    int height = node->get_parameter("height").as_int();
    int fps = node->get_parameter("fps").as_int();

    RCLCPP_INFO(node->get_logger(), "Configuration:");
    RCLCPP_INFO(node->get_logger(), "  Frame ID: %s", frame_id.c_str());
    RCLCPP_INFO(node->get_logger(), "  Topic Prefix: %s", topic_prefix.c_str());
    RCLCPP_INFO(node->get_logger(), "  Resolution: %dx%d", width, height);
    RCLCPP_INFO(node->get_logger(), "  Target FPS: %d", fps);

    try {
        // Create the fake camera
        auto camera = std::make_shared<FakeCameraNode>(node, frame_id, topic_prefix, width, height);

        // Create a timer to call captureAndPublishOnce() at regular intervals
        auto timer_period = std::chrono::milliseconds(1000 / fps);  // Convert FPS to period
        auto timer = node->create_wall_timer(
            timer_period,
            [camera]() {
                camera->captureAndPublishOnce();
            }
        );

        RCLCPP_INFO(node->get_logger(), "Fake camera running! Press Ctrl+C to stop.");
        RCLCPP_INFO(node->get_logger(), "...");
        RCLCPP_INFO(node->get_logger(), "To view the images:");
        RCLCPP_INFO(node->get_logger(), "  ros2 run rqt_image_view rqt_image_view");
        RCLCPP_INFO(node->get_logger(), "Or run the test script:");
        RCLCPP_INFO(node->get_logger(), "  python3 test_camera_feed.py --prefix %s", topic_prefix.c_str());
        RCLCPP_INFO(node->get_logger(), "...");

        // Spin - keeps the node running and timer firing
        rclcpp::spin(node);

    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    // Cleanup
    RCLCPP_INFO(node->get_logger(), "Shutting down fake camera node...");
    rclcpp::shutdown();
    return 0;
}