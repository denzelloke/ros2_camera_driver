#include <chrono>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

class FakeCameraNode : public rclcpp::Node {
public:
    FakeCameraNode() : Node("fake_camera_node"), frame_count_(0) {
        // Create publisher
        publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("camera/image/compressed", 10);

        // Create timer - 30 FPS = publish every 33ms
        timer_ = this->create_wall_timer(
                std::chrono::milliseconds(33),
                std::bind(&FakeCameraNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Fake camera node started!");
    }

private:
    void timer_callback() {
        // Create a simple test image (640x480, blue-green gradient)
        cv::Mat image(480, 640, CV_8UC3);

        // Fill with gradient + moving rectangle
        for (int y = 0; y < 480; y++) {
            for (int x = 0; x < 640; x++) {
                // Blue-green gradient (underwater-ish)
                image.at<cv::Vec3b>(y, x) = cv::Vec3b(
                        200 - y / 3,  // Blue
                        100 + y / 4,  // Green
                        50);
            }
        }

        // Add moving rectangle so we can see it's alive
        int rect_x = (frame_count_ * 5) % 640;
        cv::rectangle(
                image,
                cv::Point(rect_x, 200),
                cv::Point(rect_x + 50, 280),
                cv::Scalar(0, 255, 255),  // Yellow
                -1);

        // Add frame counter text
        cv::putText(
                image,
                "Frame: " + std::to_string(frame_count_),
                cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX,
                1.0,
                cv::Scalar(255, 255, 255),
                2);
        
        // encode image
        std::vector<uchar> buffer;
        std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 90};  // 90% quality
        cv::imencode(".jpg", image, buffer, params);

        // create ROS message
        auto msg = std::make_shared<sensor_msgs::msg::CompressedImage>();
        msg->header.stamp = this->now();
        msg->header.frame_id = "camera";
        msg->format = "jpeg";
        msg->data = buffer;

        // Publish
        publisher_->publish(*msg);

        // Log every 30 frames (once per second)
        if (frame_count_ % 30 == 0) {
            RCLCPP_INFO(this->get_logger(), "Published frame %d", frame_count_);
        }

        frame_count_++;
    }

    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int frame_count_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FakeCameraNode>());
    rclcpp::shutdown();
    return 0;
}