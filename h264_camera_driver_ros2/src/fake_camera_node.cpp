// #include <cv_bridge/cv_bridge.h>
// #include <chrono>
// #include <opencv2/opencv.hpp>
// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/image.hpp>

// class FakeCameraNode : public rclcpp::Node {
// public:
//     FakeCameraNode() : Node("fake_camera_node"), frame_count_(0) {
//         // create publisher
//         publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image", 10);

//         // create timer
//         timer_ = this->create_wall_timer(
//                 std::chrono::milliseconds(33),
//                 std::bind(&FakeCameraNode::timer_callback, this));

//         RCLCPP_INFO(this->get_logger(), "Fake camera node started!");
//     }

// private:
//     void timer_callback() {
//         // create empty matrix for image
//         cv::Mat image(480, 640, CV_8UC3);

//         // fill image with gradient
//         for (int y = 0; y < 480; y++) {
//             for (int x = 0; x < 640; x++) {
//                 image.at<cv::Vec3b>(y, x) = cv::Vec3b(200 - y / 3, 100 + y / 4, 50);
//             }
//         }

//         // moving rectangle
//         int rect_x = (frame_count_ * 5) % 640;
//         cv::rectangle(image, cv::Point(rect_x, 200), cv::Point(rect_x + 50, 280), cv::Scalar(0, 255, 255), -1);

//         // add captions to count frame
//         cv::putText(
//                 image,
//                 "Frame: " + std::to_string(frame_count_),
//                 cv::Point(10, 30),
//                 cv::FONT_HERSHEY_SIMPLEX,
//                 1.0,
//                 cv::Scalar(255, 255, 255),
//                 2);

//         // convert opencv image to ros message
//         auto msg = std::make_shared<sensor_msgs::msg::Image>();
//         cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg(*msg);
//         msg->header.stamp    = this->now();
//         msg->header.frame_id = "camera";

//         // publish
//         publisher_->publish(*msg);

//         // log to terminal every 30 frames
//         if (frame_count_ % 30 == 0) {
//             RCLCPP_INFO(this->get_logger(), "Published frame %d", frame_count_);
//         }
//         frame_count_++;
//     }

//     rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
//     rclcpp::TimerBase::SharedPtr timer_;
//     int frame_count_;
// };

// int main(int argc, char **argv) {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<FakeCameraNode>());
//     rclcpp::shutdown();
//     return 0;
// }


#include <cv_bridge/cv_bridge.h>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class FakeCameraNode : public rclcpp::Node {
public:
    FakeCameraNode() : Node("fake_camera_node"), frame_count_(0) {
        // Create publisher
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image", 10);

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

        // Convert OpenCV image to ROS message
        auto msg             = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
        msg->header.stamp    = this->now();
        msg->header.frame_id = "camera";

        // Publish
        publisher_->publish(*msg);

        // Log every 30 frames (once per second)
        if (frame_count_ % 30 == 0) {
            RCLCPP_INFO(this->get_logger(), "Published frame %d", frame_count_);
        }

        frame_count_++;
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int frame_count_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FakeCameraNode>());
    rclcpp::shutdown();
    return 0;
}