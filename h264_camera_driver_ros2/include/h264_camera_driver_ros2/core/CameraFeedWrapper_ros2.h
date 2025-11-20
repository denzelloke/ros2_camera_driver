#ifndef CAMERA_FEED_WRAPPER_ROS2_H
#define CAMERA_DEEF_WRAPPER_ROS2_H

#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <string>

// ros2 stuff
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

// jetson libraries
#include <nvbufsurface.h>
#include <nvbufsurftransform.h>

/**
 * @brief Simplified CameraFeedWrapper for ROS2 testing
 *
 * - sets up ros2 publishers for camera feeds
 * - converts frames to ros2 msgs
 * - basic buffer management for nvidia hardware
 *
 *  x no clahe
 *  x no h264 encoding/decoding
 *  x no services for snapshots
 *  x no image rectification
 */

class CameraFeedWrapper {
public:
    CameraFeedWrapper(std::shared_ptr<rclcpp::Node> node, const std::string &frame_id, const std::string &topic_prefix);

    virtual ~CameraFeedWrapper();

    virtual void captureAndPublishOnce() = 0;

protected:
    // publishing functions ======================================

    /**
     * @brief publish a raw yuv img from dma buffer
     * @param dma_buf_fd file descriptor for the nvidia dma buffer
     * @return true if successful
     */
    bool publishRawImage(int32_t dma_buf_fd);

    /**
     * @brief publish a compressed jpeg img from dma buffer
     * @param dma_buf_fd file descriptor for the nvidia dma buffer
     * @return true if successful
     */
    bool publishCompressedImage(int32_t dma_buf_fd);

    /**
     * @brief publish opencv Mat to ros2 image message
     * @param cv_image open img (bgr / yuv)
     * @param encoding image encoding format ("bgr8", "yuv420", etc)
     * @return true if successful
     */
    bool publishCvImage(const cv::Mat &cv_image, const std::string &encoding);

    // buffer helpers =============================================

    /**
     * @brief convert nvidia dma buffer to open cv format
     * @param out_image output cv img
     * @param dma_buf_fd file descriptor for the nvidia dma buffer
     * @return true if successful
     */
    bool nvBufferToMat(cv::Mat &out_image, int32_t dma_buffer_fd);

    /**
     * @brief convert opencv Mat to nvidia dma buffer
     * @param cv_image input cv img
     * @param dma_buf_fd file descriptor for the nvidia dma buffer
     * @return true if successful
     */
    bool matToNvBuffer(const cv::Mat &cv_image, int32_t dma_buf_fd);

    /**
     * @brief create dma buffer for nvidia hardware acceleratation
     * @param width img width in pixels
     * @param height img height in pixels
     * @return dma buffer descriptor, or -1 if fail
     */
    int32_t createDmaBuffer(int32_t width, int32_t height);


    // member variables ================================================

    std::shared_ptr<rclcpp::Node> node_;
    std::string frame_id_;
    std::string topic_prefix_;

    // publishers
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_raw_image_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_compressed_image_;

    // message obj (reuse every frame)
    sensor_msgs::msg::Image raw_image_msg_;
    sensor_msgs::msg::CompressedImage compressed_image_msg_;

    // thread safety??
    std::mutex publish_mutex_;

    uint32_t seq_id_;

    cv::Mat temp_yuv_image_;
    cv::Mat temp_bgr_image_;
};

#endif