#include <h264_camera_driver_ros2/core/CameraFeedWrapper_ros2.h>

CameraFeedWrapper::CameraFeedWrapper(
        std::shared_ptr<rclcpp::Node> node,
        const std::string &frame_id,
        const std::string &topic_prefix)
        : node_(node),
          frame_id_(frame_id),
          topic_prefix_(topic_prefix),
          seq_id_(0) {
    RCLCPP_INFO(node_->get_logger(), "initialising CameraFeedWrapper...");
    RCLCPP_INFO(node_->get_logger(), " Frame IF %s", frame_id_.c_str());
    RCLCPP_INFO(node_->get_logger(), " Topic prefix: %s", topic_prefix_.c_str());

    // create topic names
    std::string raw_topic        = topic_prefix_ + "/image_raw";
    std::string compressed_topic = topic_prefix_ + "/image_raw/compressed";

    // create publishers
    pub_raw_image_        = node_->create_publisher<sensor_msgs::msg::Image>(raw_topic, 10);
    pub_compressed_image_ = node_->create_publisher<sensor_msgs::msg::CompressedImage>(compressed_topic, 10);

    RCLCPP_INFO(node_->get_logger(), "Publishers created:");
    RCLCPP_INFO(node_->get_logger(), " - %s", raw_topic.c_str());
    RCLCPP_INFO(node_->get_logger(), " - %s", compressed_topic.c_str());

    // prefill message headers
    raw_image_msg_.header.frame_id        = frame_id_;
    compressed_image_msg_.header.frame_id = frame_id_;
    compressed_image_msg_.format          = "jpeg";
    RCLCPP_INFO(node_->get_logger(), "CameraFeedWrapper initialisation complete!");
}

// destructor
CameraFeedWrapper::~CameraFeedWrapper() { RCLCPP_INFO(node_->get_logger(), "Shutting down CameraFeedWrapper xoxoxo"); }

// publishing functions
bool CameraFeedWrapper::publishRawImage(int32_t dma_buf_fd) {
    std::lock_guard<std::mutex> lock(publish_mutex_);

    // convert dma buffer to opencv Mat
    cv::Mat yuv_image;
    if (!nvBufferToMat(yuv_image, dma_buf_fd)) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to convert DMA buffer to OpenCV Mat!! :( \npublishRawImage error");
        return false;
    }

    // convert yuv to bgr
    cv::Mat bgr_image;
    cv::cvtColor(yuv_image, bgr_image, cv::COLOR_YUV2BGR_I420);

    // publish
    return publishCvImage(bgr_image, "bgr8");
}

bool CameraFeedWrapper::publishCompressedImage(int32_t dma_buf_fd) {
    std::lock_guard<std::mutex> lock(publish_mutex_);

    // convert dma buffer to opencv mat
    cv::Mat yuv_image;
    if (!nvBufferToMat(yuv_image, dma_buf_fd)) {
        RCLCPP_ERROR(
                node_->get_logger(),
                "Failed to convert DMA buffer to OpenCV Mat!! :( \npublishCompressedImage error");
        return false;
    }

    // convert yuv to bgr for jpg compression
    cv::Mat bgr_image;
    cv::cvtColor(yuv_image, bgr_image, cv::COLOR_YUV2BGR_I420);

    // compress to jpg using imencode
    std::vector<uint8_t> jpeg_buffer;
    std::vector<int> compression_params;
    compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
    compression_params.push_back(80);

    if (!cv::imencode(".jpg", bgr_image, jpeg_buffer, compression_params)) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to encode bgr_image in publishCompressedImage :(");
        return false;
    }

    // build compressed image msg
    compressed_image_msg_.header.stamp    = node_->now();
    compressed_image_msg_.header.frame_id = frame_id_;
    compressed_image_msg_.format          = "jpeg";
    compressed_image_msg_.data            = jpeg_buffer;

    // PUBLISHHHH
    pub_compressed_image_->publish(compressed_image_msg_);

    seq_id_++;
    return true;
}

bool CameraFeedWrapper::publishCvImage(const cv::Mat &cv_image, const std::string &encoding) {
    // verify image is valid
    if (cv_image.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "cv_image empty in publishCvImage! :(");
        return false;
    }

    // fill header
    raw_image_msg_.header.stamp    = node_->now();
    raw_image_msg_.header.frame_id = frame_id_;

    // image dimensions
    raw_image_msg_.height   = cv_image.rows;
    raw_image_msg_.width    = cv_image.cols;
    raw_image_msg_.encoding = encoding;

    // calculate bytes per row
    // step = width * channels * bytes_per_channel
    raw_image_msg_.step = cv_image.cols * cv_image.elemSize();

    // set endian (0 is little endian)
    raw_image_msg_.is_bigendian = 0;

    // calc total data size
    size_t data_size = cv_image.rows * raw_image_msg_.step;

    // resize the data vector and copy pixel data directly
    raw_image_msg_.data.resize(data_size);

    // copy data from cv::Mat to msg
    if (cv_image.isContinuous()) {
        std::memcpy(raw_image_msg_.data.data(), cv_image.data, data_size);
    } else {
        for (int i = 0; i < cv_image.rows; i++) {
            std::memcpy(raw_image_msg_.data.data() + i * raw_image_msg_.step, cv_image.ptr(i), raw_image_msg_.step);
        }
    }

    // PUBLICSHHHHH
    pub_raw_image_->publish(raw_image_msg_);
    seq_id_++;
    return true;
}

// buffer helpers

bool CameraFeedWrapper::nvBufferToMat(cv::Mat &out_image, int32_t dma_buf_fd) {
    // get the buffer params to know the size
    NvBufSurfaceParams surf_params;

    // turns surf into a pointer to nvidia mem buffer surface
    NvBufSurface *surf = nullptr;
    if (NvBufSurfaceFromFd(dma_buf_fd, (void **)(&surf)) != 0) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to get surface from fd");
        return false;
    }

    // maps hardware batched buffers to cpu address space
    if (NvBufSurfaceMap(surf, 0, 0, NVBUF_MAP_READ) != 0) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to map surface");
        return false;
    }

    // get img dims
    // note: yuv420 has height * 1.5 row (y plane + uv planes)
    int width  = surf->surfaceList[0].width;  // surfaceList: holds a pointer to an array of batched buffers.
    int height = surf->surfaceList[0].height;

    // create opencv mat for yuv420 data
    if (out_image.empty() || out_image.cols != width || out_image.rows != height * 3 / 2) {
        out_image.create(height * 3 / 2, width, CV_8UC1);
    }

    // copy data from gpu bufer to opencv mat
    void *src_ptr = surf->surfaceList[0].mappedAddr.addr[0];
    int src_pitch = surf->surfaceList[0].pitch;

    // copy y plane
    for (int i = 0; i < height; i++) {
        memcpy(out_image.ptr(i), (uint8_t *)src_ptr + i * src_pitch, width);
    }

    // copy uv planes
    void *uv_src = (uint8_t *)src_ptr + height * src_pitch;
    for (int i = 0; i < height / 2; i++) {
        memcpy(out_image.ptr(height + i), (uint8_t *)uv_src + i * src_pitch, width);
    }

    // done alr unmap
    NvBufSurfaceUnMap(surf, 0, 0);
    return true;
}

bool CameraFeedWrapper::matToNvBuffer(const cv::Mat &cv_image, int32_t dma_buf_fd) {
    NvBufSurface *surf = nullptr;
    if (NvBufSurfaceFromFd(dma_buf_fd, (void **)(&surf)) != 0) {
        RCLCPP_ERROR(node_->get_logger(), "failed to get buffer pointer in matToNvBuffer! :(");
        return false;
    }

    if (NvBufSurfaceMap(surf, 0, 0, NVBUF_MAP_WRITE) != 0) {
        RCLCPP_ERROR(node_->get_logger(), "failed to map buffer to cpu in matToNvBuffer :(");
        return false;
    }

    // get dims
    int width  = surf->surfaceList[0].width;
    int height = surf->surfaceList[0].height;

    // verify image size matches
    if (cv_image.cols != width || cv_image.rows != height * 3 / 2) {
        RCLCPP_ERROR(
                node_->get_logger(),
                "image size mismatch. expected %d x %d. got %d x %d",
                width,
                height * 3 / 2,
                cv_image.cols,
                cv_image.rows * 3 / 2);
        NvBufSurfaceUnMap(surf, 0, 0);
        return false;
    }

    // copy data from opencv mat to gpu buffer
    void *dst_ptr = surf->surfaceList[0].mappedAddr.addr[0];
    int dst_pitch = surf->surfaceList[0].pitch;

    // copy y plane
    for (int i = 0; i < height; i++) {
        memcpy((uint8_t *)dst_ptr + i * dst_pitch, cv_image.ptr(i), width);
    }

    // copy uv plane
    void *uv_dst = (uint8_t *)dst_ptr + height * dst_pitch;
    for (int i = 0; i < height / 2; i++) {
        memcpy((uint8_t *)uv_dst + i * dst_pitch, cv_image.ptr(height + i), width);
    }

    // unmap when done
    NvBufSurfaceUnMap(surf, 0, 0);
    return true;
}

int32_t CameraFeedWrapper::createDmaBuffer(int32_t width, int32_t height) {
    // create params for dma buffer
    NvBufSurfaceAllocateParams alloc_params;
    memset(&alloc_params, 0, sizeof(alloc_params));

    alloc_params.params.width       = width;
    alloc_params.params.height      = height;
    alloc_params.params.layout      = NVBUF_LAYOUT_PITCH;
    alloc_params.params.colorFormat = NVBUF_COLOR_FORMAT_YUV420;
    alloc_params.params.memType     = NVBUF_MEM_SURFACE_ARRAY;
    alloc_params.memtag             = NvBufSurfaceTag_NONE;

    // allocate memory to the buffer, make *surf the pointer
    NvBufSurface *surf = nullptr;
    if (NvBufSurfaceAllocate(&surf, 1, &alloc_params) != 0) {
        RCLCPP_ERROR(node_->get_logger(), "failed to allocate dma buffer (%d x %d)", width, height);
        return -1;
    }

    // get file descriptor
    int32_t dma_buf_fd = surf->surfaceList[0].bufferDesc;

    RCLCPP_INFO(node_->get_logger(), "created dma buffer! :D %dx%x, fd=%d", width, height, dma_buf_fd);
    return dma_buf_fd;
}