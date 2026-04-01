#include <chrono>
#include <memory>
#include <utility>
#include <string>
#include <cstring>
#include <stdexcept>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mmap.h>
#include <linux/videodev2.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::chrono_literals;

class CameraNode : public rclcpp::Node
{
public:
    CameraNode() : Node("camera_node"), fd_(-1), buffer_start_(nullptr), buffer_length_(0)
    {
        rclcpp::QoS qos_profile(rclcpp::KeepLast(1));
        qos_profile.best_effort();
        qos_profile.durability_volatile();

        pub_ = this->create_publisher<sensor_msgs::msg::Image>("/swarm/vision/raw", qos_profile);
        init_v4l2("/dev/video0", 640, 480);
        timer_ = this->create_wall_timer(33ms, std::bind(&CameraNode::capture, this));
        RCLCPP_INFO(this->get_logger(), "Zero-Copy Camera (V4L2 -> SHM) Initialized.");
    }

    ~CameraNode()
    {
        if (buffer_start_) munmap(buffer_start_, buffer_length_);
        if (fd_ != -1) close(fd_);
    }

private:
    int fd_;
    void* buffer_start_;
    size_t buffer_length_;
    uint32_t width_;
    uint32_t height_;

    void init_v4l2(const std::string& device, uint32_t w, uint32_t h)
    {
        width_ = w; height_ = h;
        fd_ = open(device.c_str(), O_RDWR | O_NONBLOCK, 0);
        if (fd_ == -1) throw std::runtime_error("FATAL: Cannot open /dev/video0");

        struct v4l2_format fmt = {};
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width = width_;
        fmt.fmt.pix.height = height_;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
        fmt.fmt.pix.field = V4L2_FIELD_NONE;
        if (ioctl(fd_, VIDIOC_S_FMT, &fmt) == -1) throw std::runtime_error("FATAL: Format error");

        struct v4l2_requestbuffers req = {};
        req.count = 1; req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; req.memory = V4L2_MEMORY_MMAP;
        if (ioctl(fd_, VIDIOC_REQBUFS, &req) == -1) throw std::runtime_error("FATAL: REQBUFS error");

        struct v4l2_buffer buf = {};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; buf.memory = V4L2_MEMORY_MMAP; buf.index = 0;
        if (ioctl(fd_, VIDIOC_QUERYBUF, &buf) == -1) throw std::runtime_error("FATAL: QUERYBUF error");

        buffer_length_ = buf.length;
        buffer_start_ = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, buf.m.offset);
        if (buffer_start_ == MAP_FAILED) throw std::runtime_error("FATAL: mmap() failed");

        if (ioctl(fd_, VIDIOC_QBUF, &buf) == -1) throw std::runtime_error("FATAL: QBUF error");
        int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (ioctl(fd_, VIDIOC_STREAMON, &type) == -1) throw std::runtime_error("FATAL: STREAMON error");
    }

    void capture()
    {
        struct v4l2_buffer buf = {};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; buf.memory = V4L2_MEMORY_MMAP;
        if (ioctl(fd_, VIDIOC_DQBUF, &buf) == -1) return;

        auto loaned_msg = pub_->borrow_loaned_message();
        if (loaned_msg.is_valid()) {
            auto& msg = loaned_msg.get();
            msg.header.stamp = this->now();
            msg.header.frame_id = "camera_link";
            msg.height = height_; msg.width = width_;
            msg.encoding = "yuv422_yuy2";
            msg.is_bigendian = 0; msg.step = width_ * 2;
            msg.data.resize(buffer_length_);
            
            // Bypass user-space heap. Copy directly to mapped SHM.
            std::memcpy(msg.data.data(), buffer_start_, buffer_length_);
            pub_->publish(std::move(loaned_msg));
        }

        ioctl(fd_, VIDIOC_QBUF, &buf);
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraNode>());
    rclcpp::shutdown();
    return 0;
}
