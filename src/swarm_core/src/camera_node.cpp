#include <chrono>
#include <memory>
#include <utility>
#include <string>
#include <cstring>
#include <stdexcept>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include <cuda_runtime.h>

#include <opencv2/opencv.hpp>
#ifdef __aarch64__
  #include <opencv2/cudaimgproc.hpp>
  #include <opencv2/cudawarping.hpp>
#endif

#include "rclcpp/rclcpp.hpp"
#include "swarm_interfaces/msg/zero_copy_frame.hpp"

using namespace std::chrono_literals;

class CameraNode : public rclcpp::Node
{
public:
    CameraNode() : Node("camera_node"), fd_(-1), buffer_start_(nullptr), buffer_length_(0)
    {
        rclcpp::QoS qos_profile(rclcpp::KeepLast(1));
        qos_profile.best_effort();
        qos_profile.durability_volatile();

        pub_ = this->create_publisher<swarm_interfaces::msg::ZeroCopyFrame>("/swarm/vision/raw", qos_profile);
        init_v4l2("/dev/video0", 640, 480);
        timer_ = this->create_wall_timer(33ms, std::bind(&CameraNode::capture, this));
        RCLCPP_INFO(this->get_logger(), "Hardware-Agnostic Vision Node Initialized.");
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
        ioctl(fd_, VIDIOC_S_FMT, &fmt);

        struct v4l2_requestbuffers req = {};
        req.count = 1; req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; req.memory = V4L2_MEMORY_MMAP;
        ioctl(fd_, VIDIOC_REQBUFS, &req);

        struct v4l2_buffer buf = {};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; buf.memory = V4L2_MEMORY_MMAP; buf.index = 0;
        ioctl(fd_, VIDIOC_QUERYBUF, &buf);

        buffer_length_ = buf.length;
        buffer_start_ = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, buf.m.offset);
        
        ioctl(fd_, VIDIOC_QBUF, &buf);
        int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ioctl(fd_, VIDIOC_STREAMON, &type);
    }

    void capture()
    {
        struct v4l2_buffer buf = {};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; buf.memory = V4L2_MEMORY_MMAP;
        if (ioctl(fd_, VIDIOC_DQBUF, &buf) == -1) return;

        auto loaned_msg = pub_->borrow_loaned_message();
        if (loaned_msg.is_valid()) {
            auto& msg = loaned_msg.get();
            msg.timestamp = this->now().nanoseconds();
            msg.width = width_; 
            msg.height = height_;
            
            size_t copy_size = (buffer_length_ > 614400) ? 614400 : buffer_length_;
            std::memcpy(msg.data.data(), buffer_start_, copy_size);
            
#ifdef __aarch64__
            // --- JETSON EDGE PIPELINE (CUDA GPU) ---
            void* gpu_pointer = nullptr;
            cudaHostRegister(msg.data.data(), copy_size, cudaHostRegisterMapped);
            cudaHostGetDevicePointer(&gpu_pointer, msg.data.data(), 0);
            
            cv::cuda::GpuMat gpu_raw(height_, width_, CV_8UC2, gpu_pointer);
            cv::cuda::GpuMat gpu_gray, gpu_edges;
            
            cv::cuda::cvtColor(gpu_raw, gpu_gray, cv::COLOR_YUV2GRAY_YUY2);
            cv::Ptr<cv::cuda::CannyEdgeDetector> canny = cv::cuda::createCannyEdgeDetector(50.0, 100.0);
            canny->detect(gpu_gray, gpu_edges);
#else
            // --- KATANA PC PIPELINE (CPU FALLBACK) ---
            // Wraps the pre-allocated Fast DDS memory in a CPU cv::Mat. Zero copies.
            cv::Mat cpu_raw(height_, width_, CV_8UC2, msg.data.data());
            cv::Mat cpu_gray, cpu_edges;
            
            cv::cvtColor(cpu_raw, cpu_gray, cv::COLOR_YUV2GRAY_YUY2);
            cv::Canny(cpu_gray, cpu_edges, 50.0, 100.0);
#endif

            pub_->publish(std::move(loaned_msg));
        }

        ioctl(fd_, VIDIOC_QBUF, &buf);
    }

    rclcpp::Publisher<swarm_interfaces::msg::ZeroCopyFrame>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraNode>());
    rclcpp::shutdown();
    return 0;
}
