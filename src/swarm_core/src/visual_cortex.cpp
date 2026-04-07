#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

using std::placeholders::_1;

class VisualCortex : public rclcpp::Node {
public:
    VisualCortex() : Node("visual_cortex") {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/image_raw", 10, std::bind(&VisualCortex::image_callback, this, _1));
            
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/features", 10);
        RCLCPP_INFO(this->get_logger(), "Visual Cortex Online. Tracking Shi-Tomasi Features.");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            // 1. Bridge ROS 2 to OpenCV
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            
            // 2. Convert to Grayscale
            cv::Mat gray_image;
            cv::cvtColor(cv_ptr->image, gray_image, cv::COLOR_BGR2GRAY);
            
            // 3. Extract Corners (Features)
            std::vector<cv::Point2f> corners;
            cv::goodFeaturesToTrack(gray_image, corners, 100, 0.01, 10);
            
            // 4. Draw tracking markers (Green circles)
            for (const auto& corner : corners) {
                cv::circle(cv_ptr->image, corner, 5, cv::Scalar(0, 255, 0), -1);
            }
            
            // 5. Publish back to the network
            publisher_->publish(*cv_ptr->toImageMsg());
            
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisualCortex>());
    rclcpp::shutdown();
    return 0;
}
