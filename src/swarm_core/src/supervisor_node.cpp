#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "swarm_interfaces/msg/zero_copy_frame.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

class FlightSupervisor : public rclcpp::Node {
public:
    FlightSupervisor() : Node("flight_supervisor"), last_heartbeat_(this->now()) {
        sub_ = this->create_subscription<swarm_interfaces::msg::ZeroCopyFrame>(
            "/swarm/vision/raw", 10, std::bind(&FlightSupervisor::vision_callback, this, std::placeholders::_1));
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        watchdog_timer_ = this->create_wall_timer(100ms, std::bind(&FlightSupervisor::check_safety, this));
        RCLCPP_INFO(this->get_logger(), "Flight Supervisor Online. P-Controller Active.");
    }

private:
    void vision_callback(const swarm_interfaces::msg::ZeroCopyFrame::SharedPtr msg) {
        last_heartbeat_ = this->now();
        auto cmd = geometry_msgs::msg::Twist();
        
        // 1. Wrap the shared memory in OpenCV (Zero Copy)
        cv::Mat frame(msg->height, msg->width, CV_8UC2, msg->data.data());
        cv::Mat gray, edges;
        
        // 2. Extract features
        cv::cvtColor(frame, gray, cv::COLOR_YUV2GRAY_YUY2);
        cv::Canny(gray, edges, 50.0, 100.0);
        
        // 3. Calculate Image Moments (Center of Mass of the edges)
        cv::Moments m = cv::moments(edges, true);
        
        if (m.m00 > 1000) { // If there are enough edges to care about
            double cx = m.m10 / m.m00; // Calculate X coordinate of the mass
            
            // 4. The P-Controller Math
            double setpoint = msg->width / 2.0; // Ideal center (e.g., 320)
            double error = setpoint - cx;       // How far off center are the edges?
            double kp = 0.005;                  // The Gain multiplier
            
            // Steer AWAY from the edges. 
            // If edges are on the right (cx > 320), error is negative. 
            // Negative angular Z turns the drone right. We want to turn LEFT, so we invert the error.
            cmd.angular.z = kp * (-error); 
            cmd.linear.x = 0.2; // Move forward slowly while turning
            
        } else {
            // No significant obstacles, fly straight fast
            cmd.linear.x = 0.8;
            cmd.angular.z = 0.0;
        }

        cmd_pub_->publish(cmd);
    }

    void check_safety() {
        auto age = this->now() - last_heartbeat_;
        if (age > 500ms) {
            auto stop_cmd = geometry_msgs::msg::Twist(); 
            cmd_pub_->publish(stop_cmd);
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "WATCHDOG: Vision Lost. Hovering.");
        }
    }

    rclcpp::Subscription<swarm_interfaces::msg::ZeroCopyFrame>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    rclcpp::Time last_heartbeat_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FlightSupervisor>());
    rclcpp::shutdown();
    return 0;
}
