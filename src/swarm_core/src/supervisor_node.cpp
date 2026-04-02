#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "swarm_interfaces/msg/zero_copy_frame.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class FlightSupervisor : public rclcpp::Node {
public:
    FlightSupervisor() : Node("flight_supervisor"), last_heartbeat_(this->now()) {
        sub_ = this->create_subscription<swarm_interfaces::msg::ZeroCopyFrame>(
            "/swarm/vision/raw", 10, std::bind(&FlightSupervisor::vision_callback, this, std::placeholders::_1));
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        // Safety Watchdog Timer (Check every 100ms)
        watchdog_timer_ = this->create_wall_timer(100ms, std::bind(&FlightSupervisor::check_safety, this));
        RCLCPP_INFO(this->get_logger(), "Flight Supervisor Online. Watchdog Active.");
    }

private:
    void vision_callback(const swarm_interfaces::msg::ZeroCopyFrame::SharedPtr msg) {
        last_heartbeat_ = this->now();
        auto cmd = geometry_msgs::msg::Twist();
        
        // Placeholder for the Center of Mass P-Controller
        cmd.linear.x = 0.5; 
        cmd_pub_->publish(cmd);
    }

    void check_safety() {
        auto age = this->now() - last_heartbeat_;
        if (age > 500ms) {
            auto stop_cmd = geometry_msgs::msg::Twist(); 
            cmd_pub_->publish(stop_cmd);
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "WATCHDOG TRIGGERED: Vision Lost. Hovering.");
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
