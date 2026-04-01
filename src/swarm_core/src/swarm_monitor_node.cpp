#include <chrono>
#include <memory>
#include <mutex>
#include <atomic>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/uint64.hpp"

using namespace std::chrono_literals;

class SwarmMonitorNode : public rclcpp::Node
{
public:
    SwarmMonitorNode() : Node("swarm_monitor_node"), initialized_(false), triggered_(false)
    {
        rclcpp::QoS qos_profile(rclcpp::KeepLast(5));
        qos_profile.best_effort();
        qos_profile.durability_volatile();

        sub_ = this->create_subscription<std_msgs::msg::UInt64>(
            "/swarm/core/heartbeat", qos_profile, std::bind(&SwarmMonitorNode::callback, this, std::placeholders::_1));
        
        timer_ = this->create_wall_timer(100ms, std::bind(&SwarmMonitorNode::watchdog, this));
        RCLCPP_INFO(this->get_logger(), "Swarm Monitor initialized. Awaiting SHM payload.");
    }
private:
    void callback(const std_msgs::msg::UInt64::SharedPtr msg)
    {
        (void)msg;
        std::lock_guard<std::mutex> lock(mtx_);
        last_time_ = this->now();
        if (!initialized_.exchange(true)) RCLCPP_INFO(this->get_logger(), "Watchdog Armed.");
    }

    void watchdog()
    {
        if (!initialized_.load() || triggered_.load()) return;
        rclcpp::Time current, last;
        {
            std::lock_guard<std::mutex> lock(mtx_);
            current = this->now();
            last = last_time_;
        }
        if ((current - last).seconds() > 1.5) {
            triggered_.store(true);
            RCLCPP_FATAL(this->get_logger(), "CRITICAL FAULT: EXECUTING ASYNCHRONOUS FAILOVER.");
        }
    }

    rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_time_;
    std::mutex mtx_;
    std::atomic<bool> initialized_;
    std::atomic<bool> triggered_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SwarmMonitorNode>());
    rclcpp::shutdown();
    return 0;
}
