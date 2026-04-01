#include <chrono>
#include <memory>
#include <utility>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"

using namespace std::chrono_literals;

class CoreBridgeNode : public rclcpp::Node
{
public:
    CoreBridgeNode() : Node("core_bridge_node")
    {
        rclcpp::QoS qos_profile(rclcpp::KeepLast(5));
        qos_profile.best_effort();
        qos_profile.durability_volatile();

        publisher_ = this->create_publisher<std_msgs::msg::UInt64>("/swarm/core/heartbeat", qos_profile);
        timer_ = this->create_wall_timer(500ms, std::bind(&CoreBridgeNode::publish_heartbeat, this));
        RCLCPP_INFO(this->get_logger(), "Zero-Copy Bridge initialized. Publishing via /dev/shm.");
    }
private:
    void publish_heartbeat()
    {
        auto loaned_msg = publisher_->borrow_loaned_message();
        if (loaned_msg.is_valid()) {
            loaned_msg.get().data = this->now().nanoseconds();
            publisher_->publish(std::move(loaned_msg));
        }
    }
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CoreBridgeNode>());
    rclcpp::shutdown();
    return 0;
}
