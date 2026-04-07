#include <chrono>
#include <map>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

class HiveMindController : public rclcpp::Node {
public:
    HiveMindController() : Node("hive_mind_controller") {
        this->declare_parameter("drone_name", "alpha");
        self_name_ = this->get_parameter("drone_name").as_string();

        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        
        std::vector<std::string> drones = {"alpha", "bravo", "charlie"};
        for (const auto & name : drones) {
            if (name == self_name_) {
                sub_self_ = this->create_subscription<nav_msgs::msg::Odometry>(
                    "/" + name + "/odom", 10, [this](nav_msgs::msg::Odometry::SharedPtr msg){ self_odom_ = msg; });
            } else {
                subs_neighbors_[name] = this->create_subscription<nav_msgs::msg::Odometry>(
                    "/" + name + "/odom", 10, [this, name](nav_msgs::msg::Odometry::SharedPtr msg){ neighbor_odom_[name] = msg; });
            }
        }

        timer_ = this->create_wall_timer(50ms, std::bind(&HiveMindController::calculate_swarm_math, this));
    }

private:
    void calculate_swarm_math() {
        if (!self_odom_) return;

        auto cmd = geometry_msgs::msg::Twist();
        double sep_x = 0, sep_y = 0;

        for (auto const& [name, odom] : neighbor_odom_) {
            if (!odom) continue; // THE FIX: Pro-level memory safety

            double dx = self_odom_->pose.pose.position.x - odom->pose.pose.position.x;
            double dy = self_odom_->pose.pose.position.y - odom->pose.pose.position.y;
            double dist_sq = dx*dx + dy*dy;

            // THE FIX: Expanded sensory radius to 5.0 (approx 2.23m) so spawn positions trigger it
            if (dist_sq < 5.0 && dist_sq > 0.01) { 
                sep_x += (dx / dist_sq) * 1.5; 
                sep_y += (dy / dist_sq) * 1.5;
            }
        }

        cmd.linear.x = sep_x;
        cmd.linear.y = sep_y;
        pub_->publish(cmd);
    }

    std::string self_name_;
    nav_msgs::msg::Odometry::SharedPtr self_odom_;
    std::map<std::string, nav_msgs::msg::Odometry::SharedPtr> neighbor_odom_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_self_;
    std::map<std::string, rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> subs_neighbors_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HiveMindController>());
    rclcpp::shutdown();
    return 0;
}
