#include <chrono>
#include <map>
#include <string>
#include <vector>
#include <cmath>
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

        double sep_x = 0, sep_y = 0;
        double avg_vel_x = 0, avg_vel_y = 0;
        double center_x = 0, center_y = 0;
        int flock_count = 0;

        // 1. Radar Sweep
        for (auto const& [name, odom] : neighbor_odom_) {
            if (!odom) continue;

            double dx = self_odom_->pose.pose.position.x - odom->pose.pose.position.x;
            double dy = self_odom_->pose.pose.position.y - odom->pose.pose.position.y;
            double dist_sq = dx*dx + dy*dy;

            if (dist_sq < 25.0 && dist_sq > 0.01) { // 5.0m perception radius
                
                // Rule 1: Separation (Inverse Square - tight radius)
                if (dist_sq < 4.0) { 
                    sep_x += (dx / dist_sq); 
                    sep_y += (dy / dist_sq);
                }

                // Gather data for Alignment and Cohesion
                avg_vel_x += odom->twist.twist.linear.x;
                avg_vel_y += odom->twist.twist.linear.y;
                center_x += odom->pose.pose.position.x;
                center_y += odom->pose.pose.position.y;
                flock_count++;
            }
        }

        double align_x = 0, align_y = 0;
        double coh_x = 0, coh_y = 0;

        if (flock_count > 0) {
            // Rule 2: Alignment (Match Velocity)
            avg_vel_x /= flock_count;
            avg_vel_y /= flock_count;
            align_x = avg_vel_x - self_odom_->twist.twist.linear.x;
            align_y = avg_vel_y - self_odom_->twist.twist.linear.y;

            // Rule 3: Cohesion (Steer to Center of Mass)
            center_x /= flock_count;
            center_y /= flock_count;
            coh_x = center_x - self_odom_->pose.pose.position.x;
            coh_y = center_y - self_odom_->pose.pose.position.y;
        }

        // Rule 4: The Patrol Mission (Fly East)
        double mig_x = 1.0; 
        double mig_y = 0.0;

        // The Defense-Grade Weight Multipliers
        double w_sep = 2.5; 
        double w_ali = 1.0; 
        double w_coh = 0.5; 
        double w_mig = 1.5;

        auto cmd = geometry_msgs::msg::Twist();
        cmd.linear.x = (sep_x * w_sep) + (align_x * w_ali) + (coh_x * w_coh) + (mig_x * w_mig);
        cmd.linear.y = (sep_y * w_sep) + (align_y * w_ali) + (coh_y * w_coh) + (mig_y * w_mig);

        // The Governor: Clamp to physical limits
        double max_speed = 2.0;
        double speed = std::sqrt(cmd.linear.x*cmd.linear.x + cmd.linear.y*cmd.linear.y);
        if (speed > max_speed) {
            cmd.linear.x = (cmd.linear.x / speed) * max_speed;
            cmd.linear.y = (cmd.linear.y / speed) * max_speed;
        }

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
