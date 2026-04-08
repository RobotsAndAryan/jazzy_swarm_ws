#include <chrono>
#include <map>
#include <string>
#include <vector>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class HiveMindController : public rclcpp::Node {
public:
    HiveMindController() : Node("hive_mind_controller") {
        this->declare_parameter("drone_name", "alpha");
        self_name_ = this->get_parameter("drone_name").as_string();

        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        sub_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&HiveMindController::scan_callback, this, _1));

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

        waypoints_ = {
            {18.0, 0.0}, {22.0, 0.0}, {22.0, 3.0}, {22.0, 8.0}, {22.0, 15.0}
        };
        current_wp_idx_ = 0;

        timer_ = this->create_wall_timer(50ms, std::bind(&HiveMindController::calculate_vfh_math, this));
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        latest_scan_ = msg;
    }

    void calculate_vfh_math() {
        if (!self_odom_ || !latest_scan_) return;

        // 1. Separation (Don't crash into each other)
        double sep_x = 0, sep_y = 0;
        for (auto const& [name, odom] : neighbor_odom_) {
            if (!odom) continue;
            double dx = self_odom_->pose.pose.position.x - odom->pose.pose.position.x;
            double dy = self_odom_->pose.pose.position.y - odom->pose.pose.position.y;
            double dist_sq = dx*dx + dy*dy;
            if (dist_sq < 4.0 && dist_sq > 0.01) { 
                sep_x += (dx / dist_sq); sep_y += (dy / dist_sq); 
            }
        }

        // 2. VFH Histogram Setup (36 bins, 10 degrees each)
        int num_bins = 36;
        std::vector<bool> blocked_bins(num_bins, false);
        double safe_distance = 1.2; // Avoid walls within 2.5m

        for (size_t i = 0; i < latest_scan_->ranges.size(); ++i) {
            double r = latest_scan_->ranges[i];
            if (std::isinf(r) || r > latest_scan_->range_max) r = 20.0;

            if (r < safe_distance) {
                double angle = latest_scan_->angle_min + i * latest_scan_->angle_increment;
                // Normalize angle to 0 - 2PI
                while (angle < 0) angle += 2 * M_PI;
                while (angle >= 2 * M_PI) angle -= 2 * M_PI;
                
                int bin_idx = static_cast<int>((angle / (2 * M_PI)) * num_bins) % num_bins;
                blocked_bins[bin_idx] = true;
            }
        }

        // 3. Waypoint Targeting
        double mig_x = 0.0, mig_y = 0.0;
        if (current_wp_idx_ < waypoints_.size()) {
            double target_x = waypoints_[current_wp_idx_].first;
            double target_y = waypoints_[current_wp_idx_].second;
            
            double dx = target_x - self_odom_->pose.pose.position.x;
            double dy = target_y - self_odom_->pose.pose.position.y;
            double dist_to_wp = std::sqrt(dx*dx + dy*dy);
            
            if (dist_to_wp < 2.0) {
                current_wp_idx_++;
                RCLCPP_INFO(this->get_logger(), "VFH Gate Cleared!");
            } else {
                // Find Target Angle
                double target_angle = std::atan2(dy, dx);
                while (target_angle < 0) target_angle += 2 * M_PI;
                int target_bin = static_cast<int>((target_angle / (2 * M_PI)) * num_bins) % num_bins;

                // 4. VFH Gap Selection
                int chosen_bin = target_bin;
                int search_radius = 0;
                
                // If blocked, search left and right for nearest open gap
                while (blocked_bins[chosen_bin] && search_radius < num_bins / 2) {
                    search_radius++;
                    int right_bin = (target_bin + search_radius) % num_bins;
                    int left_bin = (target_bin - search_radius + num_bins) % num_bins;
                    
                    if (!blocked_bins[left_bin]) { chosen_bin = left_bin; break; }
                    if (!blocked_bins[right_bin]) { chosen_bin = right_bin; break; }
                }

                // Convert chosen bin back to a vector
                double chosen_angle = (chosen_bin * (2 * M_PI) / num_bins) + ((M_PI) / num_bins);
                mig_x = std::cos(chosen_angle);
                mig_y = std::sin(chosen_angle);
            }
        }

        // 5. Final Velocity Command
        double w_sep = 3.0, w_mig = 2.5;
        auto cmd = geometry_msgs::msg::Twist();
        cmd.linear.x = (sep_x * w_sep) + (mig_x * w_mig);
        cmd.linear.y = (sep_y * w_sep) + (mig_y * w_mig);

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
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
    std::map<std::string, nav_msgs::msg::Odometry::SharedPtr> neighbor_odom_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_self_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
    std::map<std::string, rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> subs_neighbors_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<std::pair<double, double>> waypoints_;
    size_t current_wp_idx_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HiveMindController>());
    rclcpp::shutdown();
    return 0;
}
