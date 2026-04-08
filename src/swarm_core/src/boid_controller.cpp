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

        // EPIC 5: The Flight Plan (Stratford Route)
        waypoints_ = {
            {22.0, 0.0},   // WP1: Approach the corner
            {22.0, 15.0},  // WP2: Bank Left into the intersection
            {-5.0, 15.0}   // WP3: Fly out the West exit
        };
        current_wp_idx_ = 0;

        timer_ = this->create_wall_timer(50ms, std::bind(&HiveMindController::calculate_swarm_math, this));
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        latest_scan_ = msg;
    }

    void calculate_swarm_math() {
        if (!self_odom_) return;

        double sep_x = 0, sep_y = 0;
        double avg_vel_x = 0, avg_vel_y = 0;
        double center_x = 0, center_y = 0;
        int flock_count = 0;

        for (auto const& [name, odom] : neighbor_odom_) {
            if (!odom) continue;
            double dx = self_odom_->pose.pose.position.x - odom->pose.pose.position.x;
            double dy = self_odom_->pose.pose.position.y - odom->pose.pose.position.y;
            double dist_sq = dx*dx + dy*dy;

            if (dist_sq < 25.0 && dist_sq > 0.01) {
                if (dist_sq < 4.0) { sep_x += (dx / dist_sq); sep_y += (dy / dist_sq); }
                avg_vel_x += odom->twist.twist.linear.x; avg_vel_y += odom->twist.twist.linear.y;
                center_x += odom->pose.pose.position.x; center_y += odom->pose.pose.position.y;
                flock_count++;
            }
        }

        double align_x = 0, align_y = 0, coh_x = 0, coh_y = 0;
        if (flock_count > 0) {
            avg_vel_x /= flock_count; avg_vel_y /= flock_count;
            align_x = avg_vel_x - self_odom_->twist.twist.linear.x;
            align_y = avg_vel_y - self_odom_->twist.twist.linear.y;
            center_x /= flock_count; center_y /= flock_count;
            coh_x = center_x - self_odom_->pose.pose.position.x;
            coh_y = center_y - self_odom_->pose.pose.position.y;
        }

        double obs_x = 0, obs_y = 0;
        if (latest_scan_) {
            if (filtered_ranges_.empty()) filtered_ranges_.resize(latest_scan_->ranges.size(), 20.0);
            double alpha = 0.2; 
            for (size_t i = 0; i < latest_scan_->ranges.size(); ++i) {
                double raw_range = latest_scan_->ranges[i];
                if (std::isinf(raw_range) || raw_range > latest_scan_->range_max) raw_range = 20.0; 
                filtered_ranges_[i] = (alpha * raw_range) + ((1.0 - alpha) * filtered_ranges_[i]);

                if (filtered_ranges_[i] > latest_scan_->range_min && filtered_ranges_[i] < 10.0) { 
                    double angle = latest_scan_->angle_min + i * latest_scan_->angle_increment;
                    obs_x -= (1.0 / (filtered_ranges_[i] * filtered_ranges_[i])) * std::cos(angle);
                    obs_y -= (1.0 / (filtered_ranges_[i] * filtered_ranges_[i])) * std::sin(angle);
                }
            }
        }

        // EPIC 5: Dynamic Navigation Vector
        double mig_x = 0.0, mig_y = 0.0;
        if (current_wp_idx_ < waypoints_.size()) {
            double target_x = waypoints_[current_wp_idx_].first;
            double target_y = waypoints_[current_wp_idx_].second;
            
            double dx = target_x - self_odom_->pose.pose.position.x;
            double dy = target_y - self_odom_->pose.pose.position.y;
            double dist_to_wp = std::sqrt(dx*dx + dy*dy);
            
            // If within 2 meters of waypoint, switch to next waypoint
            if (dist_to_wp < 2.0) {
                current_wp_idx_++;
                RCLCPP_INFO(this->get_logger(), "Waypoint reached! Proceeding to next.");
            } else {
                // Normalize the vector so they don't fly at Mach 10
                mig_x = dx / dist_to_wp; 
                mig_y = dy / dist_to_wp;
            }
        } else {
            // Mission Complete: Hover in place
            mig_x = 0.0; mig_y = 0.0;
        }

        double w_sep = 5.0, w_ali = 1.0, w_coh = 0.25, w_mig = 2.0, w_obs = 5.0;

        auto cmd = geometry_msgs::msg::Twist();
        cmd.linear.x = (sep_x * w_sep) + (align_x * w_ali) + (coh_x * w_coh) + (mig_x * w_mig) + (obs_x * w_obs);
        cmd.linear.y = (sep_y * w_sep) + (align_y * w_ali) + (coh_y * w_coh) + (mig_y * w_mig) + (obs_y * w_obs);

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
    std::vector<double> filtered_ranges_;
    std::map<std::string, nav_msgs::msg::Odometry::SharedPtr> neighbor_odom_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_self_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
    std::map<std::string, rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> subs_neighbors_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Waypoint Variables
    std::vector<std::pair<double, double>> waypoints_;
    size_t current_wp_idx_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HiveMindController>());
    rclcpp::shutdown();
    return 0;
}
