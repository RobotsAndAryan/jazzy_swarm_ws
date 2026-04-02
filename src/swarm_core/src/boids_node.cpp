#include <chrono>
#include <memory>
#include <vector>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp" // Placeholder for actual telemetry

using namespace std::chrono_literals;

struct BoidData {
    std::string id;
    double x, y, vx, vy;
};

class SwarmIntelligenceNode : public rclcpp::Node {
public:
    SwarmIntelligenceNode() : Node("boids_intelligence") {
        // In a real swarm, you'd subscribe to an array of all telemetry or a shared Fast DDS topic
        // For this architecture, we listen to the network and calculate the vectors
        
        calc_timer_ = this->create_wall_timer(33ms, std::bind(&SwarmIntelligenceNode::compute_boids, this));
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        RCLCPP_INFO(this->get_logger(), "Decentralized Boids Intelligence Online.");
    }

private:
    std::vector<BoidData> neighbors_; // Updated via callbacks from the Discovery Server
    double my_x_ = 0.0, my_y_ = 0.0, my_vx_ = 0.0, my_vy_ = 0.0;
    
    // Tuning Weights
    double w_sep_ = 1.5;
    double w_ali_ = 1.0;
    double w_coh_ = 1.0;
    double visual_range_ = 5.0; // meters

    void compute_boids() {
        double sep_x = 0, sep_y = 0;
        double ali_x = 0, ali_y = 0;
        double coh_x = 0, coh_y = 0;
        int count = 0;

        for (const auto& neighbor : neighbors_) {
            double dx = my_x_ - neighbor.x;
            double dy = my_y_ - neighbor.y;
            double distance = std::sqrt(dx*dx + dy*dy);

            if (distance > 0 && distance < visual_range_) {
                // 1. Separation
                sep_x += (dx / distance) / distance;
                sep_y += (dy / distance) / distance;
                
                // 2. Alignment
                ali_x += neighbor.vx;
                ali_y += neighbor.vy;
                
                // 3. Cohesion
                coh_x += neighbor.x;
                coh_y += neighbor.y;
                
                count++;
            }
        }

        if (count > 0) {
            ali_x /= count; ali_y /= count;
            coh_x = (coh_x / count) - my_x_; coh_y = (coh_y / count) - my_y_;
        }

        // Master Equation Output (Simplified mapping to Twist)
        auto cmd = geometry_msgs::msg::Twist();
        cmd.linear.x = (w_ali_ * ali_x) + (w_coh_ * coh_x);
        cmd.angular.z = (w_sep_ * sep_y); // Evasive steering based on separation Y-axis

        cmd_pub_->publish(cmd);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr calc_timer_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SwarmIntelligenceNode>());
    rclcpp::shutdown();
    return 0;
}
