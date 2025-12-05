// ===== GENERAL C++ =====
#include <iostream>
using namespace std;

void general_example() {
    int n;
    cout << "Enter a number: ";
    cin >> n;
    while(n != 1){
        cout << n << " ";
        if(n % 2 == 0){
            n = n / 2;
        }else{
            n = 3 * n + 1;
        }
    }
    cout << n << endl;
}

// ===== ROS2 EXAMPLE =====
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class WallFollowerNode : public rclcpp::Node {
private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;

public:
    WallFollowerNode() : Node("wall_follower") {
        sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
                auto cmd = geometry_msgs::msg::Twist();
                float front = msg->ranges[msg->ranges.size() / 2];
                
                while(front > 0.5){  // Keep moving while distance > 0.5m
                    cmd.linear.x = 0.2;
                    cmd.angular.z = 0.0;
                    pub_->publish(cmd);
                    RCLCPP_INFO(this->get_logger(), "Moving forward, dist: %.2f", front);
                    break;  // Exit after one iteration (async loop via callback)
                }
                
                if(front <= 0.5){  // Stop when close
                    cmd.linear.x = 0.0;
                    pub_->publish(cmd);
                    RCLCPP_INFO(this->get_logger(), "Stopped, obstacle detected");
                }
            });
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    }
};

int main(int argc, char **argv) {
    // general_example();  // Skip for ROS2
    
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WallFollowerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
