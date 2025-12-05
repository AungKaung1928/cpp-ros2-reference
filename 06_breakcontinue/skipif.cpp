// ===== GENERAL C++ =====
#include <iostream>
using namespace std;

void general_example() {
    for(int i = 1; i <= 100; i++){
        if(i % 3 == 0){
            continue;  // Skip numbers divisible by 3
        }
        cout << i << endl;
    }
}

// ===== ROS2 EXAMPLE =====
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

class SelectiveScanNode : public rclcpp::Node {
private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;

public:
    SelectiveScanNode() : Node("selective_scan") {
        sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
                float min_dist = 999.0;
                
                for(size_t i = 0; i < msg->ranges.size(); i++){
                    if(i % 3 == 0){
                        continue;  // Skip every 3rd ray to reduce processing
                    }
                    if(msg->ranges[i] < min_dist){
                        min_dist = msg->ranges[i];
                    }
                }
                
                auto cmd = geometry_msgs::msg::Twist();
                if(min_dist < 0.5){
                    cmd.linear.x = 0.0;
                    RCLCPP_WARN(this->get_logger(), "Obstacle: %.2fm", min_dist);
                }else{
                    cmd.linear.x = 0.3;
                    RCLCPP_INFO(this->get_logger(), "Clear: %.2fm", min_dist);
                }
                pub_->publish(cmd);
            });
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    }
};

int main(int argc, char **argv) {
    // general_example();  // Skip for ROS2
    
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SelectiveScanNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
