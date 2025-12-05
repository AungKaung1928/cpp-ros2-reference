// ===== GENERAL C++ =====
#include <iostream>
using namespace std;

void general_example() {
    int a, b, c;
    cout << "Enter three numbers: ";
    cin >> a >> b >> c;
    if(a > b){
        if(a > c){
            cout << a << " is maximum" << endl;
        }else{
            cout << c << " is maximum" << endl;
        }
    }else{
        if(b > c){
            cout << b << " is maximum" << endl;
        }else{
            cout << c << " is maximum" << endl;
        }
    }
}

// ===== ROS2 EXAMPLE =====
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class ObstacleDetectorNode : public rclcpp::Node {
private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;

public:
    ObstacleDetectorNode() : Node("obstacle_detector") {
        sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
                float front = msg->ranges[msg->ranges.size() / 2];
                float left = msg->ranges[msg->ranges.size() / 4];
                float right = msg->ranges[3 * msg->ranges.size() / 4];
                
                if(front > left && front > right){
                    RCLCPP_INFO(this->get_logger(), "Front is clearest: %.2f", front);
                }else if(left > right){
                    RCLCPP_INFO(this->get_logger(), "Left is clearest: %.2f", left);
                }else{
                    RCLCPP_INFO(this->get_logger(), "Right is clearest: %.2f", right);
                }
            });
    }
};

int main(int argc, char **argv) {
    // general_example();  // Skip for ROS2
    
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObstacleDetectorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
