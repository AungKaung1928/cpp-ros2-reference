// ===== GENERAL C++ =====
#include <iostream>
using namespace std;

int add(int a, int b){
    return a + b;
}

void general_example() {
    int a, b;
    cout << "Enter two numbers: ";
    cin >> a >> b;
    cout << "Sum is: " << add(a, b) << endl;
}

// ===== ROS2 EXAMPLE =====
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class ObstacleAvoidanceNode : public rclcpp::Node {
private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;

    float calculateSpeed(float distance) {
        if(distance < 0.5) return 0.0;
        if(distance < 1.0) return 0.1;
        return 0.3;
    }

    float calculateTurn(float left, float right) {
        return (left - right) * 0.5;
    }

public:
    ObstacleAvoidanceNode() : Node("obstacle_avoidance") {
        sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
                float front = msg->ranges[msg->ranges.size() / 2];
                float left = msg->ranges[msg->ranges.size() / 4];
                float right = msg->ranges[3 * msg->ranges.size() / 4];
                
                auto cmd = geometry_msgs::msg::Twist();
                cmd.linear.x = calculateSpeed(front);
                cmd.angular.z = calculateTurn(left, right);
                
                pub_->publish(cmd);
                RCLCPP_INFO(this->get_logger(), "Speed: %.2f, Turn: %.2f", 
                    cmd.linear.x, cmd.angular.z);
            });
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    }
};

int main(int argc, char **argv) {
    // general_example();  // Skip for ROS2
    
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObstacleAvoidanceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
