// ===== GENERAL C++ =====
#include <iostream>
using namespace std;

void general_example() {
    int pocketMoney = 3000;
    for(int date = 1; date <= 30; date++){
        if(date % 2 == 0){
            continue;  // Skip even dates
        }
        if(pocketMoney == 0){
            break;  // Stop when no money
        }
        cout << "Go out on date " << date << endl;
        pocketMoney -= 300;
    }
}

// ===== ROS2 EXAMPLE =====
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "geometry_msgs/msg/twist.hpp"

class BatteryMonitorNode : public rclcpp::Node {
private:
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    float battery_percent_ = 100.0;

public:
    BatteryMonitorNode() : Node("battery_monitor") {
        sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
            "battery", 10, [this](const sensor_msgs::msg::BatteryState::SharedPtr msg) {
                battery_percent_ = msg->percentage * 100.0;
            });
        
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        
        timer_ = this->create_wall_timer(
            chrono::seconds(1), [this]() {
                if(battery_percent_ < 20.0){
                    RCLCPP_WARN(this->get_logger(), "Low battery! Stopping.");
                    return;  // break equivalent - exit callback
                }
                
                if((int)battery_percent_ % 2 == 0){
                    return;  // continue equivalent - skip this cycle
                }
                
                auto cmd = geometry_msgs::msg::Twist();
                cmd.linear.x = 0.2;
                pub_->publish(cmd);
                RCLCPP_INFO(this->get_logger(), "Moving, battery: %.1f%%", battery_percent_);
            });
    }
};

int main(int argc, char **argv) {
    // general_example();  // Skip for ROS2
    
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BatteryMonitorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
