// ===== GENERAL C++ =====
#include <iostream>
using namespace std;

void general_example() {
    int a;
    cout << "Enter a number: ";
    cin >> a;
    if(a % 2 == 0){
        cout << a << " is even" << endl;
    }else{
        cout << a << " is odd" << endl;
    }
}

// ===== ROS2 EXAMPLE =====
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

class ParityCheckerNode : public rclcpp::Node {
private:
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;

public:
    ParityCheckerNode() : Node("parity_checker") {
        sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "number", 10, [this](const std_msgs::msg::Int32::SharedPtr msg) {
                auto result = std_msgs::msg::String();
                if(msg->data % 2 == 0){
                    result.data = "even";
                    RCLCPP_INFO(this->get_logger(), "%d is even", msg->data);
                }else{
                    result.data = "odd";
                    RCLCPP_INFO(this->get_logger(), "%d is odd", msg->data);
                }
                pub_->publish(result);
            });
        pub_ = this->create_publisher<std_msgs::msg::String>("parity", 10);
    }
};

int main(int argc, char **argv) {
    // general_example();  // Skip for ROS2
    
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ParityCheckerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
