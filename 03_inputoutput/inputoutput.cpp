// ===== GENERAL C++ =====
#include <iostream>
using namespace std;

void general_example() {
    int amount1, amount2;
    cout << "Enter two numbers: ";
    cin >> amount1 >> amount2;
    int sum = amount1 + amount2;
    cout << "Sum is: " << sum << endl;
}

// ===== ROS2 EXAMPLE =====
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

class AdderNode : public rclcpp::Node {
private:
    int value1_ = 0;
    int value2_ = 0;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub1_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub2_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;

public:
    AdderNode() : Node("adder_node") {
        sub1_ = this->create_subscription<std_msgs::msg::Int32>(
            "input1", 10, [this](const std_msgs::msg::Int32::SharedPtr msg) {
                value1_ = msg->data;
                publish_sum();
            });
        sub2_ = this->create_subscription<std_msgs::msg::Int32>(
            "input2", 10, [this](const std_msgs::msg::Int32::SharedPtr msg) {
                value2_ = msg->data;
                publish_sum();
            });
        pub_ = this->create_publisher<std_msgs::msg::Int32>("sum", 10);
    }

    void publish_sum() {
        auto msg = std_msgs::msg::Int32();
        msg.data = value1_ + value2_;
        pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Sum: %d", msg.data);
    }
};

int main(int argc, char **argv) {
    // general_example();  // Skip for ROS2 non-blocking
    
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AdderNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
