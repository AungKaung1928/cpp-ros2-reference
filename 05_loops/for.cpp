// ===== GENERAL C++ =====
#include <iostream>
using namespace std;

void general_example() {
    int n;
    cout << "Enter a number: ";
    cin >> n;
    for(int i = 1; i <= 10; i++){
        cout << n << " x " << i << " = " << n * i << endl;
    }
}

// ===== ROS2 EXAMPLE =====
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class PeriodicPublisherNode : public rclcpp::Node {
private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    int count_ = 0;

public:
    PeriodicPublisherNode() : Node("periodic_publisher") {
        pub_ = this->create_publisher<std_msgs::msg::String>("status", 10);
        timer_ = this->create_wall_timer(
            chrono::seconds(1), [this]() {
                auto msg = std_msgs::msg::String();
                msg.data = "Heartbeat #" + to_string(count_);
                pub_->publish(msg);
                RCLCPP_INFO(this->get_logger(), "Published: %s", msg.data.c_str());
                count_++;
                if(count_ >= 10) count_ = 0;  // Reset after 10
            });
    }
};

int main(int argc, char **argv) {
    // general_example();  // Skip for ROS2
    
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PeriodicPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
