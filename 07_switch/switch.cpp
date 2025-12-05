// ===== GENERAL C++ =====
#include <iostream>
using namespace std;

void general_example() {
    int language;
    cout << "Enter a number for language: ";
    cin >> language;
    switch(language){
        case 1:
            cout << "Hello World" << endl;
            break;
        case 2:
            cout << "Hola Mundo" << endl;
            break;
        case 3:
            cout << "Bonjour le monde" << endl;
            break;
        case 4:
            cout << "Hallo Welt" << endl;
            break;
        case 5:
            cout << "Ciao mondo" << endl;
            break;
        default:
            cout << "Language not supported" << endl;
    }
}

// ===== ROS2 EXAMPLE =====
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

class RobotModeNode : public rclcpp::Node {
private:
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;

public:
    RobotModeNode() : Node("robot_mode") {
        sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "mode", 10, [this](const std_msgs::msg::Int32::SharedPtr msg) {
                auto status = std_msgs::msg::String();
                
                switch(msg->data){
                    case 1:
                        status.data = "IDLE";
                        RCLCPP_INFO(this->get_logger(), "Mode: IDLE");
                        break;
                    case 2:
                        status.data = "MAPPING";
                        RCLCPP_INFO(this->get_logger(), "Mode: MAPPING");
                        break;
                    case 3:
                        status.data = "NAVIGATION";
                        RCLCPP_INFO(this->get_logger(), "Mode: NAVIGATION");
                        break;
                    case 4:
                        status.data = "DOCKING";
                        RCLCPP_INFO(this->get_logger(), "Mode: DOCKING");
                        break;
                    case 5:
                        status.data = "EMERGENCY_STOP";
                        RCLCPP_WARN(this->get_logger(), "Mode: EMERGENCY_STOP");
                        break;
                    default:
                        status.data = "UNKNOWN";
                        RCLCPP_ERROR(this->get_logger(), "Invalid mode");
                }
                pub_->publish(status);
            });
        pub_ = this->create_publisher<std_msgs::msg::String>("status", 10);
    }
};

int main(int argc, char **argv) {
    // general_example();  // Skip for ROS2
    
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotModeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
