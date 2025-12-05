// ===== GENERAL C++ =====
#include <iostream>
#include <vector>
#include <string>
using namespace std;

void general_example() {
    vector<string> msg {"Hello", "C++", "World", "from", "VS Code", "and the C++ extension!"};
    for (const string& word : msg) {
        cout << word << " ";
    }
    cout << endl;
}

// ===== ROS2 EXAMPLE =====
#include "rclcpp/rclcpp.hpp"

class HelloNode : public rclcpp::Node {
public:
    HelloNode() : Node("hello_node") {
        RCLCPP_INFO(this->get_logger(), "Hello ROS2 from C++!");
    }
};

int main(int argc, char **argv) {
    // General C++
    general_example();
    
    // ROS2
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HelloNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
