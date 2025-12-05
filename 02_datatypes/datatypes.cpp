// ===== GENERAL C++ =====
#include <iostream>
using namespace std;

void general_example() {
    int a = 12;
    cout << "Size of int: " << sizeof(a) << endl;
    float b = 12.5;
    cout << "Size of float: " << sizeof(b) << endl;
    char c = 'A';
    cout << "Size of char: " << sizeof(c) << endl;
    bool d = true;
    cout << "Size of bool: " << sizeof(d) << endl;
    short int si = 12;
    cout << "Size of short int: " << sizeof(si) << endl;
    long int li = 12;
    cout << "Size of long int: " << sizeof(li) << endl;
}

// ===== ROS2 EXAMPLE =====
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"

class DataTypesNode : public rclcpp::Node {
public:
    DataTypesNode() : Node("datatypes_node") {
        int32_t speed = 100;
        double distance = 5.75;
        bool is_active = true;
        
        RCLCPP_INFO(this->get_logger(), "Speed (int32): %d", speed);
        RCLCPP_INFO(this->get_logger(), "Distance (double): %.2f", distance);
        RCLCPP_INFO(this->get_logger(), "Active (bool): %s", is_active ? "true" : "false");
    }
};

int main(int argc, char **argv) {
    general_example();
    
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DataTypesNode>();
    rclcpp::shutdown();
    return 0;
}
