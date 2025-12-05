// ===== GENERAL C++ =====
#include <iostream>
using namespace std;

void general_example() {
    cout<<"-------------------\n";
    cout<<"Pointers\n";
    int a = 10;
    int *aptr;
    aptr = &a;
    cout<<&a<<endl; // Address of a
    cout<<aptr<<endl; // Address of a
    cout<<*aptr<<endl; // Dereferencing
    cout<<"-------------------\n";
    cout<<"Pointers\n";
    int b = 20;
    int *bptr = &b;
    cout<<*bptr<<endl; // Dereferencing
    *bptr = 30;
    cout<<b<<endl; // Value of b
    cout<<"-------------------\n";
    cout<<"Pointer arithmetic\n";
    int arr[] = {10, 20, 30};
    int *arrptr = arr;
    cout<<*arrptr<<endl; // 10
    cout<<*(arrptr + 1)<<endl; // 20
    cout<<*(arrptr + 2)<<endl; // 30
    cout<<"-------------------\n";
    cout<<"Pointer to Pointer\n";
    int **pptr = &bptr;
    cout<<**pptr<<endl; // 30
}

// ===== ROS2 EXAMPLE =====
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class LaserProcessorNode : public rclcpp::Node {
private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;

    void processRanges(const float* ranges, size_t size) {
        float min = *(ranges);  // First element
        float mid = *(ranges + size/2);  // Middle element
        float max = *(ranges + size - 1);  // Last element
        
        RCLCPP_INFO(this->get_logger(), "Front: %.2f, Mid: %.2f, Back: %.2f", 
            min, mid, max);
    }

public:
    LaserProcessorNode() : Node("laser_processor") {
        sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
                const float* data_ptr = msg->ranges.data();
                processRanges(data_ptr, msg->ranges.size());
                
                // Direct pointer arithmetic
                RCLCPP_INFO(this->get_logger(), "First 3 ranges: %.2f %.2f %.2f",
                    *data_ptr, *(data_ptr+1), *(data_ptr+2));
            });
    }
};

int main(int argc, char **argv) {
    // general_example();  // Skip for ROS2
    
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaserProcessorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
