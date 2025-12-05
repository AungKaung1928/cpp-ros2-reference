// ===== GENERAL C++ =====
#include <iostream>
#include <thread>
#include <chrono>
using namespace std;

void threadFunction(int threadId) {
    cout << "Thread " << threadId << " is running." << endl;
    this_thread::sleep_for(chrono::seconds(1));
    cout << "Thread " << threadId << " finished." << endl;
}

void general_example() {
    thread t1(threadFunction, 1);
    
    thread t2([](int threadId) {
        cout << "Thread " << threadId << " is running." << endl;
        this_thread::sleep_for(chrono::seconds(1));
        cout << "Thread " << threadId << " finished." << endl;
    }, 2);
    
    t1.detach();
    t2.detach();
    
    this_thread::sleep_for(chrono::seconds(2));  // Wait for detached threads
}

// ===== ROS2 EXAMPLE =====
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

class MultiThreadRobotNode : public rclcpp::Node {
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    bool running_ = true;

    void sensorProcessingThread() {
        while(running_) {
            RCLCPP_INFO(this->get_logger(), "Processing sensor data...");
            this_thread::sleep_for(chrono::milliseconds(500));
        }
    }

    void controlThread() {
        while(running_) {
            auto cmd = geometry_msgs::msg::Twist();
            cmd.linear.x = 0.2;
            pub_->publish(cmd);
            RCLCPP_INFO(this->get_logger(), "Publishing control command");
            this_thread::sleep_for(chrono::milliseconds(100));
        }
    }

public:
    MultiThreadRobotNode() : Node("multithread_robot") {
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        
        thread t1(&MultiThreadRobotNode::sensorProcessingThread, this);
        thread t2(&MultiThreadRobotNode::controlThread, this);
        
        t1.detach();
        t2.detach();
        
        RCLCPP_INFO(this->get_logger(), "Threads detached and running");
    }
    
    ~MultiThreadRobotNode() {
        running_ = false;
        this_thread::sleep_for(chrono::seconds(1));
    }
};

int main(int argc, char **argv) {
    // general_example();  // Skip for ROS2
    
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MultiThreadRobotNode>();
    this_thread::sleep_for(chrono::seconds(3));
    rclcpp::shutdown();
    return 0;
}
