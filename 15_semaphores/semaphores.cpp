// ===== GENERAL C++ =====
#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>
using namespace std;

mutex m;
condition_variable cv;
bool ready = false;

void print_id(int id){
    unique_lock<mutex> lck(m);
    while(!ready) cv.wait(lck);
    cout << "thread " << id << endl;
}

void go(){
    unique_lock<mutex> lck(m);
    ready = true;
    cv.notify_all();
}

void general_example() {
    thread threads[10];
    for(int i = 0; i < 10; ++i){
        threads[i] = thread(print_id, i);
    }
    cout << "10 threads ready" << endl;
    go();
    
    for(int i = 0; i < 10; ++i){
        threads[i].join();
    }
}

// ===== ROS2 EXAMPLE =====
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"

class SynchronizedRobotNode : public rclcpp::Node {
private:
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    mutex robot_mutex_;
    condition_variable robot_cv_;
    bool navigation_ready_ = false;

    void executorThread(int robot_id) {
        unique_lock<mutex> lck(robot_mutex_);
        while(!navigation_ready_) robot_cv_.wait(lck);
        
        auto cmd = geometry_msgs::msg::Twist();
        cmd.linear.x = 0.2 * robot_id;
        pub_->publish(cmd);
        RCLCPP_INFO(this->get_logger(), "Robot %d executing navigation", robot_id);
    }

    void startNavigation() {
        unique_lock<mutex> lck(robot_mutex_);
        navigation_ready_ = true;
        robot_cv_.notify_all();
        RCLCPP_INFO(this->get_logger(), "All robots start navigation");
    }

public:
    SynchronizedRobotNode() : Node("synchronized_robot") {
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "start_signal", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) {
                if(msg->data) startNavigation();
            });
        
        thread workers[5];
        for(int i = 0; i < 5; ++i){
            workers[i] = thread(&SynchronizedRobotNode::executorThread, this, i);
        }
        
        RCLCPP_INFO(this->get_logger(), "5 robot threads waiting for start signal");
        
        this_thread::sleep_for(chrono::seconds(2));
        startNavigation();
        
        for(int i = 0; i < 5; ++i){
            workers[i].join();
        }
    }
};

int main(int argc, char **argv) {
    // general_example();  // Skip for ROS2
    
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SynchronizedRobotNode>();
    rclcpp::shutdown();
    return 0;
}
