// ===== GENERAL C++ =====
#include <iostream>
#include <thread>
#include <mutex>
using namespace std;
int myAmount = 0;
mutex m;
void addMoney(){
  m.lock();
  ++myAmount;
  m.unlock();
}
void general_example() {
  thread t1(addMoney);
  thread t2(addMoney);
  t1.join();
  t2.join();
  cout << "Amount: " << myAmount << endl;
}
// ===== ROS2 EXAMPLE =====
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int32.hpp"
class SafeCounterNode : public rclcpp::Node {
private:
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub1_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub2_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
  mutex counter_mutex_;
  int shared_counter_ = 0;
  void callback1(const std_msgs::msg::Int32::SharedPtr msg) {
    counter_mutex_.lock();
    shared_counter_ += msg->data;
    RCLCPP_INFO(this->get_logger(), "Callback1: Counter = %d", shared_counter_);
    counter_mutex_.unlock();
    publishCounter();
  }
  void callback2(const std_msgs::msg::Int32::SharedPtr msg) {
    counter_mutex_.lock();
    shared_counter_ += msg->data;
    RCLCPP_INFO(this->get_logger(), "Callback2: Counter = %d", shared_counter_);
    counter_mutex_.unlock();
    publishCounter();
  }
  void publishCounter() {
    auto msg = std_msgs::msg::Int32();
    counter_mutex_.lock();
    msg.data = shared_counter_;
    counter_mutex_.unlock();
    pub_->publish(msg);
  }
public:
  SafeCounterNode() : Node("safe_counter") {
    sub1_ = this->create_subscription<std_msgs::msg::Int32>(
      "input1", 10, bind(&SafeCounterNode::callback1, this, placeholders::_1));
    sub2_ = this->create_subscription<std_msgs::msg::Int32>(
      "input2", 10, bind(&SafeCounterNode::callback2, this, placeholders::_1));
    pub_ = this->create_publisher<std_msgs::msg::Int32>("total", 10);
    
    RCLCPP_INFO(this->get_logger(), "SafeCounter with mutex protection running");
  }
};
int main(int argc, char **argv) {
  // general_example();  // Skip for ROS2
  
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SafeCounterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
