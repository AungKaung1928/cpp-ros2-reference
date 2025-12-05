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
  
  t1.join();  // Wait for threads to finish
  t2.join();
}
// ===== ROS2 EXAMPLE =====
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
class ParallelProcessingNode : public rclcpp::Node {
private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  float sensor_data_[360];
  void processFrontSector(int start, int end, float& result) {
    float sum = 0.0;
    for(int i = start; i < end; i++) {
      sum += sensor_data_[i];
    }
    result = sum / (end - start);
    cout << "Sector [" << start << "-" << end << "] avg: " << result << endl;
  }
public:
  ParallelProcessingNode() : Node("parallel_processing") {
    pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    
    // Simulate sensor data
    for(int i = 0; i < 360; i++) sensor_data_[i] = 1.0 + (i % 10) * 0.1;
    
    float left_avg = 0, front_avg = 0, right_avg = 0;
    
    thread t1(&ParallelProcessingNode::processFrontSector, this, 0, 120, ref(left_avg));
    thread t2(&ParallelProcessingNode::processFrontSector, this, 120, 240, ref(front_avg));
    thread t3(&ParallelProcessingNode::processFrontSector, this, 240, 360, ref(right_avg));
    
    t1.join();  // Wait for all processing
    t2.join();
    t3.join();
    
    auto cmd = geometry_msgs::msg::Twist();
    cmd.linear.x = (front_avg > 0.5) ? 0.3 : 0.0;
    pub_->publish(cmd);
    
    RCLCPP_INFO(this->get_logger(), "Parallel processing complete");
  }
};
int main(int argc, char **argv) {
  // general_example();  // Skip for ROS2
  
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ParallelProcessingNode>();
  rclcpp::shutdown();
  return 0;
}
