// ===== GENERAL C++ =====
#include <iostream>
#include <vector>
#include <algorithm>
using namespace std;
void general_example() {
  // Basic lambda
  auto greet = []() {
    cout << "Hello from lambda!" << endl;
  };
  greet();
  
  // Lambda with parameters
  auto add = [](int a, int b) {
    return a + b;
  };
  cout << "Sum: " << add(5, 3) << endl;
  
  // Lambda with capture by value
  int x = 10;
  auto captureByValue = [x]() {
    cout << "Captured x: " << x << endl;
  };
  captureByValue();
  
  // Lambda with capture by reference
  int y = 20;
  auto captureByRef = [&y]() {
    y += 5;
    cout << "Modified y: " << y << endl;
  };
  captureByRef();
  cout << "y after lambda: " << y << endl;
  
  // Lambda with vector
  vector<int> nums = {5, 2, 8, 1, 9};
  sort(nums.begin(), nums.end(), [](int a, int b) {
    return a > b;  // Descending order
  });
  
  cout << "Sorted: ";
  for(int n : nums) cout << n << " ";
  cout << endl;
}
// ===== ROS2 EXAMPLE =====
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/int32.hpp"
class LambdaRobotNode : public rclcpp::Node {
private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mode_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  int mode_ = 0;
public:
  LambdaRobotNode() : Node("lambda_robot") {
    pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    
    // Lambda in subscription callback
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10, [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        float front = msg->ranges[msg->ranges.size() / 2];
        
        auto cmd = geometry_msgs::msg::Twist();
        if(front < 0.5) {
          cmd.linear.x = 0.0;
          cmd.angular.z = 0.5;
        } else {
          cmd.linear.x = 0.3;
        }
        pub_->publish(cmd);
        RCLCPP_INFO(this->get_logger(), "Distance: %.2f", front);
      });
    
    // Lambda capturing member variable
    mode_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "mode", 10, [this](const std_msgs::msg::Int32::SharedPtr msg) {
        mode_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "Mode changed to: %d", mode_);
      });
    
    // Lambda in timer
    timer_ = this->create_wall_timer(
      chrono::seconds(1), [this]() {
        RCLCPP_INFO(this->get_logger(), "Heartbeat - Current mode: %d", mode_);
      });
    
    // Using lambda for filtering sensor data
    auto filter_ranges = [](const vector<float>& ranges) {
      vector<float> filtered;
      copy_if(ranges.begin(), ranges.end(), back_inserter(filtered),
        [](float r) { return r > 0.1 && r < 10.0; });
      return filtered;
    };
    
    vector<float> test_data = {0.05, 1.2, 15.0, 3.5, 0.0};
    auto valid = filter_ranges(test_data);
    RCLCPP_INFO(this->get_logger(), "Filtered %zu valid ranges", valid.size());
  }
};
int main(int argc, char **argv) {
  // general_example();  // Skip for ROS2
  
  rclcpp::init(argc, argv);
  auto node = make_shared<LambdaRobotNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
