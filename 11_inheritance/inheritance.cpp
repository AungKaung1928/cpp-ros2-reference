// ===== GENERAL C++ =====
#include <iostream>
using namespace std;
class Base{
  public:
    void display(){
      cout << "Display of Base" << endl;
    }
};
class Derived: public Base{
  public:
    void show(){
      cout << "Show of Derived" << endl;
    }
};
void general_example() {
  Base b;
  b.display();
  Derived d;
  d.display();
  d.show();
}
// ===== ROS2 EXAMPLE =====
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
class BaseRobot : public rclcpp::Node {
protected:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  
public:
  BaseRobot(string name) : Node(name) {
    pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }
  
  void stop() {
    auto cmd = geometry_msgs::msg::Twist();
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    pub_->publish(cmd);
    RCLCPP_INFO(this->get_logger(), "Robot stopped");
  }
};
class AutonomousRobot : public BaseRobot {
private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  
public:
  AutonomousRobot() : BaseRobot("autonomous_robot") {
    sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10, [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        navigate(msg);
      });
  }
  
  void navigate(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    float front = scan->ranges[scan->ranges.size() / 2];
    auto cmd = geometry_msgs::msg::Twist();
    
    if(front < 0.5) {
      stop();  // Use inherited method
    } else {
      cmd.linear.x = 0.3;
      pub_->publish(cmd);
      RCLCPP_INFO(this->get_logger(), "Navigating autonomously");
    }
  }
};
int main(int argc, char **argv) {
  // general_example();  // Skip for ROS2
  
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AutonomousRobot>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
