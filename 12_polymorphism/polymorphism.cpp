// ===== GENERAL C++ =====
#include <iostream>
using namespace std;
class Base{
  public:
    virtual void display(){
      cout << "Display of Base" << endl;
    }
};
class Derived: public Base{
  public:
    void display(){
      cout << "Display of Derived" << endl;
    }
};
void general_example() {
  Base *basePointer;
  Base b;
  Derived d;
  basePointer = &b;
  basePointer->display();
  basePointer = &d;
  basePointer->display();
}
// ===== ROS2 EXAMPLE =====
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
class MotionController {
public:
  virtual void move(rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub) {
    auto cmd = geometry_msgs::msg::Twist();
    cmd.linear.x = 0.2;
    pub->publish(cmd);
    cout << "Base: Moving forward" << endl;
  }
  virtual ~MotionController() = default;
};
class AggressiveController : public MotionController {
public:
  void move(rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub) override {
    auto cmd = geometry_msgs::msg::Twist();
    cmd.linear.x = 0.8;
    pub->publish(cmd);
    cout << "Aggressive: Fast forward" << endl;
  }
};
class CautiousController : public MotionController {
public:
  void move(rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub) override {
    auto cmd = geometry_msgs::msg::Twist();
    cmd.linear.x = 0.1;
    pub->publish(cmd);
    cout << "Cautious: Slow forward" << endl;
  }
};
class RobotNode : public rclcpp::Node {
private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  MotionController *controller_;
public:
  RobotNode() : Node("robot_polymorphism") {
    pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    
    AggressiveController aggressive;
    CautiousController cautious;
    
    controller_ = &aggressive;
    controller_->move(pub_);
    
    controller_ = &cautious;
    controller_->move(pub_);
  }
};
int main(int argc, char **argv) {
  // general_example();  // Skip for ROS2
  
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotNode>();
  rclcpp::shutdown();
  return 0;
}
