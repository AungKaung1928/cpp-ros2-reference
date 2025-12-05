// ===== GENERAL C++ =====
#include <iostream>
using namespace std;
void general_example() {
  int a, b;
  char op;
  cout << "Enter two numbers: ";
  cin >> a >> b;
  cout << "Enter operator: ";
  cin >> op;
  switch(op){
    case '+':
      cout << a + b << endl;
      break;
    case '-':
      cout << a - b << endl;
      break;
    case '*':
      cout << a * b << endl;
      break;
    case '/':
      cout << a / b << endl;
      break;
    default:
      cout << "Invalid Operator" << endl;
  }
}
// ===== ROS2 EXAMPLE =====
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
class RobotCommandNode : public rclcpp::Node {
private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
public:
  RobotCommandNode() : Node("robot_command") {
    sub_ = this->create_subscription<std_msgs::msg::String>(
      "command", 10, [this](const std_msgs::msg::String::SharedPtr msg) {
        auto cmd = geometry_msgs::msg::Twist();
        
        switch(msg->data[0]){
          case 'f':  // forward
            cmd.linear.x = 0.5;
            break;
          case 'b':  // backward
            cmd.linear.x = -0.5;
            break;
          case 'l':  // left
            cmd.angular.z = 0.5;
            break;
          case 'r':  // right
            cmd.angular.z = -0.5;
            break;
          case 's':  // stop
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
            break;
          default:
            RCLCPP_WARN(this->get_logger(), "Invalid command");
            return;
        }
        pub_->publish(cmd);
        RCLCPP_INFO(this->get_logger(), "Command: %s", msg->data.c_str());
      });
    pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }
};
int main(int argc, char **argv) {
  // general_example();  // Skip for ROS2
  
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotCommandNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
