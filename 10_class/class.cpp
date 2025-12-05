// ===== GENERAL C++ =====
#include <iostream>
using namespace std;
class student{
  string name;
  public:
  int age;
  bool gender;
  
  student(){
    cout << "Default Constructor" << endl;
  } 
  student(string s, int a, int g){
    name = s;
    age = a;
    gender = g;
  }
  void setName(string s){
    name = s;
  }
  void getName(){
    cout << name << endl;
  }
  void printInfo(){
    cout << "Name= " << name << endl;
    cout << "Age= " << age << endl;
    cout << "Gender= " << gender << endl;
  }
};
void general_example() {
  student a("Saksham", 22, 0);
  a.printInfo();
  cout << "-------------------" << endl;
  student b;
  b.printInfo();
  cout << "-------------------" << endl;
  student c = a;  // Copy constructor
  c.printInfo();
  cout << "-------------------" << endl;
}
// ===== ROS2 EXAMPLE =====
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
class RobotState {
  string robot_id;
  public:
  double x, y, theta;
  
  RobotState() : robot_id("robot_0"), x(0.0), y(0.0), theta(0.0) {}
  
  RobotState(string id, double px, double py, double ptheta) 
    : robot_id(id), x(px), y(py), theta(ptheta) {}
  
  void setID(string id) { robot_id = id; }
  
  void printState() {
    cout << "Robot: " << robot_id 
         << " Pos: (" << x << ", " << y << ") Theta: " << theta << endl;
  }
};
class RobotFleetNode : public rclcpp::Node {
private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  RobotState robot_;
public:
  RobotFleetNode() : Node("robot_fleet"), robot_("turtlebot_1", 0.0, 0.0, 0.0) {
    pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    
    robot_.printState();
    
    // Update position
    robot_.x = 1.5;
    robot_.y = 2.3;
    robot_.theta = 0.785;
    
    RCLCPP_INFO(this->get_logger(), "Updated state:");
    robot_.printState();
  }
};
int main(int argc, char **argv) {
  // general_example();  // Skip for ROS2
  
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotFleetNode>();
  rclcpp::shutdown();
  return 0;
}
