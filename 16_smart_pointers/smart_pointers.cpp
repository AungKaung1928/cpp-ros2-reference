// ===== GENERAL C++ =====
#include <iostream>
#include <memory>
using namespace std;
class Student {
public:
  string name;
  int age;
  
  Student(string n, int a) : name(n), age(a) {
    cout << "Student " << name << " created" << endl;
  }
  
  ~Student() {
    cout << "Student " << name << " destroyed" << endl;
  }
  
  void display() {
    cout << "Name: " << name << ", Age: " << age << endl;
  }
};
void general_example() {
  // unique_ptr - exclusive ownership
  unique_ptr<Student> s1 = make_unique<Student>("Alice", 20);
  s1->display();
  
  // shared_ptr - shared ownership
  shared_ptr<Student> s2 = make_shared<Student>("Bob", 22);
  shared_ptr<Student> s3 = s2;  // Both point to same object
  cout << "Reference count: " << s2.use_count() << endl;
  s2->display();
  
  // weak_ptr - non-owning reference
  weak_ptr<Student> w1 = s2;
  if(auto temp = w1.lock()) {
    temp->display();
  }
}
// ===== ROS2 EXAMPLE =====
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
class RobotController {
public:
  string robot_id;
  
  RobotController(string id) : robot_id(id) {
    cout << "Controller " << robot_id << " created" << endl;
  }
  
  ~RobotController() {
    cout << "Controller " << robot_id << " destroyed" << endl;
  }
  
  void move(rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub) {
    auto cmd = geometry_msgs::msg::Twist();
    cmd.linear.x = 0.3;
    pub->publish(cmd);
  }
};
class SmartPointerNode : public rclcpp::Node {
private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  shared_ptr<RobotController> controller_;  // Shared ownership
  
public:
  SmartPointerNode() : Node("smart_pointer_robot") {
    pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    
    // Create controller with shared_ptr
    controller_ = make_shared<RobotController>("Robot_1");
    
    // Multiple references to same controller
    shared_ptr<RobotController> backup_controller = controller_;
    RCLCPP_INFO(this->get_logger(), "Controller ref count: %ld", controller_.use_count());
    
    // Use controller
    controller_->move(pub_);
    
    // unique_ptr for temporary data
    unique_ptr<int[]> sensor_data = make_unique<int[]>(10);
    for(int i = 0; i < 10; i++) {
      sensor_data[i] = i * 10;
    }
    
    RCLCPP_INFO(this->get_logger(), "Smart pointers demo complete");
  }
  
  ~SmartPointerNode() {
    RCLCPP_INFO(this->get_logger(), "Node shutting down");
  }
};
int main(int argc, char **argv) {
  // general_example();  // Skip for ROS2
  
  rclcpp::init(argc, argv);
  auto node = make_shared<SmartPointerNode>();  // Node itself uses shared_ptr
  rclcpp::shutdown();
  return 0;
}
