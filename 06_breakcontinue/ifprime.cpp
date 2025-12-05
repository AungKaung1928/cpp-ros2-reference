// ===== GENERAL C++ =====
#include <iostream>
using namespace std;
void general_example() {
  int n;
  cout << "Enter a number: ";
  cin >> n;
  bool isPrime = true;
  for(int i = 2; i < n; i++){
    if(n % i == 0){
      isPrime = false;
      break;
    }
  }
  if(isPrime){
    cout << "Prime Number" << endl;
  }else{
    cout << "Not a Prime Number" << endl;
  }
}
// ===== ROS2 EXAMPLE =====
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"
class PrimeCheckerNode : public rclcpp::Node {
private:
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_;
  bool isPrime(int n) {
    if(n < 2) return false;
    for(int i = 2; i < n; i++){
      if(n % i == 0){
        return false;
      }
    }
    return true;
  }
public:
  PrimeCheckerNode() : Node("prime_checker") {
    sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "number", 10, [this](const std_msgs::msg::Int32::SharedPtr msg) {
        auto result = std_msgs::msg::Bool();
        result.data = isPrime(msg->data);
        pub_->publish(result);
        RCLCPP_INFO(this->get_logger(), "%d is %s", 
          msg->data, result.data ? "Prime" : "Not Prime");
      });
    pub_ = this->create_publisher<std_msgs::msg::Bool>("is_prime", 10);
  }
};
int main(int argc, char **argv) {
  // general_example();  // Skip for ROS2
  
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PrimeCheckerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
