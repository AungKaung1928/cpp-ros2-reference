// ===== GENERAL C++ =====
#include <iostream>
#include <thread>
#include <chrono>
using namespace std;
using namespace std::chrono;

typedef unsigned long long ull;
ull OddSum = 0;
ull EvenSum = 0;

void findOdd(ull start, ull end){
    for(ull i = start; i <= end; ++i){
        if((i & 1) == 1){
            OddSum += i;
        }
    }
}

void findEven(ull start, ull end){
    for(ull i = start; i <= end; ++i){
        if((i & 1) == 0){
            EvenSum += i;
        }
    }
}

void general_example() {
    ull start = 0, end = 1900000000;
    auto startTime = high_resolution_clock::now();
    
    thread t1(findOdd, start, end);
    thread t2(findEven, start, end);
    t1.join();
    t2.join();
    
    auto stopTime = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stopTime - startTime);
    cout << "OddSum: " << OddSum << endl;
    cout << "EvenSum: " << EvenSum << endl;
    cout << "Time: " << duration.count()/1000000 << " seconds" << endl;
}

// ===== ROS2 EXAMPLE =====
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/twist.hpp"

class PointCloudProcessorNode : public rclcpp::Node {
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    int obstacle_count_ = 0;
    int ground_count_ = 0;

    void processObstacles(const float* data, size_t start, size_t end) {
        for(size_t i = start; i < end; i++) {
            if(data[i] > 0.3) {  // Height threshold
                obstacle_count_++;
            }
        }
    }

    void processGround(const float* data, size_t start, size_t end) {
        for(size_t i = start; i < end; i++) {
            if(data[i] <= 0.3) {
                ground_count_++;
            }
        }
    }

public:
    PointCloudProcessorNode() : Node("pointcloud_processor") {
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        
        // Simulate point cloud data
        const size_t cloud_size = 100000;
        float cloud_data[cloud_size];
        for(size_t i = 0; i < cloud_size; i++) {
            cloud_data[i] = (i % 100) * 0.01;
        }
        
        auto startTime = high_resolution_clock::now();
        
        thread t1(&PointCloudProcessorNode::processObstacles, this, cloud_data, 0, cloud_size/2);
        thread t2(&PointCloudProcessorNode::processGround, this, cloud_data, cloud_size/2, cloud_size);
        t1.join();
        t2.join();
        
        auto stopTime = high_resolution_clock::now();
        auto duration = duration_cast<microseconds>(stopTime - startTime);
        
        RCLCPP_INFO(this->get_logger(), "Obstacles: %d, Ground: %d", obstacle_count_, ground_count_);
        RCLCPP_INFO(this->get_logger(), "Processing time: %ld μs", duration.count());
        
        auto cmd = geometry_msgs::msg::Twist();
        cmd.linear.x = (obstacle_count_ < 1000) ? 0.3 : 0.0;
        pub_->publish(cmd);
    }
};

int main(int argc, char **argv) {
    // general_example();  // Skip for ROS2
    
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudProcessorNode>();
    rclcpp::shutdown();
    return 0;
}
