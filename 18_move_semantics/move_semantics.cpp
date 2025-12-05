// ===== GENERAL C++ =====
#include <iostream>
#include <vector>
#include <utility>
using namespace std;

class Data {
public:
    int* ptr;
    int size;
    
    // Constructor
    Data(int s) : size(s) {
        ptr = new int[size];
        cout << "Constructor: Allocated " << size << " ints" << endl;
    }
    
    // Copy constructor
    Data(const Data& other) : size(other.size) {
        ptr = new int[size];
        copy(other.ptr, other.ptr + size, ptr);
        cout << "Copy constructor: Copied " << size << " ints" << endl;
    }
    
    // Move constructor
    Data(Data&& other) noexcept : ptr(other.ptr), size(other.size) {
        other.ptr = nullptr;
        other.size = 0;
        cout << "Move constructor: Moved " << size << " ints" << endl;
    }
    
    // Copy assignment
    Data& operator=(const Data& other) {
        if(this != &other) {
            delete[] ptr;
            size = other.size;
            ptr = new int[size];
            copy(other.ptr, other.ptr + size, ptr);
            cout << "Copy assignment" << endl;
        }
        return *this;
    }
    
    // Move assignment
    Data& operator=(Data&& other) noexcept {
        if(this != &other) {
            delete[] ptr;
            ptr = other.ptr;
            size = other.size;
            other.ptr = nullptr;
            other.size = 0;
            cout << "Move assignment" << endl;
        }
        return *this;
    }
    
    ~Data() {
        delete[] ptr;
        cout << "Destructor" << endl;
    }
};

void general_example() {
    Data d1(100);
    
    // Copy
    Data d2 = d1;
    
    // Move with std::move
    Data d3 = move(d1);
    
    // rvalue (temporary) - automatic move
    Data d4 = Data(50);
    
    cout << "Example complete" << endl;
}

// ===== ROS2 EXAMPLE =====
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

class PointCloudData {
public:
    vector<float> points;
    string frame_id;
    
    PointCloudData(int num_points) : frame_id("base_link") {
        points.resize(num_points * 3);
        for(int i = 0; i < num_points * 3; i++) {
            points[i] = i * 0.1;
        }
        cout << "Created point cloud with " << num_points << " points" << endl;
    }
    
    // Move constructor
    PointCloudData(PointCloudData&& other) noexcept 
        : points(move(other.points)), frame_id(move(other.frame_id)) {
        cout << "Moved point cloud data" << endl;
    }
    
    // Move assignment
    PointCloudData& operator=(PointCloudData&& other) noexcept {
        points = move(other.points);
        frame_id = move(other.frame_id);
        cout << "Move assigned point cloud" << endl;
        return *this;
    }
};

class MoveSemanticsNode : public rclcpp::Node {
private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    
    PointCloudData processCloud(PointCloudData&& cloud) {
        // Process and return by move
        cloud.frame_id = "map";
        return move(cloud);  // Explicit move
    }

public:
    MoveSemanticsNode() : Node("move_semantics") {
        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud", 10);
        
        // Create large data
        PointCloudData cloud1(10000);
        
        // Move to avoid copy
        PointCloudData cloud2 = move(cloud1);
        
        // Process with move semantics
        PointCloudData processed = processCloud(move(cloud2));
        
        RCLCPP_INFO(this->get_logger(), "Processed cloud has %zu points", 
            processed.points.size() / 3);
        
        // Move into vector (efficient)
        vector<PointCloudData> cloud_buffer;
        cloud_buffer.push_back(move(processed));
        
        // Create and immediately move temporary
        cloud_buffer.push_back(PointCloudData(5000));
        
        RCLCPP_INFO(this->get_logger(), "Buffer contains %zu clouds", 
            cloud_buffer.size());
    }
};

int main(int argc, char **argv) {
    // general_example();  // Skip for ROS2
    
    rclcpp::init(argc, argv);
    auto node = make_shared<MoveSemanticsNode>();
    rclcpp::shutdown();
    return 0;
}
