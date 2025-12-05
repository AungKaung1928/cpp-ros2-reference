// ===== GENERAL C++ =====
#include <iostream>
using namespace std;

// Function template
template<typename T>
T getMax(T a, T b) {
    return (a > b) ? a : b;
}

// Function template with multiple types
template<typename T, typename U>
void printPair(T first, U second) {
    cout << "First: " << first << ", Second: " << second << endl;
}

// Class template
template<typename T>
class Box {
private:
    T value;
public:
    Box(T v) : value(v) {}
    
    T getValue() { return value; }
    
    void setValue(T v) { value = v; }
    
    void display() {
        cout << "Box contains: " << value << endl;
    }
};

// Template specialization
template<>
class Box<string> {
private:
    string value;
public:
    Box(string v) : value(v) {}
    
    string getValue() { return value; }
    
    void display() {
        cout << "Box contains string: \"" << value << "\"" << endl;
    }
};

void general_example() {
    // Function templates
    cout << "Max int: " << getMax(10, 20) << endl;
    cout << "Max double: " << getMax(3.14, 2.71) << endl;
    
    printPair(100, 3.14);
    printPair(string("Robot"), 42);
    
    // Class templates
    Box<int> intBox(42);
    intBox.display();
    
    Box<double> doubleBox(3.14159);
    doubleBox.display();
    
    Box<string> stringBox("Hello");
    stringBox.display();
}

// ===== ROS2 EXAMPLE =====
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float64.hpp"

// Template for generic message processor
template<typename MsgType>
class MessageProcessor {
private:
    typename rclcpp::Subscription<MsgType>::SharedPtr sub_;
    string topic_name_;
    
public:
    MessageProcessor(rclcpp::Node* node, const string& topic) : topic_name_(topic) {
        sub_ = node->create_subscription<MsgType>(
            topic, 10, [this](const typename MsgType::SharedPtr msg) {
                process(msg);
            });
    }
    
    virtual void process(const typename MsgType::SharedPtr msg) = 0;
    virtual ~MessageProcessor() = default;
};

// Template for sensor data buffer
template<typename T>
class SensorBuffer {
private:
    T* buffer;
    size_t capacity;
    size_t index;
    
public:
    SensorBuffer(size_t size) : capacity(size), index(0) {
        buffer = new T[capacity];
    }
    
    ~SensorBuffer() {
        delete[] buffer;
    }
    
    void add(T value) {
        buffer[index % capacity] = value;
        index++;
    }
    
    T getAverage() {
        T sum = 0;
        size_t count = (index < capacity) ? index : capacity;
        for(size_t i = 0; i < count; i++) {
            sum += buffer[i];
        }
        return sum / count;
    }
    
    size_t getCount() {
        return (index < capacity) ? index : capacity;
    }
};

class LaserProcessor : public MessageProcessor<sensor_msgs::msg::LaserScan> {
private:
    rclcpp::Node* node_;
    
public:
    LaserProcessor(rclcpp::Node* node, const string& topic) 
        : MessageProcessor(node, topic), node_(node) {}
    
    void process(const sensor_msgs::msg::LaserScan::SharedPtr msg) override {
        float front = msg->ranges[msg->ranges.size() / 2];
        RCLCPP_INFO(node_->get_logger(), "Laser front distance: %.2f", front);
    }
};

class TemplateRobotNode : public rclcpp::Node {
private:
    unique_ptr<LaserProcessor> laser_processor_;
    SensorBuffer<float> distance_buffer_;
    SensorBuffer<double> velocity_buffer_;
    
public:
    TemplateRobotNode() : Node("template_robot"), distance_buffer_(10), velocity_buffer_(10) {
        // Use template-based processor
        laser_processor_ = make_unique<LaserProcessor>(this, "scan");
        
        // Test sensor buffers
        for(int i = 0; i < 15; i++) {
            distance_buffer_.add(i * 0.5f);
            velocity_buffer_.add(i * 0.1);
        }
        
        RCLCPP_INFO(this->get_logger(), "Distance avg: %.2f (count: %zu)", 
            distance_buffer_.getAverage(), distance_buffer_.getCount());
        RCLCPP_INFO(this->get_logger(), "Velocity avg: %.2f (count: %zu)", 
            velocity_buffer_.getAverage(), velocity_buffer_.getCount());
        
        // Template function usage
        auto max_dist = getMax(1.5f, 2.3f);
        auto max_vel = getMax(0.5, 0.8);
        
        RCLCPP_INFO(this->get_logger(), "Max distance: %.2f, Max velocity: %.2f", 
            max_dist, max_vel);
    }
};

int main(int argc, char **argv) {
    // general_example();  // Skip for ROS2
    
    rclcpp::init(argc, argv);
    auto node = make_shared<TemplateRobotNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
