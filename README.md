# C++ Fundamentals for ROS2 Robotics
Quick reference guide for C++ syntax and ROS2 examples on Ubuntu 22.04 with ROS2 Humble.

## Getting Started
```bash
# Clone this repository
git clone https://github.com/YOUR_USERNAME/cpp-ros2-fundamentals.git
cd cpp-ros2-fundamentals

# Compile any example
cd 01_helloworld
g++ -std=c++17 01_helloworld.cpp -o helloworld
./helloworld
```

## Contents
### Basics
- [01 - Hello World](./01_helloworld) - Basic output and program structure
- [02 - Data Types](./02_datatypes) - Primitive types, sizes, and declarations
- [03 - Input/Output](./03_inputoutput) - cin, cout, and basic I/O operations
- [04 - If/Else](./04_ifelse) - Conditional statements and decision making
- [05 - Loops](./05_loops) - for, while, and iteration patterns
- [06 - Break/Continue](./06_breakcontinue) - Loop control flow
- [07 - Switch](./07_switch) - Multi-way branching with switch statements
- [08 - Functions](./08_functions) - Function declaration, definition, and usage

### Intermediate
- [09 - Pointers](./09_pointers) - Memory addresses, dereferencing, and pointer arithmetic
- [10 - Class](./10_class) - OOP basics, constructors, encapsulation
- [11 - Inheritance](./11_inheritance) - Base and derived classes
- [12 - Polymorphism](./12_polymorphism) - Virtual functions and runtime polymorphism

### Advanced
- [13 - Threading](./13_threading) - Multi-threading with std::thread
- [14 - Mutex](./14_mutex) - Thread synchronization and race condition prevention
- [15 - Semaphores](./15_semaphores) - Condition variables and thread coordination
- [16 - Smart Pointers](./16_smart_pointers) - unique_ptr, shared_ptr, weak_ptr (Critical for ROS2)
- [17 - Lambda Functions](./17_lambda_functions) - Anonymous functions and callbacks (Critical for ROS2)
- [18 - Move Semantics](./18_move_semantics) - rvalue references and std::move for performance
- [19 - Templates](./19_templates) - Generic programming with function and class templates

## Purpose
Each file contains:
1. **General C++ Example** - Pure C++ syntax reference
2. **ROS2 Example** - Practical robotics application

## Quick Start
```bash
# Compile general C++ example
g++ -std=c++17 01_helloworld.cpp -o helloworld
./helloworld
# Build ROS2 example
colcon build --packages-select your_package
source install/setup.bash
ros2 run your_package node_name
```

## Notes
- All examples use **C++17** standard
- ROS2 examples are for **Humble** distribution
- Focus on production-grade patterns
- Smart pointers, lambdas, and move semantics are **essential** for modern ROS2

## Learning Path
**Beginners**: 01 → 08  
**Intermediate**: 09 → 12  
**Advanced/ROS2 Ready**: 13 → 19
