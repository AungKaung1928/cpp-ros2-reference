# C++ for ROS 2: one concept per lesson, each as a node

19 lessons of C++17 (hello world to templates). Each `.cpp` file has two halves:
`general_example()`, plain C++ for the concept, and a small rclcpp node that uses the
same concept on a robot topic. Target: ROS 2 Humble, Ubuntu 22.04.

Every file builds to its own executable, named after the file
(`04_ifelse/ifelse.cpp` -> `ifelse`). CI builds all of them and runs the tests in a
`ros:humble` container on every push.

## Build and run

```bash
mkdir -p ~/r2_ws/src && cd ~/r2_ws/src
git clone https://github.com/AungKaung1928/cpp-ros2-reference.git
cd ~/r2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -y
colcon build
source install/setup.bash
ros2 run cpp_ros2_reference ifelse      # waits for /scan, logs which side is clearest
```

Feed a lesson by hand, for example lesson 04's odd/even node:

```bash
ros2 run cpp_ros2_reference oddeven &
ros2 topic echo /parity &
ros2 topic pub -t 3 /number std_msgs/msg/Int32 "{data: 7}"   # logs "7 is odd"
```

Lessons 01-02 run `general_example()` before the node starts. In 03-19 that call is
commented out in `main()` so the node starts straight away; uncomment it and rebuild
to see the plain C++ output.

## Lessons and the topics their nodes use

| Lesson | Executables | Subscribes | Publishes |
| --- | --- | --- | --- |
| [01 hello world](01_helloworld) | `helloworld` | | |
| [02 data types](02_datatypes) | `datatypes` | | |
| [03 input/output](03_inputoutput) | `inputoutput` | `input1`, `input2` (Int32) | `sum` |
| [04 if/else](04_ifelse) | `ifelse`, `oddeven` | `scan`; `number` | `parity` |
| [05 loops](05_loops) | `for`, `while` | `scan` | `status`; `cmd_vel` |
| [06 break/continue](06_breakcontinue) | `breakcontinue`, `ifprime`, `skipif` | `battery`; `number`; `scan` | `cmd_vel`; `is_prime` |
| [07 switch](07_switch) | `switch`, `calculator` | `mode`; `command` | `status`; `cmd_vel` |
| [08 functions](08_functions) | `function` | `scan` | `cmd_vel` |
| [09 pointers](09_pointers) | `pointers` | `scan` | |
| [10 class](10_class) | `class` | | `cmd_vel` |
| [11 inheritance](11_inheritance) | `inheritance` | `scan` | `cmd_vel` |
| [12 polymorphism](12_polymorphism) | `polymorphism` | | `cmd_vel` |
| [13 threading](13_threading) | `threading`, `detached`, `functionpointer` | | `cmd_vel` |
| [14 mutex](14_mutex) | `mutex` | `input1`, `input2` | `total` |
| [15 semaphores](15_semaphores) | `semaphores` | `start_signal` | `cmd_vel` |
| [16 smart pointers](16_smart_pointers) | `smart_pointers` | | `cmd_vel` |
| [17 lambdas](17_lambda_functions) | `lambda_functions` | `scan`, `mode` | `cmd_vel` |
| [18 move semantics](18_move_semantics) | `move_semantics` | | `cloud` (PointCloud2) |
| [19 templates](19_templates) | `templates` | | |

Nodes that publish `cmd_vel` move a robot. Run them against a simulator
(TurtleBot3 in Gazebo) until you have read the file.

## Reading a laser scan by direction

The scan lessons pick front, left and right with
[`ray_index(msg, angle)`](include/cpp_ros2_reference/ray_index.hpp): the index comes
from `angle_min` and `angle_increment` (0 = ahead, +pi/2 = left, REP 103). Fixed
fractions such as `ranges.size() / 2` give the back of a TurtleBot3 LDS-01 scan
(0..2pi) and swap left and right on a -pi..pi scan;
[`test/test_ray_index.cpp`](test/test_ray_index.cpp) checks both layouts.

```bash
colcon test --packages-select cpp_ros2_reference && colcon test-result --verbose
```

## License

MIT, see [LICENSE](LICENSE).
