#pragma once

#include <cmath>
#include <cstddef>

#include "sensor_msgs/msg/laser_scan.hpp"

namespace cpp_ros2_reference
{
// Index of the ray nearest `angle` (rad; 0 = straight ahead, +pi/2 = left, REP 103).
// Uses angle_min/angle_increment, so it works whether the lidar scans 0..2*pi
// (TurtleBot3 LDS-01) or -pi..pi; fixed fractions like ranges.size() / 4 do not.
inline std::size_t ray_index(const sensor_msgs::msg::LaserScan & msg, double angle)
{
  const long n = static_cast<long>(msg.ranges.size());
  const long i = std::lround((angle - msg.angle_min) / msg.angle_increment) % n;
  return static_cast<std::size_t>(i < 0 ? i + n : i);
}
}  // namespace cpp_ros2_reference
