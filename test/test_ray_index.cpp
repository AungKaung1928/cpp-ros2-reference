#include <gtest/gtest.h>

#include <cmath>

#include "cpp_ros2_reference/ray_index.hpp"

using cpp_ros2_reference::ray_index;

// 360-ray scan whose range equals the ray's own angle in degrees (0..359),
// so the value read back says which direction was picked.
static sensor_msgs::msg::LaserScan scan(double angle_min)
{
  sensor_msgs::msg::LaserScan msg;
  msg.angle_min = angle_min;
  msg.angle_increment = 2 * M_PI / 360;
  for (int i = 0; i < 360; ++i) {
    const double deg = std::round((angle_min + i * msg.angle_increment) * 180.0 / M_PI);
    msg.ranges.push_back(std::fmod(deg + 360.0, 360.0));
  }
  return msg;
}

class RayIndex : public ::testing::TestWithParam<double> {};

TEST_P(RayIndex, FrontLeftRight)
{
  const auto msg = scan(GetParam());
  EXPECT_FLOAT_EQ(msg.ranges[ray_index(msg, 0.0)], 0.0);
  EXPECT_FLOAT_EQ(msg.ranges[ray_index(msg, M_PI / 2)], 90.0);   // left, REP 103
  EXPECT_FLOAT_EQ(msg.ranges[ray_index(msg, -M_PI / 2)], 270.0);  // right
}

// 0..2pi is the TurtleBot3 LDS-01 layout; -pi..pi is common on other lidars.
INSTANTIATE_TEST_SUITE_P(ScanLayouts, RayIndex, ::testing::Values(0.0, -M_PI));

TEST(RayIndexOld, SizeFractionsPointElsewhere)
{
  EXPECT_FLOAT_EQ(scan(0.0).ranges[360 / 2], 180.0);     // "front" was the back
  EXPECT_FLOAT_EQ(scan(-M_PI).ranges[360 / 4], 270.0);   // "left" was the right
}
