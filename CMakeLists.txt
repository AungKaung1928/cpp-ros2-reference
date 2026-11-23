cmake_minimum_required(VERSION 3.8)
project(cpp_ros2_reference)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

# One executable per lesson file, named after the file: 04_ifelse/oddeven.cpp -> oddeven
file(GLOB LESSONS CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/[0-9][0-9]_*/*.cpp)
foreach(src ${LESSONS})
  get_filename_component(name ${src} NAME_WE)
  add_executable(${name} ${src})
  ament_target_dependencies(${name} rclcpp geometry_msgs sensor_msgs std_msgs)
  install(TARGETS ${name} DESTINATION lib/${PROJECT_NAME})
endforeach()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_ray_index test/test_ray_index.cpp)
  ament_target_dependencies(test_ray_index sensor_msgs)
endif()

ament_package()
