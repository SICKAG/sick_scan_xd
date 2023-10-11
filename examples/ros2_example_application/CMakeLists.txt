####################################################
# CMakeLists.txt for project sick_scan_ros2_example
####################################################
cmake_minimum_required(VERSION 3.5)
project(sick_scan_ros2_example)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

link_libraries(pthread) # std::thread requires pthread on linux

set(FastRTPS_INCLUDE_DIR /opt/ros/$ENV{ROS_DISTRO}/include)
set(FastRTPS_LIBRARY_RELEASE /opt/ros/$ENV{ROS_DISTRO}/lib/libfastrtps.so)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(sick_scan_xd REQUIRED)

add_executable(sick_scan_ros2_example src/sick_scan_ros2_example.cpp)

ament_target_dependencies(
        sick_scan_ros2_example
        "rclcpp"
        "sensor_msgs"
        "std_msgs"
        "geometry_msgs"
        "visualization_msgs"
        "sick_scan_xd"
)

install(TARGETS sick_scan_ros2_example DESTINATION lib/${PROJECT_NAME})
ament_package()        
