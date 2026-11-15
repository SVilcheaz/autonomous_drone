cmake_minimum_required(VERSION 3.16)
project(autonomous_drone)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)

add_executable(hello_node src/hello_node.cpp)
target_compile_features(hello_node PRIVATE cxx_std_17)
ament_target_dependencies(hello_node rclcpp)

add_executable(px4_monitor src/px4_monitor.cpp)
target_compile_features(px4_monitor PRIVATE cxx_std_17)
ament_target_dependencies(px4_monitor rclcpp px4_msgs)

install(TARGETS hello_node px4_monitor DESTINATION lib/${PROJECT_NAME})

ament_package()
