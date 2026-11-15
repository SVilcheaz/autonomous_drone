cmake_minimum_required(VERSION 3.16)
project(autonomous_drone_perception)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(stereo_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(message_filters REQUIRED)

add_library(depth_conversion src/depth_conversion.cpp src/depth_validity.cpp)
target_compile_features(depth_conversion PUBLIC cxx_std_17)
target_compile_options(depth_conversion PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(depth_conversion PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(depth_conversion sensor_msgs stereo_msgs)

add_executable(metric_depth src/metric_depth.cpp)
target_compile_features(metric_depth PRIVATE cxx_std_17)
target_compile_options(metric_depth PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(metric_depth depth_conversion)
ament_target_dependencies(metric_depth rclcpp message_filters std_msgs)

install(TARGETS depth_conversion metric_depth
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_depth_conversion test/test_depth_conversion.cpp)
  target_link_libraries(test_depth_conversion depth_conversion)
  ament_add_gtest(test_depth_validity test/test_depth_validity.cpp)
  target_link_libraries(test_depth_validity depth_conversion)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME test_metric_depth_transport COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_metric_depth_transport.py $<TARGET_FILE:metric_depth>)
  set_tests_properties(test_metric_depth_transport PROPERTIES TIMEOUT 30
    ENVIRONMENT "ROS_DOMAIN_ID=72;ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST;ROS_STATIC_PEERS=")
endif()

ament_package()
