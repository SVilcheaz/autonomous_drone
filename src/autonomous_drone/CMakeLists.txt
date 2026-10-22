cmake_minimum_required(VERSION 3.16)
project(autonomous_drone)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_library(waypoint_mission_lib src/waypoint_mission.cpp)
target_compile_features(waypoint_mission_lib PUBLIC cxx_std_17)
target_compile_options(waypoint_mission_lib PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(waypoint_mission_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(waypoint_mission_lib rclcpp rcl_interfaces px4_msgs std_msgs std_srvs)

add_executable(waypoint_mission src/main.cpp)
target_link_libraries(waypoint_mission waypoint_mission_lib)
install(TARGETS waypoint_mission_lib waypoint_mission
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_waypoint_mission test/test_waypoint_mission.cpp TIMEOUT 120)
  target_link_libraries(test_waypoint_mission waypoint_mission_lib)
  # Test traffic must never share the project's live PX4 domain (10).
  set_tests_properties(test_waypoint_mission PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=71;ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST;ROS_STATIC_PEERS=")
endif()

ament_package()
