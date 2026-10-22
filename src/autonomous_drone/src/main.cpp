#include <exception>
#include <iostream>
#include "autonomous_drone/waypoint_mission.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    rclcpp::spin(autonomous_drone::make_waypoint_mission());
  } catch (const std::exception & error) {
    std::cerr << "Mission node: " << error.what() << '\n';
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
