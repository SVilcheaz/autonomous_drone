#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace autonomous_drone
{
std::shared_ptr<rclcpp::Node> make_waypoint_mission(
  const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
}  // namespace autonomous_drone
