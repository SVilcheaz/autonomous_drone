#pragma once

#include "sensor_msgs/msg/image.hpp"

namespace autonomous_drone_perception
{
// Conservative observation-support gate, not a stereo matcher. Requires the
// exact source pair, horizontal structure in a 31x9 neighborhood in both views,
// an in-image correspondence, and conservative support around disparity edges.
// Rejected depth stays unknown; this heuristic does not prove every match.
void reject_unsupported_depth(
  sensor_msgs::msg::Image & depth,
  const sensor_msgs::msg::Image & left,
  const sensor_msgs::msg::Image & right,
  double focal_baseline, double min_horizontal_gradient_rms, double max_disparity_px = 64.0);
}  // namespace autonomous_drone_perception
