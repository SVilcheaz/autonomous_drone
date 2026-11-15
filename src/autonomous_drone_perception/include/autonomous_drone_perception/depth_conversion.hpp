#pragma once

#include <string>
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "stereo_msgs/msg/disparity_image.hpp"

namespace autonomous_drone_perception
{
struct DepthLimits
{
  double min_depth_m{0.75};
  double max_depth_m{5.0};
  double max_disparity_px{64.0};
  std::string left_frame{"stereo_left_optical_frame"};
  std::string right_frame{"stereo_right_optical_frame"};
};

// SGM is calculated by Isaac ROS on the GPU. This adapter only validates its
// ROS contract and converts pixels to optical Z using the synchronized P matrices.
// Throws on an inconsistent contract. Unmeasured/out-of-range pixels become NaN.
sensor_msgs::msg::Image metric_depth(
  const stereo_msgs::msg::DisparityImage & disparity,
  const sensor_msgs::msg::CameraInfo & left,
  const sensor_msgs::msg::CameraInfo & right,
  const DepthLimits & limits);
}  // namespace autonomous_drone_perception
