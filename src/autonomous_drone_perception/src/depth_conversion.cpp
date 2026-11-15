#include "autonomous_drone_perception/depth_conversion.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace autonomous_drone_perception
{
namespace
{
void require(bool condition, const char * reason)
{
  if (!condition) {throw std::invalid_argument(reason);}
}
bool close(double a, double b)
{
  return std::isfinite(a) && std::isfinite(b) && std::abs(a - b) <= 1e-6;
}
}  // namespace

sensor_msgs::msg::Image metric_depth(
  const stereo_msgs::msg::DisparityImage & disparity,
  const sensor_msgs::msg::CameraInfo & left,
  const sensor_msgs::msg::CameraInfo & right,
  const DepthLimits & limits)
{
  const auto & image = disparity.image;
  require(std::isfinite(limits.min_depth_m) && std::isfinite(limits.max_depth_m) &&
    limits.min_depth_m > 0 && limits.max_depth_m > limits.min_depth_m &&
    std::isfinite(limits.max_disparity_px) && limits.max_disparity_px > 0,
    "invalid configured depth/disparity bounds");
  require(disparity.header.stamp == left.header.stamp &&
    disparity.header.stamp == right.header.stamp, "calibration must match capture stamp exactly");
  require(left.header.frame_id == limits.left_frame && right.header.frame_id == limits.right_frame,
    "unexpected calibration frames");
  require(disparity.header.frame_id == limits.left_frame ||
    disparity.header.frame_id == limits.right_frame, "unexpected disparity frame");
  require(image.width > 0 && image.height > 0 && image.width <= 4096 && image.height <= 4096 &&
    image.width == left.width && image.width == right.width &&
    image.height == left.height && image.height == right.height, "image/calibration dimensions differ");
  require(image.encoding == "32FC1" && image.is_bigendian == 0,
    "expected little-endian 32FC1 disparity");
  require(image.step >= image.width * sizeof(float) &&
    image.data.size() == static_cast<size_t>(image.step) * image.height,
    "malformed disparity buffer");
  for (const auto * info : {&left, &right}) {
    require(std::all_of(info->p.begin(), info->p.end(), [](double x) {return std::isfinite(x);}),
      "nonfinite projection matrix");
    const double identity[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    for (size_t i = 0; i < 9; ++i) {
      require(close(info->r[i], identity[i]), "expected verified rectified stereo input");
    }
    require(close(info->p[1], 0) && close(info->p[4], 0) && close(info->p[7], 0) &&
      close(info->p[8], 0) && close(info->p[9], 0) && close(info->p[10], 1) &&
      close(info->p[11], 0), "unsupported projection matrix");
    require(info->binning_x <= 1 && info->binning_y <= 1 &&
      info->roi.x_offset == 0 && info->roi.y_offset == 0 &&
      (info->roi.width == 0 || info->roi.width == info->width) &&
      (info->roi.height == 0 || info->roi.height == info->height),
      "binned/cropped calibration requires an explicit updated contract");
  }
  require(left.p[0] > 0 && left.p[5] > 0 && close(left.p[3], 0) && right.p[3] < 0,
    "invalid horizontal stereo baseline/focal length");
  for (size_t i : {size_t{0}, size_t{2}, size_t{5}, size_t{6}}) {
    require(close(left.p[i], right.p[i]), "rectified camera projections differ");
  }
  require(close(disparity.f, left.p[0]), "disparity focal length differs from calibration");
  require(std::isfinite(disparity.min_disparity) && std::isfinite(disparity.max_disparity) &&
    disparity.min_disparity >= 0 && disparity.max_disparity > disparity.min_disparity,
    "invalid disparity search interval");
  const double baseline_m = -right.p[3] / right.p[0];
  require(baseline_m > 0 && baseline_m <= 1.0, "implausible stereo baseline");

  const auto & roi = disparity.valid_window;
  const bool has_roi = roi.width > 0 || roi.height > 0;
  require(!has_roi || (roi.width > 0 && roi.height > 0 &&
    static_cast<uint64_t>(roi.x_offset) + roi.width <= image.width &&
    static_cast<uint64_t>(roi.y_offset) + roi.height <= image.height),
    "invalid disparity valid-window bounds");
  sensor_msgs::msg::Image depth;
  // VPI disparity is referenced to the LEFT image. Isaac ROS 4.6's SGM source
  // labels its output with the right frame and t=-P_right[3] (pixels*metres).
  // Use the matched left header and -P_right[3] directly, never raw f*t.
  depth.header = left.header;
  depth.width = image.width;
  depth.height = image.height;
  depth.encoding = "32FC1";
  depth.is_bigendian = 0;
  depth.step = depth.width * sizeof(float);
  depth.data.resize(static_cast<size_t>(depth.step) * depth.height);
  const auto max_disparity = std::min(limits.max_disparity_px,
    static_cast<double>(disparity.max_disparity));
  for (uint32_t y = 0; y < image.height; ++y) {
    for (uint32_t x = 0; x < image.width; ++x) {
      float d;
      std::memcpy(&d, image.data.data() + static_cast<size_t>(y) * image.step + x * sizeof(float),
        sizeof(float));
      float z = std::numeric_limits<float>::quiet_NaN();
      const bool inside = !has_roi || (x >= roi.x_offset && x < roi.x_offset + roi.width &&
        y >= roi.y_offset && y < roi.y_offset + roi.height);
      if (inside && std::isfinite(d) && d > disparity.min_disparity && d <= max_disparity) {
        const double candidate = -right.p[3] / d;
        if (candidate >= limits.min_depth_m && candidate <= limits.max_depth_m) {
          z = static_cast<float>(candidate);
        }
      }
      std::memcpy(depth.data.data() + static_cast<size_t>(y) * depth.step + x * sizeof(float),
        &z, sizeof(float));
    }
  }
  return depth;
}
}  // namespace autonomous_drone_perception
