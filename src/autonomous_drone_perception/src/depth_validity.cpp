#include "autonomous_drone_perception/depth_validity.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <vector>

namespace autonomous_drone_perception
{
namespace
{
// Span coarse texture cells; CUDA's census remains its fixed 9x7 footprint.
constexpr int rx = 15, ry = 4;
constexpr int census_rx = 4;

std::vector<double> gradient_integral(const sensor_msgs::msg::Image & image)
{
  const bool mono = image.encoding == "mono8";
  const bool rgb = image.encoding == "rgb8";
  if (!mono && !rgb && image.encoding != "bgr8") {
    throw std::invalid_argument("texture gate requires mono8/rgb8/bgr8 source images");
  }
  const size_t channels = mono ? 1 : 3;
  const size_t w = image.width, h = image.height;
  if (image.step < w * channels || image.data.size() != image.step * h) {
    throw std::invalid_argument("malformed texture image buffer");
  }
  // Integral image of squared horizontal luminance gradients. Gradient at x
  // describes the x -> x+1 pair; the last column is unused.
  std::vector<double> integral((w + 1) * (h + 1), 0);
  for (size_t y = 0; y < h; ++y) {
    double row_sum = 0;
    auto gray = [&](size_t x) {
        const auto * pixel = image.data.data() + y * image.step + x * channels;
        if (mono) {return static_cast<double>(pixel[0]);}
        const int r = rgb ? 0 : 2, b = rgb ? 2 : 0;
        return (77.0 * pixel[r] + 150.0 * pixel[1] + 29.0 * pixel[b]) / 256.0;
      };
    double previous = w ? gray(0) : 0;
    for (size_t x = 0; x < w; ++x) {
      const double next = x + 1 < w ? gray(x + 1) : previous;
      row_sum += (next - previous) * (next - previous);
      integral[(y + 1) * (w + 1) + x + 1] = integral[y * (w + 1) + x + 1] + row_sum;
      previous = next;
    }
  }
  return integral;
}

bool supported(const std::vector<double> & integral, int w, int h, int x, int y, double threshold)
{
  if (x < rx || x + rx >= w || y < ry || y + ry >= h) {return false;}
  const size_t stride = w + 1;
  const int x0 = x - rx, x1 = x + rx;
  const int y0 = y - ry, y1 = y + ry + 1;
  const double energy = integral[y1 * stride + x1] - integral[y0 * stride + x1] -
    integral[y1 * stride + x0] + integral[y0 * stride + x0];
  return energy >= threshold * threshold * (2 * rx) * (2 * ry + 1);
}
}  // namespace

void reject_unsupported_depth(
  sensor_msgs::msg::Image & depth, const sensor_msgs::msg::Image & left,
  const sensor_msgs::msg::Image & right, double focal_baseline, double min_horizontal_gradient_rms,
  double max_disparity_px)
{
  if (!std::isfinite(focal_baseline) || focal_baseline <= 0 ||
    !std::isfinite(min_horizontal_gradient_rms) || min_horizontal_gradient_rms <= 0 ||
    min_horizontal_gradient_rms > 255 || !std::isfinite(max_disparity_px) ||
    max_disparity_px <= 0 || max_disparity_px > 256 ||
    depth.encoding != "32FC1" || depth.is_bigendian ||
    depth.step != depth.width * sizeof(float) || depth.width == 0 || depth.height == 0 ||
    depth.data.size() != static_cast<size_t>(depth.step) * depth.height)
  {
    throw std::invalid_argument("invalid texture/depth gate configuration or layout");
  }
  for (const auto * image : {&left, &right}) {
    if (image->width != depth.width || image->height != depth.height ||
      image->header.stamp != depth.header.stamp)
    {
      throw std::invalid_argument("texture images must match depth dimensions and capture stamp");
    }
  }
  if (left.header.frame_id != depth.header.frame_id || right.header.frame_id.empty() ||
    right.header.frame_id == left.header.frame_id)
  {
    throw std::invalid_argument("unexpected texture image optical frames");
  }
  // Stereo occlusion width is approximately the disparity jump. Keep a band
  // around discontinuities unknown, including the census footprint. This is
  // a conservative support heuristic, not proof of correspondence correctness.
  std::vector<uint8_t> ambiguous(static_cast<size_t>(depth.width) * depth.height, 0);
  for (int y = 0; y < static_cast<int>(depth.height); ++y) {
    std::vector<int> intervals(depth.width + 1, 0);
    for (int x = 0; x < static_cast<int>(depth.width); ++x) {
      double minimum = std::numeric_limits<double>::infinity(), maximum = 0;
      for (int k = std::max(0, x - census_rx);
        k <= std::min(static_cast<int>(depth.width) - 1, x + census_rx); ++k)
      {
        float z;
        std::memcpy(&z, depth.data.data() + static_cast<size_t>(y) * depth.step +
          k * sizeof(float), sizeof(float));
        if (std::isfinite(z) && z > 0) {
          const double d = focal_baseline / z;
          minimum = std::min(minimum, d);
          maximum = std::max(maximum, d);
        }
      }
      if (maximum - minimum > 0.5) {
        const int radius = std::min(static_cast<int>(depth.width),
          static_cast<int>(std::ceil(std::min(maximum - minimum, 256.0))) + census_rx);
        ++intervals[std::max(0, x - radius)];
        --intervals[std::min(static_cast<int>(depth.width), x + radius + 1)];
      }
    }
    int active = 0;
    for (size_t x = 0; x < depth.width; ++x) {
      active += intervals[x];
      ambiguous[static_cast<size_t>(y) * depth.width + x] = active > 0;
    }
  }
  const auto left_energy = gradient_integral(left);
  const auto right_energy = gradient_integral(right);
  for (int y = 0; y < static_cast<int>(depth.height); ++y) {
    for (int x = 0; x < static_cast<int>(depth.width); ++x) {
      auto * pixel = depth.data.data() + static_cast<size_t>(y) * depth.step + x * sizeof(float);
      float z;
      std::memcpy(&z, pixel, sizeof(float));
      if (!std::isfinite(z)) {continue;}
      const double right_x = x - focal_baseline / z;
      // A plausible but wrong small disparity can put an unmatched border
      // point inside the right image. Require support for the whole search,
      // rather than relying only on that estimated correspondence.
      if (z <= 0 || x < max_disparity_px + rx ||
        right_x < rx || right_x >= static_cast<int>(depth.width) - rx ||
        ambiguous[static_cast<size_t>(y) * depth.width + x] ||
        !supported(left_energy, depth.width, depth.height, x, y, min_horizontal_gradient_rms) ||
        !supported(right_energy, depth.width, depth.height, std::lround(right_x), y,
        min_horizontal_gradient_rms))
      {
        z = std::numeric_limits<float>::quiet_NaN();
        std::memcpy(pixel, &z, sizeof(float));
      }
    }
  }
}
}  // namespace autonomous_drone_perception
