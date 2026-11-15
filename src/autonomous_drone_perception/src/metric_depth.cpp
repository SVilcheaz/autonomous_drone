#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>

#include "autonomous_drone_perception/depth_conversion.hpp"
#include "autonomous_drone_perception/depth_validity.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/exact_time.h"
#include "message_filters/synchronizer.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace autonomous_drone_perception
{
class MetricDepth : public rclcpp::Node
{
  using Disparity = stereo_msgs::msg::DisparityImage;
  using Info = sensor_msgs::msg::CameraInfo;
  using Image = sensor_msgs::msg::Image;
  using Policy = message_filters::sync_policies::ExactTime<Disparity, Info, Info, Image, Image>;
  using Steady = std::chrono::steady_clock;

public:
  MetricDepth() : Node("metric_depth")
  {
    limits_.min_depth_m = declare_parameter("min_depth_m", 0.75);
    limits_.max_depth_m = declare_parameter("max_depth_m", 5.0);
    limits_.max_disparity_px = declare_parameter("max_disparity_px", 64.0);
    limits_.left_frame = declare_parameter("left_frame", "stereo_left_optical_frame");
    limits_.right_frame = declare_parameter("right_frame", "stereo_right_optical_frame");
    max_age_ = declare_parameter("max_capture_age_s", 0.5);
    receipt_timeout_ = declare_parameter("receipt_timeout_s", 0.5);
    min_gradient_rms_ = declare_parameter("min_horizontal_gradient_rms", 2.0);
    const int queue = declare_parameter("queue_size", 5);
    if (!std::isfinite(max_age_) || max_age_ <= 0 ||
      !std::isfinite(receipt_timeout_) || receipt_timeout_ <= 0 || queue < 1 || queue > 20 ||
      !std::isfinite(limits_.min_depth_m) || !std::isfinite(limits_.max_depth_m) ||
      limits_.min_depth_m <= 0 || limits_.max_depth_m <= limits_.min_depth_m ||
      !std::isfinite(limits_.max_disparity_px) || limits_.max_disparity_px <= 0 ||
      !std::isfinite(min_gradient_rms_) || min_gradient_rms_ <= 0 || min_gradient_rms_ > 255)
    {
      throw std::invalid_argument("invalid metric depth bounds or queue size");
    }
    auto qos = rclcpp::SensorDataQoS().keep_last(queue);
    depth_pub_ = create_publisher<sensor_msgs::msg::Image>("depth", qos);
    fresh_pub_ = create_publisher<std_msgs::msg::Bool>("depth_fresh", rclcpp::QoS(1));
    disparity_.subscribe(this, "disparity", qos.get_rmw_qos_profile());
    left_.subscribe(this, "left/camera_info", qos.get_rmw_qos_profile());
    right_.subscribe(this, "right/camera_info", qos.get_rmw_qos_profile());
    left_image_.subscribe(this, "left/image_rect", qos.get_rmw_qos_profile());
    right_image_.subscribe(this, "right/image_rect", qos.get_rmw_qos_profile());
    sync_ = std::make_unique<message_filters::Synchronizer<Policy>>(
      Policy(queue), disparity_, left_, right_, left_image_, right_image_);
    sync_->registerCallback(std::bind(&MetricDepth::on_disparity, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
      std::placeholders::_4, std::placeholders::_5));
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {
        const auto now = sensor_now();
        std_msgs::msg::Bool fresh;
        const double age = (now - last_stamp_) * 1e-9;
        fresh.data = have_depth_ && now > 0 && age >= 0 && age <= max_age_ &&
          std::chrono::duration<double>(Steady::now() - last_receipt_).count() <= receipt_timeout_;
        fresh_pub_->publish(fresh);
      });
    RCLCPP_INFO(get_logger(), "Metric depth: calibrated optical Z; invalid pixels=NaN; no timer republishes depth");
  }

private:
  int64_t sensor_now()
  {
    const auto now = get_clock()->now().nanoseconds();
    if (last_now_ > 0 && now < last_now_) {
      have_depth_ = false;
      last_stamp_ = -1;
      RCLCPP_WARN(get_logger(), "Simulation clock reset: depth freshness invalidated");
    }
    last_now_ = now;
    return now;
  }

  void on_disparity(Disparity::ConstSharedPtr d, Info::ConstSharedPtr l, Info::ConstSharedPtr r,
    Image::ConstSharedPtr left_image, Image::ConstSharedPtr right_image)
  {
    const auto now = sensor_now();
    const auto stamp = rclcpp::Time(d->header.stamp).nanoseconds();
    const double age = (now - stamp) * 1e-9;
    if (!get_clock()->ros_time_is_active() || now <= 0 || age < 0 || age > max_age_ ||
      stamp <= last_stamp_)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Dropping stale, future, repeated, or unclocked disparity (age %.3f s)", age);
      return;
    }
    try {
      auto depth = metric_depth(*d, *l, *r, limits_);
      if (right_image->header.frame_id != limits_.right_frame) {
        throw std::invalid_argument("unexpected right texture image frame");
      }
      reject_unsupported_depth(depth, *left_image, *right_image, -r->p[3], min_gradient_rms_,
        std::min(limits_.max_disparity_px, static_cast<double>(d->max_disparity)));
      depth_pub_->publish(std::move(depth));
      last_stamp_ = stamp;
      last_receipt_ = Steady::now();
      have_depth_ = true;
    } catch (const std::invalid_argument & error) {
      have_depth_ = false;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Depth contract rejected: %s", error.what());
    }
  }

  DepthLimits limits_;
  double max_age_{}, receipt_timeout_{};
  double min_gradient_rms_{};
  bool have_depth_{false};
  int64_t last_stamp_{-1}, last_now_{0};
  Steady::time_point last_receipt_{};
  message_filters::Subscriber<Disparity> disparity_;
  message_filters::Subscriber<Info> left_, right_;
  message_filters::Subscriber<Image> left_image_, right_image_;
  std::unique_ptr<message_filters::Synchronizer<Policy>> sync_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr fresh_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace autonomous_drone_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    rclcpp::spin(std::make_shared<autonomous_drone_perception::MetricDepth>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("metric_depth"), "%s", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
