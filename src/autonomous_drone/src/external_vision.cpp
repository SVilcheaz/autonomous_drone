#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>

#include "builtin_interfaces/msg/time.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "rclcpp/rclcpp.hpp"

#include "autonomous_drone/px4_topic.hpp"

namespace autonomous_drone
{
namespace
{
using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using nav_msgs::msg::Odometry;
using px4_msgs::msg::VehicleOdometry;
using SteadyClock = std::chrono::steady_clock;
using namespace std::chrono_literals;

constexpr auto freshness_limit = 250ms;
constexpr auto freshness_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(freshness_limit).count();
constexpr auto unavailable = std::numeric_limits<float>::quiet_NaN();
constexpr std::size_t pending_limit = 16;

struct PendingSample
{
  SteadyClock::time_point received_at{SteadyClock::now()};
  Odometry::ConstSharedPtr odometry;
  bool healthy{false};
};

struct PreviousPose
{
  int64_t stamp;
  std::array<double, 3> position;
};

int64_t capture_stamp(const builtin_interfaces::msg::Time & stamp)
{
  constexpr int64_t ns_per_second = 1'000'000'000;
  if (stamp.sec < 0 || stamp.nanosec >= ns_per_second) {
    return 0;
  }
  return static_cast<int64_t>(stamp.sec) * ns_per_second + stamp.nanosec;
}

}  // namespace

class ExternalVision final : public rclcpp::Node
{
public:
  ExternalVision()
  : Node("external_vision")
  {
    if (!get_parameter("use_sim_time").as_bool()) {
      throw std::invalid_argument("external_vision requires the shared SITL clock (use_sim_time=true)");
    }

    const auto prefix = declare_parameter<std::string>("fmu_namespace", "/fmu");
    const auto qos = rclcpp::SensorDataQoS().keep_last(10);
    output_ = create_publisher<VehicleOdometry>(
      px4_topic<VehicleOdometry>(prefix, "/in/vehicle_visual_odometry"), qos);
    odometry_ = create_subscription<Odometry>(
      "/visual_slam/tracking/odometry", qos,
      [this](Odometry::ConstSharedPtr msg, const rclcpp::MessageInfo & info) {
        receive_odometry(msg, info);
      });
    diagnostics_ = create_subscription<DiagnosticArray>(
      "/diagnostics", qos,
      [this](DiagnosticArray::ConstSharedPtr msg) {receive_diagnostics(*msg);});
    watchdog_ = create_wall_timer(25ms, [this]() {process_pending();});
  }

private:
  void invalidate_continuity()
  {
    pending_.clear();
    previous_pose_.reset();
    reset_pending_ = published_;
  }

  void receive_odometry(Odometry::ConstSharedPtr msg, const rclcpp::MessageInfo & info)
  {
    const auto stamp = capture_stamp(msg->header.stamp);
    if (!sample_can_arrive(stamp)) {
      return;
    }
    const auto & gid = info.get_rmw_message_info().publisher_gid;
    if (publisher_gid_ && !std::equal(publisher_gid_->begin(), publisher_gid_->end(), gid.data)) {
      invalidate_continuity();
    }
    publisher_gid_.emplace();
    std::copy_n(gid.data, publisher_gid_->size(), publisher_gid_->begin());

    pending_[stamp].odometry = std::move(msg);
    process_pending();
  }

  void receive_diagnostics(const DiagnosticArray & msg)
  {
    const auto status = std::find_if(msg.status.begin(), msg.status.end(),
      [](const DiagnosticStatus & item) {
        return item.name == "Visual Slam Diagnostics" && item.hardware_id == "visual_slam";
      });
    if (status == msg.status.end()) {
      return;
    }
    const auto stamp = capture_stamp(msg.header.stamp);
    if (!sample_can_arrive(stamp)) {
      return;
    }
    if (status->level == DiagnosticStatus::OK) {
      pending_[stamp].healthy = true;
    } else {
      rejected_through_ = stamp;
      invalidate_continuity();
    }
    process_pending();
  }

  bool sample_can_arrive(int64_t stamp)
  {
    const auto now = get_clock()->now().nanoseconds();
    return stamp >= 1000 && stamp > last_published_stamp_ && stamp > rejected_through_ &&
           now > 0 && std::abs(now - stamp) <= freshness_ns;
  }

  void process_pending()
  {
    const auto wall_now = SteadyClock::now();
    const auto sim_now = get_clock()->now().nanoseconds();
    if (sim_now < last_clock_stamp_) {
      invalidate_continuity();
      rejected_through_ = 0;
      last_published_stamp_ = 0;
    }
    if (sim_now != last_clock_stamp_) {
      last_clock_activity_ = wall_now;
      last_clock_stamp_ = sim_now;
    }
    if (sim_now <= 0 || wall_now - last_clock_activity_ > freshness_limit) {
      invalidate_continuity();
      return;
    }
    if (previous_pose_ && wall_now - last_publish_time_ > freshness_limit) {
      invalidate_continuity();
    }

    for (auto item = pending_.begin(); item != pending_.end();) {
      const auto stamp = item->first;
      const auto & sample = item->second;
      if (stamp <= last_published_stamp_ || sim_now - stamp > freshness_ns ||
        wall_now - sample.received_at > freshness_limit || pending_.size() > pending_limit)
      {
        item = pending_.erase(item);
        continue;
      }
      if (!sample.healthy || !sample.odometry || stamp > sim_now) {
        ++item;
        continue;
      }
      const auto odometry = sample.odometry;
      item = pending_.erase(item);
      if (!publish(*odometry, stamp, sim_now, wall_now)) {
        rejected_through_ = stamp;
        invalidate_continuity();
        return;
      }
    }
  }

  bool publish(const Odometry & odometry, int64_t stamp, int64_t sim_now,
    SteadyClock::time_point wall_now)
  {
    const auto & p = odometry.pose.pose.position;
    const auto & q = odometry.pose.pose.orientation;
    const double norm = std::hypot(std::hypot(q.w, q.x), std::hypot(q.y, q.z));
    const std::array<double, 3> position{p.x, -p.y, -p.z};
    if (odometry.header.frame_id != "odom" || odometry.child_frame_id != "base_link" ||
      !std::isfinite(norm) || std::abs(norm - 1.0) > 1e-3 ||
      !std::all_of(position.begin(), position.end(), [](double v) {
        return std::isfinite(v) && std::abs(v) <= std::numeric_limits<float>::max();
      }))
    {
      return false;
    }

    VehicleOdometry msg{};
    msg.timestamp_sample = static_cast<uint64_t>(stamp / 1000);
    msg.timestamp = static_cast<uint64_t>(sim_now / 1000);
    // The VIO reference has an arbitrary heading. Both world and body bases
    // change from FLU to FRD, so the quaternion is conjugated by a half roll.
    msg.pose_frame = VehicleOdometry::POSE_FRAME_FRD;
    msg.position = {static_cast<float>(p.x), static_cast<float>(-p.y), static_cast<float>(-p.z)};
    msg.q = {static_cast<float>(q.w / norm), static_cast<float>(q.x / norm),
      static_cast<float>(-q.y / norm), static_cast<float>(-q.z / norm)};

    // cuVSLAM's twist uses an older pose as the reference. Differentiate the
    // last two accepted positions in the same fixed local frame as position.
    msg.velocity_frame = VehicleOdometry::VELOCITY_FRAME_UNKNOWN;
    msg.velocity.fill(unavailable);
    if (previous_pose_) {
      const auto elapsed = stamp - previous_pose_->stamp;
      if (elapsed >= 1'000'000 && elapsed <= freshness_ns) {
        const double seconds = static_cast<double>(elapsed) * 1e-9;
        const auto & prev = previous_pose_->position;
        const std::array<double, 3> velocity{
          (position[0] - prev[0]) / seconds, (position[1] - prev[1]) / seconds,
          (position[2] - prev[2]) / seconds};
        if (std::all_of(velocity.begin(), velocity.end(), [](double v) {
            return std::isfinite(v) && std::abs(v) <= std::numeric_limits<float>::max();
          }))
        {
          msg.velocity_frame = VehicleOdometry::VELOCITY_FRAME_FRD;
          msg.velocity = {static_cast<float>(velocity[0]), static_cast<float>(velocity[1]),
            static_cast<float>(velocity[2])};
        }
      } else if (elapsed > freshness_ns) {
        reset_pending_ = true;
      }
    }

    // The odometry covariance is rolling pose/twist spread, not calibrated
    // measurement uncertainty. PX4 uses estimator noise settings for NaNs.
    msg.angular_velocity.fill(unavailable);
    msg.position_variance.fill(unavailable);
    msg.orientation_variance.fill(unavailable);
    msg.velocity_variance.fill(unavailable);
    if (reset_pending_) {
      ++reset_counter_;
      reset_pending_ = false;
      RCLCPP_WARN(get_logger(), "VIO continuity interrupted, reset counter %u",
        static_cast<unsigned>(reset_counter_));
    }
    msg.reset_counter = reset_counter_;
    output_->publish(msg);
    previous_pose_ = PreviousPose{stamp, position};
    last_published_stamp_ = stamp;
    last_publish_time_ = wall_now;
    published_ = true;
    return true;
  }

  std::map<int64_t, PendingSample> pending_;
  std::optional<PreviousPose> previous_pose_;
  std::optional<std::array<uint8_t, RMW_GID_STORAGE_SIZE>> publisher_gid_;
  int64_t last_clock_stamp_{0};
  int64_t last_published_stamp_{0};
  int64_t rejected_through_{0};
  SteadyClock::time_point last_clock_activity_{};
  SteadyClock::time_point last_publish_time_{};
  bool published_{false};
  bool reset_pending_{false};
  uint8_t reset_counter_{0};
  rclcpp::Publisher<VehicleOdometry>::SharedPtr output_;
  rclcpp::Subscription<Odometry>::SharedPtr odometry_;
  rclcpp::Subscription<DiagnosticArray>::SharedPtr diagnostics_;
  rclcpp::TimerBase::SharedPtr watchdog_;
};
}  // namespace autonomous_drone

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<autonomous_drone::ExternalVision>());
  rclcpp::shutdown();
  return 0;
}
