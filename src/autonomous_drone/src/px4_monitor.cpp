#include <chrono>
#include <cmath>
#include <cstddef>
#include <memory>

#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;
using VehicleStatus = px4_msgs::msg::VehicleStatus;
using VehicleLocalPosition = px4_msgs::msg::VehicleLocalPosition;
using SteadyClock = std::chrono::steady_clock;

class Px4Monitor : public rclcpp::Node
{
public:
  Px4Monitor() : Node("px4_monitor")
  {
    declare_parameter("telemetry_timeout_s", 3.0);

    // PX4 telemetry uses best-effort delivery. Match it with sensor-data QoS.
    const auto qos = rclcpp::SensorDataQoS();

    status_subscription_ = create_subscription<VehicleStatus>(
      "/fmu/out/vehicle_status_v4", qos,
      [this](VehicleStatus::ConstSharedPtr message) {
        status_ = *message;
        last_status_time_ = SteadyClock::now();
        have_status_ = true;
        ++status_samples_;
      });

    position_subscription_ = create_subscription<VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position_v1", qos,
      [this](VehicleLocalPosition::ConstSharedPtr message) {
        position_ = *message;
        last_position_time_ = SteadyClock::now();
        have_position_ = true;
        ++position_samples_;
      });

    // Incoming messages update the cache; this timer reports it at 1 Hz.
    report_timer_ = create_wall_timer(1s, [this]() {report();});

    RCLCPP_INFO(get_logger(), "PX4 monitor started; waiting for status and position.");
  }

private:
  void report()
  {
    if (!have_status_ || !have_position_) {
      RCLCPP_WARN(get_logger(), "WAITING: status=%s position=%s",
        have_status_ ? "received" : "missing",
        have_position_ ? "received" : "missing");
      return;
    }

    // Read on every report so `ros2 param set` takes effect while running.
    const double timeout_s = get_parameter("telemetry_timeout_s").as_double();
    if (!std::isfinite(timeout_s) || timeout_s <= 0.0) {
      RCLCPP_WARN(get_logger(), "telemetry_timeout_s must be finite and positive.");
      return;
    }

    // These are local reception times, independent of PX4's timestamp clock.
    const auto now = SteadyClock::now();
    const double status_age = std::chrono::duration<double>(now - last_status_time_).count();
    const double position_age = std::chrono::duration<double>(now - last_position_time_).count();

    if (status_age > timeout_s || position_age > timeout_s) {
      RCLCPP_WARN(get_logger(), "STALE: status_age=%.1f s position_age=%.1f s limit=%.1f s",
        status_age, position_age, timeout_s);
      return;
    }

    const bool armed = status_.arming_state == VehicleStatus::ARMING_STATE_ARMED;
    RCLCPP_INFO(get_logger(), "armed=%s nav_state=%d preflight=%s failsafe=%s",
      armed ? "YES" : "NO", status_.nav_state,
      status_.pre_flight_checks_pass ? "PASS" : "BLOCKED",
      status_.failsafe ? "YES" : "NO");

    const bool estimates_valid = position_.xy_valid && position_.z_valid &&
      position_.v_xy_valid && position_.v_z_valid;
    const bool values_finite = std::isfinite(position_.x) && std::isfinite(position_.y) &&
      std::isfinite(position_.z) && std::isfinite(position_.vx) &&
      std::isfinite(position_.vy) && std::isfinite(position_.vz);

    if (!estimates_valid || !values_finite) {
      RCLCPP_WARN(get_logger(), "INVALID: local position or velocity estimate is not usable.");
      return;
    }

    const double speed_xy = std::hypot(position_.vx, position_.vy);
    RCLCPP_INFO(get_logger(),
      "telemetry=OK N=%.2f m E=%.2f m D=%.2f m speed_xy=%.2f m/s samples=%zu/%zu",
      position_.x, position_.y, position_.z, speed_xy, status_samples_, position_samples_);
  }

  VehicleStatus status_{};
  VehicleLocalPosition position_{};
  bool have_status_{false};
  bool have_position_{false};
  std::size_t status_samples_{0};
  std::size_t position_samples_{0};
  SteadyClock::time_point last_status_time_{};
  SteadyClock::time_point last_position_time_{};

  // Keep these handles alive for as long as the node needs their callbacks.
  rclcpp::Subscription<VehicleStatus>::SharedPtr status_subscription_;
  rclcpp::Subscription<VehicleLocalPosition>::SharedPtr position_subscription_;
  rclcpp::TimerBase::SharedPtr report_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Px4Monitor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
