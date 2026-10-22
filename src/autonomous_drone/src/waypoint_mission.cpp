#include "autonomous_drone/waypoint_mission.hpp"
#include "autonomous_drone/px4_topic.hpp"

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace autonomous_drone
{
using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::VehicleCommandAck;
using px4_msgs::msg::VehicleLandDetected;
using px4_msgs::msg::VehicleLocalPosition;
using px4_msgs::msg::VehicleStatus;
using Trigger = std_srvs::srv::Trigger;
using Steady = std::chrono::steady_clock;
using Point = std::array<float, 3>;

class WaypointMission final : public rclcpp::Node
{
public:
  explicit WaypointMission(const rclcpp::NodeOptions & options)
  : Node("waypoint_mission", options)
  {
    auto_start_ = declare_parameter("auto_start", false);
    auto prefix = declare_parameter<std::string>("fmu_namespace", "/fmu");
    if (prefix.empty() || prefix.front() != '/') {
      throw std::invalid_argument("fmu_namespace must be an absolute ROS namespace");
    }
    while (!prefix.empty() && prefix.back() == '/') {prefix.pop_back();}
    target_system_ = id_parameter("target_system", 1, 255);
    target_component_ = id_parameter("target_component", 1, 255);
    source_system_ = id_parameter("source_system", 254, 255);
    source_component_ = id_parameter("source_component", 191, 255);
    rate_ = positive_parameter("control_rate_hz", 20.0);
    prestream_ = positive_parameter("prestream_s", 1.5);
    takeoff_height_ = positive_parameter("takeoff_height_m", 2.0);
    tolerance_ = positive_parameter("position_tolerance_m", 0.3);
    speed_tolerance_ = positive_parameter("speed_tolerance_m_s", 0.35);
    hold_time_ = positive_parameter("hold_time_s", 2.0, true);
    waypoint_timeout_ = positive_parameter("waypoint_timeout_s", 45.0);
    retry_time_ = positive_parameter("command_retry_s", 1.0);
    command_timeout_ = positive_parameter("command_timeout_s", 10.0);
    telemetry_timeout_ = positive_parameter("telemetry_timeout_s", 3.0);
    landing_timeout_ = positive_parameter("landing_timeout_s", 60.0);
    max_radius_ = positive_parameter("max_radius_m", 35.0);
    max_height_ = positive_parameter("max_height_m", 10.0);
    const auto offsets = declare_parameter<std::vector<double>>(
      "waypoint_offsets_m", {20., 0., 2., 20., 20., 2., 0., 20., 2., 0., 0., 2.});
    if (rate_ < 10.0 || rate_ > 100.0 || prestream_ < 1.0 ||
      takeoff_height_ <= tolerance_ || takeoff_height_ > max_height_ ||
      command_timeout_ <= retry_time_ || waypoint_timeout_ <= hold_time_ ||
      landing_timeout_ <= command_timeout_ || offsets.empty() || offsets.size() % 3 != 0)
    {
      throw std::invalid_argument("Invalid mission timing, limits, or waypoint triplets");
    }
    for (std::size_t i = 0; i < offsets.size(); i += 3) {
      const double north = offsets[i], east = offsets[i + 1], up = offsets[i + 2];
      if (!std::isfinite(north) || !std::isfinite(east) || !std::isfinite(up) ||
        std::hypot(north, east) > max_radius_ || up <= tolerance_ || up > max_height_)
      {
        throw std::invalid_argument("Waypoints must be finite and inside mission radius/height limits");
      }
      offsets_.push_back({static_cast<float>(north), static_cast<float>(east),
        static_cast<float>(-up)});
    }
    if (!has_parameter("use_sim_time")) {declare_parameter("use_sim_time", false);}
    if (get_parameter("use_sim_time").as_bool()) {
      throw std::invalid_argument("Use wall time on the Jetson: use_sim_time must be false");
    }
    // A running route is immutable. Restart the node to load another configuration.
    parameter_callback_ = add_on_set_parameters_callback(
      [](const std::vector<rclcpp::Parameter> &) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = false;
        result.reason = "Mission parameters are startup-only; restart to change them";
        return result;
      });

    heartbeat_pub_ = create_publisher<OffboardControlMode>(
      px4_topic<OffboardControlMode>(prefix, "/in/offboard_control_mode"), 10);
    trajectory_pub_ = create_publisher<TrajectorySetpoint>(
      px4_topic<TrajectorySetpoint>(prefix, "/in/trajectory_setpoint"), 10);
    command_pub_ = create_publisher<VehicleCommand>(
      px4_topic<VehicleCommand>(prefix, "/in/vehicle_command"), 10);
    const auto qos = rclcpp::SensorDataQoS().keep_last(10);
    status_sub_ = create_subscription<VehicleStatus>(
      px4_topic<VehicleStatus>(prefix, "/out/vehicle_status"), qos,
      [this](VehicleStatus::ConstSharedPtr msg) {
        if (msg->system_id == target_system_ && msg->component_id == target_component_ &&
          msg->timestamp > status_.timestamp)
        {
          status_ = *msg;
          status_received_ = Steady::now();
        }
      });
    position_sub_ = create_subscription<VehicleLocalPosition>(
      px4_topic<VehicleLocalPosition>(prefix, "/out/vehicle_local_position"), qos,
      [this](VehicleLocalPosition::ConstSharedPtr msg) {
        if (msg->timestamp > position_.timestamp) {
          position_ = *msg;
          position_received_ = Steady::now();
        }
      });
    land_sub_ = create_subscription<VehicleLandDetected>(
      px4_topic<VehicleLandDetected>(prefix, "/out/vehicle_land_detected"), qos,
      [this](VehicleLandDetected::ConstSharedPtr msg) {
        if (msg->timestamp > land_.timestamp) {
          land_ = *msg;
          land_received_ = Steady::now();
        }
      });
    ack_sub_ = create_subscription<VehicleCommandAck>(
      px4_topic<VehicleCommandAck>(prefix, "/out/vehicle_command_ack"), qos,
      [this](VehicleCommandAck::ConstSharedPtr msg) {
        if (pending_command_ == 0 || msg->command != pending_command_ ||
          msg->target_system != source_system_ || msg->target_component != source_component_)
        {
          return;
        }
        RCLCPP_INFO(get_logger(), "Command %u acknowledgement: result=%u, detail=%d",
          msg->command, msg->result, msg->result_param2);
        if (msg->result != VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED &&
          msg->result != VehicleCommandAck::VEHICLE_CMD_RESULT_IN_PROGRESS &&
          msg->result != VehicleCommandAck::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED)
        {
          command_error_ = "PX4 rejected command " + std::to_string(msg->command) +
            " (result " + std::to_string(msg->result) + ")";
        }
      });
    state_pub_ = create_publisher<std_msgs::msg::String>(
      "mission/state", rclcpp::QoS(1).transient_local());
    start_service_ = create_service<Trigger>("mission/start",
      [this](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response) {
        response->message = start();
        response->success = response->message.empty();
        if (response->success) {response->message = "Mission started";}
      });
    land_service_ = create_service<Trigger>("mission/land",
      [this](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response) {
        if (state_ == State::IDLE || terminal()) {
          response->message = "No active mission";
        } else if (state_ == State::LANDING) {
          response->success = true;
          response->message = "Landing already requested";
        } else if (!fresh(status_received_) || !offboard() || !armed()) {
          fail("Landing request cannot be sent with stale status or without armed Offboard control");
          response->success = true;
          response->message = "Mission cancelled; no landing command sent";
        } else {
          begin_landing("Operator requested landing", false);
          response->success = true;
          response->message = "Landing requested at current position";
        }
      });
    set_state(State::IDLE, "Ready; call /mission/start when PX4 telemetry is available");
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate_), [this]() {tick();});
  }

private:
  enum class State {IDLE, PRESTREAM, REQUEST_OFFBOARD, REQUEST_ARM, TAKEOFF, WAYPOINT,
    LANDING, COMPLETE, FAILED};

  static const char * name(State state)
  {
    switch (state) {
      case State::IDLE: return "IDLE";
      case State::PRESTREAM: return "PRESTREAM";
      case State::REQUEST_OFFBOARD: return "REQUEST_OFFBOARD";
      case State::REQUEST_ARM: return "REQUEST_ARM";
      case State::TAKEOFF: return "TAKEOFF";
      case State::WAYPOINT: return "WAYPOINT";
      case State::LANDING: return "LANDING";
      case State::COMPLETE: return "COMPLETE";
      case State::FAILED: return "FAILED";
    }
    return "UNKNOWN";
  }

  int id_parameter(const std::string & key, int value, int maximum)
  {
    const auto id = declare_parameter<int64_t>(key, value);
    if (id < 1 || id > maximum) {throw std::invalid_argument(key + " outside valid ID range");}
    return static_cast<int>(id);
  }

  double positive_parameter(const std::string & key, double value, bool zero_allowed = false)
  {
    const auto result = declare_parameter<double>(key, value);
    if (!std::isfinite(result) || result < 0 || (!zero_allowed && result == 0)) {
      throw std::invalid_argument(key + " must be finite and positive");
    }
    return result;
  }

  static double elapsed(Steady::time_point since)
  {return std::chrono::duration<double>(Steady::now() - since).count();}

  bool fresh(const std::optional<Steady::time_point> & received) const
  {return received && elapsed(*received) <= telemetry_timeout_;}

  bool armed() const {return status_.arming_state == VehicleStatus::ARMING_STATE_ARMED;}
  bool offboard() const {return status_.nav_state == VehicleStatus::NAVIGATION_STATE_OFFBOARD;}
  bool terminal() const {return state_ == State::COMPLETE || state_ == State::FAILED;}

  bool valid_position() const
  {
    return fresh(position_received_) && position_.xy_valid && position_.z_valid &&
      position_.v_xy_valid && position_.v_z_valid &&
      std::isfinite(position_.x) && std::isfinite(position_.y) && std::isfinite(position_.z) &&
      std::isfinite(position_.vx) && std::isfinite(position_.vy) && std::isfinite(position_.vz);
  }

  std::string readiness() const
  {
    if (!fresh(status_received_)) {return "Waiting for fresh VehicleStatus from the configured PX4 ID";}
    if (!valid_position()) {return "Waiting for a valid local position and velocity estimate";}
    if (!fresh(land_received_) || !land_.landed) {return "Vehicle must be landed";}
    if (status_.arming_state != VehicleStatus::ARMING_STATE_DISARMED) {
      return "Vehicle must be disarmed";
    }
    if (status_.vehicle_type != VehicleStatus::VEHICLE_TYPE_ROTARY_WING) {
      return "This mission supports multicopters only";
    }
    // After landing PX4 can retain Offboard as the selected mode. Its arming
    // checks then require our heartbeat, so allow prestream on the ground and
    // enforce passing checks again before sending any arm command.
    if (status_.failsafe || (!status_.pre_flight_checks_pass && !offboard())) {
      return "PX4 preflight checks are not ready";
    }
    // This flag describes final in-flight magnetic alignment in the pinned
    // PX4 revision. It can be false on the ground with preflight passing.
    if (!std::isfinite(position_.heading)) {
      return "Waiting for a valid heading";
    }
    return "";
  }

  std::string start()
  {
    if (state_ != State::IDLE) {return "Mission already started; restart node for another flight";}
    const auto reason = readiness();
    if (!reason.empty()) {return reason;}
    home_ = {position_.x, position_.y, position_.z};
    goal_ = home_;
    yaw_ = position_.heading;
    xy_reset_ = position_.xy_reset_counter;
    z_reset_ = position_.z_reset_counter;
    heading_reset_ = position_.heading_reset_counter;
    set_state(State::PRESTREAM, "Streaming offboard heartbeat before requesting control");
    return "";
  }

  void set_state(State state, const std::string & reason)
  {
    state_ = state;
    state_since_ = Steady::now();
    settled_since_.reset();
    last_command_.reset();
    pending_command_ = 0;
    command_error_.clear();
    std_msgs::msg::String message;
    message.data = name(state);
    state_pub_->publish(message);
    RCLCPP_INFO(get_logger(), "%s: %s", name(state), reason.c_str());
  }

  void fail(const std::string & reason)
  {
    set_state(State::FAILED, reason);
    RCLCPP_ERROR(get_logger(), "%s. Offboard output stopped; PX4/operator retains control.",
      reason.c_str());
  }

  void begin_landing(const std::string & reason, bool failed)
  {
    mission_failed_ = failed;
    if (valid_position()) {goal_ = {position_.x, position_.y, position_.z};}
    set_state(State::LANDING, reason);
  }

  uint64_t timestamp() {return static_cast<uint64_t>(get_clock()->now().nanoseconds() / 1000);}

  void publish_control()
  {
    OffboardControlMode heartbeat{};
    heartbeat.timestamp = timestamp();
    heartbeat.position = true;
    heartbeat_pub_->publish(heartbeat);
    // During prestream send ONLY the heartbeat. Internal PX4 controllers own
    // trajectory_setpoint outside Offboard, including during autonomous landing.
    if (offboard() && valid_position()) {
      const auto nan = std::numeric_limits<float>::quiet_NaN();
      TrajectorySetpoint setpoint{};
      setpoint.timestamp = timestamp();
      setpoint.position = goal_;
      setpoint.velocity.fill(nan);
      setpoint.acceleration.fill(nan);
      setpoint.jerk.fill(nan);
      setpoint.yaw = yaw_;
      setpoint.yawspeed = nan;
      trajectory_pub_->publish(setpoint);
    }
  }

  void command(uint32_t id, float param1 = 0.f, float param2 = 0.f)
  {
    if (last_command_ && elapsed(*last_command_) < retry_time_) {return;}
    VehicleCommand msg{};
    msg.timestamp = timestamp();
    msg.command = id;
    msg.param1 = param1;
    msg.param2 = param2;
    if (id == VehicleCommand::VEHICLE_CMD_NAV_LAND) {
      // NaN latitude/longitude/yaw mean land here, preserving current heading.
      msg.param4 = std::numeric_limits<float>::quiet_NaN();
      msg.param5 = std::numeric_limits<double>::quiet_NaN();
      msg.param6 = std::numeric_limits<double>::quiet_NaN();
      msg.param7 = std::numeric_limits<float>::quiet_NaN();
    }
    msg.target_system = static_cast<uint8_t>(target_system_);
    msg.target_component = static_cast<uint8_t>(target_component_);
    msg.source_system = static_cast<uint8_t>(source_system_);
    msg.source_component = static_cast<uint16_t>(source_component_);
    msg.from_external = true;
    command_pub_->publish(msg);
    pending_command_ = id;
    last_command_ = Steady::now();
    RCLCPP_INFO(get_logger(), "Sent command %u", id);
  }

  bool reached()
  {
    const double distance = std::hypot(std::hypot(position_.x - goal_[0],
        position_.y - goal_[1]), position_.z - goal_[2]);
    const double speed = std::hypot(std::hypot(position_.vx, position_.vy), position_.vz);
    if (distance > tolerance_ || speed > speed_tolerance_) {
      settled_since_.reset();
      return false;
    }
    if (!settled_since_) {settled_since_ = Steady::now();}
    return elapsed(*settled_since_) >= hold_time_;
  }

  void select_waypoint()
  {
    const auto & offset = offsets_[waypoint_index_];
    goal_ = {home_[0] + offset[0], home_[1] + offset[1], home_[2] + offset[2]};
    set_state(State::WAYPOINT, "Flying to waypoint " + std::to_string(waypoint_index_ + 1) +
      "/" + std::to_string(offsets_.size()));
  }

  void tick_landing()
  {
    if (fresh(land_received_) && land_.landed &&
      status_.arming_state == VehicleStatus::ARMING_STATE_DISARMED)
    {
      set_state(mission_failed_ ? State::FAILED : State::COMPLETE,
        "Landing and PX4 automatic disarm confirmed");
      return;
    }
    if (elapsed(state_since_) > landing_timeout_) {
      fail("Landing/disarm confirmation timed out; inspect PX4 landing state");
      return;
    }
    if (status_.nav_state == VehicleStatus::NAVIGATION_STATE_AUTO_LAND || !armed()) {
      return;  // PX4 owns descent and automatic disarm. Never disarm in flight.
    }
    if (!offboard()) {
      fail("Control changed during landing; respecting PX4/operator mode");
      return;
    }
    if (elapsed(state_since_) > command_timeout_) {
      fail("PX4 did not enter landing mode before command timeout");
      return;
    }
    publish_control();
    command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
  }

  void tick()
  {
    if (terminal()) {return;}
    if (state_ == State::IDLE) {
      if (auto_start_) {
        const auto reason = start();
        if (!reason.empty()) {
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "%s", reason.c_str());
        }
      }
      return;
    }
    if (!fresh(status_received_)) {fail("PX4 status telemetry timed out"); return;}
    if (status_.failsafe) {fail("PX4 activated a failsafe"); return;}
    if (!command_error_.empty()) {const auto error = command_error_; fail(error); return;}
    if (state_ == State::LANDING) {tick_landing(); return;}
    if (!valid_position()) {
      if (offboard() && armed() && fresh(position_received_)) {
        begin_landing("Local position estimate became invalid", true);
      } else {
        fail("Local position telemetry timed out or became invalid");
      }
      return;
    }
    const bool controlling = state_ == State::REQUEST_ARM || state_ == State::TAKEOFF ||
      state_ == State::WAYPOINT;
    if (controlling && !offboard()) {
      fail("PX4 left Offboard; respecting PX4/operator mode"); return;
    }
    if ((state_ == State::TAKEOFF || state_ == State::WAYPOINT) && !armed()) {
      fail("Vehicle disarmed during mission; automatic rearming is disabled"); return;
    }
    if ((state_ == State::PRESTREAM || state_ == State::REQUEST_OFFBOARD) && armed()) {
      fail("Vehicle was armed outside this mission"); return;
    }
    if (position_.xy_reset_counter != xy_reset_ || position_.z_reset_counter != z_reset_)
    {
      if (controlling && armed()) {begin_landing("Estimator reference reset; abandoning route", true);}
      else {fail("Estimator reference reset before takeoff");}
      return;
    }
    if (position_.heading_reset_counter != heading_reset_) {
      if (!std::isfinite(position_.delta_heading)) {
        if (controlling && armed()) {begin_landing("Invalid estimator heading reset", true);}
        else {fail("Invalid estimator heading reset before takeoff");}
        return;
      }
      // Preserve physical yaw through normal EKF magnetic alignment on takeoff.
      yaw_ = std::remainder(yaw_ + position_.delta_heading, 6.283185307179586f);
      heading_reset_ = position_.heading_reset_counter;
      RCLCPP_INFO(get_logger(), "Adjusted yaw target after estimator heading reset");
    }
    if (std::hypot(position_.x - home_[0], position_.y - home_[1]) > max_radius_ ||
      home_[2] - position_.z > max_height_)
    {
      if (controlling && armed()) {begin_landing("Vehicle exceeded mission limits", true);}
      else {fail("Vehicle moved beyond mission limits before takeoff");}
      return;
    }

    publish_control();
    switch (state_) {
      case State::PRESTREAM:
        if (elapsed(state_since_) >= prestream_) {
          set_state(State::REQUEST_OFFBOARD, "Requesting Offboard mode");
        }
        break;
      case State::REQUEST_OFFBOARD:
        if (offboard()) {set_state(State::REQUEST_ARM, "Offboard confirmed; requesting arm");}
        else if (elapsed(state_since_) > command_timeout_) {fail("Offboard mode request timed out");}
        else {command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.f, 6.f);}
        break;
      case State::REQUEST_ARM:
        if (armed()) {
          goal_ = {home_[0], home_[1], home_[2] - static_cast<float>(takeoff_height_)};
          set_state(State::TAKEOFF, "Arming confirmed; climbing vertically");
        } else if (elapsed(state_since_) > command_timeout_) {fail("Arming/preflight readiness timed out");}
        else if (status_.pre_flight_checks_pass) {
          command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.f);
        }
        break;
      case State::TAKEOFF:
      case State::WAYPOINT:
        if (elapsed(state_since_) > waypoint_timeout_) {
          begin_landing("Waypoint/takeoff timed out; landing at current position", true);
        } else if (reached()) {
          if (state_ == State::TAKEOFF) {select_waypoint();}
          else if (++waypoint_index_ < offsets_.size()) {select_waypoint();}
          else {begin_landing("All waypoints reached; landing at final point", false);}
        }
        break;
      default: break;
    }
  }

  State state_{State::IDLE};
  bool auto_start_{false}, mission_failed_{false};
  int target_system_{1}, target_component_{1}, source_system_{254}, source_component_{191};
  double rate_{}, prestream_{}, takeoff_height_{}, tolerance_{}, speed_tolerance_{}, hold_time_{};
  double waypoint_timeout_{}, retry_time_{}, command_timeout_{}, telemetry_timeout_{};
  double landing_timeout_{}, max_radius_{}, max_height_{};
  Point home_{}, goal_{};
  float yaw_{};
  uint8_t xy_reset_{}, z_reset_{}, heading_reset_{};
  std::vector<Point> offsets_;
  std::size_t waypoint_index_{};
  Steady::time_point state_since_{};
  std::optional<Steady::time_point> status_received_, position_received_, land_received_;
  std::optional<Steady::time_point> settled_since_, last_command_;
  uint32_t pending_command_{};
  std::string command_error_;
  VehicleStatus status_{};
  VehicleLocalPosition position_{};
  VehicleLandDetected land_{};
  rclcpp::Publisher<OffboardControlMode>::SharedPtr heartbeat_pub_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Subscription<VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<VehicleLocalPosition>::SharedPtr position_sub_;
  rclcpp::Subscription<VehicleLandDetected>::SharedPtr land_sub_;
  rclcpp::Subscription<VehicleCommandAck>::SharedPtr ack_sub_;
  rclcpp::Service<Trigger>::SharedPtr start_service_, land_service_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
  rclcpp::TimerBase::SharedPtr timer_;
};

std::shared_ptr<rclcpp::Node> make_waypoint_mission(const rclcpp::NodeOptions & options)
{return std::make_shared<WaypointMission>(options);}
}  // namespace autonomous_drone
