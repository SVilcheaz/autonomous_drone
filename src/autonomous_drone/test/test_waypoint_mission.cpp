#include <gtest/gtest.h>
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
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <thread>
#include <vector>

using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using autonomous_drone::px4_topic;
using Trigger = std_srvs::srv::Trigger;
using Clock = std::chrono::steady_clock;

TEST(Topics, MatchesPinnedMessageVersions)
{
  EXPECT_EQ(px4_topic<VehicleStatus>("/fmu", "/out/vehicle_status"), "/fmu/out/vehicle_status_v4");
  EXPECT_EQ(px4_topic<VehicleLocalPosition>("/fmu", "/out/vehicle_local_position"),
    "/fmu/out/vehicle_local_position_v1");
  EXPECT_EQ(px4_topic<VehicleCommandAck>("/fmu", "/out/vehicle_command_ack"),
    "/fmu/out/vehicle_command_ack_v1");
  EXPECT_EQ(px4_topic<OffboardControlMode>("/fmu", "/in/offboard_control_mode"),
    "/fmu/in/offboard_control_mode");
}

class MissionTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    options_.arguments({"--ros-args", "-r", "__ns:=/phase1_test"});
    options_.parameter_overrides({
      {"fmu_namespace", "/phase1_test/fmu"}, {"target_system", 241},
      {"source_system", 242}, {"control_rate_hz", 50.0}, {"prestream_s", 1.05},
      {"hold_time_s", 0.12}, {"waypoint_timeout_s", 3.0},
      {"command_retry_s", 0.1}, {"command_timeout_s", 0.7},
      {"telemetry_timeout_s", 0.3}, {"landing_timeout_s", 2.0}});
    mission_ = autonomous_drone::make_waypoint_mission(options_);
    peer_ = std::make_shared<rclcpp::Node>("fake_px4", "/phase1_test");
    const auto qos = rclcpp::SensorDataQoS().keep_last(10);
    status_pub_ = peer_->create_publisher<VehicleStatus>(
      px4_topic<VehicleStatus>(prefix_, "/out/vehicle_status"), qos);
    position_pub_ = peer_->create_publisher<VehicleLocalPosition>(
      px4_topic<VehicleLocalPosition>(prefix_, "/out/vehicle_local_position"), qos);
    land_pub_ = peer_->create_publisher<VehicleLandDetected>(
      px4_topic<VehicleLandDetected>(prefix_, "/out/vehicle_land_detected"), qos);
    ack_pub_ = peer_->create_publisher<VehicleCommandAck>(
      px4_topic<VehicleCommandAck>(prefix_, "/out/vehicle_command_ack"), qos);
    heartbeat_sub_ = peer_->create_subscription<OffboardControlMode>(
      px4_topic<OffboardControlMode>(prefix_, "/in/offboard_control_mode"), qos,
      [this](OffboardControlMode::ConstSharedPtr msg) {
        EXPECT_TRUE(msg->position);
        EXPECT_FALSE(msg->velocity);
        EXPECT_GT(msg->timestamp, 0u);
        if (heartbeat_count_++ == 0) {first_heartbeat_ = Clock::now();}
        if (recover_offboard_preflight_ && heartbeat_count_ >= 50) {
          status_.pre_flight_checks_pass = true;
        }
      });
    trajectory_sub_ = peer_->create_subscription<TrajectorySetpoint>(
      px4_topic<TrajectorySetpoint>(prefix_, "/in/trajectory_setpoint"), qos,
      [this](TrajectorySetpoint::ConstSharedPtr msg) {
        setpoints_.push_back(*msg);
        EXPECT_TRUE(std::isnan(msg->velocity[0]));
        EXPECT_TRUE(std::isnan(msg->acceleration[2]));
        EXPECT_TRUE(std::isnan(msg->yawspeed));
        EXPECT_TRUE(std::isfinite(msg->yaw));
        if (!mode_seen_) {++pre_mode_setpoints_;}
        if (follow_ && status_.arming_state == VehicleStatus::ARMING_STATE_ARMED) {
          position_.x = msg->position[0];
          position_.y = msg->position[1];
          position_.z = msg->position[2];
        }
      });
    command_sub_ = peer_->create_subscription<VehicleCommand>(
      px4_topic<VehicleCommand>(prefix_, "/in/vehicle_command"), 10,
      [this](VehicleCommand::ConstSharedPtr msg) {
        commands_.push_back(*msg);
        EXPECT_EQ(msg->target_system, 241);
        EXPECT_EQ(msg->source_system, 242);
        EXPECT_TRUE(msg->from_external);
        uint8_t result = VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED;
        if (msg->command == VehicleCommand::VEHICLE_CMD_DO_SET_MODE) {
          EXPECT_GE(std::chrono::duration<double>(Clock::now() - first_heartbeat_).count(), 1.0);
          EXPECT_FLOAT_EQ(msg->param1, 1.f);
          EXPECT_FLOAT_EQ(msg->param2, 6.f);
          mode_seen_ = true;
          if (temporary_reject_mode_) {
            temporary_reject_mode_ = false;
            result = VehicleCommandAck::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
          } else if (allow_mode_) {status_.nav_state = VehicleStatus::NAVIGATION_STATE_OFFBOARD;}
        } else if (msg->command == VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM) {
          EXPECT_EQ(status_.nav_state, VehicleStatus::NAVIGATION_STATE_OFFBOARD);
          EXPECT_TRUE(status_.pre_flight_checks_pass);
          EXPECT_FLOAT_EQ(msg->param1, 1.f);  // No forced/in-flight disarm commands.
          if (reject_arm_) {result = VehicleCommandAck::VEHICLE_CMD_RESULT_DENIED;}
          else if (allow_arm_) {
            status_.arming_state = VehicleStatus::ARMING_STATE_ARMED;
            land_.landed = false;
          }
        } else if (msg->command == VehicleCommand::VEHICLE_CMD_NAV_LAND) {
          EXPECT_TRUE(std::isnan(msg->param5));
          EXPECT_TRUE(std::isnan(msg->param6));
          status_.nav_state = VehicleStatus::NAVIGATION_STATE_AUTO_LAND;
        }
        VehicleCommandAck ack{};
        ack.timestamp = peer_->now().nanoseconds() / 1000;
        ack.command = msg->command;
        ack.result = result;
        ack.target_system = msg->source_system;
        ack.target_component = msg->source_component;
        ack.from_external = true;
        ack_pub_->publish(ack);
      });
    state_sub_ = peer_->create_subscription<std_msgs::msg::String>(
      "mission/state", rclcpp::QoS(1).transient_local(),
      [this](std_msgs::msg::String::ConstSharedPtr msg) {state_ = msg->data;});
    start_client_ = peer_->create_client<Trigger>("mission/start");
    land_client_ = peer_->create_client<Trigger>("mission/land");
    status_.system_id = 241;
    status_.component_id = 1;
    status_.vehicle_type = VehicleStatus::VEHICLE_TYPE_ROTARY_WING;
    status_.arming_state = VehicleStatus::ARMING_STATE_DISARMED;
    status_.nav_state = VehicleStatus::NAVIGATION_STATE_POSCTL;
    status_.pre_flight_checks_pass = true;
    position_.x = 10.f;
    position_.y = 20.f;
    position_.z = -0.4f;
    position_.heading = 0.7f;
    position_.heading_good_for_control = false;  // Normal before in-flight alignment.
    position_.xy_valid = position_.z_valid = position_.v_xy_valid = position_.v_z_valid = true;
    land_.landed = true;
    timer_ = peer_->create_wall_timer(20ms, [this]() {
      const auto timestamp = peer_->now().nanoseconds() / 1000;
      status_.timestamp = position_.timestamp = land_.timestamp = timestamp;
      if (status_.nav_state == VehicleStatus::NAVIGATION_STATE_AUTO_LAND) {
        if (++landing_ticks_ >= 5) {land_.landed = true;}
        if (landing_ticks_ >= 12 && auto_disarm_) {
          status_.arming_state = VehicleStatus::ARMING_STATE_DISARMED;
        }
      }
      if (publish_status_) {status_pub_->publish(status_);}
      if (publish_position_) {position_pub_->publish(position_);}
      land_pub_->publish(land_);
    });
    executor_.add_node(mission_);
    executor_.add_node(peer_);
    spin_for(250ms);
  }

  void TearDown() override
  {
    executor_.remove_node(mission_);
    executor_.remove_node(peer_);
  }

  bool wait(const std::function<bool()> & condition, double seconds = 5.0)
  {
    const auto deadline = Clock::now() + std::chrono::duration<double>(seconds);
    while (Clock::now() < deadline) {
      executor_.spin_some();
      if (condition()) {return true;}
      std::this_thread::sleep_for(2ms);
    }
    return condition();
  }

  void spin_for(std::chrono::milliseconds duration)
  {
    const auto deadline = Clock::now() + duration;
    wait([deadline]() {return Clock::now() >= deadline;}, duration.count() / 1000.0 + 0.1);
  }

  Trigger::Response::SharedPtr call(const rclcpp::Client<Trigger>::SharedPtr & client)
  {
    EXPECT_TRUE(wait([&]() {return client->service_is_ready();}));
    auto future = client->async_send_request(std::make_shared<Trigger::Request>());
    if (!wait([&]() {return future.wait_for(0s) == std::future_status::ready;})) {
      ADD_FAILURE() << "Service timed out";
      return std::make_shared<Trigger::Response>();
    }
    return future.get();
  }

  bool sent(uint32_t command) const
  {
    return std::any_of(commands_.begin(), commands_.end(),
      [command](const auto & msg) {return msg.command == command;});
  }

  rclcpp::NodeOptions options_;
  rclcpp::Node::SharedPtr mission_, peer_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  const std::string prefix_{"/phase1_test/fmu"};
  VehicleStatus status_{};
  VehicleLocalPosition position_{};
  VehicleLandDetected land_{};
  std::vector<VehicleCommand> commands_;
  std::vector<TrajectorySetpoint> setpoints_;
  std::string state_;
  std::size_t heartbeat_count_{}, pre_mode_setpoints_{};
  int landing_ticks_{};
  bool follow_{true}, allow_mode_{true}, allow_arm_{true}, reject_arm_{false};
  bool publish_status_{true}, publish_position_{true}, auto_disarm_{true};
  bool mode_seen_{false}, temporary_reject_mode_{false};
  bool recover_offboard_preflight_{false};
  Clock::time_point first_heartbeat_{};
  rclcpp::Publisher<VehicleStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<VehicleLocalPosition>::SharedPtr position_pub_;
  rclcpp::Publisher<VehicleLandDetected>::SharedPtr land_pub_;
  rclcpp::Publisher<VehicleCommandAck>::SharedPtr ack_pub_;
  rclcpp::Subscription<OffboardControlMode>::SharedPtr heartbeat_sub_;
  rclcpp::Subscription<TrajectorySetpoint>::SharedPtr trajectory_sub_;
  rclcpp::Subscription<VehicleCommand>::SharedPtr command_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
  rclcpp::Client<Trigger>::SharedPtr start_client_, land_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

TEST_F(MissionTest, RemainsIdleUntilExplicitStartAndRejectsInvalidPreflight)
{
  EXPECT_EQ(state_, "IDLE");
  EXPECT_TRUE(commands_.empty());
  EXPECT_EQ(heartbeat_count_, 0u);
  status_.pre_flight_checks_pass = false;
  spin_for(60ms);
  EXPECT_FALSE(call(start_client_)->success);
  EXPECT_TRUE(commands_.empty());
}

TEST_F(MissionTest, TakesOffFliesTwentyMetreSquareAndConfirmsDisarm)
{
  ASSERT_TRUE(call(start_client_)->success);
  ASSERT_TRUE(wait([this]() {return state_ == "COMPLETE";}));
  EXPECT_EQ(pre_mode_setpoints_, 0u);
  for (const auto & expected : std::vector<std::array<float, 3>>{
      {10.f, 20.f, -2.4f}, {30.f, 20.f, -2.4f}, {30.f, 40.f, -2.4f},
      {10.f, 40.f, -2.4f}, {10.f, 20.f, -2.4f}})
  {
    EXPECT_TRUE(std::any_of(setpoints_.begin(), setpoints_.end(), [&](const auto & msg) {
      return std::abs(msg.position[0] - expected[0]) < 0.001f &&
             std::abs(msg.position[1] - expected[1]) < 0.001f &&
             std::abs(msg.position[2] - expected[2]) < 0.001f;
    }));
  }
  EXPECT_TRUE(sent(VehicleCommand::VEHICLE_CMD_NAV_LAND));
  EXPECT_EQ(status_.arming_state, VehicleStatus::ARMING_STATE_DISARMED);
  EXPECT_FALSE(call(start_client_)->success);
}

TEST_F(MissionTest, RestoresHeartbeatBeforeArmingAfterPreviousOffboardLanding)
{
  status_.nav_state = VehicleStatus::NAVIGATION_STATE_OFFBOARD;
  status_.pre_flight_checks_pass = false;
  recover_offboard_preflight_ = true;
  spin_for(60ms);
  ASSERT_TRUE(call(start_client_)->success);
  ASSERT_TRUE(wait([this]() {return state_ == "COMPLETE";}));
  EXPECT_TRUE(sent(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM));
}

TEST_F(MissionTest, NeverArmsIfOffboardPreflightRemainsUnready)
{
  status_.nav_state = VehicleStatus::NAVIGATION_STATE_OFFBOARD;
  status_.pre_flight_checks_pass = false;
  spin_for(60ms);
  ASSERT_TRUE(call(start_client_)->success);
  ASSERT_TRUE(wait([this]() {return state_ == "FAILED";}));
  EXPECT_FALSE(sent(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM));
}

TEST_F(MissionTest, AcceptedModeAckDoesNotArmWithoutModeFeedback)
{
  allow_mode_ = false;
  ASSERT_TRUE(call(start_client_)->success);
  ASSERT_TRUE(wait([this]() {return state_ == "FAILED";}));
  EXPECT_FALSE(sent(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM));
  EXPECT_TRUE(setpoints_.empty());
}

TEST_F(MissionTest, AcceptedArmAckDoesNotTakeOffWithoutArmingFeedback)
{
  allow_arm_ = false;
  ASSERT_TRUE(call(start_client_)->success);
  ASSERT_TRUE(wait([this]() {return state_ == "FAILED";}));
  ASSERT_FALSE(setpoints_.empty());
  for (const auto & msg : setpoints_) {EXPECT_FLOAT_EQ(msg.position[2], -0.4f);}
}

TEST_F(MissionTest, RetriesTemporaryCommandRejection)
{
  temporary_reject_mode_ = true;
  ASSERT_TRUE(call(start_client_)->success);
  ASSERT_TRUE(wait([this]() {return state_ == "COMPLETE";}));
  EXPECT_GE(std::count_if(commands_.begin(), commands_.end(), [](const auto & msg) {
    return msg.command == VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
  }), 2);
}

TEST_F(MissionTest, StopsOnPermanentArmingRejection)
{
  reject_arm_ = true;
  ASSERT_TRUE(call(start_client_)->success);
  ASSERT_TRUE(wait([this]() {return state_ == "FAILED";}));
  EXPECT_EQ(status_.arming_state, VehicleStatus::ARMING_STATE_DISARMED);
  EXPECT_FALSE(sent(VehicleCommand::VEHICLE_CMD_NAV_LAND));
}

TEST_F(MissionTest, DoesNotMaskTelemetryLossWithContinuedHeartbeats)
{
  follow_ = false;
  ASSERT_TRUE(call(start_client_)->success);
  ASSERT_TRUE(wait([this]() {return state_ == "TAKEOFF";}));
  publish_status_ = false;
  ASSERT_TRUE(wait([this]() {return state_ == "FAILED";}, 1.0));
  spin_for(50ms);
  const auto count = heartbeat_count_;
  spin_for(150ms);
  EXPECT_EQ(heartbeat_count_, count);
}

TEST_F(MissionTest, RespectsOperatorModeChangeAndNeverRearms)
{
  follow_ = false;
  ASSERT_TRUE(call(start_client_)->success);
  ASSERT_TRUE(wait([this]() {return state_ == "TAKEOFF";}));
  status_.nav_state = VehicleStatus::NAVIGATION_STATE_POSCTL;
  ASSERT_TRUE(wait([this]() {return state_ == "FAILED";}, 1.0));
  const auto count = commands_.size();
  spin_for(200ms);
  EXPECT_EQ(commands_.size(), count);
  EXPECT_FALSE(sent(VehicleCommand::VEHICLE_CMD_NAV_LAND));
}

TEST_F(MissionTest, WaypointTimeoutLandsAndReportsFailure)
{
  follow_ = false;
  ASSERT_TRUE(call(start_client_)->success);
  ASSERT_TRUE(wait([this]() {return state_ == "FAILED";}, 7.0));
  EXPECT_TRUE(sent(VehicleCommand::VEHICLE_CMD_NAV_LAND));
  EXPECT_EQ(status_.arming_state, VehicleStatus::ARMING_STATE_DISARMED);
}

TEST_F(MissionTest, EstimatorResetAbandonsRouteAndLands)
{
  follow_ = false;
  ASSERT_TRUE(call(start_client_)->success);
  ASSERT_TRUE(wait([this]() {return state_ == "TAKEOFF";}));
  ++position_.xy_reset_counter;
  ASSERT_TRUE(wait([this]() {return state_ == "FAILED";}));
  EXPECT_TRUE(sent(VehicleCommand::VEHICLE_CMD_NAV_LAND));
}

TEST_F(MissionTest, PreservesYawThroughNormalInFlightHeadingAlignment)
{
  follow_ = false;
  ASSERT_TRUE(call(start_client_)->success);
  ASSERT_TRUE(wait([this]() {return state_ == "TAKEOFF";}));
  ++position_.heading_reset_counter;
  position_.delta_heading = 0.2f;
  spin_for(100ms);
  ASSERT_FALSE(setpoints_.empty());
  EXPECT_EQ(state_, "TAKEOFF");
  EXPECT_NEAR(setpoints_.back().yaw, 0.9f, 0.001f);
}

TEST_F(MissionTest, OperatorCanLandEarlyAndSetpointsStopDuringPx4Landing)
{
  follow_ = false;
  auto_disarm_ = false;
  ASSERT_TRUE(call(start_client_)->success);
  ASSERT_TRUE(wait([this]() {return state_ == "TAKEOFF";}));
  ASSERT_TRUE(call(land_client_)->success);
  ASSERT_TRUE(wait([this]() {return land_.landed;}));
  spin_for(80ms);
  const auto count = setpoints_.size();
  const auto heartbeats = heartbeat_count_;
  spin_for(180ms);
  EXPECT_EQ(setpoints_.size(), count);
  EXPECT_EQ(heartbeat_count_, heartbeats);
  EXPECT_EQ(state_, "LANDING");  // Touchdown alone is insufficient.
  status_.arming_state = VehicleStatus::ARMING_STATE_DISARMED;
  EXPECT_TRUE(wait([this]() {return state_ == "COMPLETE";}));
}

TEST_F(MissionTest, DoesNotAdvanceWaypointWhileStillMoving)
{
  position_.vx = 2.f;
  ASSERT_TRUE(call(start_client_)->success);
  ASSERT_TRUE(wait([this]() {return state_ == "TAKEOFF";}));
  spin_for(350ms);
  EXPECT_EQ(state_, "TAKEOFF");
  position_.vx = 0.f;
  EXPECT_TRUE(wait([this]() {return state_ == "COMPLETE";}));
}

TEST_F(MissionTest, RejectsInvalidMissionConfiguration)
{
  for (const auto & parameter : std::vector<rclcpp::Parameter>{
      {"waypoint_offsets_m", std::vector<double>{1., 2.}},
      {"waypoint_offsets_m", std::vector<double>{100., 0., 2.}},
      {"waypoint_offsets_m", std::vector<double>{0., 0., -2.}},
      {"control_rate_hz", 1.0}, {"prestream_s", 0.1}, {"takeoff_height_m", -1.0},
      {"use_sim_time", true}})
  {
    auto options = options_;
    auto overrides = options.parameter_overrides();
    overrides.erase(std::remove_if(overrides.begin(), overrides.end(), [&](const auto & p) {
      return p.get_name() == parameter.get_name();
    }), overrides.end());
    overrides.push_back(parameter);
    options.parameter_overrides(overrides);
    EXPECT_THROW(autonomous_drone::make_waypoint_mission(options), std::invalid_argument);
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
