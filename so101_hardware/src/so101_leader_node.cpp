// Copyright 2026 Jonathan Darko Adoo
// SPDX-License-Identifier: BSD-3-Clause

// Reads the SO-101 leader arm (six STS3215 servos, LeRobot calibration) and
// publishes its pose on leader/joint_states in the follower's joint
// convention. With teleop:=true it also streams that pose to the follower's
// position controllers through a TeleopFilter: the follower first moves to
// the leader's pose at sync_velocity, then tracks it at up to max_velocity.
// If the leader stops answering, no commands are sent (the follower holds its
// last command) and the follower re-syncs slowly once it answers again.
//
// The leader is only read: no register is ever written, torque stays off.

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "so101_hardware/calibration.hpp"
#include "so101_hardware/feetech_bus.hpp"
#include "so101_hardware/serial_port.hpp"
#include "so101_hardware/teleop_filter.hpp"

namespace so101_hardware
{
namespace
{
// Same joints, order and mapping as the follower's ros2_control block.
const std::vector<std::string> kJoints = {
  "shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"};
constexpr size_t kArmJoints = 5;
constexpr double kGripperClosed = -0.174533;
constexpr double kGripperOpen = 1.74533;

std::string default_calibration_file(const std::string & leader_id)
{
  const char * lerobot_home = std::getenv("HF_LEROBOT_HOME");
  const char * home = std::getenv("HOME");
  const std::string base = lerobot_home ? lerobot_home :
    std::string(home ? home : "") + "/.cache/huggingface/lerobot";
  return base + "/calibration/teleoperators/so_leader/" + leader_id + ".json";
}
}  // namespace

class LeaderNode : public rclcpp::Node
{
public:
  LeaderNode()
  : Node("so101_leader"),
    filter_(
      declare_parameter("sync_velocity", 0.5),
      declare_parameter("max_velocity", 3.0),
      declare_parameter("sync_tolerance", 0.05))
  {
    const std::string port = declare_parameter("port", "/dev/ttyACM1");
    const std::string leader_id = declare_parameter("leader_id", "my_leader");
    std::string calibration_file = declare_parameter("calibration_file", "");
    const int baud_rate = declare_parameter("baud_rate", 1000000);
    const double rate_hz = declare_parameter("rate_hz", 100.0);
    max_read_errors_ = declare_parameter("max_consecutive_read_errors", 5);
    teleop_ = declare_parameter("teleop", false);
    if (calibration_file.empty()) {
      calibration_file = default_calibration_file(leader_id);
    }

    const auto calibration = load_lerobot_calibration(calibration_file);
    std::string error;
    if (!serial_.open(port, baud_rate, &error)) {
      throw std::runtime_error(error);
    }
    bus_ = std::make_unique<FeetechBus>(serial_, std::chrono::milliseconds(10));
    for (const auto & name : kJoints) {
      auto it = calibration.find(name);
      if (it == calibration.end()) {
        throw std::runtime_error("'" + name + "' missing from " + calibration_file);
      }
      verify_servo(name, it->second, calibration_file, port);
      ids_.push_back(static_cast<uint8_t>(it->second.id));
      mappings_.push_back(
        name == "gripper" ? JointMapping::linear(it->second, kGripperClosed, kGripperOpen) :
        JointMapping::centered(it->second));
    }
    RCLCPP_INFO(
      get_logger(), "Leader on %s: all %zu servos answered and match %s", port.c_str(),
      kJoints.size(), calibration_file.c_str());

    state_pub_ = create_publisher<sensor_msgs::msg::JointState>("leader/joint_states", 10);
    if (teleop_) {
      const std::string controller_manager =
        declare_parameter("controller_manager", "/controller_manager");
      controllers_ = {
        declare_parameter("arm_controller", "arm_forward_controller"),
        declare_parameter("gripper_controller", "gripper_forward_controller")};
      arm_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        "/" + controllers_[0] + "/commands", 10);
      gripper_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        "/" + controllers_[1] + "/commands", 10);
      list_controllers_ = create_client<controller_manager_msgs::srv::ListControllers>(
        controller_manager + "/list_controllers");
      follower_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        declare_parameter("follower_joint_states_topic", "/joint_states"), 10,
        [this](const sensor_msgs::msg::JointState & msg) {
          for (size_t i = 0; i < msg.name.size() && i < msg.position.size(); ++i) {
            follower_[msg.name[i]] = msg.position[i];
          }
        });
    }
    last_tick_ = now();
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_hz), [this]() {tick();});
  }

private:
  void verify_servo(
    const std::string & name, const MotorCalibration & c, const std::string & file,
    const std::string & port)
  {
    const auto id = static_cast<uint8_t>(c.id);
    if (!bus_->ping(id)) {
      throw std::runtime_error(
              "No reply from leader servo " + std::to_string(c.id) + " (" + name + ") on " +
              port + ": " + bus_->last_error());
    }
    uint32_t offset_raw = 0, min = 0, max = 0;
    if (!bus_->read(id, feetech::reg::kHomingOffset, &offset_raw) ||
      !bus_->read(id, feetech::reg::kMinPositionLimit, &min) ||
      !bus_->read(id, feetech::reg::kMaxPositionLimit, &max))
    {
      throw std::runtime_error("Cannot read calibration of leader servo " + name);
    }
    const int offset = feetech::decode_sign_magnitude(offset_raw, feetech::kHomingOffsetSignBit);
    if (offset != c.homing_offset || static_cast<int>(min) != c.range_min ||
      static_cast<int>(max) != c.range_max)
    {
      // Also catches swapped ports: the follower's servos hold the follower's
      // calibration, not the leader's.
      throw std::runtime_error(
              "Servo " + std::to_string(c.id) + " (" + name + ") on " + port +
              " does not match " + file + ": servo has offset=" + std::to_string(offset) +
              " range=[" + std::to_string(min) + ", " + std::to_string(max) +
              "], file has offset=" + std::to_string(c.homing_offset) + " range=[" +
              std::to_string(c.range_min) + ", " + std::to_string(c.range_max) +
              "]. Is the leader on this port? Check with so101_hardware_check.");
    }
  }

  void tick()
  {
    const rclcpp::Time stamp = now();
    const double dt = std::clamp((stamp - last_tick_).seconds(), 0.0, 0.1);
    last_tick_ = stamp;

    std::map<uint8_t, std::vector<uint8_t>> data;
    if (!bus_->sync_read(feetech::reg::kPresentPosition, ids_, &data)) {
      if (++read_errors_ == max_read_errors_) {
        RCLCPP_ERROR(
          get_logger(), "Leader not answering (%s); the follower holds its position",
          bus_->last_error().c_str());
        filter_.deactivate();
      }
      return;
    }
    if (read_errors_ >= max_read_errors_) {
      RCLCPP_INFO(get_logger(), "Leader answering again");
    }
    read_errors_ = 0;

    sensor_msgs::msg::JointState state;
    state.header.stamp = stamp;
    state.name = kJoints;
    for (size_t i = 0; i < ids_.size(); ++i) {
      const int ticks = feetech::decode_sign_magnitude(
        feetech::from_bytes(data[ids_[i]].data(), 2), feetech::kPositionSignBit);
      state.position.push_back(mappings_[i].ticks_to_position(ticks));
    }
    state_pub_->publish(state);
    if (teleop_) {
      teleoperate(state.position, dt);
    }
  }

  void teleoperate(const std::vector<double> & leader, double dt)
  {
    if (!filter_.active()) {
      // Commands sent before the position controllers are active are
      // dropped, and the ramp would then reach the follower as a jump.
      if (!controllers_active_) {
        poll_controllers();
        return;
      }
      std::vector<double> follower;
      for (const auto & name : kJoints) {
        auto it = follower_.find(name);
        if (it == follower_.end()) {
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 5000, "Waiting for the follower's joint states");
          return;
        }
        follower.push_back(it->second);
      }
      filter_.reset(follower);
      RCLCPP_INFO(get_logger(), "Moving the follower to the leader's pose");
    }
    const bool was_synced = filter_.synced();
    const auto & command = filter_.update(leader, dt);
    if (filter_.synced() && !was_synced) {
      RCLCPP_INFO(get_logger(), "Follower matches the leader; teleoperating");
    }
    std_msgs::msg::Float64MultiArray arm, gripper;
    arm.data.assign(command.begin(), command.begin() + kArmJoints);
    gripper.data.assign(command.begin() + kArmJoints, command.end());
    arm_pub_->publish(arm);
    gripper_pub_->publish(gripper);
  }

  void poll_controllers()
  {
    using controller_manager_msgs::srv::ListControllers;
    if (list_request_pending_ || !list_controllers_->service_is_ready()) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000, "Waiting for the follower's controllers (%s, %s)",
        controllers_[0].c_str(), controllers_[1].c_str());
      return;
    }
    list_request_pending_ = true;
    list_controllers_->async_send_request(
      std::make_shared<ListControllers::Request>(),
      [this](rclcpp::Client<ListControllers>::SharedFuture future) {
        list_request_pending_ = false;
        size_t active = 0;
        for (const auto & controller : future.get()->controller) {
          if (controller.state == "active" &&
          std::find(controllers_.begin(), controllers_.end(), controller.name) !=
          controllers_.end())
          {
            ++active;
          }
        }
        controllers_active_ = active == controllers_.size();
      });
  }

  TeleopFilter filter_;
  SerialPort serial_;
  std::unique_ptr<FeetechBus> bus_;
  std::vector<uint8_t> ids_;
  std::vector<JointMapping> mappings_;
  bool teleop_ = false;
  int max_read_errors_ = 5;
  int read_errors_ = 0;
  rclcpp::Time last_tick_;
  std::vector<std::string> controllers_;
  bool controllers_active_ = false;
  bool list_request_pending_ = false;
  rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr list_controllers_;
  std::map<std::string, double> follower_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr arm_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gripper_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr follower_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace so101_hardware

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int status = 0;
  try {
    rclcpp::spin(std::make_shared<so101_hardware::LeaderNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("so101_leader"), "%s", e.what());
    status = 1;
  }
  rclcpp::shutdown();
  return status;
}
