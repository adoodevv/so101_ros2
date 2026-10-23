// Copyright 2026 Jonathan Darko Adoo
// SPDX-License-Identifier: BSD-3-Clause

#include "so101_hardware/so101_system.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace so101_hardware
{

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;
namespace reg = feetech::reg;

namespace
{
constexpr char kLoad[] = "load";
constexpr char kTemperature[] = "temperature";
constexpr char kVoltage[] = "voltage";
// write() clamps commands to each joint's limits, so a joint resting outside
// them is driven to the limit as soon as torque comes on. A small overshoot
// (the folded rest pose sits a few degrees past shoulder_lift's URDF limit) is
// accepted with a warning; a larger one is refused.
constexpr double kActivationLimitTolerance = 0.1;  // rad
// Commands stop this far inside the URDF limits. MoveIt refuses to plan from a
// start state even one tick past a limit, and a servo holding a goal at the
// limit reads a tick or two either side of it (0.0015 rad per tick).
constexpr double kUrdfLimitMargin = 0.005;  // rad

std::string param_or(
  const std::unordered_map<std::string, std::string> & params,
  const std::string & key, const std::string & fallback)
{
  auto it = params.find(key);
  return it == params.end() ? fallback : it->second;
}

int int_param(
  const std::unordered_map<std::string, std::string> & params,
  const std::string & key, int fallback)
{
  auto it = params.find(key);
  return it == params.end() ? fallback : std::stoi(it->second);
}

bool bool_param(
  const std::unordered_map<std::string, std::string> & params,
  const std::string & key, bool fallback)
{
  auto it = params.find(key);
  if (it == params.end()) {
    return fallback;
  }
  return it->second == "true" || it->second == "True" || it->second == "1";
}
}  // namespace

CallbackReturn SO101SystemHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  const auto & hw = info_.hardware_parameters;
  try {
    port_ = param_or(hw, "port", "");
    calibration_file_ = param_or(hw, "calibration_file", "");
    baud_rate_ = int_param(hw, "baud_rate", baud_rate_);
    reply_timeout_ms_ = int_param(hw, "reply_timeout_ms", reply_timeout_ms_);
    max_consecutive_read_errors_ =
      int_param(hw, "max_consecutive_read_errors", max_consecutive_read_errors_);
    configure_servos_ = bool_param(hw, "configure_servos", configure_servos_);
    disable_torque_on_deactivate_ =
      bool_param(hw, "disable_torque_on_deactivate", disable_torque_on_deactivate_);
    read_only_ = bool_param(hw, "read_only", read_only_);
    p_coefficient_ = int_param(hw, "p_coefficient", p_coefficient_);
    i_coefficient_ = int_param(hw, "i_coefficient", i_coefficient_);
    d_coefficient_ = int_param(hw, "d_coefficient", d_coefficient_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Invalid hardware parameter: %s", e.what());
    return CallbackReturn::ERROR;
  }
  if (port_.empty() || calibration_file_.empty()) {
    RCLCPP_ERROR(get_logger(), "Hardware parameters 'port' and 'calibration_file' are required");
    return CallbackReturn::ERROR;
  }

  std::map<std::string, MotorCalibration> calibration;
  try {
    calibration = load_lerobot_calibration(calibration_file_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    return CallbackReturn::ERROR;
  }

  std::set<int> seen_ids;
  for (const auto & info : info_.joints) {
    Joint joint;
    joint.name = info.name;
    joint.motor = param_or(info.parameters, "motor", info.name);

    if (info.command_interfaces.size() != 1 ||
      info.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_ERROR(
        get_logger(), "Joint '%s' must have exactly one 'position' command interface",
        info.name.c_str());
      return CallbackReturn::ERROR;
    }
    bool has_position = false;
    for (const auto & state : info.state_interfaces) {
      if (state.name == hardware_interface::HW_IF_POSITION) {
        has_position = true;
      } else if (state.name == hardware_interface::HW_IF_VELOCITY) {
        joint.has_velocity = true;
      } else if (state.name == kLoad) {
        joint.has_load = true;
      } else if (state.name == kTemperature) {
        joint.has_temperature = true;
      } else if (state.name == kVoltage) {
        joint.has_voltage = true;
      } else {
        RCLCPP_ERROR(
          get_logger(), "Joint '%s': unsupported state interface '%s'", info.name.c_str(),
          state.name.c_str());
        return CallbackReturn::ERROR;
      }
    }
    if (!has_position) {
      RCLCPP_ERROR(get_logger(), "Joint '%s' needs a 'position' state interface",
        info.name.c_str());
      return CallbackReturn::ERROR;
    }

    auto cal = calibration.find(joint.motor);
    if (cal == calibration.end()) {
      RCLCPP_ERROR(
        get_logger(), "Motor '%s' (joint '%s') is not in %s", joint.motor.c_str(),
        info.name.c_str(), calibration_file_.c_str());
      return CallbackReturn::ERROR;
    }
    joint.calibration = cal->second;
    joint.id = static_cast<uint8_t>(joint.calibration.id);
    if (!seen_ids.insert(joint.id).second) {
      RCLCPP_ERROR(get_logger(), "Servo id %d is used by more than one joint", joint.id);
      return CallbackReturn::ERROR;
    }

    const std::string mapping = param_or(info.parameters, "mapping", "centered");
    try {
      if (mapping == "centered") {
        joint.mapping = std::make_unique<JointMapping>(JointMapping::centered(joint.calibration));
      } else if (mapping == "linear") {
        const auto & cmd = info.command_interfaces[0];
        if (cmd.min.empty() || cmd.max.empty()) {
          RCLCPP_ERROR(
            get_logger(), "Joint '%s': mapping 'linear' needs min and max on its command "
            "interface", info.name.c_str());
          return CallbackReturn::ERROR;
        }
        joint.mapping = std::make_unique<JointMapping>(
          JointMapping::linear(joint.calibration, std::stod(cmd.min), std::stod(cmd.max)));
      } else {
        RCLCPP_ERROR(get_logger(), "Joint '%s': unknown mapping '%s'", info.name.c_str(),
          mapping.c_str());
        return CallbackReturn::ERROR;
      }
      joint.max_torque_limit = int_param(info.parameters, "max_torque_limit", -1);
      joint.protection_current = int_param(info.parameters, "protection_current", -1);
      joint.overload_torque = int_param(info.parameters, "overload_torque", -1);
      const double a = joint.mapping->ticks_to_position(joint.calibration.range_min);
      const double b = joint.mapping->ticks_to_position(joint.calibration.range_max);
      joint.lower = std::min(a, b);
      joint.upper = std::max(a, b);
      const auto limits = info_.limits.find(info.name);
      if (limits != info_.limits.end() && limits->second.has_position_limits) {
        joint.lower = std::max(joint.lower, limits->second.min_position + kUrdfLimitMargin);
        joint.upper = std::min(joint.upper, limits->second.max_position - kUrdfLimitMargin);
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Joint '%s': invalid parameter: %s", info.name.c_str(),
        e.what());
      return CallbackReturn::ERROR;
    }
    ids_.push_back(joint.id);
    joints_.push_back(std::move(joint));
  }

  RCLCPP_INFO(
    get_logger(), "SO-101 hardware: %zu joints on %s, calibration %s", joints_.size(),
    port_.c_str(), calibration_file_.c_str());
  return CallbackReturn::SUCCESS;
}

bool SO101SystemHardware::open_bus()
{
  std::string error;
  if (!serial_.open(port_, baud_rate_, &error)) {
    RCLCPP_ERROR(get_logger(), "%s", error.c_str());
    return false;
  }
  bus_ = std::make_unique<FeetechBus>(
    serial_, std::chrono::milliseconds(reply_timeout_ms_));
  return true;
}

void SO101SystemHardware::close_bus()
{
  bus_.reset();
  serial_.close();
}

bool SO101SystemHardware::verify_servos()
{
  bool ok = true;
  for (const auto & joint : joints_) {
    if (!bus_->ping(joint.id)) {
      RCLCPP_ERROR(
        get_logger(), "No reply from servo %d (%s) on %s: %s. Check power, cabling and that "
        "this is the follower arm's port.", joint.id, joint.name.c_str(), port_.c_str(),
        bus_->last_error().c_str());
      ok = false;
      continue;
    }
    uint32_t model = 0;
    if (bus_->read(joint.id, reg::kModelNumber, &model) && model != feetech::kSts3215ModelNumber) {
      RCLCPP_WARN(
        get_logger(), "Servo %d (%s) reports model %u, expected STS3215 (%u)", joint.id,
        joint.name.c_str(), model, feetech::kSts3215ModelNumber);
    }

    uint32_t offset_raw = 0, min = 0, max = 0;
    if (!bus_->read(joint.id, reg::kHomingOffset, &offset_raw) ||
      !bus_->read(joint.id, reg::kMinPositionLimit, &min) ||
      !bus_->read(joint.id, reg::kMaxPositionLimit, &max))
    {
      RCLCPP_ERROR(get_logger(), "Cannot read calibration registers of servo %d (%s): %s",
        joint.id, joint.name.c_str(), bus_->last_error().c_str());
      ok = false;
      continue;
    }
    const int offset = feetech::decode_sign_magnitude(offset_raw, feetech::kHomingOffsetSignBit);
    const auto & c = joint.calibration;
    if (offset != c.homing_offset || static_cast<int>(min) != c.range_min ||
      static_cast<int>(max) != c.range_max)
    {
      RCLCPP_ERROR(
        get_logger(), "Servo %d (%s) calibration does not match %s: servo has offset=%d "
        "range=[%u, %u], file has offset=%d range=[%d, %d]. Run LeRobot calibration for this "
        "arm (or point calibration_file at the right file); refusing to drive it.",
        joint.id, joint.name.c_str(), calibration_file_.c_str(), offset, min, max,
        c.homing_offset, c.range_min, c.range_max);
      ok = false;
    }
  }
  return ok;
}

bool SO101SystemHardware::write_if_different(
  const Joint & joint, feetech::Register r,
  uint32_t value, bool * torque_disabled)
{
  uint32_t current = 0;
  if (!bus_->read(joint.id, r, &current)) {
    RCLCPP_ERROR(get_logger(), "Servo %d (%s): read of register %d failed: %s", joint.id,
      joint.name.c_str(), r.address, bus_->last_error().c_str());
    return false;
  }
  if (current == value) {
    return true;
  }
  if (!*torque_disabled) {
    // EEPROM writes need torque off and the lock released.
    RCLCPP_WARN(
      get_logger(), "Updating servo settings: torque is disabled while EEPROM is written, so "
      "the arm may sag. Support it if it is raised.");
    if (!set_torque(false)) {
      return false;
    }
    *torque_disabled = true;
  }
  RCLCPP_INFO(get_logger(), "Servo %d (%s): register %d %u -> %u", joint.id, joint.name.c_str(),
    r.address, current, value);
  if (!bus_->write(joint.id, r, value)) {
    RCLCPP_ERROR(get_logger(), "Servo %d (%s): write of register %d failed: %s", joint.id,
      joint.name.c_str(), r.address, bus_->last_error().c_str());
    return false;
  }
  return true;
}

bool SO101SystemHardware::apply_servo_settings()
{
  // The settings LeRobot applies on connect (SOFollower.configure and
  // FeetechMotorsBus.configure_motors), written only when they differ.
  bool torque_disabled = false;
  for (const auto & joint : joints_) {
    uint32_t phase = 0;
    if (!bus_->read(joint.id, reg::kPhase, &phase)) {
      RCLCPP_ERROR(get_logger(), "Servo %d: cannot read Phase: %s", joint.id,
        bus_->last_error().c_str());
      return false;
    }
    bool ok =
      write_if_different(joint, reg::kReturnDelayTime, 0, &torque_disabled) &&
      write_if_different(joint, reg::kMaximumAcceleration, 254, &torque_disabled) &&
      write_if_different(joint, reg::kAcceleration, 254, &torque_disabled) &&
      // Phase bit 4 off keeps Present_Position in [0, 4095].
      write_if_different(joint, reg::kPhase, phase & ~0x10u, &torque_disabled) &&
      write_if_different(joint, reg::kOperatingMode, 0, &torque_disabled) &&
      write_if_different(joint, reg::kPCoefficient, p_coefficient_, &torque_disabled) &&
      write_if_different(joint, reg::kICoefficient, i_coefficient_, &torque_disabled) &&
      write_if_different(joint, reg::kDCoefficient, d_coefficient_, &torque_disabled);
    if (ok && joint.max_torque_limit >= 0) {
      ok = write_if_different(joint, reg::kMaxTorqueLimit, joint.max_torque_limit,
          &torque_disabled);
    }
    if (ok && joint.protection_current >= 0) {
      ok = write_if_different(joint, reg::kProtectionCurrent, joint.protection_current,
          &torque_disabled);
    }
    if (ok && joint.overload_torque >= 0) {
      ok = write_if_different(joint, reg::kOverloadTorque, joint.overload_torque,
          &torque_disabled);
    }
    if (!ok) {
      return false;
    }
  }
  return true;
}

bool SO101SystemHardware::set_torque(bool enable)
{
  // Lock (55) follows torque, as in LeRobot: locked while driving, unlocked
  // (EEPROM writable) while limp.
  const std::vector<uint32_t> value(ids_.size(), enable ? 1u : 0u);
  if (!bus_->sync_write(reg::kTorqueEnable, ids_, value) ||
    !bus_->sync_write(reg::kLock, ids_, value))
  {
    RCLCPP_ERROR(get_logger(), "Failed to %s torque: %s", enable ? "enable" : "disable",
      bus_->last_error().c_str());
    return false;
  }
  torque_enabled_ = enable;
  return true;
}

bool SO101SystemHardware::read_feedback()
{
  std::map<uint8_t, std::vector<uint8_t>> data;
  if (!bus_->sync_read(reg::kFeedbackBlock, ids_, &data)) {
    return false;
  }
  for (auto & joint : joints_) {
    const uint8_t * d = data[joint.id].data();
    // Present_Position(2) Present_Velocity(2) Present_Load(2) Voltage(1) Temperature(1)
    joint.last_ticks = feetech::decode_sign_magnitude(
      feetech::from_bytes(d, 2), feetech::kPositionSignBit);
    set_state(joint.name + "/" + hardware_interface::HW_IF_POSITION,
      joint.mapping->ticks_to_position(joint.last_ticks));
    if (joint.has_velocity) {
      const int ticks_per_s = feetech::decode_sign_magnitude(
        feetech::from_bytes(d + 2, 2), feetech::kVelocitySignBit);
      set_state(joint.name + "/" + hardware_interface::HW_IF_VELOCITY,
        joint.mapping->ticks_to_velocity(ticks_per_s));
    }
    if (joint.has_load) {
      const int load = feetech::decode_sign_magnitude(
        feetech::from_bytes(d + 4, 2), feetech::kLoadSignBit);
      set_state(joint.name + "/" + kLoad, load / 1000.0);
    }
    if (joint.has_voltage) {
      set_state(joint.name + "/" + kVoltage, d[6] / 10.0);
    }
    if (joint.has_temperature) {
      set_state(joint.name + "/" + kTemperature, static_cast<double>(d[7]));
    }
  }
  return true;
}

CallbackReturn SO101SystemHardware::on_configure(const rclcpp_lifecycle::State &)
{
  if (!open_bus()) {
    return CallbackReturn::ERROR;
  }
  if (!verify_servos() || (configure_servos_ && !read_only_ && !apply_servo_settings())) {
    close_bus();
    return CallbackReturn::ERROR;
  }
  if (!read_feedback()) {
    RCLCPP_ERROR(get_logger(), "Initial read failed: %s", bus_->last_error().c_str());
    close_bus();
    return CallbackReturn::ERROR;
  }
  for (const auto & joint : joints_) {
    set_command(joint.name + "/" + hardware_interface::HW_IF_POSITION,
      get_state(joint.name + "/" + hardware_interface::HW_IF_POSITION));
  }
  consecutive_read_errors_ = 0;
  RCLCPP_INFO(get_logger(), "All %zu servos answered and match the calibration", joints_.size());
  return CallbackReturn::SUCCESS;
}

CallbackReturn SO101SystemHardware::on_activate(const rclcpp_lifecycle::State &)
{
  if (read_only_) {
    // Make sure the arm is free to move by hand; commands are ignored.
    if (!set_torque(false)) {
      return CallbackReturn::ERROR;
    }
    RCLCPP_WARN(get_logger(), "read_only: torque disabled, commands are ignored");
    return CallbackReturn::SUCCESS;
  }
  // Hold the current pose: goal = present position before torque comes on.
  if (!read_feedback()) {
    RCLCPP_ERROR(get_logger(), "Read before activation failed: %s", bus_->last_error().c_str());
    return CallbackReturn::ERROR;
  }
  bool outside_limits = false;
  for (const auto & joint : joints_) {
    const double position = get_state(joint.name + "/" + hardware_interface::HW_IF_POSITION);
    const double lower = joint.lower;
    const double upper = joint.upper;
    const double excess = std::max(lower - position, position - upper);
    if (excess > kActivationLimitTolerance) {
      RCLCPP_ERROR(
        get_logger(), "%s is at %.3f rad, outside its limits [%.3f, %.3f]: torque would snap "
        "it to the limit. Move it inside by hand and relaunch.", joint.name.c_str(),
        position, lower, upper);
      outside_limits = true;
    } else if (excess > 0.0) {
      RCLCPP_WARN(
        get_logger(), "%s is at %.3f rad, %.3f rad past its limits [%.3f, %.3f]; it will move "
        "to the limit when torque comes on", joint.name.c_str(), position, excess, lower, upper);
    }
  }
  if (outside_limits) {
    return CallbackReturn::ERROR;
  }

  std::vector<uint32_t> goals;
  for (const auto & joint : joints_) {
    const std::string name = joint.name + "/" + hardware_interface::HW_IF_POSITION;
    set_command(name, get_state(name));
    goals.push_back(feetech::encode_sign_magnitude(joint.last_ticks, feetech::kPositionSignBit));
  }
  if (!bus_->sync_write(reg::kGoalPosition, ids_, goals)) {
    RCLCPP_ERROR(get_logger(), "Failed to set goal positions: %s", bus_->last_error().c_str());
    return CallbackReturn::ERROR;
  }
  if (!set_torque(true)) {
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(get_logger(), "Torque enabled; holding the current pose");
  return CallbackReturn::SUCCESS;
}

CallbackReturn SO101SystemHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (disable_torque_on_deactivate_ && bus_ && torque_enabled_) {
    RCLCPP_INFO(get_logger(), "Disabling torque; the arm will go limp");
    if (!set_torque(false)) {
      return CallbackReturn::ERROR;
    }
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn SO101SystemHardware::on_cleanup(const rclcpp_lifecycle::State &)
{
  close_bus();
  return CallbackReturn::SUCCESS;
}

CallbackReturn SO101SystemHardware::on_shutdown(const rclcpp_lifecycle::State & previous_state)
{
  if (previous_state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    on_deactivate(previous_state);
  }
  close_bus();
  return CallbackReturn::SUCCESS;
}

CallbackReturn SO101SystemHardware::on_error(const rclcpp_lifecycle::State &)
{
  // Deliberately leave torque as it is: the servos keep holding their last
  // goal, which is safer for a raised arm than going limp.
  RCLCPP_ERROR(get_logger(), "Hardware error; servos keep holding their last goal position");
  close_bus();
  return CallbackReturn::SUCCESS;
}

return_type SO101SystemHardware::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (read_feedback()) {
    consecutive_read_errors_ = 0;
    return return_type::OK;
  }
  ++consecutive_read_errors_;
  RCLCPP_WARN_THROTTLE(
    get_logger(), *get_clock(), 1000, "Servo read failed (%d in a row): %s",
    consecutive_read_errors_, bus_->last_error().c_str());
  return consecutive_read_errors_ > max_consecutive_read_errors_ ?
         return_type::ERROR : return_type::OK;
}

return_type SO101SystemHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!torque_enabled_) {
    return return_type::OK;
  }
  std::vector<uint32_t> goals;
  goals.reserve(joints_.size());
  for (const auto & joint : joints_) {
    const double command = get_command(joint.name + "/" + hardware_interface::HW_IF_POSITION);
    const int ticks = std::isfinite(command) ?
      joint.mapping->position_to_ticks(std::clamp(command, joint.lower, joint.upper)) :
      joint.last_ticks;
    goals.push_back(feetech::encode_sign_magnitude(ticks, feetech::kPositionSignBit));
  }
  if (!bus_->sync_write(reg::kGoalPosition, ids_, goals)) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "Servo write failed: %s",
      bus_->last_error().c_str());
    return return_type::ERROR;
  }
  return return_type::OK;
}

}  // namespace so101_hardware

PLUGINLIB_EXPORT_CLASS(so101_hardware::SO101SystemHardware, hardware_interface::SystemInterface)
