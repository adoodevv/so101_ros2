// Copyright 2026 Jonathan Darko Adoo
// SPDX-License-Identifier: BSD-3-Clause

#include "so101_hardware/calibration.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <stdexcept>
#include <string>

#include "so101_hardware/feetech_protocol.hpp"

namespace so101_hardware
{

std::map<std::string, MotorCalibration> load_lerobot_calibration(const std::string & path)
{
  YAML::Node root;
  try {
    // JSON is valid YAML, so yaml-cpp reads LeRobot's calibration files.
    root = YAML::LoadFile(path);
  } catch (const std::exception & e) {
    throw std::runtime_error("cannot read calibration file '" + path + "': " + e.what());
  }
  if (!root.IsMap() || root.size() == 0) {
    throw std::runtime_error("calibration file '" + path + "' is not a non-empty JSON object");
  }
  std::map<std::string, MotorCalibration> out;
  for (const auto & entry : root) {
    const auto name = entry.first.as<std::string>();
    const auto & node = entry.second;
    MotorCalibration c;
    try {
      c.id = node["id"].as<int>();
      c.drive_mode = node["drive_mode"].as<int>();
      c.homing_offset = node["homing_offset"].as<int>();
      c.range_min = node["range_min"].as<int>();
      c.range_max = node["range_max"].as<int>();
    } catch (const std::exception & e) {
      throw std::runtime_error(
              "calibration for '" + name + "' in '" + path +
              "' needs integer id, drive_mode, homing_offset, range_min, range_max");
    }
    if (c.id < 0 || c.id > 253) {
      throw std::runtime_error("calibration for '" + name + "': id out of range");
    }
    if (!(0 <= c.range_min && c.range_min < c.range_max &&
      c.range_max < feetech::kResolution))
    {
      throw std::runtime_error(
              "calibration for '" + name + "': need 0 <= range_min < range_max <= 4095");
    }
    out[name] = c;
  }
  return out;
}

JointMapping::JointMapping(
  int range_min, int range_max, double ticks_at_zero,
  double units_per_tick)
: range_min_(range_min), range_max_(range_max), ticks_at_zero_(ticks_at_zero),
  units_per_tick_(units_per_tick)
{
}

JointMapping JointMapping::centered(const MotorCalibration & c)
{
  // Same scale as LeRobot (360 degrees over max resolution 4095), in radians.
  const double rad_per_tick = 2.0 * M_PI / (feetech::kResolution - 1);
  const double sign = c.drive_mode ? -1.0 : 1.0;
  return JointMapping(
    c.range_min, c.range_max, (c.range_min + c.range_max) / 2.0, sign * rad_per_tick);
}

JointMapping JointMapping::linear(
  const MotorCalibration & c, double position_at_min,
  double position_at_max)
{
  const double span = static_cast<double>(c.range_max - c.range_min);
  double per_tick = (position_at_max - position_at_min) / span;
  double ticks_at_zero = c.range_min - position_at_min / per_tick;
  if (c.drive_mode) {
    // Reversed: range_max maps to position_at_min.
    per_tick = -per_tick;
    ticks_at_zero = c.range_max - position_at_min / per_tick;
  }
  return JointMapping(c.range_min, c.range_max, ticks_at_zero, per_tick);
}

double JointMapping::ticks_to_position(double ticks) const
{
  return (ticks - ticks_at_zero_) * units_per_tick_;
}

int JointMapping::position_to_ticks(double position) const
{
  const double ticks = ticks_at_zero_ + position / units_per_tick_;
  return std::clamp(static_cast<int>(std::lround(ticks)), range_min_, range_max_);
}

double JointMapping::ticks_to_velocity(double ticks_per_second) const
{
  return ticks_per_second * units_per_tick_;
}

}  // namespace so101_hardware
