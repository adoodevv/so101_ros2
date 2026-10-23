// Copyright 2026 Jonathan Darko Adoo
// SPDX-License-Identifier: BSD-3-Clause

#ifndef SO101_HARDWARE__CALIBRATION_HPP_
#define SO101_HARDWARE__CALIBRATION_HPP_

#include <map>
#include <string>

namespace so101_hardware
{

// One motor's entry in a LeRobot calibration file, e.g.
// ~/.cache/huggingface/lerobot/calibration/robots/so_follower/my_follower.json
// LeRobot writes homing_offset and the range into the servo's EEPROM, so
// Present_Position is already offset-corrected and lies in [range_min, range_max].
struct MotorCalibration
{
  int id = 0;
  int drive_mode = 0;  // 1 = reversed direction
  int homing_offset = 0;
  int range_min = 0;
  int range_max = 0;
};

// Throws std::runtime_error with a readable message on a missing file or bad
// content.
std::map<std::string, MotorCalibration> load_lerobot_calibration(const std::string & path);

// Converts between servo ticks and the joint position used in the URDF.
class JointMapping
{
public:
  // Revolute joint, zero at the middle of the calibrated range (LeRobot's
  // "degrees" mode, matching the SO-101 "new calibration" URDF).
  static JointMapping centered(const MotorCalibration & calibration);
  // Linear map of [range_min, range_max] onto [position_at_min,
  // position_at_max]; used for the gripper, which LeRobot treats as 0-100 %.
  static JointMapping linear(
    const MotorCalibration & calibration,
    double position_at_min, double position_at_max);

  double ticks_to_position(double ticks) const;
  // Clamped to the calibrated range, so the servo is never asked to leave it.
  int position_to_ticks(double position) const;
  // ticks/s -> position units/s
  double ticks_to_velocity(double ticks_per_second) const;

  int range_min() const {return range_min_;}
  int range_max() const {return range_max_;}

private:
  JointMapping(int range_min, int range_max, double ticks_at_zero, double units_per_tick);

  int range_min_;
  int range_max_;
  double ticks_at_zero_;
  double units_per_tick_;  // signed; includes drive_mode
};

}  // namespace so101_hardware

#endif  // SO101_HARDWARE__CALIBRATION_HPP_
