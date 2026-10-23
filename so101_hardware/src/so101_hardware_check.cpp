// Copyright 2026 Jonathan Darko Adoo
// SPDX-License-Identifier: BSD-3-Clause

// Read-only check of an SO-101 follower arm: pings every servo, compares the
// calibration stored in the servos with a LeRobot calibration file, and prints
// the joint positions the ros2_control plugin would report. It never enables
// torque or writes a register, so the arm does not move.
//
//   ros2 run so101_hardware so101_hardware_check --port /dev/ttyACM0
//     --calibration ~/.cache/huggingface/lerobot/calibration/robots/so_follower/my_follower.json
//   add --watch to keep printing positions while you move the arm by hand.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <map>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "so101_hardware/calibration.hpp"
#include "so101_hardware/feetech_bus.hpp"
#include "so101_hardware/serial_port.hpp"

using so101_hardware::FeetechBus;
using so101_hardware::JointMapping;
using so101_hardware::MotorCalibration;
namespace feetech = so101_hardware::feetech;

namespace
{
// Same joint order and mapping as the URDF ros2_control block.
const std::vector<std::string> kJoints = {
  "shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"};
constexpr double kGripperClosed = -0.174533;
constexpr double kGripperOpen = 1.74533;
// URDF joint limits (so101_robot.urdf.xacro). Like the plugin, the check
// intersects them with the calibrated range and applies the plugin's
// activation tolerance: beyond it the plugin refuses to enable torque.
const std::map<std::string, std::pair<double, double>> kLimits = {
  {"shoulder_pan", {-1.91986, 1.91986}}, {"shoulder_lift", {-1.74533, 1.74533}},
  {"elbow_flex", {-1.69, 1.69}}, {"wrist_flex", {-1.65806, 1.65806}},
  {"wrist_roll", {-2.74385, 2.84121}}, {"gripper", {kGripperClosed, kGripperOpen}}};
constexpr double kActivationLimitTolerance = 0.1;

struct ServoCalibration
{
  int homing_offset;
  int range_min;
  int range_max;
};

bool matches(const MotorCalibration & c, const ServoCalibration & s)
{
  return c.homing_offset == s.homing_offset && c.range_min == s.range_min &&
         c.range_max == s.range_max;
}

// LeRobot keeps one file per arm id in the same directory. When the servos do
// not match the given file, look for the file they do match: usually the arm
// was last connected under a different id, which wrote that calibration.
std::vector<std::string> find_matching_files(
  const std::string & calibration_file, const std::map<std::string, ServoCalibration> & servos)
{
  std::vector<std::string> found;
  std::error_code ec;
  const auto dir = std::filesystem::path(calibration_file).parent_path();
  for (const auto & entry : std::filesystem::directory_iterator(dir, ec)) {
    if (entry.path().extension() != ".json") {
      continue;
    }
    try {
      const auto other = so101_hardware::load_lerobot_calibration(entry.path().string());
      bool all = !servos.empty();
      for (const auto & [name, servo] : servos) {
        auto it = other.find(name);
        all = all && it != other.end() && matches(it->second, servo);
      }
      if (all) {
        found.push_back(entry.path().string());
      }
    } catch (const std::exception &) {
    }
  }
  return found;
}

void usage()
{
  std::fprintf(
    stderr,
    "usage: so101_hardware_check --port DEVICE --calibration FILE [--baud 1000000] [--watch]\n");
}
}  // namespace

int main(int argc, char ** argv)
{
  std::string port, calibration_file;
  int baud = 1000000;
  bool watch = false;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--port" && i + 1 < argc) {
      port = argv[++i];
    } else if (arg == "--calibration" && i + 1 < argc) {
      calibration_file = argv[++i];
    } else if (arg == "--baud" && i + 1 < argc) {
      baud = std::stoi(argv[++i]);
    } else if (arg == "--watch") {
      watch = true;
    } else {
      usage();
      return 2;
    }
  }
  if (port.empty() || calibration_file.empty()) {
    usage();
    return 2;
  }

  std::map<std::string, MotorCalibration> calibration;
  try {
    calibration = so101_hardware::load_lerobot_calibration(calibration_file);
  } catch (const std::exception & e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }

  so101_hardware::SerialPort serial;
  std::string error;
  if (!serial.open(port, baud, &error)) {
    std::fprintf(stderr, "error: %s\n", error.c_str());
    return 1;
  }
  FeetechBus bus(serial, std::chrono::milliseconds(20));

  std::printf("Port %s @ %d baud, calibration %s\n\n", port.c_str(), baud,
    calibration_file.c_str());
  std::printf("%-14s %3s %5s %7s %-24s %s\n", "joint", "id", "ping", "model", "calibration",
    "servo errors");
  bool all_ok = true;
  std::vector<uint8_t> ids;
  std::map<uint8_t, JointMapping> mappings;
  std::map<std::string, ServoCalibration> servo_calibration;
  bool calibration_mismatch = false;
  for (const auto & name : kJoints) {
    auto it = calibration.find(name);
    if (it == calibration.end()) {
      std::printf("%-14s missing from calibration file\n", name.c_str());
      all_ok = false;
      continue;
    }
    const MotorCalibration & c = it->second;
    const auto id = static_cast<uint8_t>(c.id);
    if (!bus.ping(id)) {
      std::printf("%-14s %3d %5s  %s\n", name.c_str(), c.id, "NO", bus.last_error().c_str());
      all_ok = false;
      continue;
    }
    uint32_t model = 0, offset_raw = 0, min = 0, max = 0;
    bus.read(id, feetech::reg::kModelNumber, &model);
    const bool read_ok = bus.read(id, feetech::reg::kHomingOffset, &offset_raw) &&
      bus.read(id, feetech::reg::kMinPositionLimit, &min) &&
      bus.read(id, feetech::reg::kMaxPositionLimit, &max);
    const int offset = feetech::decode_sign_magnitude(offset_raw, feetech::kHomingOffsetSignBit);
    const ServoCalibration servo{offset, static_cast<int>(min), static_cast<int>(max)};
    if (read_ok) {
      servo_calibration[name] = servo;
    }
    const bool match = read_ok && matches(c, servo);
    char cal[64];
    if (match) {
      std::snprintf(cal, sizeof(cal), "matches");
    } else {
      std::snprintf(cal, sizeof(cal), "MISMATCH off=%d [%u,%u]", offset, min, max);
      all_ok = false;
      calibration_mismatch = true;
    }
    std::printf("%-14s %3d %5s %7u %-24s %s\n", name.c_str(), c.id, "ok", model, cal,
      feetech::describe_error(bus.last_servo_error()).c_str());
    ids.push_back(id);
    mappings.emplace(
      id, name == "gripper" ? JointMapping::linear(c, kGripperClosed, kGripperOpen) :
      JointMapping::centered(c));
  }

  bool outside_limits = false;
  do {
    outside_limits = false;
    std::map<uint8_t, std::vector<uint8_t>> data;
    if (ids.empty() || !bus.sync_read(feetech::reg::kFeedbackBlock, ids, &data)) {
      std::printf("\nsync read failed: %s\n", bus.last_error().c_str());
      return 1;
    }
    std::printf("\n%-14s %6s %9s %8s %7s %6s  %s\n", "joint", "ticks", "position", "degrees",
      "volts", "temp", "limits");
    for (size_t i = 0; i < ids.size(); ++i) {
      const uint8_t * d = data[ids[i]].data();
      const int ticks = feetech::decode_sign_magnitude(
        feetech::from_bytes(d, 2), feetech::kPositionSignBit);
      const double position = mappings.at(ids[i]).ticks_to_position(ticks);
      std::string name;
      for (const auto & [n, c] : calibration) {
        if (c.id == ids[i]) {name = n;}
      }
      const JointMapping & mapping = mappings.at(ids[i]);
      const MotorCalibration & c = calibration.at(name);
      const double a = mapping.ticks_to_position(c.range_min);
      const double b = mapping.ticks_to_position(c.range_max);
      const double lower = std::max(std::min(a, b), kLimits.at(name).first);
      const double upper = std::min(std::max(a, b), kLimits.at(name).second);
      const double excess = std::max(lower - position, position - upper);
      const char * limit = excess <= 0.0 ? "ok" :
        excess <= kActivationLimitTolerance ? "slightly outside (moves to limit on start)" :
        "OUTSIDE: move it in by hand";
      std::printf("%-14s %6d %9.4f %8.2f %7.1f %6d  %s\n", name.c_str(), ticks, position,
        position * 180.0 / M_PI, d[6] / 10.0, d[7], limit);
      if (excess > kActivationLimitTolerance) {
        outside_limits = true;
      }
    }
    if (watch) {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  } while (watch);

  if (calibration_mismatch) {
    std::printf(
      "\nThe servos hold a different calibration than %s\n"
      "(MISMATCH shows the servo's values; the file has:)\n", calibration_file.c_str());
    for (const auto & name : kJoints) {
      auto it = calibration.find(name);
      if (it != calibration.end()) {
        std::printf("  %-14s off=%d [%d,%d]\n", name.c_str(), it->second.homing_offset,
          it->second.range_min, it->second.range_max);
      }
    }
    const auto candidates = find_matching_files(calibration_file, servo_calibration);
    if (!candidates.empty()) {
      std::printf("\nThe servos match:\n");
      for (const auto & f : candidates) {
        std::printf("  %s\n", f.c_str());
      }
      std::printf(
        "Use that file (calibration_file:=..., or follower_id:=<its name> in the launch),\n"
        "or let LeRobot write the other file into the servos.\n");
    } else {
      std::printf(
        "\nNo calibration file next to it matches the servos. Recalibrate with\n"
        "lerobot-calibrate, or connect once with LeRobot using this id so it writes\n"
        "the file into the servos.\n");
    }
  }

  if (all_ok && outside_limits) {
    std::printf(
      "\nServos and calibration are fine, but a joint is outside its limits;\n"
      "the plugin will not enable torque until it is moved inside by hand.\n");
    return 1;
  }
  std::printf("\n%s\n", all_ok ?
    "All servos present and calibrated. Safe to launch so101_real_moveit.launch.py." :
    "Problems found; fix them before launching ros2_control.");
  return all_ok ? 0 : 1;
}
