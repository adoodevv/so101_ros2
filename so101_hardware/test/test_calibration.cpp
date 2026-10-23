// Copyright 2026 Jonathan Darko Adoo
// SPDX-License-Identifier: BSD-3-Clause

#include <gtest/gtest.h>

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <map>
#include <string>

#include "so101_hardware/calibration.hpp"

using so101_hardware::JointMapping;
using so101_hardware::MotorCalibration;

namespace
{
std::string data_file(const std::string & name)
{
  const char * dir = std::getenv("TEST_DATA_DIR");
  return std::string(dir ? dir : ".") + "/" + name;
}

std::string write_temp(const std::string & content)
{
  const std::string path = testing::TempDir() + "so101_calibration_test.json";
  std::ofstream(path) << content;
  return path;
}

constexpr double kRadPerTick = 2.0 * M_PI / 4095.0;
}  // namespace

// The follower calibration this arm was set up with in LeRobot (my_follower).
TEST(Calibration, LoadsLeRobotFile)
{
  const auto cal = so101_hardware::load_lerobot_calibration(
    data_file("example_follower_calibration.json"));
  ASSERT_EQ(cal.size(), 6u);
  EXPECT_EQ(cal.at("shoulder_pan").id, 1);
  EXPECT_EQ(cal.at("shoulder_lift").homing_offset, -1767);
  EXPECT_EQ(cal.at("gripper").id, 6);
  EXPECT_EQ(cal.at("gripper").range_min, 1480);
  EXPECT_EQ(cal.at("gripper").range_max, 3164);
}

TEST(Calibration, RejectsBadFiles)
{
  EXPECT_THROW(so101_hardware::load_lerobot_calibration("/nonexistent.json"),
    std::runtime_error);
  EXPECT_THROW(so101_hardware::load_lerobot_calibration(write_temp("{}")), std::runtime_error);
  EXPECT_THROW(
    so101_hardware::load_lerobot_calibration(
      write_temp(R"({"a": {"id": 1, "drive_mode": 0, "homing_offset": 0}})")),
    std::runtime_error);
  EXPECT_THROW(
    so101_hardware::load_lerobot_calibration(
      write_temp(
        R"({"a": {"id": 1, "drive_mode": 0, "homing_offset": 0, "range_min": 900,
        "range_max": 800}})")),
    std::runtime_error);
}

TEST(JointMapping, CenteredMatchesLeRobotDegrees)
{
  // shoulder_pan of my_follower: range [751, 3385], mid 2068.
  MotorCalibration c{1, 0, 1355, 751, 3385};
  const auto m = JointMapping::centered(c);
  EXPECT_NEAR(m.ticks_to_position(2068), 0.0, 1e-12);
  // LeRobot: degrees = (ticks - mid) * 360 / 4095
  const double degrees = (3000 - 2068) * 360.0 / 4095.0;
  EXPECT_NEAR(m.ticks_to_position(3000), degrees * M_PI / 180.0, 1e-12);
  EXPECT_EQ(m.position_to_ticks(0.0), 2068);
  EXPECT_EQ(m.position_to_ticks(m.ticks_to_position(1234)), 1234);
  EXPECT_NEAR(m.ticks_to_velocity(100), 100 * kRadPerTick, 1e-12);
}

TEST(JointMapping, CenteredClampsToCalibratedRange)
{
  MotorCalibration c{1, 0, 0, 751, 3385};
  const auto m = JointMapping::centered(c);
  EXPECT_EQ(m.position_to_ticks(10.0), 3385);
  EXPECT_EQ(m.position_to_ticks(-10.0), 751);
}

TEST(JointMapping, CenteredDriveModeReverses)
{
  MotorCalibration c{1, 1, 0, 1000, 3000};
  const auto m = JointMapping::centered(c);
  EXPECT_NEAR(m.ticks_to_position(2100), -100 * kRadPerTick, 1e-12);
  EXPECT_EQ(m.position_to_ticks(-100 * kRadPerTick), 2100);
}

TEST(JointMapping, LinearGripperSpansJointLimits)
{
  // gripper of my_follower: range [1480, 3164]; closed at range_min (LeRobot 0 %).
  MotorCalibration c{6, 0, -1079, 1480, 3164};
  const auto m = JointMapping::linear(c, -0.174533, 1.74533);
  EXPECT_NEAR(m.ticks_to_position(1480), -0.174533, 1e-9);
  EXPECT_NEAR(m.ticks_to_position(3164), 1.74533, 1e-9);
  EXPECT_EQ(m.position_to_ticks(-0.174533), 1480);
  EXPECT_EQ(m.position_to_ticks(1.74533), 3164);
  EXPECT_EQ(m.position_to_ticks(5.0), 3164);
  // SRDF `open` (1.5) and `closed` (-0.17) land inside the range.
  EXPECT_GT(m.position_to_ticks(1.5), 2900);
  EXPECT_LT(m.position_to_ticks(-0.17), 1490);
}

TEST(JointMapping, LinearDriveModeReverses)
{
  MotorCalibration c{6, 1, 0, 1000, 3000};
  const auto m = JointMapping::linear(c, 0.0, 2.0);
  EXPECT_NEAR(m.ticks_to_position(3000), 0.0, 1e-12);
  EXPECT_NEAR(m.ticks_to_position(1000), 2.0, 1e-12);
  EXPECT_EQ(m.position_to_ticks(0.5), 2500);
}
