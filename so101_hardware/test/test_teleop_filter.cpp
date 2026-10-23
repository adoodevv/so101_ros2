// Copyright 2026 Jonathan Darko Adoo
// SPDX-License-Identifier: BSD-3-Clause

#include <gtest/gtest.h>

#include <stdexcept>
#include <vector>

#include "so101_hardware/teleop_filter.hpp"

using so101_hardware::TeleopFilter;

TEST(TeleopFilter, SyncsAtSyncVelocityThenTracksAtMaxVelocity)
{
  TeleopFilter filter(0.5, 3.0, 0.05);
  filter.reset({0.0, 1.0});
  // 10 ms steps: at most 0.005 rad per step while syncing.
  auto command = filter.update({1.0, 1.0}, 0.01);
  EXPECT_NEAR(command[0], 0.005, 1e-12);
  EXPECT_DOUBLE_EQ(command[1], 1.0);
  EXPECT_FALSE(filter.synced());

  // 0.995 rad left at 0.005 per step: within 0.05 after 189 more steps.
  int steps = 0;
  while (!filter.synced()) {
    command = filter.update({1.0, 1.0}, 0.01);
    ASSERT_LT(++steps, 1000);
  }
  EXPECT_EQ(steps, 189);
  EXPECT_NEAR(command[0], 0.95, 1e-9);

  // Synced: 3 rad/s, 0.03 per step, and the leader is reached exactly.
  command = filter.update({1.0, 1.0}, 0.01);
  EXPECT_NEAR(command[0], 0.98, 1e-9);
  command = filter.update({1.0, 1.0}, 0.01);
  EXPECT_DOUBLE_EQ(command[0], 1.0);
  command = filter.update({0.0, 1.0}, 0.01);
  EXPECT_NEAR(command[0], 0.97, 1e-9);
}

TEST(TeleopFilter, ResetResyncsFromFollowerPose)
{
  TeleopFilter filter(0.5, 3.0, 0.05);
  filter.reset({0.0});
  filter.update({0.01}, 0.01);
  EXPECT_TRUE(filter.synced());
  filter.reset({-1.0});
  EXPECT_FALSE(filter.synced());
  EXPECT_NEAR(filter.update({0.0}, 0.01)[0], -0.995, 1e-12);
}

TEST(TeleopFilter, RejectsMisuse)
{
  EXPECT_THROW(TeleopFilter(0.0, 1.0, 0.1), std::invalid_argument);
  TeleopFilter filter(0.5, 3.0, 0.05);
  EXPECT_THROW(filter.update({0.0}, 0.01), std::logic_error);
  filter.reset({0.0, 0.0});
  EXPECT_THROW(filter.update({0.0}, 0.01), std::invalid_argument);
  filter.deactivate();
  EXPECT_FALSE(filter.active());
  // A negative dt (clock jump) never moves the command backwards.
  filter.reset({0.0});
  EXPECT_DOUBLE_EQ(filter.update({1.0}, -0.5)[0], 0.0);
}
