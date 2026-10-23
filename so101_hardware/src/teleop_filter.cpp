// Copyright 2026 Jonathan Darko Adoo
// SPDX-License-Identifier: BSD-3-Clause

#include "so101_hardware/teleop_filter.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace so101_hardware
{

TeleopFilter::TeleopFilter(double sync_velocity, double max_velocity, double sync_tolerance)
: sync_velocity_(sync_velocity), max_velocity_(max_velocity), sync_tolerance_(sync_tolerance)
{
  if (!(sync_velocity > 0.0) || !(max_velocity > 0.0) || !(sync_tolerance > 0.0)) {
    throw std::invalid_argument("TeleopFilter: velocities and tolerance must be positive");
  }
}

void TeleopFilter::reset(const std::vector<double> & follower_positions)
{
  command_ = follower_positions;
  active_ = true;
  synced_ = false;
}

const std::vector<double> & TeleopFilter::update(
  const std::vector<double> & leader_positions, double dt)
{
  if (!active_) {
    throw std::logic_error("TeleopFilter::update before reset");
  }
  if (leader_positions.size() != command_.size()) {
    throw std::invalid_argument("TeleopFilter: leader and follower joint counts differ");
  }
  const double step = (synced_ ? max_velocity_ : sync_velocity_) * std::max(dt, 0.0);
  bool within_tolerance = true;
  for (size_t i = 0; i < command_.size(); ++i) {
    command_[i] += std::clamp(leader_positions[i] - command_[i], -step, step);
    within_tolerance = within_tolerance &&
      std::abs(leader_positions[i] - command_[i]) <= sync_tolerance_;
  }
  synced_ = synced_ || within_tolerance;
  return command_;
}

}  // namespace so101_hardware
