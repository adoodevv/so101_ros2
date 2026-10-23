// Copyright 2026 Jonathan Darko Adoo
// SPDX-License-Identifier: BSD-3-Clause

#ifndef SO101_HARDWARE__TELEOP_FILTER_HPP_
#define SO101_HARDWARE__TELEOP_FILTER_HPP_

#include <vector>

namespace so101_hardware
{

// Turns leader-arm positions into follower commands without jumps.
//
// After reset() (teleop start, or the leader coming back after a dropout) the
// command starts at the follower's pose and moves toward the leader at
// sync_velocity until every joint is within sync_tolerance of it; from then
// on it tracks the leader, limited to max_velocity per joint.
class TeleopFilter
{
public:
  TeleopFilter(double sync_velocity, double max_velocity, double sync_tolerance);

  // Start (again) from the follower's current positions.
  void reset(const std::vector<double> & follower_positions);
  // Stop producing commands until the next reset().
  void deactivate() {active_ = false;}

  bool active() const {return active_;}
  bool synced() const {return synced_;}

  // Advance by dt seconds toward the leader positions; returns the command.
  // Throws std::logic_error if not active, std::invalid_argument on a size
  // mismatch.
  const std::vector<double> & update(const std::vector<double> & leader_positions, double dt);

private:
  double sync_velocity_;
  double max_velocity_;
  double sync_tolerance_;
  bool active_ = false;
  bool synced_ = false;
  std::vector<double> command_;
};

}  // namespace so101_hardware

#endif  // SO101_HARDWARE__TELEOP_FILTER_HPP_
