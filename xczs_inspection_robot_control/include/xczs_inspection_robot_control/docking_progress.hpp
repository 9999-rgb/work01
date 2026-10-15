// Copyright 2026
// SPDX-License-Identifier: BSD-3-Clause
#ifndef XCZS_INSPECTION_ROBOT_CONTROL__DOCKING_PROGRESS_HPP_
#define XCZS_INSPECTION_ROBOT_CONTROL__DOCKING_PROGRESS_HPP_

#include <chrono>
#include <cmath>

namespace xczs_inspection_robot_control
{
// Breakaway is for a stationary base, not for a base returning after overshoot.
// Rebase on either significant progress or a rebound, rather than retaining an
// all-time minimum that cannot be reached again while correcting an overshoot.
class DockingProgress
{
public:
  using Clock = std::chrono::steady_clock;

  DockingProgress(double stall_seconds, double minimum_change)
  : stall_seconds_(stall_seconds), minimum_change_(minimum_change) {}

  void reset(double error, Clock::time_point now)
  {
    reference_error_ = error;
    reference_time_ = now;
    initialized_ = true;
  }

  bool needs_breakaway(double error, Clock::time_point now)
  {
    if (!initialized_ || std::abs(error - reference_error_) >= minimum_change_) {
      reset(error, now);
      return false;
    }
    return std::chrono::duration<double>(now - reference_time_).count() >= stall_seconds_;
  }

private:
  double stall_seconds_;
  double minimum_change_;
  double reference_error_{0.0};
  Clock::time_point reference_time_{};
  bool initialized_{false};
};
}  // namespace xczs_inspection_robot_control
#endif  // XCZS_INSPECTION_ROBOT_CONTROL__DOCKING_PROGRESS_HPP_
