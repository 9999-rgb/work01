// Copyright 2026
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XCZS_INSPECTION_ROBOT_CONTROL__JERK_LIMITED_VELOCITY_PROFILE_HPP_
#define XCZS_INSPECTION_ROBOT_CONTROL__JERK_LIMITED_VELOCITY_PROFILE_HPP_

#include <algorithm>
#include <cmath>

namespace xczs_inspection_robot_control
{

class JerkLimitedVelocityProfile
{
public:
  void configure(double max_acceleration, double max_jerk)
  {
    max_acceleration_ = max_acceleration;
    max_jerk_ = max_jerk;
  }

  void set_target(double target_velocity)
  {
    if (std::abs(target_velocity - target_velocity_) < kTolerance) {
      return;
    }

    start_velocity_ = current_velocity_;
    target_velocity_ = target_velocity;
    elapsed_time_ = 0.0;

    const double velocity_change =
      std::abs(target_velocity_ - start_velocity_);
    if (velocity_change < kTolerance) {
      current_velocity_ = target_velocity_;
      transition_duration_ = 0.0;
      return;
    }

    // Exact derivative maxima for the quintic smoothstep used by update().
    constexpr double kAccelerationFactor = 1.875;
    constexpr double kJerkFactor = 5.773502691896258;
    const double acceleration_limited_duration =
      kAccelerationFactor * velocity_change / max_acceleration_;
    const double jerk_limited_duration =
      std::sqrt(kJerkFactor * velocity_change / max_jerk_);
    transition_duration_ = std::max(
      acceleration_limited_duration,
      jerk_limited_duration);
  }

  double update(double elapsed_seconds)
  {
    if (transition_duration_ <= 0.0) {
      current_velocity_ = target_velocity_;
      return current_velocity_;
    }

    elapsed_time_ = std::min(
      elapsed_time_ + elapsed_seconds,
      transition_duration_);
    const double phase = elapsed_time_ / transition_duration_;
    const double phase_squared = phase * phase;
    const double phase_cubed = phase_squared * phase;
    const double blend =
      10.0 * phase_cubed -
      15.0 * phase_cubed * phase +
      6.0 * phase_cubed * phase_squared;

    current_velocity_ =
      start_velocity_ +
      (target_velocity_ - start_velocity_) * blend;

    if (elapsed_time_ >= transition_duration_) {
      current_velocity_ = target_velocity_;
      transition_duration_ = 0.0;
    }
    return current_velocity_;
  }

  void reset()
  {
    start_velocity_ = 0.0;
    current_velocity_ = 0.0;
    target_velocity_ = 0.0;
    elapsed_time_ = 0.0;
    transition_duration_ = 0.0;
  }

private:
  static constexpr double kTolerance = 1.0e-9;

  double max_acceleration_{1.0};
  double max_jerk_{1.0};
  double start_velocity_{0.0};
  double current_velocity_{0.0};
  double target_velocity_{0.0};
  double elapsed_time_{0.0};
  double transition_duration_{0.0};
};

}  // namespace xczs_inspection_robot_control

#endif  // XCZS_INSPECTION_ROBOT_CONTROL__JERK_LIMITED_VELOCITY_PROFILE_HPP_
