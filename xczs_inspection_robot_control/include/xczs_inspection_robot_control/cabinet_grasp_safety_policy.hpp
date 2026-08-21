// Copyright 2026
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <cmath>

namespace xczs_inspection_robot_control
{

/**
 * Detect physical movement of a detented control before grasp attachment.
 *
 * The planned ready/approach path is not allowed to actuate a knob, switch or
 * door.  A sample outside either tolerance therefore indicates unintended
 * tool/cabinet contact and must make the subsequent grasp fail closed.
 */
inline bool pregrasp_detent_is_disturbed(
  double position,
  double velocity,
  double latched_detent_position,
  double position_tolerance,
  double velocity_tolerance) noexcept
{
  if (!std::isfinite(position) || !std::isfinite(velocity) ||
    !std::isfinite(latched_detent_position) ||
    !std::isfinite(position_tolerance) || position_tolerance <= 0.0 ||
    !std::isfinite(velocity_tolerance) || velocity_tolerance <= 0.0)
  {
    return true;
  }
  return std::abs(position - latched_detent_position) > position_tolerance ||
         std::abs(velocity) > velocity_tolerance;
}

}  // namespace xczs_inspection_robot_control
