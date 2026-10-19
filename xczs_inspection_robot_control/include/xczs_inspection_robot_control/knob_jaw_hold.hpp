// Copyright 2026
// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <algorithm>

namespace xczs_inspection_robot_control
{
// Advance through free-space servo lag without accumulating command error
// against an obstruction. A stationary measured jaw caps every future target
// at the same 0.35 mm lead, regardless of the number of retries.
inline double bounded_knob_jaw_close(double measured, double previous, double closed)
{
  constexpr double step = 0.00015;
  constexpr double maximum_lead = 0.00035;
  return std::min(closed,
    std::min(measured + maximum_lead, std::max(previous, measured) + step));
}

// Follow solver recoil without using the previous command as a new reference.
// The reference is frozen when closure is verified; the physical contact
// watchdog independently enforces the unchanged penetration/depth limits.
inline double bounded_knob_jaw_hold(
  double measured, double verified_target, double open, double closed)
{
  constexpr double preload = 0.00015;
  constexpr double maximum_additional_close = 0.00035;
  return std::clamp(measured + preload, open,
    std::min(closed, verified_target + maximum_additional_close));
}
}  // namespace xczs_inspection_robot_control
