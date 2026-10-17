// Copyright 2026
// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <tf2/LinearMath/Transform.h>

namespace xczs_inspection_robot_control
{
// Compare rigid-frame corrections where the robot is working. Comparing frame
// origins instead amplifies small yaw errors by the distance to a remote origin.
inline double anchor_correction_at_reference(
  const tf2::Transform & previous, const tf2::Transform & corrected,
  const tf2::Vector3 & reference_in_planning)
{
  const auto reference_in_cabinet = previous.inverse() * reference_in_planning;
  return (corrected * reference_in_cabinet).distance(reference_in_planning);
}
}
