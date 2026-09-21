// Copyright 2026
// SPDX-License-Identifier: BSD-3-Clause
#pragma once
#include <cmath>

namespace xczs_inspection_robot_control
{
// Permit the existing simulation coupling only from clear, aligned geometry.
inline bool continuous_insert_is_aligned(
  double axial, double lateral, double opposing_axes_cosine, double square_face_cosine)
{
  return std::isfinite(axial) && std::isfinite(lateral) &&
         std::isfinite(opposing_axes_cosine) && std::isfinite(square_face_cosine) &&
         axial >= 0.005 && axial <= 0.05 && lateral >= 0.0 && lateral <= 0.00025 &&
         opposing_axes_cosine >= std::cos(0.01) &&
         square_face_cosine >= std::cos(0.01);
}
}  // namespace xczs_inspection_robot_control
