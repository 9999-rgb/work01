// Copyright 2026
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XCZS_INSPECTION_ROBOT_CONTROL__ROTARY_OPERATION_POLICY_HPP_
#define XCZS_INSPECTION_ROBOT_CONTROL__ROTARY_OPERATION_POLICY_HPP_

#include <cstddef>
#include <cmath>

namespace xczs_inspection_robot_control
{

// Per-transition calibration matrices are row-major: source detent first,
// requested detent second.  Keeping this tiny policy outside the ROS node
// makes the safety-critical indexing and partial-release behavior testable.
constexpr std::size_t rotary_transition_matrix_index(
  std::size_t source_index,
  std::size_t target_index,
  std::size_t state_count) noexcept
{
  return source_index * state_count + target_index;
}

constexpr bool rotary_transition_is_adjacent(
  std::size_t source_index,
  std::size_t target_index) noexcept
{
  return source_index > target_index ?
         source_index - target_index <= 1U :
         target_index - source_index <= 1U;
}

inline double rotary_release_position(
  double initial_position,
  double target_position,
  double release_fraction,
  double no_motion_tolerance) noexcept
{
  if (std::abs(target_position - initial_position) <= no_motion_tolerance) {
    return target_position;
  }
  return initial_position +
         release_fraction * (target_position - initial_position);
}

}  // namespace xczs_inspection_robot_control

#endif  // XCZS_INSPECTION_ROBOT_CONTROL__ROTARY_OPERATION_POLICY_HPP_
