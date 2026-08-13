// Copyright 2026
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <cmath>

namespace xczs_inspection_robot_control
{

enum class StructuredControlStateUpdate
{
  IGNORE,
  INVALIDATE,
  ACCEPT,
};

/** Classify a structured control-state message before updating the cache. */
inline StructuredControlStateUpdate classify_structured_control_state(
  bool control_id_matches,
  bool declared_valid,
  double position,
  double velocity,
  double effort) noexcept
{
  if (!control_id_matches) {
    return StructuredControlStateUpdate::IGNORE;
  }
  if (!declared_valid || !std::isfinite(position) ||
    !std::isfinite(velocity) || !std::isfinite(effort))
  {
    return StructuredControlStateUpdate::INVALIDATE;
  }
  return StructuredControlStateUpdate::ACCEPT;
}

/**
 * Return whether cached structured state is safe to use.
 *
 * Position-only JointState traffic is intentionally absent from this policy:
 * it cannot refresh validity, velocity, motion, or detent state supplied by
 * the cabinet's structured state stream.
 */
constexpr bool structured_control_state_is_usable(
  bool structured_state_received,
  bool structured_state_valid,
  bool received_after_operation_start,
  bool received_within_timeout) noexcept
{
  return structured_state_received && structured_state_valid &&
         received_after_operation_start && received_within_timeout;
}

}  // namespace xczs_inspection_robot_control
