// Copyright 2026
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <algorithm>
#include <cmath>

namespace xczs_inspection_robot_control
{

enum class StructuredControlStateUpdate
{
  IGNORE,
  INVALIDATE,
  ACCEPT,
};

enum class PregraspStabilitySampleStatus
{
  WAITING,
  STABLE,
  REFERENCE_CHANGED,
};

/** Track a sign-independent peak while retaining positive button semantics. */
inline double update_absolute_position_peak(
  double previous_peak,
  double position) noexcept
{
  return std::max(std::abs(previous_peak), std::abs(position));
}

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

/**
 * Classify one structured sample against the state captured at action start.
 *
 * A usable sample that has left the original detent is an immediate safety
 * violation: the ready/approach motion may already have touched the cabinet.
 * Missing, stale, invalid, or moving samples remain non-stable and must reset
 * the caller's continuous-stability timer.
 */
inline PregraspStabilitySampleStatus classify_pregrasp_stability_sample(
  bool structured_state_received,
  bool structured_state_valid,
  bool received_after_boundary,
  bool received_within_timeout,
  bool state_id_matches,
  bool in_motion,
  double position,
  double initial_position,
  double velocity,
  double position_tolerance,
  double velocity_tolerance) noexcept
{
  if (!structured_control_state_is_usable(
      structured_state_received, structured_state_valid,
      received_after_boundary, received_within_timeout) ||
    !std::isfinite(position) || !std::isfinite(initial_position) ||
    !std::isfinite(velocity) || !std::isfinite(position_tolerance) ||
    position_tolerance <= 0.0 || !std::isfinite(velocity_tolerance) ||
    velocity_tolerance <= 0.0)
  {
    return PregraspStabilitySampleStatus::WAITING;
  }
  if (!state_id_matches ||
    std::abs(position - initial_position) > position_tolerance)
  {
    return PregraspStabilitySampleStatus::REFERENCE_CHANGED;
  }
  if (in_motion || std::abs(velocity) > velocity_tolerance) {
    return PregraspStabilitySampleStatus::WAITING;
  }
  return PregraspStabilitySampleStatus::STABLE;
}

}  // namespace xczs_inspection_robot_control
