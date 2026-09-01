// Copyright 2026
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <cmath>
#include <string>

namespace xczs_inspection_robot_control
{

/**
 * Every articulated rotary control requires grasping; a button or slider
 * never does (a slider is driven by a face press, not a fixed grasp).
 */
constexpr bool grasp_requirement_matches_control_kind(
  bool is_grasp_free,
  bool requires_grasp) noexcept
{
  return requires_grasp == !is_grasp_free;
}

/** Validate a robot-link-local grasp probe before it reaches Gazebo maths. */
inline bool robot_grasp_point_is_finite(
  double x,
  double y,
  double z) noexcept
{
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

/** Validate the wall-clock timeout used to recover an abandoned operation. */
inline bool operation_watchdog_timeout_is_valid(double timeout) noexcept
{
  return std::isfinite(timeout) && timeout > 0.0;
}

/** Bind a physical operation to one exact control and global lease session. */
inline bool operation_session_matches(
  const std::string & expected_control_id,
  const std::string & expected_lease_id,
  const std::string & actual_control_id,
  const std::string & actual_lease_id) noexcept
{
  return !expected_control_id.empty() && !expected_lease_id.empty() &&
         expected_control_id == actual_control_id &&
         expected_lease_id == actual_lease_id;
}

/** Ignore delayed fault messages from an expired or superseded lease. */
inline bool operation_fault_matches_active_lease(
  bool lease_held,
  const std::string & active_lease_id,
  const std::string & fault_lease_id) noexcept
{
  return lease_held && !active_lease_id.empty() &&
         active_lease_id == fault_lease_id;
}

/**
 * Decide whether an active control/grasp has exceeded its heartbeat timeout.
 *
 * No operation activity means there is deliberately nothing to monitor.
 * Invalid elapsed-time input fails closed once physical activity exists.
 */
inline bool operation_watchdog_has_expired(
  bool active_control,
  bool active_grasp,
  double elapsed_since_heartbeat,
  double timeout) noexcept
{
  if (!active_control && !active_grasp) {
    return false;
  }
  if (!operation_watchdog_timeout_is_valid(timeout) ||
    !std::isfinite(elapsed_since_heartbeat) ||
    elapsed_since_heartbeat < 0.0)
  {
    return true;
  }
  return elapsed_since_heartbeat >= timeout;
}

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
