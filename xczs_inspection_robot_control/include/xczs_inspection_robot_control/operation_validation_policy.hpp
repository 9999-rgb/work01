// Copyright 2026
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

namespace xczs_inspection_robot_control
{

/**
 * Decide whether an operation must be limited to collision-checked planning.
 *
 * ``operable`` means that the current robot adapter has passed the complete
 * physical operation and recovery contract for this control.  A control that
 * has not passed that contract may still be inspected with live MoveIt
 * planning, but it must never receive a trajectory or grasp command.
 */
constexpr bool requires_planning_only_validation(bool operable) noexcept
{
  return !operable;
}

/** Return whether the current policy permits a physical motion command. */
constexpr bool physical_motion_is_permitted(
  bool operable, bool validation_performed) noexcept
{
  return operable && !validation_performed;
}

/**
 * Return whether an aborted operation has entered a state that needs physical
 * recovery.
 *
 * Planning, preflight checks and base docking do not move the arm or attach a
 * cabinet grasp.  A failure in those phases must therefore leave the arm
 * alone.  Once an arm/cabinet command has been issued, or either explicit
 * safety flag says that the tool may still be engaged, retreat/stow recovery
 * remains required.
 */
constexpr bool physical_recovery_is_required(
  bool operation_executed,
  bool retreat_required,
  bool grasp_attached) noexcept
{
  return operation_executed || retreat_required || grasp_attached;
}

}  // namespace xczs_inspection_robot_control
