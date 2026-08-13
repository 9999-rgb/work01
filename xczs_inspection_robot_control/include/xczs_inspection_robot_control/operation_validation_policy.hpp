// Copyright 2026
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

namespace xczs_inspection_robot_control
{

/**
 * Describe the base-motion preparation allowed before a cabinet operation.
 *
 * Planning-only validation must remain observational with respect to the
 * robot base.  In particular, it must not change the command router mode,
 * submit an embedded Nav2 goal, run precision docking, or wait for a physical
 * docking transition.  It also must not publish stop/cancel commands when the
 * validation goal is canceled or loses its lease, because those commands can
 * interrupt unrelated MoveIt/Nav2 users.  A physical operation requires these
 * resources, but does not own them merely because its action goal was accepted:
 * ownership begins only after the global operation lease is acquired while the
 * goal is still active.  Keeping this decision in a pure policy makes that
 * safety boundary independently testable from the ROS action implementation.
 */
struct OperationPreparationPolicy
{
  bool execute_embedded_navigation{false};
  bool enter_manual_base_mode{false};
  bool execute_precision_docking{false};
  bool wait_for_scene_settle{false};
  bool requires_physical_motion_resources{false};
};

constexpr OperationPreparationPolicy operation_preparation_policy(
  bool planning_only,
  bool embedded_navigation_requested,
  bool has_navigation_station) noexcept
{
  if (planning_only) {
    return {};
  }
  return {
    embedded_navigation_requested,
    true,
    has_navigation_station,
    true,
    true,
  };
}

/**
 * Return whether a goal currently owns the shared physical motion resources.
 *
 * Goal acceptance alone is deliberately insufficient.  This prevents a goal
 * canceled while its lease request is pending from stopping another lease
 * holder's MoveIt or Nav2 work.
 */
constexpr bool physical_motion_resources_are_owned(
  bool resources_required,
  bool operation_lease_acquired,
  bool goal_is_active) noexcept
{
  return resources_required && operation_lease_acquired && goal_is_active;
}

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
