// Copyright 2026
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

namespace xczs_inspection_robot_control
{

enum class GoalTerminalDisposition
{
  SUCCEED,
  CANCEL,
  ABORT,
};

enum class ClientCancelDisposition
{
  ACCEPT_AND_STOP,
  REJECT_AFTER_PHYSICAL_COMMIT,
};

/**
 * Linearize a client cancel against the physical-outcome commit point.
 *
 * Before the requested physical state has been verified, cancellation owns
 * the race and must stop motion.  Afterwards, rejecting cancellation prevents
 * an already-applied cabinet side effect from being mislabeled as canceled
 * and retried.  Safe transport can still fail independently and abort the
 * action; this policy does not turn a transport failure into success.
 */
constexpr ClientCancelDisposition client_cancel_disposition(
  bool physical_outcome_committed) noexcept
{
  return physical_outcome_committed ?
         ClientCancelDisposition::REJECT_AFTER_PHYSICAL_COMMIT :
         ClientCancelDisposition::ACCEPT_AND_STOP;
}

/**
 * Select the terminal action state after operation cleanup has completed.
 *
 * Once the physical operation has succeeded, a cancel request arriving during
 * retreat, stow, lease release, or result finalization must not rewrite that
 * completed outcome as canceled.  Before physical success is committed,
 * cancellation still takes precedence over an ordinary failure.
 */
constexpr GoalTerminalDisposition goal_terminal_disposition(
  bool operation_succeeded, bool cancel_requested) noexcept
{
  if (operation_succeeded) {
    return GoalTerminalDisposition::SUCCEED;
  }
  return cancel_requested ?
         GoalTerminalDisposition::CANCEL : GoalTerminalDisposition::ABORT;
}

}  // namespace xczs_inspection_robot_control
