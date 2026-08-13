// Copyright 2026
// SPDX-License-Identifier: BSD-3-Clause

#include <limits>

#include "gtest/gtest.h"
#include "xczs_inspection_robot_control/action_terminal_policy.hpp"
#include "xczs_inspection_robot_control/structured_control_state_policy.hpp"

namespace xczs_inspection_robot_control
{

TEST(ActionTerminalPolicy, PhysicalSuccessWinsAConcurrentLateCancel)
{
  EXPECT_EQ(
    GoalTerminalDisposition::SUCCEED,
    goal_terminal_disposition(true, true));
  EXPECT_EQ(
    GoalTerminalDisposition::SUCCEED,
    goal_terminal_disposition(true, false));
}

TEST(ActionTerminalPolicy, CancelWinsOnlyBeforeSuccessIsCommitted)
{
  EXPECT_EQ(
    GoalTerminalDisposition::CANCEL,
    goal_terminal_disposition(false, true));
  EXPECT_EQ(
    GoalTerminalDisposition::ABORT,
    goal_terminal_disposition(false, false));
}

TEST(ActionTerminalPolicy, AppliesEveryTerminalDispositionExactlyOnce)
{
  int succeed_count = 0;
  int cancel_count = 0;
  int abort_count = 0;
  const auto succeed = [&]() {++succeed_count;};
  const auto cancel = [&]() {++cancel_count;};
  const auto abort = [&]() {++abort_count;};

  EXPECT_TRUE(
    apply_goal_terminal_disposition(
      GoalTerminalDisposition::SUCCEED, succeed, cancel, abort));
  EXPECT_EQ(succeed_count, 1);
  EXPECT_EQ(cancel_count, 0);
  EXPECT_EQ(abort_count, 0);

  EXPECT_FALSE(
    apply_goal_terminal_disposition(
      GoalTerminalDisposition::CANCEL, succeed, cancel, abort));
  EXPECT_EQ(succeed_count, 1);
  EXPECT_EQ(cancel_count, 1);
  EXPECT_EQ(abort_count, 0);

  EXPECT_FALSE(
    apply_goal_terminal_disposition(
      GoalTerminalDisposition::ABORT, succeed, cancel, abort));
  EXPECT_EQ(succeed_count, 1);
  EXPECT_EQ(cancel_count, 1);
  EXPECT_EQ(abort_count, 1);
}

TEST(ActionTerminalPolicy, CancelIsAcceptedBeforePhysicalCommit)
{
  EXPECT_EQ(
    ClientCancelDisposition::ACCEPT_AND_STOP,
    client_cancel_disposition(false));
}

TEST(ActionTerminalPolicy, CancelIsRejectedAfterPhysicalCommit)
{
  EXPECT_EQ(
    ClientCancelDisposition::REJECT_AFTER_PHYSICAL_COMMIT,
    client_cancel_disposition(true));

  // A committed physical outcome does not itself imply action success: safe
  // transport failure still follows the abort path.
  EXPECT_EQ(
    GoalTerminalDisposition::ABORT,
    goal_terminal_disposition(false, false));
}

TEST(StructuredControlStatePolicy, RejectsDeclaredInvalidAndNonFiniteSamples)
{
  constexpr double finite = 0.1;
  EXPECT_EQ(
    StructuredControlStateUpdate::INVALIDATE,
    classify_structured_control_state(true, false, finite, finite, finite));
  EXPECT_EQ(
    StructuredControlStateUpdate::INVALIDATE,
    classify_structured_control_state(
      true, true, std::numeric_limits<double>::quiet_NaN(), finite, finite));
  EXPECT_EQ(
    StructuredControlStateUpdate::INVALIDATE,
    classify_structured_control_state(
      true, true, finite, std::numeric_limits<double>::infinity(), finite));
  EXPECT_EQ(
    StructuredControlStateUpdate::INVALIDATE,
    classify_structured_control_state(
      true, true, finite, finite,
      -std::numeric_limits<double>::infinity()));
}

TEST(StructuredControlStatePolicy, IgnoresAnotherControlsSample)
{
  EXPECT_EQ(
    StructuredControlStateUpdate::IGNORE,
    classify_structured_control_state(false, false, 0.0, 0.0, 0.0));
}

TEST(StructuredControlStatePolicy, JointTrafficCannotKeepStructuredStateAlive)
{
  // A fresh JointState is deliberately not an input to this decision.
  EXPECT_FALSE(structured_control_state_is_usable(false, true, true, true));
  EXPECT_FALSE(structured_control_state_is_usable(true, false, true, true));
  EXPECT_FALSE(structured_control_state_is_usable(true, true, false, true));
  EXPECT_FALSE(structured_control_state_is_usable(true, true, true, false));
  EXPECT_TRUE(structured_control_state_is_usable(true, true, true, true));
}

}  // namespace xczs_inspection_robot_control
