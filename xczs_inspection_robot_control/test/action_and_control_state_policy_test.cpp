// Copyright 2026
// SPDX-License-Identifier: BSD-3-Clause

#include <limits>

#include "gtest/gtest.h"
#include "xczs_inspection_robot_control/action_terminal_policy.hpp"
#include "xczs_inspection_robot_control/cabinet_grasp_safety_policy.hpp"
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

TEST(StructuredControlStatePolicy, AcceptsFreshStationaryInitialDetentSample)
{
  EXPECT_EQ(
    PregraspStabilitySampleStatus::STABLE,
    classify_pregrasp_stability_sample(
      true, true, true, true, true, false,
      0.125, 0.100, -0.030, 0.025, 0.030));
}

TEST(StructuredControlStatePolicy, WaitsForUsableAndStationarySamples)
{
  EXPECT_EQ(
    PregraspStabilitySampleStatus::WAITING,
    classify_pregrasp_stability_sample(
      true, true, false, true, true, false,
      0.1, 0.1, 0.0, 0.025, 0.030));
  EXPECT_EQ(
    PregraspStabilitySampleStatus::WAITING,
    classify_pregrasp_stability_sample(
      true, false, true, true, true, false,
      0.1, 0.1, 0.0, 0.025, 0.030));
  EXPECT_EQ(
    PregraspStabilitySampleStatus::WAITING,
    classify_pregrasp_stability_sample(
      true, true, true, false, true, false,
      0.1, 0.1, 0.0, 0.025, 0.030));
  EXPECT_EQ(
    PregraspStabilitySampleStatus::WAITING,
    classify_pregrasp_stability_sample(
      true, true, true, true, true, true,
      0.1, 0.1, 0.0, 0.025, 0.030));
  EXPECT_EQ(
    PregraspStabilitySampleStatus::WAITING,
    classify_pregrasp_stability_sample(
      true, true, true, true, true, false,
      0.1, 0.1, 0.031, 0.025, 0.030));
}

TEST(StructuredControlStatePolicy, DetectsInitialStateOrPositionChange)
{
  EXPECT_EQ(
    PregraspStabilitySampleStatus::REFERENCE_CHANGED,
    classify_pregrasp_stability_sample(
      true, true, true, true, false, false,
      0.1, 0.1, 0.0, 0.025, 0.030));
  EXPECT_EQ(
    PregraspStabilitySampleStatus::REFERENCE_CHANGED,
    classify_pregrasp_stability_sample(
      true, true, true, true, true, false,
      0.126, 0.1, 0.0, 0.025, 0.030));
}

TEST(StructuredControlStatePolicy, FailsClosedForNonFiniteStabilityInputs)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(
    PregraspStabilitySampleStatus::WAITING,
    classify_pregrasp_stability_sample(
      true, true, true, true, true, false,
      nan, 0.1, 0.0, 0.025, 0.030));
  EXPECT_EQ(
    PregraspStabilitySampleStatus::WAITING,
    classify_pregrasp_stability_sample(
      true, true, true, true, true, false,
      0.1, 0.1, 0.0, 0.0, 0.030));
}

TEST(StructuredControlStatePolicy, TracksNegativeMotionAsAbsolutePeak)
{
  double peak = update_absolute_position_peak(0.0, -0.25);
  EXPECT_DOUBLE_EQ(peak, 0.25);
  peak = update_absolute_position_peak(peak, 0.10);
  EXPECT_DOUBLE_EQ(peak, 0.25);
  peak = update_absolute_position_peak(peak, -0.80);
  EXPECT_DOUBLE_EQ(peak, 0.80);
}

TEST(CabinetGraspSafetyPolicy, RequiresGraspExactlyForNonButtons)
{
  EXPECT_TRUE(grasp_requirement_matches_control_kind(true, false));
  EXPECT_TRUE(grasp_requirement_matches_control_kind(false, true));
  EXPECT_FALSE(grasp_requirement_matches_control_kind(true, true));
  EXPECT_FALSE(grasp_requirement_matches_control_kind(false, false));
}

TEST(CabinetGraspSafetyPolicy, RejectsNonFiniteRobotGraspPoints)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double infinity = std::numeric_limits<double>::infinity();
  EXPECT_TRUE(robot_grasp_point_is_finite(0.0, -0.1, 0.2));
  EXPECT_FALSE(robot_grasp_point_is_finite(nan, 0.0, 0.0));
  EXPECT_FALSE(robot_grasp_point_is_finite(0.0, infinity, 0.0));
  EXPECT_FALSE(robot_grasp_point_is_finite(0.0, 0.0, -infinity));
}

TEST(CabinetGraspSafetyPolicy, WatchdogArmsOnlyForOperationActivity)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(operation_watchdog_timeout_is_valid(4.0));
  EXPECT_FALSE(operation_watchdog_timeout_is_valid(0.0));
  EXPECT_FALSE(operation_watchdog_timeout_is_valid(nan));

  EXPECT_FALSE(operation_watchdog_has_expired(false, false, 100.0, 4.0));
  EXPECT_FALSE(operation_watchdog_has_expired(true, false, 3.999, 4.0));
  EXPECT_TRUE(operation_watchdog_has_expired(true, false, 4.0, 4.0));
  EXPECT_TRUE(operation_watchdog_has_expired(false, true, 4.1, 4.0));
}

TEST(CabinetGraspSafetyPolicy, ActiveWatchdogFailsClosedOnInvalidTiming)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(operation_watchdog_has_expired(true, false, nan, 4.0));
  EXPECT_TRUE(operation_watchdog_has_expired(false, true, -0.1, 4.0));
  EXPECT_TRUE(operation_watchdog_has_expired(true, true, 0.0, 0.0));
}

TEST(CabinetGraspSafetyPolicy, OperationSessionRequiresExactControlAndLease)
{
  EXPECT_TRUE(
    operation_session_matches(
      "rear_door", "lease-42", "rear_door", "lease-42"));
  EXPECT_FALSE(
    operation_session_matches(
      "rear_door", "lease-42", "rear_door", "lease-43"));
  EXPECT_FALSE(
    operation_session_matches(
      "rear_door", "lease-42", "side_knob", "lease-42"));
  EXPECT_FALSE(
    operation_session_matches(
      "rear_door", "", "rear_door", ""));
}

TEST(CabinetGraspSafetyPolicy, StaleReleaseCannotMatchSuccessorLease)
{
  EXPECT_TRUE(
    operation_session_matches(
      "rear_door", "lease-current", "rear_door", "lease-current"));
  EXPECT_FALSE(
    operation_session_matches(
      "rear_door", "lease-current", "rear_door", "lease-stale"));
  EXPECT_FALSE(
    operation_session_matches(
      "rear_door", "lease-current", "rear_door", ""));
}

TEST(CabinetGraspSafetyPolicy, StaleFaultCannotCancelAnotherLease)
{
  EXPECT_TRUE(
    operation_fault_matches_active_lease(
      true, "lease-current", "lease-current"));
  EXPECT_FALSE(
    operation_fault_matches_active_lease(
      true, "lease-current", "lease-stale"));
  EXPECT_FALSE(
    operation_fault_matches_active_lease(
      false, "lease-current", "lease-current"));
  EXPECT_FALSE(operation_fault_matches_active_lease(true, "", ""));
}

TEST(CabinetGraspSafetyPolicy, AcceptsOnlySettledPregraspDetent)
{
  EXPECT_FALSE(pregrasp_detent_is_disturbed(0.0, 0.0, 0.0, 0.025, 0.025));
  EXPECT_FALSE(
    pregrasp_detent_is_disturbed(0.025, -0.025, 0.0, 0.025, 0.025));
  EXPECT_TRUE(
    pregrasp_detent_is_disturbed(0.026, 0.0, 0.0, 0.025, 0.025));
  EXPECT_TRUE(
    pregrasp_detent_is_disturbed(0.0, -0.026, 0.0, 0.025, 0.025));
}

TEST(CabinetGraspSafetyPolicy, FailsClosedForInvalidMeasurementsOrLimits)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(pregrasp_detent_is_disturbed(nan, 0.0, 0.0, 0.025, 0.025));
  EXPECT_TRUE(pregrasp_detent_is_disturbed(0.0, nan, 0.0, 0.025, 0.025));
  EXPECT_TRUE(pregrasp_detent_is_disturbed(0.0, 0.0, nan, 0.025, 0.025));
  EXPECT_TRUE(pregrasp_detent_is_disturbed(0.0, 0.0, 0.0, 0.0, 0.025));
  EXPECT_TRUE(pregrasp_detent_is_disturbed(0.0, 0.0, 0.0, 0.025, -1.0));
}

}  // namespace xczs_inspection_robot_control
