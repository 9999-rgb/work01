// Copyright 2026
// SPDX-License-Identifier: BSD-3-Clause

#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "xczs_inspection_robot_control/operation_validation_policy.hpp"
#include "xczs_inspection_robot_control/rotary_operation_policy.hpp"
#include "xczs_inspection_robot_control/staging_safety_policy.hpp"

namespace xczs_inspection_robot_control
{

TEST(OperationValidationPolicy, PlanningOnlyPreparationNeverOwnsBaseMotion)
{
  for (const bool navigation_requested : {false, true}) {
    for (const bool has_navigation_station : {false, true}) {
      const auto policy = operation_preparation_policy(
        true, navigation_requested, has_navigation_station);
      EXPECT_FALSE(policy.execute_embedded_navigation);
      EXPECT_FALSE(policy.enter_manual_base_mode);
      EXPECT_FALSE(policy.execute_precision_docking);
      EXPECT_FALSE(policy.wait_for_scene_settle);
      EXPECT_FALSE(policy.requires_physical_motion_resources);
    }
  }
}

TEST(OperationValidationPolicy, PhysicalPreparationUsesConfiguredMotionLayers)
{
  const auto task_layer_navigation = operation_preparation_policy(
    false, false, true);
  EXPECT_FALSE(task_layer_navigation.execute_embedded_navigation);
  EXPECT_TRUE(task_layer_navigation.enter_manual_base_mode);
  EXPECT_TRUE(task_layer_navigation.execute_precision_docking);
  EXPECT_TRUE(task_layer_navigation.wait_for_scene_settle);
  EXPECT_TRUE(task_layer_navigation.requires_physical_motion_resources);

  const auto embedded_navigation = operation_preparation_policy(
    false, true, true);
  EXPECT_TRUE(embedded_navigation.execute_embedded_navigation);
  EXPECT_TRUE(embedded_navigation.enter_manual_base_mode);
  EXPECT_TRUE(embedded_navigation.execute_precision_docking);
  EXPECT_TRUE(embedded_navigation.wait_for_scene_settle);
  EXPECT_TRUE(embedded_navigation.requires_physical_motion_resources);

  const auto no_station = operation_preparation_policy(false, false, false);
  EXPECT_FALSE(no_station.execute_embedded_navigation);
  EXPECT_TRUE(no_station.enter_manual_base_mode);
  EXPECT_FALSE(no_station.execute_precision_docking);
  EXPECT_TRUE(no_station.wait_for_scene_settle);
  EXPECT_TRUE(no_station.requires_physical_motion_resources);
}

TEST(OperationValidationPolicy, AcceptedOrPendingGoalDoesNotOwnMotionResources)
{
  EXPECT_FALSE(physical_motion_resources_are_owned(true, false, true));
  EXPECT_FALSE(physical_motion_resources_are_owned(true, false, false));
}

TEST(OperationValidationPolicy, ActivePhysicalGoalOwnsResourcesAfterLease)
{
  EXPECT_TRUE(physical_motion_resources_are_owned(true, true, true));
  EXPECT_FALSE(physical_motion_resources_are_owned(true, true, false));
}

TEST(OperationValidationPolicy, PlanningOnlyNeverOwnsMotionResources)
{
  EXPECT_FALSE(physical_motion_resources_are_owned(false, false, true));
  EXPECT_FALSE(physical_motion_resources_are_owned(false, true, true));
}

TEST(OperationValidationPolicy, UnverifiedControlRequiresPlanningOnly)
{
  EXPECT_TRUE(requires_planning_only_validation(false));
  EXPECT_FALSE(physical_motion_is_permitted(false, true));
}

TEST(OperationValidationPolicy, VerifiedControlMayUsePhysicalExecution)
{
  EXPECT_FALSE(requires_planning_only_validation(true));
  EXPECT_TRUE(physical_motion_is_permitted(true, false));
}

TEST(OperationValidationPolicy, ValidationAlwaysBlocksPhysicalMotion)
{
  EXPECT_FALSE(physical_motion_is_permitted(true, true));
  EXPECT_FALSE(physical_motion_is_permitted(false, true));
}

TEST(OperationValidationPolicy, PreflightAndDockingFailuresNeedNoRecoveryMotion)
{
  EXPECT_FALSE(physical_recovery_is_required(false, false, false));
}

TEST(OperationValidationPolicy, ExecutedOperationStillRequiresSafetyRecovery)
{
  EXPECT_TRUE(physical_recovery_is_required(true, false, false));
}

TEST(OperationValidationPolicy, EngagementFlagsConservativelyRequireRecovery)
{
  EXPECT_TRUE(physical_recovery_is_required(false, true, false));
  EXPECT_TRUE(physical_recovery_is_required(false, false, true));
}

TEST(StagingSafetyPolicy, UsesAsymmetricFootprintInCabinetDirection)
{
  const std::vector<PlanarFootprintPoint> footprint{
    {0.42, 0.42}, {0.42, -0.42}, {-0.55, -0.42}, {-0.55, 0.42}};

  EXPECT_NEAR(cabinet_facing_footprint_extent(footprint, 0.0), 0.42, 1.0e-12);
  EXPECT_NEAR(
    cabinet_facing_footprint_extent(footprint, std::acos(-1.0)),
    0.55, 1.0e-12);
  EXPECT_NEAR(
    cabinet_facing_footprint_extent(footprint, 0.5 * std::acos(-1.0)),
    0.42, 1.0e-12);
}

TEST(StagingSafetyPolicy, IncludesPaddingAndWorstDockingError)
{
  const std::vector<PlanarFootprintPoint> footprint{
    {0.42, 0.42}, {0.42, -0.42}, {-0.55, -0.42}, {-0.55, 0.42}};
  constexpr double yaw_tolerance = 0.07;
  const double expected_rotated_extent =
    0.42 * (std::cos(yaw_tolerance) + std::sin(yaw_tolerance));
  EXPECT_NEAR(
    worst_cabinet_facing_footprint_extent(
      footprint, 0.0, yaw_tolerance),
    expected_rotated_extent, 1.0e-12);
  const double minimum_standoff = expected_rotated_extent + 0.03 + 0.008;
  EXPECT_NEAR(
    minimum_safe_station_standoff(
      footprint, 0.03, 0.008, yaw_tolerance, 0.0),
    minimum_standoff, 1.0e-12);
  EXPECT_FALSE(
    station_standoff_is_safe(
      0.400, footprint, 0.03, 0.008, yaw_tolerance, 0.0));
  EXPECT_FALSE(
    station_standoff_is_safe(
      0.458, footprint, 0.03, 0.008, yaw_tolerance, 0.0));
  EXPECT_TRUE(
    station_standoff_is_safe(
      minimum_standoff, footprint, 0.03, 0.008, yaw_tolerance, 0.0));
}

TEST(StagingSafetyPolicy, InvalidFootprintFailsClosed)
{
  EXPECT_FALSE(
    station_standoff_is_safe(
      1.0, {{0.2, 0.2}, {0.2, -0.2}}, 0.03, 0.01, 0.05, 0.0));
  EXPECT_FALSE(
    station_standoff_is_safe(
      1.0,
      {{0.2, 0.2}, {0.2, -0.2},
        {std::numeric_limits<double>::quiet_NaN(), 0.2}},
      0.03, 0.01, 0.05, 0.0));
}

TEST(StagingSafetyPolicy, WideYawIntervalUsesEachVertexRadialExtent)
{
  const std::vector<PlanarFootprintPoint> footprint{
    {0.3, 0.4}, {0.3, -0.4}, {-0.3, -0.4}, {-0.3, 0.4}};
  const double first_vertex_maximum = std::atan2(-0.4, 0.3);
  EXPECT_NEAR(
    worst_cabinet_facing_footprint_extent(
      footprint, first_vertex_maximum, 0.01),
    0.5, 1.0e-12);
  EXPECT_NEAR(
    worst_cabinet_facing_footprint_extent(
      footprint, 0.0, std::acos(-1.0)),
    0.5, 1.0e-12);
}

TEST(StagingSafetyPolicy, RequiresBothStoppedPoseErrorsWithinTolerance)
{
  EXPECT_TRUE(staging_pose_error_is_safe(0.008, -0.07, 0.008, 0.07));
  EXPECT_FALSE(staging_pose_error_is_safe(0.0081, 0.0, 0.008, 0.07));
  EXPECT_FALSE(staging_pose_error_is_safe(0.0, -0.071, 0.008, 0.07));
  EXPECT_FALSE(
    staging_pose_error_is_safe(
      std::numeric_limits<double>::quiet_NaN(), 0.0, 0.008, 0.07));
}

TEST(RotaryOperationPolicy, TransitionMatrixUsesSourceThenTarget)
{
  EXPECT_EQ(rotary_transition_matrix_index(0U, 1U, 3U), 1U);
  EXPECT_EQ(rotary_transition_matrix_index(1U, 0U, 3U), 3U);
  EXPECT_EQ(rotary_transition_matrix_index(2U, 1U, 3U), 7U);
}

TEST(RotaryOperationPolicy, ThreePositionKnobRejectsOnlySkippedDetent)
{
  EXPECT_TRUE(rotary_transition_is_adjacent(0U, 0U));
  EXPECT_TRUE(rotary_transition_is_adjacent(0U, 1U));
  EXPECT_TRUE(rotary_transition_is_adjacent(2U, 1U));
  EXPECT_FALSE(rotary_transition_is_adjacent(0U, 2U));
  EXPECT_FALSE(rotary_transition_is_adjacent(2U, 0U));
}

TEST(RotaryOperationPolicy, PartialReleaseCrossesAdjacentDetentMidpoint)
{
  constexpr double detent = 0.78539816339;
  const double center_to_right = rotary_release_position(
    0.0, detent, 0.60, 0.035);
  const double right_to_center = rotary_release_position(
    detent, 0.0, 0.60, 0.035);
  EXPECT_GT(center_to_right, 0.5 * detent);
  EXPECT_LT(right_to_center, 0.5 * detent);
  EXPECT_DOUBLE_EQ(
    rotary_release_position(detent, detent, 0.60, 0.035), detent);
}

}  // namespace xczs_inspection_robot_control
