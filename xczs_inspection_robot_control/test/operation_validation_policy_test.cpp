// Copyright 2026
// SPDX-License-Identifier: BSD-3-Clause

#include <gtest/gtest.h>

#include "xczs_inspection_robot_control/operation_validation_policy.hpp"

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

  const auto embedded_navigation = operation_preparation_policy(
    false, true, true);
  EXPECT_TRUE(embedded_navigation.execute_embedded_navigation);
  EXPECT_TRUE(embedded_navigation.enter_manual_base_mode);
  EXPECT_TRUE(embedded_navigation.execute_precision_docking);
  EXPECT_TRUE(embedded_navigation.wait_for_scene_settle);

  const auto no_station = operation_preparation_policy(false, false, false);
  EXPECT_FALSE(no_station.execute_embedded_navigation);
  EXPECT_TRUE(no_station.enter_manual_base_mode);
  EXPECT_FALSE(no_station.execute_precision_docking);
  EXPECT_TRUE(no_station.wait_for_scene_settle);
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

}  // namespace xczs_inspection_robot_control
