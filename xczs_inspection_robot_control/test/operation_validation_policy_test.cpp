// Copyright 2026
// SPDX-License-Identifier: BSD-3-Clause

#include <gtest/gtest.h>

#include "xczs_inspection_robot_control/operation_validation_policy.hpp"

namespace xczs_inspection_robot_control
{

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

}  // namespace xczs_inspection_robot_control
