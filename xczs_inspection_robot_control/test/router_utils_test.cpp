// Copyright 2026
// SPDX-License-Identifier: BSD-3-Clause

#include <cmath>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "xczs_inspection_robot_control/router_utils.hpp"

namespace xczs_inspection_robot_control
{

TEST(RouterUtils, RotatesPlanarVelocityByConfiguredYaw)
{
  constexpr double kHalfPi = 1.5707963267948966;
  const auto positive = rotate_planar_vector({0.4, -0.2}, kHalfPi);
  EXPECT_NEAR(positive.x, 0.2, 1.0e-12);
  EXPECT_NEAR(positive.y, 0.4, 1.0e-12);

  const auto negative = rotate_planar_vector({0.4, -0.2}, -kHalfPi);
  EXPECT_NEAR(negative.x, -0.2, 1.0e-12);
  EXPECT_NEAR(negative.y, -0.4, 1.0e-12);
}

TEST(RouterUtils, RotationPreservesPlanarMagnitude)
{
  const PlanarVector input{0.31, -0.47};
  const auto output = rotate_planar_vector(input, 0.73);
  EXPECT_NEAR(
    std::hypot(output.x, output.y),
    std::hypot(input.x, input.y),
    1.0e-12);
}

TEST(RouterUtils, UsesTheSameOffsetForDockingFrameYaw)
{
  constexpr double kHalfPi = 1.5707963267948966;
  EXPECT_NEAR(navigation_yaw_in_model_frame(0.0, kHalfPi), -kHalfPi, 1.0e-12);
  EXPECT_NEAR(
    navigation_yaw_in_model_frame(0.37, -0.21), 0.58, 1.0e-12);
}

TEST(RouterUtils, RejectsInvalidJointGroups)
{
  EXPECT_FALSE(has_unique_nonempty_names({}));
  EXPECT_FALSE(has_unique_nonempty_names({"joint_a", ""}));
  EXPECT_FALSE(has_unique_nonempty_names({"joint_a", "joint_a"}));
  EXPECT_TRUE(has_unique_nonempty_names({"joint_a", "joint_b"}));
  EXPECT_TRUE(has_unique_nonempty_names({}, true));

  EXPECT_TRUE(groups_are_disjoint({"joint_a"}, {"joint_b"}));
  EXPECT_FALSE(groups_are_disjoint({"joint_a"}, {"joint_a"}));
}

TEST(RouterUtils, GatesOnlyEmbeddedNavigationRequests)
{
  EXPECT_TRUE(embedded_navigation_request_is_supported(false, false));
  EXPECT_TRUE(embedded_navigation_request_is_supported(false, true));
  EXPECT_FALSE(embedded_navigation_request_is_supported(true, false));
  EXPECT_TRUE(embedded_navigation_request_is_supported(true, true));
}

}  // namespace xczs_inspection_robot_control
