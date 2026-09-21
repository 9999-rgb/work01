// Copyright 2026
// SPDX-License-Identifier: BSD-3-Clause
#include <gtest/gtest.h>
#include <limits>
#include "xczs_inspection_robot_control/insertion_alignment.hpp"
using xczs_inspection_robot_control::continuous_insert_is_aligned;
TEST(InsertionAlignment, AcceptsCoaxialClearApproach)
{
  EXPECT_TRUE(continuous_insert_is_aligned(.02, .00002, 1., 1.));
}
TEST(InsertionAlignment, RejectsWrongPositionOrOrientation)
{
  EXPECT_FALSE(continuous_insert_is_aligned(.002, .00002, 1., 1.));
  EXPECT_FALSE(continuous_insert_is_aligned(.10, .00002, 1., 1.));
  EXPECT_FALSE(continuous_insert_is_aligned(.02, .001, 1., 1.));
  EXPECT_FALSE(continuous_insert_is_aligned(.02, .00002, -1., 1.));
  EXPECT_FALSE(continuous_insert_is_aligned(.02, .00002, 1., std::cos(.2)));
}
TEST(InsertionAlignment, RejectsInvalidFeedback)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(continuous_insert_is_aligned(nan, 0., 1., 1.));
  EXPECT_FALSE(continuous_insert_is_aligned(.02, nan, 1., 1.));
  EXPECT_FALSE(continuous_insert_is_aligned(.02, 0., nan, 1.));
  EXPECT_FALSE(continuous_insert_is_aligned(.02, 0., 1., nan));
}
