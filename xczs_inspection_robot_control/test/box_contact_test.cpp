// Copyright 2026
// SPDX-License-Identifier: BSD-3-Clause
#include <gtest/gtest.h>
#include "xczs_inspection_robot_control/box_contact.hpp"

using namespace xczs_inspection_robot_control;

TEST(BoxContact, DistinguishesMicroscopicGapFromSeparationAndPenetration)
{
  ContactBox a{{0, 0, 0}, {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}}, {.01, .01, .01}};
  auto b = a;
  b.center[0] = .02002;
  EXPECT_NEAR(box_contact_separation(a, b), .00002, 1e-12);
  b.center[0] = .023;
  EXPECT_NEAR(box_contact_separation(a, b), .003, 1e-12);
  b.center[0] = .018;
  EXPECT_NEAR(box_contact_separation(a, b), -.002, 1e-12);
}

TEST(BoxContact, UsesOrientedFacesInsteadOfWorldBoundingBoxes)
{
  const double q = std::sqrt(.5);
  ContactBox a{{0, 0, 0}, {{{q, q, 0}, {-q, q, 0}, {0, 0, 1}}}, {.01, .002, .01}};
  auto b = a;
  b.center = {-.0042 * q, .0042 * q, 0};
  EXPECT_NEAR(box_contact_separation(a, b), .0002, 1e-12);
}

TEST(BoxContact, RejectsNonFiniteGeometry)
{
  ContactBox a{{0, 0, 0}, {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}}, {.01, .01, .01}};
  auto b = a;
  b.center[1] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(std::isfinite(box_contact_separation(a, b)));
}
