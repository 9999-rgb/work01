// Copyright 2026
// SPDX-License-Identifier: BSD-3-Clause

#include <stdexcept>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "xczs_inspection_robot_control/planning_scene_profile.hpp"

namespace xczs_inspection_robot_control
{

TEST(PlanningSceneProfile, AcceptsButtonAndKnobOnlyDevice)
{
  const auto profile = resolve_scene_articulation(
    {
      {"start_button", "button", ""},
      {"mode_knob", "knob", ""},
    });

  EXPECT_FALSE(profile.door_control_id.has_value());
  EXPECT_FALSE(profile.switch_control_id.has_value());
  EXPECT_TRUE(profile.switch_parent_control_id.empty());
}

TEST(PlanningSceneProfile, AcceptsDoorWithoutSwitch)
{
  const auto profile = resolve_scene_articulation(
    {{"access_door", "door", ""}});

  ASSERT_TRUE(profile.door_control_id.has_value());
  EXPECT_EQ(profile.door_control_id.value(), "access_door");
  EXPECT_FALSE(profile.switch_control_id.has_value());
}

TEST(PlanningSceneProfile, AcceptsUnparentedSwitchWithoutDoor)
{
  const auto profile = resolve_scene_articulation(
    {{"main_switch", "switch", ""}});

  EXPECT_FALSE(profile.door_control_id.has_value());
  ASSERT_TRUE(profile.switch_control_id.has_value());
  EXPECT_EQ(profile.switch_control_id.value(), "main_switch");
  EXPECT_TRUE(profile.switch_parent_control_id.empty());
}

TEST(PlanningSceneProfile, AcceptsSwitchParentedToDoor)
{
  const auto profile = resolve_scene_articulation(
    {
      {"door", "door", ""},
      {"switch", "switch", "door"},
    });

  ASSERT_TRUE(profile.door_control_id.has_value());
  ASSERT_TRUE(profile.switch_control_id.has_value());
  EXPECT_EQ(profile.door_control_id.value(), "door");
  EXPECT_EQ(profile.switch_control_id.value(), "switch");
  EXPECT_EQ(profile.switch_parent_control_id, "door");
}

TEST(PlanningSceneProfile, RejectsMissingSwitchParent)
{
  EXPECT_THROW(
    resolve_scene_articulation(
      {{"switch", "switch", "missing_door"}}),
    std::invalid_argument);
}

TEST(PlanningSceneProfile, RejectsSwitchParentThatIsNotDoor)
{
  EXPECT_THROW(
    resolve_scene_articulation(
    {
      {"button", "button", ""},
      {"switch", "switch", "button"},
    }),
    std::invalid_argument);
}

TEST(PlanningSceneProfile, RejectsMultipleDoorsOrSwitches)
{
  EXPECT_THROW(
    resolve_scene_articulation(
    {
      {"door_a", "door", ""},
      {"door_b", "door", ""},
    }),
    std::invalid_argument);
  EXPECT_THROW(
    resolve_scene_articulation(
    {
      {"switch_a", "switch", ""},
      {"switch_b", "switch", ""},
    }),
    std::invalid_argument);
}

}  // namespace xczs_inspection_robot_control
