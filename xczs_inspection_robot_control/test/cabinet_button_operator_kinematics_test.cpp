// Copyright 2026
// SPDX-License-Identifier: BSD-3-Clause

#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "moveit/robot_model/robot_model.h"
#include "srdfdom/model.h"
#include "urdf_parser/urdf_parser.h"

namespace xczs_inspection_robot_control
{

std::optional<std::string> tool_profile_kinematics_validation_error(
  const moveit::core::RobotModelConstPtr & robot_model,
  std::vector<std::string> move_group_names);

namespace
{

moveit::core::RobotModelPtr make_robot_model_without_kinematics_solver()
{
  constexpr char urdf_text[] = R"(
    <robot name="kinematics_validation_test_robot">
      <link name="base_link"/>
      <link name="tool_link"/>
      <joint name="arm_joint" type="revolute">
        <parent link="base_link"/>
        <child link="tool_link"/>
        <axis xyz="0 0 1"/>
        <limit lower="-1.0" upper="1.0" effort="1.0" velocity="1.0"/>
      </joint>
    </robot>
  )";
  constexpr char srdf_text[] = R"(
    <robot name="kinematics_validation_test_robot">
      <group name="configured_arm">
        <joint name="arm_joint"/>
      </group>
    </robot>
  )";

  const auto urdf_model = urdf::parseURDF(urdf_text);
  if (!urdf_model) {
    throw std::runtime_error("Failed to parse the unit-test URDF.");
  }
  auto srdf_model = std::make_shared<srdf::Model>();
  if (!srdf_model->initString(*urdf_model, srdf_text)) {
    throw std::runtime_error("Failed to parse the unit-test SRDF.");
  }
  return std::make_shared<moveit::core::RobotModel>(
    urdf_model, srdf_model);
}

TEST(ToolProfileKinematicsValidation, NamesGroupWhenRobotModelIsUnavailable)
{
  const auto error = tool_profile_kinematics_validation_error(
    nullptr, {"right_arm"});

  ASSERT_TRUE(error.has_value());
  EXPECT_NE(error->find("RobotModel"), std::string::npos);
  EXPECT_NE(error->find("'right_arm'"), std::string::npos);
}

TEST(ToolProfileKinematicsValidation, RejectsAndNamesMissingJointModelGroup)
{
  const auto robot_model = make_robot_model_without_kinematics_solver();
  const auto error = tool_profile_kinematics_validation_error(
    robot_model, {"missing_arm"});

  ASSERT_TRUE(error.has_value());
  EXPECT_NE(error->find("missing MoveIt JointModelGroup"), std::string::npos);
  EXPECT_NE(error->find("'missing_arm'"), std::string::npos);
}

TEST(ToolProfileKinematicsValidation, RejectsExistingGroupWithoutSolver)
{
  const auto robot_model = make_robot_model_without_kinematics_solver();
  const auto * joint_model_group =
    robot_model->getJointModelGroup("configured_arm");
  ASSERT_NE(joint_model_group, nullptr);
  ASSERT_EQ(joint_model_group->getSolverInstance(), nullptr);

  const auto error = tool_profile_kinematics_validation_error(
    robot_model, {"configured_arm"});

  ASSERT_TRUE(error.has_value());
  EXPECT_NE(error->find("kinematics solver"), std::string::npos);
  EXPECT_NE(error->find("'configured_arm'"), std::string::npos);
}

TEST(ToolProfileKinematicsValidation, ReportsEveryInvalidUniqueProfileGroup)
{
  const auto robot_model = make_robot_model_without_kinematics_solver();
  const auto error = tool_profile_kinematics_validation_error(
    robot_model, {"missing_arm", "configured_arm", "missing_arm"});

  ASSERT_TRUE(error.has_value());
  EXPECT_NE(error->find("'configured_arm'"), std::string::npos);
  EXPECT_NE(error->find("'missing_arm'"), std::string::npos);
  EXPECT_EQ(
    error->find("'missing_arm'"),
    error->rfind("'missing_arm'"));
}

}  // namespace
}  // namespace xczs_inspection_robot_control
