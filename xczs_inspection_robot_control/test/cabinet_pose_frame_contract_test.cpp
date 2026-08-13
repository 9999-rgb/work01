// Copyright 2026
// SPDX-License-Identifier: BSD-3-Clause

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

#include "gtest/gtest.h"

namespace
{

std::string read_config(const std::string & name)
{
  const std::string path = std::string(XCZS_CONTROL_CONFIG_DIR) + "/" + name;
  std::ifstream stream(path);
  if (!stream) {
    throw std::runtime_error("Could not read " + path);
  }
  std::ostringstream contents;
  contents << stream.rdbuf();
  return contents.str();
}

}  // namespace

TEST(CabinetPoseFrameContract, SpawnAndPoseAuthorityShareOdomWorld)
{
  const auto adapter = read_config("cabinet_robot_adapter.yaml");
  const auto authority = read_config("cabinet_pose.yaml");

  EXPECT_NE(
    adapter.find("\n    planning_frame: odom\n"), std::string::npos);
  EXPECT_NE(
    adapter.find("\n    pose_parent_frame: odom\n"), std::string::npos);
  EXPECT_NE(
    authority.find("\n    parent_frame: odom\n"), std::string::npos);
}
