// Copyright 2026
// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <cmath>
#include <string>
#include <unordered_map>
#include <vector>

namespace xczs_inspection_robot_control
{
// JointState permits absent position/velocity/effort arrays. A populated
// array must match name in length; never index an optional or malformed array.
inline std::unordered_map<std::string, double> finite_joint_values(
  const std::vector<std::string> & names, const std::vector<double> & values)
{
  std::unordered_map<std::string, double> result;
  if (names.size() != values.size()) {return result;}
  result.reserve(names.size());
  for (std::size_t i = 0; i < names.size(); ++i) {
    if (names[i].empty() || !std::isfinite(values[i])) {continue;}
    if (!result.emplace(names[i], values[i]).second) {return {};}
  }
  return result;
}
}  // namespace xczs_inspection_robot_control
