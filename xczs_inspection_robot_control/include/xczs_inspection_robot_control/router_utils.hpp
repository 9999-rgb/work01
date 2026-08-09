// Copyright 2026
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XCZS_INSPECTION_ROBOT_CONTROL__ROUTER_UTILS_HPP_
#define XCZS_INSPECTION_ROBOT_CONTROL__ROUTER_UTILS_HPP_

#include <algorithm>
#include <cmath>
#include <string>
#include <unordered_set>
#include <vector>

namespace xczs_inspection_robot_control
{

struct PlanarVector
{
  double x{0.0};
  double y{0.0};
};

inline PlanarVector rotate_planar_vector(
  const PlanarVector & input,
  double yaw_offset) noexcept
{
  const double cosine = std::cos(yaw_offset);
  const double sine = std::sin(yaw_offset);
  return {
    cosine * input.x - sine * input.y,
    sine * input.x + cosine * input.y,
  };
}

inline double navigation_yaw_in_model_frame(
  double navigation_yaw,
  double navigation_velocity_yaw_offset) noexcept
{
  return navigation_yaw - navigation_velocity_yaw_offset;
}

inline bool has_unique_nonempty_names(
  const std::vector<std::string> & names,
  bool allow_empty = false)
{
  if (names.empty()) {
    return allow_empty;
  }
  std::unordered_set<std::string> unique_names;
  return std::all_of(
    names.begin(), names.end(),
    [&unique_names](const std::string & name) {
      return !name.empty() && unique_names.insert(name).second;
    });
}

inline bool groups_are_disjoint(
  const std::vector<std::string> & first,
  const std::vector<std::string> & second)
{
  const std::unordered_set<std::string> first_names(
    first.begin(), first.end());
  return std::none_of(
    second.begin(), second.end(),
    [&first_names](const std::string & name) {
      return first_names.count(name) != 0U;
    });
}

inline bool embedded_navigation_request_is_supported(
  bool navigation_requested,
  bool allow_embedded_navigation) noexcept
{
  return !navigation_requested || allow_embedded_navigation;
}

}  // namespace xczs_inspection_robot_control

#endif  // XCZS_INSPECTION_ROBOT_CONTROL__ROUTER_UTILS_HPP_
