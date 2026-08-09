// Copyright 2026
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XCZS_INSPECTION_ROBOT_CONTROL__PLANNING_SCENE_PROFILE_HPP_
#define XCZS_INSPECTION_ROBOT_CONTROL__PLANNING_SCENE_PROFILE_HPP_

#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace xczs_inspection_robot_control
{

struct SceneControlProfile
{
  std::string id;
  std::string type;
  std::string parent_control_id;
};

struct SceneArticulationProfile
{
  std::optional<std::string> door_control_id;
  std::optional<std::string> switch_control_id;
  std::string switch_parent_control_id;
};

inline SceneArticulationProfile resolve_scene_articulation(
  const std::vector<SceneControlProfile> & controls)
{
  SceneArticulationProfile result;
  std::unordered_map<std::string, std::string> control_types;
  control_types.reserve(controls.size());

  for (const auto & control : controls) {
    if (control.id.empty()) {
      throw std::invalid_argument("Scene control IDs must not be empty.");
    }
    if (!control_types.emplace(control.id, control.type).second) {
      throw std::invalid_argument(
              "Duplicate scene control ID '" + control.id + "'.");
    }
    if (control.type == "door") {
      if (result.door_control_id.has_value()) {
        throw std::invalid_argument(
                "The planning-scene adapter supports at most one door.");
      }
      result.door_control_id = control.id;
    } else if (control.type == "switch") {
      if (result.switch_control_id.has_value()) {
        throw std::invalid_argument(
                "The planning-scene adapter supports at most one switch.");
      }
      result.switch_control_id = control.id;
      result.switch_parent_control_id = control.parent_control_id;
    } else if (control.type != "button" && control.type != "knob") {
      throw std::invalid_argument(
              "No planning-scene adapter is configured for control '" +
              control.id + "' of type '" + control.type + "'.");
    }
  }

  if (!result.switch_parent_control_id.empty()) {
    const auto parent = control_types.find(result.switch_parent_control_id);
    if (parent == control_types.end()) {
      throw std::invalid_argument(
              "Switch parent '" + result.switch_parent_control_id +
              "' does not exist in the shared control catalog.");
    }
    if (parent->second != "door") {
      throw std::invalid_argument(
              "Switch parent '" + result.switch_parent_control_id +
              "' must be a door.");
    }
    if (!result.door_control_id.has_value() ||
      result.door_control_id.value() != result.switch_parent_control_id)
    {
      throw std::invalid_argument(
              "The switch parent must match the configured scene door.");
    }
  }

  return result;
}

}  // namespace xczs_inspection_robot_control

#endif  // XCZS_INSPECTION_ROBOT_CONTROL__PLANNING_SCENE_PROFILE_HPP_
