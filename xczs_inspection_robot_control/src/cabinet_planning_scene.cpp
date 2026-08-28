// Copyright 2026
// SPDX-License-Identifier: BSD-3-Clause

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_set>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "moveit_msgs/msg/planning_scene.hpp"
#include "rclcpp/rclcpp.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "xczs_inspection_robot_control/cabinet_grasp_safety_policy.hpp"
#include "xczs_inspection_robot_interfaces/msg/cabinet_control_state.hpp"
#include "xczs_inspection_robot_control/planning_scene_profile.hpp"

namespace xczs_inspection_robot_control
{

namespace
{

using CabinetControlState =
  xczs_inspection_robot_interfaces::msg::CabinetControlState;

constexpr std::size_t kExpiredLeaseHistoryLimit = 64U;

struct BoxPart
{
  tf2::Vector3 size;
  tf2::Vector3 position;
};

struct ControlCollision
{
  std::string id;
  tf2::Vector3 center;
  tf2::Vector3 axis;
  bool rotary{false};
};

geometry_msgs::msg::Pose to_pose(const tf2::Transform & transform)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform.getOrigin().x();
  pose.position.y = transform.getOrigin().y();
  pose.position.z = transform.getOrigin().z();
  pose.orientation.x = transform.getRotation().x();
  pose.orientation.y = transform.getRotation().y();
  pose.orientation.z = transform.getRotation().z();
  pose.orientation.w = transform.getRotation().w();
  return pose;
}

shape_msgs::msg::SolidPrimitive box(const tf2::Vector3 & size)
{
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  primitive.dimensions = {size.x(), size.y(), size.z()};
  return primitive;
}

shape_msgs::msg::SolidPrimitive cylinder(double height, double radius)
{
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  primitive.dimensions.resize(2);
  primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] =
    height;
  primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS] =
    radius;
  return primitive;
}

tf2::Quaternion cylinder_rotation(const tf2::Vector3 & axis)
{
  const auto normalized_axis = axis.normalized();
  tf2::Quaternion rotation = tf2::shortestArcQuat(
    tf2::Vector3(0.0, 0.0, 1.0), normalized_axis);
  rotation.normalize();
  return rotation;
}

bool transform_changed(
  const tf2::Transform & first,
  const tf2::Transform & second)
{
  return first.getOrigin().distance(second.getOrigin()) > 1.0e-5 ||
         first.getRotation().angleShortestPath(second.getRotation()) > 1.0e-5;
}

}  // namespace

class CabinetPlanningScene final : public rclcpp::Node
{
public:
  CabinetPlanningScene()
  : Node("xczs_cabinet_planning_scene")
  {
    frame_id_ = required_string_parameter("frame_id", "odom");
    cabinet_frame_ = required_string_parameter(
      "cabinet_frame", "control_cabinet_frame");
    collision_object_prefix_ = declare_parameter<std::string>(
      "collision_object_prefix", "");
    require_pose_valid_ = declare_parameter<bool>("require_pose_valid", false);
    operation_watchdog_timeout_ = declare_parameter<double>(
      "operation_watchdog_timeout", 2.0);
    if (!operation_watchdog_timeout_is_valid(operation_watchdog_timeout_)) {
      throw std::invalid_argument(
              "Parameter 'operation_watchdog_timeout' must be finite and "
              "positive.");
    }
    const auto pose_valid_topic = required_string_parameter(
      "pose_valid_topic", "pose_valid");

    load_control_catalog();
    load_scene_profile();
    load_control_collisions();

    transform_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    transform_listener_ =
      std::make_unique<tf2_ros::TransformListener>(*transform_buffer_);
    planning_scene_publisher_ =
      create_publisher<moveit_msgs::msg::PlanningScene>(
      "/planning_scene", rclcpp::QoS(1).reliable().transient_local());

    active_control_subscription_ = create_subscription<std_msgs::msg::String>(
      "active_control",
      rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::String::SharedPtr message) {
        receive_active_control(message->data);
      });
    operation_heartbeat_subscription_ =
      create_subscription<std_msgs::msg::String>(
      "operation_heartbeat", rclcpp::QoS(1).reliable(),
      [this](const std_msgs::msg::String::SharedPtr message) {
        receive_operation_heartbeat(message->data);
      });
    operation_fault_subscription_ =
      create_subscription<std_msgs::msg::String>(
      "operation_fault", rclcpp::QoS(1).reliable(),
      [this](const std_msgs::msg::String::SharedPtr message) {
        receive_operation_fault(message->data);
      });
    pose_valid_subscription_ = create_subscription<std_msgs::msg::Bool>(
      pose_valid_topic,
      rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::Bool::SharedPtr message) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (pose_valid_ != message->data || !pose_valid_received_) {
          pose_valid_ = message->data;
          pose_valid_received_ = true;
          ++scene_revision_;
        }
      });
    if (has_door()) {
      door_state_subscription_ = create_control_state_subscription(
        door_control_id_, [this](double position) {
          std::lock_guard<std::mutex> lock(state_mutex_);
          if (std::abs(door_position_ - position) > 1.0e-3) {
            door_position_ = position;
            ++scene_revision_;
          }
        });
    }
    if (has_switch()) {
      switch_state_subscription_ = create_control_state_subscription(
        switch_control_id_, [this](double position) {
          std::lock_guard<std::mutex> lock(state_mutex_);
          if (std::abs(switch_position_ - position) > 1.0e-3) {
            switch_position_ = position;
            ++scene_revision_;
          }
        });
    }

    retry_timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      [this]() {publish_scene_if_needed();});
  }

private:
  void remember_expired_operation_lease_locked(
    const std::string & operation_lease_id)
  {
    if (operation_lease_id.empty() ||
      !expired_operation_lease_ids_.insert(operation_lease_id).second)
    {
      return;
    }
    expired_operation_lease_history_.push_back(operation_lease_id);
    while (expired_operation_lease_history_.size() >
      kExpiredLeaseHistoryLimit)
    {
      expired_operation_lease_ids_.erase(
        expired_operation_lease_history_.front());
      expired_operation_lease_history_.pop_front();
    }
  }

  void clear_active_operation_locked(bool expire_lease)
  {
    if (expire_lease) {
      remember_expired_operation_lease_locked(operation_heartbeat_lease_id_);
    }
    active_control_id_.clear();
    operation_heartbeat_received_ = false;
    operation_heartbeat_lease_id_.clear();
    operation_last_heartbeat_ =
      std::chrono::steady_clock::time_point{};
    operation_monitor_started_at_ =
      std::chrono::steady_clock::time_point{};
    watchdog_operation_lease_id_.clear();
    ++active_control_generation_;
    watchdog_active_control_generation_ = active_control_generation_;
    ++scene_revision_;
  }

  void receive_active_control(const std::string & control_id)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (active_control_id_ == control_id) {
      return;
    }
    remember_expired_operation_lease_locked(operation_heartbeat_lease_id_);
    active_control_id_ = control_id;
    operation_heartbeat_received_ = false;
    operation_heartbeat_lease_id_.clear();
    operation_last_heartbeat_ =
      std::chrono::steady_clock::time_point{};
    ++active_control_generation_;
    watchdog_active_control_generation_ = active_control_generation_;
    watchdog_operation_lease_id_.clear();
    operation_monitor_started_at_ = control_id.empty() ?
      std::chrono::steady_clock::time_point{} :
    std::chrono::steady_clock::now();
    ++scene_revision_;
  }

  void receive_operation_heartbeat(const std::string & operation_lease_id)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (active_control_id_.empty() || operation_lease_id.empty() ||
      expired_operation_lease_ids_.count(operation_lease_id) != 0U)
    {
      return;
    }
    const auto now = std::chrono::steady_clock::now();
    const bool collision_exemption_was_active =
      operation_heartbeat_received_;
    if (operation_heartbeat_lease_id_ != operation_lease_id) {
      remember_expired_operation_lease_locked(
        operation_heartbeat_lease_id_);
      operation_monitor_started_at_ = now;
      watchdog_operation_lease_id_ = operation_lease_id;
    }
    operation_heartbeat_received_ = true;
    operation_heartbeat_lease_id_ = operation_lease_id;
    operation_last_heartbeat_ = now;
    if (!collision_exemption_was_active) {
      // The transient active-control value alone is not authorization to
      // remove a MoveIt collision object.  Publish the exemption only after a
      // live volatile heartbeat proves that the corresponding lease holder is
      // currently running.
      ++scene_revision_;
    }
  }

  void receive_operation_fault(const std::string & operation_lease_id)
  {
    std::string cleared_control;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!operation_fault_matches_active_lease(
          !active_control_id_.empty(), operation_heartbeat_lease_id_,
          operation_lease_id))
      {
        return;
      }
      cleared_control = active_control_id_;
      clear_active_operation_locked(true);
    }
    RCLCPP_ERROR(
      get_logger(),
      "Cabinet physics fault for lease '%s' revoked MoveIt collision "
      "exemption '%s'.",
      operation_lease_id.c_str(), cleared_control.c_str());
  }

  void enforce_operation_watchdog()
  {
    std::string expired_control;
    std::string expired_lease;
    double elapsed = 0.0;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (active_control_id_.empty()) {
        return;
      }
      const auto now = std::chrono::steady_clock::now();
      if (watchdog_active_control_generation_ !=
        active_control_generation_ ||
        watchdog_operation_lease_id_ != operation_heartbeat_lease_id_)
      {
        watchdog_active_control_generation_ = active_control_generation_;
        watchdog_operation_lease_id_ = operation_heartbeat_lease_id_;
        operation_monitor_started_at_ = now;
      }
      if (operation_monitor_started_at_ ==
        std::chrono::steady_clock::time_point{})
      {
        operation_monitor_started_at_ = now;
      }
      const auto reference_time = operation_heartbeat_received_ ?
        std::max(operation_monitor_started_at_, operation_last_heartbeat_) :
        operation_monitor_started_at_;
      elapsed = std::chrono::duration<double>(
        now - reference_time).count();
      if (!operation_watchdog_has_expired(
          true, false, elapsed, operation_watchdog_timeout_))
      {
        return;
      }
      expired_control = active_control_id_;
      expired_lease = operation_heartbeat_lease_id_;
      clear_active_operation_locked(true);
    }
    RCLCPP_ERROR(
      get_logger(),
      "Cabinet planning-scene heartbeat for control '%s' lease '%s' timed "
      "out after %.3f s; its MoveIt collision exemption was revoked.",
      expired_control.c_str(),
      expired_lease.empty() ? "<none>" : expired_lease.c_str(), elapsed);
  }

  bool has_door() const
  {
    return !door_control_id_.empty();
  }

  bool has_switch() const
  {
    return !switch_control_id_.empty();
  }

  std::string collision_object_id(const std::string & legacy_id) const
  {
    if (collision_object_prefix_.empty()) {
      return legacy_id;
    }
    return collision_object_prefix_ + "__" + legacy_id;
  }

  std::string required_string_parameter(
    const std::string & name,
    const std::string & default_value)
  {
    const auto value = declare_parameter<std::string>(name, default_value);
    if (value.empty()) {
      throw std::invalid_argument("Parameter '" + name + "' must not be empty.");
    }
    return value;
  }

  tf2::Vector3 vector3_parameter(
    const std::string & name,
    const std::vector<double> & default_value,
    bool require_positive = false)
  {
    const auto values = declare_parameter<std::vector<double>>(
      name, default_value);
    if (values.size() != 3U ||
      !std::all_of(
        values.begin(), values.end(),
        [require_positive](double value) {
          return std::isfinite(value) && (!require_positive || value > 0.0);
        }))
    {
      throw std::invalid_argument(
              "Parameter '" + name + "' must contain three " +
              (require_positive ? "positive " : "finite ") + "values.");
    }
    return tf2::Vector3(values[0], values[1], values[2]);
  }

  std::pair<double, double> cylinder_size_parameter(
    const std::string & name,
    const std::vector<double> & default_value)
  {
    const auto values = declare_parameter<std::vector<double>>(
      name, default_value);
    if (values.size() != 2U ||
      !std::all_of(
        values.begin(), values.end(),
        [](double value) {return std::isfinite(value) && value > 0.0;}))
    {
      throw std::invalid_argument(
              "Parameter '" + name +
              "' must contain a positive height and radius.");
    }
    return {values[0], values[1]};
  }

  void load_control_catalog()
  {
    const auto control_ids =
      declare_parameter<std::vector<std::string>>(
      "control_ids", std::vector<std::string>{});
    if (control_ids.empty()) {
      throw std::invalid_argument(
              "Parameter 'control_ids' must contain the shared catalog.");
    }
    control_profiles_.reserve(control_ids.size());
    for (const auto & id : control_ids) {
      if (id.empty()) {
        throw std::invalid_argument("Control IDs must not be empty.");
      }
      const auto prefix = "controls." + id + ".";
      const auto type = required_string_parameter(prefix + "type", "");
      const auto parent = type == "switch" ?
        declare_parameter<std::string>(prefix + "parent_control_id", "") :
        std::string{};
      control_profiles_.push_back({id, type, parent});
    }

    const auto articulation = resolve_scene_articulation(control_profiles_);
    door_control_id_ = articulation.door_control_id.value_or("");
    switch_control_id_ = articulation.switch_control_id.value_or("");
    switch_parent_control_id_ = articulation.switch_parent_control_id;
  }

  bool has_control_type(const std::string & type) const
  {
    return std::any_of(
      control_profiles_.begin(), control_profiles_.end(),
      [&type](const SceneControlProfile & control) {
        return control.type == type;
      });
  }

  void load_scene_profile()
  {
    const auto frame_part_ids =
      declare_parameter<std::vector<std::string>>(
      "frame_part_ids", std::vector<std::string>{});
    if (frame_part_ids.empty()) {
      throw std::invalid_argument(
              "Parameter 'frame_part_ids' must contain the cabinet shell.");
    }
    for (const auto & id : frame_part_ids) {
      if (id.empty()) {
        throw std::invalid_argument("Cabinet frame part IDs must not be empty.");
      }
      const auto prefix = "frame_parts." + id + ".";
      frame_parts_.push_back(
        {
          vector3_parameter(prefix + "size", {}, true),
          vector3_parameter(prefix + "position", {})});
    }

    if (has_control_type("button")) {
      std::tie(button_collision_height_, button_collision_radius_) =
        cylinder_size_parameter("control_collision.button_size", {});
      button_collision_center_offset_ = declare_parameter<double>(
        "control_collision.button_center_offset", 0.0);
    }
    if (has_control_type("knob")) {
      std::tie(knob_collision_height_, knob_collision_radius_) =
        cylinder_size_parameter("control_collision.knob_size", {});
      knob_collision_center_offset_ = declare_parameter<double>(
        "control_collision.knob_center_offset", 0.0);
    }
    if (!std::isfinite(button_collision_center_offset_) ||
      !std::isfinite(knob_collision_center_offset_))
    {
      throw std::invalid_argument(
              "Control collision center offsets must be finite.");
    }

    if (has_door()) {
      const auto configured_door = declare_parameter<std::string>(
        "door.control_id", door_control_id_);
      if (configured_door != door_control_id_) {
        throw std::invalid_argument(
                "Scene door differs from the shared control catalog.");
      }
      door_hinge_position_ = vector3_parameter("door.hinge_position", {});
      door_axis_ = vector3_parameter("door.axis", {});
      door_panel_size_ = vector3_parameter("door.panel_size", {}, true);
      door_panel_position_ = vector3_parameter("door.panel_position", {});
      if (door_axis_.length2() < 1.0e-12) {
        throw std::invalid_argument("Parameter 'door.axis' must be non-zero.");
      }
      door_axis_.normalize();
      const auto handle_part_ids = declare_parameter<std::vector<std::string>>(
        "door.handle_part_ids", std::vector<std::string>{});
      if (handle_part_ids.empty()) {
        throw std::invalid_argument(
                "Parameter 'door.handle_part_ids' must not be empty.");
      }
      for (const auto & id : handle_part_ids) {
        if (id.empty()) {
          throw std::invalid_argument("Door handle part IDs must not be empty.");
        }
        const auto prefix = "door.handle_parts." + id + ".";
        door_handle_parts_.push_back(
          {
            vector3_parameter(prefix + "size", {}, true),
            vector3_parameter(prefix + "position", {})});
      }
    }

    if (has_switch()) {
      const auto configured_switch = declare_parameter<std::string>(
        "switch.control_id", switch_control_id_);
      if (configured_switch != switch_control_id_) {
        throw std::invalid_argument(
                "Scene switch differs from the shared control catalog.");
      }
      const auto configured_parent = declare_parameter<std::string>(
        "switch.parent_control_id", switch_parent_control_id_);
      if (configured_parent != switch_parent_control_id_) {
        throw std::invalid_argument(
                "Switch parent differs between scene and control profiles.");
      }
      switch_pivot_position_ = vector3_parameter(
        "switch.pivot_position", {});
      switch_axis_ = vector3_parameter("switch.axis", {});
      switch_size_ = vector3_parameter("switch.size", {}, true);
      switch_center_offset_ = vector3_parameter("switch.center_offset", {});
      if (switch_axis_.length2() < 1.0e-12) {
        throw std::invalid_argument(
                "Parameter 'switch.axis' must be non-zero.");
      }
      switch_axis_.normalize();
    }
  }

  void load_control_collisions()
  {
    const auto button_axis = vector3_parameter(
      "button_defaults.axis", {0.0, 0.0, -1.0});
    const auto knob_axis = vector3_parameter(
      "knob_defaults.axis", {0.0, 0.0, 1.0});
    for (const auto & profile : control_profiles_) {
      const auto prefix = "controls." + profile.id + ".";
      if (profile.type == "door" || profile.type == "switch") {
        continue;
      }
      const bool rotary = profile.type == "knob";
      auto axis = vector3_parameter(
        prefix + "axis", rotary ?
        std::vector<double>{knob_axis.x(), knob_axis.y(), knob_axis.z()} :
        std::vector<double>{button_axis.x(), button_axis.y(), button_axis.z()});
      if (axis.length2() < 1.0e-12) {
        throw std::invalid_argument(
                "Control '" + profile.id + "' has a zero collision axis.");
      }
      axis.normalize();
      const auto reference = vector3_parameter(
        prefix + (rotary ? "pivot_position" : "local_position"), {});
      const double offset = rotary ?
        knob_collision_center_offset_ : button_collision_center_offset_;
      control_collisions_.push_back(
        {profile.id, reference + axis * offset, axis, rotary});
    }
  }

  rclcpp::Subscription<CabinetControlState>::SharedPtr
  create_control_state_subscription(
    const std::string & control_id,
    std::function<void(double)> update)
  {
    return create_subscription<CabinetControlState>(
      control_id + "/state",
      rclcpp::QoS(1).reliable().transient_local(),
      [update = std::move(update)](
        const CabinetControlState::SharedPtr message) {
        if (message->valid && std::isfinite(message->position)) {
          update(message->position);
        }
      });
  }

  bool lookup_model_transform(tf2::Transform & model)
  {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (require_pose_valid_ && (!pose_valid_received_ || !pose_valid_)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Waiting for a valid cabinet pose from the scene pose authority.");
        return false;
      }
    }
    try {
      const auto transform = transform_buffer_->lookupTransform(
        frame_id_, cabinet_frame_, tf2::TimePointZero);
      tf2::fromMsg(transform.transform, model);
      model.getRotation().normalize();
      return true;
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Waiting for %s -> %s cabinet transform: %s",
        frame_id_.c_str(), cabinet_frame_.c_str(), error.what());
      return false;
    }
  }

  moveit_msgs::msg::CollisionObject make_frame(
    const tf2::Transform & model)
  {
    moveit_msgs::msg::CollisionObject frame;
    frame.header.frame_id = frame_id_;
    frame.header.stamp = now();
    frame.id = collision_object_id("control_cabinet_frame");
    frame.operation = moveit_msgs::msg::CollisionObject::ADD;
    for (const auto & part : frame_parts_) {
      frame.primitives.push_back(box(part.size));
      frame.primitive_poses.push_back(
        to_pose(
          model * tf2::Transform(
            tf2::Quaternion::getIdentity(), part.position)));
    }
    return frame;
  }

  tf2::Transform door_transform(
    const tf2::Transform & model,
    double door_position) const
  {
    tf2::Quaternion hinge_rotation;
    hinge_rotation.setRotation(door_axis_, door_position);
    return model * tf2::Transform(hinge_rotation, door_hinge_position_);
  }

  moveit_msgs::msg::CollisionObject make_door_panel(
    const tf2::Transform & model,
    double door_position)
  {
    moveit_msgs::msg::CollisionObject door;
    door.header.frame_id = frame_id_;
    door.header.stamp = now();
    door.id = collision_object_id("cabinet_control_" + door_control_id_);
    door.operation = moveit_msgs::msg::CollisionObject::ADD;
    door.primitives.push_back(box(door_panel_size_));
    door.primitive_poses.push_back(
      to_pose(
        door_transform(model, door_position) * tf2::Transform(
          tf2::Quaternion::getIdentity(), door_panel_position_)));
    return door;
  }

  moveit_msgs::msg::CollisionObject make_door_handle(
    const tf2::Transform & model,
    double door_position,
    bool remove)
  {
    moveit_msgs::msg::CollisionObject handle;
    handle.header.frame_id = frame_id_;
    handle.header.stamp = now();
    handle.id = collision_object_id(
      "cabinet_control_" + door_control_id_ + "_handle");
    if (remove) {
      handle.operation = moveit_msgs::msg::CollisionObject::REMOVE;
      return handle;
    }
    handle.operation = moveit_msgs::msg::CollisionObject::ADD;
    const auto transform = door_transform(model, door_position);
    for (const auto & part : door_handle_parts_) {
      handle.primitives.push_back(box(part.size));
      handle.primitive_poses.push_back(
        to_pose(
          transform * tf2::Transform(
            tf2::Quaternion::getIdentity(), part.position)));
    }
    return handle;
  }

  moveit_msgs::msg::CollisionObject make_control(
    const tf2::Transform & model,
    const ControlCollision & control,
    bool remove)
  {
    moveit_msgs::msg::CollisionObject object;
    object.header.frame_id = frame_id_;
    object.header.stamp = now();
    object.id = collision_object_id("cabinet_control_" + control.id);
    if (remove) {
      object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
      return object;
    }
    object.operation = moveit_msgs::msg::CollisionObject::ADD;
    object.primitives.push_back(
      control.rotary ?
      cylinder(knob_collision_height_, knob_collision_radius_) :
      cylinder(button_collision_height_, button_collision_radius_));
    object.primitive_poses.push_back(
      to_pose(
        model * tf2::Transform(
          cylinder_rotation(control.axis), control.center)));
    return object;
  }

  moveit_msgs::msg::CollisionObject make_switch(
    const tf2::Transform & model,
    double door_position,
    double switch_position,
    bool remove)
  {
    moveit_msgs::msg::CollisionObject object;
    object.header.frame_id = frame_id_;
    object.header.stamp = now();
    object.id = collision_object_id(
      "cabinet_control_" + switch_control_id_);
    if (remove) {
      object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
      return object;
    }
    object.operation = moveit_msgs::msg::CollisionObject::ADD;
    object.primitives.push_back(box(switch_size_));
    tf2::Quaternion switch_rotation;
    switch_rotation.setRotation(switch_axis_, switch_position);
    const auto parent_transform = switch_parent_control_id_.empty() ?
      model : door_transform(model, door_position);
    object.primitive_poses.push_back(
      to_pose(
        parent_transform *
        tf2::Transform(
          tf2::Quaternion::getIdentity(), switch_pivot_position_) *
        tf2::Transform(switch_rotation, tf2::Vector3(0.0, 0.0, 0.0)) *
        tf2::Transform(
          tf2::Quaternion::getIdentity(), switch_center_offset_)));
    return object;
  }

  void publish_scene_if_needed()
  {
    // Wall-clock liveness is independent of TF and simulation time.  Revoke a
    // stale collision exemption before attempting any potentially unavailable
    // pose lookup, then republish from the last latched cabinet transform.
    enforce_operation_watchdog();
    tf2::Transform observed_model;
    const bool observed_model_available =
      lookup_model_transform(observed_model);

    std::string active_control;
    std::string published_active_control;
    tf2::Transform model;
    double door_position = 0.0;
    double switch_position = 0.0;
    std::uint64_t revision = 0U;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      // Collision objects and task geometry must use one latched cabinet pose
      // for the whole live manipulation.  A transient active-control sample
      // without a volatile heartbeat is stale after a process restart and
      // must neither freeze TF nor remove a collision object.
      if (observed_model_available &&
        (!model_transform_received_ ||
        ((active_control_id_.empty() || !operation_heartbeat_received_) &&
        transform_changed(model_transform_, observed_model))))
      {
        model_transform_ = observed_model;
        model_transform_received_ = true;
        ++scene_revision_;
      }
      if (!model_transform_received_) {
        return;
      }
      if (scene_published_ && published_revision_ == scene_revision_) {
        return;
      }
      active_control = operation_heartbeat_received_ ?
        active_control_id_ : std::string{};
      published_active_control = published_active_control_id_;
      model = model_transform_;
      door_position = door_position_;
      switch_position = switch_position_;
      revision = scene_revision_;
    }

    std::vector<moveit_msgs::msg::CollisionObject> objects;
    objects.reserve(
      control_collisions_.size() + (has_door() ? 2U : 0U) +
      (has_switch() ? 1U : 0U) + 1U);
    objects.push_back(make_frame(model));
    for (const auto & control : control_collisions_) {
      if (active_control == control.id) {
        if (published_active_control != active_control) {
          objects.push_back(make_control(model, control, true));
        }
        continue;
      }
      objects.push_back(make_control(model, control, false));
    }
    if (has_switch()) {
      if (active_control == switch_control_id_) {
        if (published_active_control != active_control) {
          objects.push_back(
            make_switch(
              model, door_position, switch_position, true));
        }
      } else {
        objects.push_back(
          make_switch(
            model, door_position, switch_position, false));
      }
    }
    if (has_door()) {
      objects.push_back(make_door_panel(model, door_position));
      if (active_control == door_control_id_) {
        if (published_active_control != active_control) {
          objects.push_back(make_door_handle(model, door_position, true));
        }
      } else {
        objects.push_back(make_door_handle(model, door_position, false));
      }
    }

    if (planning_scene_publisher_->get_subscription_count() == 0U) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Waiting for the MoveIt collision-object subscriber.");
      return;
    }
    moveit_msgs::msg::PlanningScene scene;
    scene.is_diff = true;
    scene.robot_state.is_diff = true;
    scene.world.collision_objects = std::move(objects);
    planning_scene_publisher_->publish(scene);
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      scene_published_ = true;
      published_active_control_id_ = active_control;
      if (scene_revision_ == revision) {
        published_revision_ = revision;
      }
    }
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Cabinet scene profile (%zu controls) is synchronized from TF %s -> "
      "%s (active target: %s).",
      control_collisions_.size() + (has_door() ? 1U : 0U) +
      (has_switch() ? 1U : 0U), frame_id_.c_str(),
      cabinet_frame_.c_str(),
      active_control.empty() ? "none" : active_control.c_str());
  }

  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr
    planning_scene_publisher_;
  rclcpp::TimerBase::SharedPtr retry_timer_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
    active_control_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
    operation_heartbeat_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
    operation_fault_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
    pose_valid_subscription_;
  rclcpp::Subscription<CabinetControlState>::SharedPtr
    door_state_subscription_;
  rclcpp::Subscription<CabinetControlState>::SharedPtr
    switch_state_subscription_;
  std::unique_ptr<tf2_ros::Buffer> transform_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> transform_listener_;

  mutable std::mutex state_mutex_;
  std::string active_control_id_;
  std::string published_active_control_id_;
  std::uint64_t active_control_generation_{0U};
  std::uint64_t watchdog_active_control_generation_{0U};
  bool operation_heartbeat_received_{false};
  std::string operation_heartbeat_lease_id_;
  std::string watchdog_operation_lease_id_;
  std::chrono::steady_clock::time_point operation_last_heartbeat_{};
  std::chrono::steady_clock::time_point operation_monitor_started_at_{};
  std::unordered_set<std::string> expired_operation_lease_ids_;
  std::deque<std::string> expired_operation_lease_history_;
  tf2::Transform model_transform_{tf2::Transform::getIdentity()};
  double door_position_{0.0};
  double switch_position_{0.0};
  bool pose_valid_{false};
  bool pose_valid_received_{false};
  bool model_transform_received_{false};
  bool scene_published_{false};
  std::uint64_t scene_revision_{1U};
  std::uint64_t published_revision_{0U};

  std::string frame_id_;
  std::string cabinet_frame_;
  std::string collision_object_prefix_;
  bool require_pose_valid_{false};
  double operation_watchdog_timeout_{2.0};
  std::vector<BoxPart> frame_parts_;
  std::vector<SceneControlProfile> control_profiles_;
  std::vector<ControlCollision> control_collisions_;
  double button_collision_height_{0.0};
  double button_collision_radius_{0.0};
  double button_collision_center_offset_{0.0};
  double knob_collision_height_{0.0};
  double knob_collision_radius_{0.0};
  double knob_collision_center_offset_{0.0};
  std::string door_control_id_;
  tf2::Vector3 door_hinge_position_;
  tf2::Vector3 door_axis_;
  tf2::Vector3 door_panel_size_;
  tf2::Vector3 door_panel_position_;
  std::vector<BoxPart> door_handle_parts_;
  std::string switch_control_id_;
  std::string switch_parent_control_id_;
  tf2::Vector3 switch_pivot_position_;
  tf2::Vector3 switch_axis_;
  tf2::Vector3 switch_size_;
  tf2::Vector3 switch_center_offset_;
};

}  // namespace xczs_inspection_robot_control

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<
      xczs_inspection_robot_control::CabinetPlanningScene>());
  rclcpp::shutdown();
  return 0;
}
