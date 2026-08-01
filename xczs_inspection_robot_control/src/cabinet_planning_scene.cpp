// Copyright 2026
// SPDX-License-Identifier: BSD-3-Clause

#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit_msgs/msg/collision_object.hpp"
#include "rclcpp/rclcpp.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "xczs_inspection_robot_control/msg/cabinet_control_state.hpp"

namespace xczs_inspection_robot_control
{

namespace
{

using CabinetControlState =
  xczs_inspection_robot_control::msg::CabinetControlState;

constexpr char kDoorId[] = "cabinet_rear_door";
constexpr char kSwitchId[] = "cabinet_main_switch";

struct ControlCollision
{
  const char * id;
  double x;
  double y;
  double z;
  bool rotary;
};

// These are cabinet-body-local CAD pivots.  Keeping the collision registry in
// one table makes it straightforward to compare it with cabinet_controls.yaml.
constexpr std::array<ControlCollision, 31> kControlCollisions{{
  {"box_1_button_1", 0.120, 1.876, 0.011, false},
  {"box_1_button_2", 0.170, 1.876, 0.011, false},
  {"box_2_button_1", 0.449, 1.876, 0.011, false},
  {"box_2_button_2", 0.499, 1.876, 0.011, false},
  {"box_3_button_1", 0.120, 1.547, 0.011, false},
  {"box_3_button_2", 0.170, 1.547, 0.011, false},
  {"box_4_button_1", 0.449, 1.547, 0.011, false},
  {"box_4_button_2", 0.499, 1.547, 0.011, false},
  {"box_5_button_1", 0.120, 1.218, 0.011, false},
  {"box_5_button_2", 0.170, 1.218, 0.011, false},
  {"box_6_button_1", 0.449, 1.218, 0.011, false},
  {"box_6_button_2", 0.499, 1.218, 0.011, false},
  {"box_7_button_1", 0.120, 0.889, 0.011, false},
  {"box_7_button_2", 0.170, 0.889, 0.011, false},
  {"box_8_button_1", 0.449, 0.889, 0.011, false},
  {"box_8_button_2", 0.499, 0.889, 0.011, false},
  {"box_10_button_1", 0.492, 0.574, 0.011, false},
  {"box_10_button_2", 0.527, 0.574, 0.011, false},
  {"box_11_button_1", 0.492, 0.377, 0.011, false},
  {"box_11_button_2", 0.527, 0.377, 0.011, false},
  {"box_1_knob", 0.167470, 2.050940, 0.0295, true},
  {"box_2_knob", 0.496470, 2.050940, 0.0295, true},
  {"box_3_knob", 0.167470, 1.721940, 0.0295, true},
  {"box_4_knob", 0.496470, 1.721940, 0.0295, true},
  {"box_5_knob", 0.167470, 1.392940, 0.0295, true},
  {"box_6_knob", 0.496470, 1.392940, 0.0295, true},
  {"box_7_knob", 0.167470, 1.063940, 0.0295, true},
  {"box_8_knob", 0.496470, 1.063940, 0.0295, true},
  {"box_9_knob", 0.124940, 0.774526, 0.0295, true},
  {"box_10_knob", 0.135470, 0.630940, 0.0295, true},
  {"box_11_knob", 0.135470, 0.433940, 0.0295, true},
}};

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

shape_msgs::msg::SolidPrimitive box(double x, double y, double z)
{
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  primitive.dimensions = {x, y, z};
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

}  // namespace

class CabinetPlanningScene final : public rclcpp::Node
{
public:
  CabinetPlanningScene()
  : Node("xczs_cabinet_planning_scene"),
    planning_scene_interface_("", false)
  {
    frame_id_ = declare_parameter<std::string>("frame_id", "odom");
    cabinet_x_ = declare_parameter<double>("cabinet_x", 2.0);
    cabinet_y_ = declare_parameter<double>("cabinet_y", 0.33);
    cabinet_z_ = declare_parameter<double>("cabinet_z", 0.0);
    cabinet_roll_ = declare_parameter<double>(
      "cabinet_roll", 1.57079632679);
    cabinet_pitch_ = declare_parameter<double>("cabinet_pitch", 0.0);
    cabinet_yaw_ = declare_parameter<double>(
      "cabinet_yaw", -1.57079632679);

    active_control_subscription_ = create_subscription<std_msgs::msg::String>(
      "/xczs/cabinet/active_control",
      rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::String::SharedPtr message) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        active_control_id_ = message->data;
        scene_dirty_ = true;
      });
    door_state_subscription_ = create_subscription<CabinetControlState>(
      "/xczs/cabinet/cabinet_rear_door/state",
      rclcpp::QoS(1).reliable().transient_local(),
      [this](const CabinetControlState::SharedPtr message) {
        if (!message->valid || !std::isfinite(message->position)) {
          return;
        }
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (std::abs(door_position_ - message->position) > 1.0e-4) {
          door_position_ = message->position;
          scene_dirty_ = true;
        }
      });
    switch_state_subscription_ = create_subscription<CabinetControlState>(
      "/xczs/cabinet/cabinet_main_switch/state",
      rclcpp::QoS(1).reliable().transient_local(),
      [this](const CabinetControlState::SharedPtr message) {
        if (!message->valid || !std::isfinite(message->position)) {
          return;
        }
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (std::abs(switch_position_ - message->position) > 1.0e-4) {
          switch_position_ = message->position;
          scene_dirty_ = true;
        }
      });

    retry_timer_ = create_wall_timer(
      std::chrono::milliseconds(250),
      [this]() {publish_scene_if_needed();});
  }

private:
  tf2::Transform model_transform() const
  {
    tf2::Quaternion rotation;
    rotation.setRPY(cabinet_roll_, cabinet_pitch_, cabinet_yaw_);
    return tf2::Transform(
      rotation, tf2::Vector3(cabinet_x_, cabinet_y_, cabinet_z_));
  }

  moveit_msgs::msg::CollisionObject make_frame() const
  {
    moveit_msgs::msg::CollisionObject frame;
    frame.header.frame_id = frame_id_;
    frame.header.stamp = now();
    frame.id = "control_cabinet_frame";
    frame.operation = moveit_msgs::msg::CollisionObject::ADD;

    const auto model = model_transform();
    const auto add_panel =
      [&frame, &model](
      const shape_msgs::msg::SolidPrimitive & primitive,
      double x, double y, double z)
      {
        frame.primitives.push_back(primitive);
        frame.primitive_poses.push_back(to_pose(
          model * tf2::Transform(
            tf2::Quaternion::getIdentity(), tf2::Vector3(x, y, z))));
      };

    // The front panel remains closed behind the controls.  The rear panel is
    // represented by the articulated door below, so opening it exposes a
    // genuinely empty cabinet volume.
    add_panel(box(0.642, 2.210, 0.010), 0.331, 1.115, -0.025);
    add_panel(box(0.010, 2.230, 0.600), 0.005, 1.115, -0.300);
    add_panel(box(0.010, 2.230, 0.600), 0.657, 1.115, -0.300);
    add_panel(box(0.642, 0.010, 0.600), 0.331, 0.005, -0.300);
    add_panel(box(0.642, 0.010, 0.600), 0.331, 2.225, -0.300);
    return frame;
  }

  moveit_msgs::msg::CollisionObject make_door(
    double door_position,
    bool remove) const
  {
    moveit_msgs::msg::CollisionObject door;
    door.header.frame_id = frame_id_;
    door.header.stamp = now();
    door.id = "cabinet_control_" + std::string(kDoorId);
    if (remove) {
      door.operation = moveit_msgs::msg::CollisionObject::REMOVE;
      return door;
    }
    door.operation = moveit_msgs::msg::CollisionObject::ADD;
    door.primitives.push_back(box(0.642, 2.210, 0.010));
    door.primitives.push_back(box(0.025, 0.025, 0.070));
    door.primitives.push_back(box(0.025, 0.025, 0.070));
    door.primitives.push_back(box(0.025, 0.165, 0.025));

    tf2::Quaternion hinge_rotation;
    hinge_rotation.setRotation(tf2::Vector3(0.0, 1.0, 0.0), door_position);
    const tf2::Transform hinge(
      hinge_rotation, tf2::Vector3(0.010, 0.010, -0.600));
    const tf2::Transform panel_center(
      tf2::Quaternion::getIdentity(), tf2::Vector3(0.321, 1.105, -0.005));
    const auto door_transform = model_transform() * hinge;
    door.primitive_poses.push_back(to_pose(door_transform * panel_center));
    door.primitive_poses.push_back(to_pose(
      door_transform * tf2::Transform(
        tf2::Quaternion::getIdentity(),
        tf2::Vector3(0.600, 1.035, -0.045))));
    door.primitive_poses.push_back(to_pose(
      door_transform * tf2::Transform(
        tf2::Quaternion::getIdentity(),
        tf2::Vector3(0.600, 1.175, -0.045))));
    door.primitive_poses.push_back(to_pose(
      door_transform * tf2::Transform(
        tf2::Quaternion::getIdentity(),
        tf2::Vector3(0.600, 1.105, -0.080))));
    return door;
  }

  moveit_msgs::msg::CollisionObject make_control(
    const ControlCollision & control,
    bool remove) const
  {
    moveit_msgs::msg::CollisionObject object;
    object.header.frame_id = frame_id_;
    object.header.stamp = now();
    object.id = "cabinet_control_" + std::string(control.id);
    if (remove) {
      object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
      return object;
    }
    object.operation = moveit_msgs::msg::CollisionObject::ADD;
    object.primitives.push_back(
      control.rotary ? cylinder(0.046, 0.050) : cylinder(0.020, 0.022));
    const double collision_z = control.rotary ? control.z + 0.013 : control.z;
    object.primitive_poses.push_back(to_pose(
      model_transform() * tf2::Transform(
        tf2::Quaternion::getIdentity(),
        tf2::Vector3(control.x, control.y, collision_z))));
    return object;
  }

  moveit_msgs::msg::CollisionObject make_switch(
    double door_position,
    double switch_position,
    bool remove) const
  {
    moveit_msgs::msg::CollisionObject object;
    object.header.frame_id = frame_id_;
    object.header.stamp = now();
    object.id = "cabinet_control_" + std::string(kSwitchId);
    if (remove) {
      object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
      return object;
    }
    object.operation = moveit_msgs::msg::CollisionObject::ADD;
    object.primitives.push_back(box(0.020, 0.080, 0.020));

    tf2::Quaternion hinge_rotation;
    hinge_rotation.setRotation(tf2::Vector3(0.0, 1.0, 0.0), door_position);
    const tf2::Transform hinge(
      hinge_rotation, tf2::Vector3(0.010, 0.010, -0.600));
    const tf2::Transform switch_pivot(
      tf2::Quaternion::getIdentity(), tf2::Vector3(0.342, 1.680, -0.018));
    tf2::Quaternion switch_rotation;
    switch_rotation.setRotation(
      tf2::Vector3(0.0, 0.0, 1.0), switch_position);
    const tf2::Transform switch_handle(
      switch_rotation, tf2::Vector3(0.0, 0.0, 0.0));
    const tf2::Transform switch_center(
      tf2::Quaternion::getIdentity(), tf2::Vector3(0.0, 0.0, 0.010));
    object.primitive_poses.push_back(to_pose(
      model_transform() * hinge * switch_pivot * switch_handle *
      switch_center));
    return object;
  }

  void publish_scene_if_needed()
  {
    std::string active_control;
    double door_position = 0.0;
    double switch_position = 0.0;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (scene_published_ && !scene_dirty_) {
        return;
      }
      active_control = active_control_id_;
      door_position = door_position_;
      switch_position = switch_position_;
    }

    std::vector<moveit_msgs::msg::CollisionObject> objects;
    objects.reserve(kControlCollisions.size() + 3U);
    objects.push_back(make_frame());
    for (const auto & control : kControlCollisions) {
      objects.push_back(make_control(
        control, active_control == control.id));
    }
    objects.push_back(make_switch(
      door_position, switch_position, active_control == kSwitchId));
    objects.push_back(make_door(
      door_position, active_control == kDoorId));

    if (!planning_scene_interface_.applyCollisionObjects(objects)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Waiting for the MoveIt planning-scene service.");
      return;
    }
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      scene_published_ = true;
      scene_dirty_ = false;
    }
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Cabinet frame, 31 front controls, rear switch and dynamic door "
      "are synchronized in %s (active target: %s).",
      frame_id_.c_str(), active_control.empty() ? "none" : active_control.c_str());
  }

  moveit::planning_interface::PlanningSceneInterface
    planning_scene_interface_;
  rclcpp::TimerBase::SharedPtr retry_timer_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
    active_control_subscription_;
  rclcpp::Subscription<CabinetControlState>::SharedPtr
    door_state_subscription_;
  rclcpp::Subscription<CabinetControlState>::SharedPtr
    switch_state_subscription_;

  mutable std::mutex state_mutex_;
  std::string active_control_id_;
  double door_position_{0.0};
  double switch_position_{0.0};
  bool scene_dirty_{true};
  bool scene_published_{false};

  std::string frame_id_;
  double cabinet_x_{2.0};
  double cabinet_y_{0.33};
  double cabinet_z_{0.0};
  double cabinet_roll_{1.57079632679};
  double cabinet_pitch_{0.0};
  double cabinet_yaw_{-1.57079632679};
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
