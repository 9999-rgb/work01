// Copyright 2026
// SPDX-License-Identifier: BSD-3-Clause

#include <chrono>
#include <memory>
#include <string>
#include <utility>

#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit_msgs/msg/collision_object.hpp"
#include "rclcpp/rclcpp.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"

namespace xczs_inspection_robot_control
{

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

    collision_origin_x_ = declare_parameter<double>(
      "collision_origin_x", 0.331);
    collision_origin_y_ = declare_parameter<double>(
      "collision_origin_y", 1.115);
    collision_origin_z_ = declare_parameter<double>(
      "collision_origin_z", -0.304);
    collision_size_x_ = declare_parameter<double>(
      "collision_size_x", 0.662);
    collision_size_y_ = declare_parameter<double>(
      "collision_size_y", 2.230);
    collision_size_z_ = declare_parameter<double>(
      "collision_size_z", 0.608);

    retry_timer_ = create_wall_timer(
      std::chrono::seconds(1),
      [this]() {
        publish_cabinet();
      });
  }

private:
  void publish_cabinet()
  {
    moveit_msgs::msg::CollisionObject cabinet;
    cabinet.header.frame_id = frame_id_;
    cabinet.header.stamp = now();
    cabinet.id = "control_cabinet";
    cabinet.operation = moveit_msgs::msg::CollisionObject::ADD;

    shape_msgs::msg::SolidPrimitive enclosure;
    enclosure.type = shape_msgs::msg::SolidPrimitive::BOX;
    enclosure.dimensions.resize(3);
    enclosure.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] =
      collision_size_x_;
    enclosure.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] =
      collision_size_y_;
    enclosure.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] =
      collision_size_z_;

    tf2::Quaternion model_rotation;
    model_rotation.setRPY(cabinet_roll_, cabinet_pitch_, cabinet_yaw_);
    const tf2::Transform model_transform(
      model_rotation,
      tf2::Vector3(cabinet_x_, cabinet_y_, cabinet_z_));
    const tf2::Transform local_collision_transform(
      tf2::Quaternion::getIdentity(),
      tf2::Vector3(
        collision_origin_x_,
        collision_origin_y_,
        collision_origin_z_));
    const tf2::Transform collision_transform =
      model_transform * local_collision_transform;

    geometry_msgs::msg::Pose collision_pose;
    collision_pose.position.x = collision_transform.getOrigin().x();
    collision_pose.position.y = collision_transform.getOrigin().y();
    collision_pose.position.z = collision_transform.getOrigin().z();
    collision_pose.orientation.x = collision_transform.getRotation().x();
    collision_pose.orientation.y = collision_transform.getRotation().y();
    collision_pose.orientation.z = collision_transform.getRotation().z();
    collision_pose.orientation.w = collision_transform.getRotation().w();

    cabinet.primitives.push_back(std::move(enclosure));
    cabinet.primitive_poses.push_back(std::move(collision_pose));
    if (!planning_scene_interface_.applyCollisionObject(cabinet)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Waiting for MoveIt planning scene service.");
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "Added control_cabinet collision enclosure to the %s planning frame.",
      frame_id_.c_str());
    retry_timer_->cancel();
  }

  moveit::planning_interface::PlanningSceneInterface
    planning_scene_interface_;
  rclcpp::TimerBase::SharedPtr retry_timer_;

  std::string frame_id_;
  double cabinet_x_{2.0};
  double cabinet_y_{0.33};
  double cabinet_z_{0.0};
  double cabinet_roll_{1.57079632679};
  double cabinet_pitch_{0.0};
  double cabinet_yaw_{-1.57079632679};
  double collision_origin_x_{0.331};
  double collision_origin_y_{1.115};
  double collision_origin_z_{-0.304};
  double collision_size_x_{0.662};
  double collision_size_y_{2.230};
  double collision_size_z_{0.608};
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
