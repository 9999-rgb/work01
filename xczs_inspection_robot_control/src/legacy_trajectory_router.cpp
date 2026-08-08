// Copyright 2026
// SPDX-License-Identifier: BSD-3-Clause

#include <algorithm>
#include <array>
#include <cstddef>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "action_msgs/msg/goal_status.hpp"
#include "action_msgs/msg/goal_status_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace xczs_inspection_robot_control
{

namespace
{

using JointTrajectory = trajectory_msgs::msg::JointTrajectory;
using JointTrajectoryPoint = trajectory_msgs::msg::JointTrajectoryPoint;
using GoalStatus = action_msgs::msg::GoalStatus;
using GoalStatusArray = action_msgs::msg::GoalStatusArray;

const std::array<std::string, 6> kArmJoints = {
  "body_arm1",
  "arm1_arm2",
  "arm2_arm3",
  "arm3_arm4",
  "arm4_arm5",
  "arm5_end",
};

const std::array<std::string, 2> kGripperJoints = {
  "end_worklink1",
  "end_worklink2",
};

template<typename ContainerT>
std::optional<std::vector<std::size_t>> find_joint_indices(
  const std::vector<std::string> & input_joints,
  const ContainerT & requested_joints)
{
  std::vector<std::size_t> indices;
  indices.reserve(requested_joints.size());
  for (const auto & requested_joint : requested_joints) {
    const auto iterator = std::find(
      input_joints.begin(), input_joints.end(), requested_joint);
    if (iterator == input_joints.end()) {
      return std::nullopt;
    }
    indices.push_back(
      static_cast<std::size_t>(
        std::distance(input_joints.begin(), iterator)));
  }
  return indices;
}

std::vector<double> select_values(
  const std::vector<double> & input,
  const std::vector<std::size_t> & indices,
  std::size_t input_joint_count)
{
  if (input.empty()) {
    return {};
  }
  if (input.size() != input_joint_count) {
    throw std::invalid_argument(
            "trajectory point field size does not match joint_names");
  }

  std::vector<double> output;
  output.reserve(indices.size());
  for (const auto index : indices) {
    output.push_back(input[index]);
  }
  return output;
}

template<typename ContainerT>
std::optional<JointTrajectory> extract_trajectory(
  const JointTrajectory & input,
  const ContainerT & requested_joints)
{
  const auto indices = find_joint_indices(input.joint_names, requested_joints);
  if (!indices) {
    return std::nullopt;
  }

  JointTrajectory output;
  output.header = input.header;
  output.joint_names.assign(requested_joints.begin(), requested_joints.end());
  output.points.reserve(input.points.size());

  for (const auto & input_point : input.points) {
    JointTrajectoryPoint output_point;
    output_point.positions = select_values(
      input_point.positions, *indices, input.joint_names.size());
    output_point.velocities = select_values(
      input_point.velocities, *indices, input.joint_names.size());
    output_point.accelerations = select_values(
      input_point.accelerations, *indices, input.joint_names.size());
    output_point.effort = select_values(
      input_point.effort, *indices, input.joint_names.size());
    output_point.time_from_start = input_point.time_from_start;
    output.points.push_back(std::move(output_point));
  }
  return output;
}

bool has_active_goal(const GoalStatusArray & statuses)
{
  return std::any_of(
    statuses.status_list.begin(),
    statuses.status_list.end(),
    [](const GoalStatus & status) {
      return status.status == GoalStatus::STATUS_ACCEPTED ||
      status.status == GoalStatus::STATUS_EXECUTING ||
      status.status == GoalStatus::STATUS_CANCELING;
    });
}

}  // namespace

class LegacyTrajectoryRouter final : public rclcpp::Node
{
public:
  LegacyTrajectoryRouter()
  : Node("xczs_legacy_trajectory_router")
  {
    const auto input_topic = declare_parameter<std::string>(
      "joint_trajectory_topic", "/xczs/joint_trajectory");
    const auto arm_topic = declare_parameter<std::string>(
      "arm_controller_topic", "/xczs/arm_controller/joint_trajectory");
    const auto gripper_topic = declare_parameter<std::string>(
      "gripper_controller_topic",
      "/xczs/gripper_controller/joint_trajectory");
    const auto arm_status_topic = declare_parameter<std::string>(
      "arm_action_status_topic",
      "/xczs/arm_controller/follow_joint_trajectory/_action/status");
    const auto gripper_status_topic = declare_parameter<std::string>(
      "gripper_action_status_topic",
      "/xczs/gripper_controller/follow_joint_trajectory/_action/status");

    arm_publisher_ = create_publisher<JointTrajectory>(arm_topic, 10);
    gripper_publisher_ = create_publisher<JointTrajectory>(
      gripper_topic, 10);
    input_subscription_ = create_subscription<JointTrajectory>(
      input_topic,
      10,
      [this](const JointTrajectory::SharedPtr message) {
        route(*message);
      });
    arm_status_subscription_ = create_subscription<GoalStatusArray>(
      arm_status_topic,
      10,
      [this](const GoalStatusArray::SharedPtr message) {
        arm_action_active_ = has_active_goal(*message);
      });
    gripper_status_subscription_ = create_subscription<GoalStatusArray>(
      gripper_status_topic,
      10,
      [this](const GoalStatusArray::SharedPtr message) {
        gripper_action_active_ = has_active_goal(*message);
      });

    RCLCPP_INFO(
      get_logger(),
      "Routing legacy trajectories from %s to ros2_control.",
      input_topic.c_str());
  }

private:
  void route(const JointTrajectory & input)
  {
    if (input.points.empty()) {
      RCLCPP_WARN(get_logger(), "Ignoring an empty legacy trajectory.");
      return;
    }

    try {
      const auto arm_trajectory = extract_trajectory(input, kArmJoints);
      const auto gripper_trajectory = extract_trajectory(
        input, kGripperJoints);
      if (!arm_trajectory && !gripper_trajectory) {
        RCLCPP_WARN(
          get_logger(),
          "Legacy trajectory contains neither the complete arm group nor "
          "the complete gripper group.");
        return;
      }
      if (arm_trajectory && !arm_action_active_) {
        arm_publisher_->publish(*arm_trajectory);
      } else if (arm_trajectory) {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "Ignoring manual arm trajectory while MoveIt is executing.");
      }
      if (gripper_trajectory && !gripper_action_active_) {
        gripper_publisher_->publish(*gripper_trajectory);
      } else if (gripper_trajectory) {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "Ignoring manual gripper trajectory while MoveIt is executing.");
      }
    } catch (const std::invalid_argument & error) {
      RCLCPP_ERROR(
        get_logger(), "Rejected malformed legacy trajectory: %s", error.what());
    }
  }

  rclcpp::Publisher<JointTrajectory>::SharedPtr arm_publisher_;
  rclcpp::Publisher<JointTrajectory>::SharedPtr gripper_publisher_;
  rclcpp::Subscription<JointTrajectory>::SharedPtr input_subscription_;
  rclcpp::Subscription<GoalStatusArray>::SharedPtr arm_status_subscription_;
  rclcpp::Subscription<GoalStatusArray>::SharedPtr
    gripper_status_subscription_;
  bool arm_action_active_{false};
  bool gripper_action_active_{false};
};

}  // namespace xczs_inspection_robot_control

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<
      xczs_inspection_robot_control::LegacyTrajectoryRouter>());
  rclcpp::shutdown();
  return 0;
}
