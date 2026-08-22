// Copyright 2026
// SPDX-License-Identifier: BSD-3-Clause

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "action_msgs/msg/goal_status.hpp"
#include "action_msgs/msg/goal_status_array.hpp"
#include "rclcpp/expand_topic_or_service_name.hpp"
#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "xczs_inspection_robot_control/router_utils.hpp"

namespace xczs_inspection_robot_control
{

namespace
{

using JointTrajectory = trajectory_msgs::msg::JointTrajectory;
using JointTrajectoryPoint = trajectory_msgs::msg::JointTrajectoryPoint;
using GoalStatus = action_msgs::msg::GoalStatus;
using GoalStatusArray = action_msgs::msg::GoalStatusArray;

constexpr const char * kDefaultControllerNamespace = "/xczs";

void validate_trajectory_contract(const JointTrajectory & trajectory)
{
  if (!has_unique_nonempty_names(trajectory.joint_names)) {
    throw std::invalid_argument(
            "joint_names must contain unique, nonempty names");
  }

  bool has_previous_time = false;
  std::int32_t previous_seconds = 0;
  std::uint32_t previous_nanoseconds = 0U;
  for (const auto & point : trajectory.points) {
    const auto joint_count = trajectory.joint_names.size();
    if (!trajectory_numeric_field_is_valid(
        point.positions, joint_count, true))
    {
      throw std::invalid_argument(
              "every trajectory point must contain one finite position "
              "per joint");
    }
    if (!trajectory_numeric_field_is_valid(
        point.velocities, joint_count, false) ||
      !trajectory_numeric_field_is_valid(
        point.accelerations, joint_count, false) ||
      !trajectory_numeric_field_is_valid(
        point.effort, joint_count, false))
    {
      throw std::invalid_argument(
              "optional trajectory fields must be empty or contain one "
              "finite value per joint");
    }

    const auto seconds = point.time_from_start.sec;
    const auto nanoseconds = point.time_from_start.nanosec;
    if (!trajectory_duration_is_valid(seconds, nanoseconds)) {
      throw std::invalid_argument(
              "time_from_start must be a valid nonnegative duration");
    }
    if (has_previous_time &&
      !trajectory_duration_is_strictly_after(
        seconds, nanoseconds, previous_seconds, previous_nanoseconds))
    {
      throw std::invalid_argument(
              "time_from_start must increase strictly between points");
    }
    has_previous_time = true;
    previous_seconds = seconds;
    previous_nanoseconds = nanoseconds;
  }
}

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
    const auto input_topic = absolute_ros_name_parameter(
      "joint_trajectory_topic", "/xczs/joint_trajectory");

    // Prefer the multi-group schema (manual_joint_group_names + the flattened
    // manual_joint_groups.<name> parameters) when present, and fall back to
    // the legacy two-group arm/gripper schema otherwise.  Topics are derived
    // from controller_namespace + group name by the shared ros2_control
    // convention rather than repeated per group.
    auto group_names = declare_parameter<std::vector<std::string>>(
      "manual_joint_group_names", std::vector<std::string>{});
    if (!group_names.empty()) {
      const auto namespace_name = absolute_ros_name_parameter(
        "controller_namespace", kDefaultControllerNamespace);
      for (const auto & group_name : group_names) {
        const auto joints = declare_parameter<std::vector<std::string>>(
          "manual_joint_groups." + group_name, std::vector<std::string>{});
        if (!has_unique_nonempty_names(joints)) {
          throw std::invalid_argument(
                  "Parameter 'manual_joint_groups." + group_name +
                  "' must contain unique, nonempty joint names.");
        }
        const auto topic = controller_topic(namespace_name, group_name);
        const auto status_topic =
          controller_status_topic(namespace_name, group_name);
        add_route(group_name, joints, topic, status_topic);
      }
    } else {
      add_route(
        "arm",
        required_joint_group_parameter("arm_joint_names", false),
        absolute_ros_name_parameter(
          "arm_controller_topic", "/xczs/arm_controller/joint_trajectory"),
        absolute_ros_name_parameter(
          "arm_controller_status_topic",
          "/xczs/arm_controller/follow_joint_trajectory/_action/status"));
      const auto gripper_joints = required_joint_group_parameter(
        "gripper_joint_names", true);
      if (!gripper_joints.empty()) {
        add_route(
          "gripper",
          gripper_joints,
          absolute_ros_name_parameter(
            "gripper_controller_topic",
            "/xczs/gripper_controller/joint_trajectory"),
          absolute_ros_name_parameter(
            "gripper_controller_status_topic",
            "/xczs/gripper_controller/follow_joint_trajectory/_action/status"));
      }
    }

    if (routes_.empty()) {
      throw std::invalid_argument(
              "The router must define at least one controller group.");
    }
    validate_groups_are_disjoint();

    input_subscription_ = create_subscription<JointTrajectory>(
      input_topic,
      10,
      [this](const JointTrajectory::SharedPtr message) {
        route(*message);
      });

    RCLCPP_INFO(
      get_logger(),
      "Routing legacy trajectories from %s to %zu ros2_control group(s).",
      input_topic.c_str(), routes_.size());
  }

private:
  struct ControllerRoute
  {
    std::string name;
    std::vector<std::string> joints;
    rclcpp::Publisher<JointTrajectory>::SharedPtr publisher;
    rclcpp::Subscription<GoalStatusArray>::SharedPtr status_subscription;
    // 注意：这个标志初始为 false（「无活动 action → 放行手动轨迹」）。
    // 仅在收到 GoalStatusArray 时置 true。若初始化为 true（fail-closed），
    // 在状态话题静默（如空闲时控制器不发布 _action/status）期间会永久阻断
    // 手动控制，因此保留当前 fail-open 语义。
    bool action_active{false};
  };

  std::string controller_topic(
    const std::string & namespace_name,
    const std::string & group_name) const
  {
    const std::string base = namespace_name + "/" + group_name + "_controller";
    return base + "/joint_trajectory";
  }

  std::string controller_status_topic(
    const std::string & namespace_name,
    const std::string & group_name) const
  {
    const std::string base = namespace_name + "/" + group_name + "_controller";
    return base + "/follow_joint_trajectory/_action/status";
  }

  void add_route(
    const std::string & name,
    const std::vector<std::string> & joints,
    const std::string & topic,
    const std::string & status_topic)
  {
    ControllerRoute route;
    route.name = name;
    route.joints = joints;
    route.publisher = create_publisher<JointTrajectory>(topic, 10);
    route.status_subscription = create_subscription<GoalStatusArray>(
      status_topic,
      10,
      [this, index = routes_.size()](const GoalStatusArray::SharedPtr message) {
        routes_[index].action_active = has_active_goal(*message);
      });
    routes_.push_back(std::move(route));
  }

  void validate_groups_are_disjoint()
  {
    for (std::size_t i = 0; i < routes_.size(); ++i) {
      for (std::size_t j = i + 1; j < routes_.size(); ++j) {
        const auto & left = routes_[i].joints;
        const auto & right = routes_[j].joints;
        if (groups_are_disjoint(left, right)) {
          continue;
        }
        throw std::invalid_argument(
                "Controller groups '" + routes_[i].name + "' and '" +
                routes_[j].name + "' must not overlap.");
      }
    }
  }

  std::vector<std::string> required_joint_group_parameter(
    const std::string & name,
    bool allow_empty)
  {
    const auto names = declare_parameter<std::vector<std::string>>(
      name, std::vector<std::string>{});
    if (!has_unique_nonempty_names(names, allow_empty)) {
      throw std::invalid_argument(
              "Parameter '" + name +
              (allow_empty ?
              "' must be empty or contain unique, nonempty joint names." :
              "' must contain unique, nonempty joint names."));
    }
    return names;
  }

  std::string absolute_ros_name_parameter(
    const std::string & name,
    const std::string & default_value)
  {
    const auto value = declare_parameter<std::string>(name, default_value);
    if (value.empty() || value.front() != '/') {
      throw std::invalid_argument(
              "Parameter '" + name + "' must be an absolute ROS name.");
    }
    try {
      (void)rclcpp::expand_topic_or_service_name(
        value, get_name(), get_namespace(), false);
    } catch (const std::exception & error) {
      throw std::invalid_argument(
              "Parameter '" + name +
              "' must be a valid absolute ROS name: " + error.what());
    }
    return value;
  }

  void route(const JointTrajectory & input)
  {
    if (input.points.empty()) {
      RCLCPP_WARN(get_logger(), "Ignoring an empty legacy trajectory.");
      return;
    }

    try {
      validate_trajectory_contract(input);
      bool any_extracted = false;
      for (auto & route : routes_) {
        const auto sub_trajectory = extract_trajectory(input, route.joints);
        if (!sub_trajectory) {
          continue;
        }
        any_extracted = true;
        if (!route.action_active) {
          if (route.publisher->get_subscription_count() == 0U) {
            // The controller topic is not subscribed by the controller_manager
            // under the active toolset (e.g. rocker/rotate_button controllers
            // do not exist in Set A).  Surface the drop instead of silently
            // publishing into the void and reporting success in the Web UI.
            RCLCPP_WARN_THROTTLE(
              get_logger(),
              *get_clock(),
              2000,
              "Manual '%s' controller has no subscribers under the active "
              "toolset; its trajectory is dropped.",
              route.name.c_str());
            continue;
          }
          route.publisher->publish(*sub_trajectory);
        } else {
          RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "Ignoring manual '%s' trajectory while MoveIt is executing.",
            route.name.c_str());
        }
      }
      if (!any_extracted) {
        RCLCPP_WARN(
          get_logger(),
          "Legacy trajectory contains no complete controller group.");
      }
    } catch (const std::invalid_argument & error) {
      RCLCPP_ERROR(
        get_logger(), "Rejected malformed legacy trajectory: %s", error.what());
    }
  }

  rclcpp::Subscription<JointTrajectory>::SharedPtr input_subscription_;
  std::vector<ControllerRoute> routes_;
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
