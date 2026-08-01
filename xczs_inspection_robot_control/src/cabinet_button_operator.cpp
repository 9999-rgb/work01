// Copyright 2026
// SPDX-License-Identifier: BSD-3-Clause

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <future>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/robot_trajectory/robot_trajectory.h"
#include "moveit/trajectory_processing/time_optimal_trajectory_generation.h"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "xczs_inspection_robot_control/action/operate_cabinet_control.hpp"
#include "xczs_inspection_robot_control/action/press_cabinet_button.hpp"
#include "xczs_inspection_robot_control/msg/cabinet_control.hpp"
#include "xczs_inspection_robot_control/msg/cabinet_control_catalog.hpp"
#include "xczs_inspection_robot_control/msg/cabinet_control_state.hpp"
#include "xczs_inspection_robot_control/srv/set_cabinet_grasp.hpp"

namespace xczs_inspection_robot_control
{

namespace
{

using namespace std::chrono_literals;
using PressCabinetButton =
  xczs_inspection_robot_control::action::PressCabinetButton;
using PressGoalHandle = rclcpp_action::ServerGoalHandle<PressCabinetButton>;
using OperateCabinetControl =
  xczs_inspection_robot_control::action::OperateCabinetControl;
using OperateGoalHandle =
  rclcpp_action::ServerGoalHandle<OperateCabinetControl>;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using NavigationGoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;
using MoveGroupInterface =
  moveit::planning_interface::MoveGroupInterface;

constexpr char kActionName[] = "/xczs/press_cabinet_button";
constexpr char kOperateActionName[] = "/xczs/operate_cabinet_control";
constexpr char kDefaultButtonId[] = "box_10_button_1";
constexpr char kSecondButtonId[] = "box_10_button_2";
constexpr char kControlCatalogTopic[] = "/xczs/cabinet/control_catalog";
constexpr char kActiveControlTopic[] = "/xczs/cabinet/active_control";
constexpr char kResetControlsService[] = "/xczs/cabinet/reset_controls";
constexpr char kArmGroup[] = "manipulator";
constexpr char kArmTipLink[] = "end";
constexpr char kContactToolLink[] = "button_press_tip";
constexpr char kRobotModelName[] = "xczs_inspection_robot";

class OperationError : public std::runtime_error
{
public:
  OperationError(std::uint8_t error_code, const std::string & message)
  : std::runtime_error(message), error_code(error_code)
  {
  }

  std::uint8_t error_code;
};

class GenericOperationError : public std::runtime_error
{
public:
  GenericOperationError(std::uint8_t error_code, const std::string & message)
  : std::runtime_error(message), error_code(error_code)
  {
  }

  std::uint8_t error_code;
};

struct OperationPoses
{
  geometry_msgs::msg::PoseStamped staging_pose;
  geometry_msgs::msg::PoseStamped staging_pose_in_planning_frame;
  geometry_msgs::msg::Pose prepress_pose;
  geometry_msgs::msg::Pose contact_pose;
  geometry_msgs::msg::Pose pressed_pose;
};

struct RotaryOperationPoses
{
  geometry_msgs::msg::PoseStamped staging_pose;
  geometry_msgs::msg::PoseStamped staging_pose_in_planning_frame;
  geometry_msgs::msg::Pose ready_pose;
  geometry_msgs::msg::Pose grasp_pose;
};

struct ButtonSnapshot
{
  bool received{false};
  bool pressed_received{false};
  bool pressed{false};
  std::uint64_t pressed_transition_sequence{0};
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
  double max_position{0.0};
  bool valid{false};
  bool in_motion{false};
  std::string state_id;
  std::chrono::steady_clock::time_point received_at{};
  std::chrono::steady_clock::time_point pressed_received_at{};
};

struct ButtonRuntime
{
  mutable std::mutex mutex;
  std::condition_variable condition;
  ButtonSnapshot state;
};

struct ButtonSpec
{
  std::uint8_t control_type{
    xczs_inspection_robot_control::msg::CabinetControl::TYPE_BUTTON};
  std::string id;
  std::string display_name;
  std::string joint_name;
  std::string joint_state_topic;
  std::string pressed_topic;
  std::string state_topic;
  std::string unit;
  std::uint8_t supported_commands{0};
  bool requires_grasp{false};
  bool operable{true};
  std::string unavailable_reason;
  std::string parent_control_id;
  double local_x{0.0};
  double local_y{0.0};
  double local_z{0.0};
  double pivot_x{0.0};
  double pivot_y{0.0};
  double pivot_z{0.0};
  tf2::Vector3 axis{0.0, 0.0, -1.0};
  tf2::Vector3 approach_normal{0.0, 0.0, 1.0};
  double min_position{0.0};
  double max_position{0.008};
  std::vector<std::string> state_ids;
  std::vector<std::string> state_labels;
  std::vector<double> state_positions;
  std::shared_ptr<ButtonRuntime> runtime{std::make_shared<ButtonRuntime>()};
};

struct ResolvedControlGeometry
{
  tf2::Vector3 grasp_zero;
  tf2::Vector3 pivot;
  tf2::Vector3 axis;
  tf2::Vector3 approach_normal;
};

geometry_msgs::msg::Quaternion to_message(const tf2::Quaternion & quaternion)
{
  geometry_msgs::msg::Quaternion message;
  message.x = quaternion.x();
  message.y = quaternion.y();
  message.z = quaternion.z();
  message.w = quaternion.w();
  return message;
}

}  // namespace

class CabinetButtonOperator final : public rclcpp::Node
{
public:
  CabinetButtonOperator()
  : Node("xczs_cabinet_button_operator")
  {
    const auto legacy_button_id = declare_parameter<std::string>(
      "button_id", kDefaultButtonId);
    planning_frame_ = declare_parameter<std::string>(
      "planning_frame", "odom");
    navigation_frame_ = declare_parameter<std::string>(
      "navigation_frame", "map");
    const auto legacy_button_joint_name = declare_parameter<std::string>(
      "button_joint_name", "box_10_box_10_button_1");
    const auto legacy_button_joint_state_topic =
      declare_parameter<std::string>(
      "button_joint_state_topic",
      "/xczs/cabinet/box_10_button_1/joint_states");
    const auto legacy_button_pressed_topic = declare_parameter<std::string>(
      "button_pressed_topic",
      "/xczs/cabinet/box_10_button_1/pressed");
    const auto control_catalog_topic = declare_parameter<std::string>(
      "control_catalog_topic", kControlCatalogTopic);
    cabinet_frame_ = declare_parameter<std::string>(
      "cabinet_frame", "control_cabinet_frame");
    const auto grasp_service = declare_parameter<std::string>(
      "grasp_service", "/xczs/cabinet/grasp");
    const auto reset_physics_service = declare_parameter<std::string>(
      "reset_physics_service", "/xczs/cabinet/reset_physics");
    const auto manual_base_topic = declare_parameter<std::string>(
      "manual_base_topic", "/xczs/manual_cmd_vel");

    cabinet_x_ = declare_parameter<double>("cabinet_x", 2.0);
    cabinet_y_ = declare_parameter<double>("cabinet_y", 0.33);
    cabinet_z_ = declare_parameter<double>("cabinet_z", 0.0);
    cabinet_roll_ = declare_parameter<double>(
      "cabinet_roll", 1.57079632679);
    cabinet_pitch_ = declare_parameter<double>("cabinet_pitch", 0.0);
    cabinet_yaw_ = declare_parameter<double>(
      "cabinet_yaw", -1.57079632679);

    const double legacy_button_local_x = declare_parameter<double>(
      "button_local_x", 0.492);
    const double legacy_button_local_y = declare_parameter<double>(
      "button_local_y", 0.574);
    const double legacy_button_local_z = declare_parameter<double>(
      "button_local_z", 0.011494);
    tool_tip_offset_ = positive_parameter("tool_tip_offset", 0.370);
    staging_distance_ = positive_parameter("staging_distance", 1.040);
    prepress_distance_ = positive_parameter("prepress_distance", 0.060);
    contact_clearance_ = positive_parameter("contact_clearance", 0.001);
    press_depth_ = positive_parameter("press_depth", 0.007);
    button_state_timeout_ = positive_parameter("state_timeout", 1.0);
    system_wait_timeout_ = positive_parameter("system_wait_timeout", 15.0);
    navigation_timeout_ = positive_parameter(
      "navigation_timeout", 120.0);
    navigation_takeover_distance_ = positive_parameter(
      "navigation_takeover_distance", 0.15);
    docking_timeout_ = positive_parameter("docking_timeout", 20.0);
    docking_position_tolerance_ = positive_parameter(
      "docking_position_tolerance", 0.015);
    docking_yaw_tolerance_ = positive_parameter(
      "docking_yaw_tolerance", 0.10);
    docking_max_linear_speed_ = positive_parameter(
      "docking_max_linear_speed", 0.15);
    docking_max_angular_speed_ = positive_parameter(
      "docking_max_angular_speed", 0.30);
    docking_linear_gain_ = positive_parameter(
      "docking_linear_gain", 0.8);
    docking_angular_gain_ = positive_parameter(
      "docking_angular_gain", 1.2);
    base_link_yaw_offset_ = declare_parameter<double>(
      "base_link_yaw_offset", 1.57079632679);
    press_detection_timeout_ = positive_parameter(
      "press_detection_timeout", 3.0);
    release_detection_timeout_ = positive_parameter(
      "release_detection_timeout", 3.0);
    press_hold_seconds_ = positive_parameter("press_hold_seconds", 0.5);
    target_tolerance_ = positive_parameter("target_tolerance", 0.035);
    stable_velocity_tolerance_ = positive_parameter(
      "stable_velocity_tolerance", 0.03);
    stable_state_duration_ = positive_parameter(
      "stable_state_duration", 0.30);
    planning_scene_settle_seconds_ = positive_parameter(
      "planning_scene_settle_seconds", 0.50);
    rotation_waypoint_step_ = positive_parameter(
      "rotation_waypoint_step", 0.03490658504);
    cartesian_velocity_scale_ = unit_interval_parameter(
      "cartesian_velocity_scale", 0.08);
    cartesian_acceleration_scale_ = unit_interval_parameter(
      "cartesian_acceleration_scale", 0.08);

    configure_controls(
      legacy_button_id,
      legacy_button_joint_name,
      legacy_button_joint_state_topic,
      legacy_button_pressed_topic,
      {legacy_button_local_x, legacy_button_local_y,
        legacy_button_local_z});
    subscribe_to_button_states();
    control_catalog_publisher_ =
      create_publisher<
      xczs_inspection_robot_control::msg::CabinetControlCatalog>(
      control_catalog_topic,
      rclcpp::QoS(1).reliable().transient_local());
    active_control_publisher_ = create_publisher<std_msgs::msg::String>(
      kActiveControlTopic,
      rclcpp::QoS(1).reliable().transient_local());

    navigation_client_ = rclcpp_action::create_client<NavigateToPose>(
      this, "/navigate_to_pose");
    navigation_mode_client_ = create_client<std_srvs::srv::SetBool>(
      "/xczs/set_navigation_mode");
    grasp_client_ =
      create_client<xczs_inspection_robot_control::srv::SetCabinetGrasp>(
      grasp_service);
    reset_client_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
    reset_physics_client_ = create_client<std_srvs::srv::Trigger>(
      reset_physics_service, rmw_qos_profile_services_default,
      reset_client_callback_group_);
    manual_base_publisher_ = create_publisher<geometry_msgs::msg::Twist>(
      manual_base_topic, 10);
    transform_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    transform_listener_ =
      std::make_unique<tf2_ros::TransformListener>(*transform_buffer_);

    action_server_ = rclcpp_action::create_server<PressCabinetButton>(
      this,
      kActionName,
      [this](
        const rclcpp_action::GoalUUID & uuid,
        const std::shared_ptr<const PressCabinetButton::Goal> goal)
      {
        return handle_goal(uuid, goal);
      },
      [this](const std::shared_ptr<PressGoalHandle> goal_handle) {
        return handle_cancel(goal_handle);
      },
      [this](const std::shared_ptr<PressGoalHandle> goal_handle) {
        handle_accepted(goal_handle);
      });
    operate_action_server_ =
      rclcpp_action::create_server<OperateCabinetControl>(
      this,
      kOperateActionName,
      [this](
        const rclcpp_action::GoalUUID & uuid,
        const std::shared_ptr<const OperateCabinetControl::Goal> goal)
      {
        return handle_operate_goal(uuid, goal);
      },
      [this](const std::shared_ptr<OperateGoalHandle> goal_handle) {
        return handle_operate_cancel(goal_handle);
      },
      [this](const std::shared_ptr<OperateGoalHandle> goal_handle) {
        handle_operate_accepted(goal_handle);
      });
    reset_controls_service_ = create_service<std_srvs::srv::Trigger>(
      kResetControlsService,
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
      {
        reset_controls(response);
      });
    publish_control_catalog();
    publish_active_control("");

    RCLCPP_INFO(
      get_logger(),
      "Cabinet action %s controls %zu configured cabinet controls.",
      kActionName, buttons_in_order_.size());
  }

  ~CabinetButtonOperator() override
  {
    cancel_requested_.store(true);
    stop_active_motion();
    cancel_active_navigation();
    std::lock_guard<std::mutex> lock(worker_mutex_);
    if (worker_thread_.joinable()) {
      worker_thread_.join();
    }
  }

private:
  void configure_controls(
    const std::string & legacy_button_id,
    const std::string & legacy_joint_name,
    const std::string & legacy_joint_state_topic,
    const std::string & legacy_pressed_topic,
    const std::vector<double> & legacy_local_position)
  {
    const std::vector<std::string> default_control_ids =
      legacy_button_id == kDefaultButtonId ?
      std::vector<std::string>{kDefaultButtonId, kSecondButtonId} :
    std::vector<std::string>{legacy_button_id};
    const auto control_ids = declare_parameter<std::vector<std::string>>(
      "control_ids", default_control_ids);
    if (control_ids.empty()) {
      throw std::invalid_argument(
              "Parameter 'control_ids' must contain at least one control.");
    }

    const auto button_default_axis = declare_parameter<std::vector<double>>(
      "button_defaults.axis", {0.0, 0.0, -1.0});
    const auto button_default_approach =
      declare_parameter<std::vector<double>>(
      "button_defaults.approach_normal", {0.0, 0.0, 1.0});
    const double button_default_min = declare_parameter<double>(
      "button_defaults.min_position", 0.0);
    const double button_default_max = declare_parameter<double>(
      "button_defaults.max_position", 0.008);
    const auto button_default_state_ids =
      declare_parameter<std::vector<std::string>>(
      "button_defaults.state_ids", {"released", "pressed"});
    const auto button_default_state_labels =
      declare_parameter<std::vector<std::string>>(
      "button_defaults.state_labels", {"释放", "按下"});
    const auto button_default_state_positions =
      declare_parameter<std::vector<double>>(
      "button_defaults.state_positions", {0.0, 0.006});

    const auto knob_default_axis = declare_parameter<std::vector<double>>(
      "knob_defaults.axis", {0.0, 0.0, 1.0});
    const auto knob_default_approach =
      declare_parameter<std::vector<double>>(
      "knob_defaults.approach_normal", {0.0, 0.0, 1.0});
    const double knob_default_min = declare_parameter<double>(
      "knob_defaults.min_position", -0.78539816339);
    const double knob_default_max = declare_parameter<double>(
      "knob_defaults.max_position", 0.78539816339);
    const auto knob_default_state_ids =
      declare_parameter<std::vector<std::string>>(
      "knob_defaults.state_ids", {"left", "center", "right"});
    const auto knob_default_state_labels =
      declare_parameter<std::vector<std::string>>(
      "knob_defaults.state_labels", {"左档", "中档", "右档"});
    const auto knob_default_state_positions =
      declare_parameter<std::vector<double>>(
      "knob_defaults.state_positions",
      {-0.78539816339, 0.0, 0.78539816339});

    const auto checked_vector3 = [](const std::vector<double> & values,
        const std::string & name) {
        if (values.size() != 3U ||
          !std::all_of(
            values.begin(), values.end(),
            [](double value) {return std::isfinite(value);}))
        {
          throw std::invalid_argument(
                  "Parameter '" + name +
                  "' must contain three finite values.");
        }
        return tf2::Vector3(values[0], values[1], values[2]);
      };
    checked_vector3(button_default_axis, "button_defaults.axis");
    checked_vector3(
      button_default_approach, "button_defaults.approach_normal");
    checked_vector3(knob_default_axis, "knob_defaults.axis");
    checked_vector3(knob_default_approach, "knob_defaults.approach_normal");

    std::unordered_set<std::string> seen_button_ids;
    for (const auto & control_id : control_ids) {
      if (control_id.empty()) {
        throw std::invalid_argument(
                "Parameter 'control_ids' must not contain an empty ID.");
      }
      if (!seen_button_ids.insert(control_id).second) {
        throw std::invalid_argument(
                "Duplicate cabinet control ID '" + control_id + "'.");
      }

      const bool is_legacy_button = control_id == legacy_button_id;
      const bool is_second_button = control_id == kSecondButtonId;
      const std::string default_display_name = is_second_button ?
        "10号模块绿色按钮" :
        (control_id == kDefaultButtonId ?
        "10号模块红色按钮" : control_id);
      const std::string default_joint_name = is_legacy_button ?
        legacy_joint_name :
        (is_second_button ? "box_10_box_10_button_2" : control_id);
      const std::string default_joint_state_topic = is_legacy_button ?
        legacy_joint_state_topic :
        "/xczs/cabinet/" + control_id + "/joint_states";
      const std::string default_pressed_topic = is_legacy_button ?
        legacy_pressed_topic :
        "/xczs/cabinet/" + control_id + "/pressed";
      const std::vector<double> default_local_position = is_legacy_button ?
        legacy_local_position :
        (is_second_button ?
        std::vector<double>{0.527, 0.574, 0.011494} :
        std::vector<double>{0.0, 0.0, 0.0});

      const std::string prefix = "controls." + control_id + ".";
      auto button = std::make_shared<ButtonSpec>();
      button->id = control_id;
      const auto type_name = declare_parameter<std::string>(
        prefix + "type", "button");
      if (type_name == "button") {
        button->control_type =
          xczs_inspection_robot_control::msg::CabinetControl::TYPE_BUTTON;
      } else if (type_name == "knob") {
        button->control_type =
          xczs_inspection_robot_control::msg::CabinetControl::TYPE_KNOB;
      } else if (type_name == "switch") {
        button->control_type =
          xczs_inspection_robot_control::msg::CabinetControl::TYPE_SWITCH;
      } else if (type_name == "door") {
        button->control_type =
          xczs_inspection_robot_control::msg::CabinetControl::TYPE_DOOR;
      } else {
        throw std::invalid_argument(
                "Unsupported control type '" + type_name + "' for '" +
                control_id + "'.");
      }
      const bool is_button = button->control_type ==
        xczs_inspection_robot_control::msg::CabinetControl::TYPE_BUTTON;
      const bool is_knob = button->control_type ==
        xczs_inspection_robot_control::msg::CabinetControl::TYPE_KNOB;
      button->display_name = declare_parameter<std::string>(
        prefix + "display_name", default_display_name);
      button->joint_name = declare_parameter<std::string>(
        prefix + "joint_name", default_joint_name);
      button->joint_state_topic = declare_parameter<std::string>(
        prefix + "joint_state_topic", default_joint_state_topic);
      button->pressed_topic = declare_parameter<std::string>(
        prefix + "pressed_topic", is_button ? default_pressed_topic : "");
      button->state_topic = declare_parameter<std::string>(
        prefix + "state_topic",
        "/xczs/cabinet/" + control_id + "/state");
      const auto local_position = declare_parameter<std::vector<double>>(
        prefix + "local_position", default_local_position);
      const auto pivot_position = declare_parameter<std::vector<double>>(
        prefix + "pivot_position", local_position);
      const auto axis = declare_parameter<std::vector<double>>(
        prefix + "axis", is_knob ? knob_default_axis : button_default_axis);
      const auto approach_normal = declare_parameter<std::vector<double>>(
        prefix + "approach_normal",
        is_knob ? knob_default_approach : button_default_approach);
      button->min_position = declare_parameter<double>(
        prefix + "min_position",
        is_knob ? knob_default_min : button_default_min);
      button->max_position = declare_parameter<double>(
        prefix + "max_position",
        is_knob ? knob_default_max : button_default_max);
      button->state_ids = declare_parameter<std::vector<std::string>>(
        prefix + "state_ids",
        is_knob ? knob_default_state_ids : button_default_state_ids);
      button->state_labels = declare_parameter<std::vector<std::string>>(
        prefix + "state_labels",
        is_knob ? knob_default_state_labels : button_default_state_labels);
      button->state_positions = declare_parameter<std::vector<double>>(
        prefix + "state_positions",
        is_knob ? knob_default_state_positions :
        button_default_state_positions);
      button->operable = declare_parameter<bool>(
        prefix + "operable", true);
      button->unavailable_reason = declare_parameter<std::string>(
        prefix + "unavailable_reason", "");
      button->parent_control_id = declare_parameter<std::string>(
        prefix + "parent_control_id", "");
      button->requires_grasp = declare_parameter<bool>(
        prefix + "requires_grasp", !is_button);
      button->unit = is_button ? "m" : "rad";
      button->supported_commands = is_button ?
        xczs_inspection_robot_control::msg::CabinetControl::SUPPORT_PRESS :
        static_cast<std::uint8_t>(
        xczs_inspection_robot_control::msg::CabinetControl::SUPPORT_SET_STATE |
        xczs_inspection_robot_control::msg::CabinetControl::
        SUPPORT_SET_POSITION |
        xczs_inspection_robot_control::msg::CabinetControl::SUPPORT_TOGGLE);
      if (button->display_name.empty() || button->joint_name.empty() ||
        button->joint_state_topic.empty() || button->state_topic.empty() ||
        (is_button && button->pressed_topic.empty()))
      {
        throw std::invalid_argument(
                "Cabinet control '" + control_id +
                "' has an empty catalog field.");
      }
      const auto local = checked_vector3(
        local_position, prefix + "local_position");
      const auto pivot = checked_vector3(
        pivot_position, prefix + "pivot_position");
      button->axis = checked_vector3(axis, prefix + "axis");
      button->approach_normal = checked_vector3(
        approach_normal, prefix + "approach_normal");
      if (button->axis.length2() < 1.0e-12 ||
        button->approach_normal.length2() < 1.0e-12)
      {
        throw std::invalid_argument(
                "Control '" + control_id + "' has a zero axis or normal.");
      }
      button->axis.normalize();
      button->approach_normal.normalize();
      button->local_x = local.x();
      button->local_y = local.y();
      button->local_z = local.z();
      button->pivot_x = pivot.x();
      button->pivot_y = pivot.y();
      button->pivot_z = pivot.z();
      if (!std::isfinite(button->min_position) ||
        !std::isfinite(button->max_position) ||
        button->min_position >= button->max_position ||
        button->state_ids.size() != button->state_labels.size() ||
        button->state_ids.size() != button->state_positions.size() ||
        button->state_ids.empty())
      {
        throw std::invalid_argument(
                "Control '" + control_id +
                "' has invalid limits or state presets.");
      }
      for (const double position : button->state_positions) {
        if (!std::isfinite(position) || position < button->min_position - 1e-9 ||
          position > button->max_position + 1e-9)
        {
          throw std::invalid_argument(
                  "Control '" + control_id +
                  "' has a preset outside its limits.");
        }
      }
      buttons_by_id_.emplace(control_id, button);
      buttons_in_order_.push_back(std::move(button));
    }

    for (const auto & control : buttons_in_order_) {
      std::unordered_set<std::string> ancestors;
      std::string parent_id = control->parent_control_id;
      while (!parent_id.empty()) {
        const auto parent = buttons_by_id_.find(parent_id);
        if (parent == buttons_by_id_.end()) {
          throw std::invalid_argument(
                  "Control '" + control->id + "' references unknown parent '" +
                  parent_id + "'.");
        }
        if (!ancestors.insert(parent_id).second || parent_id == control->id) {
          throw std::invalid_argument(
                  "Control '" + control->id +
                  "' has a cyclic parent_control_id chain.");
        }
        if (parent->second->control_type ==
          xczs_inspection_robot_control::msg::CabinetControl::TYPE_BUTTON)
        {
          throw std::invalid_argument(
                  "Control '" + control->id +
                  "' cannot use a prismatic button as its parent.");
        }
        parent_id = parent->second->parent_control_id;
      }
    }
  }

  void subscribe_to_button_states()
  {
    for (const auto & button : buttons_in_order_) {
      button_joint_state_subscriptions_.push_back(
        create_subscription<sensor_msgs::msg::JointState>(
          button->joint_state_topic,
          10,
          [this, button](
            const sensor_msgs::msg::JointState::SharedPtr message)
          {
            receive_button_joint_state(*button, *message);
          }));
      if (!button->pressed_topic.empty()) {
        button_pressed_subscriptions_.push_back(
          create_subscription<std_msgs::msg::Bool>(
            button->pressed_topic,
            rclcpp::QoS(1).reliable().transient_local(),
            [this, button](const std_msgs::msg::Bool::SharedPtr message) {
              receive_button_pressed(*button, *message);
            }));
      }
      control_state_subscriptions_.push_back(
        create_subscription<
          xczs_inspection_robot_control::msg::CabinetControlState>(
          button->state_topic,
          rclcpp::QoS(1).reliable().transient_local(),
          [this, button](
            const xczs_inspection_robot_control::msg::CabinetControlState::
            SharedPtr message)
          {
            receive_control_state(*button, *message);
          }));
    }
  }

  void publish_control_catalog()
  {
    xczs_inspection_robot_control::msg::CabinetControlCatalog catalog;
    catalog.controls.reserve(buttons_in_order_.size());
    for (const auto & button : buttons_in_order_) {
      xczs_inspection_robot_control::msg::CabinetControl control;
      control.control_id = button->id;
      control.display_name = button->display_name;
      control.control_type =
        button->control_type;
      control.joint_name = button->joint_name;
      control.joint_state_topic = button->joint_state_topic;
      control.pressed_topic = button->pressed_topic;
      control.state_topic = button->state_topic;
      control.supported_commands = button->supported_commands;
      control.unit = button->unit;
      control.min_position = button->min_position;
      control.max_position = button->max_position;
      control.state_ids = button->state_ids;
      control.state_labels = button->state_labels;
      control.state_positions = button->state_positions;
      control.requires_grasp = button->requires_grasp;
      control.operable = button->operable;
      control.unavailable_reason = button->unavailable_reason;
      catalog.controls.push_back(std::move(control));
    }
    control_catalog_publisher_->publish(catalog);
  }

  std::shared_ptr<ButtonSpec> find_button(
    const std::string & button_id) const
  {
    const auto iterator = buttons_by_id_.find(button_id);
    return iterator == buttons_by_id_.end() ? nullptr : iterator->second;
  }

  void publish_active_control(const std::string & control_id)
  {
    std_msgs::msg::String message;
    message.data = control_id;
    active_control_publisher_->publish(message);
  }

  void reset_controls(
    const std::shared_ptr<std_srvs::srv::Trigger::Response> & response)
  {
    if (operation_active_.load()) {
      response->success = false;
      response->message =
        "Cabinet controls cannot be reset during an active operation.";
      return;
    }
    if (!reset_physics_client_->wait_for_service(2s)) {
      response->success = false;
      response->message = "Cabinet reset_physics service is unavailable.";
      return;
    }
    auto future = reset_physics_client_->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>());
    if (future.wait_for(3s) != std::future_status::ready) {
      response->success = false;
      response->message = "Cabinet reset_physics service timed out.";
      return;
    }
    try {
      const auto reset_response = future.get();
      response->success = reset_response->success;
      response->message = reset_response->message;
    } catch (const std::exception & error) {
      response->success = false;
      response->message = std::string("Cabinet reset failed: ") + error.what();
    }
  }

  rclcpp_action::GoalResponse handle_operate_goal(
    const rclcpp_action::GoalUUID &,
    const std::shared_ptr<const OperateCabinetControl::Goal> goal)
  {
    const auto control = find_button(goal->control_id);
    if (!control || !control->operable) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    const bool is_button = control->control_type ==
      xczs_inspection_robot_control::msg::CabinetControl::TYPE_BUTTON;
    bool command_valid = false;
    if (is_button) {
      command_valid = goal->command ==
        OperateCabinetControl::Goal::COMMAND_PRESS;
    } else {
      command_valid = goal->command ==
        OperateCabinetControl::Goal::COMMAND_SET_STATE ||
        goal->command ==
        OperateCabinetControl::Goal::COMMAND_SET_POSITION ||
        goal->command == OperateCabinetControl::Goal::COMMAND_TOGGLE;
      if (goal->command ==
        OperateCabinetControl::Goal::COMMAND_SET_STATE)
      {
        command_valid = std::find(
          control->state_ids.begin(), control->state_ids.end(),
          goal->target_state) != control->state_ids.end();
      } else if (goal->command ==
        OperateCabinetControl::Goal::COMMAND_SET_POSITION)
      {
        command_valid = goal->use_target_position &&
          std::isfinite(goal->target_position) &&
          goal->target_position >= control->min_position &&
          goal->target_position <= control->max_position &&
          std::any_of(
          control->state_positions.begin(), control->state_positions.end(),
          [this, &goal](double preset) {
            return std::abs(preset - goal->target_position) <=
                   target_tolerance_;
          });
      } else if (goal->command ==
        OperateCabinetControl::Goal::COMMAND_TOGGLE)
      {
        command_valid = control->state_ids.size() >= 2U;
      }
    }
    if (!command_valid) {
      RCLCPP_WARN(
        get_logger(), "Rejected unsupported operation for '%s'.",
        goal->control_id.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    bool expected = false;
    if (!operation_active_.compare_exchange_strong(expected, true)) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    cancel_requested_.store(false);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_operate_cancel(
    const std::shared_ptr<OperateGoalHandle>)
  {
    cancel_requested_.store(true);
    stop_active_motion();
    cancel_active_navigation();
    publish_manual_base_stop();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_operate_accepted(
    const std::shared_ptr<OperateGoalHandle> goal_handle)
  {
    publish_active_control(goal_handle->get_goal()->control_id);
    std::lock_guard<std::mutex> lock(worker_mutex_);
    if (worker_thread_.joinable()) {
      worker_thread_.join();
    }
    worker_thread_ = std::thread(
      [this, goal_handle]() {
        execute_operate(goal_handle);
      });
  }

  double positive_parameter(
    const std::string & name,
    double default_value)
  {
    const double value = declare_parameter<double>(name, default_value);
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument(
              "Parameter '" + name + "' must be positive.");
    }
    return value;
  }

  double unit_interval_parameter(
    const std::string & name,
    double default_value)
  {
    const double value = positive_parameter(name, default_value);
    if (value > 1.0) {
      throw std::invalid_argument(
              "Parameter '" + name + "' must not exceed 1.0.");
    }
    return value;
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    const std::shared_ptr<const PressCabinetButton::Goal> goal)
  {
    const auto control = find_button(goal->button_id);
    if (!control || control->control_type !=
      xczs_inspection_robot_control::msg::CabinetControl::TYPE_BUTTON ||
      !control->operable)
    {
      RCLCPP_WARN(
        get_logger(), "Rejected unsupported cabinet button '%s'.",
        goal->button_id.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    bool expected = false;
    if (!operation_active_.compare_exchange_strong(expected, true)) {
      RCLCPP_WARN(get_logger(), "Rejected a concurrent cabinet operation.");
      return rclcpp_action::GoalResponse::REJECT;
    }
    cancel_requested_.store(false);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<PressGoalHandle>)
  {
    cancel_requested_.store(true);
    stop_active_motion();
    cancel_active_navigation();
    publish_manual_base_stop();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<PressGoalHandle> goal_handle)
  {
    publish_active_control(goal_handle->get_goal()->button_id);
    std::lock_guard<std::mutex> lock(worker_mutex_);
    if (worker_thread_.joinable()) {
      worker_thread_.join();
    }
    worker_thread_ = std::thread(
      [this, goal_handle]() {
        execute(goal_handle);
      });
  }

  void execute(const std::shared_ptr<PressGoalHandle> goal_handle) noexcept
  {
    auto result = std::make_shared<PressCabinetButton::Result>();
    const auto button = find_button(goal_handle->get_goal()->button_id);
    std::shared_ptr<MoveGroupInterface> move_group;
    OperationPoses poses;
    bool should_attempt_retreat = false;

    try {
      if (!button) {
        throw OperationError(
                PressCabinetButton::Result::INTERNAL_ERROR,
                "The accepted cabinet button is no longer configured.");
      }
      reset_max_button_travel(*button);
      publish_feedback(
        goal_handle,
        PressCabinetButton::Feedback::WAITING_FOR_SYSTEM,
        0.02F,
        "Waiting for the cabinet button state.");
      wait_for_fresh_button_state(goal_handle, *button);
      check_cancel(goal_handle);
      interruptible_hold(goal_handle, planning_scene_settle_seconds_);

      const bool should_navigate_to_staging_pose =
        goal_handle->get_goal()->navigate_to_staging_pose;
      poses = calculate_operation_poses(
        *button, should_navigate_to_staging_pose);
      if (should_navigate_to_staging_pose) {
        publish_feedback(
          goal_handle,
          PressCabinetButton::Feedback::NAVIGATING,
          0.08F,
          "Driving the base to the cabinet staging pose.");
        navigate_to_staging_pose(goal_handle, poses.staging_pose);
      }
      set_navigation_mode(goal_handle, false);
      if (should_navigate_to_staging_pose) {
        publish_feedback(
          goal_handle,
          PressCabinetButton::Feedback::NAVIGATING,
          0.20F,
          "Refining the cabinet staging pose from Gazebo odometry.");
        dock_to_staging_pose(
          goal_handle, poses.staging_pose_in_planning_frame);
      }

      move_group = std::make_shared<MoveGroupInterface>(
        shared_from_this(),
        kArmGroup,
        std::shared_ptr<tf2_ros::Buffer>(),
        rclcpp::Duration::from_seconds(system_wait_timeout_));
      check_cancel(goal_handle);
      {
        std::lock_guard<std::mutex> lock(motion_mutex_);
        active_move_group_ = move_group;
      }
      configure_move_group(*move_group);
      if (!move_group->getCurrentState(system_wait_timeout_)) {
        throw OperationError(
                PressCabinetButton::Result::NOT_READY,
                "MoveIt did not receive the current robot state.");
      }

      publish_feedback(
        goal_handle,
        PressCabinetButton::Feedback::MOVING_TO_PREPRESS,
        0.25F,
        "Planning to the button prepress pose.");
      plan_and_execute_pose(
        *move_group, goal_handle, poses.prepress_pose);
      should_attempt_retreat = true;

      publish_feedback(
        goal_handle,
        PressCabinetButton::Feedback::APPROACHING,
        0.48F,
        "Approaching the button along the panel normal.");
      execute_cartesian_path(
        *move_group,
        goal_handle,
        {poses.contact_pose},
        cartesian_velocity_scale_,
        cartesian_acceleration_scale_);

      publish_feedback(
        goal_handle,
        PressCabinetButton::Feedback::PRESSING,
        0.67F,
        "Pressing " + button->id + " through its physical travel.");
      const auto press_transition_sequence =
        button_snapshot(*button).pressed_transition_sequence;
      execute_cartesian_path(
        *move_group,
        goal_handle,
        {poses.pressed_pose},
        cartesian_velocity_scale_ * 0.5,
        cartesian_acceleration_scale_ * 0.5);

      publish_feedback(
        goal_handle,
        PressCabinetButton::Feedback::VERIFYING_PRESS,
        0.78F,
        "Verifying the Gazebo button press state.");
      if (!wait_for_pressed_state(
          goal_handle,
          *button,
          true,
          press_detection_timeout_,
          press_transition_sequence))
      {
        throw OperationError(
                PressCabinetButton::Result::PRESS_NOT_DETECTED,
                "The arm reached the press pose, but the button did not "
                "cross its 6 mm press threshold.");
      }
      interruptible_hold(goal_handle, press_hold_seconds_);

      publish_feedback(
        goal_handle,
        PressCabinetButton::Feedback::RETRACTING,
        0.88F,
        "Retracting the probe from the cabinet.");
      const auto release_transition_sequence =
        button_snapshot(*button).pressed_transition_sequence;
      execute_cartesian_path(
        *move_group,
        goal_handle,
        {poses.contact_pose, poses.prepress_pose},
        cartesian_velocity_scale_,
        cartesian_acceleration_scale_);
      should_attempt_retreat = false;

      publish_feedback(
        goal_handle,
        PressCabinetButton::Feedback::VERIFYING_RELEASE,
        0.97F,
        "Verifying that the button spring returned to its released state.");
      if (!wait_for_pressed_state(
          goal_handle,
          *button,
          false,
          release_detection_timeout_,
          release_transition_sequence))
      {
        throw OperationError(
                PressCabinetButton::Result::RELEASE_NOT_DETECTED,
                "The probe retracted, but the button did not return below "
                "its release threshold.");
      }

      result->success = true;
      result->error_code = PressCabinetButton::Result::SUCCESS;
      result->message =
        "Pressed and released " + button->id + " successfully.";
      result->max_travel = button_snapshot(*button).max_position;
      if (finish_goal_noexcept(goal_handle, result, true)) {
        RCLCPP_INFO(
          get_logger(), "Cabinet button %s operation succeeded "
          "(max %.2f mm).",
          button->id.c_str(), result->max_travel * 1000.0);
      }
    } catch (const OperationError & error) {
      if (should_attempt_retreat && move_group && rclcpp::ok()) {
        best_effort_retreat(*move_group, poses.prepress_pose);
      }
      result->success = false;
      result->error_code = error.error_code;
      result->message = error.what();
      result->max_travel = button ?
        button_snapshot(*button).max_position : 0.0;
      const bool client_canceled = is_goal_canceling_noexcept(goal_handle);
      if (client_canceled) {
        result->error_code = PressCabinetButton::Result::CANCELED;
        result->message = "Cabinet button operation was canceled.";
      } else if (error.error_code == PressCabinetButton::Result::CANCELED) {
        result->error_code = PressCabinetButton::Result::INTERNAL_ERROR;
        result->message =
          "Cabinet button operation stopped because ROS is shutting down.";
      }
      finish_goal_noexcept(goal_handle, result, false);
      RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
    } catch (const std::exception & error) {
      if (should_attempt_retreat && move_group && rclcpp::ok()) {
        best_effort_retreat(*move_group, poses.prepress_pose);
      }
      result->success = false;
      const bool canceled = is_goal_canceling_noexcept(goal_handle);
      result->error_code = canceled ?
        PressCabinetButton::Result::CANCELED :
        PressCabinetButton::Result::INTERNAL_ERROR;
      result->message = canceled ?
        "Cabinet button operation was canceled." :
        std::string("Cabinet operation failed: ") + error.what();
      result->max_travel = button ?
        button_snapshot(*button).max_position : 0.0;
      finish_goal_noexcept(goal_handle, result, false);
      RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
    } catch (...) {
      if (should_attempt_retreat && move_group && rclcpp::ok()) {
        best_effort_retreat(*move_group, poses.prepress_pose);
      }
      result->success = false;
      const bool canceled = is_goal_canceling_noexcept(goal_handle);
      result->error_code = canceled ?
        PressCabinetButton::Result::CANCELED :
        PressCabinetButton::Result::INTERNAL_ERROR;
      result->message = canceled ?
        "Cabinet button operation was canceled." :
        "Cabinet operation failed with an unknown exception.";
      result->max_travel = button ?
        button_snapshot(*button).max_position : 0.0;
      finish_goal_noexcept(goal_handle, result, false);
      RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
    }

    stop_active_motion();
    cancel_active_navigation();
    request_navigation_mode_without_wait(false);
    {
      std::lock_guard<std::mutex> lock(motion_mutex_);
      active_move_group_.reset();
    }
    operation_active_.store(false);
    publish_active_control("");
  }

  void execute_operate(
    const std::shared_ptr<OperateGoalHandle> goal_handle) noexcept
  {
    auto result = std::make_shared<OperateCabinetControl::Result>();
    const auto control = find_button(goal_handle->get_goal()->control_id);
    std::shared_ptr<MoveGroupInterface> move_group;
    OperationPoses button_poses;
    RotaryOperationPoses rotary_poses;
    bool should_attempt_retreat = false;
    bool grasp_attached = false;
    double target_position = 0.0;
    std::string target_state;

    try {
      if (!control) {
        throw GenericOperationError(
                OperateCabinetControl::Result::INVALID_CONTROL,
                "The accepted cabinet control is no longer configured.");
      }
      reset_max_button_travel(*control);
      publish_operate_feedback(
        goal_handle,
        OperateCabinetControl::Feedback::WAITING_FOR_SYSTEM,
        0.02F, 0.0, "Waiting for a fresh physical control state.");
      wait_for_fresh_button_state(goal_handle, *control);
      check_cancel(goal_handle);
      const auto initial_state = button_snapshot(*control);
      result->initial_position = initial_state.position;
      std::tie(target_position, target_state) = resolve_operation_target(
        *control, *goal_handle->get_goal(), initial_state);
      interruptible_hold(goal_handle, planning_scene_settle_seconds_);

      const bool navigate =
        goal_handle->get_goal()->navigate_to_staging_pose;
      const bool is_button = control->control_type ==
        xczs_inspection_robot_control::msg::CabinetControl::TYPE_BUTTON;
      if (is_button) {
        button_poses = calculate_operation_poses(*control, navigate);
      } else {
        rotary_poses = calculate_rotary_operation_poses(
          *control, initial_state.position, navigate);
      }
      const auto & staging_pose = is_button ?
        button_poses.staging_pose : rotary_poses.staging_pose;
      const auto & staging_pose_in_planning_frame = is_button ?
        button_poses.staging_pose_in_planning_frame :
        rotary_poses.staging_pose_in_planning_frame;
      if (navigate) {
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::NAVIGATING,
          0.07F, target_position,
          "Driving the base to the cabinet staging pose.");
        navigate_to_staging_pose(goal_handle, staging_pose);
      }
      set_navigation_mode(goal_handle, false);
      if (navigate) {
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::DOCKING,
          0.18F, target_position,
          "Refining the staging pose from odometry.");
        dock_to_staging_pose(goal_handle, staging_pose_in_planning_frame);
      }

      move_group = std::make_shared<MoveGroupInterface>(
        shared_from_this(), kArmGroup,
        std::shared_ptr<tf2_ros::Buffer>(),
        rclcpp::Duration::from_seconds(system_wait_timeout_));
      {
        std::lock_guard<std::mutex> lock(motion_mutex_);
        active_move_group_ = move_group;
      }
      configure_move_group(*move_group);
      if (!move_group->getCurrentState(system_wait_timeout_)) {
        throw GenericOperationError(
                OperateCabinetControl::Result::NOT_READY,
                "MoveIt did not receive the current robot state.");
      }

      if (is_button) {
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::MOVING_TO_READY,
          0.25F, target_position,
          "Planning to the button ready pose.");
        plan_and_execute_pose(
          *move_group, goal_handle, button_poses.prepress_pose);
        should_attempt_retreat = true;
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::APPROACHING,
          0.47F, target_position,
          "Approaching the button along its travel axis.");
        execute_cartesian_path(
          *move_group, goal_handle, {button_poses.contact_pose},
          cartesian_velocity_scale_, cartesian_acceleration_scale_);
        const auto transition_sequence =
          button_snapshot(*control).pressed_transition_sequence;
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::MANIPULATING,
          0.65F, target_position,
          "Pressing the button through its physical travel.");
        execute_cartesian_path(
          *move_group, goal_handle, {button_poses.pressed_pose},
          cartesian_velocity_scale_ * 0.5,
          cartesian_acceleration_scale_ * 0.5);
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::VERIFYING,
          0.77F, target_position,
          "Verifying the Gazebo press transition.");
        if (!wait_for_pressed_state(
            goal_handle, *control, true, press_detection_timeout_,
            transition_sequence))
        {
          throw GenericOperationError(
                  OperateCabinetControl::Result::TARGET_NOT_REACHED,
                  "The button did not cross its press threshold.");
        }
        interruptible_hold(goal_handle, press_hold_seconds_);
        const auto release_sequence =
          button_snapshot(*control).pressed_transition_sequence;
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::RETREATING,
          0.88F, 0.0, "Retracting the probe from the button.");
        execute_cartesian_path(
          *move_group, goal_handle,
          {button_poses.contact_pose, button_poses.prepress_pose},
          cartesian_velocity_scale_, cartesian_acceleration_scale_);
        should_attempt_retreat = false;
        if (!wait_for_pressed_state(
            goal_handle, *control, false, release_detection_timeout_,
            release_sequence))
        {
          throw GenericOperationError(
                  OperateCabinetControl::Result::RELEASE_FAILED,
                  "The button did not return to its released state.");
        }
      } else {
        if (!move_group->setEndEffectorLink(kContactToolLink)) {
          throw GenericOperationError(
                  OperateCabinetControl::Result::NOT_READY,
                  "MoveIt does not contain the button_press_tip tool link.");
        }
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::MOVING_TO_READY,
          0.25F, target_position,
          "Planning the probe to the control ready pose.");
        plan_and_execute_pose(
          *move_group, goal_handle, rotary_poses.ready_pose,
          kContactToolLink);
        should_attempt_retreat = true;
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::APPROACHING,
          0.43F, target_position,
          "Approaching the physical grasp point.");
        execute_cartesian_path(
          *move_group, goal_handle, {rotary_poses.grasp_pose},
          cartesian_velocity_scale_ * 0.5,
          cartesian_acceleration_scale_ * 0.5);
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::GRASPING,
          0.52F, target_position,
          "Attaching the probe at the verified near-distance grasp point.");
        set_control_grasp(goal_handle, control->id, true);
        grasp_attached = true;
        const auto waypoints = calculate_rotation_waypoints(
          *control, initial_state.position, target_position);
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::MANIPULATING,
          0.62F, target_position,
          "Following the control joint arc without commanding the joint.");
        execute_cartesian_path(
          *move_group, goal_handle, waypoints,
          cartesian_velocity_scale_ * 0.5,
          cartesian_acceleration_scale_ * 0.5);
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::VERIFYING,
          0.80F, target_position,
          "Verifying target position and low-velocity stability.");
        wait_for_target_stable(
          goal_handle, *control, target_position, target_state);
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::RELEASING,
          0.88F, target_position, "Releasing the physical grasp.");
        set_control_grasp(goal_handle, control->id, false);
        grasp_attached = false;
        const auto target_ready = calculate_rotary_tool_pose(
          *control, target_position, prepress_distance_);
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::RETREATING,
          0.94F, target_position, "Retreating from the operated control.");
        execute_cartesian_path(
          *move_group, goal_handle, {target_ready},
          cartesian_velocity_scale_, cartesian_acceleration_scale_);
        should_attempt_retreat = false;
      }

      const auto final_state = button_snapshot(*control);
      result->success = true;
      result->error_code = OperateCabinetControl::Result::SUCCESS;
      result->message = "Operated " + control->id + " successfully.";
      result->final_position = final_state.position;
      result->peak_position = final_state.max_position;
      result->final_state = final_state.state_id;
      finish_operate_goal_noexcept(goal_handle, result, true);
    } catch (const GenericOperationError & error) {
      result->success = false;
      result->error_code = error.error_code;
      result->message = error.what();
      finish_failed_operate_goal(
        goal_handle, result, control, move_group, should_attempt_retreat,
        grasp_attached, is_operate_goal_canceling(goal_handle));
    } catch (const OperationError & error) {
      result->success = false;
      result->error_code = map_legacy_error_code(error.error_code);
      result->message = error.what();
      finish_failed_operate_goal(
        goal_handle, result, control, move_group, should_attempt_retreat,
        grasp_attached, is_operate_goal_canceling(goal_handle));
    } catch (const std::exception & error) {
      result->success = false;
      result->error_code = is_operate_goal_canceling(goal_handle) ?
        OperateCabinetControl::Result::CANCELED :
        OperateCabinetControl::Result::INTERNAL_ERROR;
      result->message = is_operate_goal_canceling(goal_handle) ?
        "Cabinet operation was canceled." :
        std::string("Cabinet operation failed: ") + error.what();
      finish_failed_operate_goal(
        goal_handle, result, control, move_group, should_attempt_retreat,
        grasp_attached, is_operate_goal_canceling(goal_handle));
    } catch (...) {
      result->success = false;
      result->error_code = OperateCabinetControl::Result::INTERNAL_ERROR;
      result->message = "Cabinet operation failed with an unknown error.";
      finish_failed_operate_goal(
        goal_handle, result, control, move_group, should_attempt_retreat,
        grasp_attached, is_operate_goal_canceling(goal_handle));
    }

    stop_active_motion();
    cancel_active_navigation();
    request_navigation_mode_without_wait(false);
    {
      std::lock_guard<std::mutex> lock(motion_mutex_);
      active_move_group_.reset();
    }
    operation_active_.store(false);
    publish_active_control("");
  }

  std::pair<double, std::string> resolve_operation_target(
    const ButtonSpec & control,
    const OperateCabinetControl::Goal & goal,
    const ButtonSnapshot & current) const
  {
    if (control.control_type ==
      xczs_inspection_robot_control::msg::CabinetControl::TYPE_BUTTON)
    {
      return {
        control.state_positions.back(), control.state_ids.back()};
    }
    if (goal.command == OperateCabinetControl::Goal::COMMAND_SET_STATE) {
      const auto iterator = std::find(
        control.state_ids.begin(), control.state_ids.end(), goal.target_state);
      if (iterator == control.state_ids.end()) {
        throw GenericOperationError(
                OperateCabinetControl::Result::UNSUPPORTED_COMMAND,
                "Unknown target state '" + goal.target_state + "'.");
      }
      const auto index = static_cast<std::size_t>(
        std::distance(control.state_ids.begin(), iterator));
      return {control.state_positions[index], control.state_ids[index]};
    }
    if (goal.command == OperateCabinetControl::Goal::COMMAND_SET_POSITION) {
      if (!goal.use_target_position || !std::isfinite(goal.target_position) ||
        goal.target_position < control.min_position ||
        goal.target_position > control.max_position)
      {
        throw GenericOperationError(
                OperateCabinetControl::Result::UNSUPPORTED_COMMAND,
                "Target position is outside the configured control limits.");
      }
      std::string closest_state;
      double closest_error = std::numeric_limits<double>::infinity();
      for (std::size_t index = 0; index < control.state_positions.size();
        ++index)
      {
        const double error = std::abs(
          goal.target_position - control.state_positions[index]);
        if (error < closest_error) {
          closest_error = error;
          closest_state = control.state_ids[index];
        }
      }
      if (closest_error > target_tolerance_) {
        throw GenericOperationError(
                OperateCabinetControl::Result::UNSUPPORTED_COMMAND,
                "Only configured physical detent positions are supported.");
      }
      const auto iterator = std::find(
        control.state_ids.begin(), control.state_ids.end(), closest_state);
      const auto index = static_cast<std::size_t>(
        std::distance(control.state_ids.begin(), iterator));
      return {control.state_positions[index], closest_state};
    }
    if (goal.command == OperateCabinetControl::Goal::COMMAND_TOGGLE) {
      std::size_t current_index = 0U;
      const auto state_iterator = std::find(
        control.state_ids.begin(), control.state_ids.end(), current.state_id);
      if (state_iterator != control.state_ids.end()) {
        current_index = static_cast<std::size_t>(
          std::distance(control.state_ids.begin(), state_iterator));
      } else {
        double closest_error = std::numeric_limits<double>::infinity();
        for (std::size_t index = 0; index < control.state_positions.size();
          ++index)
        {
          const double error = std::abs(
            current.position - control.state_positions[index]);
          if (error < closest_error) {
            closest_error = error;
            current_index = index;
          }
        }
      }
      const std::size_t target_index =
        (current_index + 1U) % control.state_positions.size();
      return {
        control.state_positions[target_index],
        control.state_ids[target_index]};
    }
    throw GenericOperationError(
            OperateCabinetControl::Result::UNSUPPORTED_COMMAND,
            "The requested command is not supported by this control.");
  }

  tf2::Transform resolve_cabinet_transform()
  {
    try {
      const auto transform = transform_buffer_->lookupTransform(
        planning_frame_, cabinet_frame_, tf2::TimePointZero, 200ms);
      tf2::Transform result;
      tf2::fromMsg(transform.transform, result);
      return result;
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Cabinet truth TF is unavailable; using launch-parameter fallback: %s",
        error.what());
    }
    tf2::Quaternion rotation;
    rotation.setRPY(cabinet_roll_, cabinet_pitch_, cabinet_yaw_);
    rotation.normalize();
    return tf2::Transform(
      rotation, tf2::Vector3(cabinet_x_, cabinet_y_, cabinet_z_));
  }

  tf2::Transform resolve_control_parent_transform(
    const ButtonSpec & control) const
  {
    std::vector<const ButtonSpec *> ancestors;
    std::string parent_id = control.parent_control_id;
    while (!parent_id.empty()) {
      const auto iterator = buttons_by_id_.find(parent_id);
      if (iterator == buttons_by_id_.end()) {
        throw GenericOperationError(
                OperateCabinetControl::Result::INTERNAL_ERROR,
                "Control '" + control.id + "' has an unresolved parent '" +
                parent_id + "'.");
      }
      ancestors.push_back(iterator->second.get());
      parent_id = iterator->second->parent_control_id;
    }
    std::reverse(ancestors.begin(), ancestors.end());

    tf2::Transform result;
    result.setIdentity();
    const auto now = std::chrono::steady_clock::now();
    for (const auto * ancestor : ancestors) {
      const auto state = button_snapshot(*ancestor);
      const bool fresh = state.received && state.valid &&
        std::chrono::duration<double>(now - state.received_at).count() <=
        button_state_timeout_;
      if (!fresh) {
        throw GenericOperationError(
                OperateCabinetControl::Result::NOT_READY,
                "Parent control '" + ancestor->id +
                "' does not have a fresh valid physical state.");
      }
      if (state.in_motion ||
        std::abs(state.velocity) > target_velocity_tolerance_)
      {
        throw GenericOperationError(
                OperateCabinetControl::Result::NOT_READY,
                "Parent control '" + ancestor->id +
                "' is moving; wait for it to become stable.");
      }

      tf2::Quaternion rotation;
      rotation.setRotation(ancestor->axis, state.position);
      rotation.normalize();
      const tf2::Vector3 pivot(
        ancestor->pivot_x, ancestor->pivot_y, ancestor->pivot_z);
      tf2::Transform joint_motion;
      joint_motion.setRotation(rotation);
      joint_motion.setOrigin(pivot - tf2::quatRotate(rotation, pivot));
      result *= joint_motion;
    }
    return result;
  }

  ResolvedControlGeometry resolve_control_geometry(
    const ButtonSpec & control) const
  {
    const tf2::Transform parent = resolve_control_parent_transform(control);
    ResolvedControlGeometry geometry{
      parent * tf2::Vector3(
        control.local_x, control.local_y, control.local_z),
      parent * tf2::Vector3(
        control.pivot_x, control.pivot_y, control.pivot_z),
      tf2::quatRotate(parent.getRotation(), control.axis),
      tf2::quatRotate(parent.getRotation(), control.approach_normal)};
    geometry.axis.normalize();
    geometry.approach_normal.normalize();
    return geometry;
  }

  tf2::Quaternion tool_rotation_from_outward(
    const tf2::Vector3 & outward) const
  {
    tf2::Vector3 tool_z = outward.normalized();
    tf2::Vector3 tool_x(0.0, 0.0, 1.0);
    tool_x -= tool_z * tool_z.dot(tool_x);
    if (tool_x.length2() < 1.0e-6) {
      tool_x = tf2::Vector3(1.0, 0.0, 0.0);
      tool_x -= tool_z * tool_z.dot(tool_x);
    }
    tool_x.normalize();
    tf2::Vector3 tool_y = tool_z.cross(tool_x);
    tool_y.normalize();
    const tf2::Matrix3x3 basis(
      tool_x.x(), tool_y.x(), tool_z.x(),
      tool_x.y(), tool_y.y(), tool_z.y(),
      tool_x.z(), tool_y.z(), tool_z.z());
    tf2::Quaternion rotation;
    basis.getRotation(rotation);
    rotation.normalize();
    return rotation;
  }

  geometry_msgs::msg::Pose calculate_rotary_tool_pose(
    const ButtonSpec & control,
    double position,
    double outward_offset)
  {
    const tf2::Transform cabinet = resolve_cabinet_transform();
    const auto geometry = resolve_control_geometry(control);
    tf2::Quaternion local_rotation;
    local_rotation.setRotation(geometry.axis, position);
    local_rotation.normalize();
    const tf2::Vector3 local_grasp = geometry.pivot +
      tf2::quatRotate(
      local_rotation, geometry.grasp_zero - geometry.pivot);
    const tf2::Vector3 grasp = cabinet * local_grasp;
    const tf2::Vector3 outward_zero = tf2::quatRotate(
      cabinet.getRotation(), geometry.approach_normal);
    const tf2::Vector3 world_axis = tf2::quatRotate(
      cabinet.getRotation(), geometry.axis);
    tf2::Quaternion world_rotation;
    world_rotation.setRotation(world_axis, position);
    world_rotation.normalize();
    const tf2::Vector3 outward = tf2::quatRotate(
      world_rotation, outward_zero).normalized();
    const tf2::Quaternion tool_zero =
      tool_rotation_from_outward(outward_zero);
    tf2::Quaternion tool_rotation = world_rotation * tool_zero;
    tool_rotation.normalize();

    geometry_msgs::msg::Pose pose;
    const tf2::Vector3 position_world = grasp + outward * outward_offset;
    pose.position.x = position_world.x();
    pose.position.y = position_world.y();
    pose.position.z = position_world.z();
    pose.orientation = to_message(tool_rotation);
    return pose;
  }

  RotaryOperationPoses calculate_rotary_operation_poses(
    const ButtonSpec & control,
    double position,
    bool include_staging_pose)
  {
    RotaryOperationPoses poses;
    poses.ready_pose = calculate_rotary_tool_pose(
      control, position, prepress_distance_);
    poses.grasp_pose = calculate_rotary_tool_pose(control, position, 0.0);
    if (!include_staging_pose) {
      return poses;
    }

    const tf2::Transform cabinet = resolve_cabinet_transform();
    const auto geometry = resolve_control_geometry(control);
    tf2::Quaternion local_rotation;
    local_rotation.setRotation(geometry.axis, position);
    local_rotation.normalize();
    const tf2::Vector3 local_grasp = geometry.pivot +
      tf2::quatRotate(
      local_rotation, geometry.grasp_zero - geometry.pivot);
    const tf2::Vector3 grasp = cabinet * local_grasp;
    const tf2::Vector3 outward_zero = tf2::quatRotate(
      cabinet.getRotation(), geometry.approach_normal);
    const tf2::Vector3 world_axis = tf2::quatRotate(
      cabinet.getRotation(), geometry.axis);
    tf2::Quaternion world_rotation;
    world_rotation.setRotation(world_axis, position);
    const tf2::Vector3 outward = tf2::quatRotate(
      world_rotation, outward_zero).normalized();
    const tf2::Vector3 inward = -outward;
    const tf2::Vector3 staging = grasp + outward * staging_distance_;

    geometry_msgs::msg::PoseStamped staging_in_planning_frame;
    staging_in_planning_frame.header.frame_id = planning_frame_;
    staging_in_planning_frame.pose.position.x = staging.x();
    staging_in_planning_frame.pose.position.y = staging.y();
    staging_in_planning_frame.pose.position.z = 0.0;
    tf2::Quaternion navigation_rotation;
    navigation_rotation.setRPY(
      0.0, 0.0, std::atan2(inward.y(), inward.x()));
    staging_in_planning_frame.pose.orientation =
      to_message(navigation_rotation);
    poses.staging_pose = staging_in_planning_frame;
    poses.staging_pose_in_planning_frame = staging_in_planning_frame;
    if (navigation_frame_ != planning_frame_) {
      try {
        poses.staging_pose = transform_buffer_->transform(
          staging_in_planning_frame, navigation_frame_,
          tf2::durationFromSec(system_wait_timeout_));
      } catch (const tf2::TransformException & error) {
        throw GenericOperationError(
                OperateCabinetControl::Result::NOT_READY,
                "Could not transform the cabinet staging pose: " +
                std::string(error.what()));
      }
    }
    return poses;
  }

  std::vector<geometry_msgs::msg::Pose> calculate_rotation_waypoints(
    const ButtonSpec & control,
    double initial_position,
    double target_position)
  {
    const double travel = target_position - initial_position;
    const std::size_t count = std::max<std::size_t>(
      1U, static_cast<std::size_t>(
        std::ceil(std::abs(travel) / rotation_waypoint_step_)));
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.reserve(count);
    for (std::size_t index = 1U; index <= count; ++index) {
      const double ratio = static_cast<double>(index) /
        static_cast<double>(count);
      waypoints.push_back(calculate_rotary_tool_pose(
        control, initial_position + travel * ratio, 0.0));
    }
    return waypoints;
  }

  template<typename GoalHandleT>
  void set_control_grasp(
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const std::string & control_id,
    bool attach)
  {
    if (!grasp_client_->wait_for_service(
        std::chrono::duration<double>(system_wait_timeout_)))
    {
      throw GenericOperationError(
              attach ? OperateCabinetControl::Result::GRASP_FAILED :
              OperateCabinetControl::Result::RELEASE_FAILED,
              "Cabinet grasp service is unavailable.");
    }
    auto request = std::make_shared<
      xczs_inspection_robot_control::srv::SetCabinetGrasp::Request>();
    request->control_id = control_id;
    request->robot_model = kRobotModelName;
    request->robot_link = kContactToolLink;
    request->attach = attach;
    auto future = grasp_client_->async_send_request(request);
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(system_wait_timeout_);
    while (future.wait_for(50ms) != std::future_status::ready) {
      check_cancel(goal_handle);
      if (std::chrono::steady_clock::now() >= deadline) {
        throw GenericOperationError(
                attach ? OperateCabinetControl::Result::GRASP_FAILED :
                OperateCabinetControl::Result::RELEASE_FAILED,
                "Cabinet grasp request timed out.");
      }
    }
    const auto response = future.get();
    if (!response->success) {
      throw GenericOperationError(
              attach ? OperateCabinetControl::Result::GRASP_FAILED :
              OperateCabinetControl::Result::RELEASE_FAILED,
              response->message + " (distance " +
              std::to_string(response->distance) + " m)");
    }
  }

  void release_control_grasp_noexcept(const std::string & control_id) noexcept
  {
    try {
      if (!grasp_client_->service_is_ready()) {
        return;
      }
      auto request = std::make_shared<
        xczs_inspection_robot_control::srv::SetCabinetGrasp::Request>();
      request->control_id = control_id;
      request->robot_model = kRobotModelName;
      request->robot_link = kContactToolLink;
      request->attach = false;
      auto future = grasp_client_->async_send_request(request);
      if (future.wait_for(2s) == std::future_status::ready) {
        const auto response = future.get();
        if (!response->success) {
          RCLCPP_ERROR(
            get_logger(), "Emergency grasp release failed: %s",
            response->message.c_str());
        }
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        get_logger(), "Emergency grasp release failed: %s", error.what());
    }
  }

  template<typename GoalHandleT>
  void wait_for_target_stable(
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const ButtonSpec & control,
    double target_position,
    const std::string & target_state)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(system_wait_timeout_);
    auto stable_since = std::chrono::steady_clock::time_point{};
    while (std::chrono::steady_clock::now() < deadline) {
      check_cancel(goal_handle);
      {
        std::unique_lock<std::mutex> lock(control.runtime->mutex);
        const auto now = std::chrono::steady_clock::now();
        const auto & state = control.runtime->state;
        const bool fresh = state.received && state.valid &&
          std::chrono::duration<double>(now - state.received_at).count() <=
          button_state_timeout_;
        const bool at_position = fresh &&
          std::abs(state.position - target_position) <= target_tolerance_;
        const bool state_matches = target_state.empty() ||
          state.state_id == target_state;
        const bool stopped = std::abs(state.velocity) <=
          stable_velocity_tolerance_;
        if (at_position && state_matches && stopped) {
          if (stable_since == std::chrono::steady_clock::time_point{}) {
            stable_since = now;
          }
          if (std::chrono::duration<double>(now - stable_since).count() >=
            stable_state_duration_)
          {
            return;
          }
        } else {
          stable_since = std::chrono::steady_clock::time_point{};
        }
        control.runtime->condition.wait_for(lock, 50ms);
      }
    }
    throw GenericOperationError(
            OperateCabinetControl::Result::TARGET_NOT_REACHED,
            control.id + " did not settle at the requested physical state.");
  }

  static std::uint8_t map_legacy_error_code(std::uint8_t error_code)
  {
    switch (error_code) {
      case PressCabinetButton::Result::INVALID_BUTTON:
        return OperateCabinetControl::Result::INVALID_CONTROL;
      case PressCabinetButton::Result::NOT_READY:
        return OperateCabinetControl::Result::NOT_READY;
      case PressCabinetButton::Result::NAVIGATION_FAILED:
        return OperateCabinetControl::Result::NAVIGATION_FAILED;
      case PressCabinetButton::Result::PLANNING_FAILED:
        return OperateCabinetControl::Result::PLANNING_FAILED;
      case PressCabinetButton::Result::EXECUTION_FAILED:
        return OperateCabinetControl::Result::EXECUTION_FAILED;
      case PressCabinetButton::Result::PRESS_NOT_DETECTED:
        return OperateCabinetControl::Result::TARGET_NOT_REACHED;
      case PressCabinetButton::Result::RELEASE_NOT_DETECTED:
        return OperateCabinetControl::Result::RELEASE_FAILED;
      case PressCabinetButton::Result::CANCELED:
        return OperateCabinetControl::Result::CANCELED;
      default:
        return OperateCabinetControl::Result::INTERNAL_ERROR;
    }
  }

  bool is_operate_goal_canceling(
    const std::shared_ptr<OperateGoalHandle> & goal_handle) const noexcept
  {
    try {
      return cancel_requested_.load() ||
        (goal_handle && goal_handle->is_canceling());
    } catch (...) {
      return cancel_requested_.load();
    }
  }

  bool finish_operate_goal_noexcept(
    const std::shared_ptr<OperateGoalHandle> & goal_handle,
    const std::shared_ptr<OperateCabinetControl::Result> & result,
    bool request_success) noexcept
  {
    try {
      if (!goal_handle || !goal_handle->is_active()) {
        return false;
      }
      if (goal_handle->is_canceling()) {
        result->success = false;
        result->error_code = OperateCabinetControl::Result::CANCELED;
        result->message = "Cabinet operation was canceled.";
        goal_handle->canceled(result);
        return false;
      }
      if (request_success) {
        goal_handle->succeed(result);
        return true;
      }
      goal_handle->abort(result);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        get_logger(), "Failed to finalize cabinet operation: %s",
        error.what());
    }
    return false;
  }

  void finish_failed_operate_goal(
    const std::shared_ptr<OperateGoalHandle> & goal_handle,
    const std::shared_ptr<OperateCabinetControl::Result> & result,
    const std::shared_ptr<ButtonSpec> & control,
    const std::shared_ptr<MoveGroupInterface> & move_group,
    bool should_attempt_retreat,
    bool grasp_attached,
    bool canceled) noexcept
  {
    if (control && (grasp_attached || control->requires_grasp)) {
      release_control_grasp_noexcept(control->id);
    }
    if (control && should_attempt_retreat && move_group && rclcpp::ok()) {
      try {
        const auto state = button_snapshot(*control);
        const bool is_button = control->control_type ==
          xczs_inspection_robot_control::msg::CabinetControl::TYPE_BUTTON;
        const auto retreat_pose = is_button ?
          calculate_operation_poses(*control, false).prepress_pose :
          calculate_rotary_tool_pose(
          *control, state.position, prepress_distance_);
        best_effort_retreat(*move_group, retreat_pose);
      } catch (const std::exception & error) {
        RCLCPP_ERROR(
          get_logger(), "Generic safety retreat failed: %s", error.what());
      }
    }
    if (control) {
      const auto final_state = button_snapshot(*control);
      result->final_position = final_state.position;
      result->peak_position = final_state.max_position;
      result->final_state = final_state.state_id;
    }
    if (canceled) {
      result->success = false;
      result->error_code = OperateCabinetControl::Result::CANCELED;
      result->message = "Cabinet operation was canceled.";
    }
    finish_operate_goal_noexcept(goal_handle, result, false);
    RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
  }

  void receive_button_joint_state(
    const ButtonSpec & button,
    const sensor_msgs::msg::JointState & message)
  {
    const auto iterator = std::find(
      message.name.begin(), message.name.end(), button.joint_name);
    if (iterator == message.name.end()) {
      return;
    }
    const auto index = static_cast<std::size_t>(
      std::distance(message.name.begin(), iterator));
    if (index >= message.position.size() ||
      !std::isfinite(message.position[index]))
    {
      return;
    }

    {
      std::lock_guard<std::mutex> lock(button.runtime->mutex);
      button.runtime->state.received = true;
      button.runtime->state.position = message.position[index];
      button.runtime->state.max_position = std::max(
        button.runtime->state.max_position,
        button.runtime->state.position);
      button.runtime->state.received_at = std::chrono::steady_clock::now();
    }
    button.runtime->condition.notify_all();
  }

  void receive_button_pressed(
    const ButtonSpec & button,
    const std_msgs::msg::Bool & message)
  {
    {
      std::lock_guard<std::mutex> lock(button.runtime->mutex);
      if (!button.runtime->state.pressed_received ||
        button.runtime->state.pressed != message.data)
      {
        ++button.runtime->state.pressed_transition_sequence;
      }
      button.runtime->state.pressed_received = true;
      button.runtime->state.pressed = message.data;
      button.runtime->state.pressed_received_at =
        std::chrono::steady_clock::now();
    }
    button.runtime->condition.notify_all();
  }

  void receive_control_state(
    const ButtonSpec & control,
    const xczs_inspection_robot_control::msg::CabinetControlState & message)
  {
    if (message.control_id != control.id || !message.valid ||
      !std::isfinite(message.position) ||
      !std::isfinite(message.velocity) || !std::isfinite(message.effort))
    {
      return;
    }
    {
      std::lock_guard<std::mutex> lock(control.runtime->mutex);
      auto & state = control.runtime->state;
      state.received = true;
      state.valid = true;
      state.position = message.position;
      state.velocity = message.velocity;
      state.effort = message.effort;
      state.max_position = std::max(state.max_position, state.position);
      state.in_motion = message.in_motion;
      state.state_id = message.state_id;
      state.received_at = std::chrono::steady_clock::now();
      state.pressed_received = true;
      state.pressed = message.activated;
      state.pressed_received_at = state.received_at;
      state.pressed_transition_sequence = message.transition_sequence;
    }
    control.runtime->condition.notify_all();
  }

  ButtonSnapshot button_snapshot(const ButtonSpec & button) const
  {
    std::lock_guard<std::mutex> lock(button.runtime->mutex);
    return button.runtime->state;
  }

  void reset_max_button_travel(const ButtonSpec & button)
  {
    std::lock_guard<std::mutex> lock(button.runtime->mutex);
    button.runtime->state.max_position = std::max(
      0.0, button.runtime->state.position);
  }

  template<typename GoalHandleT>
  void wait_for_fresh_button_state(
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const ButtonSpec & button)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(system_wait_timeout_);
    while (std::chrono::steady_clock::now() < deadline) {
      check_cancel(goal_handle);
      {
        std::unique_lock<std::mutex> lock(button.runtime->mutex);
        const auto now = std::chrono::steady_clock::now();
        const bool joint_state_fresh = button.runtime->state.received &&
          std::chrono::duration<double>(
          now - button.runtime->state.received_at).count() <=
          button_state_timeout_;
        const bool pressed_state_fresh =
          button.runtime->state.pressed_received &&
          std::chrono::duration<double>(
          now - button.runtime->state.pressed_received_at).count() <=
          button_state_timeout_;
        if (joint_state_fresh && pressed_state_fresh) {
          if (button.control_type ==
            xczs_inspection_robot_control::msg::CabinetControl::TYPE_BUTTON &&
            button.runtime->state.pressed)
          {
            throw OperationError(
                    PressCabinetButton::Result::NOT_READY,
                    button.id + " is already pressed; release it before " +
                    "starting an operation.");
          }
          return;
        }
        button.runtime->condition.wait_for(lock, 100ms);
      }
    }
    throw OperationError(
            PressCabinetButton::Result::NOT_READY,
            "No fresh state was received from " + button.id +
            ". Check the cabinet model plugin and joint-state topic.");
  }

  template<typename GoalHandleT>
  bool wait_for_pressed_state(
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const ButtonSpec & button,
    bool expected_pressed,
    double timeout_seconds,
    std::uint64_t previous_transition_sequence)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(timeout_seconds);
    while (std::chrono::steady_clock::now() < deadline) {
      check_cancel(goal_handle);
      {
        std::unique_lock<std::mutex> lock(button.runtime->mutex);
        if (button.runtime->state.pressed_received &&
          button.runtime->state.pressed == expected_pressed &&
          button.runtime->state.pressed_transition_sequence >
          previous_transition_sequence)
        {
          return true;
        }
        button.runtime->condition.wait_for(lock, 50ms);
      }
    }
    return false;
  }

  OperationPoses calculate_operation_poses(
    const ButtonSpec & button,
    bool include_staging_pose)
  {
    const tf2::Transform cabinet_transform = resolve_cabinet_transform();
    const tf2::Quaternion cabinet_rotation = cabinet_transform.getRotation();
    const auto geometry = resolve_control_geometry(button);
    const tf2::Vector3 button_face =
      cabinet_transform * geometry.grasp_zero;
    tf2::Vector3 outward = tf2::quatRotate(
      cabinet_rotation, geometry.approach_normal);
    outward.normalize();
    const tf2::Vector3 inward = -outward;

    // Keep tool +X vertical and tool -Z aligned with the press direction.
    tf2::Vector3 tool_z = outward;
    tf2::Vector3 tool_x(0.0, 0.0, 1.0);
    tool_x -= tool_z * tool_z.dot(tool_x);
    if (tool_x.length2() < 1.0e-6) {
      throw OperationError(
              PressCabinetButton::Result::INTERNAL_ERROR,
              "Cabinet panel normal is parallel to the world vertical axis.");
    }
    tool_x.normalize();
    tf2::Vector3 tool_y = tool_z.cross(tool_x);
    tool_y.normalize();
    const tf2::Matrix3x3 tool_basis(
      tool_x.x(), tool_y.x(), tool_z.x(),
      tool_x.y(), tool_y.y(), tool_z.y(),
      tool_x.z(), tool_y.z(), tool_z.z());
    tf2::Quaternion tool_rotation;
    tool_basis.getRotation(tool_rotation);
    tool_rotation.normalize();
    const tf2::Vector3 end_to_tip = tf2::quatRotate(
      tool_rotation, tf2::Vector3(0.0, 0.0, -tool_tip_offset_));

    const auto make_end_pose =
      [&](double tip_offset) -> geometry_msgs::msg::Pose {
        const tf2::Vector3 tip_position = button_face +
          inward * tip_offset;
        const tf2::Vector3 end_position = tip_position - end_to_tip;
        geometry_msgs::msg::Pose pose;
        pose.position.x = end_position.x();
        pose.position.y = end_position.y();
        pose.position.z = end_position.z();
        pose.orientation = to_message(tool_rotation);
        return pose;
      };

    OperationPoses poses;
    poses.prepress_pose = make_end_pose(-prepress_distance_);
    poses.contact_pose = make_end_pose(-contact_clearance_);
    poses.pressed_pose = make_end_pose(press_depth_);

    if (include_staging_pose) {
      geometry_msgs::msg::PoseStamped staging_in_planning_frame;
      staging_in_planning_frame.header.frame_id = planning_frame_;
      const tf2::Vector3 staging_position = button_face -
        inward * staging_distance_;
      staging_in_planning_frame.pose.position.x = staging_position.x();
      staging_in_planning_frame.pose.position.y = staging_position.y();
      staging_in_planning_frame.pose.position.z = 0.0;
      const double navigation_yaw = std::atan2(inward.y(), inward.x());
      tf2::Quaternion navigation_rotation;
      navigation_rotation.setRPY(0.0, 0.0, navigation_yaw);
      poses.staging_pose = staging_in_planning_frame;
      poses.staging_pose_in_planning_frame = staging_in_planning_frame;
      poses.staging_pose.pose.orientation = to_message(navigation_rotation);
      poses.staging_pose_in_planning_frame.pose.orientation =
        to_message(navigation_rotation);

      if (navigation_frame_ != planning_frame_) {
        try {
          poses.staging_pose = transform_buffer_->transform(
            staging_in_planning_frame,
            navigation_frame_,
            tf2::durationFromSec(system_wait_timeout_));
        } catch (const tf2::TransformException & error) {
          throw OperationError(
                  PressCabinetButton::Result::NOT_READY,
                  "Could not transform the cabinet staging pose from " +
                  planning_frame_ + " to " + navigation_frame_ + ": " +
                  error.what());
        }
      }
    }
    return poses;
  }

  void configure_move_group(MoveGroupInterface & move_group)
  {
    move_group.setPoseReferenceFrame(planning_frame_);
    move_group.setPlanningTime(10.0);
    move_group.setNumPlanningAttempts(10);
    move_group.setMaxVelocityScalingFactor(0.20);
    move_group.setMaxAccelerationScalingFactor(0.20);
    move_group.setGoalPositionTolerance(0.005);
    move_group.setGoalOrientationTolerance(0.01);
    move_group.setGoalJointTolerance(0.001);
    move_group.allowReplanning(true);
  }

  template<typename GoalHandleT>
  void plan_and_execute_pose(
    MoveGroupInterface & move_group,
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const geometry_msgs::msg::Pose & target,
    const std::string & tool_link = kArmTipLink)
  {
    check_cancel(goal_handle);
    move_group.setStartStateToCurrentState();
    if (!move_group.setPoseTarget(target, tool_link)) {
      throw OperationError(
              PressCabinetButton::Result::PLANNING_FAILED,
              "MoveIt rejected the cabinet prepress pose target.");
    }

    MoveGroupInterface::Plan plan;
    const auto planning_result = move_group.plan(plan);
    move_group.clearPoseTargets();
    if (planning_result != moveit::core::MoveItErrorCode::SUCCESS) {
      throw OperationError(
              PressCabinetButton::Result::PLANNING_FAILED,
              "MoveIt could not plan to the prepress pose. If navigation "
              "was skipped, first park the base in front of the cabinet.");
    }
    check_cancel(goal_handle);
    if (move_group.execute(plan) !=
      moveit::core::MoveItErrorCode::SUCCESS)
    {
      check_cancel(goal_handle);
      throw OperationError(
              PressCabinetButton::Result::EXECUTION_FAILED,
              "MoveIt failed to execute the prepress trajectory.");
    }
    check_cancel(goal_handle);
  }

  template<typename GoalHandleT>
  void execute_cartesian_path(
    MoveGroupInterface & move_group,
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const std::vector<geometry_msgs::msg::Pose> & waypoints,
    double velocity_scale,
    double acceleration_scale)
  {
    check_cancel(goal_handle);
    move_group.setStartStateToCurrentState();
    moveit_msgs::msg::RobotTrajectory trajectory_message;
    const double fraction = move_group.computeCartesianPath(
      waypoints,
      0.002,
      0.0,
      trajectory_message,
      true);
    if (fraction < 0.99) {
      throw OperationError(
              PressCabinetButton::Result::PLANNING_FAILED,
              "MoveIt completed only " + std::to_string(fraction * 100.0) +
              "% of the required Cartesian button path.");
    }
    retime_cartesian_trajectory(
      move_group,
      trajectory_message,
      velocity_scale,
      acceleration_scale);
    check_cancel(goal_handle);
    if (move_group.execute(trajectory_message) !=
      moveit::core::MoveItErrorCode::SUCCESS)
    {
      check_cancel(goal_handle);
      throw OperationError(
              PressCabinetButton::Result::EXECUTION_FAILED,
              "MoveIt failed to execute a Cartesian button trajectory.");
    }
    check_cancel(goal_handle);
  }

  void retime_cartesian_trajectory(
    MoveGroupInterface & move_group,
    moveit_msgs::msg::RobotTrajectory & trajectory_message,
    double velocity_scale,
    double acceleration_scale)
  {
    const auto current_state = move_group.getCurrentState(2.0);
    if (!current_state) {
      throw OperationError(
              PressCabinetButton::Result::NOT_READY,
              "MoveIt lost the current robot state while retiming the path.");
    }
    robot_trajectory::RobotTrajectory trajectory(
      move_group.getRobotModel(), kArmGroup);
    trajectory.setRobotTrajectoryMsg(*current_state, trajectory_message);
    trajectory_processing::TimeOptimalTrajectoryGeneration time_parameterizer;
    if (!time_parameterizer.computeTimeStamps(
        trajectory,
        velocity_scale,
        acceleration_scale))
    {
      throw OperationError(
              PressCabinetButton::Result::PLANNING_FAILED,
              "MoveIt could not generate a low-speed Cartesian trajectory.");
    }
    trajectory.getRobotTrajectoryMsg(trajectory_message);
  }

  void best_effort_retreat(
    MoveGroupInterface & move_group,
    const geometry_msgs::msg::Pose & prepress_pose) noexcept
  {
    try {
      move_group.stop();
      move_group.setStartStateToCurrentState();
      moveit_msgs::msg::RobotTrajectory trajectory_message;
      const double fraction = move_group.computeCartesianPath(
        {prepress_pose},
        0.002,
        0.0,
        trajectory_message,
        true);
      if (fraction < 0.99) {
        RCLCPP_ERROR(
          get_logger(), "Safety retreat path was only %.1f%% complete.",
          fraction * 100.0);
        return;
      }
      retime_cartesian_trajectory(
        move_group,
        trajectory_message,
        cartesian_velocity_scale_,
        cartesian_acceleration_scale_);
      if (move_group.execute(trajectory_message) !=
        moveit::core::MoveItErrorCode::SUCCESS)
      {
        RCLCPP_ERROR(get_logger(), "Safety retreat execution failed.");
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        get_logger(), "Safety retreat failed: %s", error.what());
    }
  }

  template<typename GoalHandleT>
  void navigate_to_staging_pose(
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const geometry_msgs::msg::PoseStamped & staging_pose)
  {
    if (!navigation_client_->wait_for_action_server(
        std::chrono::duration<double>(system_wait_timeout_)))
    {
      throw OperationError(
              PressCabinetButton::Result::NOT_READY,
              "Nav2 NavigateToPose action server is unavailable.");
    }
    set_navigation_mode(goal_handle, true);

    NavigateToPose::Goal navigation_goal;
    navigation_goal.pose = staging_pose;
    navigation_goal.pose.header.stamp = get_clock()->now();
    rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
    const auto request_abandoned = std::make_shared<std::atomic<bool>>(false);
    const auto navigation_started =
      std::make_shared<std::atomic<bool>>(false);
    const auto takeover_requested =
      std::make_shared<std::atomic<bool>>(false);
    options.goal_response_callback =
      [this, request_abandoned](NavigationGoalHandle::SharedPtr goal_handle) {
        if (!goal_handle) {
          return;
        }
        bool must_cancel = false;
        {
          std::lock_guard<std::mutex> lock(navigation_mutex_);
          must_cancel = request_abandoned->load() ||
            cancel_requested_.load();
          if (!must_cancel) {
            active_navigation_goal_ = goal_handle;
          }
        }
        if (must_cancel) {
          navigation_client_->async_cancel_goal(goal_handle);
        }
      };
    options.feedback_callback =
      [this, weak_goal = std::weak_ptr<GoalHandleT>(goal_handle),
        navigation_started, takeover_requested](
      NavigationGoalHandle::SharedPtr,
      const std::shared_ptr<const NavigateToPose::Feedback> feedback)
      {
        const double distance_remaining = feedback->distance_remaining;
        if (std::isfinite(distance_remaining)) {
          if (distance_remaining > navigation_takeover_distance_) {
            navigation_started->store(true);
          } else if (navigation_started->load()) {
            takeover_requested->store(true);
          }
        }

        const auto now = std::chrono::steady_clock::now();
        {
          std::lock_guard<std::mutex> lock(navigation_feedback_mutex_);
          if (now - last_navigation_feedback_ < 500ms) {
            return;
          }
          last_navigation_feedback_ = now;
        }
        if (const auto operation_goal = weak_goal.lock()) {
          publish_feedback(
            operation_goal,
            PressCabinetButton::Feedback::NAVIGATING,
            0.12F,
            "Nav2 distance remaining: " +
            std::to_string(distance_remaining) + " m.");
        }
      };
    auto goal_future = navigation_client_->async_send_goal(
      navigation_goal, options);
    try {
      wait_for_future(
        goal_future,
        goal_handle,
        system_wait_timeout_,
        PressCabinetButton::Result::NAVIGATION_FAILED,
        "Nav2 did not accept the staging goal in time.");
    } catch (...) {
      request_abandoned->store(true);
      cancel_active_navigation();
      throw;
    }
    const auto navigation_goal_handle = goal_future.get();
    if (!navigation_goal_handle) {
      throw OperationError(
              PressCabinetButton::Result::NAVIGATION_FAILED,
              "Nav2 rejected the cabinet staging goal.");
    }
    bool must_cancel_navigation = false;
    {
      std::lock_guard<std::mutex> lock(navigation_mutex_);
      must_cancel_navigation = cancel_requested_.load() ||
        goal_handle->is_canceling();
      if (!must_cancel_navigation) {
        active_navigation_goal_ = navigation_goal_handle;
      }
    }
    if (must_cancel_navigation) {
      navigation_client_->async_cancel_goal(navigation_goal_handle);
      check_cancel(goal_handle);
    }

    auto result_future = navigation_client_->async_get_result(
      navigation_goal_handle);
    bool precision_takeover = false;
    const auto navigation_deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(navigation_timeout_);
    while (result_future.wait_for(100ms) != std::future_status::ready) {
      check_cancel(goal_handle);
      if (takeover_requested->load()) {
        precision_takeover = true;
        auto cancel_future = navigation_client_->async_cancel_goal(
          navigation_goal_handle);
        wait_for_future(
          cancel_future,
          goal_handle,
          system_wait_timeout_,
          PressCabinetButton::Result::NAVIGATION_FAILED,
          "Nav2 did not acknowledge the precision-docking takeover.");
        break;
      }
      if (std::chrono::steady_clock::now() >= navigation_deadline) {
        throw OperationError(
                PressCabinetButton::Result::NAVIGATION_FAILED,
                "Nav2 timed out while driving to the cabinet.");
      }
    }
    if (precision_takeover) {
      wait_for_future(
        result_future,
        goal_handle,
        system_wait_timeout_,
        PressCabinetButton::Result::NAVIGATION_FAILED,
        "Nav2 did not stop for the precision-docking takeover.");
    }
    const auto wrapped_result = result_future.get();
    {
      std::lock_guard<std::mutex> lock(navigation_mutex_);
      if (active_navigation_goal_ == navigation_goal_handle) {
        active_navigation_goal_.reset();
      }
    }
    check_cancel(goal_handle);
    if (precision_takeover &&
      (wrapped_result.code == rclcpp_action::ResultCode::CANCELED ||
      wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED))
    {
      publish_feedback(
        goal_handle,
        PressCabinetButton::Feedback::NAVIGATING,
        0.16F,
        "Nav2 coarse staging complete; switching to odometry docking.");
      return;
    }
    if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      throw OperationError(
              PressCabinetButton::Result::NAVIGATION_FAILED,
              "Nav2 did not reach the cabinet staging pose.");
    }
  }

  template<typename GoalHandleT>
  void dock_to_staging_pose(
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const geometry_msgs::msg::PoseStamped & target)
  {
    tf2::Quaternion target_rotation;
    tf2::fromMsg(target.pose.orientation, target_rotation);
    const double target_body_yaw =
      tf2::getYaw(target_rotation) - base_link_yaw_offset_;
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(docking_timeout_);
    auto last_feedback = std::chrono::steady_clock::time_point{};
    int settled_cycles = 0;

    try {
      while (std::chrono::steady_clock::now() < deadline) {
        check_cancel(goal_handle);
        geometry_msgs::msg::TransformStamped current_transform;
        try {
          current_transform = transform_buffer_->lookupTransform(
            planning_frame_, "body", tf2::TimePointZero);
        } catch (const tf2::TransformException & error) {
          throw OperationError(
                  PressCabinetButton::Result::NOT_READY,
                  "Could not read the robot odometry for precision "
                  "docking: " + std::string(error.what()));
        }

        tf2::Quaternion current_rotation;
        tf2::fromMsg(
          current_transform.transform.rotation, current_rotation);
        const double current_yaw = tf2::getYaw(current_rotation);
        const double error_x = target.pose.position.x -
          current_transform.transform.translation.x;
        const double error_y = target.pose.position.y -
          current_transform.transform.translation.y;
        const double position_error = std::hypot(error_x, error_y);
        const double yaw_error = std::atan2(
          std::sin(target_body_yaw - current_yaw),
          std::cos(target_body_yaw - current_yaw));

        if (position_error <= docking_position_tolerance_ &&
          std::abs(yaw_error) <= docking_yaw_tolerance_)
        {
          ++settled_cycles;
          publish_manual_base_stop();
          if (settled_cycles >= 5) {
            return;
          }
        } else {
          settled_cycles = 0;
          const double world_speed = std::min(
            docking_max_linear_speed_,
            docking_linear_gain_ * position_error);
          const double world_velocity_x = position_error > 1.0e-9 ?
            world_speed * error_x / position_error : 0.0;
          const double world_velocity_y = position_error > 1.0e-9 ?
            world_speed * error_y / position_error : 0.0;
          geometry_msgs::msg::Twist command;
          command.linear.x =
            std::cos(current_yaw) * world_velocity_x +
            std::sin(current_yaw) * world_velocity_y;
          command.linear.y =
            -std::sin(current_yaw) * world_velocity_x +
            std::cos(current_yaw) * world_velocity_y;
          command.angular.z = std::clamp(
            docking_angular_gain_ * yaw_error,
            -docking_max_angular_speed_,
            docking_max_angular_speed_);
          manual_base_publisher_->publish(command);
        }

        const auto now = std::chrono::steady_clock::now();
        if (now - last_feedback >= 500ms) {
          last_feedback = now;
          publish_feedback(
            goal_handle,
            PressCabinetButton::Feedback::NAVIGATING,
            0.20F,
            "Precision docking error: " +
            std::to_string(position_error) + " m, yaw: " +
            std::to_string(std::abs(yaw_error)) + " rad.");
        }
        std::this_thread::sleep_for(50ms);
      }
    } catch (...) {
      publish_manual_base_stop();
      throw;
    }

    publish_manual_base_stop();
    throw OperationError(
            PressCabinetButton::Result::NAVIGATION_FAILED,
            "The robot could not reach the odometry staging pose within "
            "the precision-docking timeout.");
  }

  void publish_manual_base_stop() noexcept
  {
    try {
      manual_base_publisher_->publish(geometry_msgs::msg::Twist{});
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        get_logger(), "Failed to stop precision docking: %s", error.what());
    }
  }

  template<typename GoalHandleT>
  void set_navigation_mode(
    const std::shared_ptr<GoalHandleT> & goal_handle,
    bool enabled)
  {
    if (!navigation_mode_client_->wait_for_service(
        std::chrono::duration<double>(system_wait_timeout_)))
    {
      throw OperationError(
              PressCabinetButton::Result::NOT_READY,
              "Base navigation-mode service is unavailable.");
    }
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = enabled;
    auto future = navigation_mode_client_->async_send_request(request);
    wait_for_future(
      future,
      goal_handle,
      system_wait_timeout_,
      PressCabinetButton::Result::NOT_READY,
      "Timed out while switching the base command mode.");
    const auto response = future.get();
    if (!response->success) {
      throw OperationError(
              PressCabinetButton::Result::NOT_READY,
              "Failed to switch the base command mode: " +
              response->message);
    }
  }

  template<typename FutureT, typename GoalHandleT>
  void wait_for_future(
    FutureT & future,
    const std::shared_ptr<GoalHandleT> & goal_handle,
    double timeout_seconds,
    std::uint8_t error_code,
    const std::string & timeout_message)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(timeout_seconds);
    while (future.wait_for(100ms) != std::future_status::ready) {
      check_cancel(goal_handle);
      if (std::chrono::steady_clock::now() >= deadline) {
        throw OperationError(error_code, timeout_message);
      }
    }
  }

  template<typename GoalHandleT>
  void interruptible_hold(
    const std::shared_ptr<GoalHandleT> & goal_handle,
    double seconds)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(seconds);
    while (std::chrono::steady_clock::now() < deadline) {
      check_cancel(goal_handle);
      std::this_thread::sleep_for(20ms);
    }
  }

  template<typename GoalHandleT>
  void check_cancel(
    const std::shared_ptr<GoalHandleT> & goal_handle) const
  {
    if (cancel_requested_.load() || goal_handle->is_canceling() ||
      !rclcpp::ok())
    {
      throw OperationError(
              PressCabinetButton::Result::CANCELED,
              "Cabinet button operation was canceled.");
    }
  }

  bool is_goal_canceling_noexcept(
    const std::shared_ptr<PressGoalHandle> & goal_handle) const noexcept
  {
    try {
      return goal_handle && goal_handle->is_canceling();
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        get_logger(), "Could not read the action cancel state: %s",
        error.what());
    } catch (...) {
      RCLCPP_ERROR(
        get_logger(), "Could not read the action cancel state.");
    }
    return false;
  }

  bool finish_goal_noexcept(
    const std::shared_ptr<PressGoalHandle> & goal_handle,
    const std::shared_ptr<PressCabinetButton::Result> & result,
    bool request_success) noexcept
  {
    try {
      if (!goal_handle || !goal_handle->is_active()) {
        RCLCPP_WARN(
          get_logger(), "Action goal was already in a terminal state.");
        return false;
      }
      if (goal_handle->is_canceling()) {
        result->success = false;
        result->error_code = PressCabinetButton::Result::CANCELED;
        result->message = "Cabinet button operation was canceled.";
        goal_handle->canceled(result);
        return false;
      }
      if (request_success) {
        goal_handle->succeed(result);
        return true;
      }
      goal_handle->abort(result);
      return false;
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        get_logger(), "Failed to set the action terminal state: %s",
        error.what());
    } catch (...) {
      RCLCPP_ERROR(
        get_logger(), "Failed to set the action terminal state.");
    }

    // A cancel request may race the first terminal-state transition.
    try {
      if (goal_handle && goal_handle->is_active() &&
        goal_handle->is_canceling())
      {
        result->success = false;
        result->error_code = PressCabinetButton::Result::CANCELED;
        result->message = "Cabinet button operation was canceled.";
        goal_handle->canceled(result);
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        get_logger(), "Failed to finalize the canceled action: %s",
        error.what());
    } catch (...) {
      RCLCPP_ERROR(
        get_logger(), "Failed to finalize the canceled action.");
    }
    return false;
  }

  void publish_feedback(
    const std::shared_ptr<PressGoalHandle> & goal_handle,
    std::uint8_t phase,
    float progress,
    const std::string & message)
  {
    if (!goal_handle->is_active()) {
      return;
    }
    const auto button = find_button(goal_handle->get_goal()->button_id);
    const auto state = button ? button_snapshot(*button) : ButtonSnapshot{};
    auto feedback = std::make_shared<PressCabinetButton::Feedback>();
    feedback->phase = phase;
    feedback->progress = progress;
    feedback->button_travel = state.position;
    feedback->message = message;
    goal_handle->publish_feedback(feedback);
  }

  void publish_feedback(
    const std::shared_ptr<OperateGoalHandle> & goal_handle,
    std::uint8_t phase,
    float progress,
    const std::string & message)
  {
    double target_position = 0.0;
    const auto control = find_button(goal_handle->get_goal()->control_id);
    if (control) {
      try {
        target_position = resolve_operation_target(
          *control, *goal_handle->get_goal(),
          button_snapshot(*control)).first;
      } catch (...) {
        target_position = button_snapshot(*control).position;
      }
    }
    publish_operate_feedback(
      goal_handle, phase, progress, target_position, message);
  }

  void publish_operate_feedback(
    const std::shared_ptr<OperateGoalHandle> & goal_handle,
    std::uint8_t phase,
    float progress,
    double target_position,
    const std::string & message)
  {
    if (!goal_handle->is_active()) {
      return;
    }
    const auto control = find_button(goal_handle->get_goal()->control_id);
    const auto state = control ?
      button_snapshot(*control) : ButtonSnapshot{};
    auto feedback = std::make_shared<OperateCabinetControl::Feedback>();
    feedback->phase = phase;
    feedback->progress = progress;
    feedback->current_position = state.position;
    feedback->target_position = target_position;
    feedback->current_state = state.state_id;
    feedback->message = message;
    goal_handle->publish_feedback(feedback);
  }

  void stop_active_motion() noexcept
  {
    std::shared_ptr<MoveGroupInterface> move_group;
    {
      std::lock_guard<std::mutex> lock(motion_mutex_);
      move_group = active_move_group_;
    }
    if (move_group) {
      try {
        move_group->getMoveGroupClient().async_cancel_all_goals();
        move_group->stop();
      } catch (const std::exception & error) {
        RCLCPP_ERROR(
          get_logger(), "Failed to stop MoveIt: %s", error.what());
      }
    }
  }

  void cancel_active_navigation() noexcept
  {
    NavigationGoalHandle::SharedPtr navigation_goal;
    {
      std::lock_guard<std::mutex> lock(navigation_mutex_);
      navigation_goal = active_navigation_goal_;
      active_navigation_goal_.reset();
    }
    if (navigation_goal) {
      try {
        navigation_client_->async_cancel_goal(navigation_goal);
      } catch (const std::exception & error) {
        RCLCPP_ERROR(
          get_logger(), "Failed to cancel Nav2: %s", error.what());
      }
    }
  }

  void request_navigation_mode_without_wait(bool enabled) noexcept
  {
    try {
      if (!navigation_mode_client_->service_is_ready()) {
        return;
      }
      auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
      request->data = enabled;
      navigation_mode_client_->async_send_request(request);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        get_logger(), "Failed to request a safe base mode: %s", error.what());
    }
  }

  rclcpp_action::Server<PressCabinetButton>::SharedPtr action_server_;
  rclcpp_action::Server<OperateCabinetControl>::SharedPtr
    operate_action_server_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr navigation_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr navigation_mode_client_;
  rclcpp::Client<
    xczs_inspection_robot_control::srv::SetCabinetGrasp>::SharedPtr
    grasp_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_physics_client_;
  rclcpp::CallbackGroup::SharedPtr reset_client_callback_group_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_controls_service_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
    manual_base_publisher_;
  rclcpp::Publisher<
    xczs_inspection_robot_control::msg::CabinetControlCatalog>::SharedPtr
    control_catalog_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
    active_control_publisher_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr>
  button_joint_state_subscriptions_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr>
  button_pressed_subscriptions_;
  std::vector<rclcpp::Subscription<
      xczs_inspection_robot_control::msg::CabinetControlState>::SharedPtr>
  control_state_subscriptions_;
  std::unique_ptr<tf2_ros::Buffer> transform_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> transform_listener_;

  std::unordered_map<std::string, std::shared_ptr<ButtonSpec>> buttons_by_id_;
  std::vector<std::shared_ptr<ButtonSpec>> buttons_in_order_;
  std::mutex motion_mutex_;
  std::shared_ptr<MoveGroupInterface> active_move_group_;
  std::mutex navigation_mutex_;
  NavigationGoalHandle::SharedPtr active_navigation_goal_;
  std::mutex navigation_feedback_mutex_;
  std::chrono::steady_clock::time_point last_navigation_feedback_{};
  std::mutex worker_mutex_;
  std::thread worker_thread_;
  std::atomic<bool> operation_active_{false};
  std::atomic<bool> cancel_requested_{false};

  std::string planning_frame_;
  std::string navigation_frame_;
  std::string cabinet_frame_;
  double cabinet_x_{2.0};
  double cabinet_y_{0.33};
  double cabinet_z_{0.0};
  double cabinet_roll_{1.57079632679};
  double cabinet_pitch_{0.0};
  double cabinet_yaw_{-1.57079632679};
  double tool_tip_offset_{0.370};
  double staging_distance_{1.040};
  double prepress_distance_{0.060};
  double contact_clearance_{0.001};
  double press_depth_{0.007};
  double button_state_timeout_{1.0};
  double system_wait_timeout_{15.0};
  double navigation_timeout_{120.0};
  double navigation_takeover_distance_{0.15};
  double docking_timeout_{20.0};
  double docking_position_tolerance_{0.015};
  double docking_yaw_tolerance_{0.10};
  double docking_max_linear_speed_{0.15};
  double docking_max_angular_speed_{0.30};
  double docking_linear_gain_{0.8};
  double docking_angular_gain_{1.2};
  double base_link_yaw_offset_{1.57079632679};
  double press_detection_timeout_{3.0};
  double release_detection_timeout_{3.0};
  double press_hold_seconds_{0.5};
  double target_tolerance_{0.035};
  double stable_velocity_tolerance_{0.03};
  double stable_state_duration_{0.30};
  double planning_scene_settle_seconds_{0.50};
  double rotation_waypoint_step_{0.03490658504};
  double cartesian_velocity_scale_{0.08};
  double cartesian_acceleration_scale_{0.08};
};

}  // namespace xczs_inspection_robot_control

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<
    xczs_inspection_robot_control::CabinetButtonOperator>();
  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
