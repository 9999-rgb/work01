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
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "Eigen/Geometry"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/robot_model/robot_model.h"
#include "moveit/robot_trajectory/robot_trajectory.h"
#include "moveit/trajectory_processing/time_optimal_trajectory_generation.h"
#include "moveit/utils/moveit_error_code.h"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/expand_topic_or_service_name.hpp"
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
#include "xczs_inspection_robot_control/action_terminal_policy.hpp"
#include "xczs_inspection_robot_control/msg/cabinet_control.hpp"
#include "xczs_inspection_robot_control/msg/cabinet_control_catalog.hpp"
#include "xczs_inspection_robot_control/msg/cabinet_control_state.hpp"
#include "xczs_inspection_robot_control/operation_validation_policy.hpp"
#include "xczs_inspection_robot_control/rotary_operation_policy.hpp"
#include "xczs_inspection_robot_control/router_utils.hpp"
#include "xczs_inspection_robot_control/staging_safety_policy.hpp"
#include "xczs_inspection_robot_control/structured_control_state_policy.hpp"
#include "xczs_inspection_robot_control/srv/manage_operation_lease.hpp"
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
using ManageOperationLease =
  xczs_inspection_robot_control::srv::ManageOperationLease;

constexpr char kActionName[] = "press_cabinet_button";
constexpr char kOperateActionName[] = "operate_cabinet_control";
constexpr char kControlCatalogTopic[] = "control_catalog";
constexpr char kActiveControlTopic[] = "active_control";
constexpr char kResetControlsService[] = "reset_controls";
constexpr char kEmbeddedNavigationDisabledMessage[] =
  "Embedded cabinet navigation is disabled because navigation is executed "
  "independently by the task layer.";

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
  geometry_msgs::msg::Pose prepress_pose;
  geometry_msgs::msg::Pose contact_pose;
  geometry_msgs::msg::Pose pressed_pose;
};

struct RotaryOperationPoses
{
  geometry_msgs::msg::Pose ready_pose;
  geometry_msgs::msg::Pose door_pregrasp_pose;
  geometry_msgs::msg::Pose grasp_pose;
};

struct DoorArcProgress
{
  bool in_progress{false};
  double initial_position{0.0};
  std::size_t completed_waypoints{0U};
  std::size_t total_waypoints{0U};
};

struct ControlNavigationStation
{
  tf2::Vector3 local_anchor{0.0, 0.0, 0.0};
  tf2::Vector3 outward_axis{0.0, 0.0, 1.0};
  double standoff{0.0};
  double base_yaw_offset{0.0};
  std::string frame_id;
};

struct ControlStagingPoses
{
  geometry_msgs::msg::PoseStamped navigation_pose;
  geometry_msgs::msg::PoseStamped planning_pose;
};

// One control type's robot-side tool binding.  Every cabinet control type is
// operated by exactly one arm (MoveIt group) whose contact link
// presses/grasps/rocks the control; ``transport_named_target`` is that arm's
// safe retreat pose in the SRDF.
struct ToolProfile
{
  enum class ToolAxisOrientation
  {
    // Tool local +Z points along the control's outward (retraction) axis.
    // Matches the legacy three-cylinder design whose business point sits at
    // the opposite (-Z) end, so +Z is the empty, non-contacting side.
    ALONG_OUTWARD,
    // Tool local +Z points toward the control (opposite the outward axis).
    // Required by pincer tools (rotate_button, rocker) whose jaws / rotor are
    // at the +Z end: keeping +Z == outward would push the wrist past the
    // control into the cabinet, making the grasp pose unreachable.
    TOWARD_CONTROL,
  };

  std::string move_group;
  std::string contact_tool_link;
  // Link whose origin the cabinet grasp plugin should measure against the
  // control's grasp point.  Defaults to contact_tool_link.  Pincer tools
  // (rotate_button) carry their jaws far from the contact-tool origin, so the
  // body origin can sit well outside the grasp distance threshold even when
  // the jaws are exactly on the control; the jaw-carrier link is a better
  // distance probe.
  std::string grasp_link;
  std::string transport_named_target;
  ToolAxisOrientation tool_axis_orientation{ToolAxisOrientation::ALONG_OUTWARD};
  // Full 3-D position of the physical business point in contact_tool_link.
  // A scalar axial offset cannot represent an off-axis finger and can align
  // the empty tool centre with a button while a real finger hits the panel.
  tf2::Vector3 tool_tip_position{0.0, 0.0, 0.0};
  // A business point on a movable tool link is valid only at its calibrated
  // tool-joint state.  Reject operation instead of silently using a stale
  // geometric offset after manual tool motion.
  std::vector<std::string> calibration_joint_names;
  std::vector<double> calibration_joint_positions;
};

struct ButtonSnapshot
{
  bool received{false};
  bool structured_received{false};
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
  std::chrono::steady_clock::time_point structured_received_at{};
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
  double grasp_outward_offset{0.0};
  // Physical execution is granted only by the explicit robot-adapter
  // operable_control_ids allowlist populated during configure_controls().
  bool operable{false};
  std::string unavailable_reason;
  std::optional<ControlNavigationStation> navigation_station;
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
  double spring_stiffness{0.0};
  double press_threshold{0.0};
  double default_force{0.0};
  std::vector<std::string> state_ids;
  std::vector<std::string> state_labels;
  std::vector<double> state_positions;
  // Optional fixed roll about the contact tool's local +Z axis.  Buttons can
  // use it to keep the passive sibling tool and forearm away from the panel
  // while the measured off-axis finger remains exactly on the target.
  double tool_roll_offset{0.0};
  // Row-major [source detent][target detent] rotation about the contact
  // tool's local +Z axis.  It reorients passive sibling tools in the cabinet
  // plane without changing the calibrated contact direction.
  std::vector<double> tool_roll_offsets;
  // Over-center controls may be released after safely crossing the next
  // detent midpoint so their physical spring finishes the motion.
  double detent_release_fraction{1.0};
  // Optional named MoveIt joint seed for the ready-pose IK.  A redundant arm
  // can otherwise reach the same pose through a branch that cannot continue
  // through the subsequent Cartesian manipulation.
  std::vector<std::string> ready_joint_seed_names;
  std::vector<double> ready_joint_seed_positions;
  std::shared_ptr<ButtonRuntime> runtime{std::make_shared<ButtonRuntime>()};
};

struct ResolvedControlGeometry
{
  tf2::Vector3 grasp_zero;
  tf2::Vector3 pivot;
  tf2::Vector3 axis;
  tf2::Vector3 approach_normal;
};

constexpr double clamp_button_press_depth(
  double requested_depth, double max_position) noexcept
{
  return requested_depth < max_position ? requested_depth : max_position;
}

constexpr double button_force_tracking_limit(
  double target_depth, double max_compensation,
  double max_position) noexcept
{
  const double bounded_target = clamp_button_press_depth(
    target_depth, max_position);
  const double remaining_travel = max_position - bounded_target;
  const double bounded_compensation =
    max_compensation < remaining_travel ?
    max_compensation : remaining_travel;
  return bounded_target + bounded_compensation;
}

static_assert(
  button_force_tracking_limit(0.008, 0.001, 0.008) == 0.008,
  "Force compensation must stop at a button's maximum travel.");
static_assert(
  button_force_tracking_limit(0.0075, 0.001, 0.008) == 0.008,
  "Force compensation must clamp partial remaining travel.");
static_assert(
  button_force_tracking_limit(0.006, 0.001, 0.008) == 0.007,
  "Force compensation below the limit must preserve its configured range.");

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

std::optional<std::string> tool_profile_kinematics_validation_error(
  const moveit::core::RobotModelConstPtr & robot_model,
  std::vector<std::string> move_group_names)
{
  std::sort(move_group_names.begin(), move_group_names.end());
  move_group_names.erase(
    std::unique(move_group_names.begin(), move_group_names.end()),
    move_group_names.end());

  std::vector<std::string> errors;
  errors.reserve(move_group_names.size());
  for (const auto & move_group_name : move_group_names) {
    if (!robot_model) {
      errors.emplace_back(
        "MoveIt RobotModel is unavailable while validating tool profile "
        "JointModelGroup '" + move_group_name + "'.");
      continue;
    }

    const auto * joint_model_group =
      robot_model->getJointModelGroup(move_group_name);
    if (!joint_model_group) {
      errors.emplace_back(
        "The robot adapter tool profile references missing MoveIt "
        "JointModelGroup '" + move_group_name + "'.");
      continue;
    }
    if (!joint_model_group->getSolverInstance()) {
      errors.emplace_back(
        "MoveIt JointModelGroup '" + move_group_name +
        "' has no configured kinematics solver.");
    }
  }

  if (errors.empty()) {
    return std::nullopt;
  }

  std::ostringstream message;
  for (std::size_t index = 0; index < errors.size(); ++index) {
    if (index != 0U) {
      message << ' ';
    }
    message << errors[index];
  }
  return message.str();
}

class CabinetButtonOperator final : public rclcpp::Node
{
public:
  CabinetButtonOperator()
  : Node("xczs_cabinet_button_operator")
  {
    planning_frame_ = declare_parameter<std::string>(
      "planning_frame", "odom");
    navigation_frame_ = declare_parameter<std::string>(
      "navigation_frame", "map");
    robot_model_name_ = required_string_parameter(
      "robot_model_name", "");
    move_group_namespace_ = declare_parameter<std::string>(
      "move_group_namespace", "/");
    toolset_ = declare_parameter<std::string>("toolset", "A");
    if (toolset_ != "A" && toolset_ != "B") {
      throw std::invalid_argument(
              "Parameter 'toolset' must be 'A' or 'B'.");
    }
    load_tool_profiles();
    tool_tip_calibration_joint_tolerance_ = positive_parameter(
      "tool_tip_calibration_joint_tolerance", 0.001);
    planning_time_ = positive_parameter("planning_time", 10.0);
    planning_attempts_ = declare_parameter<int>("planning_attempts", 10);
    if (planning_attempts_ < 1 || planning_attempts_ > 100) {
      throw std::invalid_argument(
              "Parameter 'planning_attempts' must be in [1, 100].");
    }
    planning_velocity_scale_ = unit_interval_parameter(
      "planning_velocity_scale", 0.20);
    planning_acceleration_scale_ = unit_interval_parameter(
      "planning_acceleration_scale", 0.20);
    goal_position_tolerance_ = positive_parameter(
      "goal_position_tolerance", 0.005);
    goal_orientation_tolerance_ = positive_parameter(
      "goal_orientation_tolerance", 0.01);
    goal_joint_tolerance_ = positive_parameter(
      "goal_joint_tolerance", 0.001);
    allow_replanning_ = declare_parameter<bool>("allow_replanning", true);
    navigation_base_frame_ = required_string_parameter(
      "navigation_base_frame", "");
    const auto navigation_action = required_string_parameter(
      "navigation_action", "/navigate_to_pose");
    require_absolute_ros_name(
      navigation_action, "navigation_action", false);
    const auto navigation_mode_service = required_string_parameter(
      "navigation_mode_service", "/xczs/set_navigation_mode");
    require_absolute_ros_name(
      navigation_mode_service, "navigation_mode_service", true);
    allow_embedded_navigation_ = declare_parameter<bool>(
      "allow_embedded_navigation", true);
    docking_base_frame_ = required_string_parameter(
      "docking_base_frame", "");
    grasp_brake_link_ = required_string_parameter(
      "grasp_brake_link", "");
    const auto control_catalog_topic = declare_parameter<std::string>(
      "control_catalog_topic", kControlCatalogTopic);
    cabinet_frame_ = declare_parameter<std::string>(
      "cabinet_frame", "control_cabinet_frame");
    require_cabinet_pose_valid_ = declare_parameter<bool>(
      "require_cabinet_pose_valid", true);
    const auto cabinet_pose_valid_topic = required_string_parameter(
      "cabinet_pose_valid_topic", "pose_valid");
    cabinet_pose_translation_tolerance_ = positive_parameter(
      "cabinet_pose_translation_tolerance", 0.020);
    cabinet_pose_rotation_tolerance_ = positive_parameter(
      "cabinet_pose_rotation_tolerance", 0.035);
    const auto grasp_service = declare_parameter<std::string>(
      "grasp_service", "grasp");
    const auto reset_physics_service = declare_parameter<std::string>(
      "reset_physics_service", "reset_physics");
    const auto manual_cmd_vel_topic = required_string_parameter(
      "manual_cmd_vel_topic", "/xczs/manual_cmd_vel");
    require_absolute_ros_name(
      manual_cmd_vel_topic, "manual_cmd_vel_topic", false);
    const auto operation_lease_service = required_string_parameter(
      "operation_lease_service", "/xczs/operation_lease");
    operation_lease_duration_ = positive_parameter(
      "operation_lease_duration", 3.0);
    operation_lease_renew_period_ = positive_parameter(
      "operation_lease_renew_period", 0.75);
    operation_lease_request_timeout_ = positive_parameter(
      "operation_lease_request_timeout", 0.50);
    if (operation_lease_renew_period_ + operation_lease_request_timeout_ >=
      operation_lease_duration_)
    {
      throw std::invalid_argument(
              "The lease renew period plus request timeout must be shorter "
              "than the lease duration.");
    }

    prepress_distance_ = positive_parameter("prepress_distance", 0.060);
    grasp_outward_offset_ = positive_parameter(
      "grasp_outward_offset", 0.020);
    contact_clearance_ = positive_parameter("contact_clearance", 0.001);
    press_depth_ = positive_parameter("press_depth", 0.007);
    button_state_timeout_ = positive_parameter("state_timeout", 1.0);
    system_wait_timeout_ = positive_parameter("system_wait_timeout", 15.0);
    navigation_timeout_ = positive_parameter(
      "navigation_timeout", 120.0);
    navigation_takeover_distance_ = positive_parameter(
      "navigation_takeover_distance", 0.15);
    navigation_takeover_yaw_tolerance_ = positive_parameter(
      "navigation_takeover_yaw_tolerance", 0.35);
    navigation_takeover_stall_timeout_ = positive_parameter(
      "navigation_takeover_stall_timeout", 4.0);
    docking_timeout_ = positive_parameter("docking_timeout", 45.0);
    docking_position_tolerance_ = positive_parameter(
      "docking_position_tolerance", 0.015);
    docking_yaw_tolerance_ = positive_parameter(
      "docking_yaw_tolerance", 0.10);
    const auto flat_docking_base_footprint =
      declare_parameter<std::vector<double>>(
      "docking_base_footprint", std::vector<double>{});
    if ((!flat_docking_base_footprint.empty() &&
      flat_docking_base_footprint.size() < 6U) ||
      flat_docking_base_footprint.size() % 2U != 0U ||
      std::any_of(
        flat_docking_base_footprint.begin(),
        flat_docking_base_footprint.end(),
        [](double value) {return !std::isfinite(value);}))
    {
      throw std::invalid_argument(
              "Parameter 'docking_base_footprint' must contain at least "
              "three finite [x, y] points in the navigation-base frame.");
    }
    for (std::size_t index = 0U;
      index < flat_docking_base_footprint.size(); index += 2U)
    {
      docking_base_footprint_.push_back(
        {flat_docking_base_footprint[index],
          flat_docking_base_footprint[index + 1U]});
    }
    docking_base_footprint_padding_ = positive_parameter(
      "docking_base_footprint_padding", 0.03);
    docking_max_linear_speed_ = positive_parameter(
      "docking_max_linear_speed", 0.15);
    docking_max_angular_speed_ = positive_parameter(
      "docking_max_angular_speed", 0.45);
    docking_linear_gain_ = positive_parameter(
      "docking_linear_gain", 0.8);
    docking_angular_gain_ = positive_parameter(
      "docking_angular_gain", 1.2);
    const auto & parameter_overrides =
      get_node_parameters_interface()->get_parameter_overrides();
    const bool navigation_yaw_offset_overridden =
      parameter_overrides.count("navigation_velocity_yaw_offset") != 0U;
    const bool legacy_yaw_offset_overridden =
      parameter_overrides.count("base_link_yaw_offset") != 0U;
    const double configured_navigation_yaw_offset = finite_parameter(
      "navigation_velocity_yaw_offset", 1.57079632679);
    const double legacy_yaw_offset = finite_parameter(
      "base_link_yaw_offset", configured_navigation_yaw_offset);
    navigation_velocity_yaw_offset_ =
      !navigation_yaw_offset_overridden && legacy_yaw_offset_overridden ?
      legacy_yaw_offset : configured_navigation_yaw_offset;
    if (!navigation_yaw_offset_overridden && legacy_yaw_offset_overridden) {
      RCLCPP_WARN(
        get_logger(),
        "Parameter 'base_link_yaw_offset' is deprecated; use "
        "'navigation_velocity_yaw_offset'.");
    }
    press_detection_timeout_ = positive_parameter(
      "press_detection_timeout", 3.0);
    release_detection_timeout_ = positive_parameter(
      "release_detection_timeout", 3.0);
    press_hold_seconds_ = positive_parameter("press_hold_seconds", 0.5);
    force_tracking_tolerance_ = positive_parameter(
      "force_tracking_tolerance", 0.00005);
    force_tracking_max_compensation_ = positive_parameter(
      "force_tracking_max_compensation", 0.0010);
    force_tracking_settle_seconds_ = positive_parameter(
      "force_tracking_settle_seconds", 0.15);
    force_tracking_attempts_ = declare_parameter<int>(
      "force_tracking_attempts", 3);
    if (force_tracking_attempts_ < 1 || force_tracking_attempts_ > 5) {
      throw std::invalid_argument(
              "Parameter 'force_tracking_attempts' must be in [1, 5].");
    }
    button_press_minimum_cartesian_fraction_ = unit_interval_parameter(
      "button_press_minimum_cartesian_fraction", 0.95);
    if (button_press_minimum_cartesian_fraction_ < 0.90 ||
      button_press_minimum_cartesian_fraction_ > 0.99)
    {
      throw std::invalid_argument(
              "Parameter 'button_press_minimum_cartesian_fraction' must be "
              "in [0.90, 0.99].");
    }
    target_tolerance_ = positive_parameter("target_tolerance", 0.035);
    stable_velocity_tolerance_ = positive_parameter(
      "stable_velocity_tolerance", 0.03);
    stable_state_duration_ = positive_parameter(
      "stable_state_duration", 0.30);
    grasp_attach_settle_duration_ = positive_parameter(
      "grasp_attach_settle_duration", 0.15);
    grasp_release_settle_duration_ = positive_parameter(
      "grasp_release_settle_duration", 0.30);
    door_release_fraction_ = declare_parameter<double>(
      "door_release_fraction", 0.60);
    if (!std::isfinite(door_release_fraction_) ||
      door_release_fraction_ <= 0.5 || door_release_fraction_ >= 0.75)
    {
      throw std::invalid_argument(
              "Parameter 'door_release_fraction' must be in (0.5, 0.75).");
    }
    door_settle_timeout_ = positive_parameter("door_settle_timeout", 90.0);
    door_release_position_timeout_ = positive_parameter(
      "door_release_position_timeout", 10.0);
    door_detent_hysteresis_ = positive_parameter(
      "door_detent_hysteresis", 0.02);
    door_release_position_margin_ = positive_parameter(
      "door_release_position_margin", 0.01);
    door_pregrasp_clearance_ = positive_parameter(
      "door_pregrasp_clearance", 0.010);
    if (door_pregrasp_clearance_ > 0.015) {
      throw std::invalid_argument(
              "Parameter 'door_pregrasp_clearance' must be no greater "
              "than 0.015 m.");
    }
    door_release_clearance_ = positive_parameter(
      "door_release_clearance", 0.30);
    if (door_release_clearance_ < 0.28 ||
      door_release_clearance_ > 0.35)
    {
      throw std::invalid_argument(
              "Parameter 'door_release_clearance' must be in "
              "[0.28, 0.35] m.");
    }
    planning_scene_settle_seconds_ = positive_parameter(
      "planning_scene_settle_seconds", 0.50);
    rotation_waypoint_step_ = positive_parameter(
      "rotation_waypoint_step", 0.03490658504);
    cartesian_velocity_scale_ = unit_interval_parameter(
      "cartesian_velocity_scale", 0.08);
    cartesian_acceleration_scale_ = unit_interval_parameter(
      "cartesian_acceleration_scale", 0.08);
    cartesian_jump_threshold_ = positive_parameter(
      "cartesian_jump_threshold", 2.0);
    if (cartesian_jump_threshold_ < 1.0 ||
      cartesian_jump_threshold_ > 10.0)
    {
      throw std::invalid_argument(
              "Parameter 'cartesian_jump_threshold' must be in [1, 10].");
    }
    cartesian_planning_attempts_ = declare_parameter<int>(
      "cartesian_planning_attempts", 5);
    if (cartesian_planning_attempts_ < 1 || cartesian_planning_attempts_ > 10) {
      throw std::invalid_argument(
              "Parameter 'cartesian_planning_attempts' must be in [1, 10].");
    }
    door_cartesian_segment_waypoints_ = declare_parameter<int>(
      "door_cartesian_segment_waypoints", 2);
    if (door_cartesian_segment_waypoints_ != 2) {
      throw std::invalid_argument(
              "Parameter 'door_cartesian_segment_waypoints' must be 2.");
    }
    motion_planning_attempts_ = declare_parameter<int>(
      "motion_planning_attempts", 3);
    if (motion_planning_attempts_ < 2 || motion_planning_attempts_ > 10) {
      throw std::invalid_argument(
              "Parameter 'motion_planning_attempts' must be in "
              "[2, 10].");
    }

    configure_controls();
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
      this, navigation_action);
    navigation_mode_client_ = create_client<std_srvs::srv::SetBool>(
      navigation_mode_service);
    grasp_client_ =
      create_client<xczs_inspection_robot_control::srv::SetCabinetGrasp>(
      grasp_service);
    reset_client_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
    reset_physics_client_ = create_client<std_srvs::srv::Trigger>(
      reset_physics_service, rmw_qos_profile_services_default,
      reset_client_callback_group_);
    operation_lease_client_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
    operation_lease_client_ = create_client<ManageOperationLease>(
      operation_lease_service, rmw_qos_profile_services_default,
      operation_lease_client_callback_group_);
    manual_base_publisher_ = create_publisher<geometry_msgs::msg::Twist>(
      manual_cmd_vel_topic, 10);
    transform_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    transform_listener_ =
      std::make_unique<tf2_ros::TransformListener>(*transform_buffer_);
    cabinet_pose_valid_subscription_ = create_subscription<std_msgs::msg::Bool>(
      cabinet_pose_valid_topic,
      rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::Bool::SharedPtr message) {
        cabinet_pose_valid_.store(message->data);
        cabinet_pose_valid_received_.store(true);
      });

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
    // reset_controls 在 handler 内同步 wait_for_service/wait_for 可达数秒；
    // 放在独立的 Reentrant 组，避免阻塞默认组的 action server 与其它服务。
    reset_service_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
    reset_controls_service_ = create_service<std_srvs::srv::Trigger>(
      kResetControlsService,
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
      {
        reset_controls(response);
      },
      rmw_qos_profile_services_default,
      reset_service_callback_group_);
    publish_control_catalog();
    publish_active_control("");

    RCLCPP_INFO(
      get_logger(),
      "Cabinet action %s controls %zu configured cabinet controls.",
      kActionName, buttons_in_order_.size());
  }

  ~CabinetButtonOperator() override
  {
    shutdown_requested_.store(true);
    stop_active_motion();
    cancel_active_navigation();
    {
      std::lock_guard<std::mutex> lock(worker_mutex_);
      if (worker_thread_.joinable()) {
        worker_thread_.join();
      }
    }
    release_operation_lease_noexcept();
  }

private:
  enum class ActiveGoalType
  {
    NONE,
    PRESS,
    OPERATE,
  };

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

  ToolProfile read_tool_profile(const std::string & type_name)
  {
    const std::string prefix = "tool_profiles." + type_name + ".";
    ToolProfile profile;
    profile.move_group = declare_parameter<std::string>(
      prefix + "move_group", "");
    profile.contact_tool_link = declare_parameter<std::string>(
      prefix + "contact_tool_link", "");
    profile.grasp_link = declare_parameter<std::string>(
      prefix + "grasp_link", profile.contact_tool_link);
    profile.transport_named_target = declare_parameter<std::string>(
      prefix + "transport_named_target", "");
    const auto tool_axis_orientation = declare_parameter<std::string>(
      prefix + "tool_axis_orientation", "along_outward");
    if (tool_axis_orientation == "along_outward") {
      profile.tool_axis_orientation =
        ToolProfile::ToolAxisOrientation::ALONG_OUTWARD;
    } else if (tool_axis_orientation == "toward_control") {
      profile.tool_axis_orientation =
        ToolProfile::ToolAxisOrientation::TOWARD_CONTROL;
    } else {
      throw std::invalid_argument(
              "Parameter '" + prefix + "tool_axis_orientation' must be "
              "'along_outward' or 'toward_control', got '" +
              tool_axis_orientation + "'.");
    }
    const auto tool_tip_position = declare_parameter<std::vector<double>>(
      prefix + "tool_tip_position", std::vector<double>{});
    const double legacy_tool_tip_offset = declare_parameter<double>(
      prefix + "tool_tip_offset", 0.0);
    if (tool_tip_position.empty()) {
      if (!std::isfinite(legacy_tool_tip_offset) ||
        legacy_tool_tip_offset < 0.0)
      {
        throw std::invalid_argument(
                "Parameter '" + prefix +
                "tool_tip_offset' must be finite and non-negative.");
      }
      profile.tool_tip_position = tf2::Vector3(
        0.0, 0.0, -legacy_tool_tip_offset);
    } else {
      if (tool_tip_position.size() != 3U ||
        std::any_of(
          tool_tip_position.begin(), tool_tip_position.end(),
          [](double value) {return !std::isfinite(value);}))
      {
        throw std::invalid_argument(
                "Parameter '" + prefix +
                "tool_tip_position' must contain three finite values.");
      }
      if (std::abs(legacy_tool_tip_offset) > 1.0e-12) {
        throw std::invalid_argument(
                "Tool profile '" + type_name +
                "' must not declare both tool_tip_position and the legacy "
                "non-zero tool_tip_offset.");
      }
      profile.tool_tip_position = tf2::Vector3(
        tool_tip_position[0], tool_tip_position[1], tool_tip_position[2]);
    }
    profile.calibration_joint_names =
      declare_parameter<std::vector<std::string>>(
      prefix + "calibration_joint_names", std::vector<std::string>{});
    profile.calibration_joint_positions =
      declare_parameter<std::vector<double>>(
      prefix + "calibration_joint_positions", std::vector<double>{});
    const std::unordered_set<std::string> unique_calibration_joints(
      profile.calibration_joint_names.begin(),
      profile.calibration_joint_names.end());
    if (profile.calibration_joint_names.size() !=
      profile.calibration_joint_positions.size() ||
      unique_calibration_joints.size() !=
      profile.calibration_joint_names.size() ||
      std::any_of(
        profile.calibration_joint_names.begin(),
        profile.calibration_joint_names.end(),
        [](const std::string & value) {return value.empty();}) ||
      std::any_of(
        profile.calibration_joint_positions.begin(),
        profile.calibration_joint_positions.end(),
        [](double value) {return !std::isfinite(value);}))
    {
      throw std::invalid_argument(
              "Tool profile '" + type_name +
              "' calibration joint names/positions must be finite, unique "
              "and have equal length.");
    }
    return profile;
  }

  void load_tool_profiles()
  {
    using Control = xczs_inspection_robot_control::msg::CabinetControl;
    tool_profiles_[Control::TYPE_BUTTON] = read_tool_profile("button");
    tool_profiles_[Control::TYPE_KNOB] = read_tool_profile("knob");
    tool_profiles_[Control::TYPE_SWITCH] = read_tool_profile("switch");
    tool_profiles_[Control::TYPE_DOOR] = read_tool_profile("door");

    // A legacy single-arm adapter declares one scalar move_group / tip link /
    // contact tool link / transport target; apply it to every control type so
    // historical adapter files keep loading unchanged.
    if (tool_profiles_[Control::TYPE_BUTTON].move_group.empty()) {
      ToolProfile legacy;
      legacy.move_group = required_string_parameter("move_group", "");
      legacy.contact_tool_link =
        required_string_parameter("contact_tool_link", "");
      legacy.transport_named_target =
        required_string_parameter("transport_named_target", "");
      for (auto & entry : tool_profiles_) {
        entry.second = legacy;
      }
    } else {
      for (const auto & entry : tool_profiles_) {
        const auto & profile = entry.second;
        if (profile.move_group.empty() ||
          profile.contact_tool_link.empty() ||
          profile.transport_named_target.empty())
        {
          throw std::invalid_argument(
                  "Every tool_profiles entry must declare move_group, "
                  "contact_tool_link and transport_named_target.");
        }
      }
    }
    apply_tool_profile(Control::TYPE_BUTTON);
  }

  void apply_tool_profile(std::uint8_t control_type)
  {
    const auto iterator = tool_profiles_.find(control_type);
    if (iterator == tool_profiles_.end()) {
      throw std::invalid_argument(
              "No tool profile is configured for control type " +
              std::to_string(static_cast<int>(control_type)) + ".");
    }
    const auto & profile = iterator->second;
    move_group_name_ = profile.move_group;
    contact_tool_link_ = profile.contact_tool_link;
    grasp_link_ = profile.grasp_link.empty() ?
      profile.contact_tool_link : profile.grasp_link;
    transport_named_target_ = profile.transport_named_target;
    tool_axis_orientation_ = profile.tool_axis_orientation;
    tool_tip_position_ = profile.tool_tip_position;
    tool_tip_calibration_joint_names_ = profile.calibration_joint_names;
    tool_tip_calibration_joint_positions_ =
      profile.calibration_joint_positions;
  }

  void verify_tool_tip_calibration_state(
    const moveit::core::RobotState & robot_state) const
  {
    for (std::size_t index = 0U;
      index < tool_tip_calibration_joint_names_.size(); ++index)
    {
      const auto & joint_name = tool_tip_calibration_joint_names_[index];
      double measured_position;
      try {
        measured_position = robot_state.getVariablePosition(joint_name);
      } catch (const std::exception & error) {
        throw OperationError(
                PressCabinetButton::Result::NOT_READY,
                "Tool business-point calibration references unknown joint '" +
                joint_name + "': " + error.what());
      }
      const double expected_position =
        tool_tip_calibration_joint_positions_[index];
      if (!std::isfinite(measured_position) ||
        std::abs(measured_position - expected_position) >
        tool_tip_calibration_joint_tolerance_)
      {
        throw OperationError(
                PressCabinetButton::Result::NOT_READY,
                "Tool joint '" + joint_name + "' is at " +
                std::to_string(measured_position) +
                " rad/m, outside its calibrated business-point position " +
                std::to_string(expected_position) + " +/- " +
                std::to_string(tool_tip_calibration_joint_tolerance_) +
                "; arm motion was blocked. Reset the tool first.");
      }
    }
  }

  void require_absolute_ros_name(
    const std::string & value,
    const std::string & parameter_name,
    bool is_service) const
  {
    if (value.empty() || value.front() != '/') {
      throw std::invalid_argument(
              "Parameter '" + parameter_name +
              "' must be an absolute ROS name.");
    }
    try {
      (void)rclcpp::expand_topic_or_service_name(
        value, get_name(), get_namespace(), is_service);
    } catch (const std::exception & error) {
      throw std::invalid_argument(
              "Parameter '" + parameter_name +
              "' must be a valid absolute ROS name: " + error.what());
    }
  }

  void activate_goal(
    ActiveGoalType type,
    const rclcpp_action::GoalUUID & goal_id)
  {
    std::lock_guard<std::mutex> lock(active_goal_mutex_);
    active_goal_type_ = type;
    active_goal_id_ = goal_id;
    // An accepted goal does not own shared MoveIt/Nav2/base resources.  The
    // worker promotes this flag only after it has acquired the global lease.
    active_goal_owns_physical_motion_resources_ = false;
    active_goal_physical_outcome_committed_ = false;
    cancel_requested_.store(false);
  }

  void claim_active_goal_physical_motion_resources(
    ActiveGoalType type,
    const rclcpp_action::GoalUUID & goal_id,
    bool resources_required)
  {
    if (!resources_required) {
      return;
    }
    std::lock_guard<std::mutex> lock(active_goal_mutex_);
    const bool goal_is_active =
      active_goal_matches_locked(type, goal_id) &&
      !cancel_requested_.load() && !shutdown_requested_.load();
    const bool lease_is_active =
      operation_lease_held_.load() && !operation_lease_lost_.load();
    if (!physical_motion_resources_are_owned(
        resources_required, lease_is_active, goal_is_active))
    {
      throw OperationError(
              lease_is_active ? PressCabinetButton::Result::CANCELED :
              PressCabinetButton::Result::LEASE_LOST,
              lease_is_active ?
              "Cabinet button operation was canceled before it acquired "
              "physical motion resources." :
              "The global robot operation lease was lost before physical "
              "motion resources were acquired.");
    }
    active_goal_owns_physical_motion_resources_ = true;
  }

  bool active_goal_owns_physical_motion_resources(
    ActiveGoalType type,
    const rclcpp_action::GoalUUID & goal_id) const
  {
    std::lock_guard<std::mutex> lock(active_goal_mutex_);
    return active_goal_matches_locked(type, goal_id) &&
           active_goal_owns_physical_motion_resources_;
  }

  void relinquish_active_goal_physical_motion_resources(
    ActiveGoalType type,
    const rclcpp_action::GoalUUID & goal_id) noexcept
  {
    std::lock_guard<std::mutex> lock(active_goal_mutex_);
    if (active_goal_matches_locked(type, goal_id)) {
      active_goal_owns_physical_motion_resources_ = false;
    }
  }

  bool commit_active_goal_physical_outcome(
    ActiveGoalType type,
    const rclcpp_action::GoalUUID & goal_id) noexcept
  {
    std::lock_guard<std::mutex> lock(active_goal_mutex_);
    if (!active_goal_matches_locked(type, goal_id) ||
      cancel_requested_.load() || shutdown_requested_.load() ||
      operation_lease_lost_.load() || !rclcpp::ok())
    {
      return false;
    }
    active_goal_physical_outcome_committed_ = true;
    return true;
  }

  bool active_goal_matches_locked(
    ActiveGoalType type,
    const rclcpp_action::GoalUUID & goal_id) const noexcept
  {
    return active_goal_type_ == type && active_goal_id_ == goal_id;
  }

  bool cancel_requested_for(
    ActiveGoalType type,
    const rclcpp_action::GoalUUID & goal_id) const
  {
    std::lock_guard<std::mutex> lock(active_goal_mutex_);
    return !active_goal_matches_locked(type, goal_id) ||
           cancel_requested_.load();
  }

  bool cancel_requested_for(
    const std::shared_ptr<PressGoalHandle> & goal_handle) const
  {
    return !goal_handle || cancel_requested_for(
      ActiveGoalType::PRESS, goal_handle->get_goal_id());
  }

  bool cancel_requested_for(
    const std::shared_ptr<OperateGoalHandle> & goal_handle) const
  {
    return !goal_handle || cancel_requested_for(
      ActiveGoalType::OPERATE, goal_handle->get_goal_id());
  }

  template<typename GoalHandleT>
  bool goal_should_stop(
    const std::shared_ptr<GoalHandleT> & goal_handle) const noexcept
  {
    try {
      return cancel_requested_for(goal_handle) ||
             goal_handle->is_canceling() || shutdown_requested_.load() ||
             operation_lease_lost_.load() || !rclcpp::ok();
    } catch (...) {
      return true;
    }
  }

  template<typename GoalHandleT>
  bool goal_canceling_after_request(
    const std::shared_ptr<GoalHandleT> & goal_handle) const noexcept
  {
    try {
      if (!goal_handle) {
        return false;
      }
      if (!cancel_requested_for(goal_handle)) {
        return goal_handle->is_canceling();
      }

      // rclcpp_action changes the handle to CANCELING only after the user
      // cancel callback returns ACCEPT. The worker can observe our bound flag
      // during that small interval, so wait for the action state transition
      // before selecting the terminal state.
      const auto deadline = std::chrono::steady_clock::now() + 1s;
      while (!goal_handle->is_canceling() &&
        std::chrono::steady_clock::now() < deadline && rclcpp::ok())
      {
        std::this_thread::sleep_for(1ms);
      }
      return goal_handle->is_canceling();
    } catch (...) {
      return true;
    }
  }

  void clear_active_goal(
    ActiveGoalType type,
    const rclcpp_action::GoalUUID & goal_id) noexcept
  {
    std::lock_guard<std::mutex> lock(active_goal_mutex_);
    if (active_goal_matches_locked(type, goal_id)) {
      active_goal_type_ = ActiveGoalType::NONE;
      active_goal_id_ = {};
      active_goal_owns_physical_motion_resources_ = false;
      active_goal_physical_outcome_committed_ = false;
    }
  }

  rclcpp_action::CancelResponse cancel_active_goal(
    ActiveGoalType type,
    const rclcpp_action::GoalUUID & goal_id)
  {
    // Keep the binding locked until all global motion resources have been
    // stopped. This prevents a completed goal from releasing the operation
    // slot and a new goal from becoming active midway through a late cancel.
    std::lock_guard<std::mutex> lock(active_goal_mutex_);
    if (!active_goal_matches_locked(type, goal_id)) {
      return rclcpp_action::CancelResponse::REJECT;
    }
    if (client_cancel_disposition(
        active_goal_physical_outcome_committed_) ==
      ClientCancelDisposition::REJECT_AFTER_PHYSICAL_COMMIT)
    {
      return rclcpp_action::CancelResponse::REJECT;
    }
    cancel_requested_.store(true);
    if (active_goal_owns_physical_motion_resources_) {
      stop_active_motion();
      cancel_active_navigation();
      publish_manual_base_stop();
    }
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // The mounted end-effector toolset only serves the control types whose
  // contact tool link it carries: Set A (three-cylinder + two-cylinder)
  // operates buttons and doors, Set B (rotate-button + rocker) operates
  // knobs and switches.  The shared adapter allowlist may span both
  // toolsets, so the catalog must report only the mounted set as
  // physically operable; the per-operation configure_move_group() check
  // still rejects the unmounted set before any TF lookup.
  bool tool_serves_control(std::uint8_t control_type) const
  {
    using Control = xczs_inspection_robot_control::msg::CabinetControl;
    if (toolset_ == "B") {
      return control_type == Control::TYPE_KNOB ||
        control_type == Control::TYPE_SWITCH;
    }
    return control_type == Control::TYPE_BUTTON ||
      control_type == Control::TYPE_DOOR;
  }

  void configure_controls()
  {
    const auto control_ids = declare_parameter<std::vector<std::string>>(
      "control_ids", std::vector<std::string>{});
    if (control_ids.empty()) {
      throw std::invalid_argument(
              "Parameter 'control_ids' must contain at least one control.");
    }
    const auto operable_control_ids =
      declare_parameter<std::vector<std::string>>(
      "operable_control_ids", std::vector<std::string>{});
    const auto inoperable_control_reason = declare_parameter<std::string>(
      "inoperable_control_reason",
      "The control has not passed this robot adapter's complete physical "
      "operation and recovery validation.");
    // A control that IS in operable_control_ids but whose operating tool is
    // not mounted in the current end-effector toolset (e.g. Set A cannot
    // rotate a knob) is not "outside the allowlist" -- report the toolset
    // mismatch instead of the generic inoperable reason.
    const auto toolset_mismatch_reason = declare_parameter<std::string>(
      "toolset_mismatch_reason",
      "The control's operating tool is not mounted in the current end-effector "
      "toolset; the control keeps status display and planning validation only.");
    const std::unordered_set<std::string> operable_controls(
      operable_control_ids.begin(), operable_control_ids.end());
    if (operable_controls.size() != operable_control_ids.size() ||
      inoperable_control_reason.empty())
    {
      throw std::invalid_argument(
              "Robot operable_control_ids must be unique and the shared "
              "inoperable_control_reason must be non-empty.");
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
    const double button_default_spring_stiffness =
      declare_parameter<double>(
      "button_defaults.spring_stiffness", 800.0);
    const double button_default_press_threshold =
      declare_parameter<double>(
      "button_defaults.press_threshold", 0.006);
    const double button_default_force = declare_parameter<double>(
      "button_defaults.default_force", 5.0);

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
    const auto & parameter_overrides =
      get_node_parameters_interface()->get_parameter_overrides();

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

      const std::string prefix = "controls." + control_id + ".";
      auto button = std::make_shared<ButtonSpec>();
      button->id = control_id;
      const auto type_name = declare_parameter<std::string>(
        prefix + "type", "");
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
      const bool is_switch = button->control_type ==
        xczs_inspection_robot_control::msg::CabinetControl::TYPE_SWITCH;
      button->display_name = declare_parameter<std::string>(
        prefix + "display_name", control_id);
      button->joint_name = declare_parameter<std::string>(
        prefix + "joint_name", "");
      button->joint_state_topic = declare_parameter<std::string>(
        prefix + "joint_state_topic",
        control_id + "/joint_states");
      button->pressed_topic = declare_parameter<std::string>(
        prefix + "pressed_topic",
        is_button ? control_id + "/pressed" : "");
      button->state_topic = declare_parameter<std::string>(
        prefix + "state_topic",
        control_id + "/state");
      const auto local_position = declare_parameter<std::vector<double>>(
        prefix + "local_position", std::vector<double>{});
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
      if (is_knob) {
        if (button->state_ids.empty()) {
          throw std::invalid_argument(
                  "Knob '" + control_id +
                  "' must define at least one detent before tool calibration.");
        }
        const std::size_t transition_count =
          button->state_ids.size() * button->state_ids.size();
        button->tool_roll_offsets = declare_parameter<std::vector<double>>(
          prefix + "tool_roll_offsets",
          std::vector<double>(transition_count, 0.0));
        button->detent_release_fraction = declare_parameter<double>(
          prefix + "detent_release_fraction", 1.0);
        if (button->tool_roll_offsets.size() != transition_count ||
          std::any_of(
            button->tool_roll_offsets.begin(),
            button->tool_roll_offsets.end(),
            [](double value) {
              return !std::isfinite(value) ||
              std::abs(value) > std::acos(-1.0);
            }))
        {
          throw std::invalid_argument(
                  "Control '" + control_id +
                  "' tool_roll_offsets must contain one finite [-pi, pi] "
                  "entry for every source/target detent pair.");
        }
        if (!std::isfinite(button->detent_release_fraction) ||
          button->detent_release_fraction <= 0.5 ||
          button->detent_release_fraction > 1.0)
        {
          throw std::invalid_argument(
                  "Control '" + control_id +
                  "' detent_release_fraction must be in (0.5, 1.0].");
        }
      }
      // ready_joint_seed 对按钮与旋钮均有效：七轴臂对同一夹具位姿存在多组
      // IK，把自由空间目标（如 prepress）固定到 r_arm_4<0 的连续分支，可让
      // 接近、按压、撤回全程避开腕部翻转奇点。set_pose_target_with_
      // calibrated_ik_seed 在规划时按 move_group 变量名严格校验种子。
      button->ready_joint_seed_names =
        declare_parameter<std::vector<std::string>>(
        prefix + "ready_joint_seed.joint_names",
        std::vector<std::string>{});
      button->ready_joint_seed_positions =
        declare_parameter<std::vector<double>>(
        prefix + "ready_joint_seed.positions",
        std::vector<double>{});
      {
        const bool has_ready_joint_seed =
          !button->ready_joint_seed_names.empty() ||
          !button->ready_joint_seed_positions.empty();
        std::unordered_set<std::string> unique_seed_names(
          button->ready_joint_seed_names.begin(),
          button->ready_joint_seed_names.end());
        if (has_ready_joint_seed &&
          (button->ready_joint_seed_names.empty() ||
          button->ready_joint_seed_names.size() !=
          button->ready_joint_seed_positions.size() ||
          unique_seed_names.size() != button->ready_joint_seed_names.size() ||
          std::any_of(
            button->ready_joint_seed_names.begin(),
            button->ready_joint_seed_names.end(),
            [](const std::string & name) {return name.empty();}) ||
          std::any_of(
            button->ready_joint_seed_positions.begin(),
            button->ready_joint_seed_positions.end(),
            [](double value) {return !std::isfinite(value);})))
        {
          throw std::invalid_argument(
                  "Control '" + control_id +
                  "' ready_joint_seed must contain equal-length unique "
                  "joint_names and finite positions.");
        }
      }
      if (is_button) {
        button->tool_roll_offset = declare_parameter<double>(
          prefix + "tool_roll_offset", 0.0);
        if (!std::isfinite(button->tool_roll_offset) ||
          std::abs(button->tool_roll_offset) > std::acos(-1.0))
        {
          throw std::invalid_argument(
                  "Button '" + control_id +
                  "' tool_roll_offset must be finite and within [-pi, pi].");
        }
        button->spring_stiffness = declare_parameter<double>(
          prefix + "spring_stiffness", button_default_spring_stiffness);
        button->press_threshold = declare_parameter<double>(
          prefix + "press_threshold", button_default_press_threshold);
        button->default_force = declare_parameter<double>(
          prefix + "default_force", button_default_force);
      }
      const bool in_allowlist = operable_controls.count(control_id) != 0U;
      const bool tool_serves = tool_serves_control(button->control_type);
      button->operable = in_allowlist && tool_serves;
      button->unavailable_reason = declare_parameter<std::string>(
        prefix + "unavailable_reason", "");
      if (button->operable && !button->unavailable_reason.empty()) {
        throw std::invalid_argument(
                "Operable control '" + control_id +
                "' must not declare an unavailable_reason.");
      }
      if (!button->operable && button->unavailable_reason.empty()) {
        // 控件在允许清单内但当前套装未挂载其操作工具时，原因是"套装不匹配"
        // 而非"未列入 operable_control_ids"，用专门文案避免误导 Web 目录。
        button->unavailable_reason = in_allowlist && !tool_serves
          ? toolset_mismatch_reason
          : inoperable_control_reason;
      }
      const std::string station_prefix = prefix + "navigation_station.";
      const std::string station_anchor_parameter =
        station_prefix + "local_anchor";
      const std::string station_axis_parameter =
        station_prefix + "outward_axis";
      const std::string station_standoff_parameter =
        station_prefix + "standoff";
      const bool station_configured =
        parameter_overrides.count(station_anchor_parameter) != 0U ||
        parameter_overrides.count(station_axis_parameter) != 0U ||
        parameter_overrides.count(station_standoff_parameter) != 0U ||
        parameter_overrides.count(station_prefix + "base_yaw_offset") != 0U ||
        parameter_overrides.count(station_prefix + "frame_id") != 0U;
      if (station_configured) {
        if (parameter_overrides.count(station_anchor_parameter) == 0U ||
          parameter_overrides.count(station_axis_parameter) == 0U ||
          parameter_overrides.count(station_standoff_parameter) == 0U)
        {
          throw std::invalid_argument(
                  "Control '" + control_id + "' navigation_station must "
                  "define local_anchor, outward_axis and standoff.");
        }
        ControlNavigationStation station;
        station.local_anchor = checked_vector3(
          declare_parameter<std::vector<double>>(
            station_anchor_parameter, std::vector<double>{}),
          station_anchor_parameter);
        station.outward_axis = checked_vector3(
          declare_parameter<std::vector<double>>(
            station_axis_parameter, std::vector<double>{}),
          station_axis_parameter);
        if (station.outward_axis.length2() <= 1.0e-12) {
          throw std::invalid_argument(
                  "Parameter '" + station_axis_parameter +
                  "' must not be zero.");
        }
        station.outward_axis.normalize();
        station.standoff = declare_parameter<double>(
          station_standoff_parameter, 0.0);
        if (!std::isfinite(station.standoff) || station.standoff <= 0.0) {
          throw std::invalid_argument(
                  "Parameter '" + station_standoff_parameter +
                  "' must be positive and finite.");
        }
        station.base_yaw_offset = declare_parameter<double>(
          station_prefix + "base_yaw_offset", 0.0);
        if (!std::isfinite(station.base_yaw_offset)) {
          throw std::invalid_argument(
                  "Parameter '" + station_prefix +
                  "base_yaw_offset' must be finite.");
        }
        station.frame_id = declare_parameter<std::string>(
          station_prefix + "frame_id", navigation_frame_);
        if (station.frame_id != navigation_frame_) {
          throw std::invalid_argument(
                  "Control '" + control_id +
                  "' navigation_station frame must match navigation_frame '" +
                  navigation_frame_ + "'.");
        }
        if (button->operable && !station_standoff_is_safe(
            station.standoff, docking_base_footprint_,
            docking_base_footprint_padding_, docking_position_tolerance_,
            docking_yaw_tolerance_, station.base_yaw_offset))
        {
          const double minimum_standoff = minimum_safe_station_standoff(
            docking_base_footprint_, docking_base_footprint_padding_,
            docking_position_tolerance_, docking_yaw_tolerance_,
            station.base_yaw_offset);
          throw std::invalid_argument(
                  "Operable control '" + control_id +
                  "' navigation_station.standoff=" +
                  std::to_string(station.standoff) +
                  " m is unsafe for the configured docking-base footprint; "
                  "it must be at least " +
                  std::to_string(minimum_standoff) +
                  " m including footprint padding and docking tolerance.");
        }
        button->navigation_station = std::move(station);
      }
      if (button->operable && !button->navigation_station) {
        throw std::invalid_argument(
                "Operable control '" + control_id +
                "' requires an explicit robot-adapter navigation_station.");
      }
      button->parent_control_id = declare_parameter<std::string>(
        prefix + "parent_control_id", "");
      button->requires_grasp = declare_parameter<bool>(
        prefix + "requires_grasp", !is_button);
      if (!is_button) {
        const double default_grasp_offset = is_knob ?
          grasp_outward_offset_ : (is_switch ? 0.012 : 0.015);
        button->grasp_outward_offset = declare_parameter<double>(
          prefix + "grasp_outward_offset", default_grasp_offset);
        if (!std::isfinite(button->grasp_outward_offset) ||
          button->grasp_outward_offset <= 0.0)
        {
          throw std::invalid_argument(
                  "Control '" + control_id +
                  "' has an invalid grasp outward offset.");
        }
      }
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
      if (is_button &&
        (!std::isfinite(button->spring_stiffness) ||
        button->spring_stiffness <= 0.0 ||
        !std::isfinite(button->press_threshold) ||
        button->press_threshold <= 0.0 ||
        button->press_threshold > button->max_position ||
        !std::isfinite(button->default_force) ||
        button->default_force <= 0.0 ||
        button->default_force >
        button->spring_stiffness * button->max_position))
      {
        throw std::invalid_argument(
                "Button '" + control_id +
                "' has an invalid force profile.");
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
    for (const auto & operable_control : operable_controls) {
      if (buttons_by_id_.count(operable_control) == 0U) {
        throw std::invalid_argument(
                "Unknown operable_control_ids entry '" +
                operable_control + "'.");
      }
    }
  }

  void subscribe_to_button_states()
  {
    for (const auto & button : buttons_in_order_) {
      button_joint_state_subscriptions_.push_back(
        create_subscription<sensor_msgs::msg::JointState>(
          button->joint_state_topic,
          rclcpp::SensorDataQoS(),
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
      if (button->control_type ==
        xczs_inspection_robot_control::msg::CabinetControl::TYPE_BUTTON)
      {
        control.default_force = button->default_force;
        control.min_trigger_force =
          button->spring_stiffness * button->press_threshold;
        control.max_force =
          button->spring_stiffness * button->max_position;
      }
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

  void publish_active_control(const std::string & control_id) noexcept
  {
    try {
      std_msgs::msg::String message;
      message.data = control_id;
      active_control_publisher_->publish(message);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        get_logger(), "Failed to publish the active cabinet control: %s",
        error.what());
    } catch (...) {
      RCLCPP_ERROR(
        get_logger(), "Failed to publish the active cabinet control.");
    }
  }

  void reset_controls(
    const std::shared_ptr<std_srvs::srv::Trigger::Response> & response)
  {
    bool expected = false;
    if (!operation_active_.compare_exchange_strong(expected, true)) {
      response->success = false;
      response->message =
        "Cabinet controls cannot be reset during an active operation.";
      return;
    }

    try {
      if (!reset_physics_client_->wait_for_service(2s)) {
        response->success = false;
        response->message = "Cabinet reset_physics service is unavailable.";
      } else {
        auto future = reset_physics_client_->async_send_request(
          std::make_shared<std_srvs::srv::Trigger::Request>());
        if (future.wait_for(3s) != std::future_status::ready) {
          response->success = false;
          response->message = "Cabinet reset_physics service timed out.";
        } else {
          const auto reset_response = future.get();
          response->success = reset_response->success;
          response->message = reset_response->message;
        }
      }
    } catch (const std::exception & error) {
      response->success = false;
      response->message = std::string("Cabinet reset failed: ") + error.what();
    }
    operation_active_.store(false);
  }

  rclcpp_action::GoalResponse handle_operate_goal(
    const rclcpp_action::GoalUUID & uuid,
    const std::shared_ptr<const OperateCabinetControl::Goal> goal)
  {
    // Only reject when the control itself is unknown.  Command-level
    // validation (type mismatch, unsupported command, bad target) is
    // deferred to ``execute_operate`` so that the action server can run
    // its full planning-validation path and return a structured failure
    // with diagnostic details, rather than silently rejecting the goal.
    const auto control = find_button(goal->control_id);
    if (!control) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    bool expected = false;
    if (!operation_active_.compare_exchange_strong(expected, true)) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    activate_goal(ActiveGoalType::OPERATE, uuid);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_operate_cancel(
    const std::shared_ptr<OperateGoalHandle> goal_handle)
  {
    return goal_handle ?
           cancel_active_goal(
      ActiveGoalType::OPERATE, goal_handle->get_goal_id()) :
           rclcpp_action::CancelResponse::REJECT;
  }

  void handle_operate_accepted(
    const std::shared_ptr<OperateGoalHandle> goal_handle)
  {
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

  double finite_parameter(
    const std::string & name,
    double default_value)
  {
    const double value = declare_parameter<double>(name, default_value);
    if (!std::isfinite(value)) {
      throw std::invalid_argument(
              "Parameter '" + name + "' must be finite.");
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
    const rclcpp_action::GoalUUID & uuid,
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
    activate_goal(ActiveGoalType::PRESS, uuid);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<PressGoalHandle> goal_handle)
  {
    return goal_handle ?
           cancel_active_goal(ActiveGoalType::PRESS, goal_handle->get_goal_id()) :
           rclcpp_action::CancelResponse::REJECT;
  }

  void handle_accepted(const std::shared_ptr<PressGoalHandle> goal_handle)
  {
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
    const auto operation_started_at = std::chrono::steady_clock::now();
    auto result = std::make_shared<PressCabinetButton::Result>();
    const auto button = find_button(goal_handle->get_goal()->button_id);
    std::shared_ptr<MoveGroupInterface> move_group;
    OperationPoses poses;
    ControlStagingPoses staging_poses;
    bool should_attempt_retreat = false;
    bool move_group_ready_for_motion = false;
    bool request_success = false;

    if (!embedded_navigation_request_is_supported(
        goal_handle->get_goal()->navigate_to_staging_pose,
        allow_embedded_navigation_))
    {
      result->success = false;
      result->error_code = PressCabinetButton::Result::NAVIGATION_FAILED;
      result->message = kEmbeddedNavigationDisabledMessage;
      result->max_travel = button ?
        button_snapshot(*button).max_position : 0.0;
      RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
      finish_goal_noexcept(goal_handle, result, false);
      clear_active_goal(ActiveGoalType::PRESS, goal_handle->get_goal_id());
      operation_active_.store(false);
      return;
    }

    try {
      // Bind the button operation's arm group, tip link, contact tool link and
      // transport target to the button type before any MoveIt planning.
      apply_tool_profile(
        xczs_inspection_robot_control::msg::CabinetControl::TYPE_BUTTON);
      if (!button) {
        throw OperationError(
                PressCabinetButton::Result::INTERNAL_ERROR,
                "The accepted cabinet button is no longer configured.");
      }
      const bool should_navigate_to_staging_pose =
        goal_handle->get_goal()->navigate_to_staging_pose;
      acquire_operation_lease(
        goal_handle, ActiveGoalType::PRESS, true);
      if (!should_navigate_to_staging_pose) {
        publish_active_control(button->id);
      }
      reset_max_button_travel(*button);
      publish_feedback(
        goal_handle,
        PressCabinetButton::Feedback::WAITING_FOR_SYSTEM,
        0.02F,
        "Waiting for the cabinet button state.");
      wait_for_fresh_button_state(
        goal_handle, *button, operation_started_at);
      check_cancel(goal_handle);
      latch_cabinet_transform();
      interruptible_hold(goal_handle, planning_scene_settle_seconds_);

      poses = calculate_operation_poses(*button, press_depth_);
      staging_poses = calculate_control_staging_poses(*button);

      move_group = std::make_shared<MoveGroupInterface>(
        shared_from_this(),
        MoveGroupInterface::Options(
          move_group_name_, "robot_description", move_group_namespace_),
        transform_buffer_,
        rclcpp::Duration::from_seconds(system_wait_timeout_));
      check_cancel(goal_handle);
      {
        std::lock_guard<std::mutex> lock(motion_mutex_);
        active_move_group_ = move_group;
      }
      configure_move_group(*move_group);
      move_group_ready_for_motion = true;
      const auto initial_robot_state =
        synchronized_current_robot_state(*move_group);
      verify_tool_tip_calibration_state(*initial_robot_state);

      if (should_navigate_to_staging_pose) {
        publish_feedback(
          goal_handle,
          PressCabinetButton::Feedback::NAVIGATING,
          0.05F,
          "Returning the arm to its safe transport target.");
        plan_and_execute_named_target(
          *move_group, goal_handle, transport_named_target_);
        publish_feedback(
          goal_handle,
          PressCabinetButton::Feedback::NAVIGATING,
          0.08F,
          "Driving the base to the cabinet staging pose.");
        // map->odom is expected to evolve while AMCL observes a moving base.
        // The pre-navigation latch was only needed to build the map-frame
        // goal and to stow safely; do not treat localization corrections as
        // cabinet motion while Nav2 is active.
        clear_latched_cabinet_transform();
        navigate_to_staging_pose(
          goal_handle, staging_poses.navigation_pose);

        // AMCL may update map->odom while Nav2 is moving.  Refresh every
        // planning-frame target after navigation instead of docking and
        // manipulating against the odom snapshot captured before the goal.
        // Keep active_control empty during this settle interval so the
        // planning-scene node is also allowed to adopt the refreshed TF.
        latch_cabinet_transform();
        poses = calculate_operation_poses(*button, press_depth_);
        staging_poses = calculate_control_staging_poses(*button);
        interruptible_hold(goal_handle, planning_scene_settle_seconds_);
        publish_active_control(button->id);
      }
      set_navigation_mode(goal_handle, false);
      publish_feedback(
        goal_handle,
        PressCabinetButton::Feedback::NAVIGATING,
        0.20F,
        "Refining the configured control station from odometry.");
      dock_to_staging_pose(goal_handle, staging_poses.planning_pose);
      interruptible_hold(goal_handle, planning_scene_settle_seconds_);
      verify_staging_pose_before_arm_motion(
        goal_handle, staging_poses.planning_pose);
      const auto predelivery_robot_state =
        synchronized_current_robot_state(*move_group);
      verify_tool_tip_calibration_state(*predelivery_robot_state);

      publish_feedback(
        goal_handle,
        PressCabinetButton::Feedback::MOVING_TO_PREPRESS,
        0.25F,
        "Planning to the button prepress pose.");
      plan_and_execute_pose(
        *move_group, goal_handle, poses.prepress_pose, contact_tool_link_,
        nullptr, button.get());
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
        cartesian_acceleration_scale_ * 0.5,
        button_press_minimum_cartesian_fraction_);

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

      // Linearization point: both the press and release side effects have
      // been physically verified.  A later client cancel must not label this
      // completed press as canceled or stop the safety transport trajectory.
      if (!commit_active_goal_physical_outcome(
          ActiveGoalType::PRESS, goal_handle->get_goal_id()))
      {
        check_cancel(goal_handle);
      }

      publish_feedback(
        goal_handle,
        PressCabinetButton::Feedback::RETRACTING,
        0.99F,
        "Returning the arm to its safe transport target.");
      plan_and_execute_named_target(
        *move_group, goal_handle, transport_named_target_, nullptr, true);

      result->success = true;
      result->error_code = PressCabinetButton::Result::SUCCESS;
      result->message =
        "Pressed and released " + button->id + " successfully.";
      result->max_travel = button_snapshot(*button).max_position;
      request_success = true;
    } catch (const OperationError & error) {
      const bool lease_available = !operation_lease_lost_.load();
      if (lease_available && should_attempt_retreat && move_group && rclcpp::ok()) {
        best_effort_retreat(*move_group, poses.prepress_pose);
      }
      if (lease_available && move_group_ready_for_motion && move_group &&
        rclcpp::ok())
      {
        best_effort_stow(*move_group);
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
      RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
    } catch (const std::exception & error) {
      const bool lease_available = !operation_lease_lost_.load();
      if (lease_available && should_attempt_retreat && move_group && rclcpp::ok()) {
        best_effort_retreat(*move_group, poses.prepress_pose);
      }
      if (lease_available && move_group_ready_for_motion && move_group &&
        rclcpp::ok())
      {
        best_effort_stow(*move_group);
      }
      result->success = false;
      const bool canceled = is_goal_canceling_noexcept(goal_handle);
      result->error_code = operation_lease_lost_.load() ?
        PressCabinetButton::Result::LEASE_LOST : canceled ?
        PressCabinetButton::Result::CANCELED :
        PressCabinetButton::Result::INTERNAL_ERROR;
      result->message = operation_lease_lost_.load() ?
        "The global robot operation lease was lost; all motion was stopped." :
        canceled ?
        "Cabinet button operation was canceled." :
        std::string("Cabinet operation failed: ") + error.what();
      result->max_travel = button ?
        button_snapshot(*button).max_position : 0.0;
      RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
    } catch (...) {
      const bool lease_available = !operation_lease_lost_.load();
      if (lease_available && should_attempt_retreat && move_group && rclcpp::ok()) {
        best_effort_retreat(*move_group, poses.prepress_pose);
      }
      if (lease_available && move_group_ready_for_motion && move_group &&
        rclcpp::ok())
      {
        best_effort_stow(*move_group);
      }
      result->success = false;
      const bool canceled = is_goal_canceling_noexcept(goal_handle);
      result->error_code = operation_lease_lost_.load() ?
        PressCabinetButton::Result::LEASE_LOST : canceled ?
        PressCabinetButton::Result::CANCELED :
        PressCabinetButton::Result::INTERNAL_ERROR;
      result->message = operation_lease_lost_.load() ?
        "The global robot operation lease was lost; all motion was stopped." :
        canceled ?
        "Cabinet button operation was canceled." :
        "Cabinet operation failed with an unknown exception.";
      result->max_travel = button ?
        button_snapshot(*button).max_position : 0.0;
      RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
    }

    clear_latched_cabinet_transform();
    const bool owned_physical_motion_resources =
      active_goal_owns_physical_motion_resources(
      ActiveGoalType::PRESS, goal_handle->get_goal_id());
    if (owned_physical_motion_resources) {
      stop_active_motion();
      cancel_active_navigation();
      request_navigation_mode_without_wait(false);
    }
    {
      std::lock_guard<std::mutex> lock(motion_mutex_);
      active_move_group_.reset();
    }
    // Drop the ownership flag before releasing the lease.  A late cancel may
    // still bind to this goal until its terminal transition, but must not stop
    // a subsequent lease holder.
    relinquish_active_goal_physical_motion_resources(
      ActiveGoalType::PRESS, goal_handle->get_goal_id());
    release_operation_lease_noexcept();
    publish_active_control("");
    const bool goal_succeeded =
      finish_goal_noexcept(goal_handle, result, request_success);
    clear_active_goal(ActiveGoalType::PRESS, goal_handle->get_goal_id());
    operation_active_.store(false);
    if (goal_succeeded &&
      request_success && button)
    {
      RCLCPP_INFO(
        get_logger(), "Cabinet button %s operation succeeded (max %.2f mm).",
        button->id.c_str(), result->max_travel * 1000.0);
    }
  }

  void execute_operate(
    const std::shared_ptr<OperateGoalHandle> goal_handle) noexcept
  {
    const auto operation_started_at = std::chrono::steady_clock::now();
    auto result = std::make_shared<OperateCabinetControl::Result>();
    result->diagnostic_stage = "preflight";
    const auto control = find_button(goal_handle->get_goal()->control_id);
    std::shared_ptr<MoveGroupInterface> move_group;
    OperationPoses button_poses;
    RotaryOperationPoses rotary_poses;
    std::optional<ControlStagingPoses> staging_poses;
    bool should_attempt_retreat = false;
    bool grasp_attached = false;
    bool request_success = false;
    DoorArcProgress door_arc_progress;
    double target_position = 0.0;
    double rotary_tool_roll_offset = 0.0;
    double button_press_depth = 0.0;
    bool button_should_trigger = false;
    bool is_button = false;
    const bool validation_only = control &&
      requires_planning_only_validation(control->operable);
    const auto preparation_policy = operation_preparation_policy(
      validation_only,
      goal_handle->get_goal()->navigate_to_staging_pose,
      control && control->navigation_station.has_value());
    std::string target_state;
    std::unordered_map<std::string, std::string> expected_parent_states;

    if (!embedded_navigation_request_is_supported(
        goal_handle->get_goal()->navigate_to_staging_pose,
        allow_embedded_navigation_))
    {
      result->success = false;
      result->error_code = OperateCabinetControl::Result::NAVIGATION_FAILED;
      result->message = kEmbeddedNavigationDisabledMessage;
      RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
      finish_operate_goal_noexcept(goal_handle, result, false);
      clear_active_goal(ActiveGoalType::OPERATE, goal_handle->get_goal_id());
      operation_active_.store(false);
      return;
    }

    try {
      if (!control) {
        throw GenericOperationError(
                OperateCabinetControl::Result::INVALID_CONTROL,
                "The accepted cabinet control is no longer configured.");
      }
      // Bind this operation's arm group, tip link, contact tool link and
      // transport target to the control's type before any MoveIt planning.
      apply_tool_profile(control->control_type);
      if (validation_only) {
        result->policy_reason = control->unavailable_reason.empty() ?
          "The selected control has not passed this robot adapter's complete "
          "physical operation and recovery validation." :
          control->unavailable_reason;
        result->diagnostic_stage = "preflight";
      }
      is_button = control->control_type ==
        xczs_inspection_robot_control::msg::CabinetControl::TYPE_BUTTON;
      // Command-level validation: check that the command is compatible with
      // the control type and that its parameters are valid.  This runs inside
      // the accepted goal so that the caller receives a structured failure
      // with diagnostic details instead of a silent goal rejection.
      {
        bool command_valid = false;
        std::string command_reason;
        if (is_button) {
          command_valid = goal_handle->get_goal()->command ==
            OperateCabinetControl::Goal::COMMAND_PRESS;
          if (!command_valid) {
            command_reason = "Buttons only support the 'press' command.";
          }
        } else {
          const auto cmd = goal_handle->get_goal()->command;
          if (cmd == OperateCabinetControl::Goal::COMMAND_SET_STATE) {
            command_valid = std::find(
              control->state_ids.begin(), control->state_ids.end(),
              goal_handle->get_goal()->target_state) !=
              control->state_ids.end();
            if (!command_valid) {
              command_reason = "target_state '" +
                goal_handle->get_goal()->target_state +
                "' is not in the control's state list.";
            }
          } else if (cmd ==
            OperateCabinetControl::Goal::COMMAND_SET_POSITION)
          {
            command_valid = goal_handle->get_goal()->use_target_position &&
              std::isfinite(goal_handle->get_goal()->target_position) &&
              goal_handle->get_goal()->target_position >=
              control->min_position &&
              goal_handle->get_goal()->target_position <=
              control->max_position &&
              std::any_of(
              control->state_positions.begin(),
              control->state_positions.end(),
              [this, &goal_handle](double preset) {
                return std::abs(
                  preset - goal_handle->get_goal()->target_position) <=
                target_tolerance_;
              });
            if (!command_valid) {
              command_reason = "target_position is out of range or does " \
                "not match a configured detent.";
            }
          } else if (cmd == OperateCabinetControl::Goal::COMMAND_TOGGLE) {
            command_valid = control->state_ids.size() >= 2U;
            if (!command_valid) {
              command_reason = "Toggle requires at least 2 states.";
            }
          } else {
            command_reason = "Unknown command code " +
              std::to_string(static_cast<int>(cmd)) + ".";
          }
        }
        if (!command_valid) {
          result->success = false;
          result->error_code =
            OperateCabinetControl::Result::UNSUPPORTED_COMMAND;
          result->message = "Command not valid for control '" +
            control->id + "': " + command_reason;
          result->validation_performed = true;
          result->operation_executed = false;
          result->diagnostic_stage = "command_validation";
          result->policy_reason = command_reason;
          RCLCPP_WARN(
            get_logger(), "%s", result->message.c_str());
          finish_operate_goal_noexcept(goal_handle, result, false);
          clear_active_goal(
            ActiveGoalType::OPERATE, goal_handle->get_goal_id());
          operation_active_.store(false);
          return;
        }
      }
      if (is_button) {
        const double requested_force = goal_handle->get_goal()->force == 0.0 ?
          control->default_force : goal_handle->get_goal()->force;
        result->requested_force = requested_force;
        const double maximum_force =
          control->spring_stiffness * control->max_position;
        if (!std::isfinite(requested_force) || requested_force <= 0.0 ||
          requested_force > maximum_force)
        {
          throw GenericOperationError(
                  OperateCabinetControl::Result::INVALID_FORCE,
                  "Button force must be finite and within (0, " +
                  std::to_string(maximum_force) + "] N.");
        }
        button_press_depth = requested_force / control->spring_stiffness;
        button_should_trigger =
          button_press_depth + 1.0e-9 >= control->press_threshold;
      }
      acquire_operation_lease(
        goal_handle,
        ActiveGoalType::OPERATE,
        preparation_policy.requires_physical_motion_resources);
      if (!preparation_policy.execute_embedded_navigation) {
        publish_active_control(control->id);
      }
      if (!validation_only) {
        reset_max_button_travel(*control);
      }
      publish_operate_feedback(
        goal_handle,
        OperateCabinetControl::Feedback::WAITING_FOR_SYSTEM,
        0.02F, 0.0, "Waiting for a fresh physical control state.");
      wait_for_fresh_button_state(
        goal_handle, *control, operation_started_at);
      check_cancel(goal_handle);
      const auto initial_state = button_snapshot(*control);
      result->initial_position = initial_state.position;
      if (control->control_type ==
        xczs_inspection_robot_control::msg::CabinetControl::TYPE_DOOR)
      {
        door_arc_progress.initial_position = initial_state.position;
      }
      for (const auto * ancestor : control_ancestors(*control)) {
        wait_for_fresh_button_state(
          goal_handle, *ancestor, operation_started_at);
        expected_parent_states.emplace(
          ancestor->id, button_snapshot(*ancestor).state_id);
      }
      std::tie(target_position, target_state) = resolve_operation_target(
        *control, *goal_handle->get_goal(), initial_state);
      if (is_button) {
        target_position = button_press_depth;
      } else if (control->control_type ==
        xczs_inspection_robot_control::msg::CabinetControl::TYPE_KNOB)
      {
        const auto source_index = control_state_index(*control, initial_state);
        const auto target_index = control_state_index(*control, target_state);
        if (!rotary_transition_is_adjacent(source_index, target_index)) {
          throw GenericOperationError(
                  OperateCabinetControl::Result::UNSUPPORTED_COMMAND,
                  "旋钮 '" + control->id +
                  "' 不支持一次跨越两个档位；请先切换到中档，再执行下一步。");
        }
        rotary_tool_roll_offset = control->tool_roll_offsets.at(
          rotary_transition_matrix_index(
            source_index, target_index, control->state_ids.size()));
      }
      latch_cabinet_transform();

      if (is_button) {
        button_poses = calculate_operation_poses(
          *control, button_press_depth);
      } else {
        rotary_poses = calculate_rotary_operation_poses(
          *control, initial_state.position, rotary_tool_roll_offset);
      }
      if (control->navigation_station &&
        (preparation_policy.execute_embedded_navigation ||
        preparation_policy.execute_precision_docking))
      {
        staging_poses = calculate_control_staging_poses(*control);
      }
      if (preparation_policy.execute_embedded_navigation && !staging_poses) {
        throw GenericOperationError(
                OperateCabinetControl::Result::NAVIGATION_FAILED,
                "Embedded navigation requires an explicit robot-adapter "
                "navigation_station for control '" + control->id + "'.");
      }

      move_group = std::make_shared<MoveGroupInterface>(
        shared_from_this(),
        MoveGroupInterface::Options(
          move_group_name_, "robot_description", move_group_namespace_),
        transform_buffer_,
        rclcpp::Duration::from_seconds(system_wait_timeout_));
      {
        std::lock_guard<std::mutex> lock(motion_mutex_);
        active_move_group_ = move_group;
      }
      configure_move_group(*move_group);
      const auto current_robot_state =
        synchronized_current_robot_state(*move_group);
      verify_tool_tip_calibration_state(*current_robot_state);

      if (validation_only) {
        result->operation_executed = false;
        result->validation_performed = true;
        validate_inoperable_control_path(
          *move_group, goal_handle, *control, initial_state,
          target_position, button_poses, rotary_poses,
          rotary_tool_roll_offset, *current_robot_state, result);
        result->diagnostic_stage = "complete";
        result->path_fraction = 1.0;
        result->required_fraction = 1.0;
        result->moveit_error_code =
          moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
        throw GenericOperationError(
                OperateCabinetControl::Result::UNREACHABLE,
                "实时 MoveIt 运动学与碰撞规划验证全部通过，但没有发送任何"
                "机械臂轨迹、导航、精确对接、抓取或柜体写入命令；按钮触发、"
                "档位到达、串扰和安全恢复仍未经过物理闭环验证，因此继续受"
                "机器人适配层安全策略限制：" + result->policy_reason);
      }

      if (!physical_motion_is_permitted(
          control->operable, result->validation_performed))
      {
        throw GenericOperationError(
                OperateCabinetControl::Result::INTERNAL_ERROR,
                "Robot-adapter policy blocked physical cabinet motion.");
      }

      if (preparation_policy.execute_embedded_navigation) {
        result->diagnostic_stage = "navigation";
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::NAVIGATING,
          0.05F, target_position,
          "Returning the arm to its safe transport target.");
        plan_and_execute_named_target(
          *move_group, goal_handle, transport_named_target_,
          &result->operation_executed);
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::NAVIGATING,
          0.07F, target_position,
          "Driving the base to the cabinet staging pose.");
        // Nav2 may update map->odom during navigation.  The physical cabinet
        // remains fixed in odom/world, so release the pre-navigation latch and
        // establish a new manipulation latch only after Nav2 has stopped.
        clear_latched_cabinet_transform();
        navigate_to_staging_pose(
          goal_handle, staging_poses->navigation_pose);

        // Embedded Nav2 can change map->odom through localization updates.
        // Rebuild all odom-frame manipulation targets from the post-navigation
        // TF.  active_control is deliberately still empty here so the
        // planning scene can refresh its matching cabinet transform too.
        latch_cabinet_transform();
        if (is_button) {
          button_poses = calculate_operation_poses(
            *control, button_press_depth);
        } else {
          rotary_poses = calculate_rotary_operation_poses(
            *control, initial_state.position, rotary_tool_roll_offset);
        }
        staging_poses = calculate_control_staging_poses(*control);
        interruptible_hold(goal_handle, planning_scene_settle_seconds_);
        publish_active_control(control->id);
      }
      if (preparation_policy.enter_manual_base_mode) {
        result->diagnostic_stage = "docking";
        set_navigation_mode(goal_handle, false);
      }
      if (preparation_policy.execute_precision_docking) {
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::DOCKING,
          0.18F, target_position,
          "Refining the configured control station from odometry.");
        dock_to_staging_pose(goal_handle, staging_poses->planning_pose);
      }
      if (preparation_policy.wait_for_scene_settle) {
        interruptible_hold(goal_handle, planning_scene_settle_seconds_);
      }
      if (preparation_policy.execute_precision_docking) {
        verify_staging_pose_before_arm_motion(
          goal_handle, staging_poses->planning_pose);
      }
      const auto predelivery_robot_state =
        synchronized_current_robot_state(*move_group);
      verify_tool_tip_calibration_state(*predelivery_robot_state);

      if (is_button) {
        result->diagnostic_stage = "ready";
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::MOVING_TO_READY,
          0.25F, target_position,
          "Planning to the button ready pose.");
        plan_and_execute_pose(
          *move_group, goal_handle, button_poses.prepress_pose,
          contact_tool_link_, &result->operation_executed, control.get());
        should_attempt_retreat = true;
        result->diagnostic_stage = "approach";
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::APPROACHING,
          0.47F, target_position,
          "Approaching the button along its travel axis.");
        execute_cartesian_path(
          *move_group, goal_handle, {button_poses.contact_pose},
          cartesian_velocity_scale_, cartesian_acceleration_scale_,
          0.99, &result->operation_executed);
        const auto transition_sequence =
          button_snapshot(*control).pressed_transition_sequence;
        result->diagnostic_stage = "manipulation";
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::MANIPULATING,
          0.65F, target_position,
          "Pressing the button through its physical travel.");
        execute_cartesian_path(
          *move_group, goal_handle, {button_poses.pressed_pose},
          cartesian_velocity_scale_ * 0.5,
          cartesian_acceleration_scale_ * 0.5,
          button_press_minimum_cartesian_fraction_,
          &result->operation_executed);
        track_button_force(
          *move_group, goal_handle, *control, button_press_depth,
          &result->operation_executed);
        result->diagnostic_stage = "verification";
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::VERIFYING,
          0.77F, target_position,
          button_should_trigger ?
          "Verifying the Gazebo press transition." :
          "Measuring the physical travel produced by the requested force.");
        if (button_should_trigger) {
          if (!wait_for_pressed_state(
              goal_handle, *control, true, press_detection_timeout_,
              transition_sequence))
          {
            throw GenericOperationError(
                    OperateCabinetControl::Result::CONTACT_DETECTION_TIMEOUT,
                    "The requested force should have triggered the button, "
                    "but no physical press transition was detected.");
          }
          result->button_triggered = true;
        } else {
          interruptible_hold(goal_handle, press_hold_seconds_);
          result->button_triggered = button_snapshot(*control).pressed;
        }
        if (button_should_trigger) {
          interruptible_hold(goal_handle, press_hold_seconds_);
        }
        const auto release_sequence =
          button_snapshot(*control).pressed_transition_sequence;
        result->diagnostic_stage = "retreat";
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::RETREATING,
          0.88F, 0.0, "Retracting the probe from the button.");
        execute_cartesian_path(
          *move_group, goal_handle,
          {button_poses.contact_pose, button_poses.prepress_pose},
          cartesian_velocity_scale_, cartesian_acceleration_scale_,
          0.99, &result->operation_executed);
        should_attempt_retreat = false;
        result->diagnostic_stage = "verification";
        if (result->button_triggered && !wait_for_pressed_state(
            goal_handle, *control, false, release_detection_timeout_,
            release_sequence))
        {
          throw GenericOperationError(
                  OperateCabinetControl::Result::RELEASE_FAILED,
                  "The button did not return to its released state.");
        }
        const auto measured_state = button_snapshot(*control);
        result->peak_position = measured_state.max_position;
        result->estimated_force =
          measured_state.max_position * control->spring_stiffness;
        if (!button_should_trigger) {
          throw GenericOperationError(
                  OperateCabinetControl::Result::INSUFFICIENT_FORCE,
                  "力度不足：请求 " +
                  std::to_string(result->requested_force) +
                  " N，只能产生约 " +
                  std::to_string(button_press_depth * 1000.0) +
                  " mm 位移；按钮至少需要 " +
                  std::to_string(
                    control->spring_stiffness * control->press_threshold) +
                  " N（" +
                  std::to_string(control->press_threshold * 1000.0) +
                  " mm）才能触发。");
        }
        // The press and spring return are both verified above.  Commit before
        // safety transport so a late client cancel cannot cause a duplicate
        // physical press on retry.
        if (!commit_active_goal_physical_outcome(
            ActiveGoalType::OPERATE, goal_handle->get_goal_id()))
        {
          check_cancel(goal_handle);
        }
      } else {
        result->diagnostic_stage = "ready";
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::MOVING_TO_READY,
          0.25F, target_position,
          "Planning the probe to the control ready pose.");
        plan_and_execute_pose(
          *move_group, goal_handle, rotary_poses.ready_pose,
          contact_tool_link_, &result->operation_executed, control.get());
        should_attempt_retreat = true;
        result->diagnostic_stage = "approach";
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::APPROACHING,
          0.43F, target_position,
          "Approaching the physical grasp point.");
        if (control->control_type ==
          xczs_inspection_robot_control::msg::CabinetControl::TYPE_DOOR)
        {
          // A pose-only OMPL plan can select a ready-pose IK branch that
          // cannot finish the final inward Cartesian approach.  Require an
          // exact collision-checked plan to a near-grasp pose first, then
          // preserve a short straight-line final approach.
          plan_and_execute_pose(
            *move_group, goal_handle, rotary_poses.door_pregrasp_pose,
            contact_tool_link_, &result->operation_executed);
        }
        execute_cartesian_path(
          *move_group, goal_handle, {rotary_poses.grasp_pose},
          cartesian_velocity_scale_ * 0.5,
          cartesian_acceleration_scale_ * 0.5,
          0.99, &result->operation_executed);
        result->diagnostic_stage = "grasp";
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::GRASPING,
          0.52F, target_position,
          "Attaching the probe at the verified near-distance grasp point.");
        set_control_grasp(goal_handle, control->id, true);
        grasp_attached = true;
        // Let the transient grasp-active notification reach the planar
        // stabilizer before the arm starts driving the physical constraint.
        interruptible_hold(goal_handle, grasp_attach_settle_duration_);
        // Doors and calibrated knobs are over-center mechanisms.  Moving
        // safely beyond the next midpoint lets the physical detent spring
        // finish the requested travel after release, without demanding an
        // unreachable full-angle Cartesian wrist arc.
        const double manipulation_position = rotary_manipulation_position(
          *control, initial_state.position, target_position);
        const auto waypoints = calculate_rotation_waypoints(
          *control, initial_state.position, manipulation_position,
          rotary_tool_roll_offset);
        result->diagnostic_stage = "manipulation";
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::MANIPULATING,
          0.62F, target_position,
          "Following the control joint arc without commanding the joint.");
        if (control->control_type ==
          xczs_inspection_robot_control::msg::CabinetControl::TYPE_DOOR)
        {
          // A long door arc can keep MoveIt's Cartesian interpolation on an
          // IK branch that becomes invalid near the end of the sweep.  Short
          // complete segments preserve physical continuity while allowing
          // each next segment to start from the measured robot state.
          door_arc_progress.in_progress = true;
          door_arc_progress.completed_waypoints = 0U;
          door_arc_progress.total_waypoints = waypoints.size();
          execute_segmented_cartesian_path(
            *move_group, goal_handle, waypoints,
            static_cast<std::size_t>(door_cartesian_segment_waypoints_),
            cartesian_velocity_scale_ * 0.5,
            cartesian_acceleration_scale_ * 0.5,
            door_arc_progress.completed_waypoints,
            &result->operation_executed);
          door_arc_progress.in_progress = false;
        } else {
          execute_cartesian_path(
            *move_group, goal_handle, waypoints,
            cartesian_velocity_scale_ * 0.5,
            cartesian_acceleration_scale_ * 0.5,
            0.99, &result->operation_executed);
        }
        if (control->control_type ==
          xczs_inspection_robot_control::msg::CabinetControl::TYPE_DOOR &&
          std::abs(target_position - initial_state.position) >
          target_tolerance_)
        {
          publish_operate_feedback(
            goal_handle,
            OperateCabinetControl::Feedback::MANIPULATING,
            0.71F, target_position,
            "Verifying that the physical door crossed its safe release "
            "position.");
          wait_for_door_release_position(
            goal_handle, *control, initial_state.position,
            manipulation_position, std::chrono::steady_clock::now());
        }
        const bool uses_partial_knob_release =
          control->control_type ==
          xczs_inspection_robot_control::msg::CabinetControl::TYPE_KNOB &&
          control->detent_release_fraction<1.0 &&
            std::abs(target_position - initial_state.position)>
          target_tolerance_;
        if (uses_partial_knob_release) {
          publish_operate_feedback(
            goal_handle,
            OperateCabinetControl::Feedback::MANIPULATING,
            0.71F, target_position,
            "Verifying that the physical knob latched the requested "
            "adjacent detent.");
          wait_for_knob_release_position(
            goal_handle, *control, initial_state.position, target_position,
            target_state, std::chrono::steady_clock::now());
        }
        const auto release_hold_started = std::chrono::steady_clock::now();
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::MANIPULATING,
          0.74F, target_position,
          "Holding the target detent briefly before grasp release.");
        interruptible_hold(goal_handle, grasp_release_settle_duration_);
        if (control->control_type ==
          xczs_inspection_robot_control::msg::CabinetControl::TYPE_DOOR &&
          std::abs(target_position - initial_state.position) >
          target_tolerance_)
        {
          // A single threshold crossing is not sufficient if the physical
          // door rebounds during the release hold.  Require a new sample on
          // the safe side immediately before detaching the grasp.
          wait_for_door_release_position(
            goal_handle, *control, initial_state.position,
            manipulation_position, release_hold_started);
        }
        if (uses_partial_knob_release) {
          // Require a fresh target-side sample immediately before detach so a
          // transient crossing cannot be mistaken for a latched detent.
          wait_for_knob_release_position(
            goal_handle, *control, initial_state.position, target_position,
            target_state, release_hold_started);
        }
        const bool is_door = control->control_type ==
          xczs_inspection_robot_control::msg::CabinetControl::TYPE_DOOR;
        const double release_clearance = is_door ?
          door_release_clearance_ : prepress_distance_;
        const auto target_ready = calculate_rotary_tool_pose(
          *control, manipulation_position, release_clearance, false,
          rotary_tool_roll_offset);
        std::chrono::steady_clock::time_point released_at;
        if (is_door) {
          // Compliant door grasping couples only tool rotation to the hinge.
          // Keep that coupling active while translating the tool beyond the
          // remaining door sweep.  Releasing first lets the closer drive the
          // visible panel through the retreating arm.
          result->diagnostic_stage = "retreat";
          publish_operate_feedback(
            goal_handle,
            OperateCabinetControl::Feedback::RETREATING,
            0.78F, target_position,
            "Holding the door angle while clearing its remaining sweep.");
          execute_cartesian_path(
            *move_group, goal_handle, {target_ready},
            cartesian_velocity_scale_, cartesian_acceleration_scale_,
            0.99, &result->operation_executed);
          should_attempt_retreat = false;
          result->diagnostic_stage = "release";
          publish_operate_feedback(
            goal_handle,
            OperateCabinetControl::Feedback::RELEASING,
            0.84F, target_position,
            "Releasing the door after the tool cleared its sweep.");
          set_control_grasp(goal_handle, control->id, false);
          grasp_attached = false;
          released_at = std::chrono::steady_clock::now();
        } else {
          result->diagnostic_stage = "release";
          publish_operate_feedback(
            goal_handle,
            OperateCabinetControl::Feedback::RELEASING,
            0.78F, target_position,
            "Releasing the physical grasp at the requested detent.");
          set_control_grasp(goal_handle, control->id, false);
          grasp_attached = false;
          released_at = std::chrono::steady_clock::now();
          result->diagnostic_stage = "retreat";
          publish_operate_feedback(
            goal_handle,
            OperateCabinetControl::Feedback::RETREATING,
            0.84F, target_position,
            "Retreating before verifying the released control.");
          execute_cartesian_path(
            *move_group, goal_handle, {target_ready},
            cartesian_velocity_scale_, cartesian_acceleration_scale_,
            0.99, &result->operation_executed);
          should_attempt_retreat = false;
        }
        result->diagnostic_stage = "verification";
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::VERIFYING,
          0.90F, target_position,
          "Waiting for articulated parent controls to settle.");
        wait_for_parent_controls_stable(
          goal_handle, *control, expected_parent_states);
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::VERIFYING,
          0.94F, target_position,
          "Verifying target position and low-velocity stability after "
          "release.");
        wait_for_target_stable(
          goal_handle, *control, target_position, target_state, released_at);
        // The released knob/switch/door is stable at its requested detent.
        // This is the physical side-effect commit; transport remains a
        // fallible safety phase but is no longer client-cancelable.
        if (!commit_active_goal_physical_outcome(
            ActiveGoalType::OPERATE, goal_handle->get_goal_id()))
        {
          check_cancel(goal_handle);
        }
      }

      result->diagnostic_stage = "transport";
      publish_operate_feedback(
        goal_handle,
        OperateCabinetControl::Feedback::RETREATING,
        0.98F, target_position,
        "Returning the arm to its safe transport target.");
      plan_and_execute_named_target(
        *move_group, goal_handle, transport_named_target_,
        &result->operation_executed, true);

      const auto final_state = button_snapshot(*control);
      if (!result->operation_executed) {
        throw GenericOperationError(
                OperateCabinetControl::Result::INTERNAL_ERROR,
                "Cabinet operation reached final verification without "
                "issuing a physical motion command.");
      }
      result->success = true;
      result->error_code = OperateCabinetControl::Result::SUCCESS;
      result->message = "Operated " + control->id + " successfully.";
      // ``diagnostic_stage`` describes the origin of a physical-operation
      // failure.  Keep successful physical results empty so clients do not
      // mistake them for planning-only validation diagnostics.
      result->diagnostic_stage.clear();
      result->final_position = final_state.position;
      result->peak_position = final_state.max_position;
      result->final_state = final_state.state_id;
      if (is_button) {
        result->estimated_force =
          final_state.max_position * control->spring_stiffness;
        result->button_triggered =
          result->button_triggered ||
          final_state.max_position + 1.0e-9 >= control->press_threshold;
      }
      request_success = true;
    } catch (const GenericOperationError & error) {
      result->success = false;
      result->error_code = error.error_code;
      result->message = error.what();
      if (validation_only) {
        snapshot_planning_only_result(result, control);
      } else {
        recover_failed_operate_goal(
          result, control, move_group, should_attempt_retreat,
          grasp_attached, door_arc_progress, rotary_tool_roll_offset,
          is_operate_goal_canceling(goal_handle));
      }
    } catch (const OperationError & error) {
      result->success = false;
      result->error_code = map_legacy_error_code(error.error_code);
      result->message = error.what();
      if (validation_only) {
        snapshot_planning_only_result(result, control);
      } else {
        recover_failed_operate_goal(
          result, control, move_group, should_attempt_retreat,
          grasp_attached, door_arc_progress, rotary_tool_roll_offset,
          is_operate_goal_canceling(goal_handle));
      }
    } catch (const std::exception & error) {
      result->success = false;
      result->error_code = operation_lease_lost_.load() ?
        OperateCabinetControl::Result::LEASE_LOST :
        is_operate_goal_canceling(goal_handle) ?
        OperateCabinetControl::Result::CANCELED :
        OperateCabinetControl::Result::INTERNAL_ERROR;
      result->message = operation_lease_lost_.load() ?
        validation_only ?
        "The global robot operation lease was lost; planning-only validation "
        "was canceled without publishing a physical motion command." :
        "The global robot operation lease was lost; all motion was stopped." :
        is_operate_goal_canceling(goal_handle) ?
        "Cabinet operation was canceled." :
        std::string("Cabinet operation failed: ") + error.what();
      if (validation_only) {
        snapshot_planning_only_result(result, control);
      } else {
        recover_failed_operate_goal(
          result, control, move_group, should_attempt_retreat,
          grasp_attached, door_arc_progress, rotary_tool_roll_offset,
          is_operate_goal_canceling(goal_handle));
      }
    } catch (...) {
      result->success = false;
      result->error_code = operation_lease_lost_.load() ?
        OperateCabinetControl::Result::LEASE_LOST :
        OperateCabinetControl::Result::INTERNAL_ERROR;
      result->message = operation_lease_lost_.load() ?
        validation_only ?
        "The global robot operation lease was lost; planning-only validation "
        "was canceled without publishing a physical motion command." :
        "The global robot operation lease was lost; all motion was stopped." :
        "Cabinet operation failed with an unknown error.";
      if (validation_only) {
        snapshot_planning_only_result(result, control);
      } else {
        recover_failed_operate_goal(
          result, control, move_group, should_attempt_retreat,
          grasp_attached, door_arc_progress, rotary_tool_roll_offset,
          is_operate_goal_canceling(goal_handle));
      }
    }

    clear_latched_cabinet_transform();
    const bool owned_physical_motion_resources =
      active_goal_owns_physical_motion_resources(
      ActiveGoalType::OPERATE, goal_handle->get_goal_id());
    if (owned_physical_motion_resources) {
      stop_active_motion();
      cancel_active_navigation();
      if (preparation_policy.enter_manual_base_mode) {
        request_navigation_mode_without_wait(false);
      }
    }
    {
      std::lock_guard<std::mutex> lock(motion_mutex_);
      active_move_group_.reset();
    }
    relinquish_active_goal_physical_motion_resources(
      ActiveGoalType::OPERATE, goal_handle->get_goal_id());
    release_operation_lease_noexcept();
    publish_active_control("");
    finish_operate_goal_noexcept(goal_handle, result, request_success);
    clear_active_goal(ActiveGoalType::OPERATE, goal_handle->get_goal_id());
    operation_active_.store(false);
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

  std::size_t control_state_index(
    const ButtonSpec & control,
    const std::string & state_id) const
  {
    const auto iterator = std::find(
      control.state_ids.begin(), control.state_ids.end(), state_id);
    if (iterator == control.state_ids.end()) {
      throw GenericOperationError(
              OperateCabinetControl::Result::INTERNAL_ERROR,
              "Control '" + control.id +
              "' resolved an unknown target detent '" + state_id + "'.");
    }
    return static_cast<std::size_t>(
      std::distance(control.state_ids.begin(), iterator));
  }

  std::size_t control_state_index(
    const ButtonSpec & control,
    const ButtonSnapshot & state) const
  {
    const auto iterator = std::find(
      control.state_ids.begin(), control.state_ids.end(), state.state_id);
    if (iterator != control.state_ids.end()) {
      return static_cast<std::size_t>(
        std::distance(control.state_ids.begin(), iterator));
    }
    return static_cast<std::size_t>(std::distance(
             control.state_positions.begin(),
             std::min_element(
               control.state_positions.begin(), control.state_positions.end(),
               [&state](double left, double right) {
                 return std::abs(state.position - left) <
                 std::abs(state.position - right);
               })));
  }

  tf2::Transform lookup_cabinet_transform(
    const tf2::Duration & timeout = tf2::durationFromSec(0.2)) const
  {
    if (require_cabinet_pose_valid_ &&
      (!cabinet_pose_valid_received_.load() || !cabinet_pose_valid_.load()))
    {
      throw OperationError(
              PressCabinetButton::Result::NOT_READY,
              "The cabinet pose authority has no fresh valid pose.");
    }
    try {
      const auto transform = transform_buffer_->lookupTransform(
        planning_frame_, cabinet_frame_, tf2::TimePointZero, timeout);
      tf2::Transform result;
      tf2::fromMsg(transform.transform, result);
      result.getRotation().normalize();
      return result;
    } catch (const tf2::TransformException & error) {
      throw OperationError(
              PressCabinetButton::Result::NOT_READY,
              "Cabinet TF '" + planning_frame_ + "' -> '" + cabinet_frame_ +
              "' is unavailable: " + error.what());
    }
  }

  void latch_cabinet_transform()
  {
    const auto transform = lookup_cabinet_transform(
      tf2::durationFromSec(system_wait_timeout_));
    std::lock_guard<std::mutex> lock(cabinet_transform_mutex_);
    latched_cabinet_transform_ = transform;
    cabinet_transform_latched_ = true;
  }

  void clear_latched_cabinet_transform() noexcept
  {
    std::lock_guard<std::mutex> lock(cabinet_transform_mutex_);
    cabinet_transform_latched_ = false;
  }

  tf2::Transform resolve_cabinet_transform()
  {
    {
      std::lock_guard<std::mutex> lock(cabinet_transform_mutex_);
      if (cabinet_transform_latched_) {
        return latched_cabinet_transform_;
      }
    }
    return lookup_cabinet_transform();
  }

  void verify_latched_cabinet_transform() const
  {
    tf2::Transform latched;
    {
      std::lock_guard<std::mutex> lock(cabinet_transform_mutex_);
      if (!cabinet_transform_latched_) {
        return;
      }
      latched = latched_cabinet_transform_;
    }
    const auto current = lookup_cabinet_transform(tf2::durationFromSec(0.05));
    const double translation_change =
      latched.getOrigin().distance(current.getOrigin());
    const double rotation_change = latched.getRotation().angleShortestPath(
      current.getRotation());
    if (translation_change > cabinet_pose_translation_tolerance_ ||
      rotation_change > cabinet_pose_rotation_tolerance_)
    {
      throw OperationError(
              PressCabinetButton::Result::NOT_READY,
              "Cabinet pose moved by " + std::to_string(translation_change) +
              " m / " + std::to_string(rotation_change) +
              " rad during operation; motion was stopped.");
    }
  }

  std::vector<const ButtonSpec *> control_ancestors(
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
    return ancestors;
  }

  tf2::Transform resolve_control_parent_transform(
    const ButtonSpec & control,
    bool require_stable = true) const
  {
    const auto ancestors = control_ancestors(control);

    tf2::Transform result;
    result.setIdentity();
    const auto now = std::chrono::steady_clock::now();
    for (const auto * ancestor : ancestors) {
      const auto state = button_snapshot(*ancestor);
      const bool fresh = structured_control_state_is_usable(
        state.structured_received, state.valid, true,
        std::chrono::duration<double>(
          now - state.structured_received_at).count() <=
        button_state_timeout_);
      if (!fresh) {
        throw GenericOperationError(
                OperateCabinetControl::Result::NOT_READY,
                "Parent control '" + ancestor->id +
                "' does not have a fresh valid physical state.");
      }
      if (require_stable &&
        (state.in_motion ||
        std::abs(state.velocity) > stable_velocity_tolerance_))
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
    const ButtonSpec & control,
    bool require_stable_parent = true) const
  {
    const tf2::Transform parent = resolve_control_parent_transform(
      control, require_stable_parent);
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
    double outward_offset,
    bool require_stable_parent = true,
    double tool_roll_offset = 0.0)
  {
    const tf2::Transform cabinet = resolve_cabinet_transform();
    const auto geometry = resolve_control_geometry(
      control, require_stable_parent);
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
    // Pincer tools (rotate_button, rocker) carry their jaws/rotor on local +Z,
    // so +Z must point at the control for the wrist to stay out of the cabinet.
    // The offset direction (outward) remains unchanged so retraction and the
    // prepress standoff still move away from the control.
    const tf2::Vector3 tool_axis_reference =
      tool_axis_orientation_ ==
        ToolProfile::ToolAxisOrientation::TOWARD_CONTROL ?
      -outward_zero : outward_zero;
    const tf2::Quaternion tool_zero =
      tool_rotation_from_outward(tool_axis_reference);
    tf2::Quaternion tool_roll;
    tool_roll.setRotation(tf2::Vector3(0.0, 0.0, 1.0), tool_roll_offset);
    tool_roll.normalize();
    // Post-multiply so the calibration rotates around contact-tool local +Z.
    // This moves passive sibling tools within the panel plane while preserving
    // the tool's outward axis and the calibrated contact point.
    tf2::Quaternion tool_rotation = world_rotation * tool_zero * tool_roll;
    tool_rotation.normalize();

    geometry_msgs::msg::Pose pose;
    // Place the measured 3-D business point, rather than the contact-link
    // origin, on the desired grasp point.  This is essential for off-axis
    // fingers whose empty tool centre must never be aimed at the control.
    const tf2::Vector3 desired_tip_position =
      grasp + outward * outward_offset;
    const tf2::Vector3 position_world = desired_tip_position -
      tf2::quatRotate(tool_rotation, tool_tip_position_);
    pose.position.x = position_world.x();
    pose.position.y = position_world.y();
    pose.position.z = position_world.z();
    pose.orientation = to_message(tool_rotation);
    return pose;
  }

  ControlStagingPoses calculate_control_staging_poses(
    const ButtonSpec & control)
  {
    if (!control.navigation_station) {
      throw OperationError(
              PressCabinetButton::Result::INTERNAL_ERROR,
              "Control '" + control.id +
              "' has no explicit robot-adapter navigation station.");
    }
    const auto & station = *control.navigation_station;

    // Resolve the cabinet transform in the navigation frame (typically
    // "map") so the station position is authoritative for Nav2 and not
    // subject to AMCL map→odom drift.  Then transform to the planning
    // frame for MoveIt.
    const std::string nav_frame =
      station.frame_id.empty() ? navigation_frame_ : station.frame_id;
    tf2::Transform cabinet_nav;
    try {
      const auto tf = transform_buffer_->lookupTransform(
        nav_frame, cabinet_frame_, tf2::TimePointZero,
        tf2::durationFromSec(system_wait_timeout_));
      tf2::fromMsg(tf.transform, cabinet_nav);
      cabinet_nav.getRotation().normalize();
    } catch (const tf2::TransformException & error) {
      throw OperationError(
              PressCabinetButton::Result::NOT_READY,
              "Cabinet TF '" + nav_frame + "' -> '" + cabinet_frame_ +
              "' is unavailable for staging pose: " + error.what());
    }

    const tf2::Vector3 local_position = station.local_anchor +
      station.outward_axis * station.standoff;
    const tf2::Vector3 nav_position = cabinet_nav * local_position;
    tf2::Vector3 nav_outward = tf2::quatRotate(
      cabinet_nav.getRotation(), station.outward_axis);
    const double horizontal_norm = std::hypot(
      nav_outward.x(), nav_outward.y());
    if (!std::isfinite(horizontal_norm) || horizontal_norm <= 1.0e-9) {
      throw OperationError(
              PressCabinetButton::Result::INTERNAL_ERROR,
              "Control '" + control.id +
              "' navigation station has no horizontal outward direction.");
    }
    nav_outward /= nav_outward.length();

    // Build the navigation pose in the nav frame.
    ControlStagingPoses poses;
    poses.navigation_pose.header.frame_id = nav_frame;
    poses.navigation_pose.pose.position.x = nav_position.x();
    poses.navigation_pose.pose.position.y = nav_position.y();
    poses.navigation_pose.pose.position.z = nav_position.z();
    tf2::Quaternion nav_rotation;
    nav_rotation.setRPY(
      0.0, 0.0,
      std::atan2(-nav_outward.y(), -nav_outward.x()) +
      station.base_yaw_offset);
    nav_rotation.normalize();
    poses.navigation_pose.pose.orientation = to_message(nav_rotation);

    // Build the planning pose in the planning frame (for MoveIt).
    const tf2::Transform cabinet = resolve_cabinet_transform();
    const tf2::Vector3 planning_position = cabinet * local_position;
    tf2::Vector3 planning_outward = tf2::quatRotate(
      cabinet.getRotation(), station.outward_axis);
    planning_outward.normalize();

    poses.planning_pose.header.frame_id = planning_frame_;
    poses.planning_pose.pose.position.x = planning_position.x();
    poses.planning_pose.pose.position.y = planning_position.y();
    poses.planning_pose.pose.position.z = planning_position.z();
    tf2::Quaternion planning_rotation;
    planning_rotation.setRPY(
      0.0, 0.0,
      std::atan2(-planning_outward.y(), -planning_outward.x()) +
      station.base_yaw_offset);
    planning_rotation.normalize();
    poses.planning_pose.pose.orientation = to_message(planning_rotation);

    return poses;
  }

  RotaryOperationPoses calculate_rotary_operation_poses(
    const ButtonSpec & control,
    double position,
    double tool_roll_offset = 0.0)
  {
    RotaryOperationPoses poses;
    poses.ready_pose = calculate_rotary_tool_pose(
      control, position, prepress_distance_, true, tool_roll_offset);
    poses.door_pregrasp_pose = calculate_rotary_tool_pose(
      control, position,
      control.grasp_outward_offset + door_pregrasp_clearance_, true,
      tool_roll_offset);
    poses.grasp_pose = calculate_rotary_tool_pose(
      control, position, control.grasp_outward_offset, true,
      tool_roll_offset);
    return poses;
  }

  std::vector<geometry_msgs::msg::Pose> calculate_rotation_waypoints(
    const ButtonSpec & control,
    double initial_position,
    double target_position,
    double tool_roll_offset = 0.0)
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
      waypoints.push_back(
        calculate_rotary_tool_pose(
          control, initial_position + travel * ratio,
          control.grasp_outward_offset, true, tool_roll_offset));
    }
    return waypoints;
  }

  double rotary_manipulation_position(
    const ButtonSpec & control,
    double initial_position,
    double target_position) const
  {
    double release_fraction = 1.0;
    if (control.control_type ==
      xczs_inspection_robot_control::msg::CabinetControl::TYPE_DOOR)
    {
      release_fraction = door_release_fraction_;
    } else if (control.control_type ==
      xczs_inspection_robot_control::msg::CabinetControl::TYPE_KNOB)
    {
      release_fraction = control.detent_release_fraction;
    }
    return rotary_release_position(
      initial_position, target_position, release_fraction,
      target_tolerance_);
  }

  template<typename GoalHandleT>
  void set_control_grasp(
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const std::string & control_id,
    bool attach)
  {
    const auto service_deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(system_wait_timeout_);
    while (!grasp_client_->wait_for_service(50ms)) {
      check_cancel(goal_handle);
      if (std::chrono::steady_clock::now() >= service_deadline) {
        throw GenericOperationError(
                attach ? OperateCabinetControl::Result::GRASP_FAILED :
                OperateCabinetControl::Result::RELEASE_FAILED,
                "Cabinet grasp service is unavailable.");
      }
    }
    auto request = std::make_shared<
      xczs_inspection_robot_control::srv::SetCabinetGrasp::Request>();
    request->control_id = control_id;
    request->robot_model = robot_model_name_;
    request->robot_link = grasp_link_;
    request->robot_base_link = grasp_brake_link_;
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
      if (!grasp_client_->wait_for_service(1s)) {
        RCLCPP_ERROR(
          get_logger(),
          "Emergency grasp release service is unavailable for '%s'.",
          control_id.c_str());
        return;
      }
      constexpr int kReleaseAttempts = 2;
      for (int attempt = 1; attempt <= kReleaseAttempts; ++attempt) {
        auto request = std::make_shared<
          xczs_inspection_robot_control::srv::SetCabinetGrasp::Request>();
        request->control_id = control_id;
        request->robot_model = robot_model_name_;
        request->robot_link = grasp_link_;
        request->robot_base_link = grasp_brake_link_;
        request->attach = false;
        auto future = grasp_client_->async_send_request(request);
        if (future.wait_for(2s) != std::future_status::ready) {
          RCLCPP_ERROR(
            get_logger(),
            "Emergency grasp release timed out for '%s' (attempt %d/%d).",
            control_id.c_str(), attempt, kReleaseAttempts);
          continue;
        }
        const auto response = future.get();
        if (response->success) {
          return;
        }
        RCLCPP_ERROR(
          get_logger(),
          "Emergency grasp release failed for '%s' (attempt %d/%d): %s",
          control_id.c_str(), attempt, kReleaseAttempts,
          response->message.c_str());
      }
      RCLCPP_ERROR(
        get_logger(),
        "Emergency grasp release exhausted all attempts for '%s'.",
        control_id.c_str());
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        get_logger(), "Emergency grasp release failed: %s", error.what());
    }
  }

  template<typename GoalHandleT>
  void wait_for_parent_controls_stable(
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const ButtonSpec & control,
    const std::unordered_map<std::string, std::string> & expected_states)
  {
    const auto ancestors = control_ancestors(control);
    if (ancestors.empty()) {
      return;
    }

    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(system_wait_timeout_);
    auto stable_since = std::chrono::steady_clock::time_point{};
    while (std::chrono::steady_clock::now() < deadline) {
      check_cancel(goal_handle);
      const auto now = std::chrono::steady_clock::now();
      bool all_stable = true;
      auto oldest_sample = std::chrono::steady_clock::time_point::max();
      for (const auto * ancestor : ancestors) {
        const auto state = button_snapshot(*ancestor);
        const auto expected = expected_states.find(ancestor->id);
        const bool fresh = structured_control_state_is_usable(
          state.structured_received, state.valid, true,
          std::chrono::duration<double>(
            now - state.structured_received_at).count() <=
          button_state_timeout_);
        if (expected == expected_states.end()) {
          throw GenericOperationError(
                  OperateCabinetControl::Result::INTERNAL_ERROR,
                  "Missing expected parent state for '" + ancestor->id +
                  "'.");
        }
        if (!fresh || state.state_id != expected->second || state.in_motion ||
          std::abs(state.velocity) > stable_velocity_tolerance_)
        {
          all_stable = false;
          break;
        }
        oldest_sample = std::min(
          oldest_sample, state.structured_received_at);
      }
      if (all_stable) {
        if (stable_since == std::chrono::steady_clock::time_point{}) {
          stable_since = oldest_sample;
        }
        if (std::chrono::duration<double>(
            oldest_sample - stable_since).count() >=
          stable_state_duration_)
        {
          return;
        }
      } else {
        stable_since = std::chrono::steady_clock::time_point{};
      }
      std::this_thread::sleep_for(50ms);
    }
    throw GenericOperationError(
            OperateCabinetControl::Result::NOT_READY,
            "Parent controls for '" + control.id +
            "' did not return to a stable physical detent.");
  }

  template<typename GoalHandleT>
  void wait_for_door_release_position(
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const ButtonSpec & control,
    double initial_position,
    double release_position,
    std::chrono::steady_clock::time_point minimum_received_at)
  {
    const double direction = release_position - initial_position;
    if (std::abs(direction) <= target_tolerance_) {
      return;
    }
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(door_release_position_timeout_);
    const double detent_midpoint =
      0.5 * (control.min_position + control.max_position);
    const double safe_release_position = detent_midpoint +
      std::copysign(
      door_detent_hysteresis_ + door_release_position_margin_, direction);
    while (std::chrono::steady_clock::now() < deadline) {
      check_cancel(goal_handle);
      std::unique_lock<std::mutex> lock(control.runtime->mutex);
      const auto now = std::chrono::steady_clock::now();
      const auto & state = control.runtime->state;
      const bool fresh = structured_control_state_is_usable(
        state.structured_received, state.valid,
        state.structured_received_at > minimum_received_at,
        std::chrono::duration<double>(
          now - state.structured_received_at).count() <=
        button_state_timeout_);
      const bool crossed_release_position = direction > 0.0 ?
        state.position >= safe_release_position :
        state.position <= safe_release_position;
      if (fresh && crossed_release_position) {
        return;
      }
      control.runtime->condition.wait_for(lock, 50ms);
    }
    throw GenericOperationError(
            OperateCabinetControl::Result::TARGET_NOT_REACHED,
            control.id + " did not physically cross its safe release "
            "position while the grasp was attached.");
  }

  template<typename GoalHandleT>
  void wait_for_knob_release_position(
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const ButtonSpec & control,
    double initial_position,
    double target_position,
    const std::string & target_state,
    std::chrono::steady_clock::time_point minimum_received_at)
  {
    const double direction = target_position - initial_position;
    if (std::abs(direction) <= target_tolerance_) {
      return;
    }
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(system_wait_timeout_);
    const double safe_release_position =
      0.5 * (initial_position + target_position) +
      std::copysign(
      door_detent_hysteresis_ + door_release_position_margin_, direction);
    while (std::chrono::steady_clock::now() < deadline) {
      check_cancel(goal_handle);
      std::unique_lock<std::mutex> lock(control.runtime->mutex);
      const auto now = std::chrono::steady_clock::now();
      const auto & state = control.runtime->state;
      const bool fresh = structured_control_state_is_usable(
        state.structured_received, state.valid,
        state.structured_received_at > minimum_received_at,
        std::chrono::duration<double>(
          now - state.structured_received_at).count() <=
        button_state_timeout_);
      const bool crossed_release_position = direction > 0.0 ?
        state.position >= safe_release_position :
        state.position <= safe_release_position;
      if (fresh && crossed_release_position && state.state_id == target_state) {
        return;
      }
      control.runtime->condition.wait_for(lock, 50ms);
    }
    throw GenericOperationError(
            OperateCabinetControl::Result::TARGET_NOT_REACHED,
            control.id + " did not physically latch the requested adjacent "
            "detent while the grasp was attached.");
  }

  template<typename GoalHandleT>
  void wait_for_target_stable(
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const ButtonSpec & control,
    double target_position,
    const std::string & target_state,
    std::chrono::steady_clock::time_point minimum_received_at)
  {
    const double settle_timeout =
      control.control_type ==
      xczs_inspection_robot_control::msg::CabinetControl::TYPE_DOOR ?
      door_settle_timeout_ : system_wait_timeout_;
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(settle_timeout);
    auto stable_since = std::chrono::steady_clock::time_point{};
    while (std::chrono::steady_clock::now() < deadline) {
      check_cancel(goal_handle);
      {
        std::unique_lock<std::mutex> lock(control.runtime->mutex);
        const auto now = std::chrono::steady_clock::now();
        const auto & state = control.runtime->state;
        const bool fresh = structured_control_state_is_usable(
          state.structured_received, state.valid,
          state.structured_received_at > minimum_received_at,
          std::chrono::duration<double>(
            now - state.structured_received_at).count() <=
          button_state_timeout_);
        const bool at_position = fresh &&
          std::abs(state.position - target_position) <= target_tolerance_;
        const bool state_matches = target_state.empty() ||
          state.state_id == target_state;
        const bool stopped = !state.in_motion &&
          std::abs(state.velocity) <= stable_velocity_tolerance_;
        if (at_position && state_matches && stopped) {
          if (stable_since == std::chrono::steady_clock::time_point{}) {
            stable_since = state.structured_received_at;
          }
          if (std::chrono::duration<double>(
              state.structured_received_at - stable_since).count() >=
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
    const auto final_state = button_snapshot(control);
    throw GenericOperationError(
            OperateCabinetControl::Result::TARGET_NOT_REACHED,
            control.id + " did not settle at the requested physical state "
            "(state='" + final_state.state_id + "', position=" +
            std::to_string(final_state.position) + ", velocity=" +
            std::to_string(final_state.velocity) + ").");
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
      case PressCabinetButton::Result::RESOURCE_BUSY:
        return OperateCabinetControl::Result::RESOURCE_BUSY;
      case PressCabinetButton::Result::LEASE_LOST:
        return OperateCabinetControl::Result::LEASE_LOST;
      default:
        return OperateCabinetControl::Result::INTERNAL_ERROR;
    }
  }

  bool is_operate_goal_canceling(
    const std::shared_ptr<OperateGoalHandle> & goal_handle) const noexcept
  {
    return goal_canceling_after_request(goal_handle);
  }

  bool finish_operate_goal_noexcept(
    const std::shared_ptr<OperateGoalHandle> & goal_handle,
    const std::shared_ptr<OperateCabinetControl::Result> & result,
    bool request_success) noexcept
  {
    const auto finalize = [&]() {
        return apply_goal_terminal_disposition(
          goal_terminal_disposition(
            request_success, is_operate_goal_canceling(goal_handle)),
          [&]() {goal_handle->succeed(result);},
          [&]() {
            result->success = false;
            result->error_code = OperateCabinetControl::Result::CANCELED;
            result->message = "Cabinet operation was canceled.";
            goal_handle->canceled(result);
          },
          [&]() {goal_handle->abort(result);});
      };
    try {
      if (!goal_handle || !goal_handle->is_active()) {
        RCLCPP_WARN(
          get_logger(), "Cabinet operation goal was already terminal.");
        return false;
      }
      return finalize();
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        get_logger(), "Failed to finalize cabinet operation: %s",
        error.what());
    } catch (...) {
      RCLCPP_ERROR(
        get_logger(), "Failed to finalize cabinet operation.");
    }

    // A cancel request can race the first terminal-state transition.
    try {
      if (goal_handle && goal_handle->is_active()) {
        return finalize();
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        get_logger(), "Failed to retry the cabinet operation terminal state: %s",
        error.what());
    } catch (...) {
      RCLCPP_ERROR(
        get_logger(), "Failed to retry the cabinet operation terminal state.");
    }
    return false;
  }

  void snapshot_planning_only_result(
    const std::shared_ptr<OperateCabinetControl::Result> & result,
    const std::shared_ptr<ButtonSpec> & control) noexcept
  {
    // This cleanup path is deliberately observation-only.  In particular it
    // must not call the ordinary recovery helper: recovery is allowed to
    // execute retreat/stow trajectories and release a physical grasp.
    if (!result || !control) {
      return;
    }
    const auto state = button_snapshot(*control);
    result->final_position = state.position;
    result->peak_position = state.position;
    result->final_state = state.state_id;
    if (control->control_type ==
      xczs_inspection_robot_control::msg::CabinetControl::TYPE_BUTTON)
    {
      result->estimated_force =
        state.position * control->spring_stiffness;
      result->button_triggered = state.pressed;
    }
  }

  void recover_failed_operate_goal(
    const std::shared_ptr<OperateCabinetControl::Result> & result,
    const std::shared_ptr<ButtonSpec> & control,
    const std::shared_ptr<MoveGroupInterface> & move_group,
    bool should_attempt_retreat,
    bool grasp_attached,
    const DoorArcProgress & door_arc_progress,
    double rotary_tool_roll_offset,
    bool canceled) noexcept
  {
    const bool physical_recovery_required = physical_recovery_is_required(
      result->operation_executed, should_attempt_retreat, grasp_attached);
    const bool motion_recovery_allowed =
      physical_recovery_required && !operation_lease_lost_.load();
    if (!physical_recovery_required) {
      RCLCPP_INFO(
        get_logger(),
        "Skipping arm/grasp recovery after '%s': the operation failed "
        "before any manipulator command, grasp, or required retreat.",
        result->diagnostic_stage.c_str());
    }
    const bool is_door = control && control->control_type ==
      xczs_inspection_robot_control::msg::CabinetControl::TYPE_DOOR;
    bool rollback_and_retreat_succeeded = false;
    if (motion_recovery_allowed && is_door && grasp_attached &&
      door_arc_progress.in_progress &&
      move_group && rclcpp::ok())
    {
      rollback_and_retreat_succeeded = best_effort_rollback_door_arc(
        *move_group, *control, door_arc_progress,
        &result->operation_executed);
    }
    if (motion_recovery_allowed && is_door && grasp_attached &&
      should_attempt_retreat && move_group && rclcpp::ok() &&
      !rollback_and_retreat_succeeded)
    {
      try {
        if (door_arc_progress.in_progress) {
          RCLCPP_WARN(
            get_logger(),
            "Door arc rollback did not complete; using the outward safety "
            "retreat fallback while the grasp remains attached.");
        }
        const auto state = button_snapshot(*control);
        const auto retreat_pose = calculate_rotary_tool_pose(
          *control, state.position, door_release_clearance_, false,
          rotary_tool_roll_offset);
        best_effort_retreat(
          *move_group, retreat_pose, &result->operation_executed);
      } catch (const std::exception & error) {
        RCLCPP_ERROR(
          get_logger(), "Door pre-release safety retreat failed: %s",
          error.what());
      }
    }
    if (physical_recovery_required && control &&
      (grasp_attached || control->requires_grasp))
    {
      result->operation_executed = true;
      release_control_grasp_noexcept(control->id);
    }
    if (motion_recovery_allowed && control && should_attempt_retreat &&
      move_group && rclcpp::ok())
    {
      try {
        const auto state = button_snapshot(*control);
        const bool is_button = control->control_type ==
          xczs_inspection_robot_control::msg::CabinetControl::TYPE_BUTTON;
        const auto retreat_pose = is_button ?
          calculate_operation_poses(
          *control, press_depth_).prepress_pose :
          calculate_rotary_tool_pose(
          *control, state.position,
          is_door ? door_release_clearance_ : prepress_distance_, false,
          rotary_tool_roll_offset);
        best_effort_retreat(
          *move_group, retreat_pose, &result->operation_executed);
      } catch (const std::exception & error) {
        RCLCPP_ERROR(
          get_logger(), "Generic safety retreat failed: %s", error.what());
      }
    }
    if (motion_recovery_allowed && move_group && rclcpp::ok()) {
      best_effort_stow(*move_group, &result->operation_executed);
    }
    if (control) {
      const auto final_state = button_snapshot(*control);
      result->final_position = final_state.position;
      result->peak_position = final_state.max_position;
      result->final_state = final_state.state_id;
      if (control->control_type ==
        xczs_inspection_robot_control::msg::CabinetControl::TYPE_BUTTON)
      {
        result->estimated_force =
          final_state.max_position * control->spring_stiffness;
        result->button_triggered =
          result->button_triggered ||
          final_state.max_position + 1.0e-9 >= control->press_threshold;
      }
    }
    if (operation_lease_lost_.load() && !canceled) {
      result->success = false;
      result->error_code = OperateCabinetControl::Result::LEASE_LOST;
      result->message =
        "The global robot operation lease was lost; all motion was stopped.";
    } else if (canceled) {
      result->success = false;
      result->error_code = OperateCabinetControl::Result::CANCELED;
      result->message = "Cabinet operation was canceled.";
    }
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
    const auto update = classify_structured_control_state(
      message.control_id == control.id, message.valid, message.position,
      message.velocity, message.effort);
    if (update == StructuredControlStateUpdate::IGNORE) {
      return;
    }
    {
      std::lock_guard<std::mutex> lock(control.runtime->mutex);
      auto & state = control.runtime->state;
      const auto received_at = std::chrono::steady_clock::now();
      state.structured_received = true;
      state.structured_received_at = received_at;
      if (update == StructuredControlStateUpdate::INVALIDATE) {
        state.valid = false;
        control.runtime->condition.notify_all();
        return;
      }
      state.received = true;
      state.valid = true;
      state.position = message.position;
      state.velocity = message.velocity;
      state.effort = message.effort;
      state.max_position = std::max(state.max_position, state.position);
      state.in_motion = message.in_motion;
      state.state_id = message.state_id;
      state.received_at = received_at;
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
    const ButtonSpec & button,
    std::chrono::steady_clock::time_point minimum_received_at)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(system_wait_timeout_);
    while (std::chrono::steady_clock::now() < deadline) {
      check_cancel(goal_handle);
      {
        std::unique_lock<std::mutex> lock(button.runtime->mutex);
        const auto now = std::chrono::steady_clock::now();
        const bool joint_state_fresh = button.runtime->state.received &&
          button.runtime->state.received_at > minimum_received_at &&
          std::chrono::duration<double>(
          now - button.runtime->state.received_at).count() <=
          button_state_timeout_;
        const bool pressed_state_fresh =
          button.runtime->state.pressed_received &&
          button.runtime->state.pressed_received_at > minimum_received_at &&
          std::chrono::duration<double>(
          now - button.runtime->state.pressed_received_at).count() <=
          button_state_timeout_;
        const bool structured_state_fresh =
          structured_control_state_is_usable(
          button.runtime->state.structured_received,
          button.runtime->state.valid,
          button.runtime->state.structured_received_at > minimum_received_at,
          std::chrono::duration<double>(
            now - button.runtime->state.structured_received_at).count() <=
          button_state_timeout_);
        if (joint_state_fresh && pressed_state_fresh &&
          structured_state_fresh)
        {
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

  template<typename GoalHandleT>
  void track_button_force(
    MoveGroupInterface & move_group,
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const ButtonSpec & button,
    double target_travel,
    bool * operation_executed = nullptr)
  {
    double commanded_depth = clamp_button_press_depth(
      target_travel, button.max_position);
    const double maximum_depth = button_force_tracking_limit(
      target_travel, force_tracking_max_compensation_, button.max_position);
    for (int attempt = 0; attempt < force_tracking_attempts_; ++attempt) {
      interruptible_hold(goal_handle, force_tracking_settle_seconds_);
      const auto measured = button_snapshot(button);
      const double deficit = target_travel - measured.position;
      if (deficit <= force_tracking_tolerance_) {
        return;
      }

      const double next_depth = clamp_button_press_depth(
        std::min(
          maximum_depth,
          commanded_depth + deficit + force_tracking_tolerance_),
        button.max_position);
      if (next_depth <= commanded_depth + 1.0e-6) {
        return;
      }
      commanded_depth = next_depth;
      publish_operate_feedback(
        goal_handle,
        OperateCabinetControl::Feedback::MANIPULATING,
        0.70F, target_travel,
        "Correcting the physical button travel to track the requested force.");
      const auto corrected_poses = calculate_operation_poses(
        button, commanded_depth);
      execute_cartesian_path(
        move_group, goal_handle, {corrected_poses.pressed_pose},
        cartesian_velocity_scale_ * 0.35,
        cartesian_acceleration_scale_ * 0.35,
        button_press_minimum_cartesian_fraction_, operation_executed);
    }
    interruptible_hold(goal_handle, force_tracking_settle_seconds_);
  }

  OperationPoses calculate_operation_poses(
    const ButtonSpec & button,
    double press_depth)
  {
    const double bounded_press_depth = clamp_button_press_depth(
      press_depth, button.max_position);
    const tf2::Transform cabinet_transform = resolve_cabinet_transform();
    const tf2::Quaternion cabinet_rotation = cabinet_transform.getRotation();
    const auto geometry = resolve_control_geometry(button);
    const tf2::Vector3 button_face =
      cabinet_transform * geometry.grasp_zero;
    tf2::Vector3 outward = tf2::quatRotate(
      cabinet_rotation, geometry.approach_normal);
    outward.normalize();
    const tf2::Vector3 inward = -outward;

    // At zero roll keep tool +X vertical and tool -Z aligned with the press
    // direction.  A calibrated local-Z roll may then clear the passive sibling
    // tool without changing either the working direction or business point.
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
    tf2::Quaternion tool_roll;
    tool_roll.setRotation(
      tf2::Vector3(0.0, 0.0, 1.0), button.tool_roll_offset);
    tool_roll.normalize();
    tool_rotation *= tool_roll;
    tool_rotation.normalize();
    const auto make_tool_pose =
      [&](double tip_offset) -> geometry_msgs::msg::Pose {
        const tf2::Vector3 desired_tip_position =
          button_face + inward * tip_offset;
        const tf2::Vector3 link_position = desired_tip_position -
          tf2::quatRotate(tool_rotation, tool_tip_position_);
        geometry_msgs::msg::Pose pose;
        pose.position.x = link_position.x();
        pose.position.y = link_position.y();
        pose.position.z = link_position.z();
        pose.orientation = to_message(tool_rotation);
        return pose;
      };

    OperationPoses poses;
    poses.prepress_pose = make_tool_pose(-prepress_distance_);
    poses.contact_pose = make_tool_pose(-contact_clearance_);
    poses.pressed_pose = make_tool_pose(bounded_press_depth);

    return poses;
  }

  void configure_move_group(MoveGroupInterface & move_group)
  {
    const auto robot_model = move_group.getRobotModel();
    std::vector<std::string> profile_move_groups;
    profile_move_groups.reserve(tool_profiles_.size() + 1U);
    profile_move_groups.push_back(move_group_name_);
    for (const auto & entry : tool_profiles_) {
      profile_move_groups.push_back(entry.second.move_group);
    }
    const auto kinematics_error = tool_profile_kinematics_validation_error(
      robot_model, std::move(profile_move_groups));
    if (kinematics_error) {
      throw OperationError(
              PressCabinetButton::Result::NOT_READY,
              *kinematics_error);
    }
    if (!robot_model->getLinkModel(contact_tool_link_)) {
      throw OperationError(
              PressCabinetButton::Result::NOT_READY,
              "The robot adapter tool profile for MoveIt JointModelGroup '" +
              move_group_name_ + "' references missing contact tool link '" +
              contact_tool_link_ + "'.");
    }
    const auto named_targets = move_group.getNamedTargets();
    if (std::find(
        named_targets.begin(), named_targets.end(),
        transport_named_target_) == named_targets.end())
    {
      throw OperationError(
              PressCabinetButton::Result::NOT_READY,
              "The robot adapter transport target '" +
              transport_named_target_ + "' does not exist in the SRDF.");
    }
    if (!move_group.setEndEffectorLink(contact_tool_link_)) {
      throw OperationError(
              PressCabinetButton::Result::NOT_READY,
              "MoveIt cannot use configured contact tool link '" +
              contact_tool_link_ + "'.");
    }
    move_group.setPoseReferenceFrame(planning_frame_);
    move_group.setPlanningTime(planning_time_);
    move_group.setNumPlanningAttempts(planning_attempts_);
    move_group.setMaxVelocityScalingFactor(planning_velocity_scale_);
    move_group.setMaxAccelerationScalingFactor(planning_acceleration_scale_);
    move_group.setGoalPositionTolerance(goal_position_tolerance_);
    move_group.setGoalOrientationTolerance(goal_orientation_tolerance_);
    move_group.setGoalJointTolerance(goal_joint_tolerance_);
    move_group.allowReplanning(allow_replanning_);
  }

  moveit::core::RobotStatePtr synchronized_current_robot_state(
    MoveGroupInterface & move_group)
  {
    auto state = move_group.getCurrentState(system_wait_timeout_);
    if (!state) {
      throw OperationError(
              PressCabinetButton::Result::NOT_READY,
              "MoveIt did not receive the current robot state.");
    }

    const auto robot_model = move_group.getRobotModel();
    const auto * root_joint =
      robot_model == nullptr ? nullptr : robot_model->getRootJoint();
    if (root_joint == nullptr || root_joint->getVariableCount() == 0U) {
      state->update();
      return state;
    }

    try {
      const auto transform = transform_buffer_->lookupTransform(
        robot_model->getModelFrame(), robot_model->getRootLinkName(),
        tf2::TimePointZero,
        tf2::durationFromSec(system_wait_timeout_));
      const auto & translation = transform.transform.translation;
      const auto & rotation = transform.transform.rotation;
      Eigen::Quaterniond quaternion(
        rotation.w, rotation.x, rotation.y, rotation.z);
      quaternion.normalize();
      Eigen::Isometry3d root_transform = Eigen::Isometry3d::Identity();
      root_transform.linear() = quaternion.toRotationMatrix();
      root_transform.translation() = Eigen::Vector3d(
        translation.x, translation.y, translation.z);
      state->setJointPositions(root_joint, root_transform);
      state->update();
    } catch (const tf2::TransformException & error) {
      throw OperationError(
              PressCabinetButton::Result::NOT_READY,
              "MoveIt mobile-base transform '" +
              robot_model->getModelFrame() + "' -> '" +
              robot_model->getRootLinkName() +
              "' is unavailable: " + error.what());
    }
    return state;
  }

  static void update_validation_diagnostic(
    OperateCabinetControl::Result & result,
    const std::string & stage,
    double path_fraction,
    double required_fraction,
    std::int32_t moveit_error_code)
  {
    result.diagnostic_stage = stage;
    result.path_fraction = std::isfinite(path_fraction) ?
      std::max(0.0, std::min(1.0, path_fraction)) : 0.0;
    result.required_fraction = std::isfinite(required_fraction) ?
      std::max(0.0, std::min(1.0, required_fraction)) : 0.0;
    result.moveit_error_code = moveit_error_code;
  }

  static std::string planning_validation_failure_message(
    const OperateCabinetControl::Result & result,
    const std::string & detail)
  {
    std::ostringstream message;
    message << "实时 MoveIt 安全规划验证失败 [" << result.diagnostic_stage
            << "]：路径完成 " << result.path_fraction * 100.0
            << "% ，要求 " << result.required_fraction * 100.0 << "%";
    if (result.moveit_error_code != 0) {
      const moveit::core::MoveItErrorCode error(result.moveit_error_code);
      message << "，MoveIt=" << moveit::core::error_code_to_string(error)
              << "(" << result.moveit_error_code << ")";
    }
    if (!detail.empty()) {
      message << "；" << detail;
    }
    if (!result.policy_reason.empty()) {
      message << "；适配层历史限制：" << result.policy_reason;
    }
    return message.str();
  }

  moveit::core::RobotState validation_trajectory_end_state(
    MoveGroupInterface & move_group,
    const moveit::core::RobotState & start_state,
    const moveit_msgs::msg::RobotTrajectory & trajectory_message) const
  {
    robot_trajectory::RobotTrajectory trajectory(
      move_group.getRobotModel(), move_group_name_);
    trajectory.setRobotTrajectoryMsg(start_state, trajectory_message);
    if (trajectory.getWayPointCount() == 0U) {
      // MoveIt may represent an already-satisfied target as an empty plan.
      return start_state;
    }
    return trajectory.getLastWayPoint();
  }

  bool set_pose_target_with_calibrated_ik_seed(
    MoveGroupInterface & move_group,
    const moveit::core::RobotState & start_state,
    const geometry_msgs::msg::Pose & target,
    const std::string & tool_link,
    const ButtonSpec * control)
  {
    if (control == nullptr || control->ready_joint_seed_names.empty()) {
      return move_group.setPoseTarget(target, tool_link);
    }

    const auto robot_model = move_group.getRobotModel();
    const auto * joint_model_group =
      robot_model == nullptr ? nullptr :
      robot_model->getJointModelGroup(move_group_name_);
    if (joint_model_group == nullptr) {
      throw GenericOperationError(
              OperateCabinetControl::Result::NOT_READY,
              "MoveIt group '" + move_group_name_ +
              "' is unavailable while applying the calibrated IK seed.");
    }
    const auto & group_variable_names =
      joint_model_group->getVariableNames();
    std::unordered_set<std::string> configured_names(
      control->ready_joint_seed_names.begin(),
      control->ready_joint_seed_names.end());
    if (configured_names.size() != group_variable_names.size() ||
      std::any_of(
        group_variable_names.begin(), group_variable_names.end(),
        [&configured_names](const std::string & name) {
          return configured_names.count(name) == 0U;
        }))
    {
      throw GenericOperationError(
              OperateCabinetControl::Result::NOT_READY,
              "Control '" + control->id +
              "' ready_joint_seed must name every variable of MoveIt group '" +
              move_group_name_ + "' exactly once.");
    }

    moveit::core::RobotState seeded_goal(start_state);
    seeded_goal.setVariablePositions(
      control->ready_joint_seed_names,
      control->ready_joint_seed_positions);
    seeded_goal.update();
    if (!seeded_goal.satisfiesBounds(joint_model_group)) {
      throw GenericOperationError(
              OperateCabinetControl::Result::NOT_READY,
              "Control '" + control->id +
              "' ready_joint_seed violates the configured joint limits.");
    }
    if (!seeded_goal.setFromIK(
        joint_model_group, target, tool_link,
        std::min(1.0, planning_time_)))
    {
      return false;
    }
    seeded_goal.update();
    return seeded_goal.satisfiesBounds(joint_model_group) &&
           move_group.setJointValueTarget(seeded_goal);
  }

  moveit::core::RobotState validate_pose_plan_only(
    MoveGroupInterface & move_group,
    const std::shared_ptr<OperateGoalHandle> & goal_handle,
    const moveit::core::RobotState & start_state,
    const geometry_msgs::msg::Pose & target,
    const std::string & tool_link,
    const std::string & stage,
    const std::shared_ptr<OperateCabinetControl::Result> & result,
    const ButtonSpec * control = nullptr)
  {
    check_cancel(goal_handle);
    update_validation_diagnostic(
      *result, stage, 0.0, 1.0, 0);
    RCLCPP_DEBUG(
      get_logger(),
      "Planning-only target [%s] link=%s frame=%s "
      "position=(%.9f, %.9f, %.9f) orientation=(%.9f, %.9f, %.9f, %.9f)",
      stage.c_str(), tool_link.c_str(), planning_frame_.c_str(),
      target.position.x, target.position.y, target.position.z,
      target.orientation.x, target.orientation.y,
      target.orientation.z, target.orientation.w);
    move_group.setStartState(start_state);
    if (!set_pose_target_with_calibrated_ik_seed(
        move_group, start_state, target, tool_link, control))
    {
      update_validation_diagnostic(
        *result, stage, 0.0, 1.0,
        moveit_msgs::msg::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS);
      throw OperationError(
              PressCabinetButton::Result::PLANNING_FAILED,
              planning_validation_failure_message(
                *result, "MoveIt 拒绝了目标位姿或工具链接。"));
    }

    MoveGroupInterface::Plan plan;
    moveit::core::MoveItErrorCode planning_result;
    bool planned = false;
    for (int attempt = 1; attempt <= motion_planning_attempts_; ++attempt) {
      check_cancel(goal_handle);
      move_group.setStartState(start_state);
      planning_result = move_group.plan(plan);
      if (planning_result == moveit::core::MoveItErrorCode::SUCCESS) {
        planned = true;
        break;
      }
      RCLCPP_WARN(
        get_logger(),
        "Planning-only validation stage '%s' failed (attempt %d/%d, "
        "MoveIt=%d).",
        stage.c_str(), attempt, motion_planning_attempts_, planning_result.val);
    }
    move_group.clearPoseTargets();
    if (!planned) {
      update_validation_diagnostic(
        *result, stage, 0.0, 1.0, planning_result.val);
      throw OperationError(
              PressCabinetButton::Result::PLANNING_FAILED,
              planning_validation_failure_message(
                *result, "无法生成无碰撞目标位姿轨迹。"));
    }
    update_validation_diagnostic(
      *result, stage, 1.0, 1.0,
      moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
    return validation_trajectory_end_state(
      move_group, start_state, plan.trajectory_);
  }

  moveit::core::RobotState validate_named_target_plan_only(
    MoveGroupInterface & move_group,
    const std::shared_ptr<OperateGoalHandle> & goal_handle,
    const moveit::core::RobotState & start_state,
    const std::string & target_name,
    const std::string & stage,
    const std::shared_ptr<OperateCabinetControl::Result> & result)
  {
    check_cancel(goal_handle);
    update_validation_diagnostic(
      *result, stage, 0.0, 1.0, 0);
    move_group.setStartState(start_state);
    if (!move_group.setNamedTarget(target_name)) {
      update_validation_diagnostic(
        *result, stage, 0.0, 1.0,
        moveit_msgs::msg::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS);
      throw OperationError(
              PressCabinetButton::Result::PLANNING_FAILED,
              planning_validation_failure_message(
                *result, "MoveIt 不包含指定的安全命名位姿。"));
    }

    MoveGroupInterface::Plan plan;
    moveit::core::MoveItErrorCode planning_result;
    bool planned = false;
    for (int attempt = 1; attempt <= motion_planning_attempts_; ++attempt) {
      check_cancel(goal_handle);
      move_group.setStartState(start_state);
      planning_result = move_group.plan(plan);
      if (planning_result == moveit::core::MoveItErrorCode::SUCCESS) {
        planned = true;
        break;
      }
    }
    if (!planned) {
      update_validation_diagnostic(
        *result, stage, 0.0, 1.0, planning_result.val);
      throw OperationError(
              PressCabinetButton::Result::PLANNING_FAILED,
              planning_validation_failure_message(
                *result, "无法规划回到安全运输位姿。"));
    }
    update_validation_diagnostic(
      *result, stage, 1.0, 1.0,
      moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
    return validation_trajectory_end_state(
      move_group, start_state, plan.trajectory_);
  }

  moveit::core::RobotState validate_cartesian_plan_only(
    MoveGroupInterface & move_group,
    const std::shared_ptr<OperateGoalHandle> & goal_handle,
    const moveit::core::RobotState & start_state,
    const std::vector<geometry_msgs::msg::Pose> & waypoints,
    double minimum_fraction,
    const std::string & stage,
    const std::shared_ptr<OperateCabinetControl::Result> & result)
  {
    check_cancel(goal_handle);
    update_validation_diagnostic(
      *result, stage, 0.0, minimum_fraction, 0);
    if (waypoints.empty()) {
      update_validation_diagnostic(
        *result, stage, 1.0, minimum_fraction,
        moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
      return start_state;
    }
    moveit_msgs::msg::RobotTrajectory best_trajectory;
    moveit_msgs::msg::MoveItErrorCodes best_error;
    double best_fraction = -1.0;
    for (int attempt = 0; attempt < cartesian_planning_attempts_; ++attempt) {
      check_cancel(goal_handle);
      move_group.setStartState(start_state);
      moveit_msgs::msg::RobotTrajectory candidate;
      moveit_msgs::msg::MoveItErrorCodes error;
      const double fraction = move_group.computeCartesianPath(
        waypoints, 0.002, cartesian_jump_threshold_, candidate, true, &error);
      if (fraction > best_fraction) {
        best_fraction = fraction;
        best_trajectory = std::move(candidate);
        best_error = error;
      }
      if (best_fraction >= minimum_fraction) {
        break;
      }
    }
    const double reported_fraction = std::max(0.0, best_fraction);
    update_validation_diagnostic(
      *result, stage, reported_fraction, minimum_fraction, best_error.val);
    if (best_fraction < minimum_fraction) {
      throw OperationError(
              PressCabinetButton::Result::PLANNING_FAILED,
              planning_validation_failure_message(
                *result, "笛卡尔路径没有达到当前执行路径使用的安全门槛。"));
    }

    robot_trajectory::RobotTrajectory timed_trajectory(
      move_group.getRobotModel(), move_group_name_);
    timed_trajectory.setRobotTrajectoryMsg(start_state, best_trajectory);
    if (timed_trajectory.getWayPointCount() > 0U) {
      trajectory_processing::TimeOptimalTrajectoryGeneration time_parameterizer;
      if (!time_parameterizer.computeTimeStamps(
          timed_trajectory,
          cartesian_velocity_scale_,
          cartesian_acceleration_scale_))
      {
        update_validation_diagnostic(
          *result, stage, reported_fraction, minimum_fraction,
          moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN);
        throw OperationError(
                PressCabinetButton::Result::PLANNING_FAILED,
                planning_validation_failure_message(
                  *result, "笛卡尔候选轨迹无法生成安全时间参数。"));
      }
      update_validation_diagnostic(
        *result, stage, reported_fraction, minimum_fraction,
        moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
      return timed_trajectory.getLastWayPoint();
    }
    update_validation_diagnostic(
      *result, stage, reported_fraction, minimum_fraction,
      moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
    return start_state;
  }

  moveit::core::RobotState validate_segmented_cartesian_plan_only(
    MoveGroupInterface & move_group,
    const std::shared_ptr<OperateGoalHandle> & goal_handle,
    moveit::core::RobotState virtual_state,
    const std::vector<geometry_msgs::msg::Pose> & waypoints,
    std::size_t maximum_segment_waypoints,
    const std::shared_ptr<OperateCabinetControl::Result> & result)
  {
    if (maximum_segment_waypoints == 0U) {
      throw std::invalid_argument(
              "Planning-only Cartesian segment size must be positive.");
    }
    for (std::size_t begin = 0U; begin < waypoints.size();
      begin += maximum_segment_waypoints)
    {
      const auto end = std::min(
        begin + maximum_segment_waypoints, waypoints.size());
      const std::vector<geometry_msgs::msg::Pose> segment(
        waypoints.begin() + begin, waypoints.begin() + end);
      try {
        virtual_state = validate_cartesian_plan_only(
          move_group, goal_handle, virtual_state, segment, 0.99,
          "manipulation", result);
      } catch (const OperationError & error) {
        if (error.error_code != PressCabinetButton::Result::PLANNING_FAILED ||
          segment.size() == 1U)
        {
          throw;
        }
        // Match the real door executor's safe fallback without executing a
        // partial segment: retry each predicted waypoint from the preceding
        // virtual trajectory endpoint.
        for (const auto & waypoint : segment) {
          virtual_state = validate_cartesian_plan_only(
            move_group, goal_handle, virtual_state, {waypoint}, 0.99,
            "manipulation", result);
        }
      }
    }
    return virtual_state;
  }

  void validate_inoperable_control_path(
    MoveGroupInterface & move_group,
    const std::shared_ptr<OperateGoalHandle> & goal_handle,
    const ButtonSpec & control,
    const ButtonSnapshot & initial_state,
    double target_position,
    const OperationPoses & button_poses,
    const RotaryOperationPoses & rotary_poses,
    double rotary_tool_roll_offset,
    const moveit::core::RobotState & current_robot_state,
    const std::shared_ptr<OperateCabinetControl::Result> & result)
  {
    // SAFETY CONTRACT: this function and every helper it calls are planning
    // only.  They may query current Gazebo/TF state and the MoveIt planning
    // scene, but they must never execute a trajectory, navigate, dock, attach
    // a grasp, wait for a physical transition, or write a cabinet joint.
    moveit::core::RobotState virtual_state(current_robot_state);
    const bool is_button = control.control_type ==
      xczs_inspection_robot_control::msg::CabinetControl::TYPE_BUTTON;
    if (is_button) {
      publish_operate_feedback(
        goal_handle, OperateCabinetControl::Feedback::MOVING_TO_READY,
        0.25F, target_position,
        "正在用实时 MoveIt 场景验证按钮预备位姿（不会执行轨迹）。");
      virtual_state = validate_pose_plan_only(
        move_group, goal_handle, virtual_state,
        button_poses.prepress_pose, contact_tool_link_, "ready_pose", result);
      publish_operate_feedback(
        goal_handle, OperateCabinetControl::Feedback::APPROACHING,
        0.47F, target_position,
        "正在验证按钮接近路径（不会移动机械臂）。");
      virtual_state = validate_cartesian_plan_only(
        move_group, goal_handle, virtual_state,
        {button_poses.contact_pose}, 0.99, "approach", result);
      publish_operate_feedback(
        goal_handle, OperateCabinetControl::Feedback::MANIPULATING,
        0.65F, target_position,
        "正在验证请求力对应的按压路径（不会接触按钮）。");
      virtual_state = validate_cartesian_plan_only(
        move_group, goal_handle, virtual_state,
        {button_poses.pressed_pose}, button_press_minimum_cartesian_fraction_,
        "manipulation", result);
      publish_operate_feedback(
        goal_handle, OperateCabinetControl::Feedback::RETREATING,
        0.86F, 0.0,
        "正在验证按钮撤回路径（不会执行轨迹）。");
      virtual_state = validate_cartesian_plan_only(
        move_group, goal_handle, virtual_state,
        {button_poses.contact_pose, button_poses.prepress_pose},
        0.99, "retreat", result);
    } else {
      publish_operate_feedback(
        goal_handle, OperateCabinetControl::Feedback::MOVING_TO_READY,
        0.25F, target_position,
        "正在用实时 MoveIt 场景验证控件预备位姿（不会执行轨迹）。");
      virtual_state = validate_pose_plan_only(
        move_group, goal_handle, virtual_state,
        rotary_poses.ready_pose, contact_tool_link_, "ready_pose", result,
        &control);
      if (control.control_type ==
        xczs_inspection_robot_control::msg::CabinetControl::TYPE_DOOR)
      {
        virtual_state = validate_pose_plan_only(
          move_group, goal_handle, virtual_state,
          rotary_poses.door_pregrasp_pose, contact_tool_link_,
          "pregrasp", result);
      }
      publish_operate_feedback(
        goal_handle, OperateCabinetControl::Feedback::APPROACHING,
        0.43F, target_position,
        "正在验证抓取接近路径（不会建立物理抓取）。");
      virtual_state = validate_cartesian_plan_only(
        move_group, goal_handle, virtual_state,
        {rotary_poses.grasp_pose}, 0.99, "approach", result);

      const bool is_door = control.control_type ==
        xczs_inspection_robot_control::msg::CabinetControl::TYPE_DOOR;
      const double manipulation_position = rotary_manipulation_position(
        control, initial_state.position, target_position);
      const auto waypoints = calculate_rotation_waypoints(
        control, initial_state.position, manipulation_position,
        rotary_tool_roll_offset);
      publish_operate_feedback(
        goal_handle, OperateCabinetControl::Feedback::MANIPULATING,
        0.62F, target_position,
        "正在验证控件操作圆弧（不会转动或抓取控件）。");
      if (is_door) {
        virtual_state = validate_segmented_cartesian_plan_only(
          move_group, goal_handle, virtual_state, waypoints,
          static_cast<std::size_t>(door_cartesian_segment_waypoints_), result);
      } else {
        virtual_state = validate_cartesian_plan_only(
          move_group, goal_handle, virtual_state, waypoints,
          0.99, "manipulation", result);
      }
      const double release_clearance = is_door ?
        door_release_clearance_ : prepress_distance_;
      const auto target_ready = calculate_rotary_tool_pose(
        control, manipulation_position, release_clearance, false,
        rotary_tool_roll_offset);
      publish_operate_feedback(
        goal_handle, OperateCabinetControl::Feedback::RETREATING,
        0.84F, target_position,
        "正在验证控件撤离路径（不会释放不存在的抓取）。");
      virtual_state = validate_cartesian_plan_only(
        move_group, goal_handle, virtual_state, {target_ready},
        0.99, "retreat", result);
    }

    publish_operate_feedback(
      goal_handle, OperateCabinetControl::Feedback::VERIFYING,
      0.96F, target_position,
      "正在验证从预测撤回终点返回安全运输位姿。");
    (void)validate_named_target_plan_only(
      move_group, goal_handle, virtual_state,
      transport_named_target_, "transport", result);
  }

  template<typename GoalHandleT>
  void plan_and_execute_pose(
    MoveGroupInterface & move_group,
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const geometry_msgs::msg::Pose & target,
    const std::string & tool_link,
    bool * operation_executed = nullptr,
    const ButtonSpec * control = nullptr)
  {
    check_cancel(goal_handle);
    MoveGroupInterface::Plan plan;
    bool planned = false;
    for (int attempt = 1; attempt <= motion_planning_attempts_; ++attempt) {
      check_cancel(goal_handle);
      const auto current_state =
        synchronized_current_robot_state(move_group);
      move_group.setStartState(*current_state);
      if (!set_pose_target_with_calibrated_ik_seed(
          move_group, *current_state, target, tool_link, control))
      {
        throw OperationError(
                PressCabinetButton::Result::PLANNING_FAILED,
                "MoveIt rejected the cabinet pose target or its calibrated "
                "IK seed did not converge.");
      }
      const auto planning_result = move_group.plan(plan);
      move_group.clearPoseTargets();
      if (planning_result == moveit::core::MoveItErrorCode::SUCCESS) {
        planned = true;
        break;
      }
      RCLCPP_WARN(
        get_logger(),
        "MoveIt planning to the cabinet pose for '%s' failed "
        "(attempt %d/%d).",
        tool_link.c_str(), attempt, motion_planning_attempts_);
    }
    if (!planned) {
      throw OperationError(
              PressCabinetButton::Result::PLANNING_FAILED,
              "MoveIt could not plan to the prepress pose after " +
              std::to_string(motion_planning_attempts_) +
              " attempts. If navigation was skipped, first park the base "
              "in front of the cabinet.");
    }
    check_cancel(goal_handle);
    if (operation_executed) {
      *operation_executed = true;
    }
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
  void plan_and_execute_named_target(
    MoveGroupInterface & move_group,
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const std::string & target_name,
    bool * operation_executed = nullptr,
    bool physical_outcome_committed = false)
  {
    const auto check_stop = [this, &goal_handle,
        physical_outcome_committed]() {
        if (physical_outcome_committed) {
          check_safety_transport_stop(goal_handle);
        } else {
          check_cancel(goal_handle);
        }
      };
    check_stop();
    MoveGroupInterface::Plan plan;
    bool planned = false;
    for (int attempt = 1; attempt <= motion_planning_attempts_; ++attempt) {
      check_stop();
      const auto current_state =
        synchronized_current_robot_state(move_group);
      move_group.setStartState(*current_state);
      if (!move_group.setNamedTarget(target_name)) {
        throw OperationError(
                PressCabinetButton::Result::PLANNING_FAILED,
                "MoveIt does not contain the arm target '" + target_name +
                "'.");
      }
      if (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        planned = true;
        break;
      }
      RCLCPP_WARN(
        get_logger(),
        "MoveIt planning to named target '%s' failed (attempt %d/%d).",
        target_name.c_str(), attempt, motion_planning_attempts_);
    }
    if (!planned) {
      throw OperationError(
              PressCabinetButton::Result::PLANNING_FAILED,
              "MoveIt could not plan the arm to the safe '" + target_name +
              "' target after " +
              std::to_string(motion_planning_attempts_) +
              " attempts.");
    }
    check_stop();
    if (operation_executed) {
      *operation_executed = true;
    }
    if (move_group.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      check_stop();
      throw OperationError(
              PressCabinetButton::Result::EXECUTION_FAILED,
              "MoveIt could not return the arm to the safe '" + target_name +
              "' target.");
    }
    check_stop();
  }

  template<typename GoalHandleT>
  void execute_segmented_cartesian_path(
    MoveGroupInterface & move_group,
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const std::vector<geometry_msgs::msg::Pose> & waypoints,
    std::size_t maximum_segment_waypoints,
    double velocity_scale,
    double acceleration_scale,
    std::size_t & completed_waypoints,
    bool * operation_executed = nullptr)
  {
    if (maximum_segment_waypoints == 0U) {
      throw std::invalid_argument(
              "Cartesian segment size must be greater than zero.");
    }
    completed_waypoints = 0U;
    const std::size_t segment_count =
      (waypoints.size() + maximum_segment_waypoints - 1U) /
      maximum_segment_waypoints;
    std::size_t segment_index = 0U;
    for (std::size_t begin = 0; begin < waypoints.size();
      begin += maximum_segment_waypoints)
    {
      ++segment_index;
      check_cancel(goal_handle);
      const auto end = std::min(
        begin + maximum_segment_waypoints, waypoints.size());
      const std::vector<geometry_msgs::msg::Pose> segment(
        waypoints.begin() + begin, waypoints.begin() + end);
      RCLCPP_INFO(
        get_logger(),
        "Starting door Cartesian segment %zu/%zu [%zu, %zu); "
        "%zu/%zu waypoints already executed.",
        segment_index, segment_count, begin, end,
        completed_waypoints, waypoints.size());
      try {
        execute_cartesian_path(
          move_group, goal_handle, segment,
          velocity_scale, acceleration_scale, 0.99, operation_executed);
        completed_waypoints = end;
        RCLCPP_INFO(
          get_logger(),
          "Door Cartesian segment %zu/%zu [%zu, %zu) completed; "
          "%zu/%zu waypoints executed.",
          segment_index, segment_count, begin, end,
          completed_waypoints, waypoints.size());
      } catch (const OperationError & error) {
        if (error.error_code != PressCabinetButton::Result::PLANNING_FAILED ||
          segment.size() == 1U)
        {
          throw;
        }

        // Recomputing each waypoint from the measured joint state can escape
        // an IK/state-validity dead end without ever executing a partial
        // Cartesian plan. The fallback also reduces the maximum swept-door
        // update interval from four degrees to two degrees.
        RCLCPP_WARN(
          get_logger(),
          "Door Cartesian segment %zu/%zu [%zu, %zu) was not fully "
          "plannable after %zu/%zu completed waypoints; retrying it one "
          "waypoint at a time.",
          segment_index, segment_count, begin, end,
          completed_waypoints, waypoints.size());
        for (std::size_t offset = 0U; offset < segment.size(); ++offset) {
          execute_cartesian_path(
            move_group, goal_handle, {segment[offset]},
            velocity_scale, acceleration_scale, 0.99,
            operation_executed);
          completed_waypoints = begin + offset + 1U;
          RCLCPP_INFO(
            get_logger(),
            "Door Cartesian fallback waypoint %zu/%zu completed; "
            "%zu/%zu waypoints executed.",
            offset + 1U, segment.size(), completed_waypoints,
            waypoints.size());
        }
      }
    }
  }

  template<typename GoalHandleT>
  void execute_cartesian_path(
    MoveGroupInterface & move_group,
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const std::vector<geometry_msgs::msg::Pose> & waypoints,
    double velocity_scale,
    double acceleration_scale,
    double minimum_fraction = 0.99,
    bool * operation_executed = nullptr)
  {
    check_cancel(goal_handle);
    moveit_msgs::msg::RobotTrajectory trajectory_message;
    double best_fraction = -1.0;
    for (int attempt = 0; attempt < cartesian_planning_attempts_; ++attempt) {
      check_cancel(goal_handle);
      const auto current_state =
        synchronized_current_robot_state(move_group);
      move_group.setStartState(*current_state);
      moveit_msgs::msg::RobotTrajectory candidate;
      const double fraction = move_group.computeCartesianPath(
        waypoints,
        0.002,
        cartesian_jump_threshold_,
        candidate,
        true);
      if (fraction > best_fraction) {
        best_fraction = fraction;
        trajectory_message = std::move(candidate);
      }
      if (best_fraction >= minimum_fraction) {
        break;
      }
    }
    if (best_fraction < minimum_fraction) {
      throw OperationError(
              PressCabinetButton::Result::PLANNING_FAILED,
              "MoveIt completed only " +
              std::to_string(std::max(0.0, best_fraction) * 100.0) +
              "% of the Cartesian control path; this segment requires " +
              std::to_string(minimum_fraction * 100.0) + "% after " +
              std::to_string(cartesian_planning_attempts_) + " attempts.");
    }
    retime_cartesian_trajectory(
      move_group,
      trajectory_message,
      velocity_scale,
      acceleration_scale);
    check_cancel(goal_handle);
    if (operation_executed) {
      *operation_executed = true;
    }
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
    const auto current_state = synchronized_current_robot_state(move_group);
    robot_trajectory::RobotTrajectory trajectory(
      move_group.getRobotModel(), move_group_name_);
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

  bool best_effort_cartesian_move(
    MoveGroupInterface & move_group,
    const std::vector<geometry_msgs::msg::Pose> & waypoints,
    double velocity_scale,
    double acceleration_scale,
    const char * description,
    bool * operation_executed = nullptr) noexcept
  {
    try {
      move_group.stop();
      const auto current_state =
        synchronized_current_robot_state(move_group);
      move_group.setStartState(*current_state);
      moveit_msgs::msg::RobotTrajectory trajectory_message;
      const double fraction = move_group.computeCartesianPath(
        waypoints,
        0.002,
        cartesian_jump_threshold_,
        trajectory_message,
        true);
      if (fraction < 0.99) {
        RCLCPP_ERROR(
          get_logger(), "%s path was only %.1f%% complete.",
          description, fraction * 100.0);
        return false;
      }
      retime_cartesian_trajectory(
        move_group,
        trajectory_message,
        velocity_scale,
        acceleration_scale);
      if (operation_executed) {
        *operation_executed = true;
      }
      if (move_group.execute(trajectory_message) !=
        moveit::core::MoveItErrorCode::SUCCESS)
      {
        RCLCPP_ERROR(get_logger(), "%s execution failed.", description);
        return false;
      }
      return true;
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        get_logger(), "%s failed: %s", description, error.what());
    } catch (...) {
      RCLCPP_ERROR(get_logger(), "%s failed with an unknown error.", description);
    }
    return false;
  }

  bool best_effort_rollback_door_arc(
    MoveGroupInterface & move_group,
    const ButtonSpec & control,
    const DoorArcProgress & progress,
    bool * operation_executed = nullptr) noexcept
  {
    try {
      const auto current_state = button_snapshot(control);
      if (!std::isfinite(current_state.position)) {
        RCLCPP_ERROR(
          get_logger(),
          "Door arc rollback cannot start from a non-finite position.");
        return false;
      }

      std::vector<geometry_msgs::msg::Pose> rollback_waypoints;
      if (std::abs(current_state.position - progress.initial_position) >
        target_tolerance_)
      {
        rollback_waypoints = calculate_rotation_waypoints(
          control, current_state.position, progress.initial_position);
      }
      RCLCPP_WARN(
        get_logger(),
        "Door arc failed after %zu/%zu executed waypoints; rolling the "
        "grasped door back from %.4f rad to %.4f rad using %zu waypoints.",
        progress.completed_waypoints, progress.total_waypoints,
        current_state.position, progress.initial_position,
        rollback_waypoints.size());

      for (std::size_t index = 0U; index < rollback_waypoints.size(); ++index) {
        if (!best_effort_cartesian_move(
            move_group, {rollback_waypoints[index]},
            cartesian_velocity_scale_ * 0.5,
            cartesian_acceleration_scale_ * 0.5,
            "Door arc rollback waypoint", operation_executed))
        {
          RCLCPP_ERROR(
            get_logger(),
            "Door arc rollback stopped after %zu/%zu waypoints.",
            index, rollback_waypoints.size());
          return false;
        }
        RCLCPP_INFO(
          get_logger(), "Door arc rollback waypoint %zu/%zu completed.",
          index + 1U, rollback_waypoints.size());
      }

      const auto retreat_pose = calculate_rotary_tool_pose(
        control, progress.initial_position, door_release_clearance_, false);
      if (!best_effort_cartesian_move(
          move_group, {retreat_pose}, cartesian_velocity_scale_,
          cartesian_acceleration_scale_, "Door post-rollback safety retreat",
          operation_executed))
      {
        return false;
      }
      RCLCPP_INFO(
        get_logger(),
        "Door arc rollback and outward safety retreat completed while the "
        "grasp remained attached.");
      return true;
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        get_logger(), "Door arc rollback failed: %s", error.what());
    } catch (...) {
      RCLCPP_ERROR(
        get_logger(), "Door arc rollback failed with an unknown error.");
    }
    return false;
  }

  void best_effort_retreat(
    MoveGroupInterface & move_group,
    const geometry_msgs::msg::Pose & prepress_pose,
    bool * operation_executed = nullptr) noexcept
  {
    (void)best_effort_cartesian_move(
      move_group, {prepress_pose}, cartesian_velocity_scale_,
      cartesian_acceleration_scale_, "Safety retreat", operation_executed);
  }

  void best_effort_stow(
    MoveGroupInterface & move_group,
    bool * operation_executed = nullptr) noexcept
  {
    try {
      move_group.stop();
      const auto current_state =
        synchronized_current_robot_state(move_group);
      move_group.setStartState(*current_state);
      if (!move_group.setNamedTarget(transport_named_target_)) {
        RCLCPP_ERROR(
          get_logger(), "Safety target '%s' is not configured.",
          transport_named_target_.c_str());
        return;
      }
      MoveGroupInterface::Plan plan;
      if (move_group.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "Safety stow planning failed.");
        return;
      }
      if (operation_executed) {
        *operation_executed = true;
      }
      if (move_group.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "Safety stow execution failed.");
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Safety stow failed: %s", error.what());
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
    const auto near_distance_since_ns =
      std::make_shared<std::atomic<std::int64_t>>(0);
    const auto takeover_requested =
      std::make_shared<std::atomic<bool>>(false);
    const auto stalled_takeover_requested =
      std::make_shared<std::atomic<bool>>(false);
    tf2::Quaternion target_navigation_rotation;
    tf2::fromMsg(
      staging_pose.pose.orientation, target_navigation_rotation);
    const double target_navigation_yaw =
      tf2::getYaw(target_navigation_rotation);
    const double target_navigation_x = staging_pose.pose.position.x;
    const double target_navigation_y = staging_pose.pose.position.y;
    const std::string target_navigation_frame =
      staging_pose.header.frame_id;
    options.goal_response_callback =
      [this, request_abandoned,
        weak_goal = std::weak_ptr<GoalHandleT>(goal_handle)](
      NavigationGoalHandle::SharedPtr goal_handle)
      {
        if (!goal_handle) {
          return;
        }
        const auto operation_goal = weak_goal.lock();
        const bool operation_should_stop =
          !operation_goal || goal_should_stop(operation_goal);
        bool must_cancel;
        {
          std::lock_guard<std::mutex> lock(navigation_mutex_);
          must_cancel = request_abandoned->load() ||
            operation_should_stop;
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
        navigation_started, near_distance_since_ns, takeover_requested,
        stalled_takeover_requested, target_navigation_yaw,
        target_navigation_x, target_navigation_y, target_navigation_frame](
      NavigationGoalHandle::SharedPtr,
      const std::shared_ptr<const NavigateToPose::Feedback> feedback)
      {
        const double distance_remaining = feedback->distance_remaining;
        const double current_navigation_x =
          feedback->current_pose.pose.position.x;
        const double current_navigation_y =
          feedback->current_pose.pose.position.y;
        tf2::Quaternion current_navigation_rotation;
        tf2::fromMsg(
          feedback->current_pose.pose.orientation,
          current_navigation_rotation);
        const double quaternion_norm =
          current_navigation_rotation.length2();
        const bool feedback_pose_valid =
          !target_navigation_frame.empty() &&
          feedback->current_pose.header.frame_id == target_navigation_frame &&
          std::isfinite(target_navigation_x) &&
          std::isfinite(target_navigation_y) &&
          std::isfinite(current_navigation_x) &&
          std::isfinite(current_navigation_y) &&
          std::isfinite(quaternion_norm) && quaternion_norm > 1.0e-12;
        double direct_goal_distance =
          std::numeric_limits<double>::quiet_NaN();
        double yaw_error = std::numeric_limits<double>::quiet_NaN();
        if (feedback_pose_valid) {
          direct_goal_distance = std::hypot(
            target_navigation_x - current_navigation_x,
            target_navigation_y - current_navigation_y);
          current_navigation_rotation.normalize();
          const double current_navigation_yaw =
            tf2::getYaw(current_navigation_rotation);
          yaw_error = std::atan2(
            std::sin(target_navigation_yaw - current_navigation_yaw),
            std::cos(target_navigation_yaw - current_navigation_yaw));
        }
        const auto feedback_now = std::chrono::steady_clock::now();
        if (std::isfinite(direct_goal_distance) && std::isfinite(yaw_error)) {
          if (direct_goal_distance > navigation_takeover_distance_) {
            navigation_started->store(true);
            near_distance_since_ns->store(0);
            takeover_requested->store(false);
            stalled_takeover_requested->store(false);
          } else {
            const auto now_ns = std::chrono::duration_cast<
              std::chrono::nanoseconds>(
              feedback_now.time_since_epoch()).count();
            auto first_near_ns = near_distance_since_ns->load();
            if (first_near_ns == 0) {
              if (near_distance_since_ns->compare_exchange_strong(
                  first_near_ns, now_ns))
              {
                first_near_ns = now_ns;
              }
            }
            const double near_duration = static_cast<double>(
              now_ns - first_near_ns) * 1.0e-9;
            const bool yaw_ready = navigation_started->load() &&
              std::abs(yaw_error) <= navigation_takeover_yaw_tolerance_;
            const bool yaw_stalled =
              near_duration >= navigation_takeover_stall_timeout_;
            if (yaw_ready || yaw_stalled) {
              stalled_takeover_requested->store(yaw_stalled && !yaw_ready);
              takeover_requested->store(true);
            }
          }
        } else {
          near_distance_since_ns->store(0);
          takeover_requested->store(false);
          stalled_takeover_requested->store(false);
        }

        {
          std::lock_guard<std::mutex> lock(navigation_feedback_mutex_);
          if (feedback_now - last_navigation_feedback_ < 500ms) {
            return;
          }
          last_navigation_feedback_ = feedback_now;
        }
        if (const auto operation_goal = weak_goal.lock()) {
          publish_feedback(
            operation_goal,
            PressCabinetButton::Feedback::NAVIGATING,
            0.12F,
            "Nav2 distance remaining: " +
            (std::isfinite(distance_remaining) ?
            std::to_string(distance_remaining) : "unavailable") +
            " m, direct goal distance: " +
            (std::isfinite(direct_goal_distance) ?
            std::to_string(direct_goal_distance) : "unavailable") +
            " m, yaw: " +
            (std::isfinite(yaw_error) ?
            std::to_string(std::abs(yaw_error)) : "unavailable") +
            " rad." +
            (stalled_takeover_requested->load() ?
            " Safe-distance yaw stall detected; taking over." : ""));
        }
      };
    NavigationGoalHandle::SharedPtr navigation_goal_handle;
    const auto acceptance_deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(system_wait_timeout_);
    int rejected_attempts = 0;
    while (!navigation_goal_handle) {
      check_cancel(goal_handle);
      navigation_goal.pose.header.stamp = get_clock()->now();
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
      navigation_goal_handle = goal_future.get();
      if (navigation_goal_handle) {
        break;
      }
      ++rejected_attempts;
      if (std::chrono::steady_clock::now() >= acceptance_deadline) {
        throw OperationError(
                PressCabinetButton::Result::NAVIGATION_FAILED,
                "Nav2 rejected the cabinet staging goal for " +
                std::to_string(rejected_attempts) +
                " consecutive startup attempts.");
      }
      RCLCPP_WARN(
        get_logger(),
        "Nav2 rejected staging goal attempt %d while its lifecycle may "
        "still be activating; retrying.",
        rejected_attempts);
      std::this_thread::sleep_for(250ms);
    }
    bool must_cancel_navigation = false;
    const bool operation_should_stop = goal_should_stop(goal_handle);
    {
      std::lock_guard<std::mutex> lock(navigation_mutex_);
      must_cancel_navigation = operation_should_stop;
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
    const auto verified_direct_goal_distance = [this, &staging_pose]() {
        try {
          const auto current_pose = transform_buffer_->lookupTransform(
            staging_pose.header.frame_id, navigation_base_frame_,
            tf2::TimePointZero,
            200ms);
          const double current_x =
            current_pose.transform.translation.x;
          const double current_y =
            current_pose.transform.translation.y;
          if (!std::isfinite(current_x) || !std::isfinite(current_y) ||
            !std::isfinite(staging_pose.pose.position.x) ||
            !std::isfinite(staging_pose.pose.position.y))
          {
            return std::numeric_limits<double>::quiet_NaN();
          }
          return std::hypot(
            staging_pose.pose.position.x - current_x,
            staging_pose.pose.position.y - current_y);
        } catch (const tf2::TransformException & error) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Could not verify the Nav2 takeover distance: %s", error.what());
          return std::numeric_limits<double>::quiet_NaN();
        }
      };
    bool precision_takeover = false;
    const auto navigation_deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(navigation_timeout_);
    while (result_future.wait_for(100ms) != std::future_status::ready) {
      check_cancel(goal_handle);
      if (takeover_requested->load()) {
        const double verified_distance = verified_direct_goal_distance();
        if (!std::isfinite(verified_distance) ||
          verified_distance > navigation_takeover_distance_)
        {
          takeover_requested->store(false);
          stalled_takeover_requested->store(false);
          near_distance_since_ns->store(0);
          continue;
        }
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
        auto cancel_future = navigation_client_->async_cancel_goal(
          navigation_goal_handle);
        wait_for_future(
          cancel_future,
          goal_handle,
          system_wait_timeout_,
          PressCabinetButton::Result::NAVIGATION_FAILED,
          "Nav2 did not acknowledge cancellation after its staging timeout.");
        request_navigation_mode_without_wait(false);
        publish_manual_base_stop();
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
      const double stopped_distance = verified_direct_goal_distance();
      if (!std::isfinite(stopped_distance) ||
        stopped_distance >
        navigation_takeover_distance_ + docking_position_tolerance_)
      {
        throw OperationError(
                PressCabinetButton::Result::NAVIGATION_FAILED,
                "Nav2 stopped outside the verified precision-docking "
                "takeover distance.");
      }
      publish_feedback(
        goal_handle,
        PressCabinetButton::Feedback::NAVIGATING,
        0.16F,
        stalled_takeover_requested->load() ?
        "Nav2 reached the safe staging distance but its terminal yaw "
        "stalled; switching to odometry docking." :
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
  static std::uint8_t docking_feedback_phase(
    const std::shared_ptr<GoalHandleT> &)
  {
    return PressCabinetButton::Feedback::NAVIGATING;
  }

  static std::uint8_t docking_feedback_phase(
    const std::shared_ptr<OperateGoalHandle> &)
  {
    return OperateCabinetControl::Feedback::DOCKING;
  }

  template<typename GoalHandleT>
  void verify_staging_pose_before_arm_motion(
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const geometry_msgs::msg::PoseStamped & target)
  {
    check_cancel(goal_handle);
    publish_manual_base_stop();

    geometry_msgs::msg::TransformStamped current_transform;
    try {
      current_transform = transform_buffer_->lookupTransform(
        planning_frame_, docking_base_frame_, tf2::TimePointZero,
        tf2::durationFromSec(system_wait_timeout_));
    } catch (const tf2::TransformException & error) {
      throw OperationError(
              PressCabinetButton::Result::NOT_READY,
              "Cannot verify the stopped base pose before arm motion: " +
              std::string(error.what()));
    }

    tf2::Quaternion target_rotation;
    tf2::Quaternion current_rotation;
    tf2::fromMsg(target.pose.orientation, target_rotation);
    tf2::fromMsg(
      current_transform.transform.rotation, current_rotation);
    const double current_x = current_transform.transform.translation.x;
    const double current_y = current_transform.transform.translation.y;
    if (target.header.frame_id != planning_frame_ ||
      !std::isfinite(target.pose.position.x) ||
      !std::isfinite(target.pose.position.y) ||
      !std::isfinite(current_x) || !std::isfinite(current_y) ||
      !std::isfinite(target_rotation.length2()) ||
      target_rotation.length2() <= 1.0e-12 ||
      !std::isfinite(current_rotation.length2()) ||
      current_rotation.length2() <= 1.0e-12)
    {
      throw OperationError(
              PressCabinetButton::Result::NOT_READY,
              "The stopped base pose is invalid before arm motion.");
    }
    target_rotation.normalize();
    current_rotation.normalize();
    const double target_body_yaw = navigation_yaw_in_model_frame(
      tf2::getYaw(target_rotation), navigation_velocity_yaw_offset_);
    const double current_yaw = tf2::getYaw(current_rotation);
    const double position_error = std::hypot(
      target.pose.position.x - current_x,
      target.pose.position.y - current_y);
    const double yaw_error = std::atan2(
      std::sin(target_body_yaw - current_yaw),
      std::cos(target_body_yaw - current_yaw));
    if (!staging_pose_error_is_safe(
        position_error, yaw_error, docking_position_tolerance_,
        docking_yaw_tolerance_))
    {
      throw OperationError(
              PressCabinetButton::Result::NAVIGATION_FAILED,
              "The base moved outside the verified cabinet station while "
              "the planning scene settled (position error " +
              std::to_string(position_error) + " m, yaw error " +
              std::to_string(std::abs(yaw_error)) +
              " rad); arm motion was blocked.");
    }
  }

  template<typename GoalHandleT>
  void dock_to_staging_pose(
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const geometry_msgs::msg::PoseStamped & target)
  {
    if (target.header.frame_id != planning_frame_ ||
      !std::isfinite(target.pose.position.x) ||
      !std::isfinite(target.pose.position.y))
    {
      publish_manual_base_stop();
      throw OperationError(
              PressCabinetButton::Result::INTERNAL_ERROR,
              "Precision docking requires a finite target in planning frame '" +
              planning_frame_ + "'.");
    }
    tf2::Quaternion target_rotation;
    tf2::fromMsg(target.pose.orientation, target_rotation);
    if (!std::isfinite(target_rotation.length2()) ||
      target_rotation.length2() <= 1.0e-12)
    {
      publish_manual_base_stop();
      throw OperationError(
              PressCabinetButton::Result::INTERNAL_ERROR,
              "Precision docking received an invalid target orientation.");
    }
    target_rotation.normalize();
    // The base router transforms navigation velocity components with
    // R(+offset). The physical docking frame yaw is therefore the desired
    // navigation-base yaw minus that same shared frame offset.
    const double target_body_yaw = navigation_yaw_in_model_frame(
      tf2::getYaw(target_rotation), navigation_velocity_yaw_offset_);
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(docking_timeout_);
    auto last_feedback = std::chrono::steady_clock::time_point{};
    int settled_cycles = 0;
    bool takeover_distance_verified = false;

    try {
      while (std::chrono::steady_clock::now() < deadline) {
        check_cancel(goal_handle);
        geometry_msgs::msg::TransformStamped current_transform;
        try {
          current_transform = transform_buffer_->lookupTransform(
            planning_frame_, docking_base_frame_, tf2::TimePointZero);
        } catch (const tf2::TransformException & error) {
          throw OperationError(
                  PressCabinetButton::Result::NOT_READY,
                  "Could not read the robot odometry for precision "
                  "docking: " + std::string(error.what()));
        }

        tf2::Quaternion current_rotation;
        tf2::fromMsg(
          current_transform.transform.rotation, current_rotation);
        const double current_x = current_transform.transform.translation.x;
        const double current_y = current_transform.transform.translation.y;
        if (!std::isfinite(current_x) || !std::isfinite(current_y) ||
          !std::isfinite(current_rotation.length2()) ||
          current_rotation.length2() <= 1.0e-12)
        {
          throw OperationError(
                  PressCabinetButton::Result::NOT_READY,
                  "Robot odometry is invalid during precision docking.");
        }
        current_rotation.normalize();
        const double current_yaw = tf2::getYaw(current_rotation);
        const double error_x = target.pose.position.x - current_x;
        const double error_y = target.pose.position.y - current_y;
        const double position_error = std::hypot(error_x, error_y);
        const double yaw_error = std::atan2(
          std::sin(target_body_yaw - current_yaw),
          std::cos(target_body_yaw - current_yaw));
        if (!std::isfinite(position_error) || !std::isfinite(yaw_error)) {
          throw OperationError(
                  PressCabinetButton::Result::NOT_READY,
                  "Precision docking could not compute a finite pose error.");
        }
        if (!takeover_distance_verified) {
          if (position_error >
            navigation_takeover_distance_ + docking_position_tolerance_)
          {
            throw OperationError(
                    PressCabinetButton::Result::NAVIGATION_FAILED,
                    "The coarse navigation result is " +
                    std::to_string(position_error) +
                    " m from the configured control station, outside the "
                    "precision-docking takeover distance of " +
                    std::to_string(navigation_takeover_distance_) + " m.");
          }
          takeover_distance_verified = true;
        }

        if (staging_pose_error_is_safe(
            position_error, yaw_error, docking_position_tolerance_,
            docking_yaw_tolerance_))
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
            docking_feedback_phase(goal_handle),
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
  void acquire_operation_lease(
    const std::shared_ptr<GoalHandleT> & goal_handle,
    ActiveGoalType goal_type,
    bool resources_required)
  {
    operation_lease_lost_.store(false);
    check_cancel(goal_handle);
    const auto service_deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(operation_lease_request_timeout_);
    while (!operation_lease_client_->wait_for_service(20ms)) {
      check_cancel(goal_handle);
      if (std::chrono::steady_clock::now() >= service_deadline) {
        throw OperationError(
                PressCabinetButton::Result::LEASE_LOST,
                "The global robot operation lease service is unavailable; "
                "motion is disabled.");
      }
    }
    check_cancel(goal_handle);

    const std::string owner_id = std::string(get_fully_qualified_name()) +
      ":" + std::to_string(++operation_lease_owner_sequence_);
    auto request = std::make_shared<ManageOperationLease::Request>();
    request->command = ManageOperationLease::Request::ACQUIRE;
    request->owner_id = owner_id;
    request->requested_duration = operation_lease_duration_;

    ManageOperationLease::Response::SharedPtr response;
    try {
      auto future = operation_lease_client_->async_send_request(request);
      const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::duration<double>(operation_lease_request_timeout_);
      while (future.wait_for(20ms) != std::future_status::ready) {
        check_cancel(goal_handle);
        if (std::chrono::steady_clock::now() >= deadline) {
          throw OperationError(
                  PressCabinetButton::Result::LEASE_LOST,
                  "The global robot operation lease request timed out; "
                  "motion is disabled.");
        }
      }
      response = future.get();
    } catch (const OperationError &) {
      throw;
    } catch (const std::exception & error) {
      throw OperationError(
              PressCabinetButton::Result::LEASE_LOST,
              std::string("The global robot operation lease request failed: ") +
              error.what());
    }

    if (!response || !response->success) {
      const bool busy = response && response->status_code ==
        ManageOperationLease::Response::RESOURCE_BUSY;
      throw OperationError(
              busy ? PressCabinetButton::Result::RESOURCE_BUSY :
              PressCabinetButton::Result::LEASE_LOST,
              response && !response->message.empty() ? response->message :
              "The global robot operation lease was not granted.");
    }
    if (response->lease_id.empty() || response->owner_id != owner_id ||
      !std::isfinite(response->remaining_duration) ||
      response->remaining_duration <= 0.0)
    {
      throw OperationError(
              PressCabinetButton::Result::LEASE_LOST,
              "The global robot operation lease response was invalid; "
              "motion is disabled.");
    }

    {
      std::lock_guard<std::mutex> lock(operation_lease_mutex_);
      operation_lease_owner_id_ = owner_id;
      operation_lease_id_ = response->lease_id;
      operation_lease_held_.store(true);
    }
    operation_lease_stop_.store(false);
    try {
      operation_lease_renewal_thread_ = std::thread(
        [this]() {renew_operation_lease_loop();});
    } catch (...) {
      release_operation_lease_noexcept();
      throw OperationError(
              PressCabinetButton::Result::LEASE_LOST,
              "Could not start the operation lease renewal worker; "
              "motion is disabled.");
    }
    check_cancel(goal_handle);
    claim_active_goal_physical_motion_resources(
      goal_type, goal_handle->get_goal_id(), resources_required);
    check_cancel(goal_handle);
  }

  void renew_operation_lease_loop() noexcept
  {
    std::unique_lock<std::mutex> wait_lock(operation_lease_wait_mutex_);
    while (!operation_lease_stop_.load()) {
      if (operation_lease_condition_.wait_for(
          wait_lock,
          std::chrono::duration<double>(operation_lease_renew_period_),
          [this]() {return operation_lease_stop_.load();}))
      {
        return;
      }
      wait_lock.unlock();
      renew_operation_lease_once();
      wait_lock.lock();
      if (operation_lease_lost_.load()) {
        return;
      }
    }
  }

  void renew_operation_lease_once() noexcept
  {
    std::string owner_id;
    std::string lease_id;
    {
      std::lock_guard<std::mutex> lock(operation_lease_mutex_);
      if (!operation_lease_held_.load()) {
        return;
      }
      owner_id = operation_lease_owner_id_;
      lease_id = operation_lease_id_;
    }

    if (!operation_lease_client_->service_is_ready()) {
      mark_operation_lease_lost(
        "The global operation lease service became unavailable.");
      return;
    }

    auto request = std::make_shared<ManageOperationLease::Request>();
    request->command = ManageOperationLease::Request::RENEW;
    request->owner_id = owner_id;
    request->lease_id = lease_id;
    request->requested_duration = operation_lease_duration_;
    try {
      auto future = operation_lease_client_->async_send_request(request);
      const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::duration<double>(operation_lease_request_timeout_);
      while (future.wait_for(20ms) != std::future_status::ready) {
        if (operation_lease_stop_.load()) {
          return;
        }
        if (std::chrono::steady_clock::now() >= deadline) {
          mark_operation_lease_lost(
            "The global operation lease renewal timed out.");
          return;
        }
      }
      const auto response = future.get();
      if (!response || !response->success ||
        response->lease_id != lease_id || response->owner_id != owner_id ||
        !std::isfinite(response->remaining_duration) ||
        response->remaining_duration <= 0.0)
      {
        mark_operation_lease_lost(
          response && !response->message.empty() ? response->message :
          "The global operation lease renewal was rejected.");
      }
    } catch (const std::exception & error) {
      mark_operation_lease_lost(
        std::string("The global operation lease renewal failed: ") +
        error.what());
    } catch (...) {
      mark_operation_lease_lost(
        "The global operation lease renewal failed unexpectedly.");
    }
  }

  void mark_operation_lease_lost(const std::string & reason) noexcept
  {
    if (!operation_lease_held_.load() ||
      operation_lease_lost_.exchange(true))
    {
      return;
    }
    bool owns_physical_motion_resources = false;
    {
      std::lock_guard<std::mutex> lock(active_goal_mutex_);
      owns_physical_motion_resources =
        active_goal_owns_physical_motion_resources_;
    }
    if (owns_physical_motion_resources) {
      RCLCPP_ERROR(
        get_logger(), "%s All MoveIt and Nav2 motion is being stopped.",
        reason.c_str());
      stop_active_motion();
      cancel_active_navigation();
      publish_manual_base_stop();
      request_navigation_mode_without_wait(false);
    } else {
      RCLCPP_ERROR(
        get_logger(), "%s Planning-only validation is being canceled without "
        "publishing a physical stop command.", reason.c_str());
    }
    publish_active_control("");
  }

  void release_operation_lease_noexcept() noexcept
  {
    operation_lease_stop_.store(true);
    operation_lease_condition_.notify_all();
    if (operation_lease_renewal_thread_.joinable()) {
      operation_lease_renewal_thread_.join();
    }

    std::string owner_id;
    std::string lease_id;
    {
      std::lock_guard<std::mutex> lock(operation_lease_mutex_);
      if (!operation_lease_held_.exchange(false)) {
        operation_lease_lost_.store(false);
        return;
      }
      owner_id = operation_lease_owner_id_;
      lease_id = operation_lease_id_;
      operation_lease_owner_id_.clear();
      operation_lease_id_.clear();
    }

    try {
      if (operation_lease_client_->service_is_ready()) {
        auto request = std::make_shared<ManageOperationLease::Request>();
        request->command = ManageOperationLease::Request::RELEASE;
        request->owner_id = owner_id;
        request->lease_id = lease_id;
        auto future = operation_lease_client_->async_send_request(request);
        if (future.wait_for(
            std::chrono::duration<double>(operation_lease_request_timeout_)) !=
          std::future_status::ready)
        {
          RCLCPP_WARN(
            get_logger(),
            "Operation lease release timed out; its short TTL will expire.");
        } else {
          const auto response = future.get();
          if (!response || !response->success) {
            RCLCPP_WARN(
              get_logger(), "Operation lease release was not acknowledged: %s",
              response ? response->message.c_str() : "empty response");
          }
        }
      } else {
        RCLCPP_WARN(
          get_logger(),
          "Operation lease service is unavailable during release; "
          "its short TTL will expire.");
      }
    } catch (const std::exception & error) {
      RCLCPP_WARN(
        get_logger(), "Operation lease release failed: %s", error.what());
    } catch (...) {
      RCLCPP_WARN(get_logger(), "Operation lease release failed unexpectedly.");
    }
    operation_lease_lost_.store(false);
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
    if (operation_lease_lost_.load()) {
      throw OperationError(
              PressCabinetButton::Result::LEASE_LOST,
              "The global robot operation lease was lost.");
    }
    if (goal_should_stop(goal_handle)) {
      throw OperationError(
              PressCabinetButton::Result::CANCELED,
              "Cabinet button operation was canceled.");
    }
    verify_latched_cabinet_transform();
  }

  template<typename GoalHandleT>
  void check_safety_transport_stop(
    const std::shared_ptr<GoalHandleT> &) const
  {
    if (operation_lease_lost_.load()) {
      throw OperationError(
              PressCabinetButton::Result::LEASE_LOST,
              "The global robot operation lease was lost.");
    }
    if (shutdown_requested_.load() || !rclcpp::ok()) {
      throw OperationError(
              PressCabinetButton::Result::CANCELED,
              "Cabinet operation stopped because ROS is shutting down.");
    }
    verify_latched_cabinet_transform();
  }

  bool is_goal_canceling_noexcept(
    const std::shared_ptr<PressGoalHandle> & goal_handle) const noexcept
  {
    return goal_canceling_after_request(goal_handle);
  }

  bool finish_goal_noexcept(
    const std::shared_ptr<PressGoalHandle> & goal_handle,
    const std::shared_ptr<PressCabinetButton::Result> & result,
    bool request_success) noexcept
  {
    const auto finalize = [&]() {
        return apply_goal_terminal_disposition(
          goal_terminal_disposition(
            request_success, is_goal_canceling_noexcept(goal_handle)),
          [&]() {goal_handle->succeed(result);},
          [&]() {
            result->success = false;
            result->error_code = PressCabinetButton::Result::CANCELED;
            result->message = "Cabinet button operation was canceled.";
            goal_handle->canceled(result);
          },
          [&]() {goal_handle->abort(result);});
      };
    try {
      if (!goal_handle || !goal_handle->is_active()) {
        RCLCPP_WARN(
          get_logger(), "Action goal was already in a terminal state.");
        return false;
      }
      return finalize();
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
      if (goal_handle && goal_handle->is_active()) {
        return finalize();
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        get_logger(), "Failed to retry the action terminal state: %s",
        error.what());
    } catch (...) {
      RCLCPP_ERROR(
        get_logger(), "Failed to retry the action terminal state.");
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
  rclcpp::Client<ManageOperationLease>::SharedPtr operation_lease_client_;
  rclcpp::Client<
    xczs_inspection_robot_control::srv::SetCabinetGrasp>::SharedPtr
    grasp_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_physics_client_;
  rclcpp::CallbackGroup::SharedPtr reset_client_callback_group_;
  rclcpp::CallbackGroup::SharedPtr operation_lease_client_callback_group_;
  rclcpp::CallbackGroup::SharedPtr reset_service_callback_group_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_controls_service_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
    manual_base_publisher_;
  rclcpp::Publisher<
    xczs_inspection_robot_control::msg::CabinetControlCatalog>::SharedPtr
    control_catalog_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
    active_control_publisher_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
    cabinet_pose_valid_subscription_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr>
  button_joint_state_subscriptions_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr>
  button_pressed_subscriptions_;
  std::vector<rclcpp::Subscription<
      xczs_inspection_robot_control::msg::CabinetControlState>::SharedPtr>
  control_state_subscriptions_;
  std::shared_ptr<tf2_ros::Buffer> transform_buffer_;
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
  std::mutex operation_lease_mutex_;
  std::string operation_lease_owner_id_;
  std::string operation_lease_id_;
  std::atomic<bool> operation_lease_held_{false};
  std::atomic<bool> operation_lease_lost_{false};
  std::atomic<bool> operation_lease_stop_{true};
  std::atomic<std::uint64_t> operation_lease_owner_sequence_{0};
  std::mutex operation_lease_wait_mutex_;
  std::condition_variable operation_lease_condition_;
  std::thread operation_lease_renewal_thread_;
  mutable std::mutex active_goal_mutex_;
  ActiveGoalType active_goal_type_{ActiveGoalType::NONE};
  rclcpp_action::GoalUUID active_goal_id_{};
  bool active_goal_owns_physical_motion_resources_{false};
  bool active_goal_physical_outcome_committed_{false};
  std::atomic<bool> cancel_requested_{false};
  std::atomic<bool> shutdown_requested_{false};
  std::atomic<bool> cabinet_pose_valid_{false};
  std::atomic<bool> cabinet_pose_valid_received_{false};
  mutable std::mutex cabinet_transform_mutex_;
  tf2::Transform latched_cabinet_transform_{tf2::Transform::getIdentity()};
  bool cabinet_transform_latched_{false};

  std::string planning_frame_;
  std::string navigation_frame_;
  std::string cabinet_frame_;
  std::string robot_model_name_;
  std::string move_group_name_;
  std::string move_group_namespace_;
  std::string toolset_;
  std::string contact_tool_link_;
  std::string grasp_link_;
  std::string transport_named_target_;
  ToolProfile::ToolAxisOrientation tool_axis_orientation_{
    ToolProfile::ToolAxisOrientation::ALONG_OUTWARD};
  tf2::Vector3 tool_tip_position_{0.0, 0.0, 0.0};
  std::vector<std::string> tool_tip_calibration_joint_names_;
  std::vector<double> tool_tip_calibration_joint_positions_;
  double tool_tip_calibration_joint_tolerance_{0.001};
  std::unordered_map<std::uint8_t, ToolProfile> tool_profiles_;
  double planning_time_{10.0};
  int planning_attempts_{10};
  double planning_velocity_scale_{0.20};
  double planning_acceleration_scale_{0.20};
  double goal_position_tolerance_{0.005};
  double goal_orientation_tolerance_{0.01};
  double goal_joint_tolerance_{0.001};
  bool allow_replanning_{true};
  bool allow_embedded_navigation_{true};
  std::string navigation_base_frame_;
  std::string docking_base_frame_;
  std::string grasp_brake_link_;
  bool require_cabinet_pose_valid_{true};
  double cabinet_pose_translation_tolerance_{0.020};
  double cabinet_pose_rotation_tolerance_{0.035};
  double operation_lease_duration_{3.0};
  double operation_lease_renew_period_{0.75};
  double operation_lease_request_timeout_{0.50};
  double prepress_distance_{0.060};
  double grasp_outward_offset_{0.020};
  double contact_clearance_{0.001};
  double press_depth_{0.007};
  double button_state_timeout_{1.0};
  double system_wait_timeout_{15.0};
  double navigation_timeout_{120.0};
  double navigation_takeover_distance_{0.15};
  double navigation_takeover_yaw_tolerance_{0.35};
  double navigation_takeover_stall_timeout_{4.0};
  double docking_timeout_{45.0};
  double docking_position_tolerance_{0.015};
  double docking_yaw_tolerance_{0.10};
  std::vector<PlanarFootprintPoint> docking_base_footprint_;
  double docking_base_footprint_padding_{0.03};
  double docking_max_linear_speed_{0.15};
  double docking_max_angular_speed_{0.45};
  double docking_linear_gain_{0.8};
  double docking_angular_gain_{1.2};
  double navigation_velocity_yaw_offset_{1.57079632679};
  double press_detection_timeout_{3.0};
  double release_detection_timeout_{3.0};
  double press_hold_seconds_{0.5};
  double force_tracking_tolerance_{0.00005};
  double force_tracking_max_compensation_{0.0010};
  double force_tracking_settle_seconds_{0.15};
  int force_tracking_attempts_{3};
  double button_press_minimum_cartesian_fraction_{0.95};
  double target_tolerance_{0.035};
  double stable_velocity_tolerance_{0.03};
  double stable_state_duration_{0.30};
  double grasp_attach_settle_duration_{0.15};
  double grasp_release_settle_duration_{0.30};
  double door_release_fraction_{0.60};
  double door_settle_timeout_{90.0};
  double door_release_position_timeout_{10.0};
  double door_detent_hysteresis_{0.02};
  double door_release_position_margin_{0.01};
  double door_pregrasp_clearance_{0.010};
  double door_release_clearance_{0.30};
  double planning_scene_settle_seconds_{0.50};
  double rotation_waypoint_step_{0.03490658504};
  double cartesian_velocity_scale_{0.08};
  double cartesian_acceleration_scale_{0.08};
  double cartesian_jump_threshold_{2.0};
  int cartesian_planning_attempts_{5};
  int door_cartesian_segment_waypoints_{2};
  int motion_planning_attempts_{3};
};

}  // namespace xczs_inspection_robot_control

#ifndef XCZS_CABINET_BUTTON_OPERATOR_NO_MAIN
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
#endif
