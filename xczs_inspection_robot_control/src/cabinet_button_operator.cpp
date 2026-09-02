// Copyright 2026
// SPDX-License-Identifier: BSD-3-Clause

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <future>
#include <functional>
#include <limits>
#include <memory>
#include <iomanip>
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

#include "control_msgs/action/follow_joint_trajectory.hpp"
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
#include "xczs_inspection_robot_interfaces/action/operate_cabinet_control.hpp"
#include "xczs_inspection_robot_interfaces/action/press_cabinet_button.hpp"
#include "xczs_inspection_robot_control/action_terminal_policy.hpp"
#include "xczs_inspection_robot_control/cabinet_grasp_safety_policy.hpp"
#include "xczs_inspection_robot_interfaces/msg/cabinet_control.hpp"
#include "xczs_inspection_robot_interfaces/msg/cabinet_control_catalog.hpp"
#include "xczs_inspection_robot_interfaces/msg/cabinet_control_state.hpp"
#include "xczs_inspection_robot_control/operation_validation_policy.hpp"
#include "xczs_inspection_robot_control/rotary_operation_policy.hpp"
#include "xczs_inspection_robot_control/router_utils.hpp"
#include "xczs_inspection_robot_control/staging_safety_policy.hpp"
#include "xczs_inspection_robot_control/structured_control_state_policy.hpp"
#include "xczs_inspection_robot_interfaces/srv/manage_operation_lease.hpp"
#include "xczs_inspection_robot_interfaces/srv/set_cabinet_bimanual_grasp.hpp"
#include "xczs_inspection_robot_interfaces/srv/set_cabinet_grasp.hpp"
#include "xczs_inspection_robot_interfaces/srv/set_cabinet_unlock.hpp"

namespace xczs_inspection_robot_control
{

namespace
{

using namespace std::chrono_literals;
using PressCabinetButton =
  xczs_inspection_robot_interfaces::action::PressCabinetButton;
using PressGoalHandle = rclcpp_action::ServerGoalHandle<PressCabinetButton>;
using OperateCabinetControl =
  xczs_inspection_robot_interfaces::action::OperateCabinetControl;
using OperateGoalHandle =
  rclcpp_action::ServerGoalHandle<OperateCabinetControl>;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using NavigationGoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;
using MoveGroupInterface =
  moveit::planning_interface::MoveGroupInterface;
using ManageOperationLease =
  xczs_inspection_robot_interfaces::srv::ManageOperationLease;

constexpr char kActionName[] = "press_cabinet_button";
constexpr char kOperateActionName[] = "operate_cabinet_control";
constexpr char kControlCatalogTopic[] = "control_catalog";
constexpr char kActiveControlTopic[] = "active_control";
constexpr char kOperationHeartbeatTopic[] = "operation_heartbeat";
constexpr char kOperationFaultTopic[] = "operation_fault";
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
  // Near-grasp standoff used by both knobs and doors.  A pose-only OMPL plan
  // to the far ready pose can land on an IK branch whose long straight inward
  // Cartesian approach self-collides (r_arm_0 <-> r_arm_2).  Planning to a
  // pose only a few mm from the grasp point first keeps the final approach
  // short enough that any branch completes it.
  geometry_msgs::msg::Pose pregrasp_pose;
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
  // Link carrying the cabinet grasp-distance probe.  Defaults to
  // contact_tool_link.  Pincer tools
  // (rotate_button) carry their jaws far from the contact-tool origin, so the
  // body origin can sit well outside the grasp distance threshold even when
  // the jaws are exactly on the control; the jaw-carrier link is a better
  // distance probe.
  std::string grasp_link;
  // Exact grasp-distance probe in grasp_link coordinates.  Keeping the point
  // separate from the MoveIt business point also supports tools whose planned
  // contact link and Gazebo grasp link differ.
  tf2::Vector3 grasp_point_position{0.0, 0.0, 0.0};
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

// Per-arm tool profile for a bimanual drawer operation.  The right
// three-cylinder (right_arm) unlocks and grasps the right handle; the left
// two-cylinder (left_arm) grasps the left handle.  Two separate MoveIt move
// groups and two separate trajectory controllers let both arms move in
// synchrony during the pull/push.
struct BimanualToolProfile
{
  std::string move_group;
  std::string contact_tool_link;
  tf2::Vector3 tool_tip_position{0.0, 0.0, 0.0};
  std::vector<std::string> calibration_joint_names;
  std::vector<double> calibration_joint_positions;
  std::string transport_named_target{"home"};
};

// Which of the two drawer side tools a pose belongs to.  Every drawer-side
// target is computed against that side's tool profile (business-point offset,
// contact link, calibration joints and transport target).
enum class DrawerSide
{
  LEFT,
  RIGHT
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
  double peak_position{0.0};
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
    xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_BUTTON};
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
  // Keep the two policy dimensions explicit in the catalog.  ``operable`` is
  // their conjunction; clients use these fields to explain a mismatch before
  // they request navigation or motion.
  bool adapter_validated{false};
  bool toolset_compatible{false};
  std::string required_toolset;
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
  // Slider close position: pushing the open panel PAST its detent to this
  // position trips the plugin's push-push release, and the detent spring then
  // returns the panel to the closed detent.  The robot can only push a slider
  // face (no fixed grasp / no pull), so closing is a second push, not a drag.
  double slider_release_position{std::numeric_limits<double>::quiet_NaN()};
  // Open press compensation: the tool's effective flat-panel contact extends
  // ~0.025-0.031 m past the calibrated business point (varying with press
  // impulse), so a press aimed at the open detent physically carries the panel
  // further.  On the OPEN push only, subtract this offset from the press
  // target so the panel lands AT the open detent (peak <= ~0.312) instead of
  // riding up against the push-push release (0.34).
  double slider_open_press_offset{0.0};
  // Close press compensation (additive): the plugin's release trips only when
  // raw_position >= slider_release_position, so commanding the close press at
  // EXACTLY the release position is a coin-flip -- the 80 kg panel settles
  // right on the threshold and reads ~0.3399x (float just below 0.34), the
  // release never trips, the weak open-detent spring (0.48 N at 0.34) cannot
  // drag the panel past the tool, and the close hangs until the 90 s settle
  // times out (boot18).  Add this offset so the close press target is strictly
  // ABOVE the release (release + offset), guaranteeing the panel crosses the
  // threshold mid-press; once the release trips the detent target flips to
  // closed and the ~4 N closed spring drags the panel (and the tool) back to
  // the closed detent (boots 14/17).
  double slider_close_press_offset{0.0};
  // Front-drawer operation: the robot docks on the panel's OUTWARD side and
  // grasps the panel front face (plugin fixed grasp joint) instead of pushing
  // the west push-push latch.  The operator then drags the panel linearly
  // along its slide axis -- pull to open, push to close.  graspable=true in
  // the scene SDF is the physical precondition; this flag switches the
  // execution branch from the push-push flow to the grasp-drag flow.
  bool grasp_operated{false};
  // ---- Bimanual pull-out drawer contract (TYPE_DRAWER) ----
  // All points live in the cabinet/local fixture frame (== world/map for a
  // unit-pose scene spawn) and are frozen by the P0 geometry contract.  The
  // drawer slides along drawer_axis (P0 measured +X = east, toward the robot).
  // 2026-09-02: the right three-cylinder physically presses the unlock button
  // (b1p cap on top of the right handle riser) to release the rail latch; both
  // tools then press their tips INTO the two handle plates and the two arms
  // pull/push the drawer in synchrony along the axis.  No more logical-zone
  // proximity unlock, no more hovering "pull in empty air".
  bool drawer_bimanual{false};
  tf2::Vector3 drawer_axis{0.0, 0.0, 0.0};
  bool has_left_handle_point{false};
  tf2::Vector3 left_handle_point{0.0, 0.0, 0.0};
  bool has_right_handle_point{false};
  tf2::Vector3 right_handle_point{0.0, 0.0, 0.0};
  bool has_left_support_point{false};
  tf2::Vector3 left_support_point{0.0, 0.0, 0.0};
  bool has_right_support_point{false};
  tf2::Vector3 right_support_point{0.0, 0.0, 0.0};
  bool has_unlock_press_point{false};
  tf2::Vector3 unlock_press_point{0.0, 0.0, 0.0};
  // Per-drawer approach extension: how far each tool's business point rides
  // OUTWARD (along the drawer outward normal) from its handle when grasping.
  // Two- and three-cylinder effective contact reach differs, so the extension
  // is read from each drawer's contract rather than hard-coded globally.
  double drawer_grasp_outward_offset{0.020};
  // 2026-09-02: the grasp pose and the pull both press each tool tip INTO its
  // handle plate by this depth (west of the riser front face).  The plugin's
  // attach gate and coupling keepalive require both tips within
  // grasp_contact_threshold (~0.02 m) of their handles, so a firm press-in
  // replaces the old 5 cm hover — the tools genuinely contact the drawer.
  double drawer_grasp_press_depth{0.002};
  // The right tool first hovers this far OUTWARD (east) of the unlock button
  // face, then presses IN by drawer_unlock_press_depth to physically displace
  // the b1p cap on its prismatic joint (3 mm travel, collision-verified).
  double drawer_unlock_approach_offset{0.040};
  double drawer_unlock_press_depth{0.003};
  // Right-tool distance from the unlock press point accepted by the plugin
  // latch release; mirrors the scene <unlock_distance_threshold>.
  double drawer_unlock_distance_threshold{0.030};
  // Maximum left/right tool-tip translation mismatch tolerated during the
  // synchronized pull/push before the operation aborts and detaches.
  double drawer_sync_tolerance{0.050};
  double drawer_open_position{0.30};
  double drawer_closed_position{0.0};
  // Optional named MoveIt joint seed for the ready-pose IK.  A redundant arm
  // can otherwise reach the same pose through a branch that cannot continue
  // through the subsequent Cartesian manipulation.
  std::vector<std::string> ready_joint_seed_names;
  std::vector<double> ready_joint_seed_positions;
  std::shared_ptr<ButtonRuntime> runtime{std::make_shared<ButtonRuntime>()};
};

struct ControlStabilityReference
{
  const ButtonSpec * control{nullptr};
  std::string state_id;
  double position{0.0};
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
    // 双臂 drawer 轨迹直发控制器（绕过 move_group 串行 execute 的并发路径）。
    // 控制器一律部署在 /xczs 命名空间（moveit_controllers 映射同名）。
    controller_namespace_ = declare_parameter<std::string>(
      "controller_namespace", "xczs");
    toolset_ = declare_parameter<std::string>("toolset", "A");
    if (toolset_ != "A" && toolset_ != "B") {
      throw std::invalid_argument(
              "Parameter 'toolset' must be 'A' or 'B'.");
    }
    load_tool_profiles();
    tool_tip_calibration_joint_tolerance_ = positive_parameter(
      "tool_tip_calibration_joint_tolerance", 0.001);
    tool_calibration_settle_timeout_ = positive_parameter(
      "tool_calibration_settle_timeout", 6.0);
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
    const auto bimanual_grasp_service = declare_parameter<std::string>(
      "bimanual_grasp_service", "bimanual_grasp");
    const auto unlock_service = declare_parameter<std::string>(
      "unlock_service", "unlock");
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
    // P3-8 grab-and-drive drawer pull: the base translates along the drawer
    // axis at a speed proportional to the remaining drawer travel, capped at
    // drawer_base_drive_max_speed (kept slow so the 80 kg drawer tracks the
    // linear-drag coupling without overshoot).
    drawer_base_drive_max_speed_ = positive_parameter(
      "drawer_base_drive_max_speed", 0.05);
    drawer_base_drive_gain_ = positive_parameter(
      "drawer_base_drive_gain", 1.0);
    drawer_base_drive_timeout_ = positive_parameter(
      "drawer_base_drive_timeout", 60.0);
    const auto & parameter_overrides =
      get_node_parameters_interface()->get_parameter_overrides();
    const bool navigation_yaw_offset_overridden =
      parameter_overrides.count("navigation_velocity_yaw_offset") != 0U;
    const bool legacy_yaw_offset_overridden =
      parameter_overrides.count("base_link_yaw_offset") != 0U;
    const double configured_navigation_yaw_offset = finite_parameter(
      "navigation_velocity_yaw_offset", -1.57079632679);
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
    // A slider's panel settles against friction, so its detent verification
    // tolerates a small rest offset as long as the state has already flipped
    // (the state check is the primary signal; this is a secondary bound).
    slider_position_tolerance_ = positive_parameter(
      "slider_position_tolerance", 0.03);
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
    rotary_pregrasp_clearance_ = positive_parameter(
      "rotary_pregrasp_clearance", 0.010);
    if (rotary_pregrasp_clearance_ > 0.015) {
      throw std::invalid_argument(
              "Parameter 'rotary_pregrasp_clearance' must be no greater "
              "than 0.015 m.");
    }
    // Knob blades sit inside the cabinet face plane while the tool clamps
    // them, so the shared door clearance (<= 15 mm) would put the unconstrained
    // OMPL ready->pregrasp path within jaw reach of the blade and actuate it
    // before grasp.  Knobs instead stop far enough out that the whole tool
    // stays beyond the blade near face during the pose-planned pregrasp, then
    // commit the final straight push as a short validated Cartesian approach.
    // 0.050 m gives a 50 mm final approach (CLEAR on every IK branch) with a
    // >50 mm blade gap; the OMPL pregrasp plan cannot physically clip it.
    knob_pregrasp_clearance_ = positive_parameter(
      "knob_pregrasp_clearance", 0.050);
    if (knob_pregrasp_clearance_ < 0.030 ||
      knob_pregrasp_clearance_ > 0.054)
    {
      throw std::invalid_argument(
              "Parameter 'knob_pregrasp_clearance' must be in "
              "[0.030, 0.054] m.");
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
    drawer_waypoint_step_ = positive_parameter(
      "drawer_waypoint_step", 0.03);
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
      xczs_inspection_robot_interfaces::msg::CabinetControlCatalog>(
      control_catalog_topic,
      rclcpp::QoS(1).reliable().transient_local());
    active_control_publisher_ = create_publisher<std_msgs::msg::String>(
      kActiveControlTopic,
      rclcpp::QoS(1).reliable().transient_local());
    operation_heartbeat_publisher_ =
      create_publisher<std_msgs::msg::String>(
      kOperationHeartbeatTopic, rclcpp::QoS(1).reliable());
    operation_fault_subscription_ =
      create_subscription<std_msgs::msg::String>(
      kOperationFaultTopic, rclcpp::QoS(1).reliable(),
      [this](const std_msgs::msg::String::SharedPtr message) {
        if (message->data.empty()) {
          RCLCPP_WARN(
            get_logger(),
            "Ignoring a cabinet physics watchdog fault without a lease ID.");
          return;
        }
        mark_operation_lease_lost_for_exact_lease(
          "The cabinet physics watchdog reported this operation lease as "
          "abandoned.", message->data);
      });

    navigation_client_ = rclcpp_action::create_client<NavigateToPose>(
      this, navigation_action);
    // 双臂 drawer 的关节轨迹直发各自 follow_joint_trajectory 控制器，让两条
    // 执行真正并行（move_group 的 execute_trajectory action 串行处理目标，
    // 会锁死先动的那条臂——抽屉被另一条臂焊接固定，物理上无法前进）。
    left_fjt_client_ = rclcpp_action::create_client<
      control_msgs::action::FollowJointTrajectory>(
      this, controller_namespace_ + "/left_arm_controller/follow_joint_trajectory");
    right_fjt_client_ = rclcpp_action::create_client<
      control_msgs::action::FollowJointTrajectory>(
      this, controller_namespace_ + "/right_arm_controller/follow_joint_trajectory");
    navigation_mode_client_ = create_client<std_srvs::srv::SetBool>(
      navigation_mode_service);
    grasp_client_ =
      create_client<xczs_inspection_robot_interfaces::srv::SetCabinetGrasp>(
      grasp_service);
    bimanual_grasp_client_ = create_client<
      xczs_inspection_robot_interfaces::srv::SetCabinetBimanualGrasp>(
      bimanual_grasp_service);
    unlock_client_ =
      create_client<xczs_inspection_robot_interfaces::srv::SetCabinetUnlock>(
      unlock_service);
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

  enum class PendingGoalDisposition
  {
    EXECUTE,
    INVALID_CONTROL,
    INVALID_BUTTON,
    RESOURCE_BUSY,
  };

  static std::string goal_uuid_key(
    const rclcpp_action::GoalUUID & goal_id)
  {
    return std::string(
      reinterpret_cast<const char *>(goal_id.data()), goal_id.size());
  }

  void remember_pending_goal(
    const rclcpp_action::GoalUUID & goal_id,
    PendingGoalDisposition disposition)
  {
    std::lock_guard<std::mutex> lock(pending_goal_mutex_);
    pending_goal_dispositions_[goal_uuid_key(goal_id)] = disposition;
  }

  PendingGoalDisposition take_pending_goal(
    const rclcpp_action::GoalUUID & goal_id)
  {
    std::lock_guard<std::mutex> lock(pending_goal_mutex_);
    const auto key = goal_uuid_key(goal_id);
    const auto iterator = pending_goal_dispositions_.find(key);
    if (iterator == pending_goal_dispositions_.end()) {
      return PendingGoalDisposition::INVALID_CONTROL;
    }
    const auto disposition = iterator->second;
    pending_goal_dispositions_.erase(iterator);
    return disposition;
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
    const auto grasp_point_position = declare_parameter<std::vector<double>>(
      prefix + "grasp_point_position", {0.0, 0.0, 0.0});
    if (grasp_point_position.size() != 3U ||
      std::any_of(
        grasp_point_position.begin(), grasp_point_position.end(),
        [](double value) {return !std::isfinite(value);}))
    {
      throw std::invalid_argument(
              "Parameter '" + prefix +
              "grasp_point_position' must contain three finite values.");
    }
    profile.grasp_point_position = tf2::Vector3(
      grasp_point_position[0], grasp_point_position[1],
      grasp_point_position[2]);
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

  BimanualToolProfile read_bimanual_tool_profile(const std::string & side)
  {
    const std::string prefix = "drawer_tools." + side + ".";
    BimanualToolProfile profile;
    profile.move_group = declare_parameter<std::string>(
      prefix + "move_group", "");
    profile.contact_tool_link = declare_parameter<std::string>(
      prefix + "contact_tool_link", "");
    const auto tool_tip_position = declare_parameter<std::vector<double>>(
      prefix + "tool_tip_position", std::vector<double>{0.0, 0.0, 0.0});
    if (tool_tip_position.size() != 3U ||
      std::any_of(
        tool_tip_position.begin(), tool_tip_position.end(),
        [](double value) {return !std::isfinite(value);}))
    {
      throw std::invalid_argument(
              "Parameter '" + prefix +
              "tool_tip_position' must contain three finite values.");
    }
    profile.tool_tip_position = tf2::Vector3(
      tool_tip_position[0], tool_tip_position[1], tool_tip_position[2]);
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
              "Drawer tool '" + side +
              "' calibration joint names/positions must be finite, unique "
              "and have equal length.");
    }
    profile.transport_named_target = declare_parameter<std::string>(
      prefix + "transport_named_target", profile.transport_named_target);
    if (profile.transport_named_target.empty()) {
      throw std::invalid_argument(
              "Drawer tool '" + side +
              "' transport_named_target must be non-empty.");
    }
    return profile;
  }

  void require_drawer_tools_configured() const
  {
    if (!drawer_tools_configured_ ||
      drawer_left_tool_.move_group.empty() ||
      drawer_right_tool_.move_group.empty() ||
      drawer_left_tool_.contact_tool_link.empty() ||
      drawer_right_tool_.contact_tool_link.empty())
    {
      throw std::invalid_argument(
              "Bimanual drawer control requires 'drawer_tools.left' and "
              "'drawer_tools.right' tool profiles (move_group and "
              "contact_tool_link each).");
    }
  }

  void load_tool_profiles()
  {
    using Control = xczs_inspection_robot_interfaces::msg::CabinetControl;
    tool_profiles_[Control::TYPE_BUTTON] = read_tool_profile("button");
    tool_profiles_[Control::TYPE_KNOB] = read_tool_profile("knob");
    tool_profiles_[Control::TYPE_SWITCH] = read_tool_profile("switch");
    tool_profiles_[Control::TYPE_DOOR] = read_tool_profile("door");
    // A slider is driven with the same three-cylinder press tool as a button:
    // the tool tip pushes the panel face along the prismatic axis.  Copy the
    // already-declared button profile (read_tool_profile declares parameters,
    // so calling it again for the same type would throw
    // ParameterAlreadyDeclaredException).
    tool_profiles_[Control::TYPE_SLIDER] = tool_profiles_[Control::TYPE_BUTTON];

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

    // Bimanual drawer tool profiles are optional at the parameter level (a
    // legacy adapter without drawers declares neither), but required by
    // require_drawer_tools_configured() the moment any drawer control exists.
    drawer_left_tool_ = read_bimanual_tool_profile("left");
    drawer_right_tool_ = read_bimanual_tool_profile("right");
    drawer_tools_configured_ =
      !drawer_left_tool_.move_group.empty() &&
      !drawer_right_tool_.move_group.empty() &&
      !drawer_left_tool_.contact_tool_link.empty() &&
      !drawer_right_tool_.contact_tool_link.empty();
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
    grasp_point_position_ = profile.grasp_point_position;
    transport_named_target_ = profile.transport_named_target;
    tool_axis_orientation_ = profile.tool_axis_orientation;
    tool_tip_position_ = profile.tool_tip_position;
    tool_tip_calibration_joint_names_ = profile.calibration_joint_names;
    tool_tip_calibration_joint_positions_ =
      profile.calibration_joint_positions;
  }

  // Bind the legacy single-arm members to the RIGHT drawer tool so the shared
  // transport / docking / calibration code drives the right arm for a drawer
  // operation.  The left arm is driven only through the bimanual helpers
  // (plan_and_execute_bimanual_poses / execute_bimanual_segmented_cartesian
  // _path), which always name their own group and contact link.  The tool-axis
  // orientation is intentionally left untouched: the bimanual drawer uses the
  // same ALONG_OUTWARD convention as the mounted Set-A right tool.
  void apply_drawer_tool_profiles()
  {
    require_drawer_tools_configured();
    move_group_name_ = drawer_right_tool_.move_group;
    contact_tool_link_ = drawer_right_tool_.contact_tool_link;
    grasp_link_ = drawer_right_tool_.contact_tool_link;
    transport_named_target_ = drawer_right_tool_.transport_named_target;
    tool_tip_position_ = drawer_right_tool_.tool_tip_position;
    tool_tip_calibration_joint_names_ =
      drawer_right_tool_.calibration_joint_names;
    tool_tip_calibration_joint_positions_ =
      drawer_right_tool_.calibration_joint_positions;
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

  // 在 arm 运动前把工具标定关节（如三缸手指）主动合拢到标定业务位姿。
  // transport/导航/停靠只规划臂组，未受令的末端关节会自然回落到静息位
  // （约 3mm，恰在校验容差边界），导致 verify_tool_tip_calibration_state
  // 悬空失败。此函数在标定关节所在的 MoveIt 组上规划一段关节空间轨迹，
  // 把它们可靠带回标定位姿；已处于容差内时直接跳过，避免多余运动。
  // execute_motion_bounded 带宽限提前返回，真实轨迹可能仍在飞行中，因此
  // 返回前轮询标定关节直到实际回到业务位姿（或超时），并把合拢后的最新
  // 状态返回给调用方立即校验，避免校验读到 execute 之前的陈旧快照。
  template<typename GoalHandleT>
  moveit::core::RobotStatePtr ensure_calibration_position_for_arm(
    MoveGroupInterface & arm_move_group,
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const std::vector<std::string> & calibration_joint_names,
    const std::vector<double> & calibration_joint_positions)
  {
    if (calibration_joint_names.empty()) {
      // 该工具 profile 无标定关节（knob/switch），无需合拢
      return synchronized_current_robot_state(arm_move_group);
    }
    const auto current_state =
      synchronized_current_robot_state(arm_move_group);
    bool already_calibrated = true;
    for (std::size_t index = 0U;
      index < calibration_joint_names.size(); ++index)
    {
      double measured;
      try {
        measured = current_state->getVariablePosition(
          calibration_joint_names[index]);
      } catch (const std::exception & error) {
        throw OperationError(
                PressCabinetButton::Result::NOT_READY,
                "Tool calibration references unknown joint '" +
                calibration_joint_names[index] + "': " +
                error.what());
      }
      const double expected = calibration_joint_positions[index];
      if (!std::isfinite(measured) ||
        std::abs(measured - expected) > tool_tip_calibration_joint_tolerance_)
      {
        already_calibrated = false;
        break;
      }
    }
    if (already_calibrated) {
      return current_state;
    }
    const auto robot_model = arm_move_group.getRobotModel();
    std::string calibration_group;
    for (const auto * group : robot_model->getJointModelGroups()) {
      bool contains_all = true;
      for (const auto & joint_name : calibration_joint_names) {
        if (group->getJointModel(joint_name) == nullptr) {
          contains_all = false;
          break;
        }
      }
      if (contains_all) {
        calibration_group = group->getName();
        break;
      }
    }
    if (calibration_group.empty()) {
      throw OperationError(
              PressCabinetButton::Result::NOT_READY,
              "Tool calibration joints are not contained in a single MoveIt "
              "joint-model group; cannot close the tool before arm motion.");
    }
    RCLCPP_INFO(
      get_logger(),
      "Closing tool calibration joints via MoveIt group '%s' to their "
      "calibrated business-point positions before arm motion.",
      calibration_group.c_str());
    check_cancel(goal_handle);
    MoveGroupInterface tool_group(
      shared_from_this(),
      MoveGroupInterface::Options(
        calibration_group, "robot_description", move_group_namespace_),
      transform_buffer_,
      rclcpp::Duration::from_seconds(system_wait_timeout_));
    tool_group.setPoseReferenceFrame(planning_frame_);
    tool_group.setPlanningTime(planning_time_);
    tool_group.setNumPlanningAttempts(planning_attempts_);
    tool_group.setMaxVelocityScalingFactor(planning_velocity_scale_);
    tool_group.setMaxAccelerationScalingFactor(planning_acceleration_scale_);
    tool_group.setGoalJointTolerance(goal_joint_tolerance_);
    tool_group.allowReplanning(allow_replanning_);
    tool_group.setJointValueTarget(
      calibration_joint_names,
      calibration_joint_positions);
    MoveGroupInterface::Plan plan;
    const auto planning_result = tool_group.plan(plan);
    if (planning_result != moveit::core::MoveItErrorCode::SUCCESS) {
      throw OperationError(
              PressCabinetButton::Result::NOT_READY,
              "MoveIt could not plan to close tool calibration joints on "
              "group '" + calibration_group + "' to their calibrated "
              "business-point positions.");
    }
    check_cancel(goal_handle);
    const auto close_code = execute_motion_bounded(
      [&tool_group, &plan]() { return tool_group.execute(plan); },
      [this, &goal_handle]() { return goal_should_stop(goal_handle); },
      std::chrono::seconds(30), "tool calibration joints");
    if (close_code != moveit::core::MoveItErrorCode::SUCCESS) {
      check_cancel(goal_handle);
      throw OperationError(
              PressCabinetButton::Result::NOT_READY,
              "MoveIt failed to execute the tool calibration joint-close "
              "trajectory on group '" + calibration_group + "'.");
    }
    // 合拢轨迹执行成功不等同于手指已实际到位：execute_motion_bounded 会
    // 提前返回，真实轨迹可能仍在飞行中。轮询标定关节直到它们真实回到
    // 业务位姿（或超时），期间关节状态话题持续刷新。到位后返回最新状态。
    const auto settle_deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(tool_calibration_settle_timeout_);
    while (std::chrono::steady_clock::now() < settle_deadline) {
      check_cancel(goal_handle);
      const auto settled_state =
        synchronized_current_robot_state(arm_move_group);
      bool within_tolerance = true;
      for (std::size_t index = 0U;
        index < calibration_joint_names.size(); ++index)
      {
        double measured;
        try {
          measured = settled_state->getVariablePosition(
            calibration_joint_names[index]);
        } catch (const std::exception & error) {
          throw OperationError(
                  PressCabinetButton::Result::NOT_READY,
                  "Tool calibration references unknown joint '" +
                  calibration_joint_names[index] + "': " +
                  error.what());
        }
        const double expected = calibration_joint_positions[index];
        if (!std::isfinite(measured) ||
          std::abs(measured - expected) > tool_tip_calibration_joint_tolerance_)
        {
          within_tolerance = false;
          break;
        }
      }
      if (within_tolerance) {
        return settled_state;
      }
      std::this_thread::sleep_for(50ms);
    }
    check_cancel(goal_handle);
    // 超时未到位：给出逐关节实测 vs 期望，便于排查控制器静差/质量等。
    std::string detail;
    const auto final_state = synchronized_current_robot_state(arm_move_group);
    for (std::size_t index = 0U;
      index < calibration_joint_names.size(); ++index)
    {
      double measured = std::numeric_limits<double>::quiet_NaN();
      try {
        measured = final_state->getVariablePosition(
          calibration_joint_names[index]);
      } catch (const std::exception &) {
      }
      const double expected = calibration_joint_positions[index];
      if (!detail.empty()) {
        detail += "; ";
      }
      detail += calibration_joint_names[index] + "=" +
        (std::isfinite(measured) ? std::to_string(measured) : "nan") +
        " (expected " + std::to_string(expected) + ")";
    }
    throw OperationError(
            PressCabinetButton::Result::NOT_READY,
            "Tool calibration joints did not settle to their calibrated "
            "business-point positions within " +
            std::to_string(tool_calibration_settle_timeout_) + " s after "
            "the joint-close trajectory: " + detail);
  }
  // 单臂调用点：以当前已应用的工具 profile 的标定关节执行合拢。
  template<typename GoalHandleT>
  moveit::core::RobotStatePtr ensure_tool_calibration_position(
    MoveGroupInterface & arm_move_group,
    const std::shared_ptr<GoalHandleT> & goal_handle)
  {
    return ensure_calibration_position_for_arm(
      arm_move_group, goal_handle,
      tool_tip_calibration_joint_names_, tool_tip_calibration_joint_positions_);
  }

  // Close BOTH drawer tool profiles' calibration joints before any bimanual
  // arm motion.  The right arm reuses the single-arm path (the members are
  // bound to the right drawer profile); the left arm closes its own joints.
  // Returns the settled states in (left, right) order for immediate verify.
  template<typename GoalHandleT>
  std::pair<moveit::core::RobotStatePtr, moveit::core::RobotStatePtr>
  ensure_bimanual_calibration_position(
    MoveGroupInterface & left_group,
    MoveGroupInterface & right_group,
    const std::shared_ptr<GoalHandleT> & goal_handle)
  {
    const auto right_state =
      ensure_tool_calibration_position(right_group, goal_handle);
    const auto left_state = ensure_calibration_position_for_arm(
      left_group, goal_handle,
      drawer_left_tool_.calibration_joint_names,
      drawer_left_tool_.calibration_joint_positions);
    return {left_state, right_state};
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

  void release_failed_goal_resources(
    ActiveGoalType type,
    const rclcpp_action::GoalUUID & goal_id) noexcept
  {
    clear_latched_cabinet_transform();
    if (active_goal_owns_physical_motion_resources(type, goal_id)) {
      stop_active_motion();
      cancel_active_navigation();
      request_navigation_mode_without_wait(false);
    }
    {
      std::lock_guard<std::mutex> lock(motion_mutex_);
      active_move_group_.reset();
    }
    relinquish_active_goal_physical_motion_resources(type, goal_id);
    release_operation_lease_noexcept();
    publish_active_control("");
  }

  void abort_pending_press_goal(
    const std::shared_ptr<PressGoalHandle> & goal_handle,
    std::uint8_t error_code,
    const std::string & message,
    bool release_operation_slot) noexcept
  {
    auto result = std::make_shared<PressCabinetButton::Result>();
    result->success = false;
    result->error_code = error_code;
    result->message = message;
    result->failure_reason = message;
    try {
      if (goal_handle && goal_handle->is_active()) {
        goal_handle->abort(result);
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        get_logger(), "Failed to abort pending button goal: %s", error.what());
    } catch (...) {
      RCLCPP_ERROR(get_logger(), "Failed to abort pending button goal.");
    }
    if (release_operation_slot && goal_handle) {
      release_failed_goal_resources(
        ActiveGoalType::PRESS, goal_handle->get_goal_id());
      clear_active_goal(ActiveGoalType::PRESS, goal_handle->get_goal_id());
      operation_active_.store(false);
    }
  }

  void abort_pending_operate_goal(
    const std::shared_ptr<OperateGoalHandle> & goal_handle,
    std::uint8_t error_code,
    const std::string & message,
    bool release_operation_slot) noexcept
  {
    auto result = std::make_shared<OperateCabinetControl::Result>();
    result->success = false;
    result->error_code = error_code;
    result->message = message;
    result->failure_reason = message;
    result->diagnostic_stage = "accepted_callback";
    try {
      if (goal_handle && goal_handle->is_active()) {
        goal_handle->abort(result);
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        get_logger(), "Failed to abort pending cabinet goal: %s", error.what());
    } catch (...) {
      RCLCPP_ERROR(get_logger(), "Failed to abort pending cabinet goal.");
    }
    if (release_operation_slot && goal_handle) {
      release_failed_goal_resources(
        ActiveGoalType::OPERATE, goal_handle->get_goal_id());
      clear_active_goal(ActiveGoalType::OPERATE, goal_handle->get_goal_id());
      operation_active_.store(false);
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
  // physically operable; execute_operate() rejects the unmounted set before
  // changing a MoveIt profile or looking up TF.
  bool tool_serves_control(std::uint8_t control_type) const
  {
    using Control = xczs_inspection_robot_interfaces::msg::CabinetControl;
    if (toolset_ == "B") {
      return control_type == Control::TYPE_KNOB ||
             control_type == Control::TYPE_SWITCH;
    }
    return control_type == Control::TYPE_BUTTON ||
           control_type == Control::TYPE_DOOR ||
           control_type == Control::TYPE_SLIDER ||
           control_type == Control::TYPE_DRAWER;
  }

  std::string required_toolset_for_control(std::uint8_t control_type) const
  {
    using Control = xczs_inspection_robot_interfaces::msg::CabinetControl;
    if (control_type == Control::TYPE_KNOB ||
      control_type == Control::TYPE_SWITCH)
    {
      return "B";
    }
    return "A";
  }

  static bool is_slider_type(std::uint8_t control_type)
  {
    using Control = xczs_inspection_robot_interfaces::msg::CabinetControl;
    return control_type == Control::TYPE_SLIDER;
  }

  static bool is_drawer_type(std::uint8_t control_type)
  {
    using Control = xczs_inspection_robot_interfaces::msg::CabinetControl;
    return control_type == Control::TYPE_DRAWER;
  }

  static bool is_grasp_free_type(std::uint8_t control_type)
  {
    using Control = xczs_inspection_robot_interfaces::msg::CabinetControl;
    return control_type == Control::TYPE_BUTTON ||
           control_type == Control::TYPE_SLIDER;
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
    // A control whose operating tool is not mounted in the current
    // end-effector toolset (e.g. Set A cannot rotate a knob) must report the
    // toolset mismatch before the generic planning-only policy.  This gives
    // the Web client a concrete corrective action even when the control is
    // not part of the physical-operation allowlist.
    const auto toolset_mismatch_reason = declare_parameter<std::string>(
      "toolset_mismatch_reason",
      "The control's operating tool is not mounted in the current end-effector "
      "toolset; status display remains available, but valid MoveIt planning "
      "and physical execution require switching to the matching toolset.");
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
          xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_BUTTON;
      } else if (type_name == "knob") {
        button->control_type =
          xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_KNOB;
      } else if (type_name == "switch") {
        button->control_type =
          xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_SWITCH;
      } else if (type_name == "door") {
        button->control_type =
          xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_DOOR;
      } else if (type_name == "slider") {
        button->control_type =
          xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_SLIDER;
      } else if (type_name == "drawer") {
        button->control_type =
          xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_DRAWER;
      } else {
        throw std::invalid_argument(
                "Unsupported control type '" + type_name + "' for '" +
                control_id + "'.");
      }
      const bool is_button = button->control_type ==
        xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_BUTTON;
      const bool is_knob = button->control_type ==
        xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_KNOB;
      const bool is_switch = button->control_type ==
        xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_SWITCH;
      const bool is_slider = is_slider_type(button->control_type);
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
      button->adapter_validated = in_allowlist;
      button->toolset_compatible = tool_serves;
      button->required_toolset = required_toolset_for_control(
        button->control_type);
      button->operable = in_allowlist && tool_serves;
      button->unavailable_reason = declare_parameter<std::string>(
        prefix + "unavailable_reason", "");
      if (button->operable && !button->unavailable_reason.empty()) {
        throw std::invalid_argument(
                "Operable control '" + control_id +
                "' must not declare an unavailable_reason.");
      }
      if (!button->operable && button->unavailable_reason.empty()) {
        // 当前套装未挂载其操作工具时，先报告“套装不匹配”，而非笼统地报告
        // 未列入物理操作清单；切换到正确套装后才可能进入规划验证路径。
        button->unavailable_reason = !tool_serves ?
          toolset_mismatch_reason :
          inoperable_control_reason;
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
      button->grasp_operated = declare_parameter<bool>(
        prefix + "grasp_operated", false);
      const bool is_drawer = is_drawer_type(button->control_type);
      if (button->grasp_operated && !is_slider && !is_drawer) {
        throw std::invalid_argument(
                "Control '" + control_id +
                "' grasp_operated is only supported for slider and drawer "
                "controls.");
      }
      // A front-operated drawer (grasp_operated) is no longer a grasp-free
      // control: its execution branch attaches the probe to the panel and drags
      // it.  Derive the effective grasp-free flag so requires_grasp, the grasp
      // outward offset and the unit all follow the intended branch.
      const bool is_grasp_free =
        is_grasp_free_type(button->control_type) && !button->grasp_operated;
      button->requires_grasp = declare_parameter<bool>(
        prefix + "requires_grasp", !is_grasp_free);
      if (!grasp_requirement_matches_control_kind(
          is_grasp_free, button->requires_grasp))
      {
        throw std::invalid_argument(
                "Control '" + control_id + "' requires_grasp must be " +
                (is_grasp_free ?
                "false for a button or slider." :
                "true for a knob, switch or door."));
      }
      if (!is_grasp_free) {
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
      // A drawer travels linearly along its rail (m); a grasped rotary control
      // reports angular travel (rad).
      button->unit = (is_grasp_free || is_drawer) ? "m" : "rad";
      if (is_drawer) {
        // Bimanual pull-out drawer contract (P0-frozen geometry).  The drawer
        // branch drives two arms, so it requires the full dual-side contract
        // (left/right handles, supports, the right-hand unlock zone and the
        // slide axis).  Missing fields fail loudly at startup rather than at
        // operation time.
        require_drawer_tools_configured();
        button->drawer_bimanual = true;
        button->drawer_axis = checked_vector3(
          declare_parameter<std::vector<double>>(
            prefix + "drawer_axis", std::vector<double>{}),
          prefix + "drawer_axis");
        if (button->drawer_axis.length2() <= 1.0e-12) {
          throw std::invalid_argument(
                  "Control '" + control_id +
                  "' drawer_axis must be a non-zero vector.");
        }
        button->drawer_axis.normalize();
        const auto require_point = [&](const char * key,
            tf2::Vector3 & target) {
          target = checked_vector3(
            declare_parameter<std::vector<double>>(
              prefix + key, std::vector<double>{}),
            prefix + key);
        };
        require_point("left_handle_point", button->left_handle_point);
        button->has_left_handle_point = true;
        require_point("right_handle_point", button->right_handle_point);
        button->has_right_handle_point = true;
        require_point("left_support_point", button->left_support_point);
        button->has_left_support_point = true;
        require_point("right_support_point", button->right_support_point);
        button->has_right_support_point = true;
        require_point("unlock_press_point", button->unlock_press_point);
        button->has_unlock_press_point = true;
        button->drawer_grasp_outward_offset = declare_parameter<double>(
          prefix + "drawer_grasp_outward_offset",
          button->drawer_grasp_outward_offset);
        if (!std::isfinite(button->drawer_grasp_outward_offset) ||
          button->drawer_grasp_outward_offset < 0.0)
        {
          throw std::invalid_argument(
                  "Control '" + control_id +
                  "' drawer_grasp_outward_offset must be finite and "
                  "non-negative.");
        }
        button->drawer_grasp_press_depth = declare_parameter<double>(
          prefix + "drawer_grasp_press_depth",
          button->drawer_grasp_press_depth);
        if (!std::isfinite(button->drawer_grasp_press_depth) ||
          button->drawer_grasp_press_depth < 0.0)
        {
          throw std::invalid_argument(
                  "Control '" + control_id +
                  "' drawer_grasp_press_depth must be finite and "
                  "non-negative.");
        }
        button->drawer_unlock_approach_offset = declare_parameter<double>(
          prefix + "drawer_unlock_approach_offset",
          button->drawer_unlock_approach_offset);
        if (!std::isfinite(button->drawer_unlock_approach_offset) ||
          button->drawer_unlock_approach_offset <= 0.0)
        {
          throw std::invalid_argument(
                  "Control '" + control_id +
                  "' drawer_unlock_approach_offset must be positive and "
                  "finite.");
        }
        button->drawer_unlock_press_depth = declare_parameter<double>(
          prefix + "drawer_unlock_press_depth",
          button->drawer_unlock_press_depth);
        if (!std::isfinite(button->drawer_unlock_press_depth) ||
          button->drawer_unlock_press_depth <= 0.0)
        {
          throw std::invalid_argument(
                  "Control '" + control_id +
                  "' drawer_unlock_press_depth must be positive and finite.");
        }
        button->drawer_unlock_distance_threshold = declare_parameter<double>(
          prefix + "drawer_unlock_distance_threshold",
          button->drawer_unlock_distance_threshold);
        if (!std::isfinite(button->drawer_unlock_distance_threshold) ||
          button->drawer_unlock_distance_threshold <= 0.0)
        {
          throw std::invalid_argument(
                  "Control '" + control_id +
                  "' drawer_unlock_distance_threshold must be positive and "
                  "finite.");
        }
        button->drawer_sync_tolerance = declare_parameter<double>(
          prefix + "drawer_sync_tolerance", button->drawer_sync_tolerance);
        if (!std::isfinite(button->drawer_sync_tolerance) ||
          button->drawer_sync_tolerance <= 0.0)
        {
          throw std::invalid_argument(
                  "Control '" + control_id +
                  "' drawer_sync_tolerance must be positive and finite.");
        }
        if (button->state_ids.size() >= 2U &&
          button->state_positions.size() >= 2U)
        {
          // closed/open detents are the operation endpoints; the remaining
          // fields fall back to the frozen 0.0 / 0.30 defaults.
          button->drawer_closed_position = button->state_positions.front();
          button->drawer_open_position = button->state_positions.back();
        }
        if (button->drawer_open_position <= button->drawer_closed_position) {
          throw std::invalid_argument(
                  "Control '" + control_id +
                  "' open detent must be strictly greater than the closed "
                  "detent.");
        }
      }
      if (is_slider) {
        button->slider_release_position = declare_parameter<double>(
          prefix + "slider_release_position",
          std::numeric_limits<double>::quiet_NaN());
        button->slider_open_press_offset = declare_parameter<double>(
          prefix + "slider_open_press_offset", 0.0);
        if (!std::isfinite(button->slider_open_press_offset) ||
          button->slider_open_press_offset < 0.0)
        {
          throw std::invalid_argument(
                  "Slider control '" + control_id +
                  "' has an invalid slider_open_press_offset (must be "
                  "finite and non-negative).");
        }
        button->slider_close_press_offset = declare_parameter<double>(
          prefix + "slider_close_press_offset", 0.0);
        if (!std::isfinite(button->slider_close_press_offset) ||
          button->slider_close_press_offset < 0.0)
        {
          throw std::invalid_argument(
                  "Slider control '" + control_id +
                  "' has an invalid slider_close_press_offset (must be "
                  "finite and non-negative).");
        }
        if (std::isfinite(button->slider_release_position) &&
          std::isfinite(button->slider_close_press_offset) &&
          button->slider_release_position + button->slider_close_press_offset >
            button->max_position + 1e-6)
        {
          throw std::invalid_argument(
                  "Slider control '" + control_id +
                  "' requires slider_release_position + "
                  "slider_close_press_offset <= max_position so the close "
                  "press target stays within the joint travel.");
        }
      }
      // A knob is deliberately single-detent only: wrapping TOGGLE from the
      // right detent to the left detent would cross an intermediate detent
      // and is rejected by the physical transition guard.  Switches and
      // doors have two-state semantics and may expose TOGGLE.
      if (is_button) {
        button->supported_commands =
          xczs_inspection_robot_interfaces::msg::CabinetControl::SUPPORT_PRESS;
      } else {
        button->supported_commands = static_cast<std::uint8_t>(
          xczs_inspection_robot_interfaces::msg::CabinetControl::SUPPORT_SET_STATE |
          xczs_inspection_robot_interfaces::msg::CabinetControl::
          SUPPORT_SET_POSITION |
          ((is_knob || is_drawer) ? 0U :
          xczs_inspection_robot_interfaces::msg::CabinetControl::SUPPORT_TOGGLE));
      }
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
      // A front-operated drawer (grasp_operated) has no push-push release
      // mechanism: the panel is dragged along the slide axis and there is no
      // over-travel trip point, so slider_release_position is intentionally
      // absent for such controls.
      if (is_slider && !button->grasp_operated &&
        (!std::isfinite(button->slider_release_position) ||
        button->state_positions.empty() ||
        button->slider_release_position <= button->state_positions.back() ||
        button->slider_release_position > button->max_position))
      {
        throw std::invalid_argument(
                "Slider control '" + control_id +
                "' requires a slider_release_position strictly above the "
                "open detent and no further than max_position, so the robot "
                "can trip the push-push release.");
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
          xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_BUTTON)
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
          xczs_inspection_robot_interfaces::msg::CabinetControlState>(
          button->state_topic,
          rclcpp::QoS(1).reliable().transient_local(),
          [this, button](
            const xczs_inspection_robot_interfaces::msg::CabinetControlState::
            SharedPtr message)
          {
            receive_control_state(*button, *message);
          }));
    }
  }

  void publish_control_catalog()
  {
    xczs_inspection_robot_interfaces::msg::CabinetControlCatalog catalog;
    catalog.controls.reserve(buttons_in_order_.size());
    for (const auto & button : buttons_in_order_) {
      xczs_inspection_robot_interfaces::msg::CabinetControl control;
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
      control.required_toolset = button->required_toolset;
      control.toolset_compatible = button->toolset_compatible;
      control.adapter_validated = button->adapter_validated;
      if (button->control_type ==
        xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_BUTTON)
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
    std::string operation_lease_id;
    {
      std::lock_guard<std::mutex> lock(operation_heartbeat_mutex_);
      operation_heartbeat_control_id_ = control_id;
    }
    if (!control_id.empty()) {
      std::lock_guard<std::mutex> lock(operation_lease_mutex_);
      if (operation_lease_held_.load() && !operation_lease_lost_.load()) {
        operation_lease_id = operation_lease_id_;
      }
    }
    publish_active_control_message(control_id, operation_lease_id);
    if (!control_id.empty()) {
      // Acquiring the lease is also a valid initial liveness event; periodic
      // heartbeats thereafter are emitted only after successful renewals.
      publish_operation_heartbeat();
    }
  }

  void publish_operation_heartbeat() noexcept
  {
    std::string control_id;
    std::string lease_id;
    {
      std::lock_guard<std::mutex> lock(operation_heartbeat_mutex_);
      control_id = operation_heartbeat_control_id_;
    }
    {
      std::lock_guard<std::mutex> lock(operation_lease_mutex_);
      if (operation_lease_held_.load() && !operation_lease_lost_.load()) {
        lease_id = operation_lease_id_;
      }
    }
    if (control_id.empty() || lease_id.empty()) {
      return;
    }
    try {
      std_msgs::msg::String message;
      // The active-control topic already carries the control identity.  The
      // volatile heartbeat carries the globally unique lease identity so a
      // delayed request/heartbeat from an older same-control operation cannot
      // authorize a new physical grasp.
      message.data = lease_id;
      operation_heartbeat_publisher_->publish(message);
    } catch (const std::exception & error) {
      mark_operation_lease_lost_for_exact_lease(
        std::string("Failed to publish the cabinet operation heartbeat: ") +
        error.what(), lease_id);
    } catch (...) {
      mark_operation_lease_lost_for_exact_lease(
        "Failed to publish the cabinet operation heartbeat unexpectedly.",
        lease_id);
    }
  }

  void publish_active_control_message(
    const std::string & control_id,
    const std::string & operation_lease_id) noexcept
  {
    try {
      std_msgs::msg::String message;
      message.data = control_id;
      active_control_publisher_->publish(message);
    } catch (const std::exception & error) {
      if (!control_id.empty()) {
        mark_operation_lease_lost_for_exact_lease(
          std::string("Failed to publish the active cabinet control: ") +
          error.what(), operation_lease_id);
      } else {
        RCLCPP_ERROR(
          get_logger(), "Failed to publish the idle cabinet control: %s",
          error.what());
      }
    } catch (...) {
      if (!control_id.empty()) {
        mark_operation_lease_lost_for_exact_lease(
          "Failed to publish the active cabinet control unexpectedly.",
          operation_lease_id);
      } else {
        RCLCPP_ERROR(
          get_logger(),
          "Failed to publish the idle cabinet control unexpectedly.");
      }
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
    // Always accept at the transport layer.  Unknown controls and resource
    // contention are recorded and converted to an action result in the
    // accepted callback, so clients never lose the failure reason in a bare
    // GoalResponse::REJECT.
    const auto control = find_button(goal->control_id);
    PendingGoalDisposition disposition =
      control ? PendingGoalDisposition::EXECUTE :
      PendingGoalDisposition::INVALID_CONTROL;
    bool expected = false;
    if (disposition == PendingGoalDisposition::EXECUTE &&
      !operation_active_.compare_exchange_strong(expected, true))
    {
      disposition = PendingGoalDisposition::RESOURCE_BUSY;
    }
    if (disposition == PendingGoalDisposition::EXECUTE) {
      activate_goal(ActiveGoalType::OPERATE, uuid);
    }
    remember_pending_goal(uuid, disposition);
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
    const auto disposition = take_pending_goal(goal_handle->get_goal_id());
    if (disposition != PendingGoalDisposition::EXECUTE) {
      abort_pending_operate_goal(
        goal_handle,
        disposition == PendingGoalDisposition::RESOURCE_BUSY ?
        OperateCabinetControl::Result::RESOURCE_BUSY :
        OperateCabinetControl::Result::INVALID_CONTROL,
        disposition == PendingGoalDisposition::RESOURCE_BUSY ?
        "Cabinet operation rejected because another operation is active." :
        "Unknown cabinet control; no configured control matches control_id.",
        false);
      return;
    }
    std::lock_guard<std::mutex> lock(worker_mutex_);
    try {
      if (worker_thread_.joinable()) {
        worker_thread_.join();
      }
      worker_thread_ = std::thread(
        [this, goal_handle]() {
          try {
            execute_operate(goal_handle);
          } catch (const std::exception & error) {
            abort_pending_operate_goal(
              goal_handle,
              OperateCabinetControl::Result::INTERNAL_ERROR,
              std::string("Cabinet worker terminated unexpectedly: ") +
              error.what(), true);
          } catch (...) {
            abort_pending_operate_goal(
              goal_handle,
              OperateCabinetControl::Result::INTERNAL_ERROR,
              "Cabinet worker terminated unexpectedly.", true);
          }
        });
    } catch (const std::exception & error) {
      abort_pending_operate_goal(
        goal_handle,
        OperateCabinetControl::Result::INTERNAL_ERROR,
        std::string("Failed to start cabinet worker: ") + error.what(), true);
    } catch (...) {
      abort_pending_operate_goal(
        goal_handle,
        OperateCabinetControl::Result::INTERNAL_ERROR,
        "Failed to start cabinet worker.", true);
    }
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
    PendingGoalDisposition disposition =
      control && control->control_type ==
      xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_BUTTON ?
      PendingGoalDisposition::EXECUTE : PendingGoalDisposition::INVALID_BUTTON;
    bool expected = false;
    if (disposition == PendingGoalDisposition::EXECUTE &&
      !operation_active_.compare_exchange_strong(expected, true))
    {
      disposition = PendingGoalDisposition::RESOURCE_BUSY;
    }
    if (disposition == PendingGoalDisposition::EXECUTE) {
      activate_goal(ActiveGoalType::PRESS, uuid);
    }
    remember_pending_goal(uuid, disposition);
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
    const auto disposition = take_pending_goal(goal_handle->get_goal_id());
    if (disposition != PendingGoalDisposition::EXECUTE) {
      abort_pending_press_goal(
        goal_handle,
        disposition == PendingGoalDisposition::RESOURCE_BUSY ?
        PressCabinetButton::Result::RESOURCE_BUSY :
        PressCabinetButton::Result::INVALID_BUTTON,
        disposition == PendingGoalDisposition::RESOURCE_BUSY ?
        "Cabinet button operation rejected because another operation is active." :
        "Unknown cabinet button; no configured button matches button_id.",
        false);
      return;
    }
    std::lock_guard<std::mutex> lock(worker_mutex_);
    try {
      if (worker_thread_.joinable()) {
        worker_thread_.join();
      }
      worker_thread_ = std::thread(
        [this, goal_handle]() {
          try {
            execute(goal_handle);
          } catch (const std::exception & error) {
            abort_pending_press_goal(
              goal_handle,
              PressCabinetButton::Result::INTERNAL_ERROR,
              std::string("Cabinet worker terminated unexpectedly: ") +
              error.what(), true);
          } catch (...) {
            abort_pending_press_goal(
              goal_handle,
              PressCabinetButton::Result::INTERNAL_ERROR,
              "Cabinet worker terminated unexpectedly.", true);
          }
        });
    } catch (const std::exception & error) {
      abort_pending_press_goal(
        goal_handle,
        PressCabinetButton::Result::INTERNAL_ERROR,
        std::string("Failed to start cabinet worker: ") + error.what(), true);
    } catch (...) {
      abort_pending_press_goal(
        goal_handle,
        PressCabinetButton::Result::INTERNAL_ERROR,
        "Failed to start cabinet worker.", true);
    }
  }

  void execute(const std::shared_ptr<PressGoalHandle> goal_handle)
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
        button_snapshot(*button).peak_position : 0.0;
      RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
      finish_goal_noexcept(goal_handle, result, false);
      clear_active_goal(ActiveGoalType::PRESS, goal_handle->get_goal_id());
      operation_active_.store(false);
      return;
    }

    try {
      if (!button) {
        throw OperationError(
                PressCabinetButton::Result::INVALID_BUTTON,
                "The accepted cabinet button is no longer configured.");
      }
      if (!tool_serves_control(button->control_type)) {
        throw OperationError(
                PressCabinetButton::Result::TOOLSET_MISMATCH,
                "Button '" + button->id + "' requires end-effector toolset " +
                button->required_toolset + ", but the mounted toolset is " +
                toolset_ + ".");
      }
      if (!button->adapter_validated) {
        throw OperationError(
                PressCabinetButton::Result::ADAPTER_NOT_VALIDATED,
                button->unavailable_reason.empty() ?
                "This button is not physically validated by the robot "
                "adapter; use the generic cabinet operation for planning-only "
                "validation." : button->unavailable_reason);
      }
      // Bind the button operation's arm group, tip link, contact tool link and
      // transport target to the button type before any MoveIt planning.
      apply_tool_profile(
        xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_BUTTON);
      const bool should_navigate_to_staging_pose =
        goal_handle->get_goal()->navigate_to_staging_pose;
      acquire_operation_lease(
        goal_handle, ActiveGoalType::PRESS, true);
      if (!should_navigate_to_staging_pose) {
        publish_active_control(button->id);
      }
      reset_peak_position(*button);
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
      {
        // A fresh MoveGroupInterface replaces any wedged one from a prior
        // goal: abandon the execute serialization a wedged worker is holding
        // so the new goal can move on its own independent instance.  The old
        // worker's in-flight execute() only touches the old instance, which it
        // keeps alive through its own action client.
        std::lock_guard<std::mutex> lock(motion_execute_serial_->guard_mutex);
        motion_execute_serial_->wedged_lock.reset();
      }
      configure_move_group(*move_group);
      move_group_ready_for_motion = true;
      const auto initial_robot_state =
        ensure_tool_calibration_position(*move_group, goal_handle);
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
        ensure_tool_calibration_position(*move_group, goal_handle);
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
      result->physical_outcome_confirmed = true;
      result->final_state_verified = true;

      publish_feedback(
        goal_handle,
        PressCabinetButton::Feedback::RETRACTING,
        0.99F,
        "Returning the arm to its safe transport target.");
      plan_and_execute_named_target(
        *move_group, goal_handle, transport_named_target_, nullptr, true);
      result->transport_succeeded = true;

      result->success = true;
      result->error_code = PressCabinetButton::Result::SUCCESS;
      result->message =
        "Pressed and released " + button->id + " successfully.";
      result->max_travel = button_snapshot(*button).peak_position;
      result->recovery_succeeded = true;
      result->grasp_released = true;
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
        button_snapshot(*button).peak_position : 0.0;
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
        button_snapshot(*button).peak_position : 0.0;
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
        button_snapshot(*button).peak_position : 0.0;
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
    const std::shared_ptr<OperateGoalHandle> goal_handle)
  {
    const auto operation_started_at = std::chrono::steady_clock::now();
    auto result = std::make_shared<OperateCabinetControl::Result>();
    result->diagnostic_stage = "preflight";
    const auto control = find_button(goal_handle->get_goal()->control_id);
    std::shared_ptr<MoveGroupInterface> move_group;
    OperationPoses button_poses;
    OperationPoses slider_poses;
    RotaryOperationPoses rotary_poses;
    // Grasp-drag staging for a front-operated drawer (grasp_operated slider).
    RotaryOperationPoses drawer_poses;
    std::vector<geometry_msgs::msg::Pose> drawer_waypoints;
    std::optional<ControlStagingPoses> staging_poses;
    bool should_attempt_retreat = false;
    bool grasp_attached = false;
    bool request_success = false;
    DoorArcProgress door_arc_progress;
    double target_position = 0.0;
    double rotary_tool_roll_offset = 0.0;
    // Precomputed upstream so the manipulation block and the ready/pregrasp
    // branch selection share the exact same arc targets and waypoints.
    double rotary_manip_pos = 0.0;
    std::vector<geometry_msgs::msg::Pose> rotary_arc_waypoints;
    double button_press_depth = 0.0;
    bool button_should_trigger = false;
    bool is_button = false;
    bool is_slider = false;
    bool is_drawer = false;
    // Bimanual drawer state: the left arm runs on its own group, the right arm
    // on the shared move_group.  drawer_bimanual_attached tracks the plugin's
    // two-sided fixed constraint so recovery knows whether the drawer is still
    // rigidly held before it retreats either arm.
    std::shared_ptr<MoveGroupInterface> drawer_left_move_group;
    bool drawer_bimanual_attached = false;
    DrawerBimanualPoses drawer_bimanual_poses;
    std::vector<geometry_msgs::msg::Pose> drawer_left_waypoints;
    std::vector<geometry_msgs::msg::Pose> drawer_right_waypoints;
    const bool validation_only = control &&
      requires_planning_only_validation(control->operable);
    const auto preparation_policy = operation_preparation_policy(
      validation_only,
      goal_handle->get_goal()->navigate_to_staging_pose,
      control && control->navigation_station.has_value());
    std::string target_state;
    std::unordered_map<std::string, std::string> expected_parent_states;
    std::vector<ControlStabilityReference> pregrasp_stability_references;

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
      if (!tool_serves_control(control->control_type)) {
        const std::string required_toolset =
          required_toolset_for_control(control->control_type);
        result->success = false;
        result->error_code =
          OperateCabinetControl::Result::TOOLSET_MISMATCH;
        result->message = "Control '" + control->id +
          "' requires end-effector toolset " + required_toolset +
          ", but the mounted toolset is " + toolset_ + ".";
        result->validation_performed = false;
        result->operation_executed = false;
        result->diagnostic_stage = "toolset_validation";
        result->policy_reason = control->unavailable_reason.empty() ?
          "Switch to end-effector toolset " + required_toolset +
          " before operating this control." :
          control->unavailable_reason;
        result->failure_reason = result->policy_reason;
        RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
        finish_operate_goal_noexcept(goal_handle, result, false);
        clear_active_goal(ActiveGoalType::OPERATE, goal_handle->get_goal_id());
        operation_active_.store(false);
        return;
      }
      // Bind this operation's arm group, tip link, contact tool link and
      // transport target to the control's type before any MoveIt planning.
      // A bimanual drawer binds the single-arm members to the RIGHT drawer
      // tool so the shared transport/calibration/docking code drives the right
      // arm; the left arm is driven only through the bimanual helpers.
      if (is_drawer_type(control->control_type)) {
        apply_drawer_tool_profiles();
      } else {
        apply_tool_profile(control->control_type);
      }
      if (validation_only) {
        result->policy_reason = control->unavailable_reason.empty() ?
          "The selected control has not passed this robot adapter's complete "
          "physical operation and recovery validation." :
          control->unavailable_reason;
        result->diagnostic_stage = "preflight";
      }
      is_button = control->control_type ==
        xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_BUTTON;
      is_slider = is_slider_type(control->control_type);
      is_drawer = is_drawer_type(control->control_type);
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
            command_valid = (control->supported_commands &
              xczs_inspection_robot_interfaces::msg::CabinetControl::
              SUPPORT_SET_STATE) != 0U;
            if (!command_valid) {
              command_reason = "This control does not support setting a state.";
            }
            if (command_valid) {
              command_valid = std::find(
                control->state_ids.begin(), control->state_ids.end(),
                goal_handle->get_goal()->target_state) !=
                control->state_ids.end();
              if (!command_valid) {
                command_reason = "target_state '" +
                  goal_handle->get_goal()->target_state +
                  "' is not in the control's state list.";
              }
            }
          } else if (cmd ==
            OperateCabinetControl::Goal::COMMAND_SET_POSITION)
          {
            command_valid = (control->supported_commands &
              xczs_inspection_robot_interfaces::msg::CabinetControl::
              SUPPORT_SET_POSITION) != 0U;
            if (!command_valid) {
              command_reason =
                "This control does not support setting a position.";
            }
            if (command_valid) {
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
            }
          } else if (cmd == OperateCabinetControl::Goal::COMMAND_TOGGLE) {
            command_valid = (control->supported_commands &
              xczs_inspection_robot_interfaces::msg::CabinetControl::
              SUPPORT_TOGGLE) != 0U && control->state_ids.size() >= 2U;
            if (!command_valid) {
              command_reason = (control->supported_commands &
                xczs_inspection_robot_interfaces::msg::CabinetControl::
                SUPPORT_TOGGLE) == 0U ?
                "This control does not support toggle; choose an adjacent "
                "detent with set_state or set_position." :
                "Toggle requires at least 2 states.";
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
          result->failure_reason = command_reason;
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
        reset_peak_position(*control);
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
      if (!is_button) {
        pregrasp_stability_references.push_back(
          {control.get(), initial_state.state_id, initial_state.position});
      }
      if (control->control_type ==
        xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_DOOR)
      {
        door_arc_progress.initial_position = initial_state.position;
      }
      for (const auto * ancestor : control_ancestors(*control)) {
        wait_for_fresh_button_state(
          goal_handle, *ancestor, operation_started_at);
        const auto parent_initial_state = button_snapshot(*ancestor);
        expected_parent_states.emplace(
          ancestor->id, parent_initial_state.state_id);
        if (!is_button) {
          pregrasp_stability_references.push_back(
            {ancestor, parent_initial_state.state_id,
              parent_initial_state.position});
        }
      }
      std::tie(target_position, target_state) = resolve_operation_target(
        *control, *goal_handle->get_goal(), initial_state);
      if (is_button) {
        target_position = button_press_depth;
      } else if (std::abs(target_position - initial_state.position) <=
        target_tolerance_)
      {
        throw GenericOperationError(
                OperateCabinetControl::Result::UNSUPPORTED_COMMAND,
                "Control '" + control->id + "' is already at requested "
                "state '" + target_state +
                "'; select a different physical detent.");
      } else if (control->control_type ==
        xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_KNOB)
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
      if (!is_button) {
        wait_for_pregrasp_controls_stable(
          goal_handle, *control, pregrasp_stability_references,
          std::chrono::steady_clock::now());
      }
      latch_cabinet_transform();

      if (is_button) {
        button_poses = calculate_operation_poses(
          *control, button_press_depth);
      } else if (is_drawer) {
        // Bimanual drawer: left/right ready, support, grasp and the right-arm
        // unlock pose are fixed against the (frozen) P0 geometry; the matched
        // left/right waypoint pairs command the same drawer rail position.
        drawer_bimanual_poses = calculate_drawer_bimanual_operation_poses(
          *control, initial_state.position);
        calculate_drawer_bimanual_waypoints(
          *control, initial_state.position, target_position,
          drawer_left_waypoints, drawer_right_waypoints);
      } else if (is_slider && control->grasp_operated) {
        // Front-drawer: grasp-drag flow.  The manipulation waypoints follow
        // the slide axis from the panel's current position to the requested
        // detent -- pull east to open, push west to close -- with the probe
        // rigidly attached to the panel front face.
        drawer_poses = calculate_drawer_operation_poses(
          *control, initial_state.position, rotary_tool_roll_offset);
        drawer_waypoints = calculate_drawer_waypoints(
          *control, initial_state.position, target_position,
          rotary_tool_roll_offset);
      } else if (is_slider) {
        // The press face adapts to the current slide position.  Opening pushes
        // the panel forward to the target detent (target > current).  Closing
        // cannot pull the panel back (the tool only pushes a face), so the
        // robot presses it PAST the open detent, STRICTLY BEYOND the release
        // threshold (release + slider_close_press_offset) -- commanding the
        // press at exactly the release is a float coin-flip that hangs the
        // close (boot18).  Crossing the release mid-press trips the plugin's
        // push-push release, and the detent spring then returns the panel to
        // the closed detent.
        const double slider_press_target =
          target_position < initial_state.position ?
          control->slider_release_position + control->slider_close_press_offset :
          target_position - control->slider_open_press_offset;
        slider_poses = calculate_slider_operation_poses(
          *control, initial_state.position, slider_press_target);
      } else {
        rotary_poses = calculate_rotary_operation_poses(
          *control, initial_state.position, rotary_tool_roll_offset);
        // The manipulation arc is needed both for the runtime branch
        // selection below (validated against the real dock TF before any arm
        // motion) and for the actual arc execution after the grasp attaches.
        rotary_manip_pos = rotary_manipulation_position(
          *control, initial_state.position, target_position);
        rotary_arc_waypoints = calculate_rotation_waypoints(
          *control, initial_state.position, rotary_manip_pos,
          rotary_tool_roll_offset);
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
      {
        // See the press-goal equivalent above: a fresh move group replaces any
        // wedged one, so its execute serialization must be released.
        std::lock_guard<std::mutex> lock(motion_execute_serial_->guard_mutex);
        motion_execute_serial_->wedged_lock.reset();
      }
      configure_move_group(*move_group);
      moveit::core::RobotStatePtr calibrated_robot_state;
      if (is_drawer) {
        // The bimanual drawer also owns the left arm: create and configure its
        // group up front so recovery can always retreat it, then close BOTH
        // tool profiles' calibration joints before the first arm motion.
        drawer_left_move_group = std::make_shared<MoveGroupInterface>(
          shared_from_this(),
          MoveGroupInterface::Options(
            drawer_left_tool_.move_group, "robot_description",
            move_group_namespace_),
          transform_buffer_,
          rclcpp::Duration::from_seconds(system_wait_timeout_));
        // The LEFT arm's EEF is the left tool's contact link, NOT the active
        // right tool (contact_tool_link_).  Setting the right-tool link on the
        // left group made move_group resolve the Cartesian path for
        // 'r_three_cyl_base' against a left-arm group whose tip is 'l_arm_6',
        // failing every IK query (P3-8 db1 Cartesian 0%).
        configure_move_group(
          *drawer_left_move_group, drawer_left_tool_.contact_tool_link);
        const auto bimanual_states = ensure_bimanual_calibration_position(
          *drawer_left_move_group, *move_group, goal_handle);
        verify_bimanual_tool_calibration_state(
          *bimanual_states.first, *bimanual_states.second);
        calibrated_robot_state = bimanual_states.second;
      } else {
        calibrated_robot_state =
          ensure_tool_calibration_position(*move_group, goal_handle);
        verify_tool_tip_calibration_state(*calibrated_robot_state);
      }

      if (validation_only) {
        result->operation_executed = false;
        result->validation_performed = true;
        validate_inoperable_control_path(
          *move_group, goal_handle, *control, initial_state,
          target_position, is_slider ? slider_poses : button_poses,
          rotary_poses, rotary_tool_roll_offset, *calibrated_robot_state, result);
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
        } else if (is_drawer) {
          drawer_bimanual_poses = calculate_drawer_bimanual_operation_poses(
            *control, initial_state.position);
          calculate_drawer_bimanual_waypoints(
            *control, initial_state.position, target_position,
            drawer_left_waypoints, drawer_right_waypoints);
        } else if (is_slider && control->grasp_operated) {
          drawer_poses = calculate_drawer_operation_poses(
            *control, initial_state.position, rotary_tool_roll_offset);
          drawer_waypoints = calculate_drawer_waypoints(
            *control, initial_state.position, target_position,
            rotary_tool_roll_offset);
        } else if (is_slider) {
          const double slider_press_target =
            target_position < initial_state.position ?
            control->slider_release_position + control->slider_close_press_offset :
            target_position - control->slider_open_press_offset;
          slider_poses = calculate_slider_operation_poses(
            *control, initial_state.position, slider_press_target);
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
      if (is_drawer) {
        const auto bimanual_states = ensure_bimanual_calibration_position(
          *drawer_left_move_group, *move_group, goal_handle);
        verify_bimanual_tool_calibration_state(
          *bimanual_states.first, *bimanual_states.second);
      } else {
        const auto predelivery_robot_state =
          ensure_tool_calibration_position(*move_group, goal_handle);
        verify_tool_tip_calibration_state(*predelivery_robot_state);
      }

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
        result->peak_position = measured_state.peak_position;
        result->estimated_force =
          measured_state.peak_position * control->spring_stiffness;
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
        result->physical_outcome_confirmed = true;
        result->final_state_verified = true;
      } else if (is_drawer) {
        // 正面双手抽屉：与单臂 slider 抓拖流程不同，这是真正的双手操作
        // （执行方案 §4.1/§4.2/§5 状态机）。打开时：解锁 → 支撑 → 双爪 →
        // 同步抽出 → 开位确认 → 释放 → 撤回/收起；关闭时：双手重新到达开位
        // 把手 → 支撑 → 双爪 → 同步推回 → 关位确认（恢复锁止）→ 释放 →
        // 撤回/收起。抽拉与推回段都是双臂同一时间基准的同步运动；关闭不
        // 重复打开阶段的解锁推进。
        const bool closing = target_position < initial_state.position;
        result->diagnostic_stage = "ready";
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::MOVING_TO_READY,
          0.25F, target_position,
          "Planning both arms to the drawer ready poses.");
        // P3-8: ready 规划前先选左右臂的 IK 分支（select_drawer_branch_seed），
        // 保证 ready 之后整条 Cartesian 链条（解锁→支撑→预抓→抓取→拽拉）在
        // 该分支内可规划。run7 无种子时 OMPL 对右臂 ready 随机落到 r5=+2.96
        // （腕部近 +π），解锁后 5cm 下探到支撑位时 IK 在 84.4% 处耗尽；分支
        // 选择器扫肩/腕变体并干跑整链，选最小甩动且整链 ≥99% 的分支钉住。
        {
          // 打开时才先做右臂解锁推进（chain 首段为 unlock 的接近→按压两拍）；
          // 关闭时从 ready 直接下探支撑位，链条不含 unlock。
          auto right_chain =
            std::vector<std::vector<geometry_msgs::msg::Pose>>{};
          if (!closing) {
            right_chain.push_back(
              {drawer_bimanual_poses.unlock_pose,
               drawer_bimanual_poses.unlock_press_pose});
          }
          right_chain.push_back(
            {drawer_bimanual_poses.right_support_pose});
          right_chain.push_back(
            {drawer_bimanual_poses.right_pregrasp_pose,
             drawer_bimanual_poses.right_grasp_pose});
          right_chain.push_back(drawer_right_waypoints);
          const auto left_chain =
            std::vector<std::vector<geometry_msgs::msg::Pose>>{
              {drawer_bimanual_poses.left_support_pose},
              {drawer_bimanual_poses.left_pregrasp_pose,
               drawer_bimanual_poses.left_grasp_pose},
              drawer_left_waypoints};
          const auto right_branch = select_drawer_branch_seed(
            *move_group, drawer_right_tool_.move_group,
            drawer_right_tool_.contact_tool_link, goal_handle,
            drawer_bimanual_poses.right_ready_pose, right_chain,
            "drawer ready (right)");
          const auto left_branch = select_drawer_branch_seed(
            *drawer_left_move_group, drawer_left_tool_.move_group,
            drawer_left_tool_.contact_tool_link, goal_handle,
            drawer_bimanual_poses.left_ready_pose, left_chain,
            "drawer ready (left)");
          plan_and_execute_bimanual_poses(
            drawer_left_move_group, move_group, goal_handle,
            drawer_left_tool_.move_group, drawer_right_tool_.move_group,
            drawer_bimanual_poses.left_ready_pose,
            drawer_bimanual_poses.right_ready_pose,
            left_branch, right_branch,
            &result->operation_executed, "drawer ready");
        }
        should_attempt_retreat = true;
        if (!closing) {
          // 右手三电缸把右把手顶部的 b1p 解锁按钮真实压入（接近→按压→
          // 解锁→回退），不再靠近似逻辑区假解锁。
          result->diagnostic_stage = "unlock";
          publish_operate_feedback(
            goal_handle,
            OperateCabinetControl::Feedback::APPROACHING,
            0.35F, target_position,
            "Pressing the drawer unlock button with the right tool.");
          // P3-8: 解锁推进改单臂 Cartesian（run6 取证）。原关节空间
          // plan_and_execute_pose 在 ready 之后的解锁目标上选中了远侧 IK 分支
          // （右臂 r1 -1.768→+2.192，~4 rad 向北甩动），使后续全部 Cartesian
          // 支撑/预抓/抓取/拽拉都执行于该远分支内：右爪向北而非沿抽屉轴向东，
          // 双臂固定关节在抽屉处形成闭链自锁，抽屉停在 0.033 拉不动，双臂
          // 控制器双双触发 state tolerance violation 中止。改 Cartesian 直线
          // 推进后，右臂从 ready 位姿只东移 ~0.016m 进入解锁区，全程锁在
          // ready 位姿的 IK 邻域（近分支），与支撑/接近段同一机制。
          // 2026-09-02: 解锁改为 接近→按压→解锁→回退 四拍，真实把 b1p
          // 按钮压进去（不再靠接近逻辑区假解锁）：
          //   1. 接近位（按钮正面东侧 approach_offset，不接触）；
          //   2. 按压位（向里 press_depth 把 b1p 帽沿棱镜关节顶入 ~3mm，
          //      Gazebo 接触把帽推到限位，物理位移即插件按压证据）；
          //   3. 插件校验 尖端距离≤threshold 且 帽位移≥press_threshold 后解锁；
          //   4. 回退到接近位松手，让弹簧把帽弹回，锁止保持。
          if (!best_effort_cartesian_move(
              *move_group, {drawer_bimanual_poses.unlock_pose},
              cartesian_velocity_scale_ * 0.5,
              cartesian_acceleration_scale_ * 0.5,
              "drawer unlock approach", &result->operation_executed))
          {
            throw OperationError(
              PressCabinetButton::Result::EXECUTION_FAILED,
              "Drawer unlock approach Cartesian move failed.");
          }
          if (!best_effort_cartesian_move(
              *move_group, {drawer_bimanual_poses.unlock_press_pose},
              cartesian_velocity_scale_ * 0.3,
              cartesian_acceleration_scale_ * 0.3,
              "drawer unlock press", &result->operation_executed))
          {
            throw OperationError(
              PressCabinetButton::Result::EXECUTION_FAILED,
              "Drawer unlock press Cartesian move failed.");
          }
          // 让 ODE 接触把 b1p 帽推到限位（return-spring 复位抗力很小）。
          interruptible_hold(goal_handle, grasp_attach_settle_duration_);
          set_drawer_unlock(goal_handle, *control, true, true);
          // 回退松手：弹簧把帽弹回，闭锁保持，右臂随后转支撑位。
          if (!best_effort_cartesian_move(
              *move_group, {drawer_bimanual_poses.unlock_pose},
              cartesian_velocity_scale_ * 0.4,
              cartesian_acceleration_scale_ * 0.4,
              "drawer unlock retreat", &result->operation_executed))
          {
            throw OperationError(
              PressCabinetButton::Result::EXECUTION_FAILED,
              "Drawer unlock retreat Cartesian move failed.");
          }
        }
        result->diagnostic_stage = "support";
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::APPROACHING,
          0.40F, target_position,
          "Placing both tool supports against the drawer face.");
        // P3-8: 支撑位也改用双臂分段 Cartesian。实测支撑位经关节空间规划会
        // 选中远侧 IK 分支——左臂从 ready(l1=+1.342) 一路甩到 l1=-1.872、
        // 绕抽屉下方回绕再回到支撑点，物理摆动把自由滑块顶开 ~4.4cm，触发
        // 插件 pre-grasp 守卫拒绝（run5 取证）。改 Cartesian 后工具沿直线锁在
        // ready 位姿的 IK 邻域，不绕行抽屉，端点在语义上即契约支撑位姿。
        {
          std::size_t support_waypoints = 0U;
          execute_bimanual_segmented_cartesian_path(
            drawer_left_move_group, move_group, goal_handle,
            {drawer_bimanual_poses.left_support_pose},
            {drawer_bimanual_poses.right_support_pose},
            1U, cartesian_velocity_scale_ * 0.5,
            cartesian_acceleration_scale_ * 0.5,
            support_waypoints, &result->operation_executed);
        }
        result->diagnostic_stage = "approach";
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::APPROACHING,
          0.43F, target_position,
          "Approaching both drawer handles for the bimanual grasp.");
        // P3-8: 预抓 → 抓取仍为双臂分段 Cartesian。原预抓的关节空间 RRT 路径
        // 实测会把左探针带进自由抽屉（先西下钻入、以 -0.156 m/s 推挤抽屉，
        // 右臂则向北甩出），触发插件 pre-grasp 守卫拒绝抓取。Cartesian 直线
        // 插值始终保持在抽屉前脸以东（tip x>=0.150 > 脸面 0.104），不接触
        // 自由滑块，路径端点在语义上即契约位姿。
        {
          std::size_t approach_waypoints = 0U;
          execute_bimanual_segmented_cartesian_path(
            drawer_left_move_group, move_group, goal_handle,
            {drawer_bimanual_poses.left_pregrasp_pose,
             drawer_bimanual_poses.left_grasp_pose},
            {drawer_bimanual_poses.right_pregrasp_pose,
             drawer_bimanual_poses.right_grasp_pose},
            1U, cartesian_velocity_scale_ * 0.5,
            cartesian_acceleration_scale_ * 0.5,
            approach_waypoints, &result->operation_executed);
        }
        result->diagnostic_stage = "grasp";
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::GRASPING,
          0.52F, target_position,
          "Attaching both end effectors to the drawer handles.");
        set_drawer_bimanual_grasp(goal_handle, *control, true, true);
        drawer_bimanual_attached = true;
        interruptible_hold(goal_handle, grasp_attach_settle_duration_);
        result->diagnostic_stage = "manipulation";
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::MANIPULATING,
          closing ? 0.72F : 0.62F, target_position,
          closing ?
          "Driving the base to push the drawer closed." :
          "Driving the base to pull the drawer open.");
        // P3-8: 拽拉改为 grab-and-drive —— 双臂锁在抓取位不折叠（右臂前臂
        // 在 p≥0.15 与车体自碰撞，穷举 dock 平移/偏航/工具侧倾均无法到达
        // 0.3 开位），改为底盘沿抽屉轴平移，插件线性耦合让抽屉 1:1 跟随
        // 工具投影开合。attach 时已带 base_free=true 解除底盘制动。
        pull_drawer_by_base_translation(
          goal_handle, *control, target_position,
          &result->operation_executed);
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::VERIFYING,
          0.84F, target_position,
          "Verifying that the drawer reached its requested detent.");
        if (!wait_for_slider_detent(
            goal_handle, *control, target_state, target_position,
            door_settle_timeout_))
        {
          throw GenericOperationError(
                  OperateCabinetControl::Result::CONTACT_DETECTION_TIMEOUT,
                  "The drawer did not reach the requested detent '" +
                  target_state + "' after the base drive.");
        }
        result->diagnostic_stage = "release";
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::RELEASING,
          0.90F, target_position,
          "Releasing the bimanual drawer grasp at the requested detent.");
        set_drawer_bimanual_grasp(goal_handle, *control, false);
        drawer_bimanual_attached = false;
        result->grasp_released = true;
        result->diagnostic_stage = "retreat";
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::RETREATING,
          0.94F, target_position,
          "Retreating both arms before stowing.");
        best_effort_bimanual_retreat_and_stow(
          drawer_left_move_group, move_group, *control,
          &result->operation_executed);
        should_attempt_retreat = false;
        const auto drawer_state = button_snapshot(*control);
        result->peak_position = drawer_state.peak_position;
        result->button_triggered = target_state != initial_state.state_id;
        if (!commit_active_goal_physical_outcome(
            ActiveGoalType::OPERATE, goal_handle->get_goal_id()))
        {
          check_cancel(goal_handle);
        }
        result->physical_outcome_confirmed = true;
        result->final_state_verified = true;
      } else if (is_slider && control->grasp_operated) {
        // 正面抽拉抽屉：与 door/knob 的 grasp 流程同构，但为平移关节 --
        // ready 停在面板前方 prepress，精确规划到 near-grasp pregrasp 后用短
        // 直线逼近前脸，set_control_grasp(true) 把探针刚性地附着到面板，再沿
        // 滑动轴拖拽到目标 detent（拉出/推回），验证、释放、撤回。slider 无
        // 过中点释放机制，拖拽即是全部行程。
        const bool closing = target_position < initial_state.position;
        result->diagnostic_stage = "ready";
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::MOVING_TO_READY,
          0.25F, target_position,
          "Planning the probe to the drawer ready pose.");
        plan_and_execute_pose(
          *move_group, goal_handle, drawer_poses.ready_pose,
          contact_tool_link_, &result->operation_executed, control.get());
        should_attempt_retreat = true;
        result->diagnostic_stage = "approach";
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::APPROACHING,
          0.43F, target_position,
          "Approaching the drawer panel grasp point.");
        // 同 door/knob：先精确规划到 near-grasp pregrasp，避免 ready-pose 的
        // IK 分支无法完成长直线内推（r_arm_0 <-> r_arm_2 自碰撞）。
        plan_and_execute_pose(
          *move_group, goal_handle, drawer_poses.pregrasp_pose,
          contact_tool_link_, &result->operation_executed, control.get());
        wait_for_pregrasp_controls_stable(
          goal_handle, *control, pregrasp_stability_references,
          std::chrono::steady_clock::now());
        execute_cartesian_path(
          *move_group, goal_handle, {drawer_poses.grasp_pose},
          cartesian_velocity_scale_ * 0.5,
          cartesian_acceleration_scale_ * 0.5,
          0.99, &result->operation_executed);
        result->diagnostic_stage = "grasp";
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::GRASPING,
          0.52F, target_position,
          "Attaching the probe at the drawer panel front face.");
        set_control_grasp(goal_handle, control->id, true);
        grasp_attached = true;
        interruptible_hold(goal_handle, grasp_attach_settle_duration_);
        result->diagnostic_stage = "manipulation";
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::MANIPULATING,
          0.62F, target_position,
          closing ?
          "Dragging the drawer panel closed to its detent." :
          "Pulling the drawer panel open to its detent.");
        // 分段直线拖拽（同 door 长弧的分段机制）：每段从实测机器人状态重启，
        // 避免整段 Cartesian 插值在 IK 分支中途失效。
        std::size_t drawer_completed_waypoints = 0U;
        execute_segmented_cartesian_path(
          *move_group, goal_handle, drawer_waypoints,
          static_cast<std::size_t>(door_cartesian_segment_waypoints_),
          cartesian_velocity_scale_ * 0.5,
          cartesian_acceleration_scale_ * 0.5,
          drawer_completed_waypoints, &result->operation_executed);
        if (std::abs(target_position - initial_state.position) >
          target_tolerance_)
        {
          publish_operate_feedback(
            goal_handle,
            OperateCabinetControl::Feedback::MANIPULATING,
            0.71F, target_position,
            "Verifying that the drawer panel reached its requested detent.");
          if (!wait_for_slider_detent(
              goal_handle, *control, target_state, target_position,
              press_detection_timeout_))
          {
            throw GenericOperationError(
                    OperateCabinetControl::Result::CONTACT_DETECTION_TIMEOUT,
                    "The drawer panel did not reach the requested detent '" +
                    target_state + "'.");
          }
        }
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::MANIPULATING,
          0.74F, target_position,
          "Holding the target detent briefly before grasp release.");
        interruptible_hold(goal_handle, grasp_release_settle_duration_);
        result->diagnostic_stage = "release";
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::RELEASING,
          0.78F, target_position,
          "Releasing the drawer grasp at the requested detent.");
        set_control_grasp(goal_handle, control->id, false);
        grasp_attached = false;
        result->grasp_released = true;
        result->diagnostic_stage = "retreat";
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::RETREATING,
          0.84F, target_position,
          "Retreating before verifying the released drawer.");
        const auto drawer_retreat_pose = calculate_drawer_tool_pose(
          *control, target_position, prepress_distance_);
        execute_cartesian_path(
          *move_group, goal_handle, {drawer_retreat_pose},
          cartesian_velocity_scale_, cartesian_acceleration_scale_,
          0.99, &result->operation_executed);
        should_attempt_retreat = false;
        result->diagnostic_stage = "verification";
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::VERIFYING,
          0.90F, target_position,
          "Verifying that the drawer panel settled at its detent.");
        if (!wait_for_slider_detent(
            goal_handle, *control, target_state, target_position,
            door_settle_timeout_))
        {
          throw GenericOperationError(
                  OperateCabinetControl::Result::RELEASE_FAILED,
                  "The drawer panel did not latch at the requested detent.");
        }
        const auto drawer_state = button_snapshot(*control);
        result->peak_position = drawer_state.peak_position;
        result->button_triggered = target_state != initial_state.state_id;
        if (!commit_active_goal_physical_outcome(
            ActiveGoalType::OPERATE, goal_handle->get_goal_id()))
        {
          check_cancel(goal_handle);
        }
        result->physical_outcome_confirmed = true;
        result->final_state_verified = true;
      } else if (is_slider) {
        const bool closing = target_position < initial_state.position;
        result->diagnostic_stage = "ready";
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::MOVING_TO_READY,
          0.25F, target_position,
          "Planning the probe to the slider ready pose.");
        execute_slider_ready_approach_press(
          *move_group, goal_handle, result, slider_poses, closing,
          target_position, *control, &result->operation_executed);
        should_attempt_retreat = true;
        // The ready-branch retry inside the helper covers PLANNING failures
        // (OMPL can land on an IK branch from which the 0->target push cannot
        // be planned; the button type in B3a got a good branch by luck).
        // Force tracking then covers shortfalls that survive a good branch
        // (detent friction): it measures the PHYSICAL panel position and
        // re-pushes from the current face until it converges.  Only the open
        // push needs it -- a close is a push-past-detent release where the
        // panel springs away from the tool as soon as the release trips, and
        // re-pushing there would re-open the panel.
        if (!closing) {
          result->diagnostic_stage = "manipulation";
          // The open force-tracking loop must converge on the SAME
          // compensated target as the initial press: without the offset it
          // re-pushes toward the raw 0.3 detent, whose re-push peak (commanded
          // ~0.301 plus ~0.03 tool over-travel = ~0.332) can cross the
          // push-push release and spring the panel shut (boot16 open).  Both
          // the initial press and the force-tracking convergence share
          // slider_open_press_offset so the open peak stays below the release
          // with margin.
          track_slider_force(
            *move_group, goal_handle, *control,
            target_position - control->slider_open_press_offset,
            &result->operation_executed);
        }
        // Let the detent spring settle the panel before verifying; the tip is
        // still pressed against the face, so the panel is at the target.
        interruptible_hold(goal_handle, press_hold_seconds_);
        if (!closing) {
          result->diagnostic_stage = "verification";
          publish_operate_feedback(
            goal_handle,
            OperateCabinetControl::Feedback::VERIFYING,
            0.77F, target_position,
            "Verifying the slider panel reached its target detent.");
          if (!wait_for_slider_detent(
              goal_handle, *control, target_state, target_position,
              press_detection_timeout_))
          {
            throw GenericOperationError(
                    OperateCabinetControl::Result::CONTACT_DETECTION_TIMEOUT,
                    "The slider panel did not reach the requested detent '" +
                    target_state + "'.");
          }
          interruptible_hold(goal_handle, press_hold_seconds_);
        }
        result->diagnostic_stage = "retreat";
        // For a close the tip must clear the closing panel, so the retreat
        // waypoints anchor on the CLOSED face rather than the release face;
        // the panel follows the retreating tip and settles at the closed
        // detent once the tip is out of its travel.
        const OperationPoses retreat_poses = closing ?
          calculate_slider_operation_poses(
          *control, target_position, target_position) :
          slider_poses;
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::RETREATING,
          0.85F, 0.0,
          closing ?
          "Retracting clear of the closing panel; the detent spring returns "
          "it to the closed detent." :
          "Retracting the probe from the slider panel.");
        if (closing) {
          // A close is a push-past-detent release: once the release trips, the
          // 80 kg panel must spring back to the closed detent on its own, which
          // requires the tip to clear the panel's entire travel.  The earlier
          // Cartesian retreat over that ~0.36 m westward sweep failed planning
          // at ~85% on every close -- the tool cannot hold its press orientation
          // over the long sweep, and the failure aborts BEFORE executing, so the
          // tip stayed pressed against the panel and held it at the release for
          // the whole 90 s settle window (boot20 close).  A joint-space OMPL
          // retreat to the closed-face prepress is far more robust: the planning
          // scene keeps the panel as free space ("按净空规划", only db1's own
          // face cylinder, exempted during operation, lives near the path), so
          // only the single end pose must be reachable, not every intermediate
          // orientation.  The retreat is a get-clear motion; its completion is
          // only a means to the real check, the detent latch below, so a blocked
          // retreat must not fail the operation: log it and verify the panel
          // instead.  If the tool is genuinely still inside the panel's travel,
          // that verification times out and the operation fails accurately.
          try {
            plan_and_execute_pose(
              *move_group, goal_handle, retreat_poses.prepress_pose,
              contact_tool_link_, &result->operation_executed, control.get());
          } catch (const OperationError & retreat_error) {
            RCLCPP_WARN(
              get_logger(),
              "Slider close retreat could not complete (%s); the panel must "
              "still latch closed on its own -- proceeding to detent "
              "verification.",
              retreat_error.what());
          }
        } else {
          execute_cartesian_path(
            *move_group, goal_handle,
            {retreat_poses.contact_pose, retreat_poses.prepress_pose},
            cartesian_velocity_scale_, cartesian_acceleration_scale_,
            0.99, &result->operation_executed);
        }
        should_attempt_retreat = false;
        // The bistable detent must hold the panel after the tool leaves; a
        // spring-back to the source detent means the latch never engaged.  A
        // close's panel is pushed past the release (to ~0.36, riding up to the
        // joint limit) before it springs back, and the 80 kg panel coasts back
        // slowly (spring force ~4 N vs 0.2 N friction, terminal ~0.14 m/s) --
        // it can take many seconds to cross the midpoint and flip state_id to
        // closed.  The 3 s press/release detection timeouts are button-scale;
        // use the door-scale settle timeout so the verification does not race
        // the spring-back (boot17 close: panel reached 0.008 but the 3 s
        // window expired mid-return).
        result->diagnostic_stage = "verification";
        if (!wait_for_slider_detent(
            goal_handle, *control, target_state, target_position,
            door_settle_timeout_))
        {
          throw GenericOperationError(
                  OperateCabinetControl::Result::RELEASE_FAILED,
                  closing ?
                  "The slider panel did not return to the closed detent." :
                  "The slider panel did not latch at the requested detent.");
        }
        const auto measured_state = button_snapshot(*control);
        result->peak_position = measured_state.peak_position;
        result->button_triggered =
          target_state != initial_state.state_id;
        if (!commit_active_goal_physical_outcome(
            ActiveGoalType::OPERATE, goal_handle->get_goal_id()))
        {
          check_cancel(goal_handle);
        }
        result->physical_outcome_confirmed = true;
        result->final_state_verified = true;
      } else {
        result->diagnostic_stage = "ready";
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::MOVING_TO_READY,
          0.25F, target_position,
          "Planning the probe to the control ready pose.");
        // Runtime branch selection: a knob's ready/pregrasp plans must land on
        // an IK family whose whole Cartesian chain (50 mm approach + the
        // manipulation arc) is feasible from the REAL dock TF.  The configured
        // seed alone is not enough -- it can fail the arc via r_arm_0<->r_arm_2
        // self-collision or a wrist joint limit.  Doors keep their segmented
        // arc mechanism and the configured seed unchanged.
        std::vector<double> rotary_branch_seed;
        const std::vector<double> * rotary_branch_seed_ptr = nullptr;
        if (control->control_type ==
            xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_KNOB)
        {
          // The post-release retreat target (knob angle at the manipulation
          // position, prepress standoff) must match the retreat the operator
          // actually executes after releasing the grasp.
          const auto retreat_pose = calculate_rotary_tool_pose(
            *control, rotary_manip_pos, prepress_distance_, false,
            rotary_tool_roll_offset);
          rotary_branch_seed = select_rotary_branch_seed(
            *move_group, *control, rotary_poses.pregrasp_pose,
            rotary_poses.grasp_pose, rotary_arc_waypoints, retreat_pose,
            contact_tool_link_);
          if (!rotary_branch_seed.empty()) {
            rotary_branch_seed_ptr = &rotary_branch_seed;
          }
        }
        plan_and_execute_pose(
          *move_group, goal_handle, rotary_poses.ready_pose,
          contact_tool_link_, &result->operation_executed, control.get(),
          rotary_branch_seed_ptr);
        should_attempt_retreat = true;
        result->diagnostic_stage = "approach";
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::APPROACHING,
          0.43F, target_position,
          "Approaching the physical grasp point.");
        if (control->control_type ==
            xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_DOOR ||
          control->control_type ==
            xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_KNOB)
        {
          // A pose-only OMPL plan can select a ready-pose IK branch that
          // cannot finish the final inward Cartesian approach.  Require an
          // exact collision-checked plan to a near-grasp pose first, then
          // preserve a short straight-line final approach.  Knobs share the
          // failure: the left-detent ready pose lands on an r_arm branch
          // whose 94 mm straight approach self-collides r_arm_0 <-> r_arm_2;
          // a 50 mm final approach is clean on every branch.  The knob
          // pregrasp itself must sit far enough out that the pose-planned
          // motion to it cannot sweep the clamped blade (see the
          // knob_pregrasp_clearance comment); doors keep their 10 mm variant.
          // The pregrasp plan must use the same calibrated IK seed as the
          // ready plan: an unseeded OMPL plan can pick a different IK branch
          // (observed: r_arm_2 = +0.89) whose 50 mm approach self-collides at
          // ~25 % before the blade clamp, while every seed-derived branch is
          // clean over the full distance.  For knobs the seed is the runtime
          // branch picked by select_rotary_branch_seed (same as the ready
          // plan); doors keep their configured seed.
          plan_and_execute_pose(
            *move_group, goal_handle, rotary_poses.pregrasp_pose,
            contact_tool_link_, &result->operation_executed, control.get(),
            rotary_branch_seed_ptr);
        }
        wait_for_pregrasp_controls_stable(
          goal_handle, *control, pregrasp_stability_references,
          std::chrono::steady_clock::now());
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
        // unreachable full-angle Cartesian wrist arc.  These were computed
        // before the ready pose plan so the branch selection could validate
        // the same arc against the real dock TF.
        const double & manipulation_position = rotary_manip_pos;
        const auto & waypoints = rotary_arc_waypoints;
        result->diagnostic_stage = "manipulation";
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::MANIPULATING,
          0.62F, target_position,
          "Following the control joint arc without commanding the joint.");
        if (control->control_type ==
          xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_DOOR)
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
          xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_DOOR &&
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
          xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_KNOB &&
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
          xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_DOOR &&
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
          xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_DOOR;
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
          result->grasp_released = true;
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
          result->grasp_released = true;
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
        result->physical_outcome_confirmed = true;
        result->final_state_verified = true;
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
      result->transport_succeeded = true;

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
      result->peak_position = final_state.peak_position;
      result->final_state = final_state.state_id;
      if (is_button) {
        result->estimated_force =
          final_state.peak_position * control->spring_stiffness;
        result->button_triggered =
          result->button_triggered ||
          final_state.peak_position + 1.0e-9 >= control->press_threshold;
      }
      result->physical_outcome_confirmed = true;
      result->final_state_verified = true;
      result->transport_succeeded = true;
      result->recovery_succeeded = true;
      result->grasp_released = true;
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
          is_operate_goal_canceling(goal_handle),
          drawer_bimanual_attached, drawer_left_move_group);
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
          is_operate_goal_canceling(goal_handle),
          drawer_bimanual_attached, drawer_left_move_group);
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
          is_operate_goal_canceling(goal_handle),
          drawer_bimanual_attached, drawer_left_move_group);
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
          is_operate_goal_canceling(goal_handle),
          drawer_bimanual_attached, drawer_left_move_group);
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
      xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_BUTTON)
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
    const auto start = std::chrono::steady_clock::now();
    const auto deadline = start +
      std::chrono::duration<double>(system_wait_timeout_);
    for (const auto * ancestor : ancestors) {
      auto state = button_snapshot(*ancestor);
      // 父控制（如按钮的旋钮）在按压/重解析几何的瞬间可能正处于物理回落中
      // （工具拖带或重力回中）。失败即抛会把一次瞬态判成 NOT_READY；这里改为
      // 轮询等待其稳定（≤ system_wait_timeout_），稳定后按最终落位重建几何。
      // 旋钮是自由转动的展示件，不要求停在特定档位，仅要求"不再运动"。
      auto fresh = [&]() {
        return structured_control_state_is_usable(
          state.structured_received, state.valid, true,
          std::chrono::duration<double>(
            std::chrono::steady_clock::now() -
            state.structured_received_at).count() <=
          button_state_timeout_);
      };
      if (require_stable) {
        while (!(fresh() && !state.in_motion &&
          std::abs(state.velocity) <= stable_velocity_tolerance_))
        {
          if (std::chrono::steady_clock::now() >= deadline) {
            throw GenericOperationError(
                    OperateCabinetControl::Result::NOT_READY,
                    "Parent control '" + ancestor->id +
                    "' did not become stable before the operation "
                    "geometry was needed.");
          }
          std::this_thread::sleep_for(50ms);
          state = button_snapshot(*ancestor);
        }
      } else if (!fresh()) {
        throw GenericOperationError(
                OperateCabinetControl::Result::NOT_READY,
                "Parent control '" + ancestor->id +
                "' does not have a fresh valid physical state.");
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

    // Bimanual drawers slide along drawer_axis: the current position moves the
    // panel face (and therefore the grasp targets) along that axis.  From the
    // standard dock an OPEN drawer puts the wrist inside the robot body, so the
    // ready planning self-collides (reach to the open wrist collapses while the
    // body stays put).  Re-measure the standoff from the CURRENT face instead
    // by shifting the dock along the slide axis by the live drawer position;
    // this preserves the closed-drawer reach and keeps the wrist clear of the
    // body exactly as in the closed case.  Zero for a closed drawer, so the
    // open flow (and every non-drawer control) keeps the configured dock.
    const double drawer_dock_shift =
      is_drawer_type(control.control_type) ?
      button_snapshot(control).position : 0.0;
    const tf2::Vector3 local_position = station.local_anchor +
      station.outward_axis * station.standoff +
      control.drawer_axis * drawer_dock_shift;
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
    // Knobs need a far pregrasp so the pose-planned motion to it stays clear
    // of the blade; doors keep the small shared clearance they were tuned with.
    const double pregrasp_clearance =
      control.control_type ==
        xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_KNOB ?
      knob_pregrasp_clearance_ : rotary_pregrasp_clearance_;
    poses.pregrasp_pose = calculate_rotary_tool_pose(
      control, position,
      control.grasp_outward_offset + pregrasp_clearance, true,
      tool_roll_offset);
    poses.grasp_pose = calculate_rotary_tool_pose(
      control, position, control.grasp_outward_offset, true,
      tool_roll_offset);
    return poses;
  }

  /**
   * Place the measured business point on a sliding drawer front face.
   *
   * A prismatic drawer differs from a rotary control: the grasp point does not
   * swing about a pivot, it translates linearly with the panel.  The tool
   * orientation stays fixed (outward along the face normal) and only the tip
   * rides the slide axis -- local_grasp = grasp_zero + axis * position.  The
   * planner-facing geometry (grasp_zero, axis, approach_normal) comes from the
   * same resolved control geometry as the rotary path, so the front station
   * and the tool calibration are unchanged.
   */
  geometry_msgs::msg::Pose calculate_drawer_tool_pose(
    const ButtonSpec & control,
    double position,
    double outward_offset,
    double tool_roll_offset = 0.0)
  {
    const tf2::Transform cabinet = resolve_cabinet_transform();
    const auto geometry = resolve_control_geometry(control, true);
    const tf2::Vector3 local_grasp =
      geometry.grasp_zero + geometry.axis * position;
    const tf2::Vector3 grasp = cabinet * local_grasp;
    const tf2::Vector3 outward = tf2::quatRotate(
      cabinet.getRotation(), geometry.approach_normal).normalized();
    const tf2::Vector3 tool_axis_reference =
      tool_axis_orientation_ ==
      ToolProfile::ToolAxisOrientation::TOWARD_CONTROL ?
      -outward : outward;
    const tf2::Quaternion tool_zero =
      tool_rotation_from_outward(tool_axis_reference);
    tf2::Quaternion tool_roll;
    tool_roll.setRotation(tf2::Vector3(0.0, 0.0, 1.0), tool_roll_offset);
    tool_roll.normalize();
    tf2::Quaternion tool_rotation = tool_zero * tool_roll;
    tool_rotation.normalize();

    geometry_msgs::msg::Pose pose;
    // Place the measured 3-D business point, rather than the contact-link
    // origin, on the desired grasp point (same convention as the rotary tool).
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

  RotaryOperationPoses calculate_drawer_operation_poses(
    const ButtonSpec & control,
    double position,
    double tool_roll_offset = 0.0)
  {
    // Same ready/pregrasp/grasp shape as the rotary flow: the pose-planned
    // pregrasp keeps the final inward approach short so any IK branch can
    // complete it (see the ready-branch comment in the grasp flow).
    RotaryOperationPoses poses;
    poses.ready_pose = calculate_drawer_tool_pose(
      control, position, prepress_distance_, tool_roll_offset);
    poses.pregrasp_pose = calculate_drawer_tool_pose(
      control, position,
      control.grasp_outward_offset + rotary_pregrasp_clearance_,
      tool_roll_offset);
    poses.grasp_pose = calculate_drawer_tool_pose(
      control, position, control.grasp_outward_offset, tool_roll_offset);
    return poses;
  }

  std::vector<geometry_msgs::msg::Pose> calculate_drawer_waypoints(
    const ButtonSpec & control,
    double initial_position,
    double target_position,
    double tool_roll_offset = 0.0)
  {
    // A linear drag from the panel's current slide position to the target
    // detent, sampled at drawer_waypoint_step_ so each segment is short enough
    // to re-plan from the measured robot state (same rationale as the door
    // segmented arc).  The tool is rigidly attached to the panel, so the panel
    // follows the tool tip exactly along the slide axis.
    const double travel = target_position - initial_position;
    const std::size_t count = std::max<std::size_t>(
      1U, static_cast<std::size_t>(
        std::ceil(std::abs(travel) / drawer_waypoint_step_)));
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.reserve(count);
    for (std::size_t index = 1U; index <= count; ++index) {
      const double ratio = static_cast<double>(index) /
        static_cast<double>(count);
      waypoints.push_back(calculate_drawer_tool_pose(
        control, initial_position + travel * ratio,
        control.grasp_outward_offset, tool_roll_offset));
    }
    return waypoints;
  }

  const BimanualToolProfile & drawer_tool_profile(DrawerSide side) const
  {
    return side == DrawerSide::LEFT ? drawer_left_tool_ : drawer_right_tool_;
  }

  tf2::Vector3 drawer_side_point(
    const ButtonSpec & control, DrawerSide side) const
  {
    switch (side) {
      case DrawerSide::LEFT:
        if (!control.has_left_handle_point) {
          throw OperationError(
                  PressCabinetButton::Result::NOT_READY,
                  "Drawer '" + control.id +
                  "' is missing its left handle point contract.");
        }
        return control.left_handle_point;
      case DrawerSide::RIGHT:
        if (!control.has_right_handle_point) {
          throw OperationError(
                  PressCabinetButton::Result::NOT_READY,
                  "Drawer '" + control.id +
                  "' is missing its right handle point contract.");
        }
        return control.right_handle_point;
    }
    return tf2::Vector3(0.0, 0.0, 0.0);
  }

  std::string drawer_side_name(DrawerSide side) const
  {
    return side == DrawerSide::LEFT ? "left" : "right";
  }

  /**
   * Place one side's measured tool business point on the drawer.
   *
   * A bimanual drawer differs from the legacy single-arm drawer flow in that
   * the left and right tools have different business-point offsets and
   * approach the two handles from their own arm chains.  The drawer slides
   * along drawer_axis (P0: +X, toward the robot); at rail position p the
   * side's handle point sits at side_point + axis*p in cabinet frame.  The
   * tool orientation convention matches calculate_drawer_tool_pose (tool axis
   * along the outward normal), and the pose compensates the side's own
   * business-point offset so the measured tip lands on the target.
   */
  geometry_msgs::msg::Pose calculate_drawer_side_tool_pose(
    const ButtonSpec & control,
    DrawerSide side,
    double position,
    double outward_offset,
    double tool_roll_offset = 0.0)
  {
    const tf2::Transform cabinet = resolve_cabinet_transform();
    const tf2::Vector3 axis = tf2::quatRotate(
      cabinet.getRotation(), control.drawer_axis).normalized();
    const tf2::Vector3 outward = tf2::quatRotate(
      cabinet.getRotation(), control.approach_normal).normalized();
    const tf2::Vector3 side_point = drawer_side_point(control, side);
    const tf2::Vector3 local_target = side_point + axis * position;
    const tf2::Vector3 target = cabinet * local_target;
    const tf2::Vector3 tool_axis_reference =
      tool_axis_orientation_ ==
      ToolProfile::ToolAxisOrientation::TOWARD_CONTROL ?
      -outward : outward;
    const tf2::Quaternion tool_zero =
      tool_rotation_from_outward(tool_axis_reference);
    tf2::Quaternion tool_roll;
    tool_roll.setRotation(tf2::Vector3(0.0, 0.0, 1.0), tool_roll_offset);
    tool_roll.normalize();
    tf2::Quaternion tool_rotation = tool_zero * tool_roll;
    tool_rotation.normalize();

    geometry_msgs::msg::Pose pose;
    const auto & profile = drawer_tool_profile(side);
    const tf2::Vector3 desired_tip_position =
      target + outward * outward_offset;
    const tf2::Vector3 position_world = desired_tip_position -
      tf2::quatRotate(tool_rotation, profile.tool_tip_position);
    pose.position.x = position_world.x();
    pose.position.y = position_world.y();
    pose.position.z = position_world.z();
    pose.orientation = to_message(tool_rotation);
    return pose;
  }

  // The support pose keeps the side tool hovering drawer_grasp_outward_offset
  // east of the cabinet face below its handle, ready for the bimanual grasp.
  // P3-8: it deliberately does NOT press the face - the face panel is a
  // free-sliding link, so a press pushes the drawer toward closed and trips
  // the plugin's pre-grasp disturbance guard.  The support point rides the
  // drawer rail like the handle point (support + axis * position), so the
  // support tracks the drawer whether it is being opened from the closed
  // position or closed from the open position.
  geometry_msgs::msg::Pose calculate_drawer_support_pose(
    const ButtonSpec & control, DrawerSide side, double position)
  {
    const bool is_left = side == DrawerSide::LEFT;
    const tf2::Vector3 support = is_left ?
      control.left_support_point : control.right_support_point;
    if (!is_left && !control.has_right_support_point) {
      throw OperationError(
              PressCabinetButton::Result::NOT_READY,
              "Drawer '" + control.id +
              "' is missing its right support point contract.");
    }
    const tf2::Transform cabinet = resolve_cabinet_transform();
    const tf2::Vector3 axis = tf2::quatRotate(
      cabinet.getRotation(), control.drawer_axis).normalized();
    const tf2::Vector3 outward = tf2::quatRotate(
      cabinet.getRotation(), control.approach_normal).normalized();
    const tf2::Vector3 tool_axis_reference =
      tool_axis_orientation_ ==
      ToolProfile::ToolAxisOrientation::TOWARD_CONTROL ?
      -outward : outward;
    const tf2::Quaternion tool_rotation =
      tool_rotation_from_outward(tool_axis_reference);
    const auto & profile = drawer_tool_profile(side);
    const tf2::Vector3 target = cabinet * (support + axis * position);
    const tf2::Vector3 desired_tip_position =
      target + outward * control.drawer_grasp_outward_offset;
    const tf2::Vector3 position_world = desired_tip_position -
      tf2::quatRotate(tool_rotation, profile.tool_tip_position);
    geometry_msgs::msg::Pose pose;
    pose.position.x = position_world.x();
    pose.position.y = position_world.y();
    pose.position.z = position_world.z();
    pose.orientation = to_message(tool_rotation);
    return pose;
  }

  // 2026-09-02: the unlock now physically presses the b1p button.  The
  // approach pose hovers the right tool tip drawer_unlock_approach_offset
  // OUTWARD (east) of the button face (unlock_press_point, on top of the right
  // handle riser); the follow-on press pose pushes the tip INWARD by
  // drawer_unlock_press_depth to displace the cap.  The plugin authorizes the
  // unlock only while BOTH the tip is within unlock_distance_threshold of the
  // press point AND the button joint is physically displaced.
  geometry_msgs::msg::Pose calculate_drawer_unlock_pose(
    const ButtonSpec & control)
  {
    if (!control.has_unlock_press_point) {
      throw OperationError(
              PressCabinetButton::Result::NOT_READY,
              "Drawer '" + control.id +
              "' is missing its unlock press-point contract.");
    }
    const tf2::Transform cabinet = resolve_cabinet_transform();
    const tf2::Vector3 outward = tf2::quatRotate(
      cabinet.getRotation(), control.approach_normal).normalized();
    const tf2::Vector3 tool_axis_reference =
      tool_axis_orientation_ ==
      ToolProfile::ToolAxisOrientation::TOWARD_CONTROL ?
      -outward : outward;
    const tf2::Quaternion tool_rotation =
      tool_rotation_from_outward(tool_axis_reference);
    const auto & profile = drawer_tool_profile(DrawerSide::RIGHT);
    const tf2::Vector3 target = cabinet * control.unlock_press_point +
      outward * control.drawer_unlock_approach_offset;
    const tf2::Vector3 position_world = target -
      tf2::quatRotate(tool_rotation, profile.tool_tip_position);
    geometry_msgs::msg::Pose pose;
    pose.position.x = position_world.x();
    pose.position.y = position_world.y();
    pose.position.z = position_world.z();
    pose.orientation = to_message(tool_rotation);
    return pose;
  }

  // The press pose drives the right tool tip INWARD (west, into the button
  // cap) by drawer_unlock_press_depth past the unlock press point, physically
  // displacing the b1p cap on its prismatic joint (3 mm travel, verified
  // collision-free — no drawer body material sits behind the cap).  The plugin
  // reads the cap joint displacement as the unlock evidence.
  geometry_msgs::msg::Pose calculate_drawer_unlock_press_pose(
    const ButtonSpec & control)
  {
    const tf2::Transform cabinet = resolve_cabinet_transform();
    const tf2::Vector3 outward = tf2::quatRotate(
      cabinet.getRotation(), control.approach_normal).normalized();
    const tf2::Vector3 tool_axis_reference =
      tool_axis_orientation_ ==
      ToolProfile::ToolAxisOrientation::TOWARD_CONTROL ?
      -outward : outward;
    const tf2::Quaternion tool_rotation =
      tool_rotation_from_outward(tool_axis_reference);
    const auto & profile = drawer_tool_profile(DrawerSide::RIGHT);
    const tf2::Vector3 target = cabinet * control.unlock_press_point -
      outward * control.drawer_unlock_press_depth;
    const tf2::Vector3 position_world = target -
      tf2::quatRotate(tool_rotation, profile.tool_tip_position);
    geometry_msgs::msg::Pose pose;
    pose.position.x = position_world.x();
    pose.position.y = position_world.y();
    pose.position.z = position_world.z();
    pose.orientation = to_message(tool_rotation);
    return pose;
  }

  struct DrawerBimanualPoses
  {
    geometry_msgs::msg::Pose left_ready_pose;
    geometry_msgs::msg::Pose right_ready_pose;
    geometry_msgs::msg::Pose left_pregrasp_pose;
    geometry_msgs::msg::Pose right_pregrasp_pose;
    geometry_msgs::msg::Pose left_grasp_pose;
    geometry_msgs::msg::Pose right_grasp_pose;
    geometry_msgs::msg::Pose left_support_pose;
    geometry_msgs::msg::Pose right_support_pose;
    geometry_msgs::msg::Pose unlock_pose;
    geometry_msgs::msg::Pose unlock_press_pose;
  };

  // The grasp pose and the pull hold each tool tip pressed INTO its handle
  // plate by drawer_grasp_press_depth (west of the riser face), so the plugin's
  // tight attach gate and coupling keepalive see genuine contact, not a hover.
  double drawer_grasp_contact_offset(
    const ButtonSpec & control) const
  {
    return -control.drawer_grasp_press_depth;
  }

  DrawerBimanualPoses calculate_drawer_bimanual_operation_poses(
    const ButtonSpec & control,
    double initial_position)
  {
    DrawerBimanualPoses poses;
    poses.unlock_pose = calculate_drawer_unlock_pose(control);
    poses.unlock_press_pose = calculate_drawer_unlock_press_pose(control);
    poses.left_support_pose = calculate_drawer_support_pose(
      control, DrawerSide::LEFT, initial_position);
    poses.right_support_pose = calculate_drawer_support_pose(
      control, DrawerSide::RIGHT, initial_position);
    // ready: both arms hover at a safe distance on the outward side of the
    // handles; pregrasp: short final Cartesian approach; grasp: measured tips
    // pressed INTO the handle plates (fingers bite the riser face).
    poses.left_ready_pose = calculate_drawer_side_tool_pose(
      control, DrawerSide::LEFT, initial_position, prepress_distance_);
    poses.right_ready_pose = calculate_drawer_side_tool_pose(
      control, DrawerSide::RIGHT, initial_position, prepress_distance_);
    poses.left_pregrasp_pose = calculate_drawer_side_tool_pose(
      control, DrawerSide::LEFT, initial_position,
      control.drawer_grasp_outward_offset + rotary_pregrasp_clearance_);
    poses.right_pregrasp_pose = calculate_drawer_side_tool_pose(
      control, DrawerSide::RIGHT, initial_position,
      control.drawer_grasp_outward_offset + rotary_pregrasp_clearance_);
    poses.left_grasp_pose = calculate_drawer_side_tool_pose(
      control, DrawerSide::LEFT, initial_position,
      drawer_grasp_contact_offset(control));
    poses.right_grasp_pose = calculate_drawer_side_tool_pose(
      control, DrawerSide::RIGHT, initial_position,
      drawer_grasp_contact_offset(control));
    return poses;
  }

  // Matched waypoints for both arms: the i-th entry of each vector commands
  // the SAME drawer rail position, so the two arms traverse the rail in lock
  // step and the drawer never sees one-sided travel.
  void calculate_drawer_bimanual_waypoints(
    const ButtonSpec & control,
    double initial_position,
    double target_position,
    std::vector<geometry_msgs::msg::Pose> & left_waypoints,
    std::vector<geometry_msgs::msg::Pose> & right_waypoints)
  {
    const double travel = target_position - initial_position;
    const std::size_t count = std::max<std::size_t>(
      1U, static_cast<std::size_t>(
        std::ceil(std::abs(travel) / drawer_waypoint_step_)));
    left_waypoints.clear();
    right_waypoints.clear();
    left_waypoints.reserve(count);
    right_waypoints.reserve(count);
    // The pull waypoints hold the tips pressed INTO the handle plates (same
    // contact offset as the grasp pose), matching what the base-translation
    // drive actually holds — the coupling keepalive requires this contact.
    const double contact_offset = drawer_grasp_contact_offset(control);
    for (std::size_t index = 1U; index <= count; ++index) {
      const double ratio = static_cast<double>(index) /
        static_cast<double>(count);
      const double position = initial_position + travel * ratio;
      left_waypoints.push_back(calculate_drawer_side_tool_pose(
        control, DrawerSide::LEFT, position, contact_offset));
      right_waypoints.push_back(calculate_drawer_side_tool_pose(
        control, DrawerSide::RIGHT, position, contact_offset));
    }
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
      xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_DOOR)
    {
      release_fraction = door_release_fraction_;
    } else if (control.control_type ==
      xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_KNOB)
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
      xczs_inspection_robot_interfaces::srv::SetCabinetGrasp::Request>();
    request->control_id = control_id;
    {
      std::lock_guard<std::mutex> lock(operation_lease_mutex_);
      if (attach &&
        (!operation_lease_held_.load() || operation_lease_lost_.load() ||
        operation_lease_id_.empty()))
      {
        throw GenericOperationError(
                OperateCabinetControl::Result::GRASP_FAILED,
                "Cabinet grasp attach requires an active global operation "
                "lease.");
      }
      request->operation_lease_id = operation_lease_id_;
    }
    request->robot_model = robot_model_name_;
    request->robot_link = grasp_link_;
    request->robot_grasp_point.x = grasp_point_position_.x();
    request->robot_grasp_point.y = grasp_point_position_.y();
    request->robot_grasp_point.z = grasp_point_position_.z();
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
          xczs_inspection_robot_interfaces::srv::SetCabinetGrasp::Request>();
        request->control_id = control_id;
        {
          std::lock_guard<std::mutex> lock(operation_lease_mutex_);
          request->operation_lease_id = operation_lease_id_;
        }
        request->robot_model = robot_model_name_;
        request->robot_link = grasp_link_;
        request->robot_grasp_point.x = grasp_point_position_.x();
        request->robot_grasp_point.y = grasp_point_position_.y();
        request->robot_grasp_point.z = grasp_point_position_.z();
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

  void verify_bimanual_tool_calibration_state(
    const moveit::core::RobotState & left_state,
    const moveit::core::RobotState & right_state) const
  {
    const auto verify_one = [this](const BimanualToolProfile & profile,
        const moveit::core::RobotState & robot_state,
        const std::string & side) {
        for (std::size_t index = 0U;
          index < profile.calibration_joint_names.size(); ++index)
        {
          double measured;
          try {
            measured = robot_state.getVariablePosition(
              profile.calibration_joint_names[index]);
          } catch (const std::exception & error) {
            throw OperationError(
                    PressCabinetButton::Result::NOT_READY,
                    "Drawer " + side +
                    " tool business-point calibration references unknown "
                    "joint '" + profile.calibration_joint_names[index] +
                    "': " + error.what());
          }
          if (!std::isfinite(measured) ||
            std::abs(measured - profile.calibration_joint_positions[index]) >
            tool_tip_calibration_joint_tolerance_)
          {
            throw OperationError(
                    PressCabinetButton::Result::NOT_READY,
                    "Drawer " + side + " tool joint '" +
                    profile.calibration_joint_names[index] + "' is at " +
                    std::to_string(measured) +
                    " rad/m, outside its calibrated business-point position " +
                    std::to_string(profile.calibration_joint_positions[index]) +
                    " +/- " +
                    std::to_string(tool_tip_calibration_joint_tolerance_) +
                    "; bimanual motion was blocked. Reset the tool first.");
          }
        }
      };
    verify_one(drawer_left_tool_, left_state, "left");
    verify_one(drawer_right_tool_, right_state, "right");
  }

  template<typename GoalHandleT>
  void set_drawer_unlock(
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const ButtonSpec & control,
    bool unlock,
    bool require_unlocked)
  {
    const auto service_deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(system_wait_timeout_);
    while (!unlock_client_->wait_for_service(50ms)) {
      check_cancel(goal_handle);
      if (std::chrono::steady_clock::now() >= service_deadline) {
        throw GenericOperationError(
                OperateCabinetControl::Result::NOT_READY,
                "Cabinet drawer unlock service is unavailable.");
      }
    }
    auto request = std::make_shared<
      xczs_inspection_robot_interfaces::srv::SetCabinetUnlock::Request>();
    request->control_id = control.id;
    {
      std::lock_guard<std::mutex> lock(operation_lease_mutex_);
      if (unlock &&
        (!operation_lease_held_.load() || operation_lease_lost_.load() ||
        operation_lease_id_.empty()))
      {
        throw GenericOperationError(
                OperateCabinetControl::Result::NOT_READY,
                "Drawer unlock requires an active global operation lease.");
      }
      request->operation_lease_id = operation_lease_id_;
    }
    request->robot_model = robot_model_name_;
    request->right_robot_link = drawer_right_tool_.contact_tool_link;
    // The unlock pose places the tool TIP on the unlock-zone center, so tell
    // the plugin the effective tool point in the contact-link frame (tip
    // offset).  Sending (0,0,0) made the plugin measure the link ORIGIN, which
    // sits |tip| ~0.39 m away from the zone and always failed the check.
    request->right_robot_grasp_point.x = drawer_right_tool_.tool_tip_position.x();
    request->right_robot_grasp_point.y = drawer_right_tool_.tool_tip_position.y();
    request->right_robot_grasp_point.z = drawer_right_tool_.tool_tip_position.z();
    request->unlock_press_point.x = control.unlock_press_point.x();
    request->unlock_press_point.y = control.unlock_press_point.y();
    request->unlock_press_point.z = control.unlock_press_point.z();
    request->unlock = unlock;
    auto future = unlock_client_->async_send_request(request);
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(system_wait_timeout_);
    while (future.wait_for(50ms) != std::future_status::ready) {
      check_cancel(goal_handle);
      if (std::chrono::steady_clock::now() >= deadline) {
        throw GenericOperationError(
                OperateCabinetControl::Result::NOT_READY,
                "Drawer unlock request timed out.");
      }
    }
    const auto response = future.get();
    if (!response->success) {
      // 2026-09-02: the plugin now reports physical-press evidence — report it
      // so the failure is actionable (tip not at the button vs button not
      // pressed vs both).
      std::string evidence;
      if (!response->right_tool_contact && !response->pressed) {
        evidence = "; tip did not reach the button";
      } else if (!response->pressed) {
        evidence = "; tip reached the button but it was not pressed in";
      } else if (!response->right_tool_contact) {
        evidence = "; button pressed but the tip was not in contact";
      }
      throw GenericOperationError(
              OperateCabinetControl::Result::NOT_READY,
              response->message + " (distance " +
              std::to_string(response->distance) + " m" + evidence + ")");
    }
    if (require_unlocked && !response->drawer_unlocked) {
      throw GenericOperationError(
              OperateCabinetControl::Result::NOT_READY,
              "Drawer unlock was acknowledged but the rail latch stayed "
              "engaged: " + response->message);
    }
  }

  template<typename GoalHandleT>
  void set_drawer_bimanual_grasp(
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const ButtonSpec & control,
    bool attach,
    bool base_free = false)
  {
    const auto service_deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(system_wait_timeout_);
    while (!bimanual_grasp_client_->wait_for_service(50ms)) {
      check_cancel(goal_handle);
      if (std::chrono::steady_clock::now() >= service_deadline) {
        throw GenericOperationError(
                attach ? OperateCabinetControl::Result::GRASP_FAILED :
                OperateCabinetControl::Result::RELEASE_FAILED,
                "Cabinet bimanual grasp service is unavailable.");
      }
    }
    auto request = std::make_shared<
      xczs_inspection_robot_interfaces::srv::SetCabinetBimanualGrasp::Request>();
    request->control_id = control.id;
    {
      std::lock_guard<std::mutex> lock(operation_lease_mutex_);
      if (attach &&
        (!operation_lease_held_.load() || operation_lease_lost_.load() ||
        operation_lease_id_.empty()))
      {
        throw GenericOperationError(
                OperateCabinetControl::Result::GRASP_FAILED,
                "Bimanual drawer grasp attach requires an active global "
                "operation lease.");
      }
      request->operation_lease_id = operation_lease_id_;
    }
    request->robot_model = robot_model_name_;
    request->left_robot_link = drawer_left_tool_.contact_tool_link;
    request->right_robot_link = drawer_right_tool_.contact_tool_link;
    // Both tools' approach poses place their TIPS on the handle points, so the
    // plugin's distance check must be fed the tip offsets (contact-link frame),
    // not the link origin.  Mirrors the unlock request above.
    request->left_robot_grasp_point.x = drawer_left_tool_.tool_tip_position.x();
    request->left_robot_grasp_point.y = drawer_left_tool_.tool_tip_position.y();
    request->left_robot_grasp_point.z = drawer_left_tool_.tool_tip_position.z();
    request->right_robot_grasp_point.x = drawer_right_tool_.tool_tip_position.x();
    request->right_robot_grasp_point.y = drawer_right_tool_.tool_tip_position.y();
    request->right_robot_grasp_point.z = drawer_right_tool_.tool_tip_position.z();
    request->left_handle_point.x = control.left_handle_point.x();
    request->left_handle_point.y = control.left_handle_point.y();
    request->left_handle_point.z = control.left_handle_point.z();
    request->right_handle_point.x = control.right_handle_point.x();
    request->right_handle_point.y = control.right_handle_point.y();
    request->right_handle_point.z = control.right_handle_point.z();
    request->robot_base_link = grasp_brake_link_;
    request->attach = attach;
    // P3-8 grab-and-drive: the drawer pull translates the base along the
    // drawer axis, so the attach must release the chassis brake (base_free).
    request->base_free = base_free;
    auto future = bimanual_grasp_client_->async_send_request(request);
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(system_wait_timeout_);
    while (future.wait_for(50ms) != std::future_status::ready) {
      check_cancel(goal_handle);
      if (std::chrono::steady_clock::now() >= deadline) {
        throw GenericOperationError(
                attach ? OperateCabinetControl::Result::GRASP_FAILED :
                OperateCabinetControl::Result::RELEASE_FAILED,
                "Bimanual drawer grasp request timed out.");
      }
    }
    const auto response = future.get();
    if (!response->success) {
      // 2026-09-02: surface the per-side contact evidence so a hovering attach
      // is reported as such (the plugin now rejects it).
      std::string contact;
      if (attach) {
        contact = "; left_tool_contact=" +
          std::to_string(response->left_tool_contact) +
          ", right_tool_contact=" +
          std::to_string(response->right_tool_contact);
      }
      throw GenericOperationError(
              attach ? OperateCabinetControl::Result::GRASP_FAILED :
              OperateCabinetControl::Result::RELEASE_FAILED,
              response->message + " (left " +
              std::to_string(response->left_distance) + " m, right " +
              std::to_string(response->right_distance) + " m" + contact + ")");
    }
    if (attach && (!response->left_tool_contact ||
      !response->right_tool_contact))
    {
      // The plugin's attach gate already requires both contacts, but refuse a
      // hover pull even if the plugin ever relaxes: both tools must genuinely
      // touch their handles before the drawer is dragged.
      throw GenericOperationError(
              OperateCabinetControl::Result::GRASP_FAILED,
              "Bimanual drawer grasp attached without confirmed tool contact "
              "(left_tool_contact=" +
              std::to_string(response->left_tool_contact) +
              ", right_tool_contact=" +
              std::to_string(response->right_tool_contact) + "); refusing to "
              "pull in empty air.");
    }
  }

  // P3-8 grab-and-drive drawer pull.  The right arm's forearm self-collides
  // with the chassis beyond p≈0.15 and no dock shift / dock yaw / tool roll /
  // west-side push clears the full pull to the 0.3 m open detent (exhaustive
  // real-seed IK sweep).  So the pull does NOT fold the arms: it holds the
  // grasp config and translates the BASE along the drawer axis, and the
  // plugin's linear-drag coupling opens/closes the drawer 1:1 with the tools'
  // world projection.  The arm joint config never changes during the drive,
  // so no self-collision can develop.  Requires the attach to have been made
  // with base_free=true (the chassis brake was skipped so the base can move).
  template<typename GoalHandleT>
  void pull_drawer_by_base_translation(
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const ButtonSpec & control,
    double target_position,
    bool * operation_executed)
  {
    const double drawer_start = button_snapshot(control).position;
    const double travel = target_position - drawer_start;
    if (std::abs(travel) <= slider_position_tolerance_) {
      RCLCPP_INFO(
        get_logger(),
        "Drawer '%s' is already at the requested detent (%.4f); no base drive.",
        control.id.c_str(), drawer_start);
      return;
    }
    geometry_msgs::msg::TransformStamped reference_transform;
    try {
      reference_transform = transform_buffer_->lookupTransform(
        planning_frame_, docking_base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & error) {
      throw OperationError(
              PressCabinetButton::Result::NOT_READY,
              "Could not read the robot odometry before the drawer base "
              "drive: " + std::string(error.what()));
    }
    tf2::Quaternion reference_rotation;
    tf2::fromMsg(reference_transform.transform.rotation, reference_rotation);
    reference_rotation.normalize();
    const double reference_yaw = tf2::getYaw(reference_rotation);

    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(drawer_base_drive_timeout_);
    auto last_feedback = std::chrono::steady_clock::time_point{};
    bool base_commanded = false;
    try {
      while (std::chrono::steady_clock::now() < deadline) {
        check_cancel(goal_handle);
        const double drawer_now = button_snapshot(control).position;
        const double remaining = target_position - drawer_now;
        if (std::abs(remaining) <= slider_position_tolerance_) {
          publish_manual_base_stop();
          if (base_commanded && operation_executed) {
            *operation_executed = true;
          }
          return;
        }
        geometry_msgs::msg::TransformStamped current_transform;
        try {
          current_transform = transform_buffer_->lookupTransform(
            planning_frame_, docking_base_frame_, tf2::TimePointZero);
        } catch (const tf2::TransformException & error) {
          throw OperationError(
                  PressCabinetButton::Result::NOT_READY,
                  "Could not read the robot odometry during the drawer base "
                  "drive: " + std::string(error.what()));
        }
        tf2::Quaternion current_rotation;
        tf2::fromMsg(current_transform.transform.rotation, current_rotation);
        const double current_x = current_transform.transform.translation.x;
        const double current_y = current_transform.transform.translation.y;
        if (!std::isfinite(current_x) || !std::isfinite(current_y) ||
          !std::isfinite(current_rotation.length2()) ||
          current_rotation.length2() <= 1.0e-12)
        {
          throw OperationError(
                  PressCabinetButton::Result::NOT_READY,
                  "Robot odometry is invalid during the drawer base drive.");
        }
        current_rotation.normalize();
        const double current_yaw = tf2::getYaw(current_rotation);
        const int direction = remaining > 0.0 ? 1 : -1;
        const double world_speed = std::min(
          drawer_base_drive_max_speed_,
          drawer_base_drive_gain_ * std::abs(remaining));
        // Manual base twist is in the robot body frame (the router forwards
        // /xczs/manual_cmd_vel unmodified).  Transform the pure east/west
        // world velocity into the body frame and hold yaw against the yaw at
        // drive start so the tool tips stay on the handles.
        geometry_msgs::msg::Twist command;
        command.linear.x = std::cos(current_yaw) * world_speed *
          static_cast<double>(direction);
        command.linear.y = -std::sin(current_yaw) * world_speed *
          static_cast<double>(direction);
        command.angular.z = std::clamp(
          docking_angular_gain_ *
            std::atan2(
              std::sin(reference_yaw - current_yaw),
              std::cos(reference_yaw - current_yaw)),
          -docking_max_angular_speed_,
          docking_max_angular_speed_);
        manual_base_publisher_->publish(command);
        base_commanded = true;
        const auto now = std::chrono::steady_clock::now();
        if (now - last_feedback >= 500ms) {
          last_feedback = now;
          publish_operate_feedback(
            goal_handle,
            OperateCabinetControl::Feedback::MANIPULATING,
            0.62F, target_position,
            "Driving the base along the drawer axis: remaining " +
            std::to_string(std::abs(remaining)) + " m.");
        }
        std::this_thread::sleep_for(50ms);
      }
    } catch (...) {
      publish_manual_base_stop();
      throw;
    }
    publish_manual_base_stop();
    throw OperationError(
            PressCabinetButton::Result::EXECUTION_FAILED,
            "The drawer base drive did not reach the requested detent within "
            "the timeout.");
  }

  void release_drawer_bimanual_grasp_noexcept(
    const ButtonSpec & control) noexcept
  {
    try {
      if (!bimanual_grasp_client_->wait_for_service(1s)) {
        RCLCPP_ERROR(
          get_logger(),
          "Emergency bimanual grasp release service is unavailable for '%s'.",
          control.id.c_str());
        return;
      }
      constexpr int kReleaseAttempts = 2;
      for (int attempt = 1; attempt <= kReleaseAttempts; ++attempt) {
        auto request = std::make_shared<
          xczs_inspection_robot_interfaces::srv::SetCabinetBimanualGrasp::
          Request>();
        request->control_id = control.id;
        {
          std::lock_guard<std::mutex> lock(operation_lease_mutex_);
          request->operation_lease_id = operation_lease_id_;
        }
        request->robot_model = robot_model_name_;
        request->left_robot_link = drawer_left_tool_.contact_tool_link;
        request->right_robot_link = drawer_right_tool_.contact_tool_link;
        request->left_robot_grasp_point.x = drawer_left_tool_.tool_tip_position.x();
        request->left_robot_grasp_point.y = drawer_left_tool_.tool_tip_position.y();
        request->left_robot_grasp_point.z = drawer_left_tool_.tool_tip_position.z();
        request->right_robot_grasp_point.x = drawer_right_tool_.tool_tip_position.x();
        request->right_robot_grasp_point.y = drawer_right_tool_.tool_tip_position.y();
        request->right_robot_grasp_point.z = drawer_right_tool_.tool_tip_position.z();
        request->left_handle_point.x = control.left_handle_point.x();
        request->left_handle_point.y = control.left_handle_point.y();
        request->left_handle_point.z = control.left_handle_point.z();
        request->right_handle_point.x = control.right_handle_point.x();
        request->right_handle_point.y = control.right_handle_point.y();
        request->right_handle_point.z = control.right_handle_point.z();
        request->robot_base_link = grasp_brake_link_;
        request->attach = false;
        auto future = bimanual_grasp_client_->async_send_request(request);
        if (future.wait_for(2s) != std::future_status::ready) {
          RCLCPP_ERROR(
            get_logger(),
            "Emergency bimanual release timed out for '%s' (attempt %d/%d).",
            control.id.c_str(), attempt, kReleaseAttempts);
          continue;
        }
        const auto response = future.get();
        if (response->success) {
          return;
        }
        RCLCPP_ERROR(
          get_logger(),
          "Emergency bimanual release failed for '%s' (attempt %d/%d): %s",
          control.id.c_str(), attempt, kReleaseAttempts,
          response->message.c_str());
      }
      RCLCPP_ERROR(
        get_logger(),
        "Emergency bimanual release exhausted all attempts for '%s'.",
        control.id.c_str());
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        get_logger(), "Emergency bimanual release failed: %s", error.what());
    }
  }

  template<typename GoalHandleT>
  void wait_for_pregrasp_controls_stable(
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const ButtonSpec & target_control,
    const std::vector<ControlStabilityReference> & references,
    std::chrono::steady_clock::time_point boundary_started_at)
  {
    if (references.empty()) {
      throw GenericOperationError(
              OperateCabinetControl::Result::NOT_READY,
              "No initial physical-state reference is available for '" +
              target_control.id + "'.");
    }

    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(system_wait_timeout_);
    auto stable_since = std::chrono::steady_clock::time_point{};
    while (std::chrono::steady_clock::now() < deadline) {
      check_cancel(goal_handle);
      const auto now = std::chrono::steady_clock::now();
      bool all_stable = true;
      auto oldest_sample = std::chrono::steady_clock::time_point::max();
      auto newest_sample = std::chrono::steady_clock::time_point::min();
      for (const auto & reference : references) {
        if (!reference.control) {
          throw GenericOperationError(
                  OperateCabinetControl::Result::NOT_READY,
                  "A pre-grasp physical-state reference is unavailable for '" +
                  target_control.id + "'.");
        }
        const auto state = button_snapshot(*reference.control);
        const auto status = classify_pregrasp_stability_sample(
          state.structured_received,
          state.valid,
          state.structured_received_at > boundary_started_at,
          std::chrono::duration<double>(
            now - state.structured_received_at).count() <=
          button_state_timeout_,
          state.state_id == reference.state_id,
          state.in_motion,
          state.position,
          reference.position,
          state.velocity,
          target_tolerance_,
          stable_velocity_tolerance_);
        if (status ==
          PregraspStabilitySampleStatus::REFERENCE_CHANGED)
        {
          throw GenericOperationError(
                  OperateCabinetControl::Result::NOT_READY,
                  "Control '" + reference.control->id +
                  "' changed from its initial state or position before "
                  "grasp; the operation was stopped.");
        }
        if (status != PregraspStabilitySampleStatus::STABLE) {
          all_stable = false;
          break;
        }
        oldest_sample = std::min(
          oldest_sample, state.structured_received_at);
        newest_sample = std::max(
          newest_sample, state.structured_received_at);
      }
      if (all_stable) {
        if (stable_since == std::chrono::steady_clock::time_point{}) {
          // Start only once every stream has supplied a stable sample after
          // this boundary.  Requiring each stream's latest sample to advance
          // past this common point prevents a single cached sample from
          // satisfying the continuous-stability interval.
          stable_since = newest_sample;
        }
        if (oldest_sample >= stable_since &&
          std::chrono::duration<double>(
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
            "Control '" + target_control.id +
            "' and its articulated parents did not provide a continuous "
            "fresh, valid, stationary pre-grasp state.");
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
      xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_DOOR ?
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
      case PressCabinetButton::Result::TOOLSET_MISMATCH:
        return OperateCabinetControl::Result::TOOLSET_MISMATCH;
      case PressCabinetButton::Result::ADAPTER_NOT_VALIDATED:
        return OperateCabinetControl::Result::UNREACHABLE;
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
        if (result) {
          if (request_success) {
            result->failure_reason.clear();
          } else {
            result->physical_outcome_confirmed = false;
            result->final_state_verified = false;
            result->transport_succeeded = false;
            result->recovery_succeeded = false;
            result->grasp_released = false;
            if (result->failure_reason.empty()) {
              result->failure_reason = result->message.empty() ?
                "Cabinet operation failed without a diagnostic message." :
                result->message;
            }
          }
        }
        return apply_goal_terminal_disposition(
          goal_terminal_disposition(
            request_success, is_operate_goal_canceling(goal_handle)),
          [&]() {goal_handle->succeed(result);},
          [&]() {
            result->success = false;
            result->error_code = OperateCabinetControl::Result::CANCELED;
            result->message = "Cabinet operation was canceled.";
            result->failure_reason = result->message;
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
    result->peak_position = std::abs(state.position);
    result->final_state = state.state_id;
    if (control->control_type ==
      xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_BUTTON)
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
    bool canceled,
    bool drawer_bimanual_attached = false,
    const std::shared_ptr<MoveGroupInterface> & drawer_left_move_group =
      std::shared_ptr<MoveGroupInterface>()) noexcept
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
    // Bimanual drawer recovery: release the two-sided fixed constraint FIRST
    // (the drawer is still rigidly held while it is attached -- retreating
    // with the grasp on would drag the drawer), then retreat and stow BOTH
    // arms.  The single-arm retreat/stow and grasp-release paths below are
    // for door/knob/slider/button only.
    if (control && is_drawer_type(control->control_type)) {
      if (physical_recovery_required &&
        (drawer_bimanual_attached || control->requires_grasp))
      {
        result->operation_executed = true;
        release_drawer_bimanual_grasp_noexcept(*control);
      }
      if (motion_recovery_allowed && drawer_left_move_group &&
        move_group && rclcpp::ok())
      {
        best_effort_bimanual_retreat_and_stow(
          drawer_left_move_group, move_group, *control,
          &result->operation_executed);
      }
      if (control) {
        const auto final_state = button_snapshot(*control);
        result->final_position = final_state.position;
        result->peak_position = final_state.peak_position;
        result->final_state = final_state.state_id;
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
      return;
    }
    const bool is_door = control && control->control_type ==
      xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_DOOR;
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
          xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_BUTTON;
        const bool is_slider = is_slider_type(control->control_type);
        const auto retreat_pose = is_button ?
          calculate_operation_poses(
          *control, press_depth_).prepress_pose :
          (is_slider ?
           calculate_slider_operation_poses(
           *control, state.position, state.position).prepress_pose :
           calculate_rotary_tool_pose(
           *control, state.position,
           is_door ? door_release_clearance_ : prepress_distance_, false,
           rotary_tool_roll_offset));
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
      result->peak_position = final_state.peak_position;
      result->final_state = final_state.state_id;
      if (control->control_type ==
        xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_BUTTON)
      {
        result->estimated_force =
          final_state.peak_position * control->spring_stiffness;
        result->button_triggered =
          result->button_triggered ||
          final_state.peak_position + 1.0e-9 >= control->press_threshold;
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
      button.runtime->state.peak_position = update_absolute_position_peak(
        button.runtime->state.peak_position,
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
    const xczs_inspection_robot_interfaces::msg::CabinetControlState & message)
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
      state.peak_position = update_absolute_position_peak(
        state.peak_position, state.position);
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

  void reset_peak_position(const ButtonSpec & button)
  {
    std::lock_guard<std::mutex> lock(button.runtime->mutex);
    button.runtime->state.peak_position = update_absolute_position_peak(
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
            xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_BUTTON &&
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

  /**
   * Wait until a slider panel has reached its target detent.
   *
   * The plugin reports the nearest-detent state, so the state-id comparison is
   * the primary signal (robust to the panel settling short of the exact detent
   * under friction).  A small position bound around the target detent backs it
   * up so a half-latched panel that only crossed the midpoint cannot pass.
   */
  template<typename GoalHandleT>
  bool wait_for_slider_detent(
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const ButtonSpec & button,
    const std::string & target_state,
    double target_position,
    double timeout_seconds)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(timeout_seconds);
    while (std::chrono::steady_clock::now() < deadline) {
      check_cancel(goal_handle);
      {
        std::unique_lock<std::mutex> lock(button.runtime->mutex);
        if (button.runtime->state.structured_received &&
          (button.runtime->state.state_id == target_state ||
           std::abs(button.runtime->state.position - target_position) <=
           slider_position_tolerance_))
        {
          return true;
        }
        button.runtime->condition.wait_for(lock, 50ms);
      }
    }
    return false;
  }

  /**
   * Drive the slider ready (prepress) -> contact -> press chain, retrying the
   * ready planning on a fresh OMPL IK branch when the press cannot be planned.
   *
   * A 7-DOF arm has several IK families for the same prepress tool pose, and
   * the randomized ready planner lands on one each run.  Whether the following
   * 0->target Cartesian push is plannable at the configured fraction depends
   * on that branch (boot9: a bad branch gave 14%; the B3a button probe: a good
   * branch gave >=95% at the SAME dock, same poses).  PLANNING_FAILED is
   * thrown before any physical motion, so re-planning the ready on a fresh
   * branch can never double-push.  Each attempt re-plans the ready pose from
   * the current state (the arm sits at contact after a failed press), which
   * re-samples the prepress IK and therefore the contact config and the press
   * path.
   */
  template<typename GoalHandleT>
  void execute_slider_ready_approach_press(
    MoveGroupInterface & move_group,
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const std::shared_ptr<OperateCabinetControl::Result> & result,
    const OperationPoses & slider_poses,
    bool closing,
    double target_position,
    const ButtonSpec & control,
    bool * operation_executed)
  {
    constexpr int kSliderReadyRetryAttempts = 3;
    for (int attempt = 0; attempt < kSliderReadyRetryAttempts; ++attempt) {
      try {
        result->diagnostic_stage = "ready";
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::MOVING_TO_READY,
          0.25F, target_position,
          attempt == 0 ?
          "Planning the probe to the slider ready pose." :
          "The slider press was not plannable on the previous IK branch; "
          "re-planning the ready pose on a fresh branch.");
        plan_and_execute_pose(
          move_group, goal_handle, slider_poses.prepress_pose,
          contact_tool_link_, operation_executed, &control);
        result->diagnostic_stage = "approach";
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::APPROACHING,
          0.47F, target_position,
          "Approaching the slider panel along its travel axis.");
        execute_cartesian_path(
          move_group, goal_handle, {slider_poses.contact_pose},
          cartesian_velocity_scale_, cartesian_acceleration_scale_,
          0.99, operation_executed);
        result->diagnostic_stage = "manipulation";
        publish_operate_feedback(
          goal_handle,
          OperateCabinetControl::Feedback::MANIPULATING,
          closing ? 0.58F : 0.65F, target_position,
          closing ?
          "Pushing the open panel past its detent to trip the push-push "
          "release." :
          "Pushing the slider panel to the target detent.");
        execute_cartesian_path(
          move_group, goal_handle, {slider_poses.pressed_pose},
          cartesian_velocity_scale_ * 0.5,
          cartesian_acceleration_scale_ * 0.5,
          button_press_minimum_cartesian_fraction_, operation_executed);
        return;
      } catch (const OperationError & error) {
        if (error.error_code != PressCabinetButton::Result::PLANNING_FAILED ||
          attempt + 1 >= kSliderReadyRetryAttempts)
        {
          throw;
        }
        RCLCPP_WARN(
          get_logger(),
          "Slider ready/approach/press branch %d was not fully plannable "
          "(%s); re-planning the ready pose on a fresh IK branch.",
          attempt, error.what());
      }
    }
  }

  /**
   * Converge a slider panel to its target detent by re-pushing from the
   * PHYSICAL panel position, mirroring the button type's track_button_force.
   *
   * After a successful Cartesian press the panel may still sit short of the
   * detent (95% of the push leaves the tool at face+0.285 for a 0.3 target).
   * Each iteration measures the panel position, computes the remaining
   * deficit, and re-pushes from the CURRENT face
   * (calculate_slider_operation_poses) to the target.  The re-push is
   * deliberately lenient (5% minimum) so a short convergence push that cannot
   * be fully planned does not fail the operation -- the final detent
   * verification decides.  The compensation is clamped by
   * button_force_tracking_limit so it never reaches the push-push release
   * position.
   */
  template<typename GoalHandleT>
  void track_slider_force(
    MoveGroupInterface & move_group,
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const ButtonSpec & button,
    double target_position,
    bool * operation_executed = nullptr)
  {
    double commanded_position = clamp_button_press_depth(
      target_position, button.max_position);
    const double maximum_position = button_force_tracking_limit(
      target_position, force_tracking_max_compensation_, button.max_position);
    for (int attempt = 0; attempt < force_tracking_attempts_; ++attempt) {
      interruptible_hold(goal_handle, force_tracking_settle_seconds_);
      const auto measured = button_snapshot(button);
      const double deficit = target_position - measured.position;
      if (deficit <= force_tracking_tolerance_) {
        return;
      }
      const double next_position = clamp_button_press_depth(
        std::min(
          maximum_position,
          commanded_position + deficit + force_tracking_tolerance_),
        button.max_position);
      if (next_position <= commanded_position + 1.0e-6) {
        return;
      }
      commanded_position = next_position;
      publish_operate_feedback(
        goal_handle,
        OperateCabinetControl::Feedback::MANIPULATING,
        0.70F, target_position,
        "Correcting the physical slider travel to reach the requested detent.");
      const auto corrected_poses = calculate_slider_operation_poses(
        button, measured.position, commanded_position);
      try {
        execute_cartesian_path(
          move_group, goal_handle, {corrected_poses.pressed_pose},
          cartesian_velocity_scale_ * 0.35,
          cartesian_acceleration_scale_ * 0.35,
          0.05, operation_executed);
      } catch (const OperationError & error) {
        if (error.error_code != PressCabinetButton::Result::PLANNING_FAILED) {
          throw;
        }
        RCLCPP_WARN(
          get_logger(),
          "Slider convergence push (%s) could not be planned; relying on the "
          "final detent verification.", error.what());
      }
    }
    interruptible_hold(goal_handle, force_tracking_settle_seconds_);
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

  OperationPoses make_face_press_poses(
    const ButtonSpec & button,
    const tf2::Vector3 & button_face,
    double press_depth)
  {
    const tf2::Transform cabinet_transform = resolve_cabinet_transform();
    const tf2::Quaternion cabinet_rotation = cabinet_transform.getRotation();
    const auto geometry = resolve_control_geometry(button);
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
    poses.pressed_pose = make_tool_pose(press_depth);

    return poses;
  }

  OperationPoses calculate_operation_poses(
    const ButtonSpec & button,
    double press_depth)
  {
    const double bounded_press_depth = clamp_button_press_depth(
      press_depth, button.max_position);
    const tf2::Transform cabinet_transform = resolve_cabinet_transform();
    const auto geometry = resolve_control_geometry(button);
    const tf2::Vector3 button_face =
      cabinet_transform * geometry.grasp_zero;
    return make_face_press_poses(button, button_face, bounded_press_depth);
  }

  OperationPoses calculate_slider_operation_poses(
    const ButtonSpec & button,
    double current_position,
    double target_position)
  {
    const tf2::Transform cabinet_transform = resolve_cabinet_transform();
    const auto geometry = resolve_control_geometry(button);
    // The panel's press face slides with the joint, so the contact adapts to
    // the CURRENT slide position.  A signed press then lets the same front
    // approach both open the panel (target > current: the tip pushes into the
    // panel) and close it again (target < current: the tip keeps contact and
    // drags the panel back to the closed detent).
    const tf2::Vector3 face_current =
      cabinet_transform * geometry.grasp_zero +
      geometry.axis * current_position;
    const double press_depth = target_position - current_position;
    return make_face_press_poses(button, face_current, press_depth);
  }

  // Configure a MoveIt group for Cartesian tool motion.  The end-effector link
  // is normally the active tool's contact link (contact_tool_link_); bimanual
  // drawer flow configures the LEFT arm group with the LEFT tool's contact
  // link, so the caller can override the EEF per side.  The default keeps the
  // single-tool call sites unchanged.
  void configure_move_group(
    MoveGroupInterface & move_group,
    const std::string & end_effector_link = std::string())
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
    const std::string & configured_end_effector_link =
      end_effector_link.empty() ? contact_tool_link_ : end_effector_link;
    if (!robot_model->getLinkModel(configured_end_effector_link)) {
      throw OperationError(
              PressCabinetButton::Result::NOT_READY,
              "The robot adapter tool profile for MoveIt JointModelGroup '" +
              move_group_name_ + "' references missing contact tool link '" +
              configured_end_effector_link + "'.");
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
    if (!move_group.setEndEffectorLink(configured_end_effector_link)) {
      throw OperationError(
              PressCabinetButton::Result::NOT_READY,
              "MoveIt cannot use configured contact tool link '" +
              configured_end_effector_link + "'.");
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
    const ButtonSpec * control,
    const std::vector<double> * seed_positions_override = nullptr)
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
    const auto & seed_positions = seed_positions_override != nullptr ?
      *seed_positions_override : control->ready_joint_seed_positions;
    if (seed_positions.size() != control->ready_joint_seed_names.size()) {
      throw GenericOperationError(
              OperateCabinetControl::Result::NOT_READY,
              "Control '" + control->id +
              "' runtime branch seed must name every variable of MoveIt "
              "group '" + move_group_name_ + "' exactly once.");
    }

    moveit::core::RobotState seeded_goal(start_state);
    seeded_goal.setVariablePositions(
      control->ready_joint_seed_names,
      seed_positions);
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

  // Choose the IK branch for a rotary control's ready/pregrasp plans so the
  // whole Cartesian chain (50 mm approach + the manipulation arc) is feasible
  // from the REAL dock TF.  A 7-DOF arm has several IK families for one tool
  // pose; the calibrated seed is only a starting point and can fail the arc
  // (r_arm_0<->r_arm_2 self-collision near a detent, or a wrist joint pinned
  // against its limit).  Candidate branches are the configured seed and its
  // mirror (the first group joint negated).  Each candidate is dry-run with
  // computeCartesianPath -- the same path the operator will execute -- and the
  // first candidate that clears the full chain is returned.  This is a branch
  // PICKER, not a gate: if nothing clears, the configured seed is returned and
  // the real execution reports the failure honestly.
  std::vector<double> select_rotary_branch_seed(
    MoveGroupInterface & move_group,
    const ButtonSpec & control,
    const geometry_msgs::msg::Pose & pregrasp_pose,
    const geometry_msgs::msg::Pose & grasp_pose,
    const std::vector<geometry_msgs::msg::Pose> & arc_waypoints,
    const geometry_msgs::msg::Pose & retreat_pose,
    const std::string & tool_link)
  {
    if (control.ready_joint_seed_names.empty() || arc_waypoints.empty()) {
      return {};
    }
    const auto robot_model = move_group.getRobotModel();
    const auto * joint_model_group =
      robot_model == nullptr ? nullptr :
      robot_model->getJointModelGroup(move_group_name_);
    if (joint_model_group == nullptr) {
      return {};
    }
    const auto & group_variable_names = joint_model_group->getVariableNames();
    if (group_variable_names.empty() ||
      control.ready_joint_seed_names[0] != group_variable_names.front())
    {
      return {};
    }

    // A 7-DOF arm has a one-parameter family of IK branches for the same
    // pregrasp pose.  The configured seed is only one sample; different
    // shoulder (r_arm_0) seeds converge to different branches that trade off
    // clean approach/arc against a retreat with room to clear the blade.
    // Sample the branch space broadly so the operation is robust to the few
    // millimetres of docking drift between branch validation and execution.
    std::vector<std::vector<double>> candidates;
    candidates.push_back(control.ready_joint_seed_positions);
    for (const double shoulder :
      { -2.5, -2.0, -1.5, -1.0, -0.5, 0.0, 0.5, 1.0, 1.5, 2.0, 2.5 })
    {
      auto variant = control.ready_joint_seed_positions;
      variant[0] = shoulder;
      candidates.push_back(std::move(variant));
    }

    constexpr double kRequired = 0.99;
    double best_margin = -1.0;
    std::vector<double> best_margin_seed;
    double best_fraction = -1.0;
    std::vector<double> best_fraction_seed;
    for (const auto & seed : candidates) {
      try {
        const auto current_state =
          synchronized_current_robot_state(move_group);
        moveit::core::RobotState branch_state(*current_state);
        branch_state.setVariablePositions(
          control.ready_joint_seed_names, seed);
        if (!branch_state.setFromIK(
            joint_model_group, pregrasp_pose, tool_link,
            std::min(1.0, planning_time_)))
        {
          continue;
        }
        branch_state.update();
        if (!branch_state.satisfiesBounds(joint_model_group)) {
          continue;
        }

        moveit_msgs::msg::RobotTrajectory approach;
        move_group.setStartState(branch_state);
        const double f_approach = move_group.computeCartesianPath(
          {grasp_pose}, 0.002, cartesian_jump_threshold_, approach, true);
        if (f_approach < kRequired ||
          approach.joint_trajectory.points.empty())
        {
          continue;
        }

        robot_trajectory::RobotTrajectory approach_trajectory(
          move_group.getRobotModel(), move_group_name_);
        approach_trajectory.setRobotTrajectoryMsg(branch_state, approach);
        moveit::core::RobotState arc_start =
          approach_trajectory.getLastWayPoint();
        moveit_msgs::msg::RobotTrajectory arc;
        move_group.setStartState(arc_start);
        const double f_arc = move_group.computeCartesianPath(
          arc_waypoints, 0.002, cartesian_jump_threshold_, arc, true);
        if (f_arc < kRequired || arc.joint_trajectory.points.empty()) {
          continue;
        }

        // The post-release retreat is part of the same physical chain: the
        // arm must be able to pull the tool straight out of the knob at the
        // arc-end angle.  A branch can clear approach + arc and still pin a
        // joint against its limit during the retreat, so validate it too.
        robot_trajectory::RobotTrajectory arc_trajectory(
          move_group.getRobotModel(), move_group_name_);
        arc_trajectory.setRobotTrajectoryMsg(arc_start, arc);
        moveit::core::RobotState retreat_start =
          arc_trajectory.getLastWayPoint();
        moveit_msgs::msg::RobotTrajectory retreat;
        move_group.setStartState(retreat_start);
        const double f_retreat = move_group.computeCartesianPath(
          {retreat_pose}, 0.002, cartesian_jump_threshold_, retreat, true);

        const double fraction = std::min(
          {f_approach, f_arc, f_retreat});
        if (fraction > best_fraction) {
          best_fraction = fraction;
          best_fraction_seed = seed;
        }
        if (f_retreat < kRequired) {
          continue;
        }

        // Among branches that clear the whole chain, prefer the one whose
        // arc-end state sits farthest from every joint limit: it keeps the
        // retreat feasible against the docking drift that can appear between
        // this dry-run and the physical arc execution.
        const double margin = joint_limit_margin(
          retreat_start, joint_model_group);
        if (margin > best_margin) {
          best_margin = margin;
          best_margin_seed = seed;
        }
      } catch (const std::exception & error) {
        RCLCPP_WARN(
          get_logger(),
          "Rotary branch validation for '%s' failed: %s",
          control.id.c_str(), error.what());
      }
    }
    if (!best_margin_seed.empty()) {
      RCLCPP_INFO(
        get_logger(),
        "Rotary branch selected for '%s': approach+arc+retreat clear "
        "(best arc-end joint-limit margin=%.3f).",
        control.id.c_str(), best_margin);
      return best_margin_seed;
    }
    RCLCPP_WARN(
      get_logger(),
      "No rotary branch cleared approach+arc+retreat for '%s' "
      "(best min fraction=%.3f); using the best candidate. The execution "
      "reports any failure honestly.",
      control.id.c_str(), best_fraction > 0.0 ? best_fraction : 0.0);
    return best_fraction_seed.empty() ?
      control.ready_joint_seed_positions : best_fraction_seed;
  }

  double joint_limit_margin(
    const moveit::core::RobotState & state,
    const moveit::core::JointModelGroup * joint_model_group)
  {
    double min_fraction = 1.0;
    for (const auto * joint : joint_model_group->getActiveJointModels()) {
      for (const auto & variable : joint->getVariableNames()) {
        const auto & bounds = joint->getVariableBounds(variable);
        if (!bounds.position_bounded_ || bounds.min_position_ >= bounds.max_position_) {
          continue;
        }
        const double low = bounds.min_position_;
        const double high = bounds.max_position_;
        const double q = state.getVariablePosition(variable);
        const double fraction = std::min(
          (q - low) / (high - low), (high - q) / (high - low));
        min_fraction = std::min(min_fraction, fraction);
      }
    }
    return min_fraction;
  }

  // P3-8: 双手抽拉抽屉的 ready 位姿分支选择器。七轴臂对同一个 ready 工具位姿
  // 存在多组 IK 分支；run7 实测 OMPL 对右臂 ready 随机落到 r5=+2.96（腕部贴近
  // +π 限位），解锁后下探 5cm 到支撑位时 IK 在 84.4% 处耗尽（r5 无法再负转），
  // 支撑段规划失败。这里与 select_rotary_branch_seed 同构：以当前状态的 IK 分支
  // （"自然分支"，接近 home、无甩动）为基准，扫描肩部/腕部种子变体，逐个用
  // computeCartesianPath 干跑整条后续 Cartesian 链条（右臂：解锁→支撑→预抓→
  // 抓取→拽拉；左臂：支撑→预抓→抓取→拽拉），选一条整链 ≥99% 且相对自然分支
  // 甩动最小的分支。选不出则返回空，ready 走普通位姿目标（如实报告失败）。
  template<typename GoalHandleT>
  std::vector<double> select_drawer_branch_seed(
    MoveGroupInterface & move_group,
    const std::string & group_name,
    const std::string & tool_link,
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const geometry_msgs::msg::Pose & ready_pose,
    const std::vector<std::vector<geometry_msgs::msg::Pose>> & chain,
    const std::string & description)
  {
    (void)goal_handle;
    if (chain.empty()) {
      return {};
    }
    const auto robot_model = move_group.getRobotModel();
    const auto * joint_model_group =
      robot_model == nullptr ? nullptr :
      robot_model->getJointModelGroup(group_name);
    if (joint_model_group == nullptr) {
      return {};
    }
    const auto & variable_names = joint_model_group->getVariableNames();
    if (variable_names.size() != 7U) {
      return {};
    }
    const double kPi = std::acos(-1.0);

    const auto current_state = synchronized_current_robot_state(move_group);
    // 自然分支：以当前状态（home）为种子解 ready 位姿，即 OMPL 无种子时会落的
    // 分支（接近 home、无甩动）。它是所有候选变体的基准。
    moveit::core::RobotState natural(*current_state);
    if (!natural.setFromIK(
        joint_model_group, ready_pose, tool_link,
        std::min(1.0, planning_time_)))
    {
      return {};
    }
    natural.update();
    if (!natural.satisfiesBounds(joint_model_group)) {
      return {};
    }
    const auto read_positions = [&](const moveit::core::RobotState & state) {
      std::vector<double> positions;
      positions.reserve(variable_names.size());
      for (const auto & name : variable_names) {
        positions.push_back(state.getVariablePosition(name));
      }
      return positions;
    };
    const auto natural_positions = read_positions(natural);

    // 候选种子：自然分支 + 肩部(r_arm_0/l_arm_0)与腕部(r_arm_5/l_arm_5)扫描。
    // 保持其余关节为自然值，让 setFromIK 收敛到种子附近的 IK 分支。
    std::vector<std::vector<double>> candidates;
    candidates.reserve(1U + 11U + 11U);
    candidates.push_back(natural_positions);
    for (const double shoulder :
      { -2.5, -2.0, -1.5, -1.0, -0.5, 0.0, 0.5, 1.0, 1.5, 2.0, 2.5 })
    {
      auto variant = natural_positions;
      variant[0] = shoulder;
      candidates.push_back(std::move(variant));
    }
    for (const double wrist :
      { -2.5, -2.0, -1.5, -1.0, -0.5, 0.0, 0.5, 1.0, 1.5, 2.0, 2.5 })
    {
      auto variant = natural_positions;
      variant[5] = wrist;
      candidates.push_back(std::move(variant));
    }

    // 候选分支与自然分支的最大关节差（含角度回绕）。ready 用 setJointValueTarget
    // 钉到该分支；分支离自然越远，home→ready 的关节空间甩动越大（run6 的远侧
    // 解锁分支就是这种甩动的自锁来源），故只接受有限甩动且优先最小甩动。
    const double kSwingHardLimit = 5.0;
    const double kRequired = 0.99;
    const auto swing_vs_natural = [&](const std::vector<double> & values) {
      double max_swing = 0.0;
      for (std::size_t index = 0U; index < values.size(); ++index) {
        double delta = values[index] - natural_positions[index];
        delta -= std::round(delta / (2.0 * kPi)) * (2.0 * kPi);
        max_swing = std::max(max_swing, std::abs(delta));
      }
      return max_swing;
    };

    double best_fraction = -1.0;
    double best_swing = std::numeric_limits<double>::infinity();
    std::vector<double> best_clear;
    const auto run_chain = [&](const moveit::core::RobotState & start) {
      moveit::core::RobotState state(start);
      double min_fraction = 1.0;
      for (const auto & segment : chain) {
        move_group.setStartState(state);
        moveit_msgs::msg::RobotTrajectory trajectory;
        double fraction = -1.0;
        try {
          fraction = move_group.computeCartesianPath(
            segment, 0.002, cartesian_jump_threshold_, trajectory, true);
        } catch (const std::exception & error) {
          RCLCPP_WARN(
            get_logger(),
            "Drawer '%s' branch chain Cartesian threw: %s",
            description.c_str(), error.what());
        }
        if (fraction < min_fraction) {
          min_fraction = fraction;
        }
        if (fraction < kRequired || trajectory.joint_trajectory.points.empty()) {
          break;
        }
        robot_trajectory::RobotTrajectory segment_trajectory(
          move_group.getRobotModel(), group_name);
        segment_trajectory.setRobotTrajectoryMsg(state, trajectory);
        state = segment_trajectory.getLastWayPoint();
      }
      return min_fraction;
    };

    for (const auto & seed : candidates) {
      try {
        moveit::core::RobotState branch(*current_state);
        branch.setVariablePositions(variable_names, seed);
        if (!branch.setFromIK(
            joint_model_group, ready_pose, tool_link,
            std::min(1.0, planning_time_)))
        {
          continue;
        }
        branch.update();
        if (!branch.satisfiesBounds(joint_model_group)) {
          continue;
        }
        const auto branch_positions = read_positions(branch);
        const double swing = swing_vs_natural(branch_positions);
        if (swing > kSwingHardLimit) {
          continue;
        }
        const double fraction = run_chain(branch);
        if (fraction > best_fraction) {
          best_fraction = fraction;
        }
        if (fraction >= kRequired) {
          if (swing < best_swing) {
            best_swing = swing;
            best_clear = branch_positions;
          }
        } else {
          RCLCPP_DEBUG(
            get_logger(),
            "Drawer '%s' branch swing=%.3f min_fraction=%.3f.",
            description.c_str(), swing, fraction);
        }
      } catch (const std::exception & error) {
        RCLCPP_WARN(
          get_logger(),
          "Drawer '%s' branch validation failed: %s",
          description.c_str(), error.what());
      }
    }

    if (!best_clear.empty()) {
      RCLCPP_INFO(
        get_logger(),
        "Drawer '%s' ready branch selected: full Cartesian chain clears "
        "(swing=%.3f rad vs natural).",
        description.c_str(), best_swing);
      return best_clear;
    }
    RCLCPP_WARN(
      get_logger(),
      "No drawer '%s' ready branch cleared the full Cartesian chain "
      "(best min fraction=%.3f); ready will use the natural branch and "
      "report any downstream failure honestly.",
      description.c_str(), best_fraction > 0.0 ? best_fraction : 0.0);
    return {};
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
    // A calibrated seed pins OMPL to one IK branch.  When that branch is
    // unreachable or in collision, fall back to a pure pose target so the
    // validation still certifies the collision-free pose itself.
    MoveGroupInterface::Plan plan;
    moveit::core::MoveItErrorCode planning_result =
      moveit::core::MoveItErrorCode::FAILURE;
    bool planned = false;
    for (int attempt = 1; attempt <= motion_planning_attempts_; ++attempt) {
      check_cancel(goal_handle);
      move_group.setStartState(start_state);
      // Re-apply the calibrated seed on every attempt: setJointValueTarget
      // pins one IK branch, and clearPoseTargets() at the end of each
      // iteration wipes that target, so a const seed computed once would plan
      // to an empty goal (and never re-enter the seed branch) on later tries.
      const bool seed_ok = set_pose_target_with_calibrated_ik_seed(
        move_group, start_state, target, tool_link, control);
      if (seed_ok) {
        planning_result = move_group.plan(plan);
        move_group.clearPoseTargets();
        if (planning_result == moveit::core::MoveItErrorCode::SUCCESS) {
          planned = true;
          break;
        }
      }
      move_group.setStartState(start_state);
      move_group.setPoseTarget(target, tool_link);
      planning_result = move_group.plan(plan);
      move_group.clearPoseTargets();
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
        xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_BUTTON ||
      control.control_type ==
        xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_SLIDER;
    if (is_button) {
      publish_operate_feedback(
        goal_handle, OperateCabinetControl::Feedback::MOVING_TO_READY,
        0.25F, target_position,
        "正在用实时 MoveIt 场景验证控件预备位姿（不会执行轨迹）。");
      virtual_state = validate_pose_plan_only(
        move_group, goal_handle, virtual_state,
        button_poses.prepress_pose, contact_tool_link_, "ready_pose", result);
      publish_operate_feedback(
        goal_handle, OperateCabinetControl::Feedback::APPROACHING,
        0.47F, target_position,
        "正在验证控件接近路径（不会移动机械臂）。");
      virtual_state = validate_cartesian_plan_only(
        move_group, goal_handle, virtual_state,
        {button_poses.contact_pose}, 0.99, "approach", result);
      publish_operate_feedback(
        goal_handle, OperateCabinetControl::Feedback::MANIPULATING,
        0.65F, target_position,
        "正在验证请求对应的按压路径（不会接触控件）。");
      virtual_state = validate_cartesian_plan_only(
        move_group, goal_handle, virtual_state,
        {button_poses.pressed_pose}, button_press_minimum_cartesian_fraction_,
        "manipulation", result);
      publish_operate_feedback(
        goal_handle, OperateCabinetControl::Feedback::RETREATING,
        0.86F, 0.0,
        "正在验证控件撤回路径（不会执行轨迹）。");
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
            xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_DOOR ||
          control.control_type ==
            xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_KNOB)
      {
        virtual_state = validate_pose_plan_only(
          move_group, goal_handle, virtual_state,
          rotary_poses.pregrasp_pose, contact_tool_link_,
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
        xczs_inspection_robot_interfaces::msg::CabinetControl::TYPE_DOOR;
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
    const ButtonSpec * control = nullptr,
    const std::vector<double> * seed_positions_override = nullptr)
  {
    check_cancel(goal_handle);
    MoveGroupInterface::Plan plan;
    bool planned = false;
    for (int attempt = 1; attempt <= motion_planning_attempts_; ++attempt) {
      check_cancel(goal_handle);
      const auto current_state =
        synchronized_current_robot_state(move_group);
      move_group.setStartState(*current_state);
      // Prefer the calibrated IK seed so the planned branch matches the pose
      // family the seed was calibrated for.  When the seed-derived branch is
      // unreachable or in collision (e.g. dock calibration drift), fall back
      // to a pure pose target and let OMPL sample any collision-free branch.
      const bool seed_ok = set_pose_target_with_calibrated_ik_seed(
        move_group, *current_state, target, tool_link, control,
        seed_positions_override);
      moveit::core::MoveItErrorCode planning_result =
        moveit::core::MoveItErrorCode::FAILURE;
      if (seed_ok) {
        planning_result = move_group.plan(plan);
        move_group.clearPoseTargets();
        if (planning_result == moveit::core::MoveItErrorCode::SUCCESS) {
          planned = true;
          break;
        }
      }
      move_group.setStartState(*current_state);
      move_group.setPoseTarget(target, tool_link);
      const auto pose_planning_result = move_group.plan(plan);
      move_group.clearPoseTargets();
      if (pose_planning_result == moveit::core::MoveItErrorCode::SUCCESS) {
        planned = true;
        break;
      }
      RCLCPP_WARN(
        get_logger(),
        "MoveIt planning to the cabinet pose for '%s' failed "
        "(attempt %d/%d, seed=%d pose=%d).",
        tool_link.c_str(), attempt, motion_planning_attempts_,
        seed_ok ? planning_result.val : -1, pose_planning_result.val);
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
    check_cancel(goal_handle);
    const auto prepress_code = execute_motion_bounded(
      [&move_group, &plan]() { return move_group.execute(plan); },
      [this, &goal_handle]() { return goal_should_stop(goal_handle); },
      std::chrono::seconds(120), "prepress pose");
    if (prepress_code != moveit::core::MoveItErrorCode::SUCCESS) {
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
    const auto stow_code = execute_motion_bounded(
      [&move_group, &plan]() { return move_group.execute(plan); },
      [this, &goal_handle]() { return goal_should_stop(goal_handle); },
      std::chrono::seconds(120), "named target '" + target_name + "'");
    if (stow_code != moveit::core::MoveItErrorCode::SUCCESS) {
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
      // Diagnostic: report the approach start state and the tool link pose at
      // the last collision-free waypoint so a Cartesian clearance regression
      // can be reproduced and fixed from the config layer alone.
      try {
        const auto start_state = synchronized_current_robot_state(move_group);
        std::ostringstream oss;
        oss << "Cartesian fraction=" << best_fraction;
        oss << " start_joints=";
        for (const auto & name : start_state->getVariableNames()) {
          if (name.find("r_arm") == 0 || name.find("l_arm") == 0 ||
            name.find("cyl") != std::string::npos ||
            name.find("r_three") == 0)
          {
            oss << name << ":" << start_state->getVariablePosition(name)
              << " ";
          }
        }
        if (!trajectory_message.joint_trajectory.points.empty()) {
          const auto & last = trajectory_message.joint_trajectory.points.back();
          const auto & names = trajectory_message.joint_trajectory.joint_names;
          moveit::core::RobotState waypoint_state(move_group.getRobotModel());
          waypoint_state.setToDefaultValues();
          for (std::size_t i = 0; i < names.size() && i < last.positions.size();
            ++i)
          {
            waypoint_state.setVariablePosition(names[i], last.positions[i]);
          }
          waypoint_state.update();
          const Eigen::Isometry3d & link_pose =
            waypoint_state.getGlobalLinkTransform(contact_tool_link_);
          const Eigen::Vector3d t = link_pose.translation();
          oss << " last_waypoint_tool_pose=(" << t.x() << "," << t.y()
            << "," << t.z() << ")";
          oss << " waypoint_joints=";
          for (std::size_t i = 0; i < names.size() && i < last.positions.size();
            ++i)
          {
            if (names[i].find("r_arm") == 0 || names[i].find("cyl") !=
              std::string::npos || names[i].find("r_three") == 0)
            {
              oss << names[i] << ":" << last.positions[i] << " ";
            }
          }
        }
        RCLCPP_ERROR(get_logger(), "%s", oss.str().c_str());
      } catch (const std::exception & e) {
        RCLCPP_WARN(
          get_logger(), "Cartesian failure diagnostic unavailable: %s",
          e.what());
      }
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
    const auto cartesian_code = execute_motion_bounded(
      [&move_group, &trajectory_message]() {
        return move_group.execute(trajectory_message);
      },
      [this, &goal_handle]() { return goal_should_stop(goal_handle); },
      std::chrono::seconds(120), "Cartesian button path");
    if (cartesian_code != moveit::core::MoveItErrorCode::SUCCESS) {
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

  // MoveIt2 Humble's move_group.execute() busy-waits an internal done flag with
  // no timeout.  If the controller's ExecuteTrajectory result is lost, the call
  // never returns and the operator permanently blocks, holding operation_active_
  // true so every later goal returns RESOURCE_BUSY ("一直失败") until the whole
  // stack is restarted.  This wrapper runs the blocking motion on a worker
  // thread, polls a cancel predicate and a hard deadline, and when either fires
  // first asks stop_active_motion() to cancel the controller goal and then, if
  // the worker still will not return, abandons it (detach) and reports failure
  // so the operation fails cleanly instead of wedging the operator forever.
  //
  // should_stop may be null (best-effort safety retreat/stow have no goal
  // handle): then only the hard deadline applies, so a lost controller result
  // can never block a safety motion indefinitely either.  goal_should_stop on a
  // null handle would return true immediately, so callers must pass null here
  // for those noexcept best-effort paths.
  //
  // Safety properties of the wedge path (detach):
  //  - The worker never throws: motion() is wrapped so a throwing action-client
  //    cannot escape the thread and call std::terminate.
  //  - The worker holds shared ownership of the motion std::function and the
  //    result promise, so the function object survives the caller's stack; the
  //    motion's by-reference captures are only read while execute() serializes
  //    the goal, which happened before any wedge was detected.
  //  - A second execute on the same MoveGroupInterface cannot race the wedged
  //    in-flight one: every bounded motion first takes a serialization lock
  //    (motion_execute_serial_->execute_mutex), and the wedge transfers that
  //    lock into the shared serialization state until the wedged worker
  //    eventually returns.  Retreat/stow after a wedge therefore fail fast with
  //    FAILURE instead of launching a concurrent move_group.execute().  A fresh
  //    MoveGroupInterface for a later goal (acquire_active_move_group) clears
  //    the wedge lock so the new, independent instance is not bricked forever.
  moveit::core::MoveItErrorCode execute_motion_bounded(
    const std::function<moveit::core::MoveItErrorCode()> & motion,
    const std::function<bool()> & should_stop,
    const std::chrono::steady_clock::duration & hard_deadline,
    const std::string & description)
  {
    const auto serial = motion_execute_serial_;
    // Refuse immediately (no blocking) if a prior wedge still holds the
    // execute serialization: launching a second execute() on the same
    // MoveGroupInterface while the wedged worker is still inside the first
    // one would be a data race on a non-thread-safe object.
    std::unique_lock<std::mutex> execute_serial(
      serial->execute_mutex, std::try_to_lock);
    if (!execute_serial.owns_lock()) {
      RCLCPP_ERROR(
        get_logger(),
        "Bounded '%s' refused: a previous bounded motion is still in flight "
        "on this move group (wedged backend); refusing a concurrent execute.",
        description.c_str());
      return moveit::core::MoveItErrorCode::FAILURE;
    }
    const auto result_holder =
      std::make_shared<std::promise<moveit::core::MoveItErrorCode>>();
    std::future<moveit::core::MoveItErrorCode> future =
      result_holder->get_future();
    // Heap-shared copy of the motion closure: a detached worker outlives this
    // call and must not touch the caller's stack-owned std::function.
    const auto motion_holder =
      std::make_shared<std::function<moveit::core::MoveItErrorCode()>>(
        motion);
    const rclcpp::Logger bounded_logger = get_logger();
    std::thread worker([result_holder, motion_holder, serial,
        bounded_logger]() {
      try {
        result_holder->set_value((*motion_holder)());
      } catch (const std::exception & error) {
        RCLCPP_ERROR(
          bounded_logger,
          "Bounded motion threw inside its worker thread: %s",
          error.what());
        try {
          result_holder->set_value(moveit::core::MoveItErrorCode::FAILURE);
        } catch (...) {
        }
      } catch (...) {
        try {
          result_holder->set_value(moveit::core::MoveItErrorCode::FAILURE);
        } catch (...) {
        }
      }
      // If this worker was detached on the wedge path, its in-flight
      // execute() transferred the serialization lock into serial->wedged_lock;
      // clear it now that the execute has actually returned so later bounded
      // motions can run again.
      std::lock_guard<std::mutex> lock(serial->guard_mutex);
      serial->wedged_lock.reset();
    });
    const auto deadline_time =
      std::chrono::steady_clock::now() + hard_deadline;
    moveit::core::MoveItErrorCode code =
      moveit::core::MoveItErrorCode::FAILURE;
    bool worker_detached = false;
    while (true) {
      if (future.wait_for(std::chrono::milliseconds(50)) ==
        std::future_status::ready) {
        code = future.get();
        break;
      }
      const auto now = std::chrono::steady_clock::now();
      const bool stop_requested =
        (should_stop && should_stop()) || now >= deadline_time;
      if (stop_requested) {
        RCLCPP_WARN(
          get_logger(),
          "Bounded '%s' hit its deadline or was canceled; stopping motion.",
          description.c_str());
        stop_active_motion();
        const auto grace_deadline =
          now + std::chrono::milliseconds(2500);
        while (future.wait_for(std::chrono::milliseconds(50)) !=
          std::future_status::ready) {
          if (std::chrono::steady_clock::now() >= grace_deadline) {
            break;
          }
        }
        if (future.wait_for(std::chrono::seconds(0)) ==
          std::future_status::ready) {
          code = future.get();
        } else {
          RCLCPP_ERROR(
            get_logger(),
            "Bounded '%s' is permanently wedged (lost controller result); "
            "abandoning the motion worker and failing the operation.",
            description.c_str());
          // Keep the execute serialization held until the wedged worker
          // returns (worker clears it) or a fresh move group replaces this
          // one, so no retreat/stow on this instance races the in-flight
          // execute() with a concurrent move_group.execute().
          {
            std::lock_guard<std::mutex> lock(serial->guard_mutex);
            serial->wedged_lock.emplace(std::move(execute_serial));
          }
          worker.detach();
          worker_detached = true;
          code = moveit::core::MoveItErrorCode::FAILURE;
        }
        break;
      }
    }
    if (!worker_detached && worker.joinable()) {
      worker.join();
    }
    return code;
  }

  void retime_cartesian_trajectory_for_group(
    MoveGroupInterface & move_group,
    const std::string & group_name,
    moveit_msgs::msg::RobotTrajectory & trajectory_message,
    double velocity_scale,
    double acceleration_scale)
  {
    const auto current_state = synchronized_current_robot_state(move_group);
    robot_trajectory::RobotTrajectory trajectory(
      move_group.getRobotModel(), group_name);
    trajectory.setRobotTrajectoryMsg(*current_state, trajectory_message);
    trajectory_processing::TimeOptimalTrajectoryGeneration time_parameterizer;
    if (!time_parameterizer.computeTimeStamps(
        trajectory,
        velocity_scale,
        acceleration_scale))
    {
      throw OperationError(
              PressCabinetButton::Result::PLANNING_FAILED,
              "MoveIt could not generate a low-speed Cartesian trajectory "
              "for drawer group '" + group_name + "'.");
    }
    trajectory.getRobotTrajectoryMsg(trajectory_message);
  }

  struct BimanualExecutionResult
  {
    bool left_started{false};
    bool right_started{false};
    moveit::core::MoveItErrorCode left_code{
      moveit::core::MoveItErrorCode::FAILURE};
    moveit::core::MoveItErrorCode right_code{
      moveit::core::MoveItErrorCode::FAILURE};
  };

  // 把 move_group 规划好的关节轨迹直发 follow_joint_trajectory 控制器并等待
  // 结果。左右各由独立 worker 调用，因此两条臂真正并行执行（不再经过
  // move_group 的 execute_trajectory action——该 action 在单线程执行器上串行
  // 处理目标，先执行的那条臂会被另一条臂对抽屉的焊接固定物理锁死，最终以
  // GOAL_TOLERANCE_VIOLATED 超时）。超时/取消由调用方的有界执行循环经
  // stop_active_motion() 撤销控制器目标驱动本函数返回。本函数不触碰 this，
  // detach 的楔死 worker 在 operator 销毁后仍可安全运行。
  // 嵌套类型声明于类成员区（参数签名需要先于完整定义前向声明；函数体在
  // complete-class 上下文，仍可见 10461 处的完整定义）。
  struct BimanualControllerHandles;
  static moveit::core::MoveItErrorCode execute_controller_trajectory(
    const rclcpp::Logger & logger,
    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory> & client,
    const std::shared_ptr<BimanualControllerHandles> & handles,
    const trajectory_msgs::msg::JointTrajectory & joint_trajectory,
    bool left_side) noexcept
  {
    if (joint_trajectory.points.empty()) {
      return moveit::core::MoveItErrorCode::SUCCESS;
    }
    if (!client.wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(
        logger,
        "Bimanual %s-arm controller action server is unavailable.",
        left_side ? "left" : "right");
      return moveit::core::MoveItErrorCode::FAILURE;
    }
    auto & goal_slot = left_side ? handles->left : handles->right;
    auto goal = control_msgs::action::FollowJointTrajectory::Goal();
    goal.trajectory = joint_trajectory;
    // 抽屉共享导轨强制双臂 X 同步，而两条臂的规划时长可能略有差异（各自的
    // 关节速度上限不同）。给控制器目标时间留足余量，避免较慢一条臂被同步
    // 锁死而误报 GOAL_TOLERANCE_VIOLATED；真正超时由调用方 120s 有界截止
    // 负责。
    goal.goal_time_tolerance = rclcpp::Duration::from_seconds(10.0);
    try {
      const auto send_future = client.async_send_goal(goal);
      const auto send_result = send_future.get();
      if (!send_result) {
        RCLCPP_ERROR(
          logger, "Bimanual %s-arm controller goal was rejected.",
          left_side ? "left" : "right");
        return moveit::core::MoveItErrorCode::FAILURE;
      }
      {
        std::lock_guard<std::mutex> lock(handles->mutex);
        goal_slot = send_result;
      }
      const auto result_future = client.async_get_result(send_result);
      while (result_future.wait_for(std::chrono::milliseconds(50)) !=
        std::future_status::ready) {
      }
      {
        std::lock_guard<std::mutex> lock(handles->mutex);
        goal_slot.reset();
      }
      const auto wrapped_result = result_future.get();
      if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED &&
        wrapped_result.result->error_code ==
          control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL)
      {
        return moveit::core::MoveItErrorCode::SUCCESS;
      }
      RCLCPP_WARN(
        logger,
        "Bimanual %s-arm controller execution ended with status %d and "
        "error code %d.",
        left_side ? "left" : "right",
        static_cast<int>(wrapped_result.code),
        wrapped_result.result->error_code);
      return moveit::core::MoveItErrorCode::CONTROL_FAILED;
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        logger, "Bimanual %s-arm direct controller execution threw: %s",
        left_side ? "left" : "right", error.what());
      {
        std::lock_guard<std::mutex> lock(handles->mutex);
        goal_slot.reset();
      }
      return moveit::core::MoveItErrorCode::FAILURE;
    }
  }

  // Concurrent two-arm execution primitive for the bimanual drawer.
  //
  // The left and right arms are driven by SEPARATE FollowJointTrajectory
  // controllers through distinct MoveGroupInterface instances, so both
  // execute() calls can genuinely run in parallel.  The single-arm
  // execute_motion_bounded serializes every execute() on one shared mutex,
  // which would defeat the synchronization; this primitive instead runs both
  // workers under ONE hold of that serialization (both plans belong to the
  // same drawer operation), and stops BOTH arms on cancel/deadline by way of
  // stop_active_motion(), which now also cancels bimanual_active_move_groups_.
  //
  // The wedge path mirrors execute_motion_bounded: on a lost controller
  // result both workers are detached and the serialization lock is transferred
  // into motion_execute_serial_->wedged_lock.  Only the LAST worker to return
  // clears that wedge (shared atomic counter), so a normally-returning right
  // worker cannot clear the wedge while the wedged left worker is still inside
  // execute() on the left group.
  BimanualExecutionResult execute_bimanual_trajectories_bounded(
    const std::shared_ptr<MoveGroupInterface> & left_group,
    const moveit_msgs::msg::RobotTrajectory & left_trajectory,
    const std::shared_ptr<MoveGroupInterface> & right_group,
    const moveit_msgs::msg::RobotTrajectory & right_trajectory,
    const std::function<bool()> & should_stop,
    const std::chrono::steady_clock::duration & hard_deadline,
    const std::string & description)
  {
    BimanualExecutionResult result;
    const auto serial = motion_execute_serial_;
    std::unique_lock<std::mutex> execute_serial(
      serial->execute_mutex, std::try_to_lock);
    if (!execute_serial.owns_lock()) {
      RCLCPP_ERROR(
        get_logger(),
        "Bimanual '%s' refused: a previous bounded motion is still in flight "
        "(wedged backend).",
        description.c_str());
      return result;
    }
    {
      std::lock_guard<std::mutex> lock(motion_mutex_);
      bimanual_active_move_groups_ = {left_group, right_group};
    }
    struct BimanualWorkerState
    {
      std::mutex guard_mutex;
      std::atomic<int> remaining_workers{2};
      bool cleared_wedge{false};
    };
    const auto worker_state = std::make_shared<BimanualWorkerState>();
    const auto left_promise = std::make_shared<
      std::promise<moveit::core::MoveItErrorCode>>();
    auto left_future = left_promise->get_future();
    const auto right_promise = std::make_shared<
      std::promise<moveit::core::MoveItErrorCode>>();
    auto right_future = right_promise->get_future();
    const rclcpp::Logger bounded_logger = get_logger();
    // 轨迹按值拷贝进 worker：楔死 detach 后 worker 可能晚于调用方返回，
    // 不能引用调用方栈上的 left_trajectory。控制器 client 与 goal-handle 池
    // 按值捕获，detach worker 全程不触碰 this；group 由调用方在
    // bimanual_active_move_groups_ 中保持存活（stop_active_motion 用）。
    const auto run_one = [bounded_logger, serial, worker_state,
        handles = bimanual_controller_handles_,
        left_client = left_fjt_client_,
        right_client = right_fjt_client_](
        bool left_side,
        const moveit_msgs::msg::RobotTrajectory trajectory,
        const std::shared_ptr<std::promise<moveit::core::MoveItErrorCode>> &
        promise) {
        moveit::core::MoveItErrorCode code =
          moveit::core::MoveItErrorCode::FAILURE;
        try {
          auto & client = left_side ? *left_client : *right_client;
          code = execute_controller_trajectory(
            bounded_logger, client, handles, trajectory.joint_trajectory,
            left_side);
        } catch (const std::exception & error) {
          RCLCPP_ERROR(
            bounded_logger, "Bimanual worker threw: %s", error.what());
        } catch (...) {
        }
        try {
          promise->set_value(code);
        } catch (...) {
        }
        if (worker_state->remaining_workers.fetch_sub(1) == 1) {
          std::lock_guard<std::mutex> lock(serial->guard_mutex);
          serial->wedged_lock.reset();
        }
      };
    std::thread left_worker(run_one, true, left_trajectory, left_promise);
    std::thread right_worker(run_one, false, right_trajectory, right_promise);
    result.left_started = true;
    result.right_started = true;

    const auto deadline_time =
      std::chrono::steady_clock::now() + hard_deadline;
    bool left_wedged = false;
    bool right_wedged = false;
    bool worker_detached = false;
    while (true) {
      const auto left_ready = left_future.wait_for(
        std::chrono::milliseconds(50)) == std::future_status::ready;
      const auto right_ready = right_future.wait_for(
        std::chrono::milliseconds(0)) == std::future_status::ready;
      if (left_ready && right_ready) {
        result.left_code = left_future.get();
        result.right_code = right_future.get();
        break;
      }
      const auto now = std::chrono::steady_clock::now();
      const bool stop_requested =
        (should_stop && should_stop()) || now >= deadline_time;
      if (!stop_requested) {
        continue;
      }
      RCLCPP_WARN(
        get_logger(),
        "Bimanual '%s' hit its deadline or was canceled; stopping both arms.",
        description.c_str());
      stop_active_motion();
      const auto grace_deadline = now + std::chrono::milliseconds(2500);
      while (true) {
        const auto grace_left_ready = left_future.wait_for(
          std::chrono::milliseconds(50)) == std::future_status::ready;
        const auto grace_right_ready = right_future.wait_for(
          std::chrono::milliseconds(0)) == std::future_status::ready;
        if (grace_left_ready && grace_right_ready) {
          break;
        }
        if (std::chrono::steady_clock::now() >= grace_deadline) {
          break;
        }
      }
      if (left_future.wait_for(std::chrono::seconds(0)) ==
        std::future_status::ready)
      {
        result.left_code = left_future.get();
      } else {
        left_wedged = true;
      }
      if (right_future.wait_for(std::chrono::seconds(0)) ==
        std::future_status::ready)
      {
        result.right_code = right_future.get();
      } else {
        right_wedged = true;
      }
      if (left_wedged || right_wedged) {
        RCLCPP_ERROR(
          get_logger(),
          "Bimanual '%s' is permanently wedged on %s%s%s; abandoning both "
          "motion workers and failing the operation.",
          description.c_str(),
          left_wedged ? "left" : "",
          left_wedged && right_wedged ? "+" : "",
          right_wedged ? "right" : "");
        {
          std::lock_guard<std::mutex> lock(serial->guard_mutex);
          serial->wedged_lock.emplace(std::move(execute_serial));
        }
        worker_detached = true;
        left_worker.detach();
        right_worker.detach();
      }
      break;
    }
    if (!worker_detached) {
      if (left_worker.joinable()) {
        left_worker.join();
      }
      if (right_worker.joinable()) {
        right_worker.join();
      }
    }
    {
      std::lock_guard<std::mutex> lock(motion_mutex_);
      bimanual_active_move_groups_ = {};
    }
    return result;
  }

  // Plan one arm's Cartesian segment from its measured state and retime it.
  // Used by the bimanual segmented path so BOTH arm plans exist before either
  // executes (a one-sided execute would yank the drawer).
  template<typename GoalHandleT>
  moveit_msgs::msg::RobotTrajectory plan_drawer_cartesian_segment(
    MoveGroupInterface & move_group,
    const std::string & group_name,
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const std::vector<geometry_msgs::msg::Pose> & waypoints,
    double velocity_scale,
    double acceleration_scale,
    const std::string & description,
    double minimum_fraction = 0.99)
  {
    check_cancel(goal_handle);
    moveit_msgs::msg::RobotTrajectory trajectory_message;
    double best_fraction = -1.0;
    for (int attempt = 0; attempt < cartesian_planning_attempts_; ++attempt) {
      check_cancel(goal_handle);
      const auto current_state = synchronized_current_robot_state(move_group);
      move_group.setStartState(*current_state);
      moveit_msgs::msg::RobotTrajectory candidate;
      double fraction = -1.0;
      try {
        fraction = move_group.computeCartesianPath(
          waypoints, 0.002, cartesian_jump_threshold_, candidate, true);
      } catch (const std::exception & error) {
        RCLCPP_WARN(
          get_logger(),
          "Drawer '%s' Cartesian planning threw on attempt %d: %s",
          description.c_str(), attempt + 1, error.what());
      }
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
              "Drawer '" + description + "' Cartesian segment completed only " +
              std::to_string(std::max(0.0, best_fraction) * 100.0) + "% of "
              "the path (needs " +
              std::to_string(minimum_fraction * 100.0) + "%).");
    }
    retime_cartesian_trajectory_for_group(
      move_group, group_name, trajectory_message,
      velocity_scale, acceleration_scale);
    check_cancel(goal_handle);
    return trajectory_message;
  }

  // P3-8: 双臂焊接后共享同一导轨，任一时刻两把工具在导轨上的位置必须一致。
  // 两条臂各自 TOTG 重定时（关节速度上限不同）时长可能不同；时长差会让较快
  // 一条臂在慢臂到达前被固定关节顶住，ODE 闭环自锁 → 控制器 GOAL_TOLERANCE
  // 超时。把两条轨迹按较慢一条统一线性拉伸到同一末时刻，保持空间-时间比例，
  // 使双臂沿导轨严格同步。
  static void synchronize_bimanual_trajectory_durations(
    moveit_msgs::msg::RobotTrajectory & left,
    moveit_msgs::msg::RobotTrajectory & right)
  {
    const auto last_duration = [](
      const moveit_msgs::msg::RobotTrajectory & trajectory) {
      const auto & points = trajectory.joint_trajectory.points;
      if (points.empty()) {
        return 0.0;
      }
      return rclcpp::Duration(points.back().time_from_start).seconds();
    };
    const double target = std::max(
      last_duration(left), last_duration(right));
    if (target <= 0.0) {
      return;
    }
    const auto stretch_to_target = [target](
      moveit_msgs::msg::RobotTrajectory & trajectory) {
      auto & points = trajectory.joint_trajectory.points;
      if (points.empty()) {
        return;
      }
      const double duration =
        rclcpp::Duration(points.back().time_from_start).seconds();
      if (duration <= 0.0) {
        return;
      }
      const double scale = target / duration;
      for (auto & point : points) {
        point.time_from_start = rclcpp::Duration::from_seconds(
          rclcpp::Duration(point.time_from_start).seconds() * scale);
      }
    };
    stretch_to_target(left);
    stretch_to_target(right);
  }

  // Segmented, matched, concurrent drawer pull/push.  Each segment is planned
  // for BOTH arms from their measured states and then executed concurrently;
  // the i-th left/right waypoint always command the same drawer rail position.
  // A segment that cannot be planned on either arm is retried one waypoint at
  // a time; a single unplannable waypoint fails the operation (the caller
  // stops, releases the bimanual grasp and retreats both arms).
  template<typename GoalHandleT>
  void execute_bimanual_segmented_cartesian_path(
    const std::shared_ptr<MoveGroupInterface> & left_group,
    const std::shared_ptr<MoveGroupInterface> & right_group,
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const std::vector<geometry_msgs::msg::Pose> & left_waypoints,
    const std::vector<geometry_msgs::msg::Pose> & right_waypoints,
    std::size_t maximum_segment_waypoints,
    double velocity_scale,
    double acceleration_scale,
    std::size_t & completed_waypoints,
    bool * operation_executed = nullptr)
  {
    if (left_waypoints.size() != right_waypoints.size()) {
      throw std::invalid_argument(
              "Bimanual drawer waypoint vectors must have equal length.");
    }
    if (maximum_segment_waypoints == 0U) {
      throw std::invalid_argument(
              "Bimanual drawer Cartesian segment size must be greater than "
              "zero.");
    }
    completed_waypoints = 0U;
    const auto run_segment = [&](
        std::size_t begin, std::size_t end,
        const std::vector<geometry_msgs::msg::Pose> & left_segment,
        const std::vector<geometry_msgs::msg::Pose> & right_segment) {
        check_cancel(goal_handle);
        auto left_trajectory = plan_drawer_cartesian_segment(
          *left_group, drawer_left_tool_.move_group, goal_handle,
          left_segment, velocity_scale, acceleration_scale, "left");
        auto right_trajectory = plan_drawer_cartesian_segment(
          *right_group, drawer_right_tool_.move_group, goal_handle,
          right_segment, velocity_scale, acceleration_scale, "right");
        // P3-8: 两条臂各自 TOTG 重定时时长不同，焊接闭环自锁；统一到同一
        // 末时刻再执行（见 synchronize_bimanual_trajectory_durations）。
        synchronize_bimanual_trajectory_durations(
          left_trajectory, right_trajectory);
        const auto execution = execute_bimanual_trajectories_bounded(
          left_group, left_trajectory,
          right_group, right_trajectory,
          [this, &goal_handle]() { return goal_should_stop(goal_handle); },
          std::chrono::seconds(120), "drawer segment");
        if (execution.left_code != moveit::core::MoveItErrorCode::SUCCESS ||
          execution.right_code != moveit::core::MoveItErrorCode::SUCCESS)
        {
          check_cancel(goal_handle);
          throw OperationError(
                  PressCabinetButton::Result::EXECUTION_FAILED,
                  "Drawer bimanual segment execution failed (left=" +
                  std::to_string(execution.left_code.val) + " right=" +
                  std::to_string(execution.right_code.val) + ").");
        }
        if (operation_executed) {
          *operation_executed = true;
        }
        completed_waypoints = end;
      };
    const std::size_t segment_count =
      (left_waypoints.size() + maximum_segment_waypoints - 1U) /
      maximum_segment_waypoints;
    std::size_t segment_index = 0U;
    for (std::size_t begin = 0U; begin < left_waypoints.size();
      begin += maximum_segment_waypoints)
    {
      ++segment_index;
      const auto end = std::min(
        begin + maximum_segment_waypoints, left_waypoints.size());
      const std::vector<geometry_msgs::msg::Pose> left_segment(
        left_waypoints.begin() + begin, left_waypoints.begin() + end);
      const std::vector<geometry_msgs::msg::Pose> right_segment(
        right_waypoints.begin() + begin, right_waypoints.begin() + end);
      RCLCPP_INFO(
        get_logger(),
        "Starting drawer bimanual segment %zu/%zu [%zu, %zu); "
        "%zu/%zu waypoints already executed.",
        segment_index, segment_count, begin, end,
        completed_waypoints, left_waypoints.size());
      try {
        run_segment(begin, end, left_segment, right_segment);
      } catch (const OperationError & error) {
        if (error.error_code != PressCabinetButton::Result::PLANNING_FAILED ||
          left_segment.size() == 1U)
        {
          throw;
        }
        RCLCPP_WARN(
          get_logger(),
          "Drawer bimanual segment %zu/%zu was not fully plannable on both "
          "arms; retrying one waypoint at a time.",
          segment_index, segment_count);
        for (std::size_t offset = 0U; offset < left_segment.size(); ++offset) {
          run_segment(
            begin + offset, begin + offset + 1U,
            {left_segment[offset]}, {right_segment[offset]});
          RCLCPP_INFO(
            get_logger(),
            "Drawer bimanual fallback waypoint %zu/%zu completed; "
            "%zu/%zu waypoints executed.",
            offset + 1U, left_segment.size(), completed_waypoints,
            left_waypoints.size());
        }
      }
    }
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
      const auto best_effort_code = execute_motion_bounded(
        [&move_group, &trajectory_message]() {
          return move_group.execute(trajectory_message);
        },
        std::function<bool()>(), std::chrono::seconds(60),
        description);
      if (best_effort_code != moveit::core::MoveItErrorCode::SUCCESS) {
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
      const auto stow_code = execute_motion_bounded(
        [&move_group, &plan]() { return move_group.execute(plan); },
        std::function<bool()>(), std::chrono::seconds(60), "safety stow");
      if (stow_code != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "Safety stow execution failed.");
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Safety stow failed: %s", error.what());
    }
  }

  void best_effort_stow_named_target(
    MoveGroupInterface & move_group,
    const std::string & target_name,
    bool * operation_executed = nullptr) noexcept
  {
    try {
      move_group.stop();
      const auto current_state = synchronized_current_robot_state(move_group);
      move_group.setStartState(*current_state);
      if (!move_group.setNamedTarget(target_name)) {
        RCLCPP_ERROR(
          get_logger(), "Safety target '%s' is not configured.",
          target_name.c_str());
        return;
      }
      MoveGroupInterface::Plan plan;
      if (move_group.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "Safety stow planning failed for '%s'.",
          target_name.c_str());
        return;
      }
      if (operation_executed) {
        *operation_executed = true;
      }
      const auto stow_code = execute_motion_bounded(
        [&move_group, &plan]() { return move_group.execute(plan); },
        std::function<bool()>(), std::chrono::seconds(60),
        "safety stow '" + target_name + "'");
      if (stow_code != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(
          get_logger(), "Safety stow execution failed for '%s'.",
          target_name.c_str());
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Safety stow failed: %s", error.what());
    }
  }

  template<typename GoalHandleT>
  MoveGroupInterface::Plan plan_arm_pose(
    MoveGroupInterface & move_group,
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const geometry_msgs::msg::Pose & target,
    const std::string & tool_link,
    const std::string & description)
  {
    check_cancel(goal_handle);
    MoveGroupInterface::Plan plan;
    bool planned = false;
    moveit::core::RobotStatePtr start_state;
    for (int attempt = 1; attempt <= motion_planning_attempts_; ++attempt) {
      check_cancel(goal_handle);
      start_state = synchronized_current_robot_state(move_group);
      move_group.setStartState(*start_state);
      move_group.setPoseTarget(target, tool_link);
      const auto planning_result = move_group.plan(plan);
      move_group.clearPoseTargets();
      if (planning_result == moveit::core::MoveItErrorCode::SUCCESS) {
        planned = true;
        break;
      }
      RCLCPP_WARN(
        get_logger(),
        "Drawer '%s' pose planning failed (attempt %d/%d).",
        description.c_str(), attempt, motion_planning_attempts_);
    }
    if (!planned) {
      throw OperationError(
              PressCabinetButton::Result::PLANNING_FAILED,
              "MoveIt could not plan drawer '" + description +
              "' to its target pose after " +
              std::to_string(motion_planning_attempts_) + " attempts.");
    }
    check_cancel(goal_handle);
    return plan;
  }

  template<typename GoalHandleT>
  void plan_and_execute_bimanual_poses(
    const std::shared_ptr<MoveGroupInterface> & left_group,
    const std::shared_ptr<MoveGroupInterface> & right_group,
    const std::shared_ptr<GoalHandleT> & goal_handle,
    const std::string & left_group_name,
    const std::string & right_group_name,
    const geometry_msgs::msg::Pose & left_target,
    const geometry_msgs::msg::Pose & right_target,
    const std::vector<double> & left_branch_seed,
    const std::vector<double> & right_branch_seed,
    bool * operation_executed,
    const std::string & description)
  {
    // P3-8: ready 位姿若带选定的分支种子（select_drawer_branch_seed），就用
    // setJointValueTarget 把目标钉到该 IK 分支的精确关节构型，使后续整条
    // Cartesian 链条（解锁→支撑→预抓→抓取→拽拉）都可规划；否则回退普通位姿
    // 目标，让 OMPL 任意采样分支（run7 的 r5=+2.96 即由此随机产生）。
    const auto plan_arm_to_ready = [&](
        MoveGroupInterface & group, const geometry_msgs::msg::Pose & target,
        const std::string & tool_link, const std::string & group_name,
        const std::vector<double> & branch_seed, const std::string & side) {
      check_cancel(goal_handle);
      MoveGroupInterface::Plan plan;
      bool planned = false;
      for (int attempt = 1; attempt <= motion_planning_attempts_; ++attempt) {
        check_cancel(goal_handle);
        const auto current_state = synchronized_current_robot_state(group);
        group.setStartState(*current_state);
        bool branch_ok = false;
        if (!branch_seed.empty()) {
          const auto robot_model = group.getRobotModel();
          const auto * joint_model_group =
            robot_model == nullptr ? nullptr :
            robot_model->getJointModelGroup(group_name);
          if (joint_model_group != nullptr) {
            const auto & variable_names = joint_model_group->getVariableNames();
            if (variable_names.size() == branch_seed.size()) {
              moveit::core::RobotState goal_state(*current_state);
              goal_state.setVariablePositions(variable_names, branch_seed);
              goal_state.update();
              if (goal_state.satisfiesBounds(joint_model_group)) {
                branch_ok = group.setJointValueTarget(goal_state);
              }
            }
          }
        }
        moveit::core::MoveItErrorCode planning_result =
          moveit::core::MoveItErrorCode::FAILURE;
        if (branch_ok) {
          planning_result = group.plan(plan);
          group.clearPoseTargets();
          if (planning_result == moveit::core::MoveItErrorCode::SUCCESS) {
            planned = true;
            break;
          }
        }
        group.setStartState(*current_state);
        group.setPoseTarget(target, tool_link);
        const auto pose_planning_result = group.plan(plan);
        group.clearPoseTargets();
        if (pose_planning_result == moveit::core::MoveItErrorCode::SUCCESS) {
          planned = true;
          break;
        }
        RCLCPP_WARN(
          get_logger(),
          "Drawer ready planning to '%s' failed (attempt %d/%d, branch=%d "
          "pose=%d).",
          side.c_str(), attempt, motion_planning_attempts_,
          branch_ok ? planning_result.val : -1,
          pose_planning_result.val);
      }
      if (!planned) {
        throw OperationError(
                PressCabinetButton::Result::PLANNING_FAILED,
                "MoveIt could not plan drawer '" + description + "' " + side +
                " arm to its ready pose after " +
                std::to_string(motion_planning_attempts_) + " attempts.");
      }
      check_cancel(goal_handle);
      return plan;
    };

    const auto left_plan = plan_arm_to_ready(
      *left_group, left_target, drawer_left_tool_.contact_tool_link,
      left_group_name, left_branch_seed, "left");
    const auto right_plan = plan_arm_to_ready(
      *right_group, right_target, drawer_right_tool_.contact_tool_link,
      right_group_name, right_branch_seed, "right");
    const auto execution = execute_bimanual_trajectories_bounded(
      left_group, left_plan.trajectory_,
      right_group, right_plan.trajectory_,
      [this, &goal_handle]() { return goal_should_stop(goal_handle); },
      std::chrono::seconds(120), description);
    if (execution.left_code != moveit::core::MoveItErrorCode::SUCCESS ||
      execution.right_code != moveit::core::MoveItErrorCode::SUCCESS)
    {
      check_cancel(goal_handle);
      throw OperationError(
              PressCabinetButton::Result::EXECUTION_FAILED,
              "MoveIt failed to execute bimanual '" + description +
              "' (left=" + std::to_string(execution.left_code.val) +
              " right=" + std::to_string(execution.right_code.val) + ").");
    }
    if (operation_executed) {
      *operation_executed = true;
    }
    check_cancel(goal_handle);
  }

  // Best-effort concurrent pose motion for both arms, used by the drawer
  // safety recovery.  No goal handle (deadline-only), never throws.
  bool best_effort_bimanual_pose_move(
    const std::shared_ptr<MoveGroupInterface> & left_group,
    const std::shared_ptr<MoveGroupInterface> & right_group,
    const geometry_msgs::msg::Pose & left_target,
    const geometry_msgs::msg::Pose & right_target,
    const std::string & description) noexcept
  {
    if (!left_group || !right_group) {
      return false;
    }
    bool ok = false;
    try {
      MoveGroupInterface::Plan left_plan;
      MoveGroupInterface::Plan right_plan;
      bool left_planned = false;
      bool right_planned = false;
      for (int attempt = 1; attempt <= motion_planning_attempts_; ++attempt) {
        if (!left_planned) {
          const auto state = synchronized_current_robot_state(*left_group);
          left_group->setStartState(*state);
          left_group->setPoseTarget(
            left_target, drawer_left_tool_.contact_tool_link);
          if (left_group->plan(left_plan) ==
            moveit::core::MoveItErrorCode::SUCCESS)
          {
            left_group->clearPoseTargets();
            left_planned = true;
          } else {
            left_group->clearPoseTargets();
          }
        }
        if (!right_planned) {
          const auto state = synchronized_current_robot_state(*right_group);
          right_group->setStartState(*state);
          right_group->setPoseTarget(
            right_target, drawer_right_tool_.contact_tool_link);
          if (right_group->plan(right_plan) ==
            moveit::core::MoveItErrorCode::SUCCESS)
          {
            right_group->clearPoseTargets();
            right_planned = true;
          } else {
            right_group->clearPoseTargets();
          }
        }
        if (left_planned && right_planned) {
          break;
        }
      }
      if (left_planned && right_planned) {
        const auto execution = execute_bimanual_trajectories_bounded(
          left_group, left_plan.trajectory_,
          right_group, right_plan.trajectory_,
          std::function<bool()>(), std::chrono::seconds(60), description);
        ok = execution.left_code == moveit::core::MoveItErrorCode::SUCCESS &&
          execution.right_code == moveit::core::MoveItErrorCode::SUCCESS;
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        get_logger(), "Best-effort bimanual %s failed: %s",
        description.c_str(), error.what());
    }
    return ok;
  }

  void best_effort_bimanual_retreat_and_stow(
    const std::shared_ptr<MoveGroupInterface> & left_group,
    const std::shared_ptr<MoveGroupInterface> & right_group,
    const ButtonSpec & control,
    bool * operation_executed = nullptr) noexcept
  {
    if (!left_group || !right_group) {
      return;
    }
    try {
      const auto state = button_snapshot(control);
      const double position = state.received ?
        state.position : control.drawer_closed_position;
      best_effort_bimanual_pose_move(
        left_group, right_group,
        calculate_drawer_side_tool_pose(
          control, DrawerSide::LEFT, position, prepress_distance_),
        calculate_drawer_side_tool_pose(
          control, DrawerSide::RIGHT, position, prepress_distance_),
        "drawer retreat");
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        get_logger(), "Drawer best-effort retreat failed: %s", error.what());
    }
    best_effort_stow_named_target(
      *left_group, drawer_left_tool_.transport_named_target,
      operation_executed);
    best_effort_stow_named_target(
      *right_group, drawer_right_tool_.transport_named_target,
      operation_executed);
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
      RCLCPP_WARN(
        get_logger(),
        "The global operation lease service became unavailable "
        "during renewal.");
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
          RCLCPP_WARN(
            get_logger(),
            "The global operation lease renewal timed out "
            "after %.1fs.",
            operation_lease_request_timeout_);
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
        RCLCPP_WARN(
          get_logger(),
          "The global operation lease renewal was rejected: %s",
          response && !response->message.empty() ? response->message.c_str() :
          "(no message)");
        mark_operation_lease_lost(
          response && !response->message.empty() ? response->message :
          "The global operation lease renewal was rejected.");
      } else {
        // Reuse the successful global-lease renewal as the cabinet physics
        // liveness heartbeat.  Only a currently published non-empty control
        // is repeated; idle operators never arm the Gazebo watchdog.
        publish_operation_heartbeat();
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
    mark_operation_lease_lost_impl(reason, nullptr);
  }

  void mark_operation_lease_lost_for_exact_lease(
    const std::string & reason,
    const std::string & expected_lease_id) noexcept
  {
    if (expected_lease_id.empty()) {
      return;
    }
    mark_operation_lease_lost_impl(reason, &expected_lease_id);
  }

  void mark_operation_lease_lost_impl(
    const std::string & reason,
    const std::string * expected_lease_id) noexcept
  {
    {
      std::lock_guard<std::mutex> lock(operation_lease_mutex_);
      const bool lease_matches = expected_lease_id ?
        operation_fault_matches_active_lease(
        operation_lease_held_.load(), operation_lease_id_,
        *expected_lease_id) :
        operation_fault_matches_active_lease(
        operation_lease_held_.load(), operation_lease_id_,
        operation_lease_id_);
      if (!lease_matches || operation_lease_lost_.load()) {
        return;
      }
      // Latch the exact expected lease while holding the same mutex used by
      // release/acquire.  A delayed watchdog fault cannot pass validation for
      // an old lease and then race into canceling its successor.
      operation_lease_lost_.store(true);
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
        if (result) {
          if (request_success) {
            result->failure_reason.clear();
          } else {
            result->physical_outcome_confirmed = false;
            result->final_state_verified = false;
            result->transport_succeeded = false;
            result->recovery_succeeded = false;
            result->grasp_released = false;
            if (result->failure_reason.empty()) {
              result->failure_reason = result->message.empty() ?
                "Cabinet button operation failed without a diagnostic message." :
                result->message;
            }
          }
        }
        return apply_goal_terminal_disposition(
          goal_terminal_disposition(
            request_success, is_goal_canceling_noexcept(goal_handle)),
          [&]() {goal_handle->succeed(result);},
          [&]() {
            result->success = false;
            result->error_code = PressCabinetButton::Result::CANCELED;
            result->message = "Cabinet button operation was canceled.";
            result->failure_reason = result->message;
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
    std::array<std::shared_ptr<MoveGroupInterface>, 2> bimanual_groups{};
    {
      std::lock_guard<std::mutex> lock(motion_mutex_);
      move_group = active_move_group_;
      bimanual_groups = bimanual_active_move_groups_;
    }
    const auto stop_one = [this](const std::shared_ptr<MoveGroupInterface> & group) {
        if (!group) {
          return;
        }
        try {
          group->getMoveGroupClient().async_cancel_all_goals();
          group->stop();
        } catch (const std::exception & error) {
          RCLCPP_ERROR(
            get_logger(), "Failed to stop MoveIt: %s", error.what());
        }
      };
    stop_one(move_group);
    for (const auto & group : bimanual_groups) {
      stop_one(group);
    }
    // 双臂 drawer 直发控制器的目标也要撤销，否则楔死的 worker 永远等不到
    // 结果、execution 有界循环的 grace 窗口内无法回归。
    if (left_fjt_client_ && right_fjt_client_) {
      std::lock_guard<std::mutex> lock(bimanual_controller_handles_->mutex);
      if (bimanual_controller_handles_->left) {
        try {
          left_fjt_client_->async_cancel_goal(
            bimanual_controller_handles_->left);
        } catch (const std::exception &) {
        }
      }
      if (bimanual_controller_handles_->right) {
        try {
          right_fjt_client_->async_cancel_goal(
            bimanual_controller_handles_->right);
        } catch (const std::exception &) {
        }
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
    xczs_inspection_robot_interfaces::srv::SetCabinetGrasp>::SharedPtr
    grasp_client_;
  rclcpp::Client<
    xczs_inspection_robot_interfaces::srv::SetCabinetBimanualGrasp>::SharedPtr
    bimanual_grasp_client_;
  rclcpp::Client<
    xczs_inspection_robot_interfaces::srv::SetCabinetUnlock>::SharedPtr
    unlock_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_physics_client_;
  rclcpp::CallbackGroup::SharedPtr reset_client_callback_group_;
  rclcpp::CallbackGroup::SharedPtr operation_lease_client_callback_group_;
  rclcpp::CallbackGroup::SharedPtr reset_service_callback_group_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_controls_service_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
    manual_base_publisher_;
  rclcpp::Publisher<
    xczs_inspection_robot_interfaces::msg::CabinetControlCatalog>::SharedPtr
    control_catalog_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
    active_control_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
    operation_heartbeat_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
    operation_fault_subscription_;
  std::mutex operation_heartbeat_mutex_;
  std::string operation_heartbeat_control_id_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
    cabinet_pose_valid_subscription_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr>
  button_joint_state_subscriptions_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr>
  button_pressed_subscriptions_;
  std::vector<rclcpp::Subscription<
      xczs_inspection_robot_interfaces::msg::CabinetControlState>::SharedPtr>
  control_state_subscriptions_;
  std::shared_ptr<tf2_ros::Buffer> transform_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> transform_listener_;

  std::unordered_map<std::string, std::shared_ptr<ButtonSpec>> buttons_by_id_;
  std::vector<std::shared_ptr<ButtonSpec>> buttons_in_order_;
  std::mutex motion_mutex_;
  std::shared_ptr<MoveGroupInterface> active_move_group_;
  // Left/right move groups registered while a bimanual drawer motion is in
  // flight, so stop_active_motion() (and the wedge watchdog) cancel BOTH arms
  // instead of only the single active_move_group_.
  std::array<std::shared_ptr<MoveGroupInterface>, 2> bimanual_active_move_groups_{};
  // 双臂 drawer 轨迹直发控制器（绕过 move_group 串行 execute）。左/右各一个
  // action client，执行 worker 各自发送、各自等待结果，从而真正并行；活动
  // goal handle 供 stop_active_motion 在超时/取消时撤销两条臂的控制器目标。
  // handles 堆共享：detach 的楔死 worker 即使 operator 已销毁仍可安全登记/
  // 清理自己的 goal handle，绝不触碰已析构的 this。
  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr
    left_fjt_client_;
  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr
    right_fjt_client_;
  struct BimanualControllerHandles
  {
    std::mutex mutex;
    rclcpp_action::ClientGoalHandle<
      control_msgs::action::FollowJointTrajectory>::SharedPtr left;
    rclcpp_action::ClientGoalHandle<
      control_msgs::action::FollowJointTrajectory>::SharedPtr right;
  };
  std::shared_ptr<BimanualControllerHandles> bimanual_controller_handles_ =
    std::make_shared<BimanualControllerHandles>();
  // Shared serialization for MoveGroupInterface::execute() across bounded
  // motions (see execute_motion_bounded).  The mutexes are heap-shared so a
  // detached wedge worker can safely clear the transferred lock even if the
  // operator is destroyed; the execute serialization is what keeps a wedged
  // in-flight execute() from racing a later one on the same instance.
  struct MotionExecuteSerialization
  {
    std::mutex execute_mutex;
    std::mutex guard_mutex;
    std::optional<std::unique_lock<std::mutex>> wedged_lock;
  };
  std::shared_ptr<MotionExecuteSerialization> motion_execute_serial_ =
    std::make_shared<MotionExecuteSerialization>();
  std::mutex navigation_mutex_;
  NavigationGoalHandle::SharedPtr active_navigation_goal_;
  std::mutex navigation_feedback_mutex_;
  std::chrono::steady_clock::time_point last_navigation_feedback_{};
  std::mutex worker_mutex_;
  std::thread worker_thread_;
  std::mutex pending_goal_mutex_;
  std::unordered_map<std::string, PendingGoalDisposition>
    pending_goal_dispositions_;
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
  std::string controller_namespace_;
  std::string toolset_;
  std::string contact_tool_link_;
  std::string grasp_link_;
  tf2::Vector3 grasp_point_position_{0.0, 0.0, 0.0};
  std::string transport_named_target_;
  ToolProfile::ToolAxisOrientation tool_axis_orientation_{
    ToolProfile::ToolAxisOrientation::ALONG_OUTWARD};
  tf2::Vector3 tool_tip_position_{0.0, 0.0, 0.0};
  std::vector<std::string> tool_tip_calibration_joint_names_;
  std::vector<double> tool_tip_calibration_joint_positions_;
  double tool_tip_calibration_joint_tolerance_{0.001};
  double tool_calibration_settle_timeout_{6.0};
  std::unordered_map<std::uint8_t, ToolProfile> tool_profiles_;
  // Bimanual drawer tools: right three-cylinder (right_arm) + left
  // two-cylinder (left_arm), read from the adapter's drawer section.  Only
  // configured when at least one drawer control is operable.
  BimanualToolProfile drawer_left_tool_;
  BimanualToolProfile drawer_right_tool_;
  bool drawer_tools_configured_{false};
  std::string drawer_transport_named_target_;
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
  // P3-8 grab-and-drive drawer pull (base translation along the drawer axis).
  double drawer_base_drive_max_speed_{0.05};
  double drawer_base_drive_gain_{1.0};
  double drawer_base_drive_timeout_{60.0};
  double navigation_velocity_yaw_offset_{-1.57079632679};
  double press_detection_timeout_{3.0};
  double release_detection_timeout_{3.0};
  double press_hold_seconds_{0.5};
  double force_tracking_tolerance_{0.00005};
  double force_tracking_max_compensation_{0.0010};
  double force_tracking_settle_seconds_{0.15};
  int force_tracking_attempts_{3};
  double button_press_minimum_cartesian_fraction_{0.95};
  double target_tolerance_{0.035};
  double slider_position_tolerance_{0.03};
  double stable_velocity_tolerance_{0.03};
  double stable_state_duration_{0.30};
  double grasp_attach_settle_duration_{0.15};
  double grasp_release_settle_duration_{0.30};
  double door_release_fraction_{0.60};
  double door_settle_timeout_{90.0};
  double door_release_position_timeout_{10.0};
  double door_detent_hysteresis_{0.02};
  double door_release_position_margin_{0.01};
  double rotary_pregrasp_clearance_{0.010};
  double knob_pregrasp_clearance_{0.050};
  double door_release_clearance_{0.30};
  double planning_scene_settle_seconds_{0.50};
  double rotation_waypoint_step_{0.03490658504};
  // Linear spacing (m) of the drawer grasp-drag waypoints along the slide axis.
  double drawer_waypoint_step_{0.03};
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
