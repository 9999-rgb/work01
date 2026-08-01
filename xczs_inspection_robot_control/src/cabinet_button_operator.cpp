// Copyright 2026
// SPDX-License-Identifier: BSD-3-Clause

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <future>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
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
#include "std_srvs/srv/set_bool.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "xczs_inspection_robot_control/action/press_cabinet_button.hpp"

namespace xczs_inspection_robot_control
{

namespace
{

using namespace std::chrono_literals;
using PressCabinetButton =
  xczs_inspection_robot_control::action::PressCabinetButton;
using PressGoalHandle = rclcpp_action::ServerGoalHandle<PressCabinetButton>;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using NavigationGoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;
using MoveGroupInterface =
  moveit::planning_interface::MoveGroupInterface;

constexpr char kActionName[] = "/xczs/press_cabinet_button";
constexpr char kDefaultButtonId[] = "box_10_button_1";
constexpr char kArmGroup[] = "manipulator";
constexpr char kArmTipLink[] = "end";

class OperationError : public std::runtime_error
{
public:
  OperationError(std::uint8_t error_code, const std::string & message)
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
    supported_button_id_ = declare_parameter<std::string>(
      "button_id", kDefaultButtonId);
    planning_frame_ = declare_parameter<std::string>(
      "planning_frame", "odom");
    navigation_frame_ = declare_parameter<std::string>(
      "navigation_frame", "map");
    button_joint_name_ = declare_parameter<std::string>(
      "button_joint_name", "box_10_box_10_button_1");
    const auto button_joint_state_topic = declare_parameter<std::string>(
      "button_joint_state_topic",
      "/xczs/cabinet/box_10_button_1/joint_states");
    const auto button_pressed_topic = declare_parameter<std::string>(
      "button_pressed_topic",
      "/xczs/cabinet/box_10_button_1/pressed");
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

    button_local_x_ = declare_parameter<double>("button_local_x", 0.492);
    button_local_y_ = declare_parameter<double>("button_local_y", 0.574);
    button_local_z_ = declare_parameter<double>(
      "button_local_z", 0.011494);
    tool_tip_offset_ = positive_parameter("tool_tip_offset", 0.370);
    staging_distance_ = positive_parameter("staging_distance", 1.040);
    prepress_distance_ = positive_parameter("prepress_distance", 0.060);
    contact_clearance_ = positive_parameter("contact_clearance", 0.001);
    press_depth_ = positive_parameter("press_depth", 0.007);
    button_state_timeout_ = positive_parameter(
      "button_state_timeout", 1.0);
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
    cartesian_velocity_scale_ = unit_interval_parameter(
      "cartesian_velocity_scale", 0.08);
    cartesian_acceleration_scale_ = unit_interval_parameter(
      "cartesian_acceleration_scale", 0.08);

    button_joint_state_subscription_ =
      create_subscription<sensor_msgs::msg::JointState>(
      button_joint_state_topic,
      10,
      [this](const sensor_msgs::msg::JointState::SharedPtr message) {
        receive_button_joint_state(*message);
      });
    button_pressed_subscription_ = create_subscription<std_msgs::msg::Bool>(
      button_pressed_topic,
      rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::Bool::SharedPtr message) {
        receive_button_pressed(*message);
      });

    navigation_client_ = rclcpp_action::create_client<NavigateToPose>(
      this, "/navigate_to_pose");
    navigation_mode_client_ = create_client<std_srvs::srv::SetBool>(
      "/xczs/set_navigation_mode");
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

    RCLCPP_INFO(
      get_logger(),
      "Cabinet button action %s controls %s.",
      kActionName, supported_button_id_.c_str());
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
    if (goal->button_id != supported_button_id_) {
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
    std::shared_ptr<MoveGroupInterface> move_group;
    OperationPoses poses;
    bool should_attempt_retreat = false;

    try {
      reset_max_button_travel();
      publish_feedback(
        goal_handle,
        PressCabinetButton::Feedback::WAITING_FOR_SYSTEM,
        0.02F,
        "Waiting for the cabinet button state.");
      wait_for_fresh_button_state(goal_handle);
      check_cancel(goal_handle);

      const bool should_navigate_to_staging_pose =
        goal_handle->get_goal()->navigate_to_staging_pose;
      poses = calculate_operation_poses(should_navigate_to_staging_pose);
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
        "Pressing box_10_button_1 through its physical travel.");
      const auto press_transition_sequence =
        button_snapshot().pressed_transition_sequence;
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
        button_snapshot().pressed_transition_sequence;
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
        "Pressed and released box_10_button_1 successfully.";
      result->max_travel = button_snapshot().max_position;
      if (finish_goal_noexcept(goal_handle, result, true)) {
        RCLCPP_INFO(
          get_logger(), "Cabinet button operation succeeded (max %.2f mm).",
          result->max_travel * 1000.0);
      }
    } catch (const OperationError & error) {
      if (should_attempt_retreat && move_group && rclcpp::ok()) {
        best_effort_retreat(*move_group, poses.prepress_pose);
      }
      result->success = false;
      result->error_code = error.error_code;
      result->message = error.what();
      result->max_travel = button_snapshot().max_position;
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
      result->max_travel = button_snapshot().max_position;
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
      result->max_travel = button_snapshot().max_position;
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
  }

  struct ButtonSnapshot
  {
    bool received{false};
    bool pressed_received{false};
    bool pressed{false};
    std::uint64_t pressed_transition_sequence{0};
    double position{0.0};
    double max_position{0.0};
    std::chrono::steady_clock::time_point received_at{};
    std::chrono::steady_clock::time_point pressed_received_at{};
  };

  void receive_button_joint_state(
    const sensor_msgs::msg::JointState & message)
  {
    const auto iterator = std::find(
      message.name.begin(), message.name.end(), button_joint_name_);
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
      std::lock_guard<std::mutex> lock(button_mutex_);
      button_state_.received = true;
      button_state_.position = message.position[index];
      button_state_.max_position = std::max(
        button_state_.max_position, button_state_.position);
      button_state_.received_at = std::chrono::steady_clock::now();
    }
    button_condition_.notify_all();
  }

  void receive_button_pressed(const std_msgs::msg::Bool & message)
  {
    {
      std::lock_guard<std::mutex> lock(button_mutex_);
      if (!button_state_.pressed_received ||
        button_state_.pressed != message.data)
      {
        ++button_state_.pressed_transition_sequence;
      }
      button_state_.pressed_received = true;
      button_state_.pressed = message.data;
      button_state_.pressed_received_at = std::chrono::steady_clock::now();
    }
    button_condition_.notify_all();
  }

  ButtonSnapshot button_snapshot() const
  {
    std::lock_guard<std::mutex> lock(button_mutex_);
    return button_state_;
  }

  void reset_max_button_travel()
  {
    std::lock_guard<std::mutex> lock(button_mutex_);
    button_state_.max_position = std::max(0.0, button_state_.position);
  }

  void wait_for_fresh_button_state(
    const std::shared_ptr<PressGoalHandle> & goal_handle)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(system_wait_timeout_);
    while (std::chrono::steady_clock::now() < deadline) {
      check_cancel(goal_handle);
      {
        std::unique_lock<std::mutex> lock(button_mutex_);
        const auto now = std::chrono::steady_clock::now();
        const bool joint_state_fresh = button_state_.received &&
          std::chrono::duration<double>(
          now - button_state_.received_at).count() <=
          button_state_timeout_;
        const bool pressed_state_fresh = button_state_.pressed_received &&
          std::chrono::duration<double>(
          now - button_state_.pressed_received_at).count() <=
          button_state_timeout_;
        if (joint_state_fresh && pressed_state_fresh) {
          if (button_state_.pressed) {
            throw OperationError(
                    PressCabinetButton::Result::NOT_READY,
                    "box_10_button_1 is already pressed; release it before "
                    "starting an operation.");
          }
          return;
        }
        button_condition_.wait_for(lock, 100ms);
      }
    }
    throw OperationError(
            PressCabinetButton::Result::NOT_READY,
            "No fresh state was received from box_10_button_1. Check the "
            "cabinet model plugin and joint-state topic.");
  }

  bool wait_for_pressed_state(
    const std::shared_ptr<PressGoalHandle> & goal_handle,
    bool expected_pressed,
    double timeout_seconds,
    std::uint64_t previous_transition_sequence)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(timeout_seconds);
    while (std::chrono::steady_clock::now() < deadline) {
      check_cancel(goal_handle);
      {
        std::unique_lock<std::mutex> lock(button_mutex_);
        if (button_state_.received &&
          button_state_.pressed == expected_pressed &&
          button_state_.pressed_transition_sequence >
          previous_transition_sequence)
        {
          return true;
        }
        button_condition_.wait_for(lock, 50ms);
      }
    }
    return false;
  }

  OperationPoses calculate_operation_poses(bool include_staging_pose)
  {
    tf2::Quaternion cabinet_rotation;
    cabinet_rotation.setRPY(
      cabinet_roll_, cabinet_pitch_, cabinet_yaw_);
    cabinet_rotation.normalize();
    const tf2::Transform cabinet_transform(
      cabinet_rotation,
      tf2::Vector3(cabinet_x_, cabinet_y_, cabinet_z_));
    const tf2::Vector3 button_face = cabinet_transform * tf2::Vector3(
      button_local_x_, button_local_y_, button_local_z_);
    tf2::Vector3 inward = tf2::quatRotate(
      cabinet_rotation, tf2::Vector3(0.0, 0.0, -1.0));
    inward.normalize();
    const tf2::Vector3 outward = -inward;

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

  void plan_and_execute_pose(
    MoveGroupInterface & move_group,
    const std::shared_ptr<PressGoalHandle> & goal_handle,
    const geometry_msgs::msg::Pose & target)
  {
    check_cancel(goal_handle);
    move_group.setStartStateToCurrentState();
    if (!move_group.setPoseTarget(target, kArmTipLink)) {
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

  void execute_cartesian_path(
    MoveGroupInterface & move_group,
    const std::shared_ptr<PressGoalHandle> & goal_handle,
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

  void navigate_to_staging_pose(
    const std::shared_ptr<PressGoalHandle> & goal_handle,
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
      [this, weak_goal = std::weak_ptr<PressGoalHandle>(goal_handle),
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

  void dock_to_staging_pose(
    const std::shared_ptr<PressGoalHandle> & goal_handle,
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

  void set_navigation_mode(
    const std::shared_ptr<PressGoalHandle> & goal_handle,
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

  template<typename FutureT>
  void wait_for_future(
    FutureT & future,
    const std::shared_ptr<PressGoalHandle> & goal_handle,
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

  void interruptible_hold(
    const std::shared_ptr<PressGoalHandle> & goal_handle,
    double seconds)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(seconds);
    while (std::chrono::steady_clock::now() < deadline) {
      check_cancel(goal_handle);
      std::this_thread::sleep_for(20ms);
    }
  }

  void check_cancel(
    const std::shared_ptr<PressGoalHandle> & goal_handle) const
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
    const auto state = button_snapshot();
    auto feedback = std::make_shared<PressCabinetButton::Feedback>();
    feedback->phase = phase;
    feedback->progress = progress;
    feedback->button_travel = state.position;
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
  rclcpp_action::Client<NavigateToPose>::SharedPtr navigation_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr navigation_mode_client_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
    manual_base_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
    button_joint_state_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
    button_pressed_subscription_;
  std::unique_ptr<tf2_ros::Buffer> transform_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> transform_listener_;

  mutable std::mutex button_mutex_;
  std::condition_variable button_condition_;
  ButtonSnapshot button_state_;
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

  std::string supported_button_id_;
  std::string planning_frame_;
  std::string navigation_frame_;
  std::string button_joint_name_;
  double cabinet_x_{2.0};
  double cabinet_y_{0.33};
  double cabinet_z_{0.0};
  double cabinet_roll_{1.57079632679};
  double cabinet_pitch_{0.0};
  double cabinet_yaw_{-1.57079632679};
  double button_local_x_{0.492};
  double button_local_y_{0.574};
  double button_local_z_{0.011494};
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
