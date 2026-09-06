// Copyright 2026
// SPDX-License-Identifier: BSD-3-Clause

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/ode/ODECollision.hh>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <future>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "xczs_inspection_robot_control/cabinet_grasp_safety_policy.hpp"
#include "xczs_inspection_robot_interfaces/msg/cabinet_control.hpp"
#include "xczs_inspection_robot_interfaces/msg/cabinet_control_state.hpp"
#include "xczs_inspection_robot_interfaces/srv/set_cabinet_grasp.hpp"
#include "xczs_inspection_robot_interfaces/srv/set_cabinet_bimanual_grasp.hpp"
#include "xczs_inspection_robot_interfaces/srv/set_cabinet_playback.hpp"
#include "xczs_inspection_robot_interfaces/srv/set_cabinet_unlock.hpp"

namespace xczs_inspection_robot_control
{

namespace
{

using CabinetControl =
  xczs_inspection_robot_interfaces::msg::CabinetControl;
using CabinetControlState =
  xczs_inspection_robot_interfaces::msg::CabinetControlState;
using SetCabinetGrasp =
  xczs_inspection_robot_interfaces::srv::SetCabinetGrasp;
using SetCabinetBimanualGrasp =
  xczs_inspection_robot_interfaces::srv::SetCabinetBimanualGrasp;
using SetCabinetUnlock =
  xczs_inspection_robot_interfaces::srv::SetCabinetUnlock;
using SetCabinetPlayback =
  xczs_inspection_robot_interfaces::srv::SetCabinetPlayback;

constexpr auto kPhysicsRequestTimeout = std::chrono::seconds(2);
constexpr auto kWatchdogReleaseRetryDelay = std::chrono::milliseconds(100);
constexpr auto kWatchdogReleaseLogPeriod = std::chrono::seconds(1);
constexpr std::size_t kExpiredLeaseHistoryLimit = 64U;
constexpr char kRuntimeGraspJointName[] = "xczs_cabinet_runtime_grasp";
constexpr char kRuntimeBaseBrakeJointName[] =
  "xczs_cabinet_runtime_base_brake";
// A bimanual drawer attaches the left tool to the left handle and the right
// tool to the right handle, so two runtime fixed joints plus the shared base
// brake are created together (both or neither).
constexpr char kRuntimeBimanualLeftJointName[] =
  "xczs_cabinet_runtime_bimanual_left";
constexpr char kRuntimeBimanualRightJointName[] =
  "xczs_cabinet_runtime_bimanual_right";
// While a drawer is latched (locked) it is parked at the closed detent by a
// firm centering spring so it cannot drift out of the rail gate.
constexpr double kDefaultDrawerLatchStiffness = 500.0;
// Re-latch only when the drawer has settled below this linear speed so a
// closing push that is still in flight cannot trip the lock early.
constexpr double kDrawerReLatchVelocityThreshold = 0.05;
// Upper bound on the restoring (parking) effort applied to a control that is
// not being grasped.  The restoring spring is stiffness*position_error, which
// grows without bound once a joint overshoots its stop; when that force
// exceeds the joint limit's effort (100 N) the ODE stop can no longer hold
// and the joint runs away (b1p cap was blown to ~1e9 m, cascading NaN to
// every body).  Clamping the restoring effort below the limit effort lets the
// stop always win.  The value covers every legit parking load: button spring
// 800 N/m * 3 mm = 2.4 N, door/slider detent 12 N/m * 0.3 m = 3.6 N, locked
// drawer latch 500 N/m * 0.1 m = 50 N.  Grasp-coupling drive is separate and
// already bounded by grasp_coupling_max_effort.
constexpr double kMaxParkingEffort = 50.0;
// Visual playback (2026-09-06): how far the first trajectory sample may sit
// from the drawer's current rail position before START is rejected as a jump.
constexpr double kPlaybackStartWindow = 0.03;
constexpr std::size_t kPlaybackMaxSamples = 4096;

enum class ControlKind
{
  kButton,
  kKnob,
  kSwitch,
  kDoor,
  kSlider,
  kDrawer,
};

std::string required_text(
  const sdf::ElementPtr & element,
  const std::string & name)
{
  if (!element->HasElement(name)) {
    throw std::invalid_argument(
            "Cabinet control is missing required <" + name + ">.");
  }
  const auto value = element->Get<std::string>(name);
  if (value.empty()) {
    throw std::invalid_argument(
            "Cabinet control <" + name + "> must not be empty.");
  }
  return value;
}

double optional_double(
  const sdf::ElementPtr & element,
  const std::string & name,
  double default_value)
{
  const double value = element->Get<double>(name, default_value).first;
  if (!std::isfinite(value)) {
    throw std::invalid_argument(
            "Cabinet control <" + name + "> must be finite.");
  }
  return value;
}

bool optional_bool(
  const sdf::ElementPtr & element,
  const std::string & name,
  bool default_value)
{
  return element->Get<bool>(name, default_value).first;
}

std::vector<double> parse_doubles(const std::string & text)
{
  std::istringstream stream(text);
  std::vector<double> values;
  double value = 0.0;
  while (stream >> value) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument("Cabinet control numeric list is not finite.");
    }
    values.push_back(value);
  }
  if (!stream.eof()) {
    throw std::invalid_argument(
            "Cabinet control numeric list contains invalid text: " + text);
  }
  return values;
}

std::vector<std::string> parse_words(const std::string & text)
{
  std::istringstream stream(text);
  std::vector<std::string> values;
  std::string value;
  while (stream >> value) {
    values.push_back(value);
  }
  return values;
}

ignition::math::Vector3d parse_vector3(const std::string & text)
{
  const auto values = parse_doubles(text);
  if (values.size() != 3U) {
    throw std::invalid_argument(
            "Cabinet control grasp_point must contain exactly 3 values.");
  }
  return {values[0], values[1], values[2]};
}

void require_absolute_topic(
  const std::string & topic,
  const std::string & field)
{
  if (topic.empty() || topic.front() != '/') {
    throw std::invalid_argument(
            "Cabinet control " + field + " must be an absolute ROS name.");
  }
}

ControlKind parse_kind(const std::string & value)
{
  if (value == "button") {
    return ControlKind::kButton;
  }
  if (value == "knob") {
    return ControlKind::kKnob;
  }
  if (value == "switch") {
    return ControlKind::kSwitch;
  }
  if (value == "door") {
    return ControlKind::kDoor;
  }
  if (value == "slider") {
    return ControlKind::kSlider;
  }
  if (value == "drawer") {
    return ControlKind::kDrawer;
  }
  throw std::invalid_argument(
          "Unsupported cabinet control type: " + value);
}

std::uint8_t message_type(ControlKind kind)
{
  switch (kind) {
    case ControlKind::kButton:
      return CabinetControl::TYPE_BUTTON;
    case ControlKind::kKnob:
      return CabinetControl::TYPE_KNOB;
    case ControlKind::kSwitch:
      return CabinetControl::TYPE_SWITCH;
    case ControlKind::kDoor:
      return CabinetControl::TYPE_DOOR;
    case ControlKind::kSlider:
      return CabinetControl::TYPE_SLIDER;
    case ControlKind::kDrawer:
      return CabinetControl::TYPE_DRAWER;
  }
  return CabinetControl::TYPE_BUTTON;
}

std::size_t nearest_index(
  const std::vector<double> & positions,
  double position)
{
  std::size_t nearest = 0U;
  double nearest_distance = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index < positions.size(); ++index) {
    const double distance = std::abs(position - positions[index]);
    if (distance < nearest_distance) {
      nearest = index;
      nearest_distance = distance;
    }
  }
  return nearest;
}

}  // namespace

class CabinetStatePlugin final : public gazebo::ModelPlugin
{
public:
  ~CabinetStatePlugin() override
  {
    // Stop the node spin thread before tearing down its publishers/services so
    // no executor callback can fire into a half-destroyed plugin.
    stop_spin_thread();
    const auto callback_lifetime = callback_lifetime_;
    if (callback_lifetime) {
      std::lock_guard<std::mutex> lock(callback_lifetime->mutex);
      callback_lifetime->shutting_down = true;
    }

    std::deque<std::shared_ptr<PhysicsRequest>> pending_requests;
    {
      std::lock_guard<std::mutex> lock(request_mutex_);
      shutting_down_.store(true);
      pending_requests.swap(requests_);
    }
    reset_service_.reset();
    grasp_service_.reset();
    bimanual_grasp_service_.reset();
    unlock_service_.reset();
    active_control_subscription_.reset();
    operation_heartbeat_subscription_.reset();
    update_connection_.reset();
    fail_requests(
      pending_requests, "Cabinet state plugin is shutting down.");

    // Disconnecting the event prevents new updates; this lock waits for an
    // already-running physics callback before touching Gazebo objects.
    {
      std::lock_guard<std::mutex> physics_lock(physics_callback_mutex_);
      const auto release = release_grasp_constraint();
      if (!release.success && ros_node_) {
        RCLCPP_ERROR(ros_node_->get_logger(), "%s", release.message.c_str());
      }
      restore_all_actuation_collisions();
    }
    // on_update may publish a watchdog fault.  Reset its publisher only after
    // disconnecting and joining the physics callback through the mutex above.
    operation_fault_publisher_.reset();

    if (callback_lifetime) {
      std::unique_lock<std::mutex> lock(callback_lifetime->mutex);
      callback_lifetime->condition.wait(
        lock, [&callback_lifetime]() {
          return callback_lifetime->active_callbacks == 0U;
        });
      callback_lifetime->owner = nullptr;
    }
  }

  void Load(
    gazebo::physics::ModelPtr model,
    sdf::ElementPtr sdf) override
  {
    model_ = std::move(model);
    world_ = model_->GetWorld();
    ros_node_ = create_ros_node(sdf);
    if (!ros_node_) {
      gzerr << "Cabinet state plugin could not create a ROS 2 node.\n";
      return;
    }
    start_spin_thread();

    try {
      const double publish_rate = optional_double(
        sdf, "state_publish_rate", 20.0);
      if (publish_rate <= 0.0) {
        throw std::invalid_argument(
                "Cabinet state_publish_rate must be positive.");
      }
      publish_period_ = 1.0 / publish_rate;
      grasp_distance_threshold_ = optional_double(
        sdf, "grasp_distance_threshold", 0.12);
      if (grasp_distance_threshold_ <= 0.0) {
        throw std::invalid_argument(
                "Cabinet grasp_distance_threshold must be positive.");
      }
      operation_watchdog_timeout_ = optional_double(
        sdf, "operation_watchdog_timeout", 2.0);
      if (!operation_watchdog_timeout_is_valid(operation_watchdog_timeout_)) {
        throw std::invalid_argument(
                "Cabinet operation_watchdog_timeout must be positive.");
      }
      reset_service_name_ = sdf->Get<std::string>(
        "reset_service", "/xczs/cabinet/reset_physics").first;
      grasp_service_name_ = sdf->Get<std::string>(
        "grasp_service", "/xczs/cabinet/grasp").first;
      bimanual_grasp_service_name_ = sdf->Get<std::string>(
        "bimanual_grasp_service", "/xczs/cabinet/bimanual_grasp").first;
      unlock_service_name_ = sdf->Get<std::string>(
        "unlock_service", "/xczs/cabinet/unlock").first;
      // Visual playback is a per-instance drawer service: when the SDF does not
      // pin one, derive it from the instance's grasp service (same namespace,
      // "/playback" leaf) so multiple cabinet plugin instances in one world can
      // never collide on a shared default name.
      if (sdf->HasElement("playback_service")) {
        playback_service_name_ = sdf->Get<std::string>("playback_service");
      } else {
        const auto slash = grasp_service_name_.find_last_of('/');
        playback_service_name_ = (slash == std::string::npos) ?
          "/xczs/cabinet/playback" :
          grasp_service_name_.substr(0, slash) + "/playback";
      }
      grasp_active_topic_ = sdf->Get<std::string>(
        "grasp_active_topic", "/xczs/cabinet/grasp_active").first;
      active_control_topic_ = sdf->Get<std::string>(
        "active_control_topic", "/xczs/cabinet/active_control").first;
      operation_heartbeat_topic_ = sdf->Get<std::string>(
        "operation_heartbeat_topic",
        "/xczs/cabinet/operation_heartbeat").first;
      operation_fault_topic_ = sdf->Get<std::string>(
        "operation_fault_topic",
        "/xczs/cabinet/operation_fault").first;
      require_absolute_topic(reset_service_name_, "reset_service");
      require_absolute_topic(grasp_service_name_, "grasp_service");
      require_absolute_topic(
        bimanual_grasp_service_name_, "bimanual_grasp_service");
      require_absolute_topic(unlock_service_name_, "unlock_service");
      require_absolute_topic(playback_service_name_, "playback_service");
      require_absolute_topic(grasp_active_topic_, "grasp_active_topic");
      require_absolute_topic(active_control_topic_, "active_control_topic");
      require_absolute_topic(
        operation_heartbeat_topic_, "operation_heartbeat_topic");
      require_absolute_topic(
        operation_fault_topic_, "operation_fault_topic");
      configure_controls(sdf);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(ros_node_->get_logger(), "%s", error.what());
      controls_.clear();
      return;
    }

    callback_lifetime_ = std::make_shared<ServiceCallbackLifetime>();
    callback_lifetime_->owner = this;
    const auto callback_lifetime = callback_lifetime_;
    grasp_active_publisher_ =
      ros_node_->create_publisher<std_msgs::msg::Bool>(
      grasp_active_topic_, rclcpp::QoS(1).reliable().transient_local());
    publish_grasp_active(false);
    operation_fault_publisher_ =
      ros_node_->create_publisher<std_msgs::msg::String>(
      operation_fault_topic_, rclcpp::QoS(1).reliable());
    active_control_subscription_ =
      ros_node_->create_subscription<std_msgs::msg::String>(
      active_control_topic_, rclcpp::QoS(1).reliable().transient_local(),
      [callback_lifetime](const std_msgs::msg::String::SharedPtr message) {
        auto * owner = acquire_service_callback(callback_lifetime);
        if (!owner) {
          return;
        }
        ServiceCallbackLease lease(callback_lifetime);
        owner->receive_active_control(message->data);
      });
    operation_heartbeat_subscription_ =
      ros_node_->create_subscription<std_msgs::msg::String>(
      operation_heartbeat_topic_, rclcpp::QoS(1).reliable(),
      [callback_lifetime](const std_msgs::msg::String::SharedPtr message) {
        auto * owner = acquire_service_callback(callback_lifetime);
        if (!owner) {
          return;
        }
        ServiceCallbackLease lease(callback_lifetime);
        owner->receive_operation_heartbeat(message->data);
      });
    reset_service_ = ros_node_->create_service<std_srvs::srv::Trigger>(
      reset_service_name_,
      [callback_lifetime](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
      {
        auto * owner = acquire_service_callback(callback_lifetime);
        if (!owner) {
          response->success = false;
          response->message = "Cabinet state plugin is shutting down.";
          return;
        }
        ServiceCallbackLease lease(callback_lifetime);
        const auto outcome = owner->submit_physics_request(
          PhysicsRequest::Kind::kReset, {}, {}, {}, {}, {}, {}, false);
        response->success = outcome.success;
        response->message = outcome.message;
      });
    grasp_service_ = ros_node_->create_service<SetCabinetGrasp>(
      grasp_service_name_,
      [callback_lifetime](
        const std::shared_ptr<SetCabinetGrasp::Request> request,
        std::shared_ptr<SetCabinetGrasp::Response> response)
      {
        auto * owner = acquire_service_callback(callback_lifetime);
        if (!owner) {
          response->success = false;
          response->message = "Cabinet state plugin is shutting down.";
          response->distance =
          std::numeric_limits<double>::quiet_NaN();
          return;
        }
        ServiceCallbackLease lease(callback_lifetime);
        const auto outcome = owner->submit_physics_request(
          PhysicsRequest::Kind::kGrasp,
          request->control_id,
          request->operation_lease_id,
          request->robot_model,
          request->robot_link,
          ignition::math::Vector3d(
            request->robot_grasp_point.x,
            request->robot_grasp_point.y,
            request->robot_grasp_point.z),
          request->robot_base_link,
          request->attach);
        response->success = outcome.success;
        response->message = outcome.message;
        response->distance = outcome.distance;
      });
    bimanual_grasp_service_ = ros_node_->create_service<SetCabinetBimanualGrasp>(
      bimanual_grasp_service_name_,
      [callback_lifetime](
        const std::shared_ptr<SetCabinetBimanualGrasp::Request> request,
        std::shared_ptr<SetCabinetBimanualGrasp::Response> response)
      {
        auto * owner = acquire_service_callback(callback_lifetime);
        if (!owner) {
          response->success = false;
          response->message = "Cabinet state plugin is shutting down.";
          response->left_distance =
            std::numeric_limits<double>::quiet_NaN();
          response->right_distance =
            std::numeric_limits<double>::quiet_NaN();
          response->left_tool_contact = false;
          response->right_tool_contact = false;
          return;
        }
        ServiceCallbackLease lease(callback_lifetime);
        const auto outcome = owner->submit_bimanual_request(
          request->control_id,
          request->operation_lease_id,
          request->robot_model,
          request->left_robot_link,
          request->right_robot_link,
          ignition::math::Vector3d(
            request->left_robot_grasp_point.x,
            request->left_robot_grasp_point.y,
            request->left_robot_grasp_point.z),
          ignition::math::Vector3d(
            request->right_robot_grasp_point.x,
            request->right_robot_grasp_point.y,
            request->right_robot_grasp_point.z),
          request->robot_base_link,
          request->attach,
          request->base_free);
        response->success = outcome.success;
        response->message = outcome.message;
        response->left_grasped = !std::isnan(outcome.left_distance);
        response->right_grasped = !std::isnan(outcome.right_distance);
        response->left_distance = outcome.left_distance;
        response->right_distance = outcome.right_distance;
        response->left_tool_contact = outcome.left_tool_contact;
        response->right_tool_contact = outcome.right_tool_contact;
      });
    unlock_service_ = ros_node_->create_service<SetCabinetUnlock>(
      unlock_service_name_,
      [callback_lifetime](
        const std::shared_ptr<SetCabinetUnlock::Request> request,
        std::shared_ptr<SetCabinetUnlock::Response> response)
      {
        auto * owner = acquire_service_callback(callback_lifetime);
        if (!owner) {
          response->success = false;
          response->message = "Cabinet state plugin is shutting down.";
          response->distance = std::numeric_limits<double>::quiet_NaN();
          response->pressed = false;
          response->right_tool_contact = false;
          return;
        }
        ServiceCallbackLease lease(callback_lifetime);
        const auto outcome = owner->submit_unlock_request(
          request->control_id,
          request->operation_lease_id,
          request->robot_model,
          request->right_robot_link,
          ignition::math::Vector3d(
            request->right_robot_grasp_point.x,
            request->right_robot_grasp_point.y,
            request->right_robot_grasp_point.z),
          request->unlock);
        response->success = outcome.success;
        response->message = outcome.message;
        response->distance = outcome.distance;
        response->drawer_unlocked = outcome.success &&
          request->unlock;
        response->pressed = outcome.pressed;
        response->right_tool_contact = outcome.right_tool_contact;
        // 2026-09-06 AGENT doc §4.2: mode + simulated-linkage evidence.
        response->unlock_mode = outcome.unlock_mode;
        response->simulation_acceptance = outcome.simulation_acceptance;
        response->left_hold_distance = outcome.left_hold_distance;
        response->right_hold_distance = outcome.right_hold_distance;
        response->left_handle_held = outcome.left_handle_held;
        response->right_handle_held = outcome.right_handle_held;
      });
    drawer_controls_present_ = false;
    for (const auto & control : controls_) {
      if (control.kind == ControlKind::kDrawer) {
        drawer_controls_present_ = true;
        break;
      }
    }
    if (drawer_controls_present_) {
      playback_service_ = ros_node_->create_service<SetCabinetPlayback>(
        playback_service_name_,
        [callback_lifetime](
          const std::shared_ptr<SetCabinetPlayback::Request> request,
          std::shared_ptr<SetCabinetPlayback::Response> response)
        {
          auto * owner = acquire_service_callback(callback_lifetime);
          if (!owner) {
            response->success = false;
            response->message = "Cabinet state plugin is shutting down.";
            response->position = std::numeric_limits<double>::quiet_NaN();
            response->finished = false;
            return;
          }
          ServiceCallbackLease lease(callback_lifetime);
          auto physics_request = std::make_shared<PhysicsRequest>();
          physics_request->kind = PhysicsRequest::Kind::kPlayback;
          physics_request->control_id = request->control_id;
          physics_request->operation_lease_id = request->operation_lease_id;
          physics_request->playback_command = request->command;
          physics_request->playback_samples.reserve(
            request->trajectory.points.size());
          for (const auto & point : request->trajectory.points) {
            const double stamp =
              rclcpp::Duration(point.time_from_start).seconds();
            const double value = point.positions.empty()
              ? std::numeric_limits<double>::quiet_NaN()
              : point.positions.front();
            physics_request->playback_samples.emplace_back(stamp, value);
          }
          const auto outcome = owner->run_physics_request(physics_request);
          response->success = outcome.success;
          response->message = outcome.message;
          response->position = outcome.distance;
          response->finished = outcome.finished;
        });
    }

    const double simulation_time = world_->SimTime().Double();
    last_publish_time_ = simulation_time - publish_period_;
    for (auto & control : controls_) {
      update_control_state(control, false);
      publish_control(control);
    }
    last_publish_time_ = simulation_time;

    update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      [callback_lifetime](const gazebo::common::UpdateInfo &) {
        auto * owner = acquire_service_callback(callback_lifetime);
        if (!owner) {
          return;
        }
        ServiceCallbackLease lease(callback_lifetime);
        owner->on_update();
      });
    if (drawer_controls_present_) {
      RCLCPP_INFO(
        ros_node_->get_logger(),
        "Cabinet state plugin loaded %zu controls; reset=%s grasp=%s "
        "bimanual=%s unlock=%s playback=%s.",
        controls_.size(), reset_service_name_.c_str(),
        grasp_service_name_.c_str(), bimanual_grasp_service_name_.c_str(),
        unlock_service_name_.c_str(), playback_service_name_.c_str());
    } else {
      RCLCPP_INFO(
        ros_node_->get_logger(),
        "Cabinet state plugin loaded %zu controls; reset=%s grasp=%s "
        "bimanual=%s unlock=%s (no drawer controls, no playback service).",
        controls_.size(), reset_service_name_.c_str(),
        grasp_service_name_.c_str(), bimanual_grasp_service_name_.c_str(),
        unlock_service_name_.c_str());
    }
  }

  void Reset() override
  {
    if (controls_.empty()) {
      return;
    }
    std::lock_guard<std::mutex> physics_lock(physics_callback_mutex_);
    const auto outcome = reset_controls();
    if (!outcome.success && ros_node_) {
      RCLCPP_ERROR(ros_node_->get_logger(), "%s", outcome.message.c_str());
    }
    last_publish_time_ = world_->SimTime().Double();
  }

private:
  struct ActuationCollision
  {
    gazebo::physics::CollisionPtr collision;
    unsigned int category_bits{GZ_ALL_COLLIDE};
    unsigned int collide_bits{GZ_ALL_COLLIDE};
  };

  struct Control
  {
    std::string id;
    ControlKind kind{ControlKind::kButton};
    std::uint8_t type{CabinetControl::TYPE_BUTTON};
    std::string joint_name;
    gazebo::physics::JointPtr joint;
    gazebo::physics::LinkPtr link;
    std::string joint_state_topic;
    std::string pressed_topic;
    std::string state_topic;
    std::vector<double> detents;
    std::vector<std::string> state_ids;
    double stiffness{0.0};
    double grasp_stiffness{0.0};
    double damping{0.0};
    double grasp_damping{0.0};
    double grasp_coupling_stiffness{0.0};
    double grasp_coupling_damping{0.0};
    double grasp_coupling_max_effort{0.0};
    std::vector<ActuationCollision> actuation_collisions;
    double actuation_collision_restore_delay{1.5};
    double actuation_collision_restore_distance{0.5};
    double collision_restore_not_before{0.0};
    gazebo::physics::LinkPtr collision_restore_robot_link;
    ignition::math::Vector3d collision_restore_robot_grasp_point{
      0.0, 0.0, 0.0};
    bool actuation_collision_suppressed{false};
    std::string actuation_collision_operation_lease_id;
    double detent_hysteresis{0.0};
    double press_threshold{0.006};
    double release_threshold{0.003};
    double motion_tolerance{0.025};
    double reset_position{0.0};
    // Push-push release for a slider: pushing the panel PAST the open detent
    // while it is latched trips the latch, and the detent spring then returns
    // the panel to the closed detent.  Disabled (infinity) unless configured.
    double slider_release_position{
      std::numeric_limits<double>::infinity()};
    // The tripped latch stays tripped while the panel falls back, re-arming the
    // bistable ratchet only once the panel is below this position.  Without
    // this, the closing travel would cross the midpoint and re-latch open.
    double slider_release_reengage_position{0.0};
    bool slider_released{false};
    bool graspable{false};
    ignition::math::Vector3d grasp_point{0.0, 0.0, 0.0};
    // Bimanual drawer geometry, all in the local frame of the prismatic drawer
    // link.  The left handle reuses grasp_point; the right handle and the
    // unlock press point are drawer-only fields.
    ignition::math::Vector3d right_grasp_point{0.0, 0.0, 0.0};
    // 2026-09-03 AGENT §3: unlock_press_point is the CENTRE of the right-handle
    // LOGICAL unlock zone (drawer link frame), not a visible button.  Unlock
    // requires BOTH the unlock-motor contact link (the robot link the request
    // names) within unlock_distance_threshold of this centre AND the
    // robot-model unlock_motor_joint actually extended (see
    // handle_unlock_request).  No button-joint evidence, no b1p displacement.
    ignition::math::Vector3d unlock_press_point{0.0, 0.0, 0.0};
    double unlock_distance_threshold{0.10};
    // Robot-model joint that is the right-hand unlock motor (e.g.
    // r_three_cyl_finger3_joint).  The plugin reads its REAL gazebo position to
    // prove the motor actually extended.  Required for drawer unlock.
    std::string unlock_motor_joint;
    double unlock_retracted_position{0.0};
    // Motor position must reach retracted + this floor to count as extended.
    double unlock_extension_floor{0.001};
    // Position above which the motor is bottoming out - unlock is refused.
    double unlock_extension_ceiling{0.0254};
    // 2026-09-06 AGENT doc §4.2 simulated_linkage (db1 sim config ONLY).  Strict
    // physical contact stays the DEFAULT; this flag exists only on the db1 sim
    // control.  It models "the right unlock motor drives the right-handle latch
    // release through an UNMODELLED linkage": the button stays on the right
    // handle, the visible rod geometry is unchanged, and finger3's real tip is
    // NOT required at the button.  Granting still needs the REAL unlock motor
    // stroke (pressed) plus BOTH handles physically held — the two hook rods
    // named by unlock_hold_{left,right}_link must keep their real tips within
    // grasp_contact_threshold of the drawer's left/right handle grasp points —
    // plus a valid lease session and a drawer that is latched and unattached.
    bool unlock_simulated_linkage{false};
    std::string unlock_linkage_description;
    // Robot-model hook rods (link + link-local tip point) that hold the drawer
    // handles while the (simulated) latch release happens.  Empty link in a
    // simulated_linkage control is a startup configuration error (see parse).
    std::string unlock_hold_left_link;
    ignition::math::Vector3d unlock_hold_left_point{0.0, 0.0, 0.0};
    std::string unlock_hold_right_link;
    ignition::math::Vector3d unlock_hold_right_point{0.0, 0.0, 0.0};
    // Bimanual attach / coupling contact contract (2026-09-02).  Attach is only
    // granted while both tool tips are within grasp_contact_threshold of their
    // handles (tight, ~0.02m, i.e. actual contact — hovering is rejected).
    // During the linear coupling drag the plugin keeps re-checking the same
    // per-side distances; if a tip loses contact for longer than
    // coupling_contact_grace_seconds the coupling is aborted and an
    // operation_fault is emitted (no more pulling in empty air).
    double grasp_contact_threshold{0.02};
    double coupling_contact_grace_seconds{0.5};
    double coupling_contact_lost_since{-1.0};
    double drawer_latch_stiffness{kDefaultDrawerLatchStiffness};
    // Rail latch: locked means the drawer is parked at the closed detent and a
    // bimanual attach is refused.  Unlock (right tool in the logical unlock
    // zone) clears the flag; the latch re-engages automatically once the drawer
    // has been opened at least once, returned to closed, and settled.
    bool drawer_unlocked{false};
    bool drawer_has_opened{false};
    // Visual playback (2026-09-06, SetCabinetPlayback): while playback_active
    // this drawer's latch/spring/coupling physics is bypassed and the prismatic
    // rail joint is driven kinematically along a piecewise-linear (time, q)
    // schedule.  Only a drawer control accepts playback.  All playback fields
    // are written by the physics request drain and read only inside on_update,
    // both under physics_callback_mutex_ (single writer).
    bool playback_active{false};
    bool playback_paused{false};
    bool playback_finished{false};
    std::string playback_lease_id;
    std::vector<std::pair<double, double>> playback_samples;  // (time, position)
    double playback_start_sim{0.0};
    double playback_elapsed{0.0};
    double playback_last_sim{-1.0};
    std::size_t reset_state_index{0U};
    std::size_t detent_target_index{0U};
    std::size_t state_index{0U};
    std::string state_id;
    bool activated{false};
    bool in_motion{false};
    std::uint64_t transition_sequence{0U};
    double position{0.0};
    double velocity{0.0};
    double effort{0.0};
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_publisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pressed_publisher;
    rclcpp::Publisher<CabinetControlState>::SharedPtr state_publisher;
  };

  struct PhysicsOutcome
  {
    bool success{false};
    std::string message;
    double distance{std::numeric_limits<double>::quiet_NaN()};
    // Bimanual drawer outcomes carry per-side distances.
    double left_distance{std::numeric_limits<double>::quiet_NaN()};
    double right_distance{std::numeric_limits<double>::quiet_NaN()};
    // Physical-contact evidence.  Unlock (2026-09-03): pressed = the unlock
    // motor joint was actually extended within [retracted+floor, ceiling) and
    // right_tool_contact = the motor's contact link tip was within the unlock
    // zone threshold.  Bimanual attach: per-side tool contact flags within the
    // drawer's tight grasp_contact_threshold.
    bool pressed{false};
    bool left_tool_contact{false};
    bool right_tool_contact{false};
    // 2026-09-06 AGENT doc §4.2: unlock acceptance mode + simulated-linkage
    // evidence.  unlock_mode is "strict" (default: finger3 real tip inside the
    // logical zone) or "simulated_linkage" (db1 sim config: real motor stroke +
    // both handles held; the latch release is driven through an unmodelled
    // linkage).  simulation_acceptance is true ONLY on a simulated_linkage
    // grant, so consumers can never report that branch as a real finger3 button
    // press.  The hold distances are plugin-measured hook-rod tips vs the
    // drawer handle grasp points (m); NaN when a mode does not measure that
    // side.
    std::string unlock_mode{"strict"};
    bool simulation_acceptance{false};
    double left_hold_distance{std::numeric_limits<double>::quiet_NaN()};
    double right_hold_distance{std::numeric_limits<double>::quiet_NaN()};
    bool left_handle_held{false};
    bool right_handle_held{false};
    // Visual playback (2026-09-06): true when a START schedule had fully
    // completed before the request was applied.
    bool finished{false};
  };

  struct PhysicsRequest
  {
    enum class Kind {kReset, kGrasp, kBimanualGrasp, kUnlock, kPlayback};
    static constexpr std::uint8_t kPlaybackStart = 0U;
    static constexpr std::uint8_t kPlaybackHold = 1U;
    static constexpr std::uint8_t kPlaybackRelease = 2U;
    enum class State {kPending, kExecuting, kCanceled, kCompleted};
    Kind kind{Kind::kReset};
    std::string control_id;
    std::string operation_lease_id;
    std::uint64_t active_control_generation{0U};
    std::string robot_model;
    std::string robot_link;
    ignition::math::Vector3d robot_grasp_point{0.0, 0.0, 0.0};
    std::string robot_base_link;
    bool attach{false};
    // P3-8 grab-and-drive: true skips the base-brake joint so the robot base
    // can translate along the drawer axis during the pull (arms hold their
    // grasp config; the linear-drag coupling opens/closes the drawer 1:1).
    bool base_free{false};
    // Bimanual drawer fields.
    std::string left_robot_link;
    std::string right_robot_link;
    ignition::math::Vector3d left_robot_grasp_point{0.0, 0.0, 0.0};
    ignition::math::Vector3d right_robot_grasp_point{0.0, 0.0, 0.0};
    ignition::math::Vector3d left_handle_point{0.0, 0.0, 0.0};
    ignition::math::Vector3d right_handle_point{0.0, 0.0, 0.0};
    ignition::math::Vector3d unlock_press_point{0.0, 0.0, 0.0};
    bool drawer_unlock{false};
    // Visual playback (2026-09-06): command + 1-D rail schedule (time, q).
    std::uint8_t playback_command{kPlaybackStart};
    std::vector<std::pair<double, double>> playback_samples;
    std::promise<PhysicsOutcome> promise;
    std::atomic<State> state{State::kPending};
    std::atomic<bool> completed{false};
  };

  struct ServiceCallbackLifetime
  {
    std::mutex mutex;
    std::condition_variable condition;
    CabinetStatePlugin * owner{nullptr};
    std::size_t active_callbacks{0U};
    bool shutting_down{false};
  };

  class ServiceCallbackLease
  {
public:
    explicit ServiceCallbackLease(
      std::shared_ptr<ServiceCallbackLifetime> lifetime)
    : lifetime_(std::move(lifetime)) {}

    ServiceCallbackLease(const ServiceCallbackLease &) = delete;
    ServiceCallbackLease & operator=(const ServiceCallbackLease &) = delete;

    ~ServiceCallbackLease()
    {
      std::lock_guard<std::mutex> lock(lifetime_->mutex);
      if (lifetime_->active_callbacks > 0U) {
        --lifetime_->active_callbacks;
      }
      lifetime_->condition.notify_all();
    }

private:
    std::shared_ptr<ServiceCallbackLifetime> lifetime_;
  };

  static CabinetStatePlugin * acquire_service_callback(
    const std::shared_ptr<ServiceCallbackLifetime> & lifetime)
  {
    std::lock_guard<std::mutex> lock(lifetime->mutex);
    if (lifetime->shutting_down || !lifetime->owner) {
      return nullptr;
    }
    ++lifetime->active_callbacks;
    return lifetime->owner;
  }

  void configure_controls(const sdf::ElementPtr & sdf)
  {
    if (!sdf->HasElement("control")) {
      throw std::invalid_argument(
              "Cabinet state plugin requires at least one <control>.");
    }
    const auto state_qos = rclcpp::QoS(1).reliable().transient_local();
    auto element = sdf->GetElement("control");
    while (element) {
      Control control;
      control.id = required_text(element, "control_id");
      if (control_indices_.count(control.id) != 0U) {
        throw std::invalid_argument(
                "Duplicate cabinet control id: " + control.id);
      }
      control.kind = parse_kind(required_text(element, "control_type"));
      control.type = message_type(control.kind);
      control.joint_name = required_text(element, "joint_name");
      control.joint = model_->GetJoint(control.joint_name);
      if (!control.joint) {
        throw std::invalid_argument(
                "Cabinet joint not found: " + control.joint_name);
      }
      control.link = control.joint->GetChild();
      if (!control.link) {
        throw std::invalid_argument(
                "Cabinet joint has no child link: " + control.joint_name);
      }
      control.joint_state_topic = required_text(
        element, "joint_state_topic");
      control.pressed_topic = required_text(element, "pressed_topic");
      control.state_topic = required_text(element, "state_topic");
      require_absolute_topic(control.joint_state_topic, "joint_state_topic");
      require_absolute_topic(control.pressed_topic, "pressed_topic");
      require_absolute_topic(control.state_topic, "state_topic");
      control.state_ids = parse_words(required_text(element, "state_ids"));
      control.reset_position = optional_double(
        element, "reset_position", 0.0);
      control.damping = optional_double(element, "spring_damping", 0.1);
      control.grasp_damping = optional_double(
        element, "grasp_damping", control.damping);
      control.grasp_coupling_stiffness = optional_double(
        element, "grasp_coupling_stiffness", 0.0);
      control.grasp_coupling_damping = optional_double(
        element, "grasp_coupling_damping", 0.0);
      control.grasp_coupling_max_effort = optional_double(
        element, "grasp_coupling_max_effort", 0.0);
      control.motion_tolerance = optional_double(
        element, "motion_tolerance", 0.025);
      control.graspable = optional_bool(element, "graspable", false);
      if (element->HasElement("grasp_point")) {
        control.grasp_point = parse_vector3(
          element->Get<std::string>("grasp_point"));
      }
      if (element->HasElement("actuation_collision")) {
        auto collision_element = element->GetElement("actuation_collision");
        while (collision_element) {
          const auto collision_reference =
            collision_element->Get<std::string>();
          if (collision_reference.empty()) {
            throw std::invalid_argument(
                    "Cabinet <actuation_collision> must not be empty.");
          }
          const auto collision = find_actuation_collision(
            control.link, collision_reference);
          if (!collision) {
            throw std::invalid_argument(
                    "Cabinet actuation collision was not found: " +
                    collision_reference);
          }
          const auto duplicate = std::find_if(
            control.actuation_collisions.begin(),
            control.actuation_collisions.end(),
            [&collision](const ActuationCollision & configured) {
              return configured.collision == collision;
            });
          if (duplicate == control.actuation_collisions.end()) {
            const auto ode_collision =
              boost::dynamic_pointer_cast<gazebo::physics::ODECollision>(
              collision);
            if (!ode_collision || !ode_collision->GetCollisionId()) {
              throw std::invalid_argument(
                      "Cabinet actuation collision masking requires the "
                      "Gazebo ODE physics backend: " + collision_reference);
            }
            const auto collision_id = ode_collision->GetCollisionId();
            control.actuation_collisions.push_back(
              {collision,
                static_cast<unsigned int>(
                  dGeomGetCategoryBits(collision_id)),
                static_cast<unsigned int>(
                  dGeomGetCollideBits(collision_id))});
          }
          collision_element = collision_element->GetNextElement(
            "actuation_collision");
        }
        control.actuation_collision_restore_delay = optional_double(
          element, "actuation_collision_restore_delay", 1.5);
        control.actuation_collision_restore_distance = optional_double(
          element, "actuation_collision_restore_distance", 0.5);
      }

      if (control.damping < 0.0 || control.grasp_damping < 0.0 ||
        control.grasp_coupling_stiffness < 0.0 ||
        control.grasp_coupling_damping < 0.0 ||
        control.grasp_coupling_max_effort < 0.0 ||
        control.motion_tolerance <= 0.0 ||
        control.actuation_collision_restore_delay < 0.0 ||
        control.actuation_collision_restore_distance <= 0.0)
      {
        throw std::invalid_argument(
                "Cabinet control damping values must be non-negative and "
                "motion tolerance must be positive: " + control.id);
      }
      if (!control.actuation_collisions.empty() && !control.graspable) {
        throw std::invalid_argument(
                "Cabinet actuation collision suppression requires a "
                "graspable control: " + control.id);
      }
      const bool has_grasp_coupling =
        control.grasp_coupling_stiffness > 0.0 ||
        control.grasp_coupling_damping > 0.0 ||
        control.grasp_coupling_max_effort > 0.0;
      if (has_grasp_coupling &&
        (!control.graspable ||
        control.grasp_coupling_stiffness <= 0.0 ||
        control.grasp_coupling_damping <= 0.0 ||
        control.grasp_coupling_max_effort <= 0.0))
      {
        throw std::invalid_argument(
                "Cabinet compliant grasp coupling requires a graspable "
                "control and positive stiffness, damping and effort: " +
                control.id);
      }
      const double lower = control.joint->LowerLimit(0);
      const double upper = control.joint->UpperLimit(0);
      if (control.reset_position < lower - 1.0e-6 ||
        control.reset_position > upper + 1.0e-6)
      {
        throw std::invalid_argument(
                "Cabinet reset position exceeds joint limits: " + control.id);
      }
      if (control.state_ids.empty() ||
        std::any_of(
          control.state_ids.begin(), control.state_ids.end(),
          [](const std::string & value) {return value.empty();}) ||
        std::unordered_set<std::string>(
          control.state_ids.begin(), control.state_ids.end()).size() !=
        control.state_ids.size())
      {
        throw std::invalid_argument(
                "Cabinet state IDs must be non-empty and unique: " +
                control.id);
      }
      if (control.kind == ControlKind::kButton) {
        if (control.state_ids.size() != 2U) {
          throw std::invalid_argument(
                  "Button state_ids must be: released pressed.");
        }
        control.stiffness = optional_double(
          element, "spring_stiffness", 800.0);
        control.grasp_stiffness = control.stiffness;
        control.press_threshold = optional_double(
          element, "press_threshold", 0.006);
        control.release_threshold = optional_double(
          element, "release_threshold", 0.003);
        if (control.stiffness <= 0.0 || control.press_threshold <= 0.0 ||
          control.release_threshold < 0.0 ||
          control.release_threshold >= control.press_threshold ||
          control.press_threshold >= control.joint->UpperLimit(0))
        {
          throw std::invalid_argument(
                  "Invalid button spring or threshold configuration: " +
                  control.id);
        }
        if (control.graspable) {
          throw std::invalid_argument(
                  "Momentary cabinet buttons must not be graspable.");
        }
      } else {
        control.detents = parse_doubles(required_text(element, "detents"));
        control.stiffness = optional_double(
          element, "detent_stiffness", 0.6);
        control.grasp_stiffness = optional_double(
          element, "grasp_stiffness", control.stiffness);
        control.detent_hysteresis = optional_double(
          element, "detent_hysteresis", 0.0);
        if (control.detents.size() < 2U ||
          control.detents.size() != control.state_ids.size() ||
          !std::is_sorted(control.detents.begin(), control.detents.end()) ||
          std::adjacent_find(
            control.detents.begin(), control.detents.end(),
            [](double left, double right) {
              return std::abs(left - right) <= 1.0e-9;
            }) != control.detents.end() ||
          control.stiffness <= 0.0 || control.grasp_stiffness < 0.0 ||
          control.detent_hysteresis < 0.0)
        {
          throw std::invalid_argument(
                  "Invalid detent configuration for cabinet control: " +
                  control.id);
        }
        if (control.detents.front() < lower - 1.0e-6 ||
          control.detents.back() > upper + 1.0e-6)
        {
          throw std::invalid_argument(
                  "Cabinet detents exceed joint limits: " + control.id);
        }
        control.reset_state_index = nearest_index(
          control.detents, control.reset_position);
        control.detent_target_index = control.reset_state_index;
        double minimum_detent_gap = std::numeric_limits<double>::infinity();
        for (std::size_t index = 1U; index < control.detents.size(); ++index) {
          minimum_detent_gap = std::min(
            minimum_detent_gap,
            control.detents[index] - control.detents[index - 1U]);
        }
        if (control.detent_hysteresis >= 0.5 * minimum_detent_gap) {
          throw std::invalid_argument(
                  "Cabinet detent hysteresis must be smaller than half the "
                  "minimum detent spacing: " + control.id);
        }
        if (std::abs(
            control.detents[control.reset_state_index] -
            control.reset_position) > 1.0e-6)
        {
          throw std::invalid_argument(
                  "Cabinet reset position must match a detent: " + control.id);
        }
        // Push-push release is slider-only: the parse must not run for knobs,
        // switches or doors, and sliders without the element keep the struct
        // default (infinity) so the feature stays disabled instead of throwing
        // inside optional_double's finite check.
        if (control.kind == ControlKind::kSlider) {
          if (element->HasElement("slider_release_position")) {
            control.slider_release_position = optional_double(
              element, "slider_release_position",
              std::numeric_limits<double>::infinity());
          }
          control.slider_release_reengage_position = optional_double(
            element, "slider_release_reengage_position",
            0.5 * (control.detents.front() + control.detents.back()) -
            2.0 * control.detent_hysteresis);
          if (std::isfinite(control.slider_release_position)) {
            const double detent_midpoint =
              0.5 * (control.detents.front() + control.detents.back());
            if (control.slider_release_position <= control.detents.back() ||
              control.slider_release_position > upper)
            {
              throw std::invalid_argument(
                      "Slider release position must lie strictly above the open "
                      "detent and inside the joint limits: " + control.id);
            }
            if (control.slider_release_reengage_position <=
                control.detents.front() ||
              control.slider_release_reengage_position >= detent_midpoint)
            {
              throw std::invalid_argument(
                      "Slider release re-engage position must lie strictly below "
                      "the detent midpoint: " + control.id);
            }
          }
        }
        // A drawer is a bimanual pull-out: left tool attaches to the left
        // handle (grasp_point), right tool to the right handle, and the rail
        // latch is released by pushing the right tool into the logical unlock
        // zone.  No compliant coupling, no push-push release — those are
        // single-handed semantics.
        if (control.kind == ControlKind::kDrawer) {
          if (!control.graspable) {
            throw std::invalid_argument(
                    "Cabinet drawer controls must be graspable: " + control.id);
          }
          if (!element->HasElement("right_grasp_point") ||
            !element->HasElement("unlock_press_point") ||
            !element->HasElement("unlock_motor_joint"))
          {
            throw std::invalid_argument(
                    "Cabinet drawer control requires <right_grasp_point>, "
                    "<unlock_press_point> and <unlock_motor_joint>: " +
                    control.id);
          }
          control.right_grasp_point = parse_vector3(
            element->Get<std::string>("right_grasp_point"));
          control.unlock_press_point = parse_vector3(
            element->Get<std::string>("unlock_press_point"));
          control.unlock_distance_threshold = optional_double(
            element, "unlock_distance_threshold", 0.10);
          control.drawer_latch_stiffness = optional_double(
            element, "drawer_latch_stiffness", kDefaultDrawerLatchStiffness);
          // 2026-09-03 AGENT §3: the drawer names the ROBOT-model joint that is
          // the unlock motor.  The plugin reads its real gazebo position for
          // the extension evidence; no in-plugin button control is involved.
          control.unlock_motor_joint = element->Get<std::string>(
            "unlock_motor_joint");
          control.unlock_retracted_position = optional_double(
            element, "unlock_retracted_position", 0.0);
          control.unlock_extension_floor = optional_double(
            element, "unlock_extension_floor", 0.001);
          control.unlock_extension_ceiling = optional_double(
            element, "unlock_extension_ceiling", 0.0254);
          control.grasp_contact_threshold = optional_double(
            element, "grasp_contact_threshold", 0.02);
          control.coupling_contact_grace_seconds = optional_double(
            element, "coupling_contact_grace_seconds", 0.5);
          // 2026-09-06 AGENT doc §4.2 simulated_linkage: parsed only where the
          // sim config opts in.  Default remains strict physical contact (flag
          // false, no behaviour change).  When enabled the drawer must ALSO
          // name the two robot hook rods (link + link-local tip point) that
          // must keep holding the handles while the simulated latch release
          // happens; a missing side is a configuration error, never a silent
          // relaxation.
          control.unlock_simulated_linkage = optional_bool(
            element, "unlock_simulated_linkage", false);
          control.unlock_linkage_description =
            element->Get<std::string>(
              "unlock_linkage_description", "").first;
          control.unlock_hold_left_link =
            element->Get<std::string>(
              "unlock_hold_left_link", "").first;
          control.unlock_hold_left_point = parse_vector3(
            element->Get<std::string>(
              "unlock_hold_left_point", "0 0 0").first);
          control.unlock_hold_right_link =
            element->Get<std::string>(
              "unlock_hold_right_link", "").first;
          control.unlock_hold_right_point = parse_vector3(
            element->Get<std::string>(
              "unlock_hold_right_point", "0 0 0").first);
          if (control.unlock_simulated_linkage &&
            (control.unlock_hold_left_link.empty() ||
             control.unlock_hold_right_link.empty() ||
             !element->HasElement("unlock_hold_left_point") ||
             !element->HasElement("unlock_hold_right_point")))
          {
            throw std::invalid_argument(
                    "simulated_linkage drawer control must name both "
                    "<unlock_hold_left_link>/<unlock_hold_right_link> and "
                    "explicit <unlock_hold_left_point>/<unlock_hold_right_point> "
                    "(robot hook-rod links + link-local tip points): " +
                    control.id);
          }
          if (control.detents.size() != 2U) {
            throw std::invalid_argument(
                    "Cabinet drawer controls require exactly two detents "
                    "(closed open): " + control.id);
          }
          if (control.unlock_distance_threshold <= 0.0 ||
            control.drawer_latch_stiffness <= 0.0 ||
            control.grasp_contact_threshold <= 0.0 ||
            control.coupling_contact_grace_seconds <= 0.0)
          {
            throw std::invalid_argument(
                    "Cabinet drawer unlock_distance_threshold, "
                    "drawer_latch_stiffness, grasp_contact_threshold and "
                    "coupling_contact_grace_seconds must be positive: " +
                    control.id);
          }
          if (control.unlock_extension_floor < 0.0 ||
            control.unlock_retracted_position < 0.0 ||
            control.unlock_extension_ceiling <=
              control.unlock_retracted_position +
              control.unlock_extension_floor)
          {
            throw std::invalid_argument(
                    "Cabinet drawer unlock motor range must satisfy 0 <= "
                    "retracted, floor >= 0 and ceiling > retracted + floor: " +
                    control.id);
          }
          // A drawer may declare compliant grasp coupling (grasp_coupling_*).
          // For the bimanual drawer this REPLACES the hard tool-drawer weld: a
          // bounded spring-damper on the rail drives the drawer to track the
          // tools' commanded motion, so a kinematic arm teleport never fights
          // the drawer's inertia through a cross-model fixed joint (run8).
        }
      }

      control.joint_state_publisher =
        ros_node_->create_publisher<sensor_msgs::msg::JointState>(
        control.joint_state_topic, rclcpp::SensorDataQoS());
      control.pressed_publisher =
        ros_node_->create_publisher<std_msgs::msg::Bool>(
        control.pressed_topic, state_qos);
      control.state_publisher =
        ros_node_->create_publisher<CabinetControlState>(
        control.state_topic, state_qos);

      control_indices_.emplace(control.id, controls_.size());
      controls_.push_back(std::move(control));
      element = element->GetNextElement("control");
    }

  }

  void on_update()
  {
    std::lock_guard<std::mutex> physics_lock(physics_callback_mutex_);
    if (shutting_down_.load()) {
      return;
    }
    enforce_operation_watchdog();
    process_physics_requests();
    if (grasp_joint_ && active_control_link_ && active_robot_link_ptr_) {
      const auto relative_pose =
        active_control_link_->WorldPose().Inverse() *
        active_robot_link_ptr_->WorldPose();
      max_grasp_linear_error_ = std::max(
        max_grasp_linear_error_,
        relative_pose.Pos().Distance(grasp_relative_pose_.Pos()));
    }
    if (bimanual_grasp_is_active() && active_control_link_ &&
      active_left_robot_link_ptr_ && active_right_robot_link_ptr_)
    {
      const auto left_relative_pose =
        active_control_link_->WorldPose().Inverse() *
        active_left_robot_link_ptr_->WorldPose();
      const auto right_relative_pose =
        active_control_link_->WorldPose().Inverse() *
        active_right_robot_link_ptr_->WorldPose();
      max_left_grasp_linear_error_ = std::max(
        max_left_grasp_linear_error_,
        left_relative_pose.Pos().Distance(left_grasp_relative_pose_.Pos()));
      max_right_grasp_linear_error_ = std::max(
        max_right_grasp_linear_error_,
        right_relative_pose.Pos().Distance(right_grasp_relative_pose_.Pos()));
    }
    const double simulation_time = world_->SimTime().Double();
    for (auto & control : controls_) {
      const double raw_position = control.joint->Position(0);
      const double velocity = control.joint->GetVelocity(0);
      // Visual playback (2026-09-06): while a drawer is under a kinematic
      // playback schedule its rail latch/spring/coupling physics and the
      // pre-grasp disturbance detector are bypassed and the prismatic joint is
      // driven along the (time, q) samples.  Single writer: the schedule is
      // written only by the physics-request drain under physics_callback_mutex_.
      if (control.playback_active) {
        drive_drawer_playback(control, simulation_time);
        continue;
      }
      const bool control_is_being_grasped = grasp_is_active() &&
        active_grasp_control_ == control.id;
      record_pregrasp_disturbance(
        control, raw_position, velocity, control_is_being_grasped);
      double target = 0.0;
      if (control.kind != ControlKind::kButton) {
        const auto previous_detent_target_index =
          control.detent_target_index;
        // A detent may change only while this exact control is held by the
        // robot.  A cross-model grasp can transmit a short ODE impulse through
        // the cabinet assembly; letting unrelated joints track their raw
        // positions here would turn that disturbance into a persistent knob
        // or switch state change.  Latching every non-button target after
        // release also models the mechanical detent consistently.
        //
        // A slider is the exception: the robot only presses its face (there
        // is no fixed grasp), so the bistable detent follows the physical
        // position continuously.  Pressing the panel past the midpoint flips
        // the latch to the other detent, and the spring then assists the
        // remaining travel; releasing it leaves the panel parked at the
        // nearest detent.
        //
        // A drawer is bimanual and follows the physical position exactly like
        // a slider, but only while its rail latch is released.  While locked
        // the drawer is parked at the closed detent; once the unlock zone has
        // been engaged and the drawer has opened at least once, the latch
        // re-engages automatically when the drawer returns to the closed
        // position and settles (the operator releases and the ODE rail comes
        // to rest).
        if (control.kind == ControlKind::kDrawer) {
          if (control.drawer_unlocked) {
            while (control.detent_target_index + 1U < control.detents.size()) {
              const double boundary = 0.5 * (
                control.detents[control.detent_target_index] +
                control.detents[control.detent_target_index + 1U]) +
                control.detent_hysteresis;
              if (raw_position <= boundary) {
                break;
              }
              ++control.detent_target_index;
            }
            while (control.detent_target_index > 0U) {
              const double boundary = 0.5 * (
                control.detents[control.detent_target_index - 1U] +
                control.detents[control.detent_target_index]) -
                control.detent_hysteresis;
              if (raw_position >= boundary) {
                break;
              }
              --control.detent_target_index;
            }
            if (control.detent_target_index >= 1U) {
              control.drawer_has_opened = true;
            }
            if (control.drawer_has_opened &&
              !bimanual_grasp_is_active() &&
              raw_position <= control.detents.front() +
                control.motion_tolerance &&
              std::abs(velocity) <= kDrawerReLatchVelocityThreshold)
            {
              control.drawer_unlocked = false;
              control.drawer_has_opened = false;
              control.detent_target_index = control.reset_state_index;
              RCLCPP_INFO(
                ros_node_->get_logger(),
                "Drawer '%s' returned to the closed position and settled; "
                "the rail latch re-engaged.",
                control.id.c_str());
            }
          } else {
            control.detent_target_index = control.reset_state_index;
          }
        } else if (control.kind == ControlKind::kSlider &&
          control.slider_released)
        {
          // The push-release latch stays tripped while the panel falls back to
          // the closed detent; only once it is below the re-engage position
          // does the bistable ratchet re-arm (so the closing travel cannot
          // cross the midpoint and latch open again).
          control.detent_target_index = control.reset_state_index;
          if (raw_position <= control.slider_release_reengage_position) {
            control.slider_released = false;
          }
        } else if (control.kind == ControlKind::kSlider ||
          control_is_being_grasped) {
          while (control.detent_target_index + 1U < control.detents.size()) {
            const double boundary = 0.5 * (
              control.detents[control.detent_target_index] +
              control.detents[control.detent_target_index + 1U]) +
              control.detent_hysteresis;
            if (raw_position <= boundary) {
              break;
            }
            ++control.detent_target_index;
          }
          while (control.detent_target_index > 0U) {
            const double boundary = 0.5 * (
              control.detents[control.detent_target_index - 1U] +
              control.detents[control.detent_target_index]) -
              control.detent_hysteresis;
            if (raw_position >= boundary) {
              break;
            }
            --control.detent_target_index;
          }
        }
        if (control.kind == ControlKind::kSlider &&
          !control.slider_released &&
          std::isfinite(control.slider_release_position) &&
          raw_position >= control.slider_release_position)
        {
          // A push-push latch: pushing the open panel PAST its detent trips
          // the release, and the detent spring then returns the panel to the
          // closed detent.  This is how the robot closes a slider it can only
          // push (the tool cannot pull the panel back from the front).
          control.slider_released = true;
          control.detent_target_index = control.reset_state_index;
          RCLCPP_INFO(
            ros_node_->get_logger(),
            "Push-push release tripped for slider '%s' at %.6f rad; the "
            "detent spring is returning the panel to '%s'.",
            control.id.c_str(), raw_position,
            control.state_ids[control.reset_state_index].c_str());
        }
        if (control.detent_target_index != previous_detent_target_index) {
          RCLCPP_INFO(
            ros_node_->get_logger(),
            "Physical detent target for '%s' changed to '%s' at %.6f rad.",
            control.id.c_str(),
            control.state_ids[control.detent_target_index].c_str(),
            raw_position);
        }
        target = control.detents[control.detent_target_index];
      }
      const double effective_damping = control_is_being_grasped ?
        control.grasp_damping : control.damping;
      double effective_stiffness = control_is_being_grasped ?
        control.grasp_stiffness : control.stiffness;
      if (control.kind == ControlKind::kDrawer &&
        !control.drawer_unlocked)
      {
        // The locked rail holds the drawer firmly at the closed detent so it
        // cannot drift out of the gate between operations.
        effective_stiffness = control.drawer_latch_stiffness;
      }
      // A button's return spring is one-sided: it only restores from the
      // pressed side (position above target).  Real buttons sit flush with
      // their bezel and have no spring pulling them below flush.  A two-sided
      // spring here fights the ODE soft joint stop (limit effort 100 N) every
      // time the cap sinks a fraction below the lower limit: the spring
      // launches it back up at up to the effort clamp, sustaining a ~1 m/s
      // limit bounce (b1p measured 2026-09-02) instead of settling.  Below
      // the limit only the damping term acts, which bleeds the bounce energy
      // and lets the cap come to rest at the stop.  Doors, sliders, knobs and
      // drawers keep the two-sided detent/latch spring (they hold at
      // non-zero detents).
      const double spring_error =
        (control.kind == ControlKind::kButton) ?
        std::max(0.0, raw_position - target) :
        (raw_position - target);
      control.effort =
        -effective_stiffness * spring_error -
        effective_damping * velocity;
      if (!control_is_being_grasped) {
        // Bounded restoring force (see kMaxParkingEffort): an ungrasped joint
        // that overshoots its stop must never receive a restoring effort big
        // enough to defeat the ODE limit, or the divergence cascades to the
        // whole scene.  A grasped control is exempt: its drive is the already-
        // bounded grasp-coupling effort on top of a normally-small centering
        // term, and clamping it would break the validated knob/drawer grasp.
        control.effort = std::clamp(
          control.effort, -kMaxParkingEffort, kMaxParkingEffort);
      }
      if (control_is_being_grasped &&
        control.grasp_coupling_stiffness > 0.0)
      {
        if (control.kind == ControlKind::kDrawer && bimanual_coupling_active_ &&
          active_left_robot_link_ptr_ && active_right_robot_link_ptr_)
        {
          // Contact keepalive (2026-09-02): re-check each cycle that BOTH tool
          // tips are still pressed against their handles.  Pulling a drawer
          // whose handles have slipped out of the tips is "pulling in empty
          // air" — the previous bug.  While a tip is separated the drawer is
          // held (no drive effort); if the separation outlasts
          // coupling_contact_grace_seconds the coupling is aborted, an
          // operation_fault is emitted, and the operator can re-grasp and
          // retry instead of silently dragging nothing.
          const auto drawer_pose = control.link->WorldPose();
          const auto left_handle_world = drawer_pose.Pos() +
            drawer_pose.Rot().RotateVector(control.grasp_point);
          const auto right_handle_world = drawer_pose.Pos() +
            drawer_pose.Rot().RotateVector(control.right_grasp_point);
          const auto left_pose = active_left_robot_link_ptr_->WorldPose();
          const auto left_tip_world = left_pose.Pos() +
            left_pose.Rot().RotateVector(active_left_robot_grasp_point_);
          const auto right_pose = active_right_robot_link_ptr_->WorldPose();
          const auto right_tip_world = right_pose.Pos() +
            right_pose.Rot().RotateVector(active_right_robot_grasp_point_);
          const double keepalive_left = left_tip_world.Distance(left_handle_world);
          const double keepalive_right = right_tip_world.Distance(right_handle_world);
          const bool contact_ok = std::isfinite(keepalive_left) &&
            std::isfinite(keepalive_right) &&
            keepalive_left <= control.grasp_contact_threshold &&
            keepalive_right <= control.grasp_contact_threshold;
          if (contact_ok) {
            control.coupling_contact_lost_since = -1.0;
          } else if (control.coupling_contact_lost_since < 0.0) {
            control.coupling_contact_lost_since = simulation_time;
          }
          if (!contact_ok &&
            simulation_time - control.coupling_contact_lost_since >
            control.coupling_contact_grace_seconds)
          {
            // Contact lost beyond the grace period: abort the drag.  Fully
            // release the grasp bookkeeping (coupling hold, active session
            // state and the grasp-active topic) so the plugin, the grasp
            // aggregator and the Web agree the pull has ended the moment it
            // dies; the operator's later attach=false is then a harmless
            // "already released" no-op and a re-grasp must re-prove contact.
            const std::string lost_lease_id = active_grasp_lease_id_;
            control.coupling_contact_lost_since = -1.0;
            RCLCPP_WARN(
              ros_node_->get_logger(),
              "Bimanual drawer '%s' coupling aborted: tool contact lost "
              "(left %.6f m, right %.6f m beyond grasp_contact_threshold "
              "%.6f m for %.3f s).",
              control.id.c_str(), keepalive_left, keepalive_right,
              control.grasp_contact_threshold,
              control.coupling_contact_grace_seconds);
            release_bimanual_grasp_constraint();
            publish_operation_fault(lost_lease_id);
            // Skip the coupling drive this cycle: the drawer rests where it
            // is, and the operator must re-grasp before any further pull.
            continue;
          }
          if (!contact_ok) {
            // Within the grace window: hold the drawer (no drive effort) so a
            // transient bounce does not tear it, and so the drawer does not
            // snap toward a moving tool.
            continue;
          }
          // Linear-drag coupling for the bimanual drawer (see the attach): the
          // rail is driven toward the tools' commanded motion since attach.
          // The reaction force lands on the cabinet rail, not on the light
          // tools, so the kinematic arm teleport tracks its trajectory
          // perfectly and the joint_trajectory controller never sees a
          // tracking error.  The tools only hold the handles; the drawer
          // follows them through the bounded rail spring-damper.
          ignition::math::Vector3d axis = control.joint->GlobalAxis(0);
          axis.Normalize();
          const ignition::math::Vector3d left_position =
            active_left_robot_link_ptr_->WorldPose().Pos();
          const ignition::math::Vector3d right_position =
            active_right_robot_link_ptr_->WorldPose().Pos();
          const double tool_projection = axis.Dot(
            0.5 * (left_position + right_position));
          const double desired_position = std::clamp(
            bimanual_grasp_drawer_position_ +
              (tool_projection - bimanual_grasp_tool_projection_),
            control.joint->LowerLimit(0), control.joint->UpperLimit(0));
          const double position_error = desired_position - raw_position;
          const double coupling_effort = std::clamp(
            control.grasp_coupling_stiffness * position_error -
            control.grasp_coupling_damping * velocity,
            -control.grasp_coupling_max_effort,
            control.grasp_coupling_max_effort);
          control.effort += coupling_effort;
          max_grasp_coupling_effort_ = std::max(
            max_grasp_coupling_effort_, std::abs(coupling_effort));
        } else {
          // gazebo_ros2_control's plain position backend moves the arm
          // kinematically every controller update.  That motion is
          // authoritative for the simulated tool but can repeatedly inject
          // error into a cross-model ODE fixed joint.  Recover the commanded
          // one-DOF motion analytically from the tool orientation and apply it
          // as a compliant physical torque.  The cabinet joint is never
          // teleported: inertia, limits, damping, collisions and the fixed
          // grasp remain active.
          auto delta_rotation = active_robot_link_ptr_->WorldPose().Rot() *
            grasp_tool_rotation_.Inverse();
          delta_rotation.Normalize();
          const ignition::math::Vector3d imaginary{
            delta_rotation.X(), delta_rotation.Y(), delta_rotation.Z()};
          const double projected_sine = grasp_axis_world_.Dot(imaginary);
          const double projected_norm = std::hypot(
            delta_rotation.W(), projected_sine);
          if (std::isfinite(projected_norm) && projected_norm > 1.0e-8) {
            const double wrapped_twist = std::remainder(
              2.0 * std::atan2(projected_sine, delta_rotation.W()),
              2.0 * M_PI);
            const double twist_step = std::remainder(
              wrapped_twist - grasp_previous_twist_, 2.0 * M_PI);
            grasp_unwrapped_twist_ += twist_step;
            grasp_previous_twist_ = wrapped_twist;
            const double desired_position = std::clamp(
              grasp_initial_position_ + grasp_unwrapped_twist_,
              control.joint->LowerLimit(0), control.joint->UpperLimit(0));
            const double position_error = desired_position - raw_position;
            const double coupling_effort = std::clamp(
              control.grasp_coupling_stiffness * position_error -
              control.grasp_coupling_damping * velocity,
              -control.grasp_coupling_max_effort,
              control.grasp_coupling_max_effort);
            control.effort += coupling_effort;
            max_grasp_angle_error_ = std::max(
              max_grasp_angle_error_, std::abs(position_error));
            max_grasp_coupling_effort_ = std::max(
              max_grasp_coupling_effort_, std::abs(coupling_effort));
          }
        }
      }
      control.joint->SetForce(0, control.effort);
      if (control.actuation_collision_suppressed &&
        !control_is_being_grasped &&
        simulation_time >= control.collision_restore_not_before &&
        actuation_operation_is_complete(control) &&
        actuation_tool_is_clear(control) &&
        std::abs(raw_position - target) <= control.motion_tolerance &&
        std::abs(velocity) <= control.motion_tolerance)
      {
        set_actuation_collision_enabled(control, true);
      }
      update_control_state(control, true);
    }

    if (simulation_time < last_publish_time_) {
      last_publish_time_ = simulation_time - publish_period_;
    }
    if (simulation_time - last_publish_time_ < publish_period_) {
      return;
    }
    last_publish_time_ = simulation_time;
    for (const auto & control : controls_) {
      publish_control(control);
    }
  }

  bool bimanual_grasp_is_active() const
  {
    return bimanual_coupling_active_ || static_cast<bool>(left_grasp_joint_) ||
      static_cast<bool>(right_grasp_joint_);
  }

  bool grasp_is_active() const
  {
    return static_cast<bool>(grasp_joint_) || compliant_grasp_active_ ||
      bimanual_grasp_is_active();
  }

  void reset_pregrasp_tracking_locked() noexcept
  {
    pregrasp_disturbance_detected_ = false;
    pregrasp_disturbance_control_.clear();
    pregrasp_max_position_error_ = 0.0;
    pregrasp_max_velocity_ = 0.0;
    grasp_engaged_during_operation_ = false;
  }

  void receive_active_control(const std::string & control_id)
  {
    std::lock_guard<std::mutex> lock(active_control_mutex_);
    active_control_received_ = true;
    if (active_operation_control_ != control_id) {
      // Retire the old lease before clearing its heartbeat.  A delayed
      // heartbeat or queued same-control grasp request from that session must
      // never become valid again after an idle boundary/new generation.
      remember_expired_operation_lease_locked(
        operation_heartbeat_lease_id_);
      ++active_control_generation_;
      reset_pregrasp_tracking_locked();
      operation_heartbeat_received_ = false;
      operation_heartbeat_lease_id_.clear();
      operation_last_heartbeat_ = std::chrono::steady_clock::time_point{};
      ++operation_heartbeat_sequence_;
    }
    active_operation_control_ = control_id;
  }

  void receive_operation_heartbeat(const std::string & operation_lease_id)
  {
    std::lock_guard<std::mutex> lock(active_control_mutex_);
    if (operation_lease_id.empty() || active_operation_control_.empty() ||
      expired_operation_lease_ids_.count(operation_lease_id) != 0U)
    {
      return;
    }
    if (operation_heartbeat_lease_id_ != operation_lease_id) {
      // A same-control successor lease supersedes its predecessor even if an
      // idle active-control sample was lost.  Permanently reject delayed
      // traffic from the predecessor within the bounded replay window.
      remember_expired_operation_lease_locked(
        operation_heartbeat_lease_id_);
    }
    operation_heartbeat_received_ = true;
    operation_heartbeat_lease_id_ = operation_lease_id;
    operation_last_heartbeat_ = std::chrono::steady_clock::now();
    ++operation_heartbeat_sequence_;
  }

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

  bool operation_heartbeat_authorizes_grasp_locked(
    const PhysicsRequest & request) const
  {
    if (!active_control_received_ ||
      request.active_control_generation != active_control_generation_ ||
      !operation_heartbeat_received_ ||
      expired_operation_lease_ids_.count(request.operation_lease_id) != 0U ||
      !operation_session_matches(
        request.control_id, request.operation_lease_id,
        active_operation_control_, operation_heartbeat_lease_id_))
    {
      return false;
    }
    // Take now only after the heartbeat snapshot while holding the callback
    // mutex.  A newer callback can therefore never produce a future timestamp
    // and spuriously trip the fail-closed elapsed-time policy.
    const auto now = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration<double>(
      now - operation_last_heartbeat_).count();
    return !operation_watchdog_has_expired(
      true, false, elapsed, operation_watchdog_timeout_);
  }

  Control * first_suppressed_control() noexcept
  {
    const auto found = std::find_if(
      controls_.begin(), controls_.end(),
      [](const Control & control) {
        return control.actuation_collision_suppressed;
      });
    return found == controls_.end() ? nullptr : &*found;
  }

  void log_watchdog_release_failure(
    const std::string & reason,
    std::chrono::steady_clock::time_point now) noexcept
  {
    watchdog_release_retry_not_before_ = now + kWatchdogReleaseRetryDelay;
    if (watchdog_last_release_error_log_ !=
      std::chrono::steady_clock::time_point{} &&
      now - watchdog_last_release_error_log_ < kWatchdogReleaseLogPeriod)
    {
      return;
    }
    watchdog_last_release_error_log_ = now;
    RCLCPP_ERROR(
      ros_node_->get_logger(),
      "Watchdog could not fully release the cabinet grasp; it will retry "
      "on the Gazebo update thread: %s", reason.c_str());
  }

  void publish_operation_fault(const std::string & operation_lease_id) noexcept
  {
    if (!operation_fault_publisher_ || operation_lease_id.empty()) {
      return;
    }
    try {
      std_msgs::msg::String message;
      message.data = operation_lease_id;
      operation_fault_publisher_->publish(message);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        ros_node_->get_logger(),
        "Could not publish the cabinet watchdog lease fault: %s",
        error.what());
    } catch (...) {
      RCLCPP_ERROR(
        ros_node_->get_logger(),
        "Could not publish the cabinet watchdog lease fault.");
    }
  }

  void clear_watchdog_recovery() noexcept
  {
    watchdog_recovery_pending_ = false;
    watchdog_release_completed_ = false;
    watchdog_recovery_control_.clear();
    watchdog_recovery_lease_id_.clear();
    watchdog_release_retry_not_before_ =
      std::chrono::steady_clock::time_point{};
    watchdog_last_release_error_log_ =
      std::chrono::steady_clock::time_point{};
    watchdog_monitor_started_at_ =
      std::chrono::steady_clock::time_point{};
    watchdog_last_valid_heartbeat_ =
      std::chrono::steady_clock::time_point{};
    watchdog_activity_control_.clear();
    watchdog_activity_lease_id_.clear();
  }

  void continue_watchdog_recovery(
    std::chrono::steady_clock::time_point now) noexcept
  {
    if (!watchdog_recovery_pending_) {
      return;
    }
    if (!watchdog_release_completed_) {
      if (now < watchdog_release_retry_not_before_) {
        return;
      }
      try {
        const auto release = release_grasp_constraint();
        if (!release.success) {
          log_watchdog_release_failure(release.message, now);
          return;
        }
        watchdog_release_completed_ = true;
      } catch (const std::exception & error) {
        log_watchdog_release_failure(error.what(), now);
        return;
      } catch (...) {
        log_watchdog_release_failure("unknown Gazebo release exception", now);
        return;
      }
      // release_grasp_constraint schedules the normal collision restoration.
      // Keep its real tool-clearance gate: after a crash the tool may remain
      // geometrically overlapped, so automatically waiving clearance could
      // re-enable ODE contact with an unsafe impulse.  The recovery interlock
      // rejects every new attach until the tool actually moves clear (or an
      // explicit reset restores the masks).
    }

    const auto control_it = control_indices_.find(watchdog_recovery_control_);
    if (control_it != control_indices_.end() &&
      controls_[control_it->second].actuation_collision_suppressed)
    {
      return;
    }
    RCLCPP_INFO(
      ros_node_->get_logger(),
      "Cabinet watchdog recovery for '%s' completed; grasp, base brake and "
      "collision policy are safe for a later lease.",
      watchdog_recovery_control_.empty() ? "<unknown>" :
      watchdog_recovery_control_.c_str());
    clear_watchdog_recovery();
  }

  void enforce_operation_watchdog()
  {
    bool heartbeat_received = false;
    std::string active_control;
    std::string heartbeat_lease_id;
    std::chrono::steady_clock::time_point heartbeat;
    std::uint64_t active_control_generation = 0U;
    std::uint64_t heartbeat_sequence = 0U;
    {
      std::lock_guard<std::mutex> lock(active_control_mutex_);
      heartbeat_received = operation_heartbeat_received_;
      active_control = active_operation_control_;
      heartbeat_lease_id = operation_heartbeat_lease_id_;
      heartbeat = operation_last_heartbeat_;
      active_control_generation = active_control_generation_;
      heartbeat_sequence = operation_heartbeat_sequence_;
    }
    // Taking now after the synchronized snapshot guarantees heartbeat <= now.
    const auto now = std::chrono::steady_clock::now();
    if (watchdog_recovery_pending_) {
      continue_watchdog_recovery(now);
      return;
    }

    const bool active_grasp = grasp_is_active() ||
      static_cast<bool>(base_brake_joint_);
    auto * suppressed_control = first_suppressed_control();
    std::string activity_control;
    std::string activity_lease_id;
    if (active_grasp) {
      activity_control = active_grasp_control_;
      activity_lease_id = active_grasp_lease_id_;
    } else if (suppressed_control) {
      activity_control = suppressed_control->id;
      activity_lease_id =
        suppressed_control->actuation_collision_operation_lease_id;
    } else {
      activity_control = active_control;
      activity_lease_id = heartbeat_lease_id;
    }
    const bool activity_exists = !active_control.empty() || active_grasp ||
      suppressed_control != nullptr;
    if (!activity_exists) {
      watchdog_monitor_started_at_ =
        std::chrono::steady_clock::time_point{};
      watchdog_last_valid_heartbeat_ =
        std::chrono::steady_clock::time_point{};
      watchdog_activity_control_.clear();
      watchdog_activity_lease_id_.clear();
      return;
    }

    if (watchdog_activity_control_ != activity_control ||
      watchdog_activity_lease_id_ != activity_lease_id)
    {
      watchdog_activity_control_ = activity_control;
      watchdog_activity_lease_id_ = activity_lease_id;
      watchdog_monitor_started_at_ = now;
      watchdog_last_valid_heartbeat_ =
        std::chrono::steady_clock::time_point{};
    }
    if (watchdog_monitor_started_at_ ==
      std::chrono::steady_clock::time_point{})
    {
      watchdog_monitor_started_at_ = now;
    }

    const bool heartbeat_matches_activity = heartbeat_received &&
      operation_session_matches(
      activity_control, activity_lease_id,
      active_control, heartbeat_lease_id);
    if (heartbeat_matches_activity &&
      heartbeat > watchdog_last_valid_heartbeat_)
    {
      watchdog_last_valid_heartbeat_ = heartbeat;
    }
    const auto reference_time = std::max(
      watchdog_monitor_started_at_, watchdog_last_valid_heartbeat_);
    const double elapsed =
      std::chrono::duration<double>(now - reference_time).count();
    const bool physical_session_was_revoked = active_grasp &&
      active_grasp_control_generation_ != active_control_generation;
    if (!physical_session_was_revoked &&
      !operation_watchdog_has_expired(
        !active_control.empty(), active_grasp || suppressed_control,
        elapsed, operation_watchdog_timeout_))
    {
      return;
    }

    {
      std::lock_guard<std::mutex> lock(active_control_mutex_);
      // Only a newer heartbeat for this exact monitored session may defeat a
      // timeout.  Unrelated lease traffic must not postpone release of an
      // abandoned physical session.  For a pre-attach operation with no
      // physical hazard, a control-generation change is re-evaluated next tick.
      const bool current_session_heartbeat = operation_session_matches(
        activity_control, activity_lease_id,
        active_operation_control_, operation_heartbeat_lease_id_);
      if ((active_control_generation_ != active_control_generation ||
        operation_heartbeat_sequence_ != heartbeat_sequence) &&
        current_session_heartbeat)
      {
        watchdog_last_valid_heartbeat_ = operation_last_heartbeat_;
        return;
      }
      if (!active_grasp && !suppressed_control &&
        active_control_generation_ != active_control_generation)
      {
        return;
      }
      active_control_received_ = true;
      active_operation_control_.clear();
      ++active_control_generation_;
      operation_heartbeat_received_ = false;
      operation_heartbeat_lease_id_.clear();
      operation_last_heartbeat_ =
        std::chrono::steady_clock::time_point{};
      ++operation_heartbeat_sequence_;
      remember_expired_operation_lease_locked(activity_lease_id);
      reset_pregrasp_tracking_locked();
    }

    watchdog_recovery_pending_ = true;
    watchdog_release_completed_ = false;
    watchdog_recovery_control_ = activity_control;
    watchdog_recovery_lease_id_ = activity_lease_id;
    watchdog_release_retry_not_before_ = now;
    RCLCPP_ERROR(
      ros_node_->get_logger(),
      physical_session_was_revoked ?
      "Cabinet operation session for '%s' was explicitly revoked; starting "
      "fail-safe physical recovery (last heartbeat %.3f s ago)." :
      "Cabinet operation lease heartbeat for '%s' timed out after %.3f s; "
      "starting fail-safe physical recovery.",
      activity_control.empty() ? "<unknown>" : activity_control.c_str(),
      elapsed);
    // Notify a surviving operator before physical release.  It accepts the
    // fault only if this exact global lease is still current, then stops
    // MoveIt/Nav2/controller goals.  The physics-thread release below remains
    // independent so a dead operator cannot block recovery.
    publish_operation_fault(activity_lease_id);
    continue_watchdog_recovery(now);
  }

  bool actuation_tool_is_clear(const Control & control) const
  {
    if (!control.collision_restore_robot_link) {
      // A suppressed collision without an observable robot probe cannot prove
      // clearance and therefore fails closed.  This also protects partial
      // attach failures whose Gazebo bookkeeping never reached full commit.
      return false;
    }
    const auto control_pose = control.link->WorldPose();
    const auto grasp_position = control_pose.Pos() +
      control_pose.Rot().RotateVector(control.grasp_point);
    const auto robot_pose =
      control.collision_restore_robot_link->WorldPose();
    const auto robot_grasp_position = robot_pose.Pos() +
      robot_pose.Rot().RotateVector(
      control.collision_restore_robot_grasp_point);
    const double distance = robot_grasp_position.Distance(grasp_position);
    return std::isfinite(distance) &&
           distance >= control.actuation_collision_restore_distance;
  }

  bool actuation_operation_is_complete(const Control & control) const
  {
    if (watchdog_recovery_pending_ &&
      watchdog_release_completed_ &&
      watchdog_recovery_control_ == control.id)
    {
      // A later lease heartbeat must not keep an abandoned collision mask
      // disabled forever.  The recovery interlock rejects new attachment until
      // this control's configured delay and physical settle gates complete.
      return true;
    }
    std::lock_guard<std::mutex> lock(active_control_mutex_);
    return active_control_received_ && active_operation_control_.empty();
  }

  void record_pregrasp_disturbance(
    const Control & control,
    double position,
    double velocity,
    bool control_is_being_grasped)
  {
    if (control.kind == ControlKind::kButton || control_is_being_grasped ||
      control.detent_target_index >= control.detents.size())
    {
      return;
    }
    // 2026-09-02: 显式解锁后抽屉为自由滑块，operator 的解锁按压/回退/支撑
    // 行程会主动推动它——这些运动是操作内容而非"预抓取扰动"。扰动闩存只对
    // 锁定控件生效（解锁成功即 operator 物理接管，见 handle_unlock_request
    // 成功分支的清零）。
    if (control.kind == ControlKind::kDrawer && control.drawer_unlocked) {
      return;
    }
    const double detent_position =
      control.detents[control.detent_target_index];
    if (!pregrasp_detent_is_disturbed(
        position, velocity, detent_position,
        control.motion_tolerance, control.motion_tolerance))
    {
      return;
    }

    bool first_disturbance = false;
    {
      std::lock_guard<std::mutex> lock(active_control_mutex_);
      // A grasp has already been attached during this operation, so the
      // control is no longer in its pre-grasp phase.  The detent snap on
      // release and settling motion belong to the physical outcome.
      if (grasp_engaged_during_operation_ || !active_control_received_ ||
        active_operation_control_ != control.id)
      {
        return;
      }
      first_disturbance = !pregrasp_disturbance_detected_;
      pregrasp_disturbance_detected_ = true;
      pregrasp_disturbance_control_ = control.id;
      pregrasp_max_position_error_ = std::max(
        pregrasp_max_position_error_, std::abs(position - detent_position));
      pregrasp_max_velocity_ = std::max(
        pregrasp_max_velocity_, std::abs(velocity));
    }
    if (first_disturbance) {
      RCLCPP_ERROR(
        ros_node_->get_logger(),
        "Unsafe pre-grasp movement detected for cabinet control '%s': "
        "position %.6f rad, latched detent %.6f rad, velocity %.6f rad/s.",
        control.id.c_str(), position, detent_position, velocity);
    }
  }

  void update_control_state(Control & control, bool count_transition)
  {
    const std::string previous_state = control.state_id;
    const bool previous_activated = control.activated;
    const double raw_position = control.joint->Position(0);
    control.position = control.kind == ControlKind::kButton ?
      std::max(0.0, raw_position) : raw_position;
    control.velocity = control.joint->GetVelocity(0);

    if (control.kind == ControlKind::kButton) {
      if (!control.activated && control.position >= control.press_threshold) {
        control.activated = true;
      } else if (
        control.activated && control.position <= control.release_threshold)
      {
        control.activated = false;
      }
      control.state_index = control.activated ? 1U : 0U;
      control.state_id = control.state_ids[control.state_index];
      control.in_motion = std::abs(control.velocity) > 0.002;
    } else {
      control.state_index = nearest_index(control.detents, control.position);
      control.state_id = control.state_ids[control.state_index];
      control.activated = control.state_index != control.reset_state_index;
      control.in_motion =
        std::abs(control.velocity) > control.motion_tolerance ||
        std::abs(control.position - control.detents[control.state_index]) >
        control.motion_tolerance;
    }

    if (count_transition && !previous_state.empty() &&
      (previous_state != control.state_id ||
      previous_activated != control.activated))
    {
      ++control.transition_sequence;
    }
  }

  void publish_control(const Control & control) const
  {
    const auto stamp = ros_node_->get_clock()->now();
    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = stamp;
    joint_state.name.push_back(control.joint_name);
    joint_state.position.push_back(control.position);
    joint_state.velocity.push_back(control.velocity);
    joint_state.effort.push_back(control.effort);
    control.joint_state_publisher->publish(joint_state);

    std_msgs::msg::Bool pressed;
    pressed.data = control.activated;
    control.pressed_publisher->publish(pressed);

    CabinetControlState state;
    state.header.stamp = stamp;
    state.control_id = control.id;
    state.control_type = control.type;
    state.valid = true;
    state.position = control.position;
    state.velocity = control.velocity;
    state.effort = control.effort;
    const double lower = control.joint->LowerLimit(0);
    const double upper = control.joint->UpperLimit(0);
    state.normalized_position = upper > lower ?
      std::clamp((control.position - lower) / (upper - lower), 0.0, 1.0) :
      0.0;
    state.state_id = control.state_id;
    state.activated = control.activated;
    state.in_motion = control.in_motion;
    state.transition_sequence = control.transition_sequence;
    control.state_publisher->publish(state);
  }

  gazebo::physics::CollisionPtr find_actuation_collision(
    const gazebo::physics::LinkPtr & default_link,
    const std::string & reference) const
  {
    auto link = default_link;
    auto collision_name = reference;
    const auto separator = reference.find("::");
    if (separator != std::string::npos) {
      const auto link_name = reference.substr(0U, separator);
      collision_name = reference.substr(separator + 2U);
      link = model_->GetLink(link_name);
    }
    if (!link || collision_name.empty()) {
      return {};
    }

    if (const auto collision = link->GetCollision(collision_name)) {
      return collision;
    }
    // URDF-to-SDF appends `_collision` even when the named URDF collision
    // already ends with that suffix.  A collision belonging to a fixed child
    // link is additionally folded into a name such as
    // `<parent>_fixed_joint_lump__<name>_collision_1`.  Keep the plugin
    // configuration expressed in stable URDF names and recognize both Gazebo
    // normalization forms here.
    const std::vector<std::string> candidate_names{
      collision_name,
      collision_name + "_collision",
    };
    for (const auto & collision : link->GetCollisions()) {
      if (!collision) {
        continue;
      }
      const auto scoped_name = collision->GetScopedName();
      for (const auto & candidate : candidate_names) {
        const auto suffix = std::string("::") + candidate;
        const auto lump_marker =
          std::string("_fixed_joint_lump__") + candidate;
        const auto lump_position = scoped_name.rfind(lump_marker);
        const auto lump_end = lump_position == std::string::npos ?
          std::string::npos : lump_position + lump_marker.size();
        const auto has_valid_lump_suffix =
          lump_end != std::string::npos &&
          (lump_end == scoped_name.size() ||
          (lump_end + 1U < scoped_name.size() &&
          scoped_name[lump_end] == '_' &&
          std::all_of(
            scoped_name.begin() + static_cast<std::ptrdiff_t>(lump_end + 1U),
            scoped_name.end(),
            [](const char character) {
              return character >= '0' && character <= '9';
            })));
        if (collision->GetName() == candidate ||
          scoped_name == candidate ||
          (scoped_name.size() > suffix.size() &&
          scoped_name.compare(
            scoped_name.size() - suffix.size(), suffix.size(), suffix) == 0) ||
          has_valid_lump_suffix)
        {
          return collision;
        }
      }
    }
    return {};
  }

  void set_actuation_collision_enabled(Control & control, bool enabled)
  {
    if (control.actuation_collisions.empty() ||
      control.actuation_collision_suppressed == !enabled)
    {
      return;
    }
    for (const auto & configured : control.actuation_collisions) {
      if (configured.collision) {
        // ODE accepts a contact when either geom's category mask matches the
        // other geom's collide mask. Clearing only collide bits therefore
        // still leaves robot-vs-control contacts active through the opposite
        // half of that test. Disable both masks and restore their exact
        // pre-actuation values once the tool is clear.
        configured.collision->SetCategoryBits(
          enabled ? configured.category_bits : 0u);
        configured.collision->SetCollideBits(
          enabled ? configured.collide_bits : 0u);
      }
    }
    control.actuation_collision_suppressed = !enabled;
    if (enabled) {
      control.collision_restore_not_before = 0.0;
      control.collision_restore_robot_link.reset();
      control.collision_restore_robot_grasp_point.Set(0.0, 0.0, 0.0);
      control.actuation_collision_operation_lease_id.clear();
    }
    if (ros_node_) {
      RCLCPP_INFO(
        ros_node_->get_logger(),
        "%s %zu actuation collision(s) for cabinet control '%s'.",
        enabled ? "Restored" : "Suppressed",
        control.actuation_collisions.size(), control.id.c_str());
    }
  }

  void suppress_actuation_collision(
    Control & control,
    const std::string & operation_lease_id)
  {
    set_actuation_collision_enabled(control, false);
    if (control.actuation_collision_suppressed) {
      control.actuation_collision_operation_lease_id = operation_lease_id;
      control.collision_restore_not_before =
        std::numeric_limits<double>::infinity();
    }
  }

  void schedule_actuation_collision_restore(
    const std::string & control_id,
    const gazebo::physics::LinkPtr & robot_link,
    const ignition::math::Vector3d & robot_grasp_point)
  {
    const auto control_it = control_indices_.find(control_id);
    if (control_it == control_indices_.end()) {
      return;
    }
    auto & control = controls_[control_it->second];
    if (control.actuation_collision_suppressed) {
      control.collision_restore_not_before = world_->SimTime().Double() +
        control.actuation_collision_restore_delay;
      control.collision_restore_robot_link = robot_link;
      control.collision_restore_robot_grasp_point = robot_grasp_point;
    }
  }

  void restore_all_actuation_collisions()
  {
    for (auto & control : controls_) {
      set_actuation_collision_enabled(control, true);
    }
  }

  void publish_grasp_active(bool active) const
  {
    if (!grasp_active_publisher_) {
      return;
    }
    std_msgs::msg::Bool message;
    message.data = active;
    grasp_active_publisher_->publish(message);
  }

  PhysicsOutcome reset_controls()
  {
    const auto release = release_grasp_constraint();
    if (!release.success) {
      return {false,
        "Cabinet reset could not release the active grasp: " +
        release.message,
        std::numeric_limits<double>::quiet_NaN()};
    }
    if (watchdog_recovery_pending_) {
      for (const auto & control : controls_) {
        if (control.actuation_collision_suppressed &&
          !actuation_tool_is_clear(control))
        {
          return {false,
            "Cabinet watchdog recovery is latched for '" + control.id +
            "'. Move the robot tool beyond the configured clearance before "
            "resetting collision masks.",
            std::numeric_limits<double>::quiet_NaN()};
        }
      }
    }
    restore_all_actuation_collisions();
    clear_watchdog_recovery();
    {
      std::lock_guard<std::mutex> lock(active_control_mutex_);
      pregrasp_disturbance_detected_ = false;
      pregrasp_disturbance_control_.clear();
      pregrasp_max_position_error_ = 0.0;
      pregrasp_max_velocity_ = 0.0;
      grasp_engaged_during_operation_ = false;
    }
    for (auto & control : controls_) {
      control.joint->SetForce(0, 0.0);
      control.joint->SetVelocity(0, 0.0);
    }

    // Reset articulated parents before their children.  The switch is a child
    // of the rear door, so moving the door afterwards would perturb an already
    // reset switch in ODE.
    std::vector<Control *> reset_order;
    reset_order.reserve(controls_.size());
    for (auto & control : controls_) {
      reset_order.push_back(&control);
    }
    std::stable_sort(
      reset_order.begin(), reset_order.end(),
      [](const Control * left, const Control * right) {
        const auto rank = [](ControlKind kind) {
          if (kind == ControlKind::kDoor) {
            return 0;
          }
          if (kind == ControlKind::kSwitch) {
            return 1;
          }
          return 2;
        };
        return rank(left->kind) < rank(right->kind);
      });
    for (auto * control : reset_order) {
      // Gazebo/ODE can return false for a nested joint even when it reaches the
      // requested pose.  The physical state is therefore verified below rather
      // than trusting this backend-specific return value alone.
      control->joint->SetPosition(0, control->reset_position, false);
      control->detent_target_index = control->reset_state_index;
      control->slider_released = false;
      if (control->kind == ControlKind::kDrawer) {
        control->drawer_unlocked = false;
        control->drawer_has_opened = false;
      }
    }
    // Moving an articulated parent can transfer velocity back into nested
    // controls in ODE.  Clear every joint again after the complete hierarchy
    // has reached its reset pose so reset returns a stable physical state.
    for (auto & control : controls_) {
      control.joint->SetForce(0, 0.0);
      control.joint->SetVelocity(0, 0.0);
    }

    std::vector<std::string> failed_joints;
    for (auto & control : controls_) {
      control.effort = 0.0;
      update_control_state(control, false);
      const double position_tolerance =
        control.kind == ControlKind::kButton ?
        control.release_threshold : control.motion_tolerance;
      if (control.state_index != control.reset_state_index ||
        std::abs(control.position - control.reset_position) >
        position_tolerance)
      {
        failed_joints.push_back(control.joint_name);
      }
      ++control.transition_sequence;
      publish_control(control);
    }
    if (!failed_joints.empty()) {
      std::ostringstream message;
      message << "Failed to reset cabinet joints:";
      for (const auto & joint : failed_joints) {
        message << ' ' << joint;
      }
      return {false, message.str(), 0.0};
    }
    return {true, "Cabinet physics reset complete.", 0.0};
  }

  PhysicsOutcome run_physics_request(
    const std::shared_ptr<PhysicsRequest> & request)
  {
    auto result = request->promise.get_future();
    {
      std::lock_guard<std::mutex> lock(request_mutex_);
      if (shutting_down_.load() || !update_connection_) {
        return {false, "Cabinet physics plugin is not running.",
          std::numeric_limits<double>::quiet_NaN()};
      }
      requests_.push_back(request);
    }
    if (result.wait_for(kPhysicsRequestTimeout) != std::future_status::ready) {
      auto expected = PhysicsRequest::State::kPending;
      if (request->state.compare_exchange_strong(
          expected, PhysicsRequest::State::kCanceled))
      {
        return {false, "Timed out waiting for the Gazebo physics update; "
          "the request was canceled before execution.",
          std::numeric_limits<double>::quiet_NaN()};
      }
      // Once the physics thread has claimed a request, returning a timeout
      // would allow the attach/reset to happen after the caller saw failure.
      // Wait for the already-started bounded Gazebo operation instead.
      result.wait();
    }
    return result.get();
  }

  PhysicsOutcome submit_physics_request(
    PhysicsRequest::Kind kind,
    std::string control_id,
    std::string operation_lease_id,
    std::string robot_model,
    std::string robot_link,
    ignition::math::Vector3d robot_grasp_point,
    std::string robot_base_link,
    bool attach)
  {
    auto request = std::make_shared<PhysicsRequest>();
    request->kind = kind;
    request->control_id = std::move(control_id);
    request->operation_lease_id = std::move(operation_lease_id);
    request->robot_model = std::move(robot_model);
    request->robot_link = std::move(robot_link);
    request->robot_grasp_point = robot_grasp_point;
    request->robot_base_link = std::move(robot_base_link);
    request->attach = attach;
    if (kind == PhysicsRequest::Kind::kGrasp && attach) {
      std::lock_guard<std::mutex> lock(active_control_mutex_);
      request->active_control_generation = active_control_generation_;
    }
    return run_physics_request(request);
  }

  PhysicsOutcome submit_unlock_request(
    std::string control_id,
    std::string operation_lease_id,
    std::string robot_model,
    std::string right_robot_link,
    ignition::math::Vector3d right_robot_grasp_point,
    bool unlock)
  {
    auto request = std::make_shared<PhysicsRequest>();
    request->kind = PhysicsRequest::Kind::kUnlock;
    request->control_id = std::move(control_id);
    request->operation_lease_id = std::move(operation_lease_id);
    request->robot_model = std::move(robot_model);
    request->right_robot_link = std::move(right_robot_link);
    request->right_robot_grasp_point = right_robot_grasp_point;
    request->drawer_unlock = unlock;
    {
      std::lock_guard<std::mutex> lock(active_control_mutex_);
      request->active_control_generation = active_control_generation_;
    }
    return run_physics_request(request);
  }

  PhysicsOutcome submit_bimanual_request(
    std::string control_id,
    std::string operation_lease_id,
    std::string robot_model,
    std::string left_robot_link,
    std::string right_robot_link,
    ignition::math::Vector3d left_robot_grasp_point,
    ignition::math::Vector3d right_robot_grasp_point,
    std::string robot_base_link,
    bool attach,
    bool base_free)
  {
    auto request = std::make_shared<PhysicsRequest>();
    request->kind = PhysicsRequest::Kind::kBimanualGrasp;
    request->control_id = std::move(control_id);
    request->operation_lease_id = std::move(operation_lease_id);
    request->robot_model = std::move(robot_model);
    request->left_robot_link = std::move(left_robot_link);
    request->right_robot_link = std::move(right_robot_link);
    request->left_robot_grasp_point = left_robot_grasp_point;
    request->right_robot_grasp_point = right_robot_grasp_point;
    request->robot_base_link = std::move(robot_base_link);
    request->attach = attach;
    request->base_free = base_free;
    if (attach) {
      std::lock_guard<std::mutex> lock(active_control_mutex_);
      request->active_control_generation = active_control_generation_;
    }
    return run_physics_request(request);
  }

  void process_physics_requests()
  {
    std::deque<std::shared_ptr<PhysicsRequest>> requests;
    {
      std::lock_guard<std::mutex> lock(request_mutex_);
      requests.swap(requests_);
    }
    for (const auto & request : requests) {
      auto expected = PhysicsRequest::State::kPending;
      if (!request->state.compare_exchange_strong(
          expected, PhysicsRequest::State::kExecuting))
      {
        complete_request(
          request,
          {false, "Gazebo physics request was canceled before execution.",
            std::numeric_limits<double>::quiet_NaN()});
        continue;
      }
      PhysicsOutcome outcome;
      try {
        switch (request->kind) {
          case PhysicsRequest::Kind::kReset:
            outcome = reset_controls();
            break;
          case PhysicsRequest::Kind::kBimanualGrasp:
            outcome = handle_bimanual_grasp_request(*request);
            break;
          case PhysicsRequest::Kind::kUnlock:
            outcome = handle_unlock_request(*request);
            break;
          case PhysicsRequest::Kind::kPlayback:
            outcome = handle_playback_request(*request);
            break;
          default:
            outcome = handle_grasp_request(*request);
            break;
        }
      } catch (const std::exception & error) {
        outcome = {
          false,
          "Gazebo cabinet physics request failed: " +
          std::string(error.what()),
          std::numeric_limits<double>::quiet_NaN()};
      } catch (...) {
        outcome = {
          false,
          "Gazebo cabinet physics request failed with an unknown error.",
          std::numeric_limits<double>::quiet_NaN()};
      }
      complete_request(request, std::move(outcome));
    }
  }

  // Visual playback (2026-09-06): drives one drawer's prismatic rail joint
  // along its (time, q) schedule.  Called from on_update for a control whose
  // playback_active flag is set, so latch/spring/coupling physics and the
  // pre-grasp disturbance detector are already bypassed.  Positions are set in
  // place and the velocity zeroed so the ODE rail does not accumulate a
  // corrective jump; a single tick is clamped so a long world stall cannot
  // teleport the drawer.
  void drive_drawer_playback(Control & control, double simulation_time)
  {
    if (control.playback_last_sim >= 0.0) {
      const double dt = simulation_time - control.playback_last_sim;
      if (!control.playback_paused && !control.playback_finished &&
        dt > 0.0 && control.playback_samples.size() >= 2U)
      {
        control.playback_elapsed += std::min(dt, 0.25);
      }
    }
    control.playback_last_sim = simulation_time;

    double target = control.playback_samples.back().second;
    if (control.playback_samples.size() >= 2U) {
      const double total = control.playback_samples.back().first;
      if (control.playback_elapsed < total) {
        const auto upper = std::upper_bound(
          control.playback_samples.begin(), control.playback_samples.end(),
          control.playback_elapsed,
          [](double t, const std::pair<double, double> & sample) {
            return t < sample.first;
          });
        if (upper == control.playback_samples.end()) {
          target = control.playback_samples.back().second;
        } else if (upper == control.playback_samples.begin()) {
          target = control.playback_samples.front().second;
        } else {
          const auto & previous = *(upper - 1);
          const double span = upper->first - previous.first;
          const double factor = span > 0.0 ?
            (control.playback_elapsed - previous.first) / span : 0.0;
          target = previous.second + factor * (upper->second - previous.second);
        }
      } else {
        control.playback_finished = true;
        control.playback_elapsed = total;
        target = control.playback_samples.back().second;
      }
    }

    control.joint->SetPosition(0, target);
    control.joint->SetVelocity(0, 0.0);
    control.effort = 0.0;
    update_control_state(control, true);
  }

  PhysicsOutcome handle_playback_request(const PhysicsRequest & request)
  {
    const auto it = control_indices_.find(request.control_id);
    if (it == control_indices_.end()) {
      return {false, "Unknown drawer playback control: " + request.control_id,
        std::numeric_limits<double>::quiet_NaN()};
    }
    Control & control = controls_[it->second];
    if (control.kind != ControlKind::kDrawer) {
      return {false, "Control '" + request.control_id +
        "' is not a drawer and does not accept visual playback.",
        std::numeric_limits<double>::quiet_NaN()};
    }
    if (request.operation_lease_id.empty()) {
      return {false, "Drawer '" + request.control_id +
        "' playback requires a non-empty operation_lease_id.",
        std::numeric_limits<double>::quiet_NaN()};
    }
    if (control.playback_active &&
      control.playback_lease_id != request.operation_lease_id)
    {
      return {false, "Drawer '" + request.control_id +
        "' playback is owned by another operation lease.",
        std::numeric_limits<double>::quiet_NaN()};
    }
    // A physical grasp/coupling session on this drawer wins over playback.
    if ((grasp_is_active() || bimanual_grasp_is_active()) &&
      active_grasp_control_ == request.control_id)
    {
      return {false, "Drawer '" + request.control_id +
        "' is physically grasped; playback refused.",
        std::numeric_limits<double>::quiet_NaN()};
    }

    if (request.playback_command == PhysicsRequest::kPlaybackRelease) {
      const bool was_finished = control.playback_finished;
      control.playback_active = false;
      control.playback_paused = false;
      control.playback_finished = false;
      control.playback_lease_id.clear();
      control.playback_samples.clear();
      control.playback_last_sim = -1.0;
      PhysicsOutcome outcome{
        true, "Drawer '" + request.control_id + "' playback released.",
        control.joint->Position(0)};
      outcome.finished = was_finished;
      return outcome;
    }
    if (request.playback_command == PhysicsRequest::kPlaybackHold) {
      if (!control.playback_active) {
        return {true, "Drawer '" + request.control_id +
          "' has no active playback; nothing to hold.",
          control.joint->Position(0)};
      }
      control.playback_paused = true;
      return {true, "Drawer '" + request.control_id +
        "' playback paused at the current rail position.",
        control.joint->Position(0)};
    }
    if (request.playback_command != PhysicsRequest::kPlaybackStart) {
      return {false, "Drawer '" + request.control_id +
        "' playback received an unknown command.",
        control.joint->Position(0)};
    }

    // START: validate and adopt the schedule.
    const double lower = control.joint->LowerLimit(0);
    const double upper = control.joint->UpperLimit(0);
    double previous_time = -1.0;
    bool starts_at_zero = false;
    std::vector<std::pair<double, double>> samples;
    samples.reserve(request.playback_samples.size());
    for (const auto & sample : request.playback_samples) {
      const double t = sample.first;
      const double q = sample.second;
      if (!std::isfinite(t) || !std::isfinite(q) || t < 0.0 ||
        t <= previous_time || q < lower || q > upper)
      {
        return {false, "Drawer '" + request.control_id +
          "' playback trajectory rejected: samples must be finite, have "
          "strictly increasing time_from_start >= 0, and keep the rail "
          "position inside the drawer joint limits.",
          std::numeric_limits<double>::quiet_NaN()};
      }
      if (std::abs(t) <= 1e-9) {
        starts_at_zero = true;
      }
      previous_time = t;
      samples.push_back(sample);
    }
    if (samples.size() < 2U || !starts_at_zero ||
      samples.size() > kPlaybackMaxSamples)
    {
      return {false, "Drawer '" + request.control_id +
        "' playback needs 2.." + std::to_string(kPlaybackMaxSamples) +
        " samples with the first exactly at t=0.",
        std::numeric_limits<double>::quiet_NaN()};
    }
    const double current = control.joint->Position(0);
    if (std::abs(samples.front().second - current) > kPlaybackStartWindow) {
      return {false, "Drawer '" + request.control_id +
        "' playback START refused: the schedule starts " +
        std::to_string(samples.front().second - current) +
        " m from the current rail position (allowed window " +
        std::to_string(kPlaybackStartWindow) + " m).",
        current};
    }

    control.playback_samples = std::move(samples);
    control.playback_active = true;
    control.playback_paused = false;
    control.playback_finished = false;
    control.playback_lease_id = request.operation_lease_id;
    control.playback_elapsed = 0.0;
    control.playback_last_sim = world_->SimTime().Double();
    // The rail is now kinematic; reflect that in the latch state so a later
    // RELEASE at the closed position re-engages normally and a mid-travel
    // RELEASE is not fought by the closed-detent latch.
    control.drawer_unlocked = true;
    control.drawer_has_opened = true;
    PhysicsOutcome outcome{
      true, "Drawer '" + request.control_id + "' playback started.",
      current};
    outcome.finished = control.playback_samples.empty();
    return outcome;
  }

  PhysicsOutcome recover_failed_grasp_attach(
    Control & control,
    const PhysicsRequest & request,
    const gazebo::physics::LinkPtr & robot_link,
    const std::string & failure,
    double distance) noexcept
  {
    bool release_completed = false;
    std::string cleanup_failure;
    try {
      const auto release = release_grasp_constraint();
      release_completed = release.success;
      if (!release.success) {
        cleanup_failure = release.message;
      }
    } catch (const std::exception & error) {
      cleanup_failure = error.what();
    } catch (...) {
      cleanup_failure = "unknown Gazebo release exception";
    }

    if (!release_completed && grasp_is_active()) {
      // If a partially-created joint remains, keep/establish the same narrow
      // actuation-collision suppression used by a normal grasp.  Never restore
      // that mask while the failed physical constraint may still form an ODE
      // closed loop.
      try {
        suppress_actuation_collision(control, request.operation_lease_id);
      } catch (const std::exception & error) {
        cleanup_failure += "; collision suppression also failed: ";
        cleanup_failure += error.what();
      } catch (...) {
        cleanup_failure +=
          "; collision suppression also failed unexpectedly";
      }
    }
    if (control.actuation_collision_suppressed) {
      // Init() can throw before active_robot_link_ptr_ is committed, or release
      // can remove the grasp joint and then fail on the base brake.  Preserve
      // the already-resolved request probe explicitly so a later successful
      // retry can still prove real clearance and complete collision recovery.
      schedule_actuation_collision_restore(
        control.id, robot_link, request.robot_grasp_point);
    }

    if (!release_completed || control.actuation_collision_suppressed) {
      const auto now = std::chrono::steady_clock::now();
      // The caller holds active_control_mutex_ across the attach commit and
      // this cleanup path.  Permanently reject this failed lease session before
      // publishing its fault so delayed heartbeats/requests cannot resurrect
      // a partially recovered grasp.
      remember_expired_operation_lease_locked(request.operation_lease_id);
      active_control_received_ = true;
      active_operation_control_.clear();
      ++active_control_generation_;
      operation_heartbeat_received_ = false;
      operation_heartbeat_lease_id_.clear();
      operation_last_heartbeat_ =
        std::chrono::steady_clock::time_point{};
      ++operation_heartbeat_sequence_;
      reset_pregrasp_tracking_locked();
      watchdog_recovery_pending_ = true;
      watchdog_release_completed_ = release_completed;
      watchdog_recovery_control_ = control.id;
      watchdog_recovery_lease_id_ = request.operation_lease_id;
      watchdog_release_retry_not_before_ = release_completed ?
        std::chrono::steady_clock::time_point{} :
      now + kWatchdogReleaseRetryDelay;
      if (!release_completed) {
        log_watchdog_release_failure(cleanup_failure, now);
      }
      publish_operation_fault(request.operation_lease_id);
    }

    return {false,
      "Failed to create cabinet grasp constraint: " + failure +
      (cleanup_failure.empty() ? "" :
      "; fail-safe cleanup is pending: " + cleanup_failure),
      distance};
  }

  PhysicsOutcome recover_failed_bimanual_attach(
    Control & control,
    const PhysicsRequest & request,
    double left_distance,
    double right_distance,
    const std::string & failure) noexcept
  {
    bool release_completed = false;
    std::string cleanup_failure;
    try {
      const auto release = release_bimanual_grasp_constraint();
      release_completed = release.success;
      if (!release.success) {
        cleanup_failure = release.message;
      }
    } catch (const std::exception & error) {
      cleanup_failure = error.what();
    } catch (...) {
      cleanup_failure = "unknown Gazebo bimanual release exception";
    }

    if (!release_completed) {
      // The caller holds active_control_mutex_ across the attach commit and
      // this cleanup path.  Permanently reject this failed lease session before
      // publishing its fault so delayed heartbeats/requests cannot resurrect a
      // partially recovered bimanual grasp.
      const auto now = std::chrono::steady_clock::now();
      remember_expired_operation_lease_locked(request.operation_lease_id);
      active_control_received_ = true;
      active_operation_control_.clear();
      ++active_control_generation_;
      operation_heartbeat_received_ = false;
      operation_heartbeat_lease_id_.clear();
      operation_last_heartbeat_ =
        std::chrono::steady_clock::time_point{};
      ++operation_heartbeat_sequence_;
      reset_pregrasp_tracking_locked();
      watchdog_recovery_pending_ = true;
      watchdog_release_completed_ = release_completed;
      watchdog_recovery_control_ = control.id;
      watchdog_recovery_lease_id_ = request.operation_lease_id;
      watchdog_release_retry_not_before_ = now + kWatchdogReleaseRetryDelay;
      log_watchdog_release_failure(cleanup_failure, now);
      publish_operation_fault(request.operation_lease_id);
    }

    PhysicsOutcome outcome;
    outcome.success = false;
    outcome.message =
      "Failed to create bimanual drawer grasp constraint: " + failure +
      (cleanup_failure.empty() ? "" :
      "; fail-safe cleanup is pending: " + cleanup_failure);
    outcome.left_distance = left_distance;
    outcome.right_distance = right_distance;
    outcome.left_tool_contact = std::isfinite(left_distance) &&
      left_distance <= control.grasp_contact_threshold;
    outcome.right_tool_contact = std::isfinite(right_distance) &&
      right_distance <= control.grasp_contact_threshold;
    return outcome;
  }

  PhysicsOutcome handle_unlock_request(const PhysicsRequest & request)
  {
    const auto control_it = control_indices_.find(request.control_id);
    if (control_it == control_indices_.end()) {
      return {false, "Unknown drawer unlock control: " + request.control_id,
        std::numeric_limits<double>::quiet_NaN()};
    }
    auto & control = controls_[control_it->second];
    if (control.kind != ControlKind::kDrawer) {
      return {false, "Unlock is only supported for drawer controls: " + control.id,
        std::numeric_limits<double>::quiet_NaN()};
    }
    if (!request.drawer_unlock) {
      // Explicit re-lock used by cancel / reset cleanup.  It only tightens a
      // rail latch, so it is always safe and needs no session authority.
      control.drawer_unlocked = false;
      control.drawer_has_opened = false;
      control.detent_target_index = control.reset_state_index;
      RCLCPP_INFO(
        ros_node_->get_logger(),
        "Drawer '%s' rail latch re-engaged by explicit unlock=false.",
        control.id.c_str());
      return {true, "Drawer rail latch re-engaged.", 0.0};
    }

    if (request.operation_lease_id.empty()) {
      return {false,
        "Drawer unlock requires a non-empty operation lease identity.",
        std::numeric_limits<double>::quiet_NaN()};
    }
    if (watchdog_recovery_pending_) {
      return {false,
        "Cabinet watchdog recovery is still releasing the previous physical "
        "operation; an unlock is not yet safe.",
        std::numeric_limits<double>::quiet_NaN()};
    }
    if (bimanual_grasp_is_active() && active_grasp_control_ == control.id) {
      return {false,
        "The drawer is already grasped; unlock must happen before attach.",
        std::numeric_limits<double>::quiet_NaN()};
    }
    if (!robot_grasp_point_is_finite(
        request.right_robot_grasp_point.X(),
        request.right_robot_grasp_point.Y(),
        request.right_robot_grasp_point.Z()))
    {
      return {false,
        "Right robot grasp point must contain three finite link-local "
        "coordinates.",
        std::numeric_limits<double>::quiet_NaN()};
    }
    {
      std::lock_guard<std::mutex> lock(active_control_mutex_);
      if (!operation_heartbeat_authorizes_grasp_locked(request)) {
        return {false,
          "Drawer unlock requires a fresh heartbeat from the exact active "
          "control and global operation lease session.",
          std::numeric_limits<double>::quiet_NaN()};
      }
    }
    if (request.robot_model.empty() || request.right_robot_link.empty()) {
      return {false,
        "robot_model and right_robot_link are required for unlock.",
        std::numeric_limits<double>::quiet_NaN()};
    }
    const auto robot_model = world_->ModelByName(request.robot_model);
    if (!robot_model || robot_model == model_) {
      return {false, "Robot model was not found or is invalid: " +
        request.robot_model, std::numeric_limits<double>::quiet_NaN()};
    }
    const auto right_link = robot_model->GetLink(request.right_robot_link);
    if (!right_link) {
      return {false, "Robot right link was not found: " +
        request.right_robot_link, std::numeric_limits<double>::quiet_NaN()};
    }

    const bool sim_linkage = control.unlock_simulated_linkage;

    // 2026-09-03 AGENT §3: unlock_press_point is the centre of the right-handle
    // LOGICAL unlock zone (drawer link frame), so the distance check tracks the
    // drawer whether it is open or closed.  STRICT mode (the default) requires
    // the unlock motor's contact link tip (the robot link the request names)
    // within unlock_distance_threshold of that centre AND the robot-model
    // unlock motor joint actually extended (position in
    // [retracted+floor, ceiling)) — real physical evidence read under the
    // physics lock, not a free-position claim, and no b1p indicator
    // displacement is involved.
    const auto target_pose = control.link->WorldPose();
    const auto unlock_press_world = target_pose.Pos() +
      target_pose.Rot().RotateVector(control.unlock_press_point);
    const auto right_pose = right_link->WorldPose();
    const auto right_tool_world = right_pose.Pos() +
      right_pose.Rot().RotateVector(request.right_robot_grasp_point);
    const double distance = right_tool_world.Distance(unlock_press_world);
    const bool right_tool_contact =
      std::isfinite(distance) &&
      distance <= control.unlock_distance_threshold;

    gazebo::physics::JointPtr motor_joint;
    if (!control.unlock_motor_joint.empty()) {
      motor_joint = robot_model->GetJoint(control.unlock_motor_joint);
    }
    const double motor_position =
      motor_joint ? motor_joint->Position(0) : 0.0;
    const bool motor_over_ceiling = motor_joint != nullptr &&
      motor_position >= control.unlock_extension_ceiling;
    const bool pressed = motor_joint != nullptr &&
      motor_position >= control.unlock_retracted_position +
        control.unlock_extension_floor &&
      !motor_over_ceiling;

    if (!sim_linkage && !right_tool_contact) {
      // STRICT physical contact refusal (default mode) — unchanged behaviour.
      PhysicsOutcome outcome;
      outcome.success = false;
      outcome.message =
        "Unlock contact link is not inside the unlock zone (distance to "
        "unlock zone centre " + std::to_string(distance) + " m exceeds "
        "threshold " + std::to_string(control.unlock_distance_threshold) +
        " m).";
      outcome.distance = distance;
      outcome.right_tool_contact = false;
      outcome.pressed = pressed;
      return outcome;
    }
    if (!pressed) {
      // Both modes require the REAL unlock-motor stroke.
      PhysicsOutcome outcome;
      outcome.success = false;
      outcome.unlock_mode = sim_linkage ? "simulated_linkage" : "strict";
      const std::string prefix =
        sim_linkage ? "simulated_linkage unlock refused: " : "";
      if (control.unlock_motor_joint.empty()) {
        outcome.message = prefix +
          "Drawer unlock has no configured <unlock_motor_joint>: " +
          control.id;
      } else if (!motor_joint) {
        outcome.message = prefix +
          "Unlock motor joint was not found on the robot model: " +
          control.unlock_motor_joint;
      } else if (motor_over_ceiling) {
        outcome.message = prefix +
          "unlock motor is over its extension ceiling (position " +
          std::to_string(motor_position) + " m >= ceiling " +
          std::to_string(control.unlock_extension_ceiling) + " m); "
          "unlock refused - the motor must not bottom out.";
      } else {
        outcome.message = prefix +
          "the unlock motor is not extended (joint position " +
          std::to_string(motor_position) + " m < retracted " +
          std::to_string(control.unlock_retracted_position) + " + floor " +
          std::to_string(control.unlock_extension_floor) + " m).";
      }
      outcome.distance = distance;
      // In strict mode the tool tip already passed the zone check; in
      // simulated_linkage mode finger3 is NOT required at the button.
      outcome.right_tool_contact = !sim_linkage;
      outcome.pressed = false;
      return outcome;
    }

    if (sim_linkage) {
      // simulated_linkage (db1 sim config only): the visible finger3 rod may be
      // geometrically incapable of reaching the button (the drawer's logical
      // unlock zone sits on the right-handle plate that the RIGHT HOOK rod
      // holds, 0.11+ m lateral from finger3's swept path).  This mode models
      // "the right unlock motor drives the right-handle latch release through
      // an UNMODELLED linkage": the button stays on the right handle, the tool
      // geometry is unchanged, and finger3's real tip is NOT claimed at the
      // button.  Granting still needs BOTH handles physically held — measured
      // independently here, exactly like a bimanual attach: the two hook-rod
      // tips named by the control config must be within grasp_contact_threshold
      // of the drawer's left/right handle grasp points.  Nothing is faked with
      // SetEntityState and no Web target can unlatch on its own.
      const auto left_hold_link =
        robot_model->GetLink(control.unlock_hold_left_link);
      const auto right_hold_link =
        robot_model->GetLink(control.unlock_hold_right_link);
      if (!left_hold_link || !right_hold_link) {
        PhysicsOutcome outcome;
        outcome.success = false;
        outcome.unlock_mode = "simulated_linkage";
        outcome.message =
          "simulated_linkage unlock refused: handle-hold robot links "
          "<unlock_hold_left_link> '" + control.unlock_hold_left_link +
          "' and/or <unlock_hold_right_link> '" +
          control.unlock_hold_right_link + "' were not found on the robot "
          "model.";
        outcome.distance = distance;
        outcome.pressed = true;
        return outcome;
      }
      const auto left_hold_world = left_hold_link->WorldPose().Pos() +
        left_hold_link->WorldPose().Rot().RotateVector(
          control.unlock_hold_left_point);
      const auto left_handle_world = target_pose.Pos() +
        target_pose.Rot().RotateVector(control.grasp_point);
      const auto right_hold_world = right_hold_link->WorldPose().Pos() +
        right_hold_link->WorldPose().Rot().RotateVector(
          control.unlock_hold_right_point);
      const auto right_handle_world = target_pose.Pos() +
        target_pose.Rot().RotateVector(control.right_grasp_point);
      const double left_hold_distance =
        left_hold_world.Distance(left_handle_world);
      const double right_hold_distance =
        right_hold_world.Distance(right_handle_world);
      const bool left_handle_held = std::isfinite(left_hold_distance) &&
        left_hold_distance <= control.grasp_contact_threshold;
      const bool right_handle_held = std::isfinite(right_hold_distance) &&
        right_hold_distance <= control.grasp_contact_threshold;
      if (!left_handle_held || !right_handle_held) {
        PhysicsOutcome outcome;
        outcome.success = false;
        outcome.unlock_mode = "simulated_linkage";
        outcome.message =
          "simulated_linkage unlock refused: the drawer handles are not both "
          "held (left hook rod '" + control.unlock_hold_left_link + "' tip " +
          std::to_string(left_hold_distance) + " m from the left handle grasp "
          "point, right hook rod '" + control.unlock_hold_right_link + "' tip " +
          std::to_string(right_hold_distance) + " m from the right handle "
          "grasp point; grasp_contact_threshold " +
          std::to_string(control.grasp_contact_threshold) + " m).";
        outcome.distance = distance;
        outcome.pressed = true;
        outcome.left_hold_distance = left_hold_distance;
        outcome.right_hold_distance = right_hold_distance;
        outcome.left_handle_held = left_handle_held;
        outcome.right_handle_held = right_handle_held;
        return outcome;
      }
    }

    if (control.drawer_unlocked) {
      PhysicsOutcome outcome;
      outcome.success = true;
      outcome.message = sim_linkage ?
        "Drawer is already unlocked (simulated_linkage)." :
        "Drawer is already unlocked.";
      outcome.distance = distance;
      outcome.right_tool_contact = !sim_linkage;
      outcome.pressed = true;
      outcome.unlock_mode = sim_linkage ? "simulated_linkage" : "strict";
      return outcome;
    }
    control.drawer_unlocked = true;
    control.drawer_has_opened = false;
    control.detent_target_index = control.reset_state_index;
    {
      // 2026-09-02: 解锁成功 = operator 已物理接管该抽屉。此前锁定阶段闩存
      // 的"预抓取扰动"（如解锁按压行程把抽屉轻推离闩位）不再构成拦截——否则
      // 随后的支撑/抓取必被误拒（run 实测 latched vel 0.050265 > tol 0.05）。
      std::lock_guard<std::mutex> lock(active_control_mutex_);
      pregrasp_disturbance_detected_ = false;
      pregrasp_disturbance_control_.clear();
      pregrasp_max_position_error_ = 0.0;
      pregrasp_max_velocity_ = 0.0;
    }
    PhysicsOutcome outcome;
    outcome.success = true;
    outcome.pressed = true;
    outcome.distance = distance;
    if (sim_linkage) {
      // Measure the handle-hold evidence for the acceptance record even though
      // the gates already passed (the request is granted on this branch).
      const auto left_hold_link =
        robot_model->GetLink(control.unlock_hold_left_link);
      const auto right_hold_link =
        robot_model->GetLink(control.unlock_hold_right_link);
      if (left_hold_link) {
        const auto left_hold_world = left_hold_link->WorldPose().Pos() +
          left_hold_link->WorldPose().Rot().RotateVector(
            control.unlock_hold_left_point);
        outcome.left_hold_distance =
          left_hold_world.Distance(target_pose.Pos() +
            target_pose.Rot().RotateVector(control.grasp_point));
        outcome.left_handle_held = std::isfinite(
          outcome.left_hold_distance) &&
          outcome.left_hold_distance <= control.grasp_contact_threshold;
      }
      if (right_hold_link) {
        const auto right_hold_world = right_hold_link->WorldPose().Pos() +
          right_hold_link->WorldPose().Rot().RotateVector(
            control.unlock_hold_right_point);
        outcome.right_hold_distance =
          right_hold_world.Distance(target_pose.Pos() +
            target_pose.Rot().RotateVector(control.right_grasp_point));
        outcome.right_handle_held = std::isfinite(
          outcome.right_hold_distance) &&
          outcome.right_hold_distance <= control.grasp_contact_threshold;
      }
      outcome.unlock_mode = "simulated_linkage";
      // simulation_acceptance marks an actual simulated-linkage grant so no
      // consumer can ever report this as "finger3 real tip pressed the button".
      outcome.simulation_acceptance = true;
      outcome.right_tool_contact = false;
      const std::string label =
        control.unlock_linkage_description.empty() ?
        "right unlock motor drives the right-handle latch release through an "
        "unmodelled linkage; finger3 real tip is NOT at the button" :
        control.unlock_linkage_description;
      RCLCPP_WARN(
        ros_node_->get_logger(),
        "Drawer '%s' rail latch released under AGENT doc §4.2 "
        "simulated_linkage: unlock motor '%s' REALLY extended to %.6f m "
        "(simulated) '%s'; finger3 real tip %.6f m from the button zone. Left "
        "handle hold %.6f m, right handle hold %.6f m (grasp_contact_threshold "
        "%.4f m).",
        control.id.c_str(), control.unlock_motor_joint.c_str(),
        motor_position, label.c_str(), distance,
        outcome.left_hold_distance, outcome.right_hold_distance,
        control.grasp_contact_threshold);
      outcome.message =
        "Drawer unlocked (simulated_linkage: " + label + "; unlock motor '" +
        control.unlock_motor_joint + "' real stroke " +
        std::to_string(motor_position) + " m; left handle hold " +
        std::to_string(outcome.left_hold_distance) + " m, right handle hold " +
        std::to_string(outcome.right_hold_distance) + " m; threshold " +
        std::to_string(control.grasp_contact_threshold) + " m).";
      return outcome;
    }
    RCLCPP_INFO(
      ros_node_->get_logger(),
      "Drawer '%s' rail latch released: unlock motor '%s' extended to %.6f m "
      "with its contact link %.6f m from the unlock zone centre.",
      control.id.c_str(), control.unlock_motor_joint.c_str(),
      motor_position, distance);
    outcome.unlock_mode = "strict";
    outcome.message = "Drawer unlocked.";
    outcome.right_tool_contact = true;
    return outcome;
  }

  PhysicsOutcome handle_bimanual_grasp_request(const PhysicsRequest & request)
  {
    if (!robot_grasp_point_is_finite(
        request.left_robot_grasp_point.X(),
        request.left_robot_grasp_point.Y(),
        request.left_robot_grasp_point.Z()) ||
      !robot_grasp_point_is_finite(
        request.right_robot_grasp_point.X(),
        request.right_robot_grasp_point.Y(),
        request.right_robot_grasp_point.Z()))
    {
      return {false,
        "Both robot grasp points must contain three finite link-local "
        "coordinates.",
        std::numeric_limits<double>::quiet_NaN()};
    }
    const auto control_it = control_indices_.find(request.control_id);
    if (control_it == control_indices_.end()) {
      return {false, "Unknown cabinet bimanual grasp control: " +
        request.control_id, std::numeric_limits<double>::quiet_NaN()};
    }
    auto & control = controls_[control_it->second];
    if (control.kind != ControlKind::kDrawer) {
      return {false,
        "Bimanual grasp is only supported for drawer controls: " + control.id,
        std::numeric_limits<double>::quiet_NaN()};
    }

    if (!request.attach) {
      if (!bimanual_grasp_is_active()) {
        return {true, "Bimanual drawer grasp is already released.", 0.0};
      }
      if (active_grasp_control_ != control.id) {
        return {false,
          "A different cabinet control is currently grasped: " +
          active_grasp_control_,
          std::numeric_limits<double>::quiet_NaN()};
      }
      if (!operation_session_matches(
          active_grasp_control_, active_grasp_lease_id_, control.id,
          request.operation_lease_id))
      {
        return {false,
          "Bimanual drawer grasp release requires the exact active operation "
          "lease identity.",
          std::numeric_limits<double>::quiet_NaN()};
      }
      return release_bimanual_grasp_constraint();
    }

    if (request.operation_lease_id.empty()) {
      return {false,
        "Bimanual drawer grasp attach requires a non-empty operation lease "
        "identity.",
        std::numeric_limits<double>::quiet_NaN()};
    }
    if (watchdog_recovery_pending_) {
      return {false,
        "Cabinet watchdog recovery is still releasing the previous physical "
        "operation; a new grasp is not yet safe.",
        std::numeric_limits<double>::quiet_NaN()};
    }
    if (!control.drawer_unlocked) {
      return {false,
        "Drawer is locked; unlock it first (SetCabinetUnlock): " + control.id,
        std::numeric_limits<double>::quiet_NaN()};
    }
    {
      std::lock_guard<std::mutex> lock(active_control_mutex_);
      if (!operation_heartbeat_authorizes_grasp_locked(request)) {
        return {false,
          "Bimanual drawer grasp attach requires a fresh heartbeat from the "
          "exact active control and global operation lease session.",
          std::numeric_limits<double>::quiet_NaN()};
      }
      if (grasp_is_active() || base_brake_joint_) {
        if (active_grasp_control_ == control.id &&
          active_grasp_lease_id_ == request.operation_lease_id &&
          active_robot_model_ == request.robot_model &&
          active_left_robot_link_ == request.left_robot_link &&
          active_right_robot_link_ == request.right_robot_link)
        {
          PhysicsOutcome outcome;
          outcome.success = true;
          outcome.message = "Bimanual drawer grasp is already attached.";
          outcome.left_distance = 0.0;
          outcome.right_distance = 0.0;
          outcome.left_tool_contact = true;
          outcome.right_tool_contact = true;
          return outcome;
        }
        return {false,
          "Another cabinet grasp or operation lease is active: " +
          active_grasp_control_,
          std::numeric_limits<double>::quiet_NaN()};
      }
    }
    if (request.robot_model.empty() || request.left_robot_link.empty() ||
      request.right_robot_link.empty() || request.robot_base_link.empty())
    {
      return {false,
        "robot_model, left_robot_link, right_robot_link and robot_base_link "
        "are required for bimanual attach.",
        std::numeric_limits<double>::quiet_NaN()};
    }

    // 2026-09-02: 预抓取扰动守卫只护"锁定"控件。此处能到达必已通过上方
    // drawer_unlocked 前置检查，即抽屉已解锁、归 operator 接管——自由抽屉
    // 在解锁按压/回退/支撑后的位置与运动均为操作内容（且位姿已按实测刷新），
    // 不得再因"扰动"拒抓。守卫保留为防御（若将来放松 2800 的解锁前置）。
    if (!control.drawer_unlocked) {
      const double raw_position = control.joint->Position(0);
      const double raw_velocity = control.joint->GetVelocity(0);
      const double latched_detent =
        control.detents.at(control.detent_target_index);
      bool approach_disturbed = pregrasp_detent_is_disturbed(
        raw_position, raw_velocity, latched_detent,
        control.motion_tolerance, control.motion_tolerance);
      double maximum_position_error = std::abs(raw_position - latched_detent);
      double maximum_velocity = std::abs(raw_velocity);
      {
        std::lock_guard<std::mutex> lock(active_control_mutex_);
        if (pregrasp_disturbance_detected_ &&
          pregrasp_disturbance_control_ == control.id)
        {
          approach_disturbed = true;
          maximum_position_error = std::max(
            maximum_position_error, pregrasp_max_position_error_);
          maximum_velocity = std::max(
            maximum_velocity, pregrasp_max_velocity_);
        }
      }
      if (approach_disturbed) {
        return {false,
          "Unsafe pre-grasp movement was detected for cabinet drawer '" +
          control.id + "' (maximum detent error " +
          std::to_string(maximum_position_error) +
          " m, maximum velocity " + std::to_string(maximum_velocity) +
          " m/s). The ready/approach path may have contacted the drawer.",
          std::numeric_limits<double>::quiet_NaN()};
      }
    }

    const auto robot_model = world_->ModelByName(request.robot_model);
    if (!robot_model || robot_model == model_) {
      return {false, "Robot model was not found or is invalid: " +
        request.robot_model, std::numeric_limits<double>::quiet_NaN()};
    }
    const auto left_link = robot_model->GetLink(request.left_robot_link);
    if (!left_link) {
      return {false, "Robot left link was not found: " +
        request.left_robot_link, std::numeric_limits<double>::quiet_NaN()};
    }
    const auto right_link = robot_model->GetLink(request.right_robot_link);
    if (!right_link) {
      return {false, "Robot right link was not found: " +
        request.right_robot_link, std::numeric_limits<double>::quiet_NaN()};
    }
    const auto robot_base_link = robot_model->GetLink(request.robot_base_link);
    if (!robot_base_link) {
      return {false, "Robot base brake link was not found: " +
        request.robot_base_link, std::numeric_limits<double>::quiet_NaN()};
    }

    // Both tools must be simultaneously within the drawer's tight grasp
    // contact threshold of their handle (2026-09-02: no more hovering — the
    // tips must actually contact the handles, 2mm pressed into the plates via
    // the operator's press depth).  A single-side clamp would rack the drawer
    // under the pull torque, so the attach is rejected unless BOTH distances
    // pass, and the per-side contact flags are reported to the caller.
    const auto target_pose = control.link->WorldPose();
    const auto left_handle_world = target_pose.Pos() +
      target_pose.Rot().RotateVector(control.grasp_point);
    const auto right_handle_world = target_pose.Pos() +
      target_pose.Rot().RotateVector(control.right_grasp_point);
    const auto left_pose = left_link->WorldPose();
    const auto left_tool_world = left_pose.Pos() +
      left_pose.Rot().RotateVector(request.left_robot_grasp_point);
    const auto right_pose = right_link->WorldPose();
    const auto right_tool_world = right_pose.Pos() +
      right_pose.Rot().RotateVector(request.right_robot_grasp_point);
    const double left_distance = left_tool_world.Distance(left_handle_world);
    const double right_distance = right_tool_world.Distance(right_handle_world);
    const bool left_tool_contact = std::isfinite(left_distance) &&
      left_distance <= control.grasp_contact_threshold;
    const bool right_tool_contact = std::isfinite(right_distance) &&
      right_distance <= control.grasp_contact_threshold;
    if (!left_tool_contact || !right_tool_contact)
    {
      PhysicsOutcome outcome;
      outcome.success = false;
      outcome.message =
        "Bimanual drawer grasp requires BOTH tools actually contacting their "
        "handles (left " + std::to_string(left_distance) + " m, right " +
        std::to_string(right_distance) + " m; grasp_contact_threshold " +
        std::to_string(control.grasp_contact_threshold) + " m); single-side "
        "or hovering attach is rejected.";
      outcome.left_distance = left_distance;
      outcome.right_distance = right_distance;
      outcome.left_tool_contact = left_tool_contact;
      outcome.right_tool_contact = right_tool_contact;
      return outcome;
    }

    // Revalidate immediately before the irreversible Gazebo operations and
    // hold the short session lock through commit.
    std::unique_lock<std::mutex> active_session_lock(active_control_mutex_);
    if (!operation_heartbeat_authorizes_grasp_locked(request)) {
      return {false,
        "Bimanual drawer grasp lease/heartbeat changed before physical "
        "attachment.",
        std::numeric_limits<double>::quiet_NaN(), left_distance, right_distance};
    }
    try {
      // Anchor the chassis for the lifetime of the physical grasp so the pull
      // reaction cannot drag the whole robot away from the world-frame path.
      // P3-8 grab-and-drive: the caller declared base_free=true because the
      // pull translates the BASE along the drawer axis (arms hold their grasp
      // config), so the chassis brake is skipped or the base could not move.
      if (!request.base_free) {
        base_brake_joint_ = model_->CreateJoint(
          kRuntimeBaseBrakeJointName, "fixed", gazebo::physics::LinkPtr(),
          robot_base_link);
        if (!base_brake_joint_) {
          return {false, "Gazebo could not create the robot base brake joint.",
            std::numeric_limits<double>::quiet_NaN(), left_distance, right_distance};
        }
        base_brake_joint_->Init();
      }
      if (control.grasp_coupling_stiffness > 0.0) {
        // Linear-drag coupling mode: do NOT weld the tools to the drawer.  The
        // coupling drives the rail toward the tools' commanded motion.  run8
        // root cause: a hard cross-model fixed joint + kinematic arm teleport
        // + 80 kg drawer inertia is a closed chain that flings the ~1.9 kg tool
        // west every controller cycle until the arm controllers abort with
        // PATH_TOLERANCE_VIOLATED.  Here the reaction lands on the cabinet rail
        // instead of the tools, so the teleported tools track perfectly.
        ignition::math::Vector3d axis = control.joint->GlobalAxis(0);
        axis.Normalize();
        const auto left_pose = left_link->WorldPose().Pos();
        const auto right_pose = right_link->WorldPose().Pos();
        bimanual_coupling_active_ = true;
        bimanual_grasp_drawer_position_ = control.joint->Position(0);
        bimanual_grasp_tool_projection_ = axis.Dot(
          0.5 * (left_pose + right_pose));
      } else {
        left_grasp_joint_ = model_->CreateJoint(
          kRuntimeBimanualLeftJointName, "fixed", control.link, left_link);
        if (!left_grasp_joint_) {
          return recover_failed_bimanual_attach(
            control, request, left_distance, right_distance,
            "Gazebo could not create the left bimanual drawer grasp joint.");
        }
        left_grasp_joint_->Init();
        right_grasp_joint_ = model_->CreateJoint(
          kRuntimeBimanualRightJointName, "fixed", control.link, right_link);
        if (!right_grasp_joint_) {
          return recover_failed_bimanual_attach(
            control, request, left_distance, right_distance,
            "Gazebo could not create the right bimanual drawer grasp joint.");
        }
        right_grasp_joint_->Init();
      }
      active_control_link_ = control.link;
      active_left_robot_link_ptr_ = left_link;
      active_right_robot_link_ptr_ = right_link;
      left_grasp_relative_pose_ =
        control.link->WorldPose().Inverse() * left_link->WorldPose();
      right_grasp_relative_pose_ =
        control.link->WorldPose().Inverse() * right_link->WorldPose();
      max_left_grasp_linear_error_ = 0.0;
      max_right_grasp_linear_error_ = 0.0;
      active_grasp_control_ = control.id;
      active_grasp_lease_id_ = request.operation_lease_id;
      active_grasp_control_generation_ = request.active_control_generation;
      active_robot_model_ = request.robot_model;
      active_left_robot_link_ = request.left_robot_link;
      active_right_robot_link_ = request.right_robot_link;
      active_left_robot_grasp_point_ = request.left_robot_grasp_point;
      active_right_robot_grasp_point_ = request.right_robot_grasp_point;
      publish_grasp_active(true);
      // Once a grasp attaches, the control leaves its pre-grasp phase: the
      // operation's release-settle motion is its own outcome, not an unsafe
      // pre-grasp disturbance.
      grasp_engaged_during_operation_ = true;
      suppress_actuation_collision(control, request.operation_lease_id);
    } catch (const std::exception & error) {
      return recover_failed_bimanual_attach(
        control, request, left_distance, right_distance, error.what());
    } catch (...) {
      return recover_failed_bimanual_attach(
        control, request, left_distance, right_distance,
        "unknown Gazebo bimanual attach exception");
    }
    PhysicsOutcome outcome;
    outcome.success = true;
    outcome.message = "Bimanual drawer grasp attached.";
    outcome.left_distance = left_distance;
    outcome.right_distance = right_distance;
    outcome.left_tool_contact = true;
    outcome.right_tool_contact = true;
    return outcome;
  }

  PhysicsOutcome handle_grasp_request(const PhysicsRequest & request)
  {
    if (!robot_grasp_point_is_finite(
        request.robot_grasp_point.X(), request.robot_grasp_point.Y(),
        request.robot_grasp_point.Z()))
    {
      return {false,
        "Robot grasp point must contain three finite link-local coordinates.",
        std::numeric_limits<double>::quiet_NaN()};
    }
    const auto control_it = control_indices_.find(request.control_id);
    if (control_it == control_indices_.end()) {
      return {false, "Unknown cabinet grasp control: " + request.control_id,
        std::numeric_limits<double>::quiet_NaN()};
    }
    auto & control = controls_[control_it->second];
    if (control.kind == ControlKind::kDrawer) {
      // A drawer is bimanual by design: a single-side grasp would only clamp
      // one handle and let the drawer rack under the pull torque, so the
      // single grasp service must not touch it.
      return {false,
        "Drawer controls require the bimanual grasp service: " + control.id,
        std::numeric_limits<double>::quiet_NaN()};
    }
    if (!control.graspable || control.kind == ControlKind::kButton) {
      return {false, "Cabinet control does not support grasping: " + control.id,
        std::numeric_limits<double>::quiet_NaN()};
    }

    if (!request.attach) {
      if (!grasp_is_active() && !base_brake_joint_) {
        return {true, "Cabinet grasp is already released.", 0.0};
      }
      if (active_grasp_control_ != control.id) {
        return {false,
          "A different cabinet control is currently grasped: " +
          active_grasp_control_,
          std::numeric_limits<double>::quiet_NaN()};
      }
      if (!operation_session_matches(
          active_grasp_control_, active_grasp_lease_id_, control.id,
          request.operation_lease_id))
      {
        return {false,
          "Cabinet grasp release requires the exact active operation lease "
          "identity.",
          std::numeric_limits<double>::quiet_NaN()};
      }
      return release_grasp_constraint();
    }

    if (request.operation_lease_id.empty()) {
      return {false,
        "Cabinet grasp attach requires a non-empty operation lease identity.",
        std::numeric_limits<double>::quiet_NaN()};
    }
    if (watchdog_recovery_pending_) {
      return {false,
        "Cabinet watchdog recovery is still releasing the previous physical "
        "operation; a new grasp is not yet safe.",
        std::numeric_limits<double>::quiet_NaN()};
    }
    if (!grasp_is_active() && !base_brake_joint_ &&
      first_suppressed_control())
    {
      return {false,
        "A prior cabinet grasp is still restoring its collision policy; a "
        "new grasp is not yet safe.",
        std::numeric_limits<double>::quiet_NaN()};
    }
    {
      std::lock_guard<std::mutex> lock(active_control_mutex_);
      if (!operation_heartbeat_authorizes_grasp_locked(request)) {
        return {false,
          "Cabinet grasp attach requires a fresh heartbeat from the exact "
          "active control and global operation lease session.",
          std::numeric_limits<double>::quiet_NaN()};
      }
      if (grasp_is_active() || base_brake_joint_) {
        if (active_grasp_control_ == control.id &&
          active_grasp_lease_id_ == request.operation_lease_id &&
          active_robot_model_ == request.robot_model &&
          active_robot_link_ == request.robot_link)
        {
          return {true, "Cabinet grasp is already attached.", 0.0};
        }
        return {false,
          "Another cabinet grasp or operation lease is active: " +
          active_grasp_control_,
          std::numeric_limits<double>::quiet_NaN()};
      }
    }
    if (request.robot_model.empty() || request.robot_link.empty() ||
      request.robot_base_link.empty())
    {
      return {false,
        "robot_model, robot_link and robot_base_link are required for attach.",
        std::numeric_limits<double>::quiet_NaN()};
    }

    // 2026-09-02: 预抓取扰动守卫只护"锁定"控件；对显式解锁的抽屉豁免（同
    // handle_bimanual_grasp_request）。非 drawer 控件的 drawer_unlocked 恒为
    // false，守卫照常生效，行为不变。
    if (!control.drawer_unlocked) {
      const double raw_position = control.joint->Position(0);
      const double raw_velocity = control.joint->GetVelocity(0);
      const double latched_detent =
        control.detents.at(control.detent_target_index);
      bool approach_disturbed = pregrasp_detent_is_disturbed(
        raw_position, raw_velocity, latched_detent,
        control.motion_tolerance, control.motion_tolerance);
      double maximum_position_error = std::abs(
        raw_position - latched_detent);
      double maximum_velocity = std::abs(raw_velocity);
      {
        std::lock_guard<std::mutex> lock(active_control_mutex_);
        if (pregrasp_disturbance_detected_ &&
          pregrasp_disturbance_control_ == control.id)
        {
          approach_disturbed = true;
          maximum_position_error = std::max(
            maximum_position_error, pregrasp_max_position_error_);
          maximum_velocity = std::max(
            maximum_velocity, pregrasp_max_velocity_);
        }
      }
      if (approach_disturbed) {
        return {false,
          "Unsafe pre-grasp movement was detected for cabinet control '" +
          control.id + "' (maximum detent error " +
          std::to_string(maximum_position_error) +
          " rad, maximum velocity " + std::to_string(maximum_velocity) +
          " rad/s). The ready/approach path may have contacted the control.",
          std::numeric_limits<double>::quiet_NaN()};
      }
    }
    const auto robot_model = world_->ModelByName(request.robot_model);
    if (!robot_model || robot_model == model_) {
      return {false, "Robot model was not found or is invalid: " +
        request.robot_model, std::numeric_limits<double>::quiet_NaN()};
    }
    const auto robot_link = robot_model->GetLink(request.robot_link);
    if (!robot_link) {
      return {false, "Robot link was not found: " + request.robot_link,
        std::numeric_limits<double>::quiet_NaN()};
    }
    const auto robot_base_link = robot_model->GetLink(request.robot_base_link);
    if (!robot_base_link) {
      return {false, "Robot base brake link was not found: " +
        request.robot_base_link, std::numeric_limits<double>::quiet_NaN()};
    }

    const auto target_pose = control.link->WorldPose();
    const auto target_point = target_pose.Pos() +
      target_pose.Rot().RotateVector(control.grasp_point);
    const auto robot_pose = robot_link->WorldPose();
    const auto robot_grasp_point = robot_pose.Pos() +
      robot_pose.Rot().RotateVector(request.robot_grasp_point);
    const double distance = robot_grasp_point.Distance(target_point);
    if (!std::isfinite(distance) || distance > grasp_distance_threshold_) {
      return {false,
        "Robot grasp point is outside the cabinet grasp distance threshold.",
        distance};
    }
    auto grasp_axis_world = control.joint->GlobalAxis(0);
    if (control.grasp_coupling_stiffness > 0.0 &&
      (!grasp_axis_world.IsFinite() || grasp_axis_world.Length() <= 1.0e-8))
    {
      return {false,
        "Cabinet control has no valid world joint axis for compliant grasp: " +
        control.id, distance};
    }
    grasp_axis_world.Normalize();

    // Revalidate immediately before the irreversible Gazebo operations and
    // hold the short session lock through commit.  An active-control change or
    // same-control lease replacement can therefore never race CreateJoint.
    std::unique_lock<std::mutex> active_session_lock(active_control_mutex_);
    if (!operation_heartbeat_authorizes_grasp_locked(request)) {
      return {false,
        "Cabinet grasp lease/heartbeat changed before physical attachment.",
        distance};
    }
    try {
      // A real mobile manipulator engages its wheel brakes before exerting
      // cabinet forces. Anchor the chassis for the lifetime of the physical
      // grasp so the door reaction cannot drag the whole robot away from the
      // world-frame Cartesian path.
      base_brake_joint_ = model_->CreateJoint(
        kRuntimeBaseBrakeJointName, "fixed", gazebo::physics::LinkPtr(),
        robot_base_link);
      if (!base_brake_joint_) {
        return {false, "Gazebo could not create the robot base brake joint.",
          distance};
      }
      base_brake_joint_->Init();
      if (control.grasp_coupling_stiffness > 0.0) {
        // A kinematic ros2_control position update and a cross-model 6-DOF
        // fixed joint form an inconsistent closed loop.  This control has an
        // explicitly configured one-DOF compliant physical coupling instead,
        // so do not add the conflicting ODE constraint as well.
        compliant_grasp_active_ = true;
      } else {
        grasp_joint_ = model_->CreateJoint(
          kRuntimeGraspJointName, "fixed", control.link, robot_link);
        if (!grasp_joint_) {
          return recover_failed_grasp_attach(
            control, request, robot_link,
            "Gazebo could not create the cabinet grasp joint.", distance);
        }
        // Model::CreateJoint loads the current relative pose; Init activates
        // it without teleporting either model or writing a control angle.
        grasp_joint_->Init();
      }
      active_control_link_ = control.link;
      active_robot_link_ptr_ = robot_link;
      grasp_relative_pose_ =
        control.link->WorldPose().Inverse() * robot_link->WorldPose();
      max_grasp_linear_error_ = 0.0;
      grasp_initial_position_ = control.joint->Position(0);
      grasp_tool_rotation_ = robot_link->WorldPose().Rot();
      grasp_axis_world_ = grasp_axis_world;
      grasp_previous_twist_ = 0.0;
      grasp_unwrapped_twist_ = 0.0;
      max_grasp_angle_error_ = 0.0;
      max_grasp_coupling_effort_ = 0.0;
      active_grasp_control_ = control.id;
      active_grasp_lease_id_ = request.operation_lease_id;
      active_grasp_control_generation_ = request.active_control_generation;
      active_robot_model_ = request.robot_model;
      active_robot_link_ = request.robot_link;
      active_robot_grasp_point_ = request.robot_grasp_point;
      publish_grasp_active(true);
      // Once a grasp attaches, the control leaves its pre-grasp phase: the
      // operation's release-settle motion is its own outcome, not an unsafe
      // pre-grasp disturbance.
      grasp_engaged_during_operation_ = true;
      // The fixed grasp and an articulated door otherwise form a closed
      // contact loop when the panel sweeps through the robot. Suppress only
      // the configured moving panel while it is robot-guided; the revolute
      // joint, spring force and grasp constraint remain fully physical.
      suppress_actuation_collision(control, request.operation_lease_id);
    } catch (const std::exception & error) {
      return recover_failed_grasp_attach(
        control, request, robot_link, error.what(), distance);
    } catch (...) {
      return recover_failed_grasp_attach(
        control, request, robot_link,
        "unknown Gazebo attach exception", distance);
    }
    return {true, "Cabinet grasp attached.", distance};
  }

  PhysicsOutcome release_single_grasp_constraint()
  {
    const auto released_control = active_grasp_control_;
    const auto released_robot_link = active_robot_link_ptr_;
    const auto released_robot_grasp_point = active_robot_grasp_point_;
    if (!released_control.empty()) {
      RCLCPP_INFO(
        ros_node_->get_logger(),
        "Releasing cabinet grasp '%s' (maximum fixed-constraint linear "
        "error %.6f m, compliant angle error %.6f rad, coupling effort "
        "%.3f).",
        released_control.c_str(), max_grasp_linear_error_,
        max_grasp_angle_error_, max_grasp_coupling_effort_);
    }
    if (grasp_joint_) {
      // Remove the ODE constraint itself before touching Gazebo's model
      // registry.  Resetting the shared pointer (or relying on RemoveJoint)
      // alone can leave the dynamically-created fixed constraint attached
      // until another link motion wakes the solver, which pins a released
      // door to the retreating tool.
      grasp_joint_->Detach();
      // Gazebo can clear the model joint registry before the ModelPlugin is
      // destroyed.  Treat that stale bookkeeping handle as already released.
      if (!model_ || !model_->GetJoint(kRuntimeGraspJointName)) {
        grasp_joint_.reset();
      } else if (!model_->RemoveJoint(kRuntimeGraspJointName)) {
        return {false,
          "Gazebo could not remove runtime cabinet grasp joint '" +
          std::string(kRuntimeGraspJointName) + "'.",
          std::numeric_limits<double>::quiet_NaN()};
      }
      grasp_joint_.reset();
    }
    compliant_grasp_active_ = false;
    if (base_brake_joint_) {
      base_brake_joint_->Detach();
      if (!model_ || !model_->GetJoint(kRuntimeBaseBrakeJointName)) {
        base_brake_joint_.reset();
      } else if (!model_->RemoveJoint(kRuntimeBaseBrakeJointName)) {
        return {false,
          "Gazebo could not remove runtime robot base brake joint '" +
          std::string(kRuntimeBaseBrakeJointName) + "'.",
          std::numeric_limits<double>::quiet_NaN()};
      }
      base_brake_joint_.reset();
    }
    active_grasp_control_.clear();
    active_grasp_lease_id_.clear();
    active_grasp_control_generation_ = 0U;
    active_robot_model_.clear();
    active_robot_link_.clear();
    active_robot_grasp_point_.Set(0.0, 0.0, 0.0);
    active_control_link_.reset();
    active_robot_link_ptr_.reset();
    max_grasp_linear_error_ = 0.0;
    grasp_initial_position_ = 0.0;
    grasp_tool_rotation_ = ignition::math::Quaterniond::Identity;
    grasp_axis_world_ = ignition::math::Vector3d::UnitZ;
    grasp_previous_twist_ = 0.0;
    grasp_unwrapped_twist_ = 0.0;
    max_grasp_angle_error_ = 0.0;
    max_grasp_coupling_effort_ = 0.0;
    publish_grasp_active(false);
    schedule_actuation_collision_restore(
      released_control, released_robot_link, released_robot_grasp_point);
    return {true, "Cabinet grasp released.", 0.0};
  }

  PhysicsOutcome release_bimanual_grasp_constraint()
  {
    const auto released_control = active_grasp_control_;
    const auto released_left_link = active_left_robot_link_ptr_;
    const auto released_left_grasp_point = active_left_robot_grasp_point_;
    if (!released_control.empty() &&
      (left_grasp_joint_ || right_grasp_joint_ || bimanual_coupling_active_))
    {
      RCLCPP_INFO(
        ros_node_->get_logger(),
        "Releasing bimanual drawer grasp '%s' (maximum fixed-constraint "
        "linear errors left %.6f m, right %.6f m%s).",
        released_control.c_str(), max_left_grasp_linear_error_,
        max_right_grasp_linear_error_,
        bimanual_coupling_active_ ? ", linear coupling" : "");
    }
    for (const auto & entry : {
      std::make_pair(kRuntimeBimanualLeftJointName, &left_grasp_joint_),
      std::make_pair(kRuntimeBimanualRightJointName, &right_grasp_joint_)})
    {
      const char * const joint_name = entry.first;
      auto & joint = *entry.second;
      if (!joint) {
        continue;
      }
      joint->Detach();
      if (!model_ || !model_->GetJoint(joint_name)) {
        joint.reset();
      } else if (!model_->RemoveJoint(joint_name)) {
        return {false,
          "Gazebo could not remove runtime bimanual grasp joint '" +
          std::string(joint_name) + ".",
          std::numeric_limits<double>::quiet_NaN()};
      }
      joint.reset();
    }
    if (base_brake_joint_) {
      base_brake_joint_->Detach();
      if (!model_ || !model_->GetJoint(kRuntimeBaseBrakeJointName)) {
        base_brake_joint_.reset();
      } else if (!model_->RemoveJoint(kRuntimeBaseBrakeJointName)) {
        return {false,
          "Gazebo could not remove runtime robot base brake joint '" +
          std::string(kRuntimeBaseBrakeJointName) + "'.",
          std::numeric_limits<double>::quiet_NaN()};
      }
      base_brake_joint_.reset();
    }
    bimanual_coupling_active_ = false;
    bimanual_grasp_drawer_position_ = 0.0;
    bimanual_grasp_tool_projection_ = 0.0;
    active_grasp_control_.clear();
    active_grasp_lease_id_.clear();
    active_grasp_control_generation_ = 0U;
    active_robot_model_.clear();
    active_left_robot_link_.clear();
    active_right_robot_link_.clear();
    active_left_robot_grasp_point_.Set(0.0, 0.0, 0.0);
    active_right_robot_grasp_point_.Set(0.0, 0.0, 0.0);
    active_control_link_.reset();
    active_left_robot_link_ptr_.reset();
    active_right_robot_link_ptr_.reset();
    left_grasp_relative_pose_ = ignition::math::Pose3d::Zero;
    right_grasp_relative_pose_ = ignition::math::Pose3d::Zero;
    max_left_grasp_linear_error_ = 0.0;
    max_right_grasp_linear_error_ = 0.0;
    publish_grasp_active(false);
    // The auto re-latch in on_update re-engages the rail lock once the drawer
    // returns to the closed position and settles.
    schedule_actuation_collision_restore(
      released_control, released_left_link, released_left_grasp_point);
    return {true, "Bimanual drawer grasp released.", 0.0};
  }

  // Release whichever physical grasp session is active (single or bimanual).
  // The two sessions are mutually exclusive, so at most one owns the base brake
  // and the shared active-grasp bookkeeping at any instant.
  PhysicsOutcome release_grasp_constraint()
  {
    const auto single = release_single_grasp_constraint();
    const auto bimanual = release_bimanual_grasp_constraint();
    if (!single.success && !bimanual.success) {
      return {false, single.message + "; " + bimanual.message,
        std::numeric_limits<double>::quiet_NaN()};
    }
    if (!single.success) {
      return {false, single.message, std::numeric_limits<double>::quiet_NaN()};
    }
    if (!bimanual.success) {
      return {false, bimanual.message, std::numeric_limits<double>::quiet_NaN()};
    }
    return {true, "Cabinet grasp constraints released.", 0.0};
  }

  static void complete_request(
    const std::shared_ptr<PhysicsRequest> & request,
    PhysicsOutcome outcome)
  {
    if (!request->completed.exchange(true)) {
      request->promise.set_value(std::move(outcome));
      request->state.store(PhysicsRequest::State::kCompleted);
    }
  }

  static void fail_requests(
    const std::deque<std::shared_ptr<PhysicsRequest>> & requests,
    const std::string & message)
  {
    for (const auto & request : requests) {
      auto expected = PhysicsRequest::State::kPending;
      request->state.compare_exchange_strong(
        expected, PhysicsRequest::State::kCanceled);
      complete_request(
        request,
        {false, message, std::numeric_limits<double>::quiet_NaN()});
    }
  }

  // Create the plugin's ROS 2 node directly instead of through
  // gazebo_ros::Node::Get.  That API registers the node in a process-lifetime
  // static map, so when a scene switch removes this model the node survives in
  // gzserver's rcl context and the next re-spawn of the same scene is refused
  // ("Found multiple nodes with same name"); the plugin then silently runs
  // without ROS.  Owning the node here ties its lifetime to the plugin
  // instance, so the rcl context frees the name when the model is removed and a
  // later scene switch can re-create it.  The node is spun by a dedicated
  // executor thread the plugin owns for the same reason (the gazebo_ros global
  // executor would keep the node alive past model removal).
  static rclcpp::Node::SharedPtr create_ros_node(sdf::ElementPtr sdf)
  {
    std::string node_name = sdf->Get<std::string>(
      "name", "xczs_cabinet_state").first;
    if (sdf->HasElement("node_name")) {
      node_name = sdf->Get<std::string>("node_name");
    }
    std::string ns = "/";
    if (sdf->HasElement("ros") &&
        sdf->GetElement("ros")->HasElement("namespace")) {
      ns = sdf->GetElement("ros")->Get<std::string>("namespace");
    }
    // 忽略进程级全局参数（gazebo_ros2_control 会给 gzserver 注入 __ns:=/xczs
    // 重映射），严格按插件 <ros><namespace> 显式命名空间建节点，与 gazebo_ros::
    // Node 历史行为一致（FQN = /xczs/cabinet/<scene>/xczs_<scene>_state）。
    rclcpp::NodeOptions options;
    options.use_global_arguments(false);
    return std::make_shared<rclcpp::Node>(node_name, ns, options);
  }

  void start_spin_thread()
  {
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(ros_node_);
    spinning_.store(true);
    spin_thread_ = std::make_unique<std::thread>([this]() {
      while (rclcpp::ok() && spinning_.load()) {
        executor_->spin_once();
      }
    });
  }

  void stop_spin_thread()
  {
    if (!spinning_.exchange(false)) {
      return;
    }
    if (executor_) {
      executor_->cancel();
    }
    if (spin_thread_ && spin_thread_->joinable()) {
      spin_thread_->join();
    }
    if (executor_) {
      executor_->remove_node(ros_node_);
      executor_.reset();
    }
    spin_thread_.reset();
  }

  gazebo::physics::ModelPtr model_;
  gazebo::physics::WorldPtr world_;
  gazebo::event::ConnectionPtr update_connection_;
  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;
  std::unique_ptr<std::thread> spin_thread_;
  std::atomic<bool> spinning_{false};
  std::vector<Control> controls_;
  std::unordered_map<std::string, std::size_t> control_indices_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  rclcpp::Service<SetCabinetGrasp>::SharedPtr grasp_service_;
  rclcpp::Service<SetCabinetBimanualGrasp>::SharedPtr bimanual_grasp_service_;
  rclcpp::Service<SetCabinetUnlock>::SharedPtr unlock_service_;
  rclcpp::Service<SetCabinetPlayback>::SharedPtr playback_service_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr grasp_active_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
    active_control_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
    operation_heartbeat_subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
    operation_fault_publisher_;
  std::string reset_service_name_;
  std::string grasp_service_name_;
  std::string bimanual_grasp_service_name_;
  std::string unlock_service_name_;
  std::string playback_service_name_;
  bool drawer_controls_present_{false};
  std::string grasp_active_topic_;
  std::string active_control_topic_;
  std::string operation_heartbeat_topic_;
  std::string operation_fault_topic_;
  double publish_period_{0.05};
  double last_publish_time_{0.0};
  double grasp_distance_threshold_{0.12};
  double operation_watchdog_timeout_{2.0};
  std::shared_ptr<ServiceCallbackLifetime> callback_lifetime_;
  std::mutex physics_callback_mutex_;
  std::mutex request_mutex_;
  std::deque<std::shared_ptr<PhysicsRequest>> requests_;
  std::atomic<bool> shutting_down_{false};
  mutable std::mutex active_control_mutex_;
  bool active_control_received_{false};
  std::string active_operation_control_;
  std::uint64_t active_control_generation_{0U};
  bool operation_heartbeat_received_{false};
  std::string operation_heartbeat_lease_id_;
  std::chrono::steady_clock::time_point operation_last_heartbeat_{};
  std::uint64_t operation_heartbeat_sequence_{0U};
  std::unordered_set<std::string> expired_operation_lease_ids_;
  std::deque<std::string> expired_operation_lease_history_;
  bool pregrasp_disturbance_detected_{false};
  std::string pregrasp_disturbance_control_;
  double pregrasp_max_position_error_{0.0};
  double pregrasp_max_velocity_{0.0};
  // Set once a grasp attaches during the active operation and reset when the
  // operation control changes or controls are reset.  Suppresses the pre-grasp
  // disturbance detector for the rest of the operation so the post-release
  // detent settle is never misreported as an unsafe approach bump.
  bool grasp_engaged_during_operation_{false};
  std::chrono::steady_clock::time_point watchdog_monitor_started_at_{};
  std::chrono::steady_clock::time_point watchdog_last_valid_heartbeat_{};
  std::string watchdog_activity_control_;
  std::string watchdog_activity_lease_id_;
  bool watchdog_recovery_pending_{false};
  bool watchdog_release_completed_{false};
  std::string watchdog_recovery_control_;
  std::string watchdog_recovery_lease_id_;
  std::chrono::steady_clock::time_point watchdog_release_retry_not_before_{};
  std::chrono::steady_clock::time_point watchdog_last_release_error_log_{};

  gazebo::physics::JointPtr grasp_joint_;
  bool compliant_grasp_active_{false};
  // Bimanual drawer session: two fixed joints (left tool to left handle, right
  // tool to right handle) plus the shared base brake.  Mutually exclusive with
  // the single-hand session above.
  gazebo::physics::JointPtr left_grasp_joint_;
  gazebo::physics::JointPtr right_grasp_joint_;
  gazebo::physics::JointPtr base_brake_joint_;
  // Linear-drag coupling session: instead of welding both tools to the drawer,
  // the rail is driven by a bounded spring-damper toward the tools' commanded
  // motion since the grasp attach.  The reaction lands on the cabinet rail (not
  // the light tools), so a kinematic arm teleport never fights an 80 kg
  // drawer's inertia through a hard cross-model fixed joint (run8 root cause).
  bool bimanual_coupling_active_{false};
  double bimanual_grasp_drawer_position_{0.0};
  double bimanual_grasp_tool_projection_{0.0};
  gazebo::physics::LinkPtr active_control_link_;
  gazebo::physics::LinkPtr active_robot_link_ptr_;
  gazebo::physics::LinkPtr active_left_robot_link_ptr_;
  gazebo::physics::LinkPtr active_right_robot_link_ptr_;
  ignition::math::Pose3d grasp_relative_pose_;
  ignition::math::Pose3d left_grasp_relative_pose_;
  ignition::math::Pose3d right_grasp_relative_pose_;
  double max_grasp_linear_error_{0.0};
  double max_left_grasp_linear_error_{0.0};
  double max_right_grasp_linear_error_{0.0};
  double grasp_initial_position_{0.0};
  ignition::math::Quaterniond grasp_tool_rotation_{
    ignition::math::Quaterniond::Identity};
  ignition::math::Vector3d grasp_axis_world_{
    ignition::math::Vector3d::UnitZ};
  double grasp_previous_twist_{0.0};
  double grasp_unwrapped_twist_{0.0};
  double max_grasp_angle_error_{0.0};
  double max_grasp_coupling_effort_{0.0};
  std::string active_grasp_control_;
  std::string active_grasp_lease_id_;
  std::uint64_t active_grasp_control_generation_{0U};
  std::string active_robot_model_;
  std::string active_robot_link_;
  ignition::math::Vector3d active_robot_grasp_point_{0.0, 0.0, 0.0};
  std::string active_left_robot_link_;
  std::string active_right_robot_link_;
  ignition::math::Vector3d active_left_robot_grasp_point_{0.0, 0.0, 0.0};
  ignition::math::Vector3d active_right_robot_grasp_point_{0.0, 0.0, 0.0};
};

GZ_REGISTER_MODEL_PLUGIN(CabinetStatePlugin)

}  // namespace xczs_inspection_robot_control
