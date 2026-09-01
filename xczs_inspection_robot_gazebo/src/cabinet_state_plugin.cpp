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

constexpr auto kPhysicsRequestTimeout = std::chrono::seconds(2);
constexpr auto kWatchdogReleaseRetryDelay = std::chrono::milliseconds(100);
constexpr auto kWatchdogReleaseLogPeriod = std::chrono::seconds(1);
constexpr std::size_t kExpiredLeaseHistoryLimit = 64U;
constexpr char kRuntimeGraspJointName[] = "xczs_cabinet_runtime_grasp";
constexpr char kRuntimeBaseBrakeJointName[] =
  "xczs_cabinet_runtime_base_brake";

enum class ControlKind
{
  kButton,
  kKnob,
  kSwitch,
  kDoor,
  kSlider,
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
    RCLCPP_INFO(
      ros_node_->get_logger(),
      "Cabinet state plugin loaded %zu controls; reset=%s grasp=%s.",
      controls_.size(), reset_service_name_.c_str(),
      grasp_service_name_.c_str());
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
  };

  struct PhysicsRequest
  {
    enum class Kind {kReset, kGrasp};
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
    const double simulation_time = world_->SimTime().Double();
    for (auto & control : controls_) {
      const double raw_position = control.joint->Position(0);
      const double velocity = control.joint->GetVelocity(0);
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
        if (control.kind == ControlKind::kSlider &&
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
      const double effective_stiffness = control_is_being_grasped ?
        control.grasp_stiffness : control.stiffness;
      control.effort =
        -effective_stiffness * (raw_position - target) -
        effective_damping * velocity;
      if (control_is_being_grasped &&
        control.grasp_coupling_stiffness > 0.0)
      {
        // gazebo_ros2_control's plain position backend moves the arm
        // kinematically every controller update.  That motion is authoritative
        // for the simulated tool but can repeatedly inject error into a
        // cross-model ODE fixed joint.  Recover the commanded one-DOF motion
        // analytically from the tool orientation and apply it as a compliant
        // physical torque.  The cabinet joint is never teleported: inertia,
        // limits, damping, collisions and the fixed grasp remain active.
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

  bool grasp_is_active() const
  {
    return static_cast<bool>(grasp_joint_) || compliant_grasp_active_;
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
        outcome = request->kind == PhysicsRequest::Kind::kReset ?
          reset_controls() : handle_grasp_request(*request);
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

  PhysicsOutcome release_grasp_constraint()
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
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr grasp_active_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
    active_control_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
    operation_heartbeat_subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
    operation_fault_publisher_;
  std::string reset_service_name_;
  std::string grasp_service_name_;
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
  gazebo::physics::JointPtr base_brake_joint_;
  gazebo::physics::LinkPtr active_control_link_;
  gazebo::physics::LinkPtr active_robot_link_ptr_;
  ignition::math::Pose3d grasp_relative_pose_;
  double max_grasp_linear_error_{0.0};
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
};

GZ_REGISTER_MODEL_PLUGIN(CabinetStatePlugin)

}  // namespace xczs_inspection_robot_control
