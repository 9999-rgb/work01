// Copyright 2026
// SPDX-License-Identifier: BSD-3-Clause

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/ode/ODEJoint.hh>
#include <gazebo_ros/node.hpp>

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
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "xczs_inspection_robot_control/msg/cabinet_control.hpp"
#include "xczs_inspection_robot_control/msg/cabinet_control_state.hpp"
#include "xczs_inspection_robot_control/srv/set_cabinet_grasp.hpp"

namespace xczs_inspection_robot_control
{

namespace
{

using CabinetControl =
  xczs_inspection_robot_control::msg::CabinetControl;
using CabinetControlState =
  xczs_inspection_robot_control::msg::CabinetControlState;
using SetCabinetGrasp =
  xczs_inspection_robot_control::srv::SetCabinetGrasp;

constexpr auto kPhysicsRequestTimeout = std::chrono::seconds(2);
constexpr char kRuntimeGraspJointName[] = "xczs_cabinet_runtime_grasp";
constexpr char kRuntimeBaseBrakeJointName[] =
  "xczs_cabinet_runtime_base_brake";

enum class ControlKind
{
  kButton,
  kKnob,
  kSwitch,
  kDoor,
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
    ros_node_ = gazebo_ros::Node::Get(sdf);
    if (!ros_node_) {
      gzerr << "Cabinet state plugin could not create a ROS 2 node.\n";
      return;
    }

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
      grasp_robot_base_link_ = sdf->Get<std::string>(
        "grasp_robot_base_link", "body").first;
      if (grasp_robot_base_link_.empty()) {
        throw std::invalid_argument(
                "Cabinet grasp_robot_base_link must not be empty.");
      }
      grasp_constraint_erp_ = optional_double(
        sdf, "grasp_constraint_erp", 0.8);
      grasp_constraint_cfm_ = optional_double(
        sdf, "grasp_constraint_cfm", 1.0e-8);
      if (grasp_constraint_erp_ <= 0.0 || grasp_constraint_erp_ > 1.0 ||
        grasp_constraint_cfm_ < 0.0)
      {
        throw std::invalid_argument(
                "Cabinet grasp constraint ERP must be in (0, 1] and CFM "
                "must be non-negative.");
      }
      reset_service_name_ = sdf->Get<std::string>(
        "reset_service", "/xczs/cabinet/reset_physics").first;
      grasp_service_name_ = sdf->Get<std::string>(
        "grasp_service", "/xczs/cabinet/grasp").first;
      grasp_active_topic_ = sdf->Get<std::string>(
        "grasp_active_topic", "/xczs/cabinet/grasp_active").first;
      require_absolute_topic(reset_service_name_, "reset_service");
      require_absolute_topic(grasp_service_name_, "grasp_service");
      require_absolute_topic(grasp_active_topic_, "grasp_active_topic");
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
          PhysicsRequest::Kind::kReset, {}, {}, {}, false);
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
          request->robot_model,
          request->robot_link,
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
    gazebo::physics::CollisionPtr actuation_collision;
    double actuation_collision_restore_delay{1.5};
    double collision_restore_not_before{0.0};
    bool actuation_collision_suppressed{false};
    double detent_hysteresis{0.0};
    double press_threshold{0.006};
    double release_threshold{0.003};
    double motion_tolerance{0.025};
    double reset_position{0.0};
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
    std::string robot_model;
    std::string robot_link;
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
      control.motion_tolerance = optional_double(
        element, "motion_tolerance", 0.025);
      control.graspable = optional_bool(element, "graspable", false);
      if (element->HasElement("grasp_point")) {
        control.grasp_point = parse_vector3(
          element->Get<std::string>("grasp_point"));
      }
      if (element->HasElement("actuation_collision")) {
        const auto collision_name = required_text(
          element, "actuation_collision");
        control.actuation_collision = control.link->GetCollision(
          collision_name);
        if (!control.actuation_collision) {
          for (const auto & collision : control.link->GetCollisions()) {
            if (!collision) {
              continue;
            }
            const auto scoped_name = collision->GetScopedName();
            const auto suffix = std::string("::") + collision_name;
            const auto converted_name = collision_name + "_collision";
            const auto converted_suffix =
              std::string("::") + converted_name;
            if (collision->GetName() == collision_name ||
              collision->GetName() == converted_name ||
              scoped_name == collision_name ||
              (scoped_name.size() > suffix.size() &&
              scoped_name.compare(
                scoped_name.size() - suffix.size(), suffix.size(), suffix) ==
              0) ||
              (scoped_name.size() > converted_suffix.size() &&
              scoped_name.compare(
                scoped_name.size() - converted_suffix.size(),
                converted_suffix.size(), converted_suffix) == 0))
            {
              control.actuation_collision = collision;
              break;
            }
          }
        }
        if (!control.actuation_collision) {
          throw std::invalid_argument(
                  "Cabinet actuation collision was not found on child link: " +
                  collision_name);
        }
        control.actuation_collision_restore_delay = optional_double(
          element, "actuation_collision_restore_delay", 1.5);
      }

      if (control.damping < 0.0 || control.grasp_damping < 0.0 ||
        control.motion_tolerance <= 0.0 ||
        control.actuation_collision_restore_delay < 0.0)
      {
        throw std::invalid_argument(
                "Cabinet control damping values must be non-negative and "
                "motion tolerance must be positive: " + control.id);
      }
      if (control.actuation_collision && !control.graspable) {
        throw std::invalid_argument(
                "Cabinet actuation collision suppression requires a "
                "graspable control: " + control.id);
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
    process_physics_requests();
    const double simulation_time = world_->SimTime().Double();
    for (auto & control : controls_) {
      const double raw_position = control.joint->Position(0);
      const double velocity = control.joint->GetVelocity(0);
      const bool control_is_being_grasped = grasp_joint_ &&
        active_grasp_control_ == control.id;
      double target = 0.0;
      if (control.kind != ControlKind::kButton) {
        const auto previous_detent_target_index =
          control.detent_target_index;
        const bool latched_control_is_being_grasped =
          control.detent_hysteresis > 0.0 && control_is_being_grasped;
        if (control.detent_hysteresis == 0.0 ||
          latched_control_is_being_grasped)
        {
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
      control.joint->SetForce(0, control.effort);
      if (control.actuation_collision_suppressed &&
        !control_is_being_grasped &&
        simulation_time >= control.collision_restore_not_before &&
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

  void set_actuation_collision_enabled(Control & control, bool enabled)
  {
    if (!control.actuation_collision ||
      control.actuation_collision_suppressed == !enabled)
    {
      return;
    }
    control.actuation_collision->SetCollideBits(enabled ? 0xFFFFu : 0u);
    control.actuation_collision_suppressed = !enabled;
    if (enabled) {
      control.collision_restore_not_before = 0.0;
    }
    if (ros_node_) {
      RCLCPP_INFO(
        ros_node_->get_logger(),
        "%s actuation collision for cabinet control '%s'.",
        enabled ? "Restored" : "Suppressed", control.id.c_str());
    }
  }

  void suppress_actuation_collision(Control & control)
  {
    set_actuation_collision_enabled(control, false);
    if (control.actuation_collision_suppressed) {
      control.collision_restore_not_before =
        std::numeric_limits<double>::infinity();
    }
  }

  void schedule_actuation_collision_restore(const std::string & control_id)
  {
    const auto control_it = control_indices_.find(control_id);
    if (control_it == control_indices_.end()) {
      return;
    }
    auto & control = controls_[control_it->second];
    if (control.actuation_collision_suppressed) {
      control.collision_restore_not_before = world_->SimTime().Double() +
        control.actuation_collision_restore_delay;
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
    restore_all_actuation_collisions();
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
    std::string robot_model,
    std::string robot_link,
    bool attach)
  {
    auto request = std::make_shared<PhysicsRequest>();
    request->kind = kind;
    request->control_id = std::move(control_id);
    request->robot_model = std::move(robot_model);
    request->robot_link = std::move(robot_link);
    request->attach = attach;
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

  PhysicsOutcome handle_grasp_request(const PhysicsRequest & request)
  {
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
      if (!grasp_joint_ && !base_brake_joint_) {
        return {true, "Cabinet grasp is already released.", 0.0};
      }
      if (active_grasp_control_ != control.id) {
        return {false,
          "A different cabinet control is currently grasped: " +
          active_grasp_control_,
          std::numeric_limits<double>::quiet_NaN()};
      }
      return release_grasp_constraint();
    }

    if (grasp_joint_ || base_brake_joint_) {
      if (active_grasp_control_ == control.id &&
        active_robot_model_ == request.robot_model &&
        active_robot_link_ == request.robot_link)
      {
        return {true, "Cabinet grasp is already attached.", 0.0};
      }
      return {false,
        "Another cabinet grasp is active: " + active_grasp_control_,
        std::numeric_limits<double>::quiet_NaN()};
    }
    if (request.robot_model.empty() || request.robot_link.empty()) {
      return {false, "robot_model and robot_link are required for attach.",
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
    const auto robot_base_link = robot_model->GetLink(
      grasp_robot_base_link_);
    if (!robot_base_link) {
      return {false, "Robot base brake link was not found: " +
        grasp_robot_base_link_, std::numeric_limits<double>::quiet_NaN()};
    }

    const auto target_pose = control.link->WorldPose();
    const auto target_point = target_pose.Pos() +
      target_pose.Rot().RotateVector(control.grasp_point);
    const double distance =
      robot_link->WorldPose().Pos().Distance(target_point);
    if (!std::isfinite(distance) || distance > grasp_distance_threshold_) {
      return {false,
        "Robot link is outside the cabinet grasp distance threshold.",
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
      configure_runtime_fixed_joint(base_brake_joint_);
      grasp_joint_ = model_->CreateJoint(
        kRuntimeGraspJointName, "fixed", control.link, robot_link);
      if (!grasp_joint_) {
        const auto release = release_grasp_constraint();
        return {false,
          "Gazebo could not create the cabinet grasp joint." +
          (release.success ? "" : " Cleanup failed: " + release.message),
          distance};
      }
      // Model::CreateJoint loads the current relative pose; Init activates it
      // without teleporting either model or directly setting a control angle.
      grasp_joint_->Init();
      configure_runtime_fixed_joint(grasp_joint_);
      active_grasp_control_ = control.id;
      active_robot_model_ = request.robot_model;
      active_robot_link_ = request.robot_link;
      publish_grasp_active(true);
      // The fixed grasp and an articulated door otherwise form a closed
      // contact loop when the panel sweeps through the robot. Suppress only
      // the configured moving panel while it is robot-guided; the revolute
      // joint, spring force and grasp constraint remain fully physical.
      suppress_actuation_collision(control);
    } catch (const std::exception & error) {
      const auto release = release_grasp_constraint();
      set_actuation_collision_enabled(control, true);
      return {false,
        "Failed to create cabinet grasp constraint: " +
        std::string(error.what()) +
        (release.success ? "" : "; cleanup failed: " + release.message),
        distance};
    }
    return {true, "Cabinet grasp attached.", distance};
  }

  void configure_runtime_fixed_joint(
    const gazebo::physics::JointPtr & joint) const
  {
    const auto ode_joint = boost::dynamic_pointer_cast<
      gazebo::physics::ODEJoint>(joint);
    if (!ode_joint) {
      return;
    }
    ode_joint->SetERP(grasp_constraint_erp_);
    ode_joint->SetCFM(grasp_constraint_cfm_);
  }

  PhysicsOutcome release_grasp_constraint()
  {
    const auto released_control = active_grasp_control_;
    if (grasp_joint_) {
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
    if (base_brake_joint_) {
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
    active_robot_model_.clear();
    active_robot_link_.clear();
    publish_grasp_active(false);
    schedule_actuation_collision_restore(released_control);
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

  gazebo::physics::ModelPtr model_;
  gazebo::physics::WorldPtr world_;
  gazebo::event::ConnectionPtr update_connection_;
  gazebo_ros::Node::SharedPtr ros_node_;
  std::vector<Control> controls_;
  std::unordered_map<std::string, std::size_t> control_indices_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  rclcpp::Service<SetCabinetGrasp>::SharedPtr grasp_service_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr grasp_active_publisher_;
  std::string reset_service_name_;
  std::string grasp_service_name_;
  std::string grasp_active_topic_;
  double publish_period_{0.05};
  double last_publish_time_{0.0};
  double grasp_distance_threshold_{0.12};
  std::string grasp_robot_base_link_{"body"};
  double grasp_constraint_erp_{0.8};
  double grasp_constraint_cfm_{1.0e-8};

  std::shared_ptr<ServiceCallbackLifetime> callback_lifetime_;
  std::mutex physics_callback_mutex_;
  std::mutex request_mutex_;
  std::deque<std::shared_ptr<PhysicsRequest>> requests_;
  std::atomic<bool> shutting_down_{false};

  gazebo::physics::JointPtr grasp_joint_;
  gazebo::physics::JointPtr base_brake_joint_;
  std::string active_grasp_control_;
  std::string active_robot_model_;
  std::string active_robot_link_;
};

GZ_REGISTER_MODEL_PLUGIN(CabinetStatePlugin)

}  // namespace xczs_inspection_robot_control
