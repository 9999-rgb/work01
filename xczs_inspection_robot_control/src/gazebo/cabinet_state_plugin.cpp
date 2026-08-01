// Copyright 2026
// SPDX-License-Identifier: BSD-3-Clause

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_ros/node.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
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
    shutting_down_.store(true);
    update_connection_.reset();
    reset_service_.reset();
    grasp_service_.reset();
    fail_pending_requests("Cabinet state plugin is shutting down.");
    release_grasp_constraint();
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
        sdf, "state_publish_rate", 50.0);
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
      reset_service_name_ = sdf->Get<std::string>(
        "reset_service", "/xczs/cabinet/reset_physics").first;
      grasp_service_name_ = sdf->Get<std::string>(
        "grasp_service", "/xczs/cabinet/grasp").first;
      require_absolute_topic(reset_service_name_, "reset_service");
      require_absolute_topic(grasp_service_name_, "grasp_service");
      configure_controls(sdf);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(ros_node_->get_logger(), "%s", error.what());
      controls_.clear();
      return;
    }

    reset_service_ = ros_node_->create_service<std_srvs::srv::Trigger>(
      reset_service_name_,
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
      {
        const auto outcome = submit_physics_request(
          PhysicsRequest::Kind::kReset, {}, {}, {}, false);
        response->success = outcome.success;
        response->message = outcome.message;
      });
    grasp_service_ = ros_node_->create_service<SetCabinetGrasp>(
      grasp_service_name_,
      [this](
        const std::shared_ptr<SetCabinetGrasp::Request> request,
        std::shared_ptr<SetCabinetGrasp::Response> response)
      {
        const auto outcome = submit_physics_request(
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
      std::bind(&CabinetStatePlugin::on_update, this));
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
    reset_controls();
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
    double damping{0.0};
    double press_threshold{0.006};
    double release_threshold{0.003};
    double motion_tolerance{0.025};
    double reset_position{0.0};
    bool graspable{false};
    ignition::math::Vector3d grasp_point{0.0, 0.0, 0.0};
    std::size_t reset_state_index{0U};
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
    Kind kind{Kind::kReset};
    std::string control_id;
    std::string robot_model;
    std::string robot_link;
    bool attach{false};
    std::promise<PhysicsOutcome> promise;
    std::atomic<bool> expired{false};
    std::atomic<bool> completed{false};
  };

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
      control.motion_tolerance = optional_double(
        element, "motion_tolerance", 0.025);
      control.graspable = optional_bool(element, "graspable", false);
      if (element->HasElement("grasp_point")) {
        control.grasp_point = parse_vector3(
          element->Get<std::string>("grasp_point"));
      }

      if (control.damping < 0.0 || control.motion_tolerance <= 0.0) {
        throw std::invalid_argument(
                "Cabinet control damping must be non-negative and motion "
                "tolerance must be positive: " + control.id);
      }
      if (control.kind == ControlKind::kButton) {
        if (control.state_ids.size() != 2U) {
          throw std::invalid_argument(
                  "Button state_ids must be: released pressed.");
        }
        control.stiffness = optional_double(
          element, "spring_stiffness", 800.0);
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
        if (control.detents.size() < 2U ||
          control.detents.size() != control.state_ids.size() ||
          !std::is_sorted(control.detents.begin(), control.detents.end()) ||
          control.stiffness <= 0.0)
        {
          throw std::invalid_argument(
                  "Invalid detent configuration for cabinet control: " +
                  control.id);
        }
        const double lower = control.joint->LowerLimit(0);
        const double upper = control.joint->UpperLimit(0);
        if (control.detents.front() < lower - 1.0e-6 ||
          control.detents.back() > upper + 1.0e-6)
        {
          throw std::invalid_argument(
                  "Cabinet detents exceed joint limits: " + control.id);
        }
        control.reset_state_index = nearest_index(
          control.detents, control.reset_position);
      }

      control.joint_state_publisher =
        ros_node_->create_publisher<sensor_msgs::msg::JointState>(
        control.joint_state_topic, rclcpp::QoS(10).reliable());
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
    process_physics_requests();
    for (auto & control : controls_) {
      const double raw_position = control.joint->Position(0);
      const double velocity = control.joint->GetVelocity(0);
      const double target = control.kind == ControlKind::kButton ?
        0.0 : control.detents[nearest_index(control.detents, raw_position)];
      control.effort =
        -control.stiffness * (raw_position - target) -
        control.damping * velocity;
      control.joint->SetForce(0, control.effort);
      update_control_state(control, true);
    }

    const double simulation_time = world_->SimTime().Double();
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

  void reset_controls()
  {
    release_grasp_constraint();
    for (auto & control : controls_) {
      control.joint->SetForce(0, 0.0);
      control.joint->SetVelocity(0, 0.0);
      if (!control.joint->SetPosition(0, control.reset_position, false)) {
        RCLCPP_WARN(
          ros_node_->get_logger(),
          "Failed to reset cabinet joint %s.",
          control.joint_name.c_str());
      }
      control.effort = 0.0;
      update_control_state(control, false);
      ++control.transition_sequence;
      publish_control(control);
    }
  }

  PhysicsOutcome submit_physics_request(
    PhysicsRequest::Kind kind,
    std::string control_id,
    std::string robot_model,
    std::string robot_link,
    bool attach)
  {
    if (shutting_down_.load() || !update_connection_) {
      return {false, "Cabinet physics plugin is not running.",
        std::numeric_limits<double>::quiet_NaN()};
    }
    auto request = std::make_shared<PhysicsRequest>();
    request->kind = kind;
    request->control_id = std::move(control_id);
    request->robot_model = std::move(robot_model);
    request->robot_link = std::move(robot_link);
    request->attach = attach;
    auto result = request->promise.get_future();
    {
      std::lock_guard<std::mutex> lock(request_mutex_);
      requests_.push_back(request);
    }
    if (result.wait_for(kPhysicsRequestTimeout) != std::future_status::ready) {
      request->expired.store(true);
      return {false, "Timed out waiting for the Gazebo physics update.",
        std::numeric_limits<double>::quiet_NaN()};
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
      if (request->expired.load()) {
        complete_request(
          request,
          {false, "Gazebo physics request expired.",
            std::numeric_limits<double>::quiet_NaN()});
        continue;
      }
      if (request->kind == PhysicsRequest::Kind::kReset) {
        reset_controls();
        complete_request(
          request,
          {true, "Cabinet physics reset complete.", 0.0});
      } else {
        complete_request(request, handle_grasp_request(*request));
      }
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
      if (!grasp_joint_) {
        return {true, "Cabinet grasp is already released.", 0.0};
      }
      if (active_grasp_control_ != control.id) {
        return {false,
          "A different cabinet control is currently grasped: " +
          active_grasp_control_,
          std::numeric_limits<double>::quiet_NaN()};
      }
      release_grasp_constraint();
      return {true, "Cabinet grasp released.", 0.0};
    }

    if (grasp_joint_) {
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
      grasp_joint_ = model_->CreateJoint(
        kRuntimeGraspJointName, "fixed", control.link, robot_link);
      if (!grasp_joint_) {
        return {false, "Gazebo could not create the cabinet grasp joint.",
          distance};
      }
      // Model::CreateJoint loads the current relative pose; Init activates it
      // without teleporting either model or directly setting a control angle.
      grasp_joint_->Init();
    } catch (const std::exception & error) {
      release_grasp_constraint();
      return {false,
        "Failed to create cabinet grasp constraint: " +
        std::string(error.what()), distance};
    }
    active_grasp_control_ = control.id;
    active_robot_model_ = request.robot_model;
    active_robot_link_ = request.robot_link;
    return {true, "Cabinet grasp attached.", distance};
  }

  void release_grasp_constraint()
  {
    if (grasp_joint_) {
      // Model::RemoveJoint owns the complete joint teardown in Gazebo 11,
      // including Detach() and Fini().  Calling those methods here first would
      // finalize the backend joint twice and can crash during reset/shutdown.
      if (!model_->RemoveJoint(kRuntimeGraspJointName)) {
        RCLCPP_WARN(
          ros_node_->get_logger(),
          "Gazebo could not remove runtime cabinet grasp joint '%s'.",
          kRuntimeGraspJointName);
      }
      grasp_joint_.reset();
    }
    active_grasp_control_.clear();
    active_robot_model_.clear();
    active_robot_link_.clear();
  }

  static void complete_request(
    const std::shared_ptr<PhysicsRequest> & request,
    PhysicsOutcome outcome)
  {
    if (!request->completed.exchange(true)) {
      request->promise.set_value(std::move(outcome));
    }
  }

  void fail_pending_requests(const std::string & message)
  {
    std::deque<std::shared_ptr<PhysicsRequest>> requests;
    {
      std::lock_guard<std::mutex> lock(request_mutex_);
      requests.swap(requests_);
    }
    for (const auto & request : requests) {
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
  std::string reset_service_name_;
  std::string grasp_service_name_;
  double publish_period_{0.02};
  double last_publish_time_{0.0};
  double grasp_distance_threshold_{0.12};

  std::mutex request_mutex_;
  std::deque<std::shared_ptr<PhysicsRequest>> requests_;
  std::atomic<bool> shutting_down_{false};

  gazebo::physics::JointPtr grasp_joint_;
  std::string active_grasp_control_;
  std::string active_robot_model_;
  std::string active_robot_link_;
};

GZ_REGISTER_MODEL_PLUGIN(CabinetStatePlugin)

}  // namespace xczs_inspection_robot_control
