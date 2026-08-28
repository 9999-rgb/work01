// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from xczs_inspection_robot_control:action/OperateCabinetControl.idl
// generated code does not contain a copyright notice

#ifndef XCZS_INSPECTION_ROBOT_CONTROL__ACTION__DETAIL__OPERATE_CABINET_CONTROL__TRAITS_HPP_
#define XCZS_INSPECTION_ROBOT_CONTROL__ACTION__DETAIL__OPERATE_CABINET_CONTROL__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "xczs_inspection_robot_control/action/detail/operate_cabinet_control__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace xczs_inspection_robot_control
{

namespace action
{

inline void to_flow_style_yaml(
  const OperateCabinetControl_Goal & msg,
  std::ostream & out)
{
  out << "{";
  // member: control_id
  {
    out << "control_id: ";
    rosidl_generator_traits::value_to_yaml(msg.control_id, out);
    out << ", ";
  }

  // member: command
  {
    out << "command: ";
    rosidl_generator_traits::value_to_yaml(msg.command, out);
    out << ", ";
  }

  // member: target_state
  {
    out << "target_state: ";
    rosidl_generator_traits::value_to_yaml(msg.target_state, out);
    out << ", ";
  }

  // member: target_position
  {
    out << "target_position: ";
    rosidl_generator_traits::value_to_yaml(msg.target_position, out);
    out << ", ";
  }

  // member: use_target_position
  {
    out << "use_target_position: ";
    rosidl_generator_traits::value_to_yaml(msg.use_target_position, out);
    out << ", ";
  }

  // member: force
  {
    out << "force: ";
    rosidl_generator_traits::value_to_yaml(msg.force, out);
    out << ", ";
  }

  // member: navigate_to_staging_pose
  {
    out << "navigate_to_staging_pose: ";
    rosidl_generator_traits::value_to_yaml(msg.navigate_to_staging_pose, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const OperateCabinetControl_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: control_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "control_id: ";
    rosidl_generator_traits::value_to_yaml(msg.control_id, out);
    out << "\n";
  }

  // member: command
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "command: ";
    rosidl_generator_traits::value_to_yaml(msg.command, out);
    out << "\n";
  }

  // member: target_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target_state: ";
    rosidl_generator_traits::value_to_yaml(msg.target_state, out);
    out << "\n";
  }

  // member: target_position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target_position: ";
    rosidl_generator_traits::value_to_yaml(msg.target_position, out);
    out << "\n";
  }

  // member: use_target_position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "use_target_position: ";
    rosidl_generator_traits::value_to_yaml(msg.use_target_position, out);
    out << "\n";
  }

  // member: force
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "force: ";
    rosidl_generator_traits::value_to_yaml(msg.force, out);
    out << "\n";
  }

  // member: navigate_to_staging_pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "navigate_to_staging_pose: ";
    rosidl_generator_traits::value_to_yaml(msg.navigate_to_staging_pose, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const OperateCabinetControl_Goal & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace xczs_inspection_robot_control

namespace rosidl_generator_traits
{

[[deprecated("use xczs_inspection_robot_control::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const xczs_inspection_robot_control::action::OperateCabinetControl_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  xczs_inspection_robot_control::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use xczs_inspection_robot_control::action::to_yaml() instead")]]
inline std::string to_yaml(const xczs_inspection_robot_control::action::OperateCabinetControl_Goal & msg)
{
  return xczs_inspection_robot_control::action::to_yaml(msg);
}

template<>
inline const char * data_type<xczs_inspection_robot_control::action::OperateCabinetControl_Goal>()
{
  return "xczs_inspection_robot_control::action::OperateCabinetControl_Goal";
}

template<>
inline const char * name<xczs_inspection_robot_control::action::OperateCabinetControl_Goal>()
{
  return "xczs_inspection_robot_control/action/OperateCabinetControl_Goal";
}

template<>
struct has_fixed_size<xczs_inspection_robot_control::action::OperateCabinetControl_Goal>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<xczs_inspection_robot_control::action::OperateCabinetControl_Goal>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<xczs_inspection_robot_control::action::OperateCabinetControl_Goal>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace xczs_inspection_robot_control
{

namespace action
{

inline void to_flow_style_yaml(
  const OperateCabinetControl_Result & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: error_code
  {
    out << "error_code: ";
    rosidl_generator_traits::value_to_yaml(msg.error_code, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << ", ";
  }

  // member: initial_position
  {
    out << "initial_position: ";
    rosidl_generator_traits::value_to_yaml(msg.initial_position, out);
    out << ", ";
  }

  // member: final_position
  {
    out << "final_position: ";
    rosidl_generator_traits::value_to_yaml(msg.final_position, out);
    out << ", ";
  }

  // member: peak_position
  {
    out << "peak_position: ";
    rosidl_generator_traits::value_to_yaml(msg.peak_position, out);
    out << ", ";
  }

  // member: final_state
  {
    out << "final_state: ";
    rosidl_generator_traits::value_to_yaml(msg.final_state, out);
    out << ", ";
  }

  // member: requested_force
  {
    out << "requested_force: ";
    rosidl_generator_traits::value_to_yaml(msg.requested_force, out);
    out << ", ";
  }

  // member: estimated_force
  {
    out << "estimated_force: ";
    rosidl_generator_traits::value_to_yaml(msg.estimated_force, out);
    out << ", ";
  }

  // member: button_triggered
  {
    out << "button_triggered: ";
    rosidl_generator_traits::value_to_yaml(msg.button_triggered, out);
    out << ", ";
  }

  // member: validation_performed
  {
    out << "validation_performed: ";
    rosidl_generator_traits::value_to_yaml(msg.validation_performed, out);
    out << ", ";
  }

  // member: operation_executed
  {
    out << "operation_executed: ";
    rosidl_generator_traits::value_to_yaml(msg.operation_executed, out);
    out << ", ";
  }

  // member: diagnostic_stage
  {
    out << "diagnostic_stage: ";
    rosidl_generator_traits::value_to_yaml(msg.diagnostic_stage, out);
    out << ", ";
  }

  // member: path_fraction
  {
    out << "path_fraction: ";
    rosidl_generator_traits::value_to_yaml(msg.path_fraction, out);
    out << ", ";
  }

  // member: required_fraction
  {
    out << "required_fraction: ";
    rosidl_generator_traits::value_to_yaml(msg.required_fraction, out);
    out << ", ";
  }

  // member: moveit_error_code
  {
    out << "moveit_error_code: ";
    rosidl_generator_traits::value_to_yaml(msg.moveit_error_code, out);
    out << ", ";
  }

  // member: policy_reason
  {
    out << "policy_reason: ";
    rosidl_generator_traits::value_to_yaml(msg.policy_reason, out);
    out << ", ";
  }

  // member: failure_reason
  {
    out << "failure_reason: ";
    rosidl_generator_traits::value_to_yaml(msg.failure_reason, out);
    out << ", ";
  }

  // member: physical_outcome_confirmed
  {
    out << "physical_outcome_confirmed: ";
    rosidl_generator_traits::value_to_yaml(msg.physical_outcome_confirmed, out);
    out << ", ";
  }

  // member: final_state_verified
  {
    out << "final_state_verified: ";
    rosidl_generator_traits::value_to_yaml(msg.final_state_verified, out);
    out << ", ";
  }

  // member: transport_succeeded
  {
    out << "transport_succeeded: ";
    rosidl_generator_traits::value_to_yaml(msg.transport_succeeded, out);
    out << ", ";
  }

  // member: recovery_succeeded
  {
    out << "recovery_succeeded: ";
    rosidl_generator_traits::value_to_yaml(msg.recovery_succeeded, out);
    out << ", ";
  }

  // member: grasp_released
  {
    out << "grasp_released: ";
    rosidl_generator_traits::value_to_yaml(msg.grasp_released, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const OperateCabinetControl_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: error_code
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "error_code: ";
    rosidl_generator_traits::value_to_yaml(msg.error_code, out);
    out << "\n";
  }

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }

  // member: initial_position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "initial_position: ";
    rosidl_generator_traits::value_to_yaml(msg.initial_position, out);
    out << "\n";
  }

  // member: final_position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "final_position: ";
    rosidl_generator_traits::value_to_yaml(msg.final_position, out);
    out << "\n";
  }

  // member: peak_position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "peak_position: ";
    rosidl_generator_traits::value_to_yaml(msg.peak_position, out);
    out << "\n";
  }

  // member: final_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "final_state: ";
    rosidl_generator_traits::value_to_yaml(msg.final_state, out);
    out << "\n";
  }

  // member: requested_force
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "requested_force: ";
    rosidl_generator_traits::value_to_yaml(msg.requested_force, out);
    out << "\n";
  }

  // member: estimated_force
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "estimated_force: ";
    rosidl_generator_traits::value_to_yaml(msg.estimated_force, out);
    out << "\n";
  }

  // member: button_triggered
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "button_triggered: ";
    rosidl_generator_traits::value_to_yaml(msg.button_triggered, out);
    out << "\n";
  }

  // member: validation_performed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "validation_performed: ";
    rosidl_generator_traits::value_to_yaml(msg.validation_performed, out);
    out << "\n";
  }

  // member: operation_executed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "operation_executed: ";
    rosidl_generator_traits::value_to_yaml(msg.operation_executed, out);
    out << "\n";
  }

  // member: diagnostic_stage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "diagnostic_stage: ";
    rosidl_generator_traits::value_to_yaml(msg.diagnostic_stage, out);
    out << "\n";
  }

  // member: path_fraction
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "path_fraction: ";
    rosidl_generator_traits::value_to_yaml(msg.path_fraction, out);
    out << "\n";
  }

  // member: required_fraction
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "required_fraction: ";
    rosidl_generator_traits::value_to_yaml(msg.required_fraction, out);
    out << "\n";
  }

  // member: moveit_error_code
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "moveit_error_code: ";
    rosidl_generator_traits::value_to_yaml(msg.moveit_error_code, out);
    out << "\n";
  }

  // member: policy_reason
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "policy_reason: ";
    rosidl_generator_traits::value_to_yaml(msg.policy_reason, out);
    out << "\n";
  }

  // member: failure_reason
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "failure_reason: ";
    rosidl_generator_traits::value_to_yaml(msg.failure_reason, out);
    out << "\n";
  }

  // member: physical_outcome_confirmed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "physical_outcome_confirmed: ";
    rosidl_generator_traits::value_to_yaml(msg.physical_outcome_confirmed, out);
    out << "\n";
  }

  // member: final_state_verified
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "final_state_verified: ";
    rosidl_generator_traits::value_to_yaml(msg.final_state_verified, out);
    out << "\n";
  }

  // member: transport_succeeded
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "transport_succeeded: ";
    rosidl_generator_traits::value_to_yaml(msg.transport_succeeded, out);
    out << "\n";
  }

  // member: recovery_succeeded
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "recovery_succeeded: ";
    rosidl_generator_traits::value_to_yaml(msg.recovery_succeeded, out);
    out << "\n";
  }

  // member: grasp_released
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "grasp_released: ";
    rosidl_generator_traits::value_to_yaml(msg.grasp_released, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const OperateCabinetControl_Result & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace xczs_inspection_robot_control

namespace rosidl_generator_traits
{

[[deprecated("use xczs_inspection_robot_control::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const xczs_inspection_robot_control::action::OperateCabinetControl_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  xczs_inspection_robot_control::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use xczs_inspection_robot_control::action::to_yaml() instead")]]
inline std::string to_yaml(const xczs_inspection_robot_control::action::OperateCabinetControl_Result & msg)
{
  return xczs_inspection_robot_control::action::to_yaml(msg);
}

template<>
inline const char * data_type<xczs_inspection_robot_control::action::OperateCabinetControl_Result>()
{
  return "xczs_inspection_robot_control::action::OperateCabinetControl_Result";
}

template<>
inline const char * name<xczs_inspection_robot_control::action::OperateCabinetControl_Result>()
{
  return "xczs_inspection_robot_control/action/OperateCabinetControl_Result";
}

template<>
struct has_fixed_size<xczs_inspection_robot_control::action::OperateCabinetControl_Result>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<xczs_inspection_robot_control::action::OperateCabinetControl_Result>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<xczs_inspection_robot_control::action::OperateCabinetControl_Result>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace xczs_inspection_robot_control
{

namespace action
{

inline void to_flow_style_yaml(
  const OperateCabinetControl_Feedback & msg,
  std::ostream & out)
{
  out << "{";
  // member: phase
  {
    out << "phase: ";
    rosidl_generator_traits::value_to_yaml(msg.phase, out);
    out << ", ";
  }

  // member: progress
  {
    out << "progress: ";
    rosidl_generator_traits::value_to_yaml(msg.progress, out);
    out << ", ";
  }

  // member: current_position
  {
    out << "current_position: ";
    rosidl_generator_traits::value_to_yaml(msg.current_position, out);
    out << ", ";
  }

  // member: target_position
  {
    out << "target_position: ";
    rosidl_generator_traits::value_to_yaml(msg.target_position, out);
    out << ", ";
  }

  // member: current_state
  {
    out << "current_state: ";
    rosidl_generator_traits::value_to_yaml(msg.current_state, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const OperateCabinetControl_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: phase
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "phase: ";
    rosidl_generator_traits::value_to_yaml(msg.phase, out);
    out << "\n";
  }

  // member: progress
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "progress: ";
    rosidl_generator_traits::value_to_yaml(msg.progress, out);
    out << "\n";
  }

  // member: current_position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_position: ";
    rosidl_generator_traits::value_to_yaml(msg.current_position, out);
    out << "\n";
  }

  // member: target_position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target_position: ";
    rosidl_generator_traits::value_to_yaml(msg.target_position, out);
    out << "\n";
  }

  // member: current_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_state: ";
    rosidl_generator_traits::value_to_yaml(msg.current_state, out);
    out << "\n";
  }

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const OperateCabinetControl_Feedback & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace xczs_inspection_robot_control

namespace rosidl_generator_traits
{

[[deprecated("use xczs_inspection_robot_control::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const xczs_inspection_robot_control::action::OperateCabinetControl_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  xczs_inspection_robot_control::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use xczs_inspection_robot_control::action::to_yaml() instead")]]
inline std::string to_yaml(const xczs_inspection_robot_control::action::OperateCabinetControl_Feedback & msg)
{
  return xczs_inspection_robot_control::action::to_yaml(msg);
}

template<>
inline const char * data_type<xczs_inspection_robot_control::action::OperateCabinetControl_Feedback>()
{
  return "xczs_inspection_robot_control::action::OperateCabinetControl_Feedback";
}

template<>
inline const char * name<xczs_inspection_robot_control::action::OperateCabinetControl_Feedback>()
{
  return "xczs_inspection_robot_control/action/OperateCabinetControl_Feedback";
}

template<>
struct has_fixed_size<xczs_inspection_robot_control::action::OperateCabinetControl_Feedback>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<xczs_inspection_robot_control::action::OperateCabinetControl_Feedback>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<xczs_inspection_robot_control::action::OperateCabinetControl_Feedback>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'goal'
#include "xczs_inspection_robot_control/action/detail/operate_cabinet_control__traits.hpp"

namespace xczs_inspection_robot_control
{

namespace action
{

inline void to_flow_style_yaml(
  const OperateCabinetControl_SendGoal_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: goal
  {
    out << "goal: ";
    to_flow_style_yaml(msg.goal, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const OperateCabinetControl_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: goal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal:\n";
    to_block_style_yaml(msg.goal, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const OperateCabinetControl_SendGoal_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace xczs_inspection_robot_control

namespace rosidl_generator_traits
{

[[deprecated("use xczs_inspection_robot_control::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const xczs_inspection_robot_control::action::OperateCabinetControl_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  xczs_inspection_robot_control::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use xczs_inspection_robot_control::action::to_yaml() instead")]]
inline std::string to_yaml(const xczs_inspection_robot_control::action::OperateCabinetControl_SendGoal_Request & msg)
{
  return xczs_inspection_robot_control::action::to_yaml(msg);
}

template<>
inline const char * data_type<xczs_inspection_robot_control::action::OperateCabinetControl_SendGoal_Request>()
{
  return "xczs_inspection_robot_control::action::OperateCabinetControl_SendGoal_Request";
}

template<>
inline const char * name<xczs_inspection_robot_control::action::OperateCabinetControl_SendGoal_Request>()
{
  return "xczs_inspection_robot_control/action/OperateCabinetControl_SendGoal_Request";
}

template<>
struct has_fixed_size<xczs_inspection_robot_control::action::OperateCabinetControl_SendGoal_Request>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value && has_fixed_size<xczs_inspection_robot_control::action::OperateCabinetControl_Goal>::value> {};

template<>
struct has_bounded_size<xczs_inspection_robot_control::action::OperateCabinetControl_SendGoal_Request>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value && has_bounded_size<xczs_inspection_robot_control::action::OperateCabinetControl_Goal>::value> {};

template<>
struct is_message<xczs_inspection_robot_control::action::OperateCabinetControl_SendGoal_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace xczs_inspection_robot_control
{

namespace action
{

inline void to_flow_style_yaml(
  const OperateCabinetControl_SendGoal_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: accepted
  {
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << ", ";
  }

  // member: stamp
  {
    out << "stamp: ";
    to_flow_style_yaml(msg.stamp, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const OperateCabinetControl_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: accepted
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << "\n";
  }

  // member: stamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stamp:\n";
    to_block_style_yaml(msg.stamp, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const OperateCabinetControl_SendGoal_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace xczs_inspection_robot_control

namespace rosidl_generator_traits
{

[[deprecated("use xczs_inspection_robot_control::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const xczs_inspection_robot_control::action::OperateCabinetControl_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  xczs_inspection_robot_control::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use xczs_inspection_robot_control::action::to_yaml() instead")]]
inline std::string to_yaml(const xczs_inspection_robot_control::action::OperateCabinetControl_SendGoal_Response & msg)
{
  return xczs_inspection_robot_control::action::to_yaml(msg);
}

template<>
inline const char * data_type<xczs_inspection_robot_control::action::OperateCabinetControl_SendGoal_Response>()
{
  return "xczs_inspection_robot_control::action::OperateCabinetControl_SendGoal_Response";
}

template<>
inline const char * name<xczs_inspection_robot_control::action::OperateCabinetControl_SendGoal_Response>()
{
  return "xczs_inspection_robot_control/action/OperateCabinetControl_SendGoal_Response";
}

template<>
struct has_fixed_size<xczs_inspection_robot_control::action::OperateCabinetControl_SendGoal_Response>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<xczs_inspection_robot_control::action::OperateCabinetControl_SendGoal_Response>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<xczs_inspection_robot_control::action::OperateCabinetControl_SendGoal_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<xczs_inspection_robot_control::action::OperateCabinetControl_SendGoal>()
{
  return "xczs_inspection_robot_control::action::OperateCabinetControl_SendGoal";
}

template<>
inline const char * name<xczs_inspection_robot_control::action::OperateCabinetControl_SendGoal>()
{
  return "xczs_inspection_robot_control/action/OperateCabinetControl_SendGoal";
}

template<>
struct has_fixed_size<xczs_inspection_robot_control::action::OperateCabinetControl_SendGoal>
  : std::integral_constant<
    bool,
    has_fixed_size<xczs_inspection_robot_control::action::OperateCabinetControl_SendGoal_Request>::value &&
    has_fixed_size<xczs_inspection_robot_control::action::OperateCabinetControl_SendGoal_Response>::value
  >
{
};

template<>
struct has_bounded_size<xczs_inspection_robot_control::action::OperateCabinetControl_SendGoal>
  : std::integral_constant<
    bool,
    has_bounded_size<xczs_inspection_robot_control::action::OperateCabinetControl_SendGoal_Request>::value &&
    has_bounded_size<xczs_inspection_robot_control::action::OperateCabinetControl_SendGoal_Response>::value
  >
{
};

template<>
struct is_service<xczs_inspection_robot_control::action::OperateCabinetControl_SendGoal>
  : std::true_type
{
};

template<>
struct is_service_request<xczs_inspection_robot_control::action::OperateCabinetControl_SendGoal_Request>
  : std::true_type
{
};

template<>
struct is_service_response<xczs_inspection_robot_control::action::OperateCabinetControl_SendGoal_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"

namespace xczs_inspection_robot_control
{

namespace action
{

inline void to_flow_style_yaml(
  const OperateCabinetControl_GetResult_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const OperateCabinetControl_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const OperateCabinetControl_GetResult_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace xczs_inspection_robot_control

namespace rosidl_generator_traits
{

[[deprecated("use xczs_inspection_robot_control::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const xczs_inspection_robot_control::action::OperateCabinetControl_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  xczs_inspection_robot_control::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use xczs_inspection_robot_control::action::to_yaml() instead")]]
inline std::string to_yaml(const xczs_inspection_robot_control::action::OperateCabinetControl_GetResult_Request & msg)
{
  return xczs_inspection_robot_control::action::to_yaml(msg);
}

template<>
inline const char * data_type<xczs_inspection_robot_control::action::OperateCabinetControl_GetResult_Request>()
{
  return "xczs_inspection_robot_control::action::OperateCabinetControl_GetResult_Request";
}

template<>
inline const char * name<xczs_inspection_robot_control::action::OperateCabinetControl_GetResult_Request>()
{
  return "xczs_inspection_robot_control/action/OperateCabinetControl_GetResult_Request";
}

template<>
struct has_fixed_size<xczs_inspection_robot_control::action::OperateCabinetControl_GetResult_Request>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<xczs_inspection_robot_control::action::OperateCabinetControl_GetResult_Request>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<xczs_inspection_robot_control::action::OperateCabinetControl_GetResult_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'result'
// already included above
// #include "xczs_inspection_robot_control/action/detail/operate_cabinet_control__traits.hpp"

namespace xczs_inspection_robot_control
{

namespace action
{

inline void to_flow_style_yaml(
  const OperateCabinetControl_GetResult_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << ", ";
  }

  // member: result
  {
    out << "result: ";
    to_flow_style_yaml(msg.result, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const OperateCabinetControl_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }

  // member: result
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "result:\n";
    to_block_style_yaml(msg.result, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const OperateCabinetControl_GetResult_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace xczs_inspection_robot_control

namespace rosidl_generator_traits
{

[[deprecated("use xczs_inspection_robot_control::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const xczs_inspection_robot_control::action::OperateCabinetControl_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  xczs_inspection_robot_control::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use xczs_inspection_robot_control::action::to_yaml() instead")]]
inline std::string to_yaml(const xczs_inspection_robot_control::action::OperateCabinetControl_GetResult_Response & msg)
{
  return xczs_inspection_robot_control::action::to_yaml(msg);
}

template<>
inline const char * data_type<xczs_inspection_robot_control::action::OperateCabinetControl_GetResult_Response>()
{
  return "xczs_inspection_robot_control::action::OperateCabinetControl_GetResult_Response";
}

template<>
inline const char * name<xczs_inspection_robot_control::action::OperateCabinetControl_GetResult_Response>()
{
  return "xczs_inspection_robot_control/action/OperateCabinetControl_GetResult_Response";
}

template<>
struct has_fixed_size<xczs_inspection_robot_control::action::OperateCabinetControl_GetResult_Response>
  : std::integral_constant<bool, has_fixed_size<xczs_inspection_robot_control::action::OperateCabinetControl_Result>::value> {};

template<>
struct has_bounded_size<xczs_inspection_robot_control::action::OperateCabinetControl_GetResult_Response>
  : std::integral_constant<bool, has_bounded_size<xczs_inspection_robot_control::action::OperateCabinetControl_Result>::value> {};

template<>
struct is_message<xczs_inspection_robot_control::action::OperateCabinetControl_GetResult_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<xczs_inspection_robot_control::action::OperateCabinetControl_GetResult>()
{
  return "xczs_inspection_robot_control::action::OperateCabinetControl_GetResult";
}

template<>
inline const char * name<xczs_inspection_robot_control::action::OperateCabinetControl_GetResult>()
{
  return "xczs_inspection_robot_control/action/OperateCabinetControl_GetResult";
}

template<>
struct has_fixed_size<xczs_inspection_robot_control::action::OperateCabinetControl_GetResult>
  : std::integral_constant<
    bool,
    has_fixed_size<xczs_inspection_robot_control::action::OperateCabinetControl_GetResult_Request>::value &&
    has_fixed_size<xczs_inspection_robot_control::action::OperateCabinetControl_GetResult_Response>::value
  >
{
};

template<>
struct has_bounded_size<xczs_inspection_robot_control::action::OperateCabinetControl_GetResult>
  : std::integral_constant<
    bool,
    has_bounded_size<xczs_inspection_robot_control::action::OperateCabinetControl_GetResult_Request>::value &&
    has_bounded_size<xczs_inspection_robot_control::action::OperateCabinetControl_GetResult_Response>::value
  >
{
};

template<>
struct is_service<xczs_inspection_robot_control::action::OperateCabinetControl_GetResult>
  : std::true_type
{
};

template<>
struct is_service_request<xczs_inspection_robot_control::action::OperateCabinetControl_GetResult_Request>
  : std::true_type
{
};

template<>
struct is_service_response<xczs_inspection_robot_control::action::OperateCabinetControl_GetResult_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'feedback'
// already included above
// #include "xczs_inspection_robot_control/action/detail/operate_cabinet_control__traits.hpp"

namespace xczs_inspection_robot_control
{

namespace action
{

inline void to_flow_style_yaml(
  const OperateCabinetControl_FeedbackMessage & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: feedback
  {
    out << "feedback: ";
    to_flow_style_yaml(msg.feedback, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const OperateCabinetControl_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: feedback
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "feedback:\n";
    to_block_style_yaml(msg.feedback, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const OperateCabinetControl_FeedbackMessage & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace xczs_inspection_robot_control

namespace rosidl_generator_traits
{

[[deprecated("use xczs_inspection_robot_control::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const xczs_inspection_robot_control::action::OperateCabinetControl_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  xczs_inspection_robot_control::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use xczs_inspection_robot_control::action::to_yaml() instead")]]
inline std::string to_yaml(const xczs_inspection_robot_control::action::OperateCabinetControl_FeedbackMessage & msg)
{
  return xczs_inspection_robot_control::action::to_yaml(msg);
}

template<>
inline const char * data_type<xczs_inspection_robot_control::action::OperateCabinetControl_FeedbackMessage>()
{
  return "xczs_inspection_robot_control::action::OperateCabinetControl_FeedbackMessage";
}

template<>
inline const char * name<xczs_inspection_robot_control::action::OperateCabinetControl_FeedbackMessage>()
{
  return "xczs_inspection_robot_control/action/OperateCabinetControl_FeedbackMessage";
}

template<>
struct has_fixed_size<xczs_inspection_robot_control::action::OperateCabinetControl_FeedbackMessage>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value && has_fixed_size<xczs_inspection_robot_control::action::OperateCabinetControl_Feedback>::value> {};

template<>
struct has_bounded_size<xczs_inspection_robot_control::action::OperateCabinetControl_FeedbackMessage>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value && has_bounded_size<xczs_inspection_robot_control::action::OperateCabinetControl_Feedback>::value> {};

template<>
struct is_message<xczs_inspection_robot_control::action::OperateCabinetControl_FeedbackMessage>
  : std::true_type {};

}  // namespace rosidl_generator_traits


namespace rosidl_generator_traits
{

template<>
struct is_action<xczs_inspection_robot_control::action::OperateCabinetControl>
  : std::true_type
{
};

template<>
struct is_action_goal<xczs_inspection_robot_control::action::OperateCabinetControl_Goal>
  : std::true_type
{
};

template<>
struct is_action_result<xczs_inspection_robot_control::action::OperateCabinetControl_Result>
  : std::true_type
{
};

template<>
struct is_action_feedback<xczs_inspection_robot_control::action::OperateCabinetControl_Feedback>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits


#endif  // XCZS_INSPECTION_ROBOT_CONTROL__ACTION__DETAIL__OPERATE_CABINET_CONTROL__TRAITS_HPP_
