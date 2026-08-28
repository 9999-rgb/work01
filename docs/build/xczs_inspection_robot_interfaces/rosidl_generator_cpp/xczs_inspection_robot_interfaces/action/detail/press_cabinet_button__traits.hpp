// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from xczs_inspection_robot_interfaces:action/PressCabinetButton.idl
// generated code does not contain a copyright notice

#ifndef XCZS_INSPECTION_ROBOT_INTERFACES__ACTION__DETAIL__PRESS_CABINET_BUTTON__TRAITS_HPP_
#define XCZS_INSPECTION_ROBOT_INTERFACES__ACTION__DETAIL__PRESS_CABINET_BUTTON__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "xczs_inspection_robot_interfaces/action/detail/press_cabinet_button__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace xczs_inspection_robot_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const PressCabinetButton_Goal & msg,
  std::ostream & out)
{
  out << "{";
  // member: button_id
  {
    out << "button_id: ";
    rosidl_generator_traits::value_to_yaml(msg.button_id, out);
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
  const PressCabinetButton_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: button_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "button_id: ";
    rosidl_generator_traits::value_to_yaml(msg.button_id, out);
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

inline std::string to_yaml(const PressCabinetButton_Goal & msg, bool use_flow_style = false)
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

}  // namespace xczs_inspection_robot_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use xczs_inspection_robot_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const xczs_inspection_robot_interfaces::action::PressCabinetButton_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  xczs_inspection_robot_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use xczs_inspection_robot_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const xczs_inspection_robot_interfaces::action::PressCabinetButton_Goal & msg)
{
  return xczs_inspection_robot_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<xczs_inspection_robot_interfaces::action::PressCabinetButton_Goal>()
{
  return "xczs_inspection_robot_interfaces::action::PressCabinetButton_Goal";
}

template<>
inline const char * name<xczs_inspection_robot_interfaces::action::PressCabinetButton_Goal>()
{
  return "xczs_inspection_robot_interfaces/action/PressCabinetButton_Goal";
}

template<>
struct has_fixed_size<xczs_inspection_robot_interfaces::action::PressCabinetButton_Goal>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<xczs_inspection_robot_interfaces::action::PressCabinetButton_Goal>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<xczs_inspection_robot_interfaces::action::PressCabinetButton_Goal>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace xczs_inspection_robot_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const PressCabinetButton_Result & msg,
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

  // member: max_travel
  {
    out << "max_travel: ";
    rosidl_generator_traits::value_to_yaml(msg.max_travel, out);
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
  const PressCabinetButton_Result & msg,
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

  // member: max_travel
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "max_travel: ";
    rosidl_generator_traits::value_to_yaml(msg.max_travel, out);
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

inline std::string to_yaml(const PressCabinetButton_Result & msg, bool use_flow_style = false)
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

}  // namespace xczs_inspection_robot_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use xczs_inspection_robot_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const xczs_inspection_robot_interfaces::action::PressCabinetButton_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  xczs_inspection_robot_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use xczs_inspection_robot_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const xczs_inspection_robot_interfaces::action::PressCabinetButton_Result & msg)
{
  return xczs_inspection_robot_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<xczs_inspection_robot_interfaces::action::PressCabinetButton_Result>()
{
  return "xczs_inspection_robot_interfaces::action::PressCabinetButton_Result";
}

template<>
inline const char * name<xczs_inspection_robot_interfaces::action::PressCabinetButton_Result>()
{
  return "xczs_inspection_robot_interfaces/action/PressCabinetButton_Result";
}

template<>
struct has_fixed_size<xczs_inspection_robot_interfaces::action::PressCabinetButton_Result>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<xczs_inspection_robot_interfaces::action::PressCabinetButton_Result>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<xczs_inspection_robot_interfaces::action::PressCabinetButton_Result>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace xczs_inspection_robot_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const PressCabinetButton_Feedback & msg,
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

  // member: button_travel
  {
    out << "button_travel: ";
    rosidl_generator_traits::value_to_yaml(msg.button_travel, out);
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
  const PressCabinetButton_Feedback & msg,
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

  // member: button_travel
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "button_travel: ";
    rosidl_generator_traits::value_to_yaml(msg.button_travel, out);
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

inline std::string to_yaml(const PressCabinetButton_Feedback & msg, bool use_flow_style = false)
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

}  // namespace xczs_inspection_robot_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use xczs_inspection_robot_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const xczs_inspection_robot_interfaces::action::PressCabinetButton_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  xczs_inspection_robot_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use xczs_inspection_robot_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const xczs_inspection_robot_interfaces::action::PressCabinetButton_Feedback & msg)
{
  return xczs_inspection_robot_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<xczs_inspection_robot_interfaces::action::PressCabinetButton_Feedback>()
{
  return "xczs_inspection_robot_interfaces::action::PressCabinetButton_Feedback";
}

template<>
inline const char * name<xczs_inspection_robot_interfaces::action::PressCabinetButton_Feedback>()
{
  return "xczs_inspection_robot_interfaces/action/PressCabinetButton_Feedback";
}

template<>
struct has_fixed_size<xczs_inspection_robot_interfaces::action::PressCabinetButton_Feedback>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<xczs_inspection_robot_interfaces::action::PressCabinetButton_Feedback>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<xczs_inspection_robot_interfaces::action::PressCabinetButton_Feedback>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'goal'
#include "xczs_inspection_robot_interfaces/action/detail/press_cabinet_button__traits.hpp"

namespace xczs_inspection_robot_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const PressCabinetButton_SendGoal_Request & msg,
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
  const PressCabinetButton_SendGoal_Request & msg,
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

inline std::string to_yaml(const PressCabinetButton_SendGoal_Request & msg, bool use_flow_style = false)
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

}  // namespace xczs_inspection_robot_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use xczs_inspection_robot_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const xczs_inspection_robot_interfaces::action::PressCabinetButton_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  xczs_inspection_robot_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use xczs_inspection_robot_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const xczs_inspection_robot_interfaces::action::PressCabinetButton_SendGoal_Request & msg)
{
  return xczs_inspection_robot_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<xczs_inspection_robot_interfaces::action::PressCabinetButton_SendGoal_Request>()
{
  return "xczs_inspection_robot_interfaces::action::PressCabinetButton_SendGoal_Request";
}

template<>
inline const char * name<xczs_inspection_robot_interfaces::action::PressCabinetButton_SendGoal_Request>()
{
  return "xczs_inspection_robot_interfaces/action/PressCabinetButton_SendGoal_Request";
}

template<>
struct has_fixed_size<xczs_inspection_robot_interfaces::action::PressCabinetButton_SendGoal_Request>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value && has_fixed_size<xczs_inspection_robot_interfaces::action::PressCabinetButton_Goal>::value> {};

template<>
struct has_bounded_size<xczs_inspection_robot_interfaces::action::PressCabinetButton_SendGoal_Request>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value && has_bounded_size<xczs_inspection_robot_interfaces::action::PressCabinetButton_Goal>::value> {};

template<>
struct is_message<xczs_inspection_robot_interfaces::action::PressCabinetButton_SendGoal_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace xczs_inspection_robot_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const PressCabinetButton_SendGoal_Response & msg,
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
  const PressCabinetButton_SendGoal_Response & msg,
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

inline std::string to_yaml(const PressCabinetButton_SendGoal_Response & msg, bool use_flow_style = false)
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

}  // namespace xczs_inspection_robot_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use xczs_inspection_robot_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const xczs_inspection_robot_interfaces::action::PressCabinetButton_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  xczs_inspection_robot_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use xczs_inspection_robot_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const xczs_inspection_robot_interfaces::action::PressCabinetButton_SendGoal_Response & msg)
{
  return xczs_inspection_robot_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<xczs_inspection_robot_interfaces::action::PressCabinetButton_SendGoal_Response>()
{
  return "xczs_inspection_robot_interfaces::action::PressCabinetButton_SendGoal_Response";
}

template<>
inline const char * name<xczs_inspection_robot_interfaces::action::PressCabinetButton_SendGoal_Response>()
{
  return "xczs_inspection_robot_interfaces/action/PressCabinetButton_SendGoal_Response";
}

template<>
struct has_fixed_size<xczs_inspection_robot_interfaces::action::PressCabinetButton_SendGoal_Response>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<xczs_inspection_robot_interfaces::action::PressCabinetButton_SendGoal_Response>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<xczs_inspection_robot_interfaces::action::PressCabinetButton_SendGoal_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<xczs_inspection_robot_interfaces::action::PressCabinetButton_SendGoal>()
{
  return "xczs_inspection_robot_interfaces::action::PressCabinetButton_SendGoal";
}

template<>
inline const char * name<xczs_inspection_robot_interfaces::action::PressCabinetButton_SendGoal>()
{
  return "xczs_inspection_robot_interfaces/action/PressCabinetButton_SendGoal";
}

template<>
struct has_fixed_size<xczs_inspection_robot_interfaces::action::PressCabinetButton_SendGoal>
  : std::integral_constant<
    bool,
    has_fixed_size<xczs_inspection_robot_interfaces::action::PressCabinetButton_SendGoal_Request>::value &&
    has_fixed_size<xczs_inspection_robot_interfaces::action::PressCabinetButton_SendGoal_Response>::value
  >
{
};

template<>
struct has_bounded_size<xczs_inspection_robot_interfaces::action::PressCabinetButton_SendGoal>
  : std::integral_constant<
    bool,
    has_bounded_size<xczs_inspection_robot_interfaces::action::PressCabinetButton_SendGoal_Request>::value &&
    has_bounded_size<xczs_inspection_robot_interfaces::action::PressCabinetButton_SendGoal_Response>::value
  >
{
};

template<>
struct is_service<xczs_inspection_robot_interfaces::action::PressCabinetButton_SendGoal>
  : std::true_type
{
};

template<>
struct is_service_request<xczs_inspection_robot_interfaces::action::PressCabinetButton_SendGoal_Request>
  : std::true_type
{
};

template<>
struct is_service_response<xczs_inspection_robot_interfaces::action::PressCabinetButton_SendGoal_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"

namespace xczs_inspection_robot_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const PressCabinetButton_GetResult_Request & msg,
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
  const PressCabinetButton_GetResult_Request & msg,
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

inline std::string to_yaml(const PressCabinetButton_GetResult_Request & msg, bool use_flow_style = false)
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

}  // namespace xczs_inspection_robot_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use xczs_inspection_robot_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const xczs_inspection_robot_interfaces::action::PressCabinetButton_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  xczs_inspection_robot_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use xczs_inspection_robot_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const xczs_inspection_robot_interfaces::action::PressCabinetButton_GetResult_Request & msg)
{
  return xczs_inspection_robot_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<xczs_inspection_robot_interfaces::action::PressCabinetButton_GetResult_Request>()
{
  return "xczs_inspection_robot_interfaces::action::PressCabinetButton_GetResult_Request";
}

template<>
inline const char * name<xczs_inspection_robot_interfaces::action::PressCabinetButton_GetResult_Request>()
{
  return "xczs_inspection_robot_interfaces/action/PressCabinetButton_GetResult_Request";
}

template<>
struct has_fixed_size<xczs_inspection_robot_interfaces::action::PressCabinetButton_GetResult_Request>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<xczs_inspection_robot_interfaces::action::PressCabinetButton_GetResult_Request>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<xczs_inspection_robot_interfaces::action::PressCabinetButton_GetResult_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'result'
// already included above
// #include "xczs_inspection_robot_interfaces/action/detail/press_cabinet_button__traits.hpp"

namespace xczs_inspection_robot_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const PressCabinetButton_GetResult_Response & msg,
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
  const PressCabinetButton_GetResult_Response & msg,
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

inline std::string to_yaml(const PressCabinetButton_GetResult_Response & msg, bool use_flow_style = false)
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

}  // namespace xczs_inspection_robot_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use xczs_inspection_robot_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const xczs_inspection_robot_interfaces::action::PressCabinetButton_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  xczs_inspection_robot_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use xczs_inspection_robot_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const xczs_inspection_robot_interfaces::action::PressCabinetButton_GetResult_Response & msg)
{
  return xczs_inspection_robot_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<xczs_inspection_robot_interfaces::action::PressCabinetButton_GetResult_Response>()
{
  return "xczs_inspection_robot_interfaces::action::PressCabinetButton_GetResult_Response";
}

template<>
inline const char * name<xczs_inspection_robot_interfaces::action::PressCabinetButton_GetResult_Response>()
{
  return "xczs_inspection_robot_interfaces/action/PressCabinetButton_GetResult_Response";
}

template<>
struct has_fixed_size<xczs_inspection_robot_interfaces::action::PressCabinetButton_GetResult_Response>
  : std::integral_constant<bool, has_fixed_size<xczs_inspection_robot_interfaces::action::PressCabinetButton_Result>::value> {};

template<>
struct has_bounded_size<xczs_inspection_robot_interfaces::action::PressCabinetButton_GetResult_Response>
  : std::integral_constant<bool, has_bounded_size<xczs_inspection_robot_interfaces::action::PressCabinetButton_Result>::value> {};

template<>
struct is_message<xczs_inspection_robot_interfaces::action::PressCabinetButton_GetResult_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<xczs_inspection_robot_interfaces::action::PressCabinetButton_GetResult>()
{
  return "xczs_inspection_robot_interfaces::action::PressCabinetButton_GetResult";
}

template<>
inline const char * name<xczs_inspection_robot_interfaces::action::PressCabinetButton_GetResult>()
{
  return "xczs_inspection_robot_interfaces/action/PressCabinetButton_GetResult";
}

template<>
struct has_fixed_size<xczs_inspection_robot_interfaces::action::PressCabinetButton_GetResult>
  : std::integral_constant<
    bool,
    has_fixed_size<xczs_inspection_robot_interfaces::action::PressCabinetButton_GetResult_Request>::value &&
    has_fixed_size<xczs_inspection_robot_interfaces::action::PressCabinetButton_GetResult_Response>::value
  >
{
};

template<>
struct has_bounded_size<xczs_inspection_robot_interfaces::action::PressCabinetButton_GetResult>
  : std::integral_constant<
    bool,
    has_bounded_size<xczs_inspection_robot_interfaces::action::PressCabinetButton_GetResult_Request>::value &&
    has_bounded_size<xczs_inspection_robot_interfaces::action::PressCabinetButton_GetResult_Response>::value
  >
{
};

template<>
struct is_service<xczs_inspection_robot_interfaces::action::PressCabinetButton_GetResult>
  : std::true_type
{
};

template<>
struct is_service_request<xczs_inspection_robot_interfaces::action::PressCabinetButton_GetResult_Request>
  : std::true_type
{
};

template<>
struct is_service_response<xczs_inspection_robot_interfaces::action::PressCabinetButton_GetResult_Response>
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
// #include "xczs_inspection_robot_interfaces/action/detail/press_cabinet_button__traits.hpp"

namespace xczs_inspection_robot_interfaces
{

namespace action
{

inline void to_flow_style_yaml(
  const PressCabinetButton_FeedbackMessage & msg,
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
  const PressCabinetButton_FeedbackMessage & msg,
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

inline std::string to_yaml(const PressCabinetButton_FeedbackMessage & msg, bool use_flow_style = false)
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

}  // namespace xczs_inspection_robot_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use xczs_inspection_robot_interfaces::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const xczs_inspection_robot_interfaces::action::PressCabinetButton_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  xczs_inspection_robot_interfaces::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use xczs_inspection_robot_interfaces::action::to_yaml() instead")]]
inline std::string to_yaml(const xczs_inspection_robot_interfaces::action::PressCabinetButton_FeedbackMessage & msg)
{
  return xczs_inspection_robot_interfaces::action::to_yaml(msg);
}

template<>
inline const char * data_type<xczs_inspection_robot_interfaces::action::PressCabinetButton_FeedbackMessage>()
{
  return "xczs_inspection_robot_interfaces::action::PressCabinetButton_FeedbackMessage";
}

template<>
inline const char * name<xczs_inspection_robot_interfaces::action::PressCabinetButton_FeedbackMessage>()
{
  return "xczs_inspection_robot_interfaces/action/PressCabinetButton_FeedbackMessage";
}

template<>
struct has_fixed_size<xczs_inspection_robot_interfaces::action::PressCabinetButton_FeedbackMessage>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value && has_fixed_size<xczs_inspection_robot_interfaces::action::PressCabinetButton_Feedback>::value> {};

template<>
struct has_bounded_size<xczs_inspection_robot_interfaces::action::PressCabinetButton_FeedbackMessage>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value && has_bounded_size<xczs_inspection_robot_interfaces::action::PressCabinetButton_Feedback>::value> {};

template<>
struct is_message<xczs_inspection_robot_interfaces::action::PressCabinetButton_FeedbackMessage>
  : std::true_type {};

}  // namespace rosidl_generator_traits


namespace rosidl_generator_traits
{

template<>
struct is_action<xczs_inspection_robot_interfaces::action::PressCabinetButton>
  : std::true_type
{
};

template<>
struct is_action_goal<xczs_inspection_robot_interfaces::action::PressCabinetButton_Goal>
  : std::true_type
{
};

template<>
struct is_action_result<xczs_inspection_robot_interfaces::action::PressCabinetButton_Result>
  : std::true_type
{
};

template<>
struct is_action_feedback<xczs_inspection_robot_interfaces::action::PressCabinetButton_Feedback>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits


#endif  // XCZS_INSPECTION_ROBOT_INTERFACES__ACTION__DETAIL__PRESS_CABINET_BUTTON__TRAITS_HPP_
