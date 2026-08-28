// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from xczs_inspection_robot_interfaces:msg/CabinetControlState.idl
// generated code does not contain a copyright notice

#ifndef XCZS_INSPECTION_ROBOT_INTERFACES__MSG__DETAIL__CABINET_CONTROL_STATE__TRAITS_HPP_
#define XCZS_INSPECTION_ROBOT_INTERFACES__MSG__DETAIL__CABINET_CONTROL_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "xczs_inspection_robot_interfaces/msg/detail/cabinet_control_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace xczs_inspection_robot_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const CabinetControlState & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: control_id
  {
    out << "control_id: ";
    rosidl_generator_traits::value_to_yaml(msg.control_id, out);
    out << ", ";
  }

  // member: control_type
  {
    out << "control_type: ";
    rosidl_generator_traits::value_to_yaml(msg.control_type, out);
    out << ", ";
  }

  // member: valid
  {
    out << "valid: ";
    rosidl_generator_traits::value_to_yaml(msg.valid, out);
    out << ", ";
  }

  // member: position
  {
    out << "position: ";
    rosidl_generator_traits::value_to_yaml(msg.position, out);
    out << ", ";
  }

  // member: velocity
  {
    out << "velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity, out);
    out << ", ";
  }

  // member: effort
  {
    out << "effort: ";
    rosidl_generator_traits::value_to_yaml(msg.effort, out);
    out << ", ";
  }

  // member: normalized_position
  {
    out << "normalized_position: ";
    rosidl_generator_traits::value_to_yaml(msg.normalized_position, out);
    out << ", ";
  }

  // member: state_id
  {
    out << "state_id: ";
    rosidl_generator_traits::value_to_yaml(msg.state_id, out);
    out << ", ";
  }

  // member: activated
  {
    out << "activated: ";
    rosidl_generator_traits::value_to_yaml(msg.activated, out);
    out << ", ";
  }

  // member: in_motion
  {
    out << "in_motion: ";
    rosidl_generator_traits::value_to_yaml(msg.in_motion, out);
    out << ", ";
  }

  // member: transition_sequence
  {
    out << "transition_sequence: ";
    rosidl_generator_traits::value_to_yaml(msg.transition_sequence, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CabinetControlState & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: control_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "control_id: ";
    rosidl_generator_traits::value_to_yaml(msg.control_id, out);
    out << "\n";
  }

  // member: control_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "control_type: ";
    rosidl_generator_traits::value_to_yaml(msg.control_type, out);
    out << "\n";
  }

  // member: valid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "valid: ";
    rosidl_generator_traits::value_to_yaml(msg.valid, out);
    out << "\n";
  }

  // member: position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "position: ";
    rosidl_generator_traits::value_to_yaml(msg.position, out);
    out << "\n";
  }

  // member: velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity, out);
    out << "\n";
  }

  // member: effort
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "effort: ";
    rosidl_generator_traits::value_to_yaml(msg.effort, out);
    out << "\n";
  }

  // member: normalized_position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "normalized_position: ";
    rosidl_generator_traits::value_to_yaml(msg.normalized_position, out);
    out << "\n";
  }

  // member: state_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "state_id: ";
    rosidl_generator_traits::value_to_yaml(msg.state_id, out);
    out << "\n";
  }

  // member: activated
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "activated: ";
    rosidl_generator_traits::value_to_yaml(msg.activated, out);
    out << "\n";
  }

  // member: in_motion
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "in_motion: ";
    rosidl_generator_traits::value_to_yaml(msg.in_motion, out);
    out << "\n";
  }

  // member: transition_sequence
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "transition_sequence: ";
    rosidl_generator_traits::value_to_yaml(msg.transition_sequence, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CabinetControlState & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace xczs_inspection_robot_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use xczs_inspection_robot_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const xczs_inspection_robot_interfaces::msg::CabinetControlState & msg,
  std::ostream & out, size_t indentation = 0)
{
  xczs_inspection_robot_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use xczs_inspection_robot_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const xczs_inspection_robot_interfaces::msg::CabinetControlState & msg)
{
  return xczs_inspection_robot_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<xczs_inspection_robot_interfaces::msg::CabinetControlState>()
{
  return "xczs_inspection_robot_interfaces::msg::CabinetControlState";
}

template<>
inline const char * name<xczs_inspection_robot_interfaces::msg::CabinetControlState>()
{
  return "xczs_inspection_robot_interfaces/msg/CabinetControlState";
}

template<>
struct has_fixed_size<xczs_inspection_robot_interfaces::msg::CabinetControlState>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<xczs_inspection_robot_interfaces::msg::CabinetControlState>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<xczs_inspection_robot_interfaces::msg::CabinetControlState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // XCZS_INSPECTION_ROBOT_INTERFACES__MSG__DETAIL__CABINET_CONTROL_STATE__TRAITS_HPP_
