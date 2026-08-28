// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from xczs_inspection_robot_interfaces:srv/SwitchToolset.idl
// generated code does not contain a copyright notice

#ifndef XCZS_INSPECTION_ROBOT_INTERFACES__SRV__DETAIL__SWITCH_TOOLSET__TRAITS_HPP_
#define XCZS_INSPECTION_ROBOT_INTERFACES__SRV__DETAIL__SWITCH_TOOLSET__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "xczs_inspection_robot_interfaces/srv/detail/switch_toolset__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace xczs_inspection_robot_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const SwitchToolset_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: toolset
  {
    out << "toolset: ";
    rosidl_generator_traits::value_to_yaml(msg.toolset, out);
    out << ", ";
  }

  // member: expected_generation
  {
    out << "expected_generation: ";
    rosidl_generator_traits::value_to_yaml(msg.expected_generation, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SwitchToolset_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: toolset
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "toolset: ";
    rosidl_generator_traits::value_to_yaml(msg.toolset, out);
    out << "\n";
  }

  // member: expected_generation
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "expected_generation: ";
    rosidl_generator_traits::value_to_yaml(msg.expected_generation, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SwitchToolset_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace xczs_inspection_robot_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use xczs_inspection_robot_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const xczs_inspection_robot_interfaces::srv::SwitchToolset_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  xczs_inspection_robot_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use xczs_inspection_robot_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const xczs_inspection_robot_interfaces::srv::SwitchToolset_Request & msg)
{
  return xczs_inspection_robot_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<xczs_inspection_robot_interfaces::srv::SwitchToolset_Request>()
{
  return "xczs_inspection_robot_interfaces::srv::SwitchToolset_Request";
}

template<>
inline const char * name<xczs_inspection_robot_interfaces::srv::SwitchToolset_Request>()
{
  return "xczs_inspection_robot_interfaces/srv/SwitchToolset_Request";
}

template<>
struct has_fixed_size<xczs_inspection_robot_interfaces::srv::SwitchToolset_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<xczs_inspection_robot_interfaces::srv::SwitchToolset_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<xczs_inspection_robot_interfaces::srv::SwitchToolset_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace xczs_inspection_robot_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const SwitchToolset_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: accepted
  {
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << ", ";
  }

  // member: state
  {
    out << "state: ";
    rosidl_generator_traits::value_to_yaml(msg.state, out);
    out << ", ";
  }

  // member: active_toolset
  {
    out << "active_toolset: ";
    rosidl_generator_traits::value_to_yaml(msg.active_toolset, out);
    out << ", ";
  }

  // member: target_toolset
  {
    out << "target_toolset: ";
    rosidl_generator_traits::value_to_yaml(msg.target_toolset, out);
    out << ", ";
  }

  // member: generation
  {
    out << "generation: ";
    rosidl_generator_traits::value_to_yaml(msg.generation, out);
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
  const SwitchToolset_Response & msg,
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

  // member: state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "state: ";
    rosidl_generator_traits::value_to_yaml(msg.state, out);
    out << "\n";
  }

  // member: active_toolset
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "active_toolset: ";
    rosidl_generator_traits::value_to_yaml(msg.active_toolset, out);
    out << "\n";
  }

  // member: target_toolset
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target_toolset: ";
    rosidl_generator_traits::value_to_yaml(msg.target_toolset, out);
    out << "\n";
  }

  // member: generation
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "generation: ";
    rosidl_generator_traits::value_to_yaml(msg.generation, out);
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

inline std::string to_yaml(const SwitchToolset_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace xczs_inspection_robot_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use xczs_inspection_robot_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const xczs_inspection_robot_interfaces::srv::SwitchToolset_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  xczs_inspection_robot_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use xczs_inspection_robot_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const xczs_inspection_robot_interfaces::srv::SwitchToolset_Response & msg)
{
  return xczs_inspection_robot_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<xczs_inspection_robot_interfaces::srv::SwitchToolset_Response>()
{
  return "xczs_inspection_robot_interfaces::srv::SwitchToolset_Response";
}

template<>
inline const char * name<xczs_inspection_robot_interfaces::srv::SwitchToolset_Response>()
{
  return "xczs_inspection_robot_interfaces/srv/SwitchToolset_Response";
}

template<>
struct has_fixed_size<xczs_inspection_robot_interfaces::srv::SwitchToolset_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<xczs_inspection_robot_interfaces::srv::SwitchToolset_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<xczs_inspection_robot_interfaces::srv::SwitchToolset_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<xczs_inspection_robot_interfaces::srv::SwitchToolset>()
{
  return "xczs_inspection_robot_interfaces::srv::SwitchToolset";
}

template<>
inline const char * name<xczs_inspection_robot_interfaces::srv::SwitchToolset>()
{
  return "xczs_inspection_robot_interfaces/srv/SwitchToolset";
}

template<>
struct has_fixed_size<xczs_inspection_robot_interfaces::srv::SwitchToolset>
  : std::integral_constant<
    bool,
    has_fixed_size<xczs_inspection_robot_interfaces::srv::SwitchToolset_Request>::value &&
    has_fixed_size<xczs_inspection_robot_interfaces::srv::SwitchToolset_Response>::value
  >
{
};

template<>
struct has_bounded_size<xczs_inspection_robot_interfaces::srv::SwitchToolset>
  : std::integral_constant<
    bool,
    has_bounded_size<xczs_inspection_robot_interfaces::srv::SwitchToolset_Request>::value &&
    has_bounded_size<xczs_inspection_robot_interfaces::srv::SwitchToolset_Response>::value
  >
{
};

template<>
struct is_service<xczs_inspection_robot_interfaces::srv::SwitchToolset>
  : std::true_type
{
};

template<>
struct is_service_request<xczs_inspection_robot_interfaces::srv::SwitchToolset_Request>
  : std::true_type
{
};

template<>
struct is_service_response<xczs_inspection_robot_interfaces::srv::SwitchToolset_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // XCZS_INSPECTION_ROBOT_INTERFACES__SRV__DETAIL__SWITCH_TOOLSET__TRAITS_HPP_
