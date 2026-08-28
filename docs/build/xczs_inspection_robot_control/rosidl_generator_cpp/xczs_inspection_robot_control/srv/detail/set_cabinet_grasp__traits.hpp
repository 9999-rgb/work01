// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from xczs_inspection_robot_control:srv/SetCabinetGrasp.idl
// generated code does not contain a copyright notice

#ifndef XCZS_INSPECTION_ROBOT_CONTROL__SRV__DETAIL__SET_CABINET_GRASP__TRAITS_HPP_
#define XCZS_INSPECTION_ROBOT_CONTROL__SRV__DETAIL__SET_CABINET_GRASP__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "xczs_inspection_robot_control/srv/detail/set_cabinet_grasp__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'robot_grasp_point'
#include "geometry_msgs/msg/detail/point__traits.hpp"

namespace xczs_inspection_robot_control
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetCabinetGrasp_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: control_id
  {
    out << "control_id: ";
    rosidl_generator_traits::value_to_yaml(msg.control_id, out);
    out << ", ";
  }

  // member: operation_lease_id
  {
    out << "operation_lease_id: ";
    rosidl_generator_traits::value_to_yaml(msg.operation_lease_id, out);
    out << ", ";
  }

  // member: robot_model
  {
    out << "robot_model: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_model, out);
    out << ", ";
  }

  // member: robot_link
  {
    out << "robot_link: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_link, out);
    out << ", ";
  }

  // member: robot_grasp_point
  {
    out << "robot_grasp_point: ";
    to_flow_style_yaml(msg.robot_grasp_point, out);
    out << ", ";
  }

  // member: robot_base_link
  {
    out << "robot_base_link: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_base_link, out);
    out << ", ";
  }

  // member: attach
  {
    out << "attach: ";
    rosidl_generator_traits::value_to_yaml(msg.attach, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetCabinetGrasp_Request & msg,
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

  // member: operation_lease_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "operation_lease_id: ";
    rosidl_generator_traits::value_to_yaml(msg.operation_lease_id, out);
    out << "\n";
  }

  // member: robot_model
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "robot_model: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_model, out);
    out << "\n";
  }

  // member: robot_link
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "robot_link: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_link, out);
    out << "\n";
  }

  // member: robot_grasp_point
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "robot_grasp_point:\n";
    to_block_style_yaml(msg.robot_grasp_point, out, indentation + 2);
  }

  // member: robot_base_link
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "robot_base_link: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_base_link, out);
    out << "\n";
  }

  // member: attach
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "attach: ";
    rosidl_generator_traits::value_to_yaml(msg.attach, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetCabinetGrasp_Request & msg, bool use_flow_style = false)
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

}  // namespace xczs_inspection_robot_control

namespace rosidl_generator_traits
{

[[deprecated("use xczs_inspection_robot_control::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const xczs_inspection_robot_control::srv::SetCabinetGrasp_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  xczs_inspection_robot_control::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use xczs_inspection_robot_control::srv::to_yaml() instead")]]
inline std::string to_yaml(const xczs_inspection_robot_control::srv::SetCabinetGrasp_Request & msg)
{
  return xczs_inspection_robot_control::srv::to_yaml(msg);
}

template<>
inline const char * data_type<xczs_inspection_robot_control::srv::SetCabinetGrasp_Request>()
{
  return "xczs_inspection_robot_control::srv::SetCabinetGrasp_Request";
}

template<>
inline const char * name<xczs_inspection_robot_control::srv::SetCabinetGrasp_Request>()
{
  return "xczs_inspection_robot_control/srv/SetCabinetGrasp_Request";
}

template<>
struct has_fixed_size<xczs_inspection_robot_control::srv::SetCabinetGrasp_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<xczs_inspection_robot_control::srv::SetCabinetGrasp_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<xczs_inspection_robot_control::srv::SetCabinetGrasp_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace xczs_inspection_robot_control
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetCabinetGrasp_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << ", ";
  }

  // member: distance
  {
    out << "distance: ";
    rosidl_generator_traits::value_to_yaml(msg.distance, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetCabinetGrasp_Response & msg,
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

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }

  // member: distance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "distance: ";
    rosidl_generator_traits::value_to_yaml(msg.distance, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetCabinetGrasp_Response & msg, bool use_flow_style = false)
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

}  // namespace xczs_inspection_robot_control

namespace rosidl_generator_traits
{

[[deprecated("use xczs_inspection_robot_control::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const xczs_inspection_robot_control::srv::SetCabinetGrasp_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  xczs_inspection_robot_control::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use xczs_inspection_robot_control::srv::to_yaml() instead")]]
inline std::string to_yaml(const xczs_inspection_robot_control::srv::SetCabinetGrasp_Response & msg)
{
  return xczs_inspection_robot_control::srv::to_yaml(msg);
}

template<>
inline const char * data_type<xczs_inspection_robot_control::srv::SetCabinetGrasp_Response>()
{
  return "xczs_inspection_robot_control::srv::SetCabinetGrasp_Response";
}

template<>
inline const char * name<xczs_inspection_robot_control::srv::SetCabinetGrasp_Response>()
{
  return "xczs_inspection_robot_control/srv/SetCabinetGrasp_Response";
}

template<>
struct has_fixed_size<xczs_inspection_robot_control::srv::SetCabinetGrasp_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<xczs_inspection_robot_control::srv::SetCabinetGrasp_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<xczs_inspection_robot_control::srv::SetCabinetGrasp_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<xczs_inspection_robot_control::srv::SetCabinetGrasp>()
{
  return "xczs_inspection_robot_control::srv::SetCabinetGrasp";
}

template<>
inline const char * name<xczs_inspection_robot_control::srv::SetCabinetGrasp>()
{
  return "xczs_inspection_robot_control/srv/SetCabinetGrasp";
}

template<>
struct has_fixed_size<xczs_inspection_robot_control::srv::SetCabinetGrasp>
  : std::integral_constant<
    bool,
    has_fixed_size<xczs_inspection_robot_control::srv::SetCabinetGrasp_Request>::value &&
    has_fixed_size<xczs_inspection_robot_control::srv::SetCabinetGrasp_Response>::value
  >
{
};

template<>
struct has_bounded_size<xczs_inspection_robot_control::srv::SetCabinetGrasp>
  : std::integral_constant<
    bool,
    has_bounded_size<xczs_inspection_robot_control::srv::SetCabinetGrasp_Request>::value &&
    has_bounded_size<xczs_inspection_robot_control::srv::SetCabinetGrasp_Response>::value
  >
{
};

template<>
struct is_service<xczs_inspection_robot_control::srv::SetCabinetGrasp>
  : std::true_type
{
};

template<>
struct is_service_request<xczs_inspection_robot_control::srv::SetCabinetGrasp_Request>
  : std::true_type
{
};

template<>
struct is_service_response<xczs_inspection_robot_control::srv::SetCabinetGrasp_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // XCZS_INSPECTION_ROBOT_CONTROL__SRV__DETAIL__SET_CABINET_GRASP__TRAITS_HPP_
