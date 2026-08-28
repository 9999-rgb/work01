// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from xczs_inspection_robot_control:srv/ManageOperationLease.idl
// generated code does not contain a copyright notice

#ifndef XCZS_INSPECTION_ROBOT_CONTROL__SRV__DETAIL__MANAGE_OPERATION_LEASE__TRAITS_HPP_
#define XCZS_INSPECTION_ROBOT_CONTROL__SRV__DETAIL__MANAGE_OPERATION_LEASE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "xczs_inspection_robot_control/srv/detail/manage_operation_lease__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace xczs_inspection_robot_control
{

namespace srv
{

inline void to_flow_style_yaml(
  const ManageOperationLease_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: command
  {
    out << "command: ";
    rosidl_generator_traits::value_to_yaml(msg.command, out);
    out << ", ";
  }

  // member: owner_id
  {
    out << "owner_id: ";
    rosidl_generator_traits::value_to_yaml(msg.owner_id, out);
    out << ", ";
  }

  // member: lease_id
  {
    out << "lease_id: ";
    rosidl_generator_traits::value_to_yaml(msg.lease_id, out);
    out << ", ";
  }

  // member: requested_duration
  {
    out << "requested_duration: ";
    rosidl_generator_traits::value_to_yaml(msg.requested_duration, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ManageOperationLease_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: command
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "command: ";
    rosidl_generator_traits::value_to_yaml(msg.command, out);
    out << "\n";
  }

  // member: owner_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "owner_id: ";
    rosidl_generator_traits::value_to_yaml(msg.owner_id, out);
    out << "\n";
  }

  // member: lease_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "lease_id: ";
    rosidl_generator_traits::value_to_yaml(msg.lease_id, out);
    out << "\n";
  }

  // member: requested_duration
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "requested_duration: ";
    rosidl_generator_traits::value_to_yaml(msg.requested_duration, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ManageOperationLease_Request & msg, bool use_flow_style = false)
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
  const xczs_inspection_robot_control::srv::ManageOperationLease_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  xczs_inspection_robot_control::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use xczs_inspection_robot_control::srv::to_yaml() instead")]]
inline std::string to_yaml(const xczs_inspection_robot_control::srv::ManageOperationLease_Request & msg)
{
  return xczs_inspection_robot_control::srv::to_yaml(msg);
}

template<>
inline const char * data_type<xczs_inspection_robot_control::srv::ManageOperationLease_Request>()
{
  return "xczs_inspection_robot_control::srv::ManageOperationLease_Request";
}

template<>
inline const char * name<xczs_inspection_robot_control::srv::ManageOperationLease_Request>()
{
  return "xczs_inspection_robot_control/srv/ManageOperationLease_Request";
}

template<>
struct has_fixed_size<xczs_inspection_robot_control::srv::ManageOperationLease_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<xczs_inspection_robot_control::srv::ManageOperationLease_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<xczs_inspection_robot_control::srv::ManageOperationLease_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace xczs_inspection_robot_control
{

namespace srv
{

inline void to_flow_style_yaml(
  const ManageOperationLease_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: status_code
  {
    out << "status_code: ";
    rosidl_generator_traits::value_to_yaml(msg.status_code, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << ", ";
  }

  // member: lease_id
  {
    out << "lease_id: ";
    rosidl_generator_traits::value_to_yaml(msg.lease_id, out);
    out << ", ";
  }

  // member: owner_id
  {
    out << "owner_id: ";
    rosidl_generator_traits::value_to_yaml(msg.owner_id, out);
    out << ", ";
  }

  // member: remaining_duration
  {
    out << "remaining_duration: ";
    rosidl_generator_traits::value_to_yaml(msg.remaining_duration, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ManageOperationLease_Response & msg,
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

  // member: status_code
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status_code: ";
    rosidl_generator_traits::value_to_yaml(msg.status_code, out);
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

  // member: lease_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "lease_id: ";
    rosidl_generator_traits::value_to_yaml(msg.lease_id, out);
    out << "\n";
  }

  // member: owner_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "owner_id: ";
    rosidl_generator_traits::value_to_yaml(msg.owner_id, out);
    out << "\n";
  }

  // member: remaining_duration
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "remaining_duration: ";
    rosidl_generator_traits::value_to_yaml(msg.remaining_duration, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ManageOperationLease_Response & msg, bool use_flow_style = false)
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
  const xczs_inspection_robot_control::srv::ManageOperationLease_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  xczs_inspection_robot_control::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use xczs_inspection_robot_control::srv::to_yaml() instead")]]
inline std::string to_yaml(const xczs_inspection_robot_control::srv::ManageOperationLease_Response & msg)
{
  return xczs_inspection_robot_control::srv::to_yaml(msg);
}

template<>
inline const char * data_type<xczs_inspection_robot_control::srv::ManageOperationLease_Response>()
{
  return "xczs_inspection_robot_control::srv::ManageOperationLease_Response";
}

template<>
inline const char * name<xczs_inspection_robot_control::srv::ManageOperationLease_Response>()
{
  return "xczs_inspection_robot_control/srv/ManageOperationLease_Response";
}

template<>
struct has_fixed_size<xczs_inspection_robot_control::srv::ManageOperationLease_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<xczs_inspection_robot_control::srv::ManageOperationLease_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<xczs_inspection_robot_control::srv::ManageOperationLease_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<xczs_inspection_robot_control::srv::ManageOperationLease>()
{
  return "xczs_inspection_robot_control::srv::ManageOperationLease";
}

template<>
inline const char * name<xczs_inspection_robot_control::srv::ManageOperationLease>()
{
  return "xczs_inspection_robot_control/srv/ManageOperationLease";
}

template<>
struct has_fixed_size<xczs_inspection_robot_control::srv::ManageOperationLease>
  : std::integral_constant<
    bool,
    has_fixed_size<xczs_inspection_robot_control::srv::ManageOperationLease_Request>::value &&
    has_fixed_size<xczs_inspection_robot_control::srv::ManageOperationLease_Response>::value
  >
{
};

template<>
struct has_bounded_size<xczs_inspection_robot_control::srv::ManageOperationLease>
  : std::integral_constant<
    bool,
    has_bounded_size<xczs_inspection_robot_control::srv::ManageOperationLease_Request>::value &&
    has_bounded_size<xczs_inspection_robot_control::srv::ManageOperationLease_Response>::value
  >
{
};

template<>
struct is_service<xczs_inspection_robot_control::srv::ManageOperationLease>
  : std::true_type
{
};

template<>
struct is_service_request<xczs_inspection_robot_control::srv::ManageOperationLease_Request>
  : std::true_type
{
};

template<>
struct is_service_response<xczs_inspection_robot_control::srv::ManageOperationLease_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // XCZS_INSPECTION_ROBOT_CONTROL__SRV__DETAIL__MANAGE_OPERATION_LEASE__TRAITS_HPP_
