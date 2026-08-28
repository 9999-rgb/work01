// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from xczs_inspection_robot_control:srv/ManageOperationLease.idl
// generated code does not contain a copyright notice

#ifndef XCZS_INSPECTION_ROBOT_CONTROL__SRV__DETAIL__MANAGE_OPERATION_LEASE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define XCZS_INSPECTION_ROBOT_CONTROL__SRV__DETAIL__MANAGE_OPERATION_LEASE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "xczs_inspection_robot_control/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "xczs_inspection_robot_control/srv/detail/manage_operation_lease__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace xczs_inspection_robot_control
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_xczs_inspection_robot_control
cdr_serialize(
  const xczs_inspection_robot_control::srv::ManageOperationLease_Request & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_xczs_inspection_robot_control
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  xczs_inspection_robot_control::srv::ManageOperationLease_Request & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_xczs_inspection_robot_control
get_serialized_size(
  const xczs_inspection_robot_control::srv::ManageOperationLease_Request & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_xczs_inspection_robot_control
max_serialized_size_ManageOperationLease_Request(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace xczs_inspection_robot_control

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_xczs_inspection_robot_control
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, xczs_inspection_robot_control, srv, ManageOperationLease_Request)();

#ifdef __cplusplus
}
#endif

// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "xczs_inspection_robot_control/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
// already included above
// #include "xczs_inspection_robot_control/srv/detail/manage_operation_lease__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// already included above
// #include "fastcdr/Cdr.h"

namespace xczs_inspection_robot_control
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_xczs_inspection_robot_control
cdr_serialize(
  const xczs_inspection_robot_control::srv::ManageOperationLease_Response & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_xczs_inspection_robot_control
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  xczs_inspection_robot_control::srv::ManageOperationLease_Response & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_xczs_inspection_robot_control
get_serialized_size(
  const xczs_inspection_robot_control::srv::ManageOperationLease_Response & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_xczs_inspection_robot_control
max_serialized_size_ManageOperationLease_Response(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace xczs_inspection_robot_control

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_xczs_inspection_robot_control
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, xczs_inspection_robot_control, srv, ManageOperationLease_Response)();

#ifdef __cplusplus
}
#endif

#include "rmw/types.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "xczs_inspection_robot_control/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_xczs_inspection_robot_control
const rosidl_service_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, xczs_inspection_robot_control, srv, ManageOperationLease)();

#ifdef __cplusplus
}
#endif

#endif  // XCZS_INSPECTION_ROBOT_CONTROL__SRV__DETAIL__MANAGE_OPERATION_LEASE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
