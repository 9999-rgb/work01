// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from xczs_inspection_robot_interfaces:srv/SwitchToolset.idl
// generated code does not contain a copyright notice

#ifndef XCZS_INSPECTION_ROBOT_INTERFACES__SRV__DETAIL__SWITCH_TOOLSET__STRUCT_H_
#define XCZS_INSPECTION_ROBOT_INTERFACES__SRV__DETAIL__SWITCH_TOOLSET__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'toolset'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/SwitchToolset in the package xczs_inspection_robot_interfaces.
typedef struct xczs_inspection_robot_interfaces__srv__SwitchToolset_Request
{
  rosidl_runtime_c__String toolset;
  uint64_t expected_generation;
} xczs_inspection_robot_interfaces__srv__SwitchToolset_Request;

// Struct for a sequence of xczs_inspection_robot_interfaces__srv__SwitchToolset_Request.
typedef struct xczs_inspection_robot_interfaces__srv__SwitchToolset_Request__Sequence
{
  xczs_inspection_robot_interfaces__srv__SwitchToolset_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} xczs_inspection_robot_interfaces__srv__SwitchToolset_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'state'
// Member 'active_toolset'
// Member 'target_toolset'
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/SwitchToolset in the package xczs_inspection_robot_interfaces.
typedef struct xczs_inspection_robot_interfaces__srv__SwitchToolset_Response
{
  bool accepted;
  rosidl_runtime_c__String state;
  rosidl_runtime_c__String active_toolset;
  rosidl_runtime_c__String target_toolset;
  uint64_t generation;
  rosidl_runtime_c__String message;
} xczs_inspection_robot_interfaces__srv__SwitchToolset_Response;

// Struct for a sequence of xczs_inspection_robot_interfaces__srv__SwitchToolset_Response.
typedef struct xczs_inspection_robot_interfaces__srv__SwitchToolset_Response__Sequence
{
  xczs_inspection_robot_interfaces__srv__SwitchToolset_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} xczs_inspection_robot_interfaces__srv__SwitchToolset_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // XCZS_INSPECTION_ROBOT_INTERFACES__SRV__DETAIL__SWITCH_TOOLSET__STRUCT_H_
