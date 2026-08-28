// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from xczs_inspection_robot_control:srv/SetCabinetGrasp.idl
// generated code does not contain a copyright notice

#ifndef XCZS_INSPECTION_ROBOT_CONTROL__SRV__DETAIL__SET_CABINET_GRASP__STRUCT_H_
#define XCZS_INSPECTION_ROBOT_CONTROL__SRV__DETAIL__SET_CABINET_GRASP__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'control_id'
// Member 'operation_lease_id'
// Member 'robot_model'
// Member 'robot_link'
// Member 'robot_base_link'
#include "rosidl_runtime_c/string.h"
// Member 'robot_grasp_point'
#include "geometry_msgs/msg/detail/point__struct.h"

/// Struct defined in srv/SetCabinetGrasp in the package xczs_inspection_robot_control.
typedef struct xczs_inspection_robot_control__srv__SetCabinetGrasp_Request
{
  rosidl_runtime_c__String control_id;
  rosidl_runtime_c__String operation_lease_id;
  rosidl_runtime_c__String robot_model;
  rosidl_runtime_c__String robot_link;
  geometry_msgs__msg__Point robot_grasp_point;
  rosidl_runtime_c__String robot_base_link;
  bool attach;
} xczs_inspection_robot_control__srv__SetCabinetGrasp_Request;

// Struct for a sequence of xczs_inspection_robot_control__srv__SetCabinetGrasp_Request.
typedef struct xczs_inspection_robot_control__srv__SetCabinetGrasp_Request__Sequence
{
  xczs_inspection_robot_control__srv__SetCabinetGrasp_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} xczs_inspection_robot_control__srv__SetCabinetGrasp_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/SetCabinetGrasp in the package xczs_inspection_robot_control.
typedef struct xczs_inspection_robot_control__srv__SetCabinetGrasp_Response
{
  bool success;
  rosidl_runtime_c__String message;
  double distance;
} xczs_inspection_robot_control__srv__SetCabinetGrasp_Response;

// Struct for a sequence of xczs_inspection_robot_control__srv__SetCabinetGrasp_Response.
typedef struct xczs_inspection_robot_control__srv__SetCabinetGrasp_Response__Sequence
{
  xczs_inspection_robot_control__srv__SetCabinetGrasp_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} xczs_inspection_robot_control__srv__SetCabinetGrasp_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // XCZS_INSPECTION_ROBOT_CONTROL__SRV__DETAIL__SET_CABINET_GRASP__STRUCT_H_
