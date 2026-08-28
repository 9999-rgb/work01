// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from xczs_inspection_robot_control:msg/CabinetControlState.idl
// generated code does not contain a copyright notice

#ifndef XCZS_INSPECTION_ROBOT_CONTROL__MSG__DETAIL__CABINET_CONTROL_STATE__STRUCT_H_
#define XCZS_INSPECTION_ROBOT_CONTROL__MSG__DETAIL__CABINET_CONTROL_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'control_id'
// Member 'state_id'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/CabinetControlState in the package xczs_inspection_robot_control.
typedef struct xczs_inspection_robot_control__msg__CabinetControlState
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__String control_id;
  uint8_t control_type;
  bool valid;
  double position;
  double velocity;
  double effort;
  double normalized_position;
  rosidl_runtime_c__String state_id;
  bool activated;
  bool in_motion;
  uint64_t transition_sequence;
} xczs_inspection_robot_control__msg__CabinetControlState;

// Struct for a sequence of xczs_inspection_robot_control__msg__CabinetControlState.
typedef struct xczs_inspection_robot_control__msg__CabinetControlState__Sequence
{
  xczs_inspection_robot_control__msg__CabinetControlState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} xczs_inspection_robot_control__msg__CabinetControlState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // XCZS_INSPECTION_ROBOT_CONTROL__MSG__DETAIL__CABINET_CONTROL_STATE__STRUCT_H_
