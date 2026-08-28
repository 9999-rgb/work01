// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from xczs_inspection_robot_interfaces:msg/CabinetControl.idl
// generated code does not contain a copyright notice

#ifndef XCZS_INSPECTION_ROBOT_INTERFACES__MSG__DETAIL__CABINET_CONTROL__STRUCT_H_
#define XCZS_INSPECTION_ROBOT_INTERFACES__MSG__DETAIL__CABINET_CONTROL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'TYPE_BUTTON'.
enum
{
  xczs_inspection_robot_interfaces__msg__CabinetControl__TYPE_BUTTON = 0
};

/// Constant 'TYPE_KNOB'.
enum
{
  xczs_inspection_robot_interfaces__msg__CabinetControl__TYPE_KNOB = 1
};

/// Constant 'TYPE_SWITCH'.
enum
{
  xczs_inspection_robot_interfaces__msg__CabinetControl__TYPE_SWITCH = 2
};

/// Constant 'TYPE_DOOR'.
enum
{
  xczs_inspection_robot_interfaces__msg__CabinetControl__TYPE_DOOR = 3
};

/// Constant 'SUPPORT_PRESS'.
enum
{
  xczs_inspection_robot_interfaces__msg__CabinetControl__SUPPORT_PRESS = 1
};

/// Constant 'SUPPORT_SET_STATE'.
enum
{
  xczs_inspection_robot_interfaces__msg__CabinetControl__SUPPORT_SET_STATE = 2
};

/// Constant 'SUPPORT_SET_POSITION'.
enum
{
  xczs_inspection_robot_interfaces__msg__CabinetControl__SUPPORT_SET_POSITION = 4
};

/// Constant 'SUPPORT_TOGGLE'.
enum
{
  xczs_inspection_robot_interfaces__msg__CabinetControl__SUPPORT_TOGGLE = 8
};

// Include directives for member types
// Member 'control_id'
// Member 'display_name'
// Member 'joint_name'
// Member 'joint_state_topic'
// Member 'pressed_topic'
// Member 'state_topic'
// Member 'unit'
// Member 'state_ids'
// Member 'state_labels'
// Member 'unavailable_reason'
// Member 'required_toolset'
#include "rosidl_runtime_c/string.h"
// Member 'state_positions'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/CabinetControl in the package xczs_inspection_robot_interfaces.
typedef struct xczs_inspection_robot_interfaces__msg__CabinetControl
{
  rosidl_runtime_c__String control_id;
  rosidl_runtime_c__String display_name;
  uint8_t control_type;
  rosidl_runtime_c__String joint_name;
  rosidl_runtime_c__String joint_state_topic;
  rosidl_runtime_c__String pressed_topic;
  rosidl_runtime_c__String state_topic;
  uint8_t supported_commands;
  rosidl_runtime_c__String unit;
  double min_position;
  double max_position;
  rosidl_runtime_c__String__Sequence state_ids;
  rosidl_runtime_c__String__Sequence state_labels;
  rosidl_runtime_c__double__Sequence state_positions;
  bool requires_grasp;
  bool operable;
  rosidl_runtime_c__String unavailable_reason;
  /// The end-effector set required by this control (A or B).  This is exposed
  /// separately from ``operable`` so clients can provide an actionable error
  /// before attempting navigation when the other set is mounted.
  rosidl_runtime_c__String required_toolset;
  bool toolset_compatible;
  /// True when this control is in the robot adapter's physically validated
  /// allowlist.  A false value means planning-only validation is allowed, but no
  /// physical cabinet command may be sent.
  bool adapter_validated;
  double default_force;
  double min_trigger_force;
  double max_force;
} xczs_inspection_robot_interfaces__msg__CabinetControl;

// Struct for a sequence of xczs_inspection_robot_interfaces__msg__CabinetControl.
typedef struct xczs_inspection_robot_interfaces__msg__CabinetControl__Sequence
{
  xczs_inspection_robot_interfaces__msg__CabinetControl * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} xczs_inspection_robot_interfaces__msg__CabinetControl__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // XCZS_INSPECTION_ROBOT_INTERFACES__MSG__DETAIL__CABINET_CONTROL__STRUCT_H_
