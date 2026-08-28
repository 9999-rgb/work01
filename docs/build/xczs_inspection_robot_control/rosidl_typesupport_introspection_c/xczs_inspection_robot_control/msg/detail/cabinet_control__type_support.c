// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from xczs_inspection_robot_control:msg/CabinetControl.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "xczs_inspection_robot_control/msg/detail/cabinet_control__rosidl_typesupport_introspection_c.h"
#include "xczs_inspection_robot_control/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "xczs_inspection_robot_control/msg/detail/cabinet_control__functions.h"
#include "xczs_inspection_robot_control/msg/detail/cabinet_control__struct.h"


// Include directives for member types
// Member `control_id`
// Member `display_name`
// Member `joint_name`
// Member `joint_state_topic`
// Member `pressed_topic`
// Member `state_topic`
// Member `unit`
// Member `state_ids`
// Member `state_labels`
// Member `unavailable_reason`
// Member `required_toolset`
#include "rosidl_runtime_c/string_functions.h"
// Member `state_positions`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void xczs_inspection_robot_control__msg__CabinetControl__rosidl_typesupport_introspection_c__CabinetControl_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  xczs_inspection_robot_control__msg__CabinetControl__init(message_memory);
}

void xczs_inspection_robot_control__msg__CabinetControl__rosidl_typesupport_introspection_c__CabinetControl_fini_function(void * message_memory)
{
  xczs_inspection_robot_control__msg__CabinetControl__fini(message_memory);
}

size_t xczs_inspection_robot_control__msg__CabinetControl__rosidl_typesupport_introspection_c__size_function__CabinetControl__state_ids(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * xczs_inspection_robot_control__msg__CabinetControl__rosidl_typesupport_introspection_c__get_const_function__CabinetControl__state_ids(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * xczs_inspection_robot_control__msg__CabinetControl__rosidl_typesupport_introspection_c__get_function__CabinetControl__state_ids(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void xczs_inspection_robot_control__msg__CabinetControl__rosidl_typesupport_introspection_c__fetch_function__CabinetControl__state_ids(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    xczs_inspection_robot_control__msg__CabinetControl__rosidl_typesupport_introspection_c__get_const_function__CabinetControl__state_ids(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void xczs_inspection_robot_control__msg__CabinetControl__rosidl_typesupport_introspection_c__assign_function__CabinetControl__state_ids(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    xczs_inspection_robot_control__msg__CabinetControl__rosidl_typesupport_introspection_c__get_function__CabinetControl__state_ids(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool xczs_inspection_robot_control__msg__CabinetControl__rosidl_typesupport_introspection_c__resize_function__CabinetControl__state_ids(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

size_t xczs_inspection_robot_control__msg__CabinetControl__rosidl_typesupport_introspection_c__size_function__CabinetControl__state_labels(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * xczs_inspection_robot_control__msg__CabinetControl__rosidl_typesupport_introspection_c__get_const_function__CabinetControl__state_labels(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * xczs_inspection_robot_control__msg__CabinetControl__rosidl_typesupport_introspection_c__get_function__CabinetControl__state_labels(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void xczs_inspection_robot_control__msg__CabinetControl__rosidl_typesupport_introspection_c__fetch_function__CabinetControl__state_labels(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    xczs_inspection_robot_control__msg__CabinetControl__rosidl_typesupport_introspection_c__get_const_function__CabinetControl__state_labels(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void xczs_inspection_robot_control__msg__CabinetControl__rosidl_typesupport_introspection_c__assign_function__CabinetControl__state_labels(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    xczs_inspection_robot_control__msg__CabinetControl__rosidl_typesupport_introspection_c__get_function__CabinetControl__state_labels(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool xczs_inspection_robot_control__msg__CabinetControl__rosidl_typesupport_introspection_c__resize_function__CabinetControl__state_labels(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

size_t xczs_inspection_robot_control__msg__CabinetControl__rosidl_typesupport_introspection_c__size_function__CabinetControl__state_positions(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * xczs_inspection_robot_control__msg__CabinetControl__rosidl_typesupport_introspection_c__get_const_function__CabinetControl__state_positions(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * xczs_inspection_robot_control__msg__CabinetControl__rosidl_typesupport_introspection_c__get_function__CabinetControl__state_positions(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void xczs_inspection_robot_control__msg__CabinetControl__rosidl_typesupport_introspection_c__fetch_function__CabinetControl__state_positions(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    xczs_inspection_robot_control__msg__CabinetControl__rosidl_typesupport_introspection_c__get_const_function__CabinetControl__state_positions(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void xczs_inspection_robot_control__msg__CabinetControl__rosidl_typesupport_introspection_c__assign_function__CabinetControl__state_positions(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    xczs_inspection_robot_control__msg__CabinetControl__rosidl_typesupport_introspection_c__get_function__CabinetControl__state_positions(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool xczs_inspection_robot_control__msg__CabinetControl__rosidl_typesupport_introspection_c__resize_function__CabinetControl__state_positions(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember xczs_inspection_robot_control__msg__CabinetControl__rosidl_typesupport_introspection_c__CabinetControl_message_member_array[23] = {
  {
    "control_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_control__msg__CabinetControl, control_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "display_name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_control__msg__CabinetControl, display_name),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "control_type",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_control__msg__CabinetControl, control_type),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "joint_name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_control__msg__CabinetControl, joint_name),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "joint_state_topic",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_control__msg__CabinetControl, joint_state_topic),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "pressed_topic",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_control__msg__CabinetControl, pressed_topic),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "state_topic",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_control__msg__CabinetControl, state_topic),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "supported_commands",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_control__msg__CabinetControl, supported_commands),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "unit",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_control__msg__CabinetControl, unit),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "min_position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_control__msg__CabinetControl, min_position),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "max_position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_control__msg__CabinetControl, max_position),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "state_ids",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_control__msg__CabinetControl, state_ids),  // bytes offset in struct
    NULL,  // default value
    xczs_inspection_robot_control__msg__CabinetControl__rosidl_typesupport_introspection_c__size_function__CabinetControl__state_ids,  // size() function pointer
    xczs_inspection_robot_control__msg__CabinetControl__rosidl_typesupport_introspection_c__get_const_function__CabinetControl__state_ids,  // get_const(index) function pointer
    xczs_inspection_robot_control__msg__CabinetControl__rosidl_typesupport_introspection_c__get_function__CabinetControl__state_ids,  // get(index) function pointer
    xczs_inspection_robot_control__msg__CabinetControl__rosidl_typesupport_introspection_c__fetch_function__CabinetControl__state_ids,  // fetch(index, &value) function pointer
    xczs_inspection_robot_control__msg__CabinetControl__rosidl_typesupport_introspection_c__assign_function__CabinetControl__state_ids,  // assign(index, value) function pointer
    xczs_inspection_robot_control__msg__CabinetControl__rosidl_typesupport_introspection_c__resize_function__CabinetControl__state_ids  // resize(index) function pointer
  },
  {
    "state_labels",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_control__msg__CabinetControl, state_labels),  // bytes offset in struct
    NULL,  // default value
    xczs_inspection_robot_control__msg__CabinetControl__rosidl_typesupport_introspection_c__size_function__CabinetControl__state_labels,  // size() function pointer
    xczs_inspection_robot_control__msg__CabinetControl__rosidl_typesupport_introspection_c__get_const_function__CabinetControl__state_labels,  // get_const(index) function pointer
    xczs_inspection_robot_control__msg__CabinetControl__rosidl_typesupport_introspection_c__get_function__CabinetControl__state_labels,  // get(index) function pointer
    xczs_inspection_robot_control__msg__CabinetControl__rosidl_typesupport_introspection_c__fetch_function__CabinetControl__state_labels,  // fetch(index, &value) function pointer
    xczs_inspection_robot_control__msg__CabinetControl__rosidl_typesupport_introspection_c__assign_function__CabinetControl__state_labels,  // assign(index, value) function pointer
    xczs_inspection_robot_control__msg__CabinetControl__rosidl_typesupport_introspection_c__resize_function__CabinetControl__state_labels  // resize(index) function pointer
  },
  {
    "state_positions",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_control__msg__CabinetControl, state_positions),  // bytes offset in struct
    NULL,  // default value
    xczs_inspection_robot_control__msg__CabinetControl__rosidl_typesupport_introspection_c__size_function__CabinetControl__state_positions,  // size() function pointer
    xczs_inspection_robot_control__msg__CabinetControl__rosidl_typesupport_introspection_c__get_const_function__CabinetControl__state_positions,  // get_const(index) function pointer
    xczs_inspection_robot_control__msg__CabinetControl__rosidl_typesupport_introspection_c__get_function__CabinetControl__state_positions,  // get(index) function pointer
    xczs_inspection_robot_control__msg__CabinetControl__rosidl_typesupport_introspection_c__fetch_function__CabinetControl__state_positions,  // fetch(index, &value) function pointer
    xczs_inspection_robot_control__msg__CabinetControl__rosidl_typesupport_introspection_c__assign_function__CabinetControl__state_positions,  // assign(index, value) function pointer
    xczs_inspection_robot_control__msg__CabinetControl__rosidl_typesupport_introspection_c__resize_function__CabinetControl__state_positions  // resize(index) function pointer
  },
  {
    "requires_grasp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_control__msg__CabinetControl, requires_grasp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "operable",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_control__msg__CabinetControl, operable),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "unavailable_reason",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_control__msg__CabinetControl, unavailable_reason),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "required_toolset",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_control__msg__CabinetControl, required_toolset),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "toolset_compatible",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_control__msg__CabinetControl, toolset_compatible),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "adapter_validated",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_control__msg__CabinetControl, adapter_validated),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "default_force",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_control__msg__CabinetControl, default_force),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "min_trigger_force",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_control__msg__CabinetControl, min_trigger_force),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "max_force",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_control__msg__CabinetControl, max_force),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers xczs_inspection_robot_control__msg__CabinetControl__rosidl_typesupport_introspection_c__CabinetControl_message_members = {
  "xczs_inspection_robot_control__msg",  // message namespace
  "CabinetControl",  // message name
  23,  // number of fields
  sizeof(xczs_inspection_robot_control__msg__CabinetControl),
  xczs_inspection_robot_control__msg__CabinetControl__rosidl_typesupport_introspection_c__CabinetControl_message_member_array,  // message members
  xczs_inspection_robot_control__msg__CabinetControl__rosidl_typesupport_introspection_c__CabinetControl_init_function,  // function to initialize message memory (memory has to be allocated)
  xczs_inspection_robot_control__msg__CabinetControl__rosidl_typesupport_introspection_c__CabinetControl_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t xczs_inspection_robot_control__msg__CabinetControl__rosidl_typesupport_introspection_c__CabinetControl_message_type_support_handle = {
  0,
  &xczs_inspection_robot_control__msg__CabinetControl__rosidl_typesupport_introspection_c__CabinetControl_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_xczs_inspection_robot_control
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, xczs_inspection_robot_control, msg, CabinetControl)() {
  if (!xczs_inspection_robot_control__msg__CabinetControl__rosidl_typesupport_introspection_c__CabinetControl_message_type_support_handle.typesupport_identifier) {
    xczs_inspection_robot_control__msg__CabinetControl__rosidl_typesupport_introspection_c__CabinetControl_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &xczs_inspection_robot_control__msg__CabinetControl__rosidl_typesupport_introspection_c__CabinetControl_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
