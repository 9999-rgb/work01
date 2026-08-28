// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from xczs_inspection_robot_control:msg/CabinetControlCatalog.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "xczs_inspection_robot_control/msg/detail/cabinet_control_catalog__rosidl_typesupport_introspection_c.h"
#include "xczs_inspection_robot_control/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "xczs_inspection_robot_control/msg/detail/cabinet_control_catalog__functions.h"
#include "xczs_inspection_robot_control/msg/detail/cabinet_control_catalog__struct.h"


// Include directives for member types
// Member `controls`
#include "xczs_inspection_robot_control/msg/cabinet_control.h"
// Member `controls`
#include "xczs_inspection_robot_control/msg/detail/cabinet_control__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void xczs_inspection_robot_control__msg__CabinetControlCatalog__rosidl_typesupport_introspection_c__CabinetControlCatalog_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  xczs_inspection_robot_control__msg__CabinetControlCatalog__init(message_memory);
}

void xczs_inspection_robot_control__msg__CabinetControlCatalog__rosidl_typesupport_introspection_c__CabinetControlCatalog_fini_function(void * message_memory)
{
  xczs_inspection_robot_control__msg__CabinetControlCatalog__fini(message_memory);
}

size_t xczs_inspection_robot_control__msg__CabinetControlCatalog__rosidl_typesupport_introspection_c__size_function__CabinetControlCatalog__controls(
  const void * untyped_member)
{
  const xczs_inspection_robot_control__msg__CabinetControl__Sequence * member =
    (const xczs_inspection_robot_control__msg__CabinetControl__Sequence *)(untyped_member);
  return member->size;
}

const void * xczs_inspection_robot_control__msg__CabinetControlCatalog__rosidl_typesupport_introspection_c__get_const_function__CabinetControlCatalog__controls(
  const void * untyped_member, size_t index)
{
  const xczs_inspection_robot_control__msg__CabinetControl__Sequence * member =
    (const xczs_inspection_robot_control__msg__CabinetControl__Sequence *)(untyped_member);
  return &member->data[index];
}

void * xczs_inspection_robot_control__msg__CabinetControlCatalog__rosidl_typesupport_introspection_c__get_function__CabinetControlCatalog__controls(
  void * untyped_member, size_t index)
{
  xczs_inspection_robot_control__msg__CabinetControl__Sequence * member =
    (xczs_inspection_robot_control__msg__CabinetControl__Sequence *)(untyped_member);
  return &member->data[index];
}

void xczs_inspection_robot_control__msg__CabinetControlCatalog__rosidl_typesupport_introspection_c__fetch_function__CabinetControlCatalog__controls(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const xczs_inspection_robot_control__msg__CabinetControl * item =
    ((const xczs_inspection_robot_control__msg__CabinetControl *)
    xczs_inspection_robot_control__msg__CabinetControlCatalog__rosidl_typesupport_introspection_c__get_const_function__CabinetControlCatalog__controls(untyped_member, index));
  xczs_inspection_robot_control__msg__CabinetControl * value =
    (xczs_inspection_robot_control__msg__CabinetControl *)(untyped_value);
  *value = *item;
}

void xczs_inspection_robot_control__msg__CabinetControlCatalog__rosidl_typesupport_introspection_c__assign_function__CabinetControlCatalog__controls(
  void * untyped_member, size_t index, const void * untyped_value)
{
  xczs_inspection_robot_control__msg__CabinetControl * item =
    ((xczs_inspection_robot_control__msg__CabinetControl *)
    xczs_inspection_robot_control__msg__CabinetControlCatalog__rosidl_typesupport_introspection_c__get_function__CabinetControlCatalog__controls(untyped_member, index));
  const xczs_inspection_robot_control__msg__CabinetControl * value =
    (const xczs_inspection_robot_control__msg__CabinetControl *)(untyped_value);
  *item = *value;
}

bool xczs_inspection_robot_control__msg__CabinetControlCatalog__rosidl_typesupport_introspection_c__resize_function__CabinetControlCatalog__controls(
  void * untyped_member, size_t size)
{
  xczs_inspection_robot_control__msg__CabinetControl__Sequence * member =
    (xczs_inspection_robot_control__msg__CabinetControl__Sequence *)(untyped_member);
  xczs_inspection_robot_control__msg__CabinetControl__Sequence__fini(member);
  return xczs_inspection_robot_control__msg__CabinetControl__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember xczs_inspection_robot_control__msg__CabinetControlCatalog__rosidl_typesupport_introspection_c__CabinetControlCatalog_message_member_array[1] = {
  {
    "controls",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_control__msg__CabinetControlCatalog, controls),  // bytes offset in struct
    NULL,  // default value
    xczs_inspection_robot_control__msg__CabinetControlCatalog__rosidl_typesupport_introspection_c__size_function__CabinetControlCatalog__controls,  // size() function pointer
    xczs_inspection_robot_control__msg__CabinetControlCatalog__rosidl_typesupport_introspection_c__get_const_function__CabinetControlCatalog__controls,  // get_const(index) function pointer
    xczs_inspection_robot_control__msg__CabinetControlCatalog__rosidl_typesupport_introspection_c__get_function__CabinetControlCatalog__controls,  // get(index) function pointer
    xczs_inspection_robot_control__msg__CabinetControlCatalog__rosidl_typesupport_introspection_c__fetch_function__CabinetControlCatalog__controls,  // fetch(index, &value) function pointer
    xczs_inspection_robot_control__msg__CabinetControlCatalog__rosidl_typesupport_introspection_c__assign_function__CabinetControlCatalog__controls,  // assign(index, value) function pointer
    xczs_inspection_robot_control__msg__CabinetControlCatalog__rosidl_typesupport_introspection_c__resize_function__CabinetControlCatalog__controls  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers xczs_inspection_robot_control__msg__CabinetControlCatalog__rosidl_typesupport_introspection_c__CabinetControlCatalog_message_members = {
  "xczs_inspection_robot_control__msg",  // message namespace
  "CabinetControlCatalog",  // message name
  1,  // number of fields
  sizeof(xczs_inspection_robot_control__msg__CabinetControlCatalog),
  xczs_inspection_robot_control__msg__CabinetControlCatalog__rosidl_typesupport_introspection_c__CabinetControlCatalog_message_member_array,  // message members
  xczs_inspection_robot_control__msg__CabinetControlCatalog__rosidl_typesupport_introspection_c__CabinetControlCatalog_init_function,  // function to initialize message memory (memory has to be allocated)
  xczs_inspection_robot_control__msg__CabinetControlCatalog__rosidl_typesupport_introspection_c__CabinetControlCatalog_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t xczs_inspection_robot_control__msg__CabinetControlCatalog__rosidl_typesupport_introspection_c__CabinetControlCatalog_message_type_support_handle = {
  0,
  &xczs_inspection_robot_control__msg__CabinetControlCatalog__rosidl_typesupport_introspection_c__CabinetControlCatalog_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_xczs_inspection_robot_control
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, xczs_inspection_robot_control, msg, CabinetControlCatalog)() {
  xczs_inspection_robot_control__msg__CabinetControlCatalog__rosidl_typesupport_introspection_c__CabinetControlCatalog_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, xczs_inspection_robot_control, msg, CabinetControl)();
  if (!xczs_inspection_robot_control__msg__CabinetControlCatalog__rosidl_typesupport_introspection_c__CabinetControlCatalog_message_type_support_handle.typesupport_identifier) {
    xczs_inspection_robot_control__msg__CabinetControlCatalog__rosidl_typesupport_introspection_c__CabinetControlCatalog_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &xczs_inspection_robot_control__msg__CabinetControlCatalog__rosidl_typesupport_introspection_c__CabinetControlCatalog_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
