// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from xczs_inspection_robot_interfaces:action/OperateCabinetControl.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "xczs_inspection_robot_interfaces/action/detail/operate_cabinet_control__rosidl_typesupport_introspection_c.h"
#include "xczs_inspection_robot_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "xczs_inspection_robot_interfaces/action/detail/operate_cabinet_control__functions.h"
#include "xczs_inspection_robot_interfaces/action/detail/operate_cabinet_control__struct.h"


// Include directives for member types
// Member `control_id`
// Member `target_state`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void xczs_inspection_robot_interfaces__action__OperateCabinetControl_Goal__rosidl_typesupport_introspection_c__OperateCabinetControl_Goal_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_Goal__init(message_memory);
}

void xczs_inspection_robot_interfaces__action__OperateCabinetControl_Goal__rosidl_typesupport_introspection_c__OperateCabinetControl_Goal_fini_function(void * message_memory)
{
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_Goal__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember xczs_inspection_robot_interfaces__action__OperateCabinetControl_Goal__rosidl_typesupport_introspection_c__OperateCabinetControl_Goal_message_member_array[7] = {
  {
    "control_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_interfaces__action__OperateCabinetControl_Goal, control_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "command",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_interfaces__action__OperateCabinetControl_Goal, command),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "target_state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_interfaces__action__OperateCabinetControl_Goal, target_state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "target_position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_interfaces__action__OperateCabinetControl_Goal, target_position),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "use_target_position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_interfaces__action__OperateCabinetControl_Goal, use_target_position),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "force",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_interfaces__action__OperateCabinetControl_Goal, force),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "navigate_to_staging_pose",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_interfaces__action__OperateCabinetControl_Goal, navigate_to_staging_pose),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers xczs_inspection_robot_interfaces__action__OperateCabinetControl_Goal__rosidl_typesupport_introspection_c__OperateCabinetControl_Goal_message_members = {
  "xczs_inspection_robot_interfaces__action",  // message namespace
  "OperateCabinetControl_Goal",  // message name
  7,  // number of fields
  sizeof(xczs_inspection_robot_interfaces__action__OperateCabinetControl_Goal),
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_Goal__rosidl_typesupport_introspection_c__OperateCabinetControl_Goal_message_member_array,  // message members
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_Goal__rosidl_typesupport_introspection_c__OperateCabinetControl_Goal_init_function,  // function to initialize message memory (memory has to be allocated)
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_Goal__rosidl_typesupport_introspection_c__OperateCabinetControl_Goal_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t xczs_inspection_robot_interfaces__action__OperateCabinetControl_Goal__rosidl_typesupport_introspection_c__OperateCabinetControl_Goal_message_type_support_handle = {
  0,
  &xczs_inspection_robot_interfaces__action__OperateCabinetControl_Goal__rosidl_typesupport_introspection_c__OperateCabinetControl_Goal_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_xczs_inspection_robot_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, xczs_inspection_robot_interfaces, action, OperateCabinetControl_Goal)() {
  if (!xczs_inspection_robot_interfaces__action__OperateCabinetControl_Goal__rosidl_typesupport_introspection_c__OperateCabinetControl_Goal_message_type_support_handle.typesupport_identifier) {
    xczs_inspection_robot_interfaces__action__OperateCabinetControl_Goal__rosidl_typesupport_introspection_c__OperateCabinetControl_Goal_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &xczs_inspection_robot_interfaces__action__OperateCabinetControl_Goal__rosidl_typesupport_introspection_c__OperateCabinetControl_Goal_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "xczs_inspection_robot_interfaces/action/detail/operate_cabinet_control__rosidl_typesupport_introspection_c.h"
// already included above
// #include "xczs_inspection_robot_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "xczs_inspection_robot_interfaces/action/detail/operate_cabinet_control__functions.h"
// already included above
// #include "xczs_inspection_robot_interfaces/action/detail/operate_cabinet_control__struct.h"


// Include directives for member types
// Member `message`
// Member `final_state`
// Member `diagnostic_stage`
// Member `policy_reason`
// Member `failure_reason`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result__rosidl_typesupport_introspection_c__OperateCabinetControl_Result_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result__init(message_memory);
}

void xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result__rosidl_typesupport_introspection_c__OperateCabinetControl_Result_fini_function(void * message_memory)
{
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result__rosidl_typesupport_introspection_c__OperateCabinetControl_Result_message_member_array[23] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "error_code",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result, error_code),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "message",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result, message),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "initial_position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result, initial_position),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "final_position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result, final_position),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "peak_position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result, peak_position),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "final_state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result, final_state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "requested_force",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result, requested_force),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "estimated_force",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result, estimated_force),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "button_triggered",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result, button_triggered),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "validation_performed",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result, validation_performed),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "operation_executed",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result, operation_executed),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "diagnostic_stage",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result, diagnostic_stage),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "path_fraction",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result, path_fraction),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "required_fraction",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result, required_fraction),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "moveit_error_code",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result, moveit_error_code),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "policy_reason",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result, policy_reason),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "failure_reason",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result, failure_reason),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "physical_outcome_confirmed",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result, physical_outcome_confirmed),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "final_state_verified",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result, final_state_verified),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "transport_succeeded",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result, transport_succeeded),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "recovery_succeeded",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result, recovery_succeeded),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "grasp_released",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result, grasp_released),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result__rosidl_typesupport_introspection_c__OperateCabinetControl_Result_message_members = {
  "xczs_inspection_robot_interfaces__action",  // message namespace
  "OperateCabinetControl_Result",  // message name
  23,  // number of fields
  sizeof(xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result),
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result__rosidl_typesupport_introspection_c__OperateCabinetControl_Result_message_member_array,  // message members
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result__rosidl_typesupport_introspection_c__OperateCabinetControl_Result_init_function,  // function to initialize message memory (memory has to be allocated)
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result__rosidl_typesupport_introspection_c__OperateCabinetControl_Result_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result__rosidl_typesupport_introspection_c__OperateCabinetControl_Result_message_type_support_handle = {
  0,
  &xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result__rosidl_typesupport_introspection_c__OperateCabinetControl_Result_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_xczs_inspection_robot_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, xczs_inspection_robot_interfaces, action, OperateCabinetControl_Result)() {
  if (!xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result__rosidl_typesupport_introspection_c__OperateCabinetControl_Result_message_type_support_handle.typesupport_identifier) {
    xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result__rosidl_typesupport_introspection_c__OperateCabinetControl_Result_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result__rosidl_typesupport_introspection_c__OperateCabinetControl_Result_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "xczs_inspection_robot_interfaces/action/detail/operate_cabinet_control__rosidl_typesupport_introspection_c.h"
// already included above
// #include "xczs_inspection_robot_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "xczs_inspection_robot_interfaces/action/detail/operate_cabinet_control__functions.h"
// already included above
// #include "xczs_inspection_robot_interfaces/action/detail/operate_cabinet_control__struct.h"


// Include directives for member types
// Member `current_state`
// Member `message`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void xczs_inspection_robot_interfaces__action__OperateCabinetControl_Feedback__rosidl_typesupport_introspection_c__OperateCabinetControl_Feedback_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_Feedback__init(message_memory);
}

void xczs_inspection_robot_interfaces__action__OperateCabinetControl_Feedback__rosidl_typesupport_introspection_c__OperateCabinetControl_Feedback_fini_function(void * message_memory)
{
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_Feedback__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember xczs_inspection_robot_interfaces__action__OperateCabinetControl_Feedback__rosidl_typesupport_introspection_c__OperateCabinetControl_Feedback_message_member_array[6] = {
  {
    "phase",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_interfaces__action__OperateCabinetControl_Feedback, phase),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "progress",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_interfaces__action__OperateCabinetControl_Feedback, progress),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "current_position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_interfaces__action__OperateCabinetControl_Feedback, current_position),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "target_position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_interfaces__action__OperateCabinetControl_Feedback, target_position),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "current_state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_interfaces__action__OperateCabinetControl_Feedback, current_state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "message",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_interfaces__action__OperateCabinetControl_Feedback, message),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers xczs_inspection_robot_interfaces__action__OperateCabinetControl_Feedback__rosidl_typesupport_introspection_c__OperateCabinetControl_Feedback_message_members = {
  "xczs_inspection_robot_interfaces__action",  // message namespace
  "OperateCabinetControl_Feedback",  // message name
  6,  // number of fields
  sizeof(xczs_inspection_robot_interfaces__action__OperateCabinetControl_Feedback),
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_Feedback__rosidl_typesupport_introspection_c__OperateCabinetControl_Feedback_message_member_array,  // message members
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_Feedback__rosidl_typesupport_introspection_c__OperateCabinetControl_Feedback_init_function,  // function to initialize message memory (memory has to be allocated)
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_Feedback__rosidl_typesupport_introspection_c__OperateCabinetControl_Feedback_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t xczs_inspection_robot_interfaces__action__OperateCabinetControl_Feedback__rosidl_typesupport_introspection_c__OperateCabinetControl_Feedback_message_type_support_handle = {
  0,
  &xczs_inspection_robot_interfaces__action__OperateCabinetControl_Feedback__rosidl_typesupport_introspection_c__OperateCabinetControl_Feedback_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_xczs_inspection_robot_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, xczs_inspection_robot_interfaces, action, OperateCabinetControl_Feedback)() {
  if (!xczs_inspection_robot_interfaces__action__OperateCabinetControl_Feedback__rosidl_typesupport_introspection_c__OperateCabinetControl_Feedback_message_type_support_handle.typesupport_identifier) {
    xczs_inspection_robot_interfaces__action__OperateCabinetControl_Feedback__rosidl_typesupport_introspection_c__OperateCabinetControl_Feedback_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &xczs_inspection_robot_interfaces__action__OperateCabinetControl_Feedback__rosidl_typesupport_introspection_c__OperateCabinetControl_Feedback_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "xczs_inspection_robot_interfaces/action/detail/operate_cabinet_control__rosidl_typesupport_introspection_c.h"
// already included above
// #include "xczs_inspection_robot_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "xczs_inspection_robot_interfaces/action/detail/operate_cabinet_control__functions.h"
// already included above
// #include "xczs_inspection_robot_interfaces/action/detail/operate_cabinet_control__struct.h"


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"
// Member `goal`
#include "xczs_inspection_robot_interfaces/action/operate_cabinet_control.h"
// Member `goal`
// already included above
// #include "xczs_inspection_robot_interfaces/action/detail/operate_cabinet_control__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Request__rosidl_typesupport_introspection_c__OperateCabinetControl_SendGoal_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Request__init(message_memory);
}

void xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Request__rosidl_typesupport_introspection_c__OperateCabinetControl_SendGoal_Request_fini_function(void * message_memory)
{
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Request__rosidl_typesupport_introspection_c__OperateCabinetControl_SendGoal_Request_message_member_array[2] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Request, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "goal",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Request, goal),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Request__rosidl_typesupport_introspection_c__OperateCabinetControl_SendGoal_Request_message_members = {
  "xczs_inspection_robot_interfaces__action",  // message namespace
  "OperateCabinetControl_SendGoal_Request",  // message name
  2,  // number of fields
  sizeof(xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Request),
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Request__rosidl_typesupport_introspection_c__OperateCabinetControl_SendGoal_Request_message_member_array,  // message members
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Request__rosidl_typesupport_introspection_c__OperateCabinetControl_SendGoal_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Request__rosidl_typesupport_introspection_c__OperateCabinetControl_SendGoal_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Request__rosidl_typesupport_introspection_c__OperateCabinetControl_SendGoal_Request_message_type_support_handle = {
  0,
  &xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Request__rosidl_typesupport_introspection_c__OperateCabinetControl_SendGoal_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_xczs_inspection_robot_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, xczs_inspection_robot_interfaces, action, OperateCabinetControl_SendGoal_Request)() {
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Request__rosidl_typesupport_introspection_c__OperateCabinetControl_SendGoal_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Request__rosidl_typesupport_introspection_c__OperateCabinetControl_SendGoal_Request_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, xczs_inspection_robot_interfaces, action, OperateCabinetControl_Goal)();
  if (!xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Request__rosidl_typesupport_introspection_c__OperateCabinetControl_SendGoal_Request_message_type_support_handle.typesupport_identifier) {
    xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Request__rosidl_typesupport_introspection_c__OperateCabinetControl_SendGoal_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Request__rosidl_typesupport_introspection_c__OperateCabinetControl_SendGoal_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "xczs_inspection_robot_interfaces/action/detail/operate_cabinet_control__rosidl_typesupport_introspection_c.h"
// already included above
// #include "xczs_inspection_robot_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "xczs_inspection_robot_interfaces/action/detail/operate_cabinet_control__functions.h"
// already included above
// #include "xczs_inspection_robot_interfaces/action/detail/operate_cabinet_control__struct.h"


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/time.h"
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Response__rosidl_typesupport_introspection_c__OperateCabinetControl_SendGoal_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Response__init(message_memory);
}

void xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Response__rosidl_typesupport_introspection_c__OperateCabinetControl_SendGoal_Response_fini_function(void * message_memory)
{
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Response__rosidl_typesupport_introspection_c__OperateCabinetControl_SendGoal_Response_message_member_array[2] = {
  {
    "accepted",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Response, accepted),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "stamp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Response, stamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Response__rosidl_typesupport_introspection_c__OperateCabinetControl_SendGoal_Response_message_members = {
  "xczs_inspection_robot_interfaces__action",  // message namespace
  "OperateCabinetControl_SendGoal_Response",  // message name
  2,  // number of fields
  sizeof(xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Response),
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Response__rosidl_typesupport_introspection_c__OperateCabinetControl_SendGoal_Response_message_member_array,  // message members
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Response__rosidl_typesupport_introspection_c__OperateCabinetControl_SendGoal_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Response__rosidl_typesupport_introspection_c__OperateCabinetControl_SendGoal_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Response__rosidl_typesupport_introspection_c__OperateCabinetControl_SendGoal_Response_message_type_support_handle = {
  0,
  &xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Response__rosidl_typesupport_introspection_c__OperateCabinetControl_SendGoal_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_xczs_inspection_robot_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, xczs_inspection_robot_interfaces, action, OperateCabinetControl_SendGoal_Response)() {
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Response__rosidl_typesupport_introspection_c__OperateCabinetControl_SendGoal_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  if (!xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Response__rosidl_typesupport_introspection_c__OperateCabinetControl_SendGoal_Response_message_type_support_handle.typesupport_identifier) {
    xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Response__rosidl_typesupport_introspection_c__OperateCabinetControl_SendGoal_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Response__rosidl_typesupport_introspection_c__OperateCabinetControl_SendGoal_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "xczs_inspection_robot_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "xczs_inspection_robot_interfaces/action/detail/operate_cabinet_control__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers xczs_inspection_robot_interfaces__action__detail__operate_cabinet_control__rosidl_typesupport_introspection_c__OperateCabinetControl_SendGoal_service_members = {
  "xczs_inspection_robot_interfaces__action",  // service namespace
  "OperateCabinetControl_SendGoal",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // xczs_inspection_robot_interfaces__action__detail__operate_cabinet_control__rosidl_typesupport_introspection_c__OperateCabinetControl_SendGoal_Request_message_type_support_handle,
  NULL  // response message
  // xczs_inspection_robot_interfaces__action__detail__operate_cabinet_control__rosidl_typesupport_introspection_c__OperateCabinetControl_SendGoal_Response_message_type_support_handle
};

static rosidl_service_type_support_t xczs_inspection_robot_interfaces__action__detail__operate_cabinet_control__rosidl_typesupport_introspection_c__OperateCabinetControl_SendGoal_service_type_support_handle = {
  0,
  &xczs_inspection_robot_interfaces__action__detail__operate_cabinet_control__rosidl_typesupport_introspection_c__OperateCabinetControl_SendGoal_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, xczs_inspection_robot_interfaces, action, OperateCabinetControl_SendGoal_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, xczs_inspection_robot_interfaces, action, OperateCabinetControl_SendGoal_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_xczs_inspection_robot_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, xczs_inspection_robot_interfaces, action, OperateCabinetControl_SendGoal)() {
  if (!xczs_inspection_robot_interfaces__action__detail__operate_cabinet_control__rosidl_typesupport_introspection_c__OperateCabinetControl_SendGoal_service_type_support_handle.typesupport_identifier) {
    xczs_inspection_robot_interfaces__action__detail__operate_cabinet_control__rosidl_typesupport_introspection_c__OperateCabinetControl_SendGoal_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)xczs_inspection_robot_interfaces__action__detail__operate_cabinet_control__rosidl_typesupport_introspection_c__OperateCabinetControl_SendGoal_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, xczs_inspection_robot_interfaces, action, OperateCabinetControl_SendGoal_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, xczs_inspection_robot_interfaces, action, OperateCabinetControl_SendGoal_Response)()->data;
  }

  return &xczs_inspection_robot_interfaces__action__detail__operate_cabinet_control__rosidl_typesupport_introspection_c__OperateCabinetControl_SendGoal_service_type_support_handle;
}

// already included above
// #include <stddef.h>
// already included above
// #include "xczs_inspection_robot_interfaces/action/detail/operate_cabinet_control__rosidl_typesupport_introspection_c.h"
// already included above
// #include "xczs_inspection_robot_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "xczs_inspection_robot_interfaces/action/detail/operate_cabinet_control__functions.h"
// already included above
// #include "xczs_inspection_robot_interfaces/action/detail/operate_cabinet_control__struct.h"


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Request__rosidl_typesupport_introspection_c__OperateCabinetControl_GetResult_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Request__init(message_memory);
}

void xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Request__rosidl_typesupport_introspection_c__OperateCabinetControl_GetResult_Request_fini_function(void * message_memory)
{
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Request__rosidl_typesupport_introspection_c__OperateCabinetControl_GetResult_Request_message_member_array[1] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Request, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Request__rosidl_typesupport_introspection_c__OperateCabinetControl_GetResult_Request_message_members = {
  "xczs_inspection_robot_interfaces__action",  // message namespace
  "OperateCabinetControl_GetResult_Request",  // message name
  1,  // number of fields
  sizeof(xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Request),
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Request__rosidl_typesupport_introspection_c__OperateCabinetControl_GetResult_Request_message_member_array,  // message members
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Request__rosidl_typesupport_introspection_c__OperateCabinetControl_GetResult_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Request__rosidl_typesupport_introspection_c__OperateCabinetControl_GetResult_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Request__rosidl_typesupport_introspection_c__OperateCabinetControl_GetResult_Request_message_type_support_handle = {
  0,
  &xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Request__rosidl_typesupport_introspection_c__OperateCabinetControl_GetResult_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_xczs_inspection_robot_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, xczs_inspection_robot_interfaces, action, OperateCabinetControl_GetResult_Request)() {
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Request__rosidl_typesupport_introspection_c__OperateCabinetControl_GetResult_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  if (!xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Request__rosidl_typesupport_introspection_c__OperateCabinetControl_GetResult_Request_message_type_support_handle.typesupport_identifier) {
    xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Request__rosidl_typesupport_introspection_c__OperateCabinetControl_GetResult_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Request__rosidl_typesupport_introspection_c__OperateCabinetControl_GetResult_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "xczs_inspection_robot_interfaces/action/detail/operate_cabinet_control__rosidl_typesupport_introspection_c.h"
// already included above
// #include "xczs_inspection_robot_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "xczs_inspection_robot_interfaces/action/detail/operate_cabinet_control__functions.h"
// already included above
// #include "xczs_inspection_robot_interfaces/action/detail/operate_cabinet_control__struct.h"


// Include directives for member types
// Member `result`
// already included above
// #include "xczs_inspection_robot_interfaces/action/operate_cabinet_control.h"
// Member `result`
// already included above
// #include "xczs_inspection_robot_interfaces/action/detail/operate_cabinet_control__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Response__rosidl_typesupport_introspection_c__OperateCabinetControl_GetResult_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Response__init(message_memory);
}

void xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Response__rosidl_typesupport_introspection_c__OperateCabinetControl_GetResult_Response_fini_function(void * message_memory)
{
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Response__rosidl_typesupport_introspection_c__OperateCabinetControl_GetResult_Response_message_member_array[2] = {
  {
    "status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Response, status),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "result",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Response, result),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Response__rosidl_typesupport_introspection_c__OperateCabinetControl_GetResult_Response_message_members = {
  "xczs_inspection_robot_interfaces__action",  // message namespace
  "OperateCabinetControl_GetResult_Response",  // message name
  2,  // number of fields
  sizeof(xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Response),
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Response__rosidl_typesupport_introspection_c__OperateCabinetControl_GetResult_Response_message_member_array,  // message members
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Response__rosidl_typesupport_introspection_c__OperateCabinetControl_GetResult_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Response__rosidl_typesupport_introspection_c__OperateCabinetControl_GetResult_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Response__rosidl_typesupport_introspection_c__OperateCabinetControl_GetResult_Response_message_type_support_handle = {
  0,
  &xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Response__rosidl_typesupport_introspection_c__OperateCabinetControl_GetResult_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_xczs_inspection_robot_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, xczs_inspection_robot_interfaces, action, OperateCabinetControl_GetResult_Response)() {
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Response__rosidl_typesupport_introspection_c__OperateCabinetControl_GetResult_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, xczs_inspection_robot_interfaces, action, OperateCabinetControl_Result)();
  if (!xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Response__rosidl_typesupport_introspection_c__OperateCabinetControl_GetResult_Response_message_type_support_handle.typesupport_identifier) {
    xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Response__rosidl_typesupport_introspection_c__OperateCabinetControl_GetResult_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Response__rosidl_typesupport_introspection_c__OperateCabinetControl_GetResult_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "xczs_inspection_robot_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "xczs_inspection_robot_interfaces/action/detail/operate_cabinet_control__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers xczs_inspection_robot_interfaces__action__detail__operate_cabinet_control__rosidl_typesupport_introspection_c__OperateCabinetControl_GetResult_service_members = {
  "xczs_inspection_robot_interfaces__action",  // service namespace
  "OperateCabinetControl_GetResult",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // xczs_inspection_robot_interfaces__action__detail__operate_cabinet_control__rosidl_typesupport_introspection_c__OperateCabinetControl_GetResult_Request_message_type_support_handle,
  NULL  // response message
  // xczs_inspection_robot_interfaces__action__detail__operate_cabinet_control__rosidl_typesupport_introspection_c__OperateCabinetControl_GetResult_Response_message_type_support_handle
};

static rosidl_service_type_support_t xczs_inspection_robot_interfaces__action__detail__operate_cabinet_control__rosidl_typesupport_introspection_c__OperateCabinetControl_GetResult_service_type_support_handle = {
  0,
  &xczs_inspection_robot_interfaces__action__detail__operate_cabinet_control__rosidl_typesupport_introspection_c__OperateCabinetControl_GetResult_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, xczs_inspection_robot_interfaces, action, OperateCabinetControl_GetResult_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, xczs_inspection_robot_interfaces, action, OperateCabinetControl_GetResult_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_xczs_inspection_robot_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, xczs_inspection_robot_interfaces, action, OperateCabinetControl_GetResult)() {
  if (!xczs_inspection_robot_interfaces__action__detail__operate_cabinet_control__rosidl_typesupport_introspection_c__OperateCabinetControl_GetResult_service_type_support_handle.typesupport_identifier) {
    xczs_inspection_robot_interfaces__action__detail__operate_cabinet_control__rosidl_typesupport_introspection_c__OperateCabinetControl_GetResult_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)xczs_inspection_robot_interfaces__action__detail__operate_cabinet_control__rosidl_typesupport_introspection_c__OperateCabinetControl_GetResult_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, xczs_inspection_robot_interfaces, action, OperateCabinetControl_GetResult_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, xczs_inspection_robot_interfaces, action, OperateCabinetControl_GetResult_Response)()->data;
  }

  return &xczs_inspection_robot_interfaces__action__detail__operate_cabinet_control__rosidl_typesupport_introspection_c__OperateCabinetControl_GetResult_service_type_support_handle;
}

// already included above
// #include <stddef.h>
// already included above
// #include "xczs_inspection_robot_interfaces/action/detail/operate_cabinet_control__rosidl_typesupport_introspection_c.h"
// already included above
// #include "xczs_inspection_robot_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "xczs_inspection_robot_interfaces/action/detail/operate_cabinet_control__functions.h"
// already included above
// #include "xczs_inspection_robot_interfaces/action/detail/operate_cabinet_control__struct.h"


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"
// Member `feedback`
// already included above
// #include "xczs_inspection_robot_interfaces/action/operate_cabinet_control.h"
// Member `feedback`
// already included above
// #include "xczs_inspection_robot_interfaces/action/detail/operate_cabinet_control__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void xczs_inspection_robot_interfaces__action__OperateCabinetControl_FeedbackMessage__rosidl_typesupport_introspection_c__OperateCabinetControl_FeedbackMessage_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_FeedbackMessage__init(message_memory);
}

void xczs_inspection_robot_interfaces__action__OperateCabinetControl_FeedbackMessage__rosidl_typesupport_introspection_c__OperateCabinetControl_FeedbackMessage_fini_function(void * message_memory)
{
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_FeedbackMessage__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember xczs_inspection_robot_interfaces__action__OperateCabinetControl_FeedbackMessage__rosidl_typesupport_introspection_c__OperateCabinetControl_FeedbackMessage_message_member_array[2] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_interfaces__action__OperateCabinetControl_FeedbackMessage, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "feedback",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_interfaces__action__OperateCabinetControl_FeedbackMessage, feedback),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers xczs_inspection_robot_interfaces__action__OperateCabinetControl_FeedbackMessage__rosidl_typesupport_introspection_c__OperateCabinetControl_FeedbackMessage_message_members = {
  "xczs_inspection_robot_interfaces__action",  // message namespace
  "OperateCabinetControl_FeedbackMessage",  // message name
  2,  // number of fields
  sizeof(xczs_inspection_robot_interfaces__action__OperateCabinetControl_FeedbackMessage),
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_FeedbackMessage__rosidl_typesupport_introspection_c__OperateCabinetControl_FeedbackMessage_message_member_array,  // message members
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_FeedbackMessage__rosidl_typesupport_introspection_c__OperateCabinetControl_FeedbackMessage_init_function,  // function to initialize message memory (memory has to be allocated)
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_FeedbackMessage__rosidl_typesupport_introspection_c__OperateCabinetControl_FeedbackMessage_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t xczs_inspection_robot_interfaces__action__OperateCabinetControl_FeedbackMessage__rosidl_typesupport_introspection_c__OperateCabinetControl_FeedbackMessage_message_type_support_handle = {
  0,
  &xczs_inspection_robot_interfaces__action__OperateCabinetControl_FeedbackMessage__rosidl_typesupport_introspection_c__OperateCabinetControl_FeedbackMessage_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_xczs_inspection_robot_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, xczs_inspection_robot_interfaces, action, OperateCabinetControl_FeedbackMessage)() {
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_FeedbackMessage__rosidl_typesupport_introspection_c__OperateCabinetControl_FeedbackMessage_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_FeedbackMessage__rosidl_typesupport_introspection_c__OperateCabinetControl_FeedbackMessage_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, xczs_inspection_robot_interfaces, action, OperateCabinetControl_Feedback)();
  if (!xczs_inspection_robot_interfaces__action__OperateCabinetControl_FeedbackMessage__rosidl_typesupport_introspection_c__OperateCabinetControl_FeedbackMessage_message_type_support_handle.typesupport_identifier) {
    xczs_inspection_robot_interfaces__action__OperateCabinetControl_FeedbackMessage__rosidl_typesupport_introspection_c__OperateCabinetControl_FeedbackMessage_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &xczs_inspection_robot_interfaces__action__OperateCabinetControl_FeedbackMessage__rosidl_typesupport_introspection_c__OperateCabinetControl_FeedbackMessage_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
