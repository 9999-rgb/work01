// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from xczs_inspection_robot_control:srv/SetCabinetGrasp.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "xczs_inspection_robot_control/srv/detail/set_cabinet_grasp__rosidl_typesupport_introspection_c.h"
#include "xczs_inspection_robot_control/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "xczs_inspection_robot_control/srv/detail/set_cabinet_grasp__functions.h"
#include "xczs_inspection_robot_control/srv/detail/set_cabinet_grasp__struct.h"


// Include directives for member types
// Member `control_id`
// Member `operation_lease_id`
// Member `robot_model`
// Member `robot_link`
// Member `robot_base_link`
#include "rosidl_runtime_c/string_functions.h"
// Member `robot_grasp_point`
#include "geometry_msgs/msg/point.h"
// Member `robot_grasp_point`
#include "geometry_msgs/msg/detail/point__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void xczs_inspection_robot_control__srv__SetCabinetGrasp_Request__rosidl_typesupport_introspection_c__SetCabinetGrasp_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  xczs_inspection_robot_control__srv__SetCabinetGrasp_Request__init(message_memory);
}

void xczs_inspection_robot_control__srv__SetCabinetGrasp_Request__rosidl_typesupport_introspection_c__SetCabinetGrasp_Request_fini_function(void * message_memory)
{
  xczs_inspection_robot_control__srv__SetCabinetGrasp_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember xczs_inspection_robot_control__srv__SetCabinetGrasp_Request__rosidl_typesupport_introspection_c__SetCabinetGrasp_Request_message_member_array[7] = {
  {
    "control_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_control__srv__SetCabinetGrasp_Request, control_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "operation_lease_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_control__srv__SetCabinetGrasp_Request, operation_lease_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "robot_model",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_control__srv__SetCabinetGrasp_Request, robot_model),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "robot_link",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_control__srv__SetCabinetGrasp_Request, robot_link),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "robot_grasp_point",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_control__srv__SetCabinetGrasp_Request, robot_grasp_point),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "robot_base_link",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_control__srv__SetCabinetGrasp_Request, robot_base_link),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "attach",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_control__srv__SetCabinetGrasp_Request, attach),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers xczs_inspection_robot_control__srv__SetCabinetGrasp_Request__rosidl_typesupport_introspection_c__SetCabinetGrasp_Request_message_members = {
  "xczs_inspection_robot_control__srv",  // message namespace
  "SetCabinetGrasp_Request",  // message name
  7,  // number of fields
  sizeof(xczs_inspection_robot_control__srv__SetCabinetGrasp_Request),
  xczs_inspection_robot_control__srv__SetCabinetGrasp_Request__rosidl_typesupport_introspection_c__SetCabinetGrasp_Request_message_member_array,  // message members
  xczs_inspection_robot_control__srv__SetCabinetGrasp_Request__rosidl_typesupport_introspection_c__SetCabinetGrasp_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  xczs_inspection_robot_control__srv__SetCabinetGrasp_Request__rosidl_typesupport_introspection_c__SetCabinetGrasp_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t xczs_inspection_robot_control__srv__SetCabinetGrasp_Request__rosidl_typesupport_introspection_c__SetCabinetGrasp_Request_message_type_support_handle = {
  0,
  &xczs_inspection_robot_control__srv__SetCabinetGrasp_Request__rosidl_typesupport_introspection_c__SetCabinetGrasp_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_xczs_inspection_robot_control
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, xczs_inspection_robot_control, srv, SetCabinetGrasp_Request)() {
  xczs_inspection_robot_control__srv__SetCabinetGrasp_Request__rosidl_typesupport_introspection_c__SetCabinetGrasp_Request_message_member_array[4].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Point)();
  if (!xczs_inspection_robot_control__srv__SetCabinetGrasp_Request__rosidl_typesupport_introspection_c__SetCabinetGrasp_Request_message_type_support_handle.typesupport_identifier) {
    xczs_inspection_robot_control__srv__SetCabinetGrasp_Request__rosidl_typesupport_introspection_c__SetCabinetGrasp_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &xczs_inspection_robot_control__srv__SetCabinetGrasp_Request__rosidl_typesupport_introspection_c__SetCabinetGrasp_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "xczs_inspection_robot_control/srv/detail/set_cabinet_grasp__rosidl_typesupport_introspection_c.h"
// already included above
// #include "xczs_inspection_robot_control/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "xczs_inspection_robot_control/srv/detail/set_cabinet_grasp__functions.h"
// already included above
// #include "xczs_inspection_robot_control/srv/detail/set_cabinet_grasp__struct.h"


// Include directives for member types
// Member `message`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void xczs_inspection_robot_control__srv__SetCabinetGrasp_Response__rosidl_typesupport_introspection_c__SetCabinetGrasp_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  xczs_inspection_robot_control__srv__SetCabinetGrasp_Response__init(message_memory);
}

void xczs_inspection_robot_control__srv__SetCabinetGrasp_Response__rosidl_typesupport_introspection_c__SetCabinetGrasp_Response_fini_function(void * message_memory)
{
  xczs_inspection_robot_control__srv__SetCabinetGrasp_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember xczs_inspection_robot_control__srv__SetCabinetGrasp_Response__rosidl_typesupport_introspection_c__SetCabinetGrasp_Response_message_member_array[3] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_control__srv__SetCabinetGrasp_Response, success),  // bytes offset in struct
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
    offsetof(xczs_inspection_robot_control__srv__SetCabinetGrasp_Response, message),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "distance",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(xczs_inspection_robot_control__srv__SetCabinetGrasp_Response, distance),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers xczs_inspection_robot_control__srv__SetCabinetGrasp_Response__rosidl_typesupport_introspection_c__SetCabinetGrasp_Response_message_members = {
  "xczs_inspection_robot_control__srv",  // message namespace
  "SetCabinetGrasp_Response",  // message name
  3,  // number of fields
  sizeof(xczs_inspection_robot_control__srv__SetCabinetGrasp_Response),
  xczs_inspection_robot_control__srv__SetCabinetGrasp_Response__rosidl_typesupport_introspection_c__SetCabinetGrasp_Response_message_member_array,  // message members
  xczs_inspection_robot_control__srv__SetCabinetGrasp_Response__rosidl_typesupport_introspection_c__SetCabinetGrasp_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  xczs_inspection_robot_control__srv__SetCabinetGrasp_Response__rosidl_typesupport_introspection_c__SetCabinetGrasp_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t xczs_inspection_robot_control__srv__SetCabinetGrasp_Response__rosidl_typesupport_introspection_c__SetCabinetGrasp_Response_message_type_support_handle = {
  0,
  &xczs_inspection_robot_control__srv__SetCabinetGrasp_Response__rosidl_typesupport_introspection_c__SetCabinetGrasp_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_xczs_inspection_robot_control
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, xczs_inspection_robot_control, srv, SetCabinetGrasp_Response)() {
  if (!xczs_inspection_robot_control__srv__SetCabinetGrasp_Response__rosidl_typesupport_introspection_c__SetCabinetGrasp_Response_message_type_support_handle.typesupport_identifier) {
    xczs_inspection_robot_control__srv__SetCabinetGrasp_Response__rosidl_typesupport_introspection_c__SetCabinetGrasp_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &xczs_inspection_robot_control__srv__SetCabinetGrasp_Response__rosidl_typesupport_introspection_c__SetCabinetGrasp_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "xczs_inspection_robot_control/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "xczs_inspection_robot_control/srv/detail/set_cabinet_grasp__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers xczs_inspection_robot_control__srv__detail__set_cabinet_grasp__rosidl_typesupport_introspection_c__SetCabinetGrasp_service_members = {
  "xczs_inspection_robot_control__srv",  // service namespace
  "SetCabinetGrasp",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // xczs_inspection_robot_control__srv__detail__set_cabinet_grasp__rosidl_typesupport_introspection_c__SetCabinetGrasp_Request_message_type_support_handle,
  NULL  // response message
  // xczs_inspection_robot_control__srv__detail__set_cabinet_grasp__rosidl_typesupport_introspection_c__SetCabinetGrasp_Response_message_type_support_handle
};

static rosidl_service_type_support_t xczs_inspection_robot_control__srv__detail__set_cabinet_grasp__rosidl_typesupport_introspection_c__SetCabinetGrasp_service_type_support_handle = {
  0,
  &xczs_inspection_robot_control__srv__detail__set_cabinet_grasp__rosidl_typesupport_introspection_c__SetCabinetGrasp_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, xczs_inspection_robot_control, srv, SetCabinetGrasp_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, xczs_inspection_robot_control, srv, SetCabinetGrasp_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_xczs_inspection_robot_control
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, xczs_inspection_robot_control, srv, SetCabinetGrasp)() {
  if (!xczs_inspection_robot_control__srv__detail__set_cabinet_grasp__rosidl_typesupport_introspection_c__SetCabinetGrasp_service_type_support_handle.typesupport_identifier) {
    xczs_inspection_robot_control__srv__detail__set_cabinet_grasp__rosidl_typesupport_introspection_c__SetCabinetGrasp_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)xczs_inspection_robot_control__srv__detail__set_cabinet_grasp__rosidl_typesupport_introspection_c__SetCabinetGrasp_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, xczs_inspection_robot_control, srv, SetCabinetGrasp_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, xczs_inspection_robot_control, srv, SetCabinetGrasp_Response)()->data;
  }

  return &xczs_inspection_robot_control__srv__detail__set_cabinet_grasp__rosidl_typesupport_introspection_c__SetCabinetGrasp_service_type_support_handle;
}
