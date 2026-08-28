// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from xczs_inspection_robot_interfaces:srv/ManageOperationLease.idl
// generated code does not contain a copyright notice

#ifndef XCZS_INSPECTION_ROBOT_INTERFACES__SRV__DETAIL__MANAGE_OPERATION_LEASE__STRUCT_H_
#define XCZS_INSPECTION_ROBOT_INTERFACES__SRV__DETAIL__MANAGE_OPERATION_LEASE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'ACQUIRE'.
enum
{
  xczs_inspection_robot_interfaces__srv__ManageOperationLease_Request__ACQUIRE = 0
};

/// Constant 'RENEW'.
enum
{
  xczs_inspection_robot_interfaces__srv__ManageOperationLease_Request__RENEW = 1
};

/// Constant 'RELEASE'.
enum
{
  xczs_inspection_robot_interfaces__srv__ManageOperationLease_Request__RELEASE = 2
};

// Include directives for member types
// Member 'owner_id'
// Member 'lease_id'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/ManageOperationLease in the package xczs_inspection_robot_interfaces.
typedef struct xczs_inspection_robot_interfaces__srv__ManageOperationLease_Request
{
  uint8_t command;
  rosidl_runtime_c__String owner_id;
  rosidl_runtime_c__String lease_id;
  double requested_duration;
} xczs_inspection_robot_interfaces__srv__ManageOperationLease_Request;

// Struct for a sequence of xczs_inspection_robot_interfaces__srv__ManageOperationLease_Request.
typedef struct xczs_inspection_robot_interfaces__srv__ManageOperationLease_Request__Sequence
{
  xczs_inspection_robot_interfaces__srv__ManageOperationLease_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} xczs_inspection_robot_interfaces__srv__ManageOperationLease_Request__Sequence;


// Constants defined in the message

/// Constant 'GRANTED'.
enum
{
  xczs_inspection_robot_interfaces__srv__ManageOperationLease_Response__GRANTED = 0
};

/// Constant 'RESOURCE_BUSY'.
enum
{
  xczs_inspection_robot_interfaces__srv__ManageOperationLease_Response__RESOURCE_BUSY = 1
};

/// Constant 'INVALID_REQUEST'.
enum
{
  xczs_inspection_robot_interfaces__srv__ManageOperationLease_Response__INVALID_REQUEST = 2
};

/// Constant 'NOT_OWNER'.
enum
{
  xczs_inspection_robot_interfaces__srv__ManageOperationLease_Response__NOT_OWNER = 3
};

/// Constant 'RELEASED'.
enum
{
  xczs_inspection_robot_interfaces__srv__ManageOperationLease_Response__RELEASED = 4
};

// Include directives for member types
// Member 'message'
// Member 'lease_id'
// Member 'owner_id'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/ManageOperationLease in the package xczs_inspection_robot_interfaces.
typedef struct xczs_inspection_robot_interfaces__srv__ManageOperationLease_Response
{
  bool success;
  uint8_t status_code;
  rosidl_runtime_c__String message;
  rosidl_runtime_c__String lease_id;
  rosidl_runtime_c__String owner_id;
  double remaining_duration;
} xczs_inspection_robot_interfaces__srv__ManageOperationLease_Response;

// Struct for a sequence of xczs_inspection_robot_interfaces__srv__ManageOperationLease_Response.
typedef struct xczs_inspection_robot_interfaces__srv__ManageOperationLease_Response__Sequence
{
  xczs_inspection_robot_interfaces__srv__ManageOperationLease_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} xczs_inspection_robot_interfaces__srv__ManageOperationLease_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // XCZS_INSPECTION_ROBOT_INTERFACES__SRV__DETAIL__MANAGE_OPERATION_LEASE__STRUCT_H_
