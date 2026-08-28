// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from xczs_inspection_robot_interfaces:msg/CabinetControlCatalog.idl
// generated code does not contain a copyright notice

#ifndef XCZS_INSPECTION_ROBOT_INTERFACES__MSG__DETAIL__CABINET_CONTROL_CATALOG__STRUCT_H_
#define XCZS_INSPECTION_ROBOT_INTERFACES__MSG__DETAIL__CABINET_CONTROL_CATALOG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'controls'
#include "xczs_inspection_robot_interfaces/msg/detail/cabinet_control__struct.h"

/// Struct defined in msg/CabinetControlCatalog in the package xczs_inspection_robot_interfaces.
typedef struct xczs_inspection_robot_interfaces__msg__CabinetControlCatalog
{
  xczs_inspection_robot_interfaces__msg__CabinetControl__Sequence controls;
} xczs_inspection_robot_interfaces__msg__CabinetControlCatalog;

// Struct for a sequence of xczs_inspection_robot_interfaces__msg__CabinetControlCatalog.
typedef struct xczs_inspection_robot_interfaces__msg__CabinetControlCatalog__Sequence
{
  xczs_inspection_robot_interfaces__msg__CabinetControlCatalog * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} xczs_inspection_robot_interfaces__msg__CabinetControlCatalog__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // XCZS_INSPECTION_ROBOT_INTERFACES__MSG__DETAIL__CABINET_CONTROL_CATALOG__STRUCT_H_
