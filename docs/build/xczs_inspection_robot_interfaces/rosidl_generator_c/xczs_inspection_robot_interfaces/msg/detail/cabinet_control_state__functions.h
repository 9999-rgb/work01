// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from xczs_inspection_robot_interfaces:msg/CabinetControlState.idl
// generated code does not contain a copyright notice

#ifndef XCZS_INSPECTION_ROBOT_INTERFACES__MSG__DETAIL__CABINET_CONTROL_STATE__FUNCTIONS_H_
#define XCZS_INSPECTION_ROBOT_INTERFACES__MSG__DETAIL__CABINET_CONTROL_STATE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "xczs_inspection_robot_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "xczs_inspection_robot_interfaces/msg/detail/cabinet_control_state__struct.h"

/// Initialize msg/CabinetControlState message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * xczs_inspection_robot_interfaces__msg__CabinetControlState
 * )) before or use
 * xczs_inspection_robot_interfaces__msg__CabinetControlState__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_xczs_inspection_robot_interfaces
bool
xczs_inspection_robot_interfaces__msg__CabinetControlState__init(xczs_inspection_robot_interfaces__msg__CabinetControlState * msg);

/// Finalize msg/CabinetControlState message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_xczs_inspection_robot_interfaces
void
xczs_inspection_robot_interfaces__msg__CabinetControlState__fini(xczs_inspection_robot_interfaces__msg__CabinetControlState * msg);

/// Create msg/CabinetControlState message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * xczs_inspection_robot_interfaces__msg__CabinetControlState__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_xczs_inspection_robot_interfaces
xczs_inspection_robot_interfaces__msg__CabinetControlState *
xczs_inspection_robot_interfaces__msg__CabinetControlState__create();

/// Destroy msg/CabinetControlState message.
/**
 * It calls
 * xczs_inspection_robot_interfaces__msg__CabinetControlState__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_xczs_inspection_robot_interfaces
void
xczs_inspection_robot_interfaces__msg__CabinetControlState__destroy(xczs_inspection_robot_interfaces__msg__CabinetControlState * msg);

/// Check for msg/CabinetControlState message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_xczs_inspection_robot_interfaces
bool
xczs_inspection_robot_interfaces__msg__CabinetControlState__are_equal(const xczs_inspection_robot_interfaces__msg__CabinetControlState * lhs, const xczs_inspection_robot_interfaces__msg__CabinetControlState * rhs);

/// Copy a msg/CabinetControlState message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_xczs_inspection_robot_interfaces
bool
xczs_inspection_robot_interfaces__msg__CabinetControlState__copy(
  const xczs_inspection_robot_interfaces__msg__CabinetControlState * input,
  xczs_inspection_robot_interfaces__msg__CabinetControlState * output);

/// Initialize array of msg/CabinetControlState messages.
/**
 * It allocates the memory for the number of elements and calls
 * xczs_inspection_robot_interfaces__msg__CabinetControlState__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_xczs_inspection_robot_interfaces
bool
xczs_inspection_robot_interfaces__msg__CabinetControlState__Sequence__init(xczs_inspection_robot_interfaces__msg__CabinetControlState__Sequence * array, size_t size);

/// Finalize array of msg/CabinetControlState messages.
/**
 * It calls
 * xczs_inspection_robot_interfaces__msg__CabinetControlState__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_xczs_inspection_robot_interfaces
void
xczs_inspection_robot_interfaces__msg__CabinetControlState__Sequence__fini(xczs_inspection_robot_interfaces__msg__CabinetControlState__Sequence * array);

/// Create array of msg/CabinetControlState messages.
/**
 * It allocates the memory for the array and calls
 * xczs_inspection_robot_interfaces__msg__CabinetControlState__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_xczs_inspection_robot_interfaces
xczs_inspection_robot_interfaces__msg__CabinetControlState__Sequence *
xczs_inspection_robot_interfaces__msg__CabinetControlState__Sequence__create(size_t size);

/// Destroy array of msg/CabinetControlState messages.
/**
 * It calls
 * xczs_inspection_robot_interfaces__msg__CabinetControlState__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_xczs_inspection_robot_interfaces
void
xczs_inspection_robot_interfaces__msg__CabinetControlState__Sequence__destroy(xczs_inspection_robot_interfaces__msg__CabinetControlState__Sequence * array);

/// Check for msg/CabinetControlState message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_xczs_inspection_robot_interfaces
bool
xczs_inspection_robot_interfaces__msg__CabinetControlState__Sequence__are_equal(const xczs_inspection_robot_interfaces__msg__CabinetControlState__Sequence * lhs, const xczs_inspection_robot_interfaces__msg__CabinetControlState__Sequence * rhs);

/// Copy an array of msg/CabinetControlState messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_xczs_inspection_robot_interfaces
bool
xczs_inspection_robot_interfaces__msg__CabinetControlState__Sequence__copy(
  const xczs_inspection_robot_interfaces__msg__CabinetControlState__Sequence * input,
  xczs_inspection_robot_interfaces__msg__CabinetControlState__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // XCZS_INSPECTION_ROBOT_INTERFACES__MSG__DETAIL__CABINET_CONTROL_STATE__FUNCTIONS_H_
