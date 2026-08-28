// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from xczs_inspection_robot_interfaces:srv/SwitchToolset.idl
// generated code does not contain a copyright notice

#ifndef XCZS_INSPECTION_ROBOT_INTERFACES__SRV__DETAIL__SWITCH_TOOLSET__FUNCTIONS_H_
#define XCZS_INSPECTION_ROBOT_INTERFACES__SRV__DETAIL__SWITCH_TOOLSET__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "xczs_inspection_robot_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "xczs_inspection_robot_interfaces/srv/detail/switch_toolset__struct.h"

/// Initialize srv/SwitchToolset message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * xczs_inspection_robot_interfaces__srv__SwitchToolset_Request
 * )) before or use
 * xczs_inspection_robot_interfaces__srv__SwitchToolset_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_xczs_inspection_robot_interfaces
bool
xczs_inspection_robot_interfaces__srv__SwitchToolset_Request__init(xczs_inspection_robot_interfaces__srv__SwitchToolset_Request * msg);

/// Finalize srv/SwitchToolset message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_xczs_inspection_robot_interfaces
void
xczs_inspection_robot_interfaces__srv__SwitchToolset_Request__fini(xczs_inspection_robot_interfaces__srv__SwitchToolset_Request * msg);

/// Create srv/SwitchToolset message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * xczs_inspection_robot_interfaces__srv__SwitchToolset_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_xczs_inspection_robot_interfaces
xczs_inspection_robot_interfaces__srv__SwitchToolset_Request *
xczs_inspection_robot_interfaces__srv__SwitchToolset_Request__create();

/// Destroy srv/SwitchToolset message.
/**
 * It calls
 * xczs_inspection_robot_interfaces__srv__SwitchToolset_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_xczs_inspection_robot_interfaces
void
xczs_inspection_robot_interfaces__srv__SwitchToolset_Request__destroy(xczs_inspection_robot_interfaces__srv__SwitchToolset_Request * msg);

/// Check for srv/SwitchToolset message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_xczs_inspection_robot_interfaces
bool
xczs_inspection_robot_interfaces__srv__SwitchToolset_Request__are_equal(const xczs_inspection_robot_interfaces__srv__SwitchToolset_Request * lhs, const xczs_inspection_robot_interfaces__srv__SwitchToolset_Request * rhs);

/// Copy a srv/SwitchToolset message.
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
xczs_inspection_robot_interfaces__srv__SwitchToolset_Request__copy(
  const xczs_inspection_robot_interfaces__srv__SwitchToolset_Request * input,
  xczs_inspection_robot_interfaces__srv__SwitchToolset_Request * output);

/// Initialize array of srv/SwitchToolset messages.
/**
 * It allocates the memory for the number of elements and calls
 * xczs_inspection_robot_interfaces__srv__SwitchToolset_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_xczs_inspection_robot_interfaces
bool
xczs_inspection_robot_interfaces__srv__SwitchToolset_Request__Sequence__init(xczs_inspection_robot_interfaces__srv__SwitchToolset_Request__Sequence * array, size_t size);

/// Finalize array of srv/SwitchToolset messages.
/**
 * It calls
 * xczs_inspection_robot_interfaces__srv__SwitchToolset_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_xczs_inspection_robot_interfaces
void
xczs_inspection_robot_interfaces__srv__SwitchToolset_Request__Sequence__fini(xczs_inspection_robot_interfaces__srv__SwitchToolset_Request__Sequence * array);

/// Create array of srv/SwitchToolset messages.
/**
 * It allocates the memory for the array and calls
 * xczs_inspection_robot_interfaces__srv__SwitchToolset_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_xczs_inspection_robot_interfaces
xczs_inspection_robot_interfaces__srv__SwitchToolset_Request__Sequence *
xczs_inspection_robot_interfaces__srv__SwitchToolset_Request__Sequence__create(size_t size);

/// Destroy array of srv/SwitchToolset messages.
/**
 * It calls
 * xczs_inspection_robot_interfaces__srv__SwitchToolset_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_xczs_inspection_robot_interfaces
void
xczs_inspection_robot_interfaces__srv__SwitchToolset_Request__Sequence__destroy(xczs_inspection_robot_interfaces__srv__SwitchToolset_Request__Sequence * array);

/// Check for srv/SwitchToolset message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_xczs_inspection_robot_interfaces
bool
xczs_inspection_robot_interfaces__srv__SwitchToolset_Request__Sequence__are_equal(const xczs_inspection_robot_interfaces__srv__SwitchToolset_Request__Sequence * lhs, const xczs_inspection_robot_interfaces__srv__SwitchToolset_Request__Sequence * rhs);

/// Copy an array of srv/SwitchToolset messages.
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
xczs_inspection_robot_interfaces__srv__SwitchToolset_Request__Sequence__copy(
  const xczs_inspection_robot_interfaces__srv__SwitchToolset_Request__Sequence * input,
  xczs_inspection_robot_interfaces__srv__SwitchToolset_Request__Sequence * output);

/// Initialize srv/SwitchToolset message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * xczs_inspection_robot_interfaces__srv__SwitchToolset_Response
 * )) before or use
 * xczs_inspection_robot_interfaces__srv__SwitchToolset_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_xczs_inspection_robot_interfaces
bool
xczs_inspection_robot_interfaces__srv__SwitchToolset_Response__init(xczs_inspection_robot_interfaces__srv__SwitchToolset_Response * msg);

/// Finalize srv/SwitchToolset message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_xczs_inspection_robot_interfaces
void
xczs_inspection_robot_interfaces__srv__SwitchToolset_Response__fini(xczs_inspection_robot_interfaces__srv__SwitchToolset_Response * msg);

/// Create srv/SwitchToolset message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * xczs_inspection_robot_interfaces__srv__SwitchToolset_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_xczs_inspection_robot_interfaces
xczs_inspection_robot_interfaces__srv__SwitchToolset_Response *
xczs_inspection_robot_interfaces__srv__SwitchToolset_Response__create();

/// Destroy srv/SwitchToolset message.
/**
 * It calls
 * xczs_inspection_robot_interfaces__srv__SwitchToolset_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_xczs_inspection_robot_interfaces
void
xczs_inspection_robot_interfaces__srv__SwitchToolset_Response__destroy(xczs_inspection_robot_interfaces__srv__SwitchToolset_Response * msg);

/// Check for srv/SwitchToolset message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_xczs_inspection_robot_interfaces
bool
xczs_inspection_robot_interfaces__srv__SwitchToolset_Response__are_equal(const xczs_inspection_robot_interfaces__srv__SwitchToolset_Response * lhs, const xczs_inspection_robot_interfaces__srv__SwitchToolset_Response * rhs);

/// Copy a srv/SwitchToolset message.
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
xczs_inspection_robot_interfaces__srv__SwitchToolset_Response__copy(
  const xczs_inspection_robot_interfaces__srv__SwitchToolset_Response * input,
  xczs_inspection_robot_interfaces__srv__SwitchToolset_Response * output);

/// Initialize array of srv/SwitchToolset messages.
/**
 * It allocates the memory for the number of elements and calls
 * xczs_inspection_robot_interfaces__srv__SwitchToolset_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_xczs_inspection_robot_interfaces
bool
xczs_inspection_robot_interfaces__srv__SwitchToolset_Response__Sequence__init(xczs_inspection_robot_interfaces__srv__SwitchToolset_Response__Sequence * array, size_t size);

/// Finalize array of srv/SwitchToolset messages.
/**
 * It calls
 * xczs_inspection_robot_interfaces__srv__SwitchToolset_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_xczs_inspection_robot_interfaces
void
xczs_inspection_robot_interfaces__srv__SwitchToolset_Response__Sequence__fini(xczs_inspection_robot_interfaces__srv__SwitchToolset_Response__Sequence * array);

/// Create array of srv/SwitchToolset messages.
/**
 * It allocates the memory for the array and calls
 * xczs_inspection_robot_interfaces__srv__SwitchToolset_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_xczs_inspection_robot_interfaces
xczs_inspection_robot_interfaces__srv__SwitchToolset_Response__Sequence *
xczs_inspection_robot_interfaces__srv__SwitchToolset_Response__Sequence__create(size_t size);

/// Destroy array of srv/SwitchToolset messages.
/**
 * It calls
 * xczs_inspection_robot_interfaces__srv__SwitchToolset_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_xczs_inspection_robot_interfaces
void
xczs_inspection_robot_interfaces__srv__SwitchToolset_Response__Sequence__destroy(xczs_inspection_robot_interfaces__srv__SwitchToolset_Response__Sequence * array);

/// Check for srv/SwitchToolset message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_xczs_inspection_robot_interfaces
bool
xczs_inspection_robot_interfaces__srv__SwitchToolset_Response__Sequence__are_equal(const xczs_inspection_robot_interfaces__srv__SwitchToolset_Response__Sequence * lhs, const xczs_inspection_robot_interfaces__srv__SwitchToolset_Response__Sequence * rhs);

/// Copy an array of srv/SwitchToolset messages.
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
xczs_inspection_robot_interfaces__srv__SwitchToolset_Response__Sequence__copy(
  const xczs_inspection_robot_interfaces__srv__SwitchToolset_Response__Sequence * input,
  xczs_inspection_robot_interfaces__srv__SwitchToolset_Response__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // XCZS_INSPECTION_ROBOT_INTERFACES__SRV__DETAIL__SWITCH_TOOLSET__FUNCTIONS_H_
