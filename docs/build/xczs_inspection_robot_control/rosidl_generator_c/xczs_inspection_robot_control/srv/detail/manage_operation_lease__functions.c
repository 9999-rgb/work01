// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from xczs_inspection_robot_control:srv/ManageOperationLease.idl
// generated code does not contain a copyright notice
#include "xczs_inspection_robot_control/srv/detail/manage_operation_lease__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `owner_id`
// Member `lease_id`
#include "rosidl_runtime_c/string_functions.h"

bool
xczs_inspection_robot_control__srv__ManageOperationLease_Request__init(xczs_inspection_robot_control__srv__ManageOperationLease_Request * msg)
{
  if (!msg) {
    return false;
  }
  // command
  // owner_id
  if (!rosidl_runtime_c__String__init(&msg->owner_id)) {
    xczs_inspection_robot_control__srv__ManageOperationLease_Request__fini(msg);
    return false;
  }
  // lease_id
  if (!rosidl_runtime_c__String__init(&msg->lease_id)) {
    xczs_inspection_robot_control__srv__ManageOperationLease_Request__fini(msg);
    return false;
  }
  // requested_duration
  return true;
}

void
xczs_inspection_robot_control__srv__ManageOperationLease_Request__fini(xczs_inspection_robot_control__srv__ManageOperationLease_Request * msg)
{
  if (!msg) {
    return;
  }
  // command
  // owner_id
  rosidl_runtime_c__String__fini(&msg->owner_id);
  // lease_id
  rosidl_runtime_c__String__fini(&msg->lease_id);
  // requested_duration
}

bool
xczs_inspection_robot_control__srv__ManageOperationLease_Request__are_equal(const xczs_inspection_robot_control__srv__ManageOperationLease_Request * lhs, const xczs_inspection_robot_control__srv__ManageOperationLease_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // command
  if (lhs->command != rhs->command) {
    return false;
  }
  // owner_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->owner_id), &(rhs->owner_id)))
  {
    return false;
  }
  // lease_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->lease_id), &(rhs->lease_id)))
  {
    return false;
  }
  // requested_duration
  if (lhs->requested_duration != rhs->requested_duration) {
    return false;
  }
  return true;
}

bool
xczs_inspection_robot_control__srv__ManageOperationLease_Request__copy(
  const xczs_inspection_robot_control__srv__ManageOperationLease_Request * input,
  xczs_inspection_robot_control__srv__ManageOperationLease_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // command
  output->command = input->command;
  // owner_id
  if (!rosidl_runtime_c__String__copy(
      &(input->owner_id), &(output->owner_id)))
  {
    return false;
  }
  // lease_id
  if (!rosidl_runtime_c__String__copy(
      &(input->lease_id), &(output->lease_id)))
  {
    return false;
  }
  // requested_duration
  output->requested_duration = input->requested_duration;
  return true;
}

xczs_inspection_robot_control__srv__ManageOperationLease_Request *
xczs_inspection_robot_control__srv__ManageOperationLease_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  xczs_inspection_robot_control__srv__ManageOperationLease_Request * msg = (xczs_inspection_robot_control__srv__ManageOperationLease_Request *)allocator.allocate(sizeof(xczs_inspection_robot_control__srv__ManageOperationLease_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(xczs_inspection_robot_control__srv__ManageOperationLease_Request));
  bool success = xczs_inspection_robot_control__srv__ManageOperationLease_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
xczs_inspection_robot_control__srv__ManageOperationLease_Request__destroy(xczs_inspection_robot_control__srv__ManageOperationLease_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    xczs_inspection_robot_control__srv__ManageOperationLease_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
xczs_inspection_robot_control__srv__ManageOperationLease_Request__Sequence__init(xczs_inspection_robot_control__srv__ManageOperationLease_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  xczs_inspection_robot_control__srv__ManageOperationLease_Request * data = NULL;

  if (size) {
    data = (xczs_inspection_robot_control__srv__ManageOperationLease_Request *)allocator.zero_allocate(size, sizeof(xczs_inspection_robot_control__srv__ManageOperationLease_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = xczs_inspection_robot_control__srv__ManageOperationLease_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        xczs_inspection_robot_control__srv__ManageOperationLease_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
xczs_inspection_robot_control__srv__ManageOperationLease_Request__Sequence__fini(xczs_inspection_robot_control__srv__ManageOperationLease_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      xczs_inspection_robot_control__srv__ManageOperationLease_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

xczs_inspection_robot_control__srv__ManageOperationLease_Request__Sequence *
xczs_inspection_robot_control__srv__ManageOperationLease_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  xczs_inspection_robot_control__srv__ManageOperationLease_Request__Sequence * array = (xczs_inspection_robot_control__srv__ManageOperationLease_Request__Sequence *)allocator.allocate(sizeof(xczs_inspection_robot_control__srv__ManageOperationLease_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = xczs_inspection_robot_control__srv__ManageOperationLease_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
xczs_inspection_robot_control__srv__ManageOperationLease_Request__Sequence__destroy(xczs_inspection_robot_control__srv__ManageOperationLease_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    xczs_inspection_robot_control__srv__ManageOperationLease_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
xczs_inspection_robot_control__srv__ManageOperationLease_Request__Sequence__are_equal(const xczs_inspection_robot_control__srv__ManageOperationLease_Request__Sequence * lhs, const xczs_inspection_robot_control__srv__ManageOperationLease_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!xczs_inspection_robot_control__srv__ManageOperationLease_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
xczs_inspection_robot_control__srv__ManageOperationLease_Request__Sequence__copy(
  const xczs_inspection_robot_control__srv__ManageOperationLease_Request__Sequence * input,
  xczs_inspection_robot_control__srv__ManageOperationLease_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(xczs_inspection_robot_control__srv__ManageOperationLease_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    xczs_inspection_robot_control__srv__ManageOperationLease_Request * data =
      (xczs_inspection_robot_control__srv__ManageOperationLease_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!xczs_inspection_robot_control__srv__ManageOperationLease_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          xczs_inspection_robot_control__srv__ManageOperationLease_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!xczs_inspection_robot_control__srv__ManageOperationLease_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `message`
// Member `lease_id`
// Member `owner_id`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

bool
xczs_inspection_robot_control__srv__ManageOperationLease_Response__init(xczs_inspection_robot_control__srv__ManageOperationLease_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // status_code
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    xczs_inspection_robot_control__srv__ManageOperationLease_Response__fini(msg);
    return false;
  }
  // lease_id
  if (!rosidl_runtime_c__String__init(&msg->lease_id)) {
    xczs_inspection_robot_control__srv__ManageOperationLease_Response__fini(msg);
    return false;
  }
  // owner_id
  if (!rosidl_runtime_c__String__init(&msg->owner_id)) {
    xczs_inspection_robot_control__srv__ManageOperationLease_Response__fini(msg);
    return false;
  }
  // remaining_duration
  return true;
}

void
xczs_inspection_robot_control__srv__ManageOperationLease_Response__fini(xczs_inspection_robot_control__srv__ManageOperationLease_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // status_code
  // message
  rosidl_runtime_c__String__fini(&msg->message);
  // lease_id
  rosidl_runtime_c__String__fini(&msg->lease_id);
  // owner_id
  rosidl_runtime_c__String__fini(&msg->owner_id);
  // remaining_duration
}

bool
xczs_inspection_robot_control__srv__ManageOperationLease_Response__are_equal(const xczs_inspection_robot_control__srv__ManageOperationLease_Response * lhs, const xczs_inspection_robot_control__srv__ManageOperationLease_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // status_code
  if (lhs->status_code != rhs->status_code) {
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->message), &(rhs->message)))
  {
    return false;
  }
  // lease_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->lease_id), &(rhs->lease_id)))
  {
    return false;
  }
  // owner_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->owner_id), &(rhs->owner_id)))
  {
    return false;
  }
  // remaining_duration
  if (lhs->remaining_duration != rhs->remaining_duration) {
    return false;
  }
  return true;
}

bool
xczs_inspection_robot_control__srv__ManageOperationLease_Response__copy(
  const xczs_inspection_robot_control__srv__ManageOperationLease_Response * input,
  xczs_inspection_robot_control__srv__ManageOperationLease_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // status_code
  output->status_code = input->status_code;
  // message
  if (!rosidl_runtime_c__String__copy(
      &(input->message), &(output->message)))
  {
    return false;
  }
  // lease_id
  if (!rosidl_runtime_c__String__copy(
      &(input->lease_id), &(output->lease_id)))
  {
    return false;
  }
  // owner_id
  if (!rosidl_runtime_c__String__copy(
      &(input->owner_id), &(output->owner_id)))
  {
    return false;
  }
  // remaining_duration
  output->remaining_duration = input->remaining_duration;
  return true;
}

xczs_inspection_robot_control__srv__ManageOperationLease_Response *
xczs_inspection_robot_control__srv__ManageOperationLease_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  xczs_inspection_robot_control__srv__ManageOperationLease_Response * msg = (xczs_inspection_robot_control__srv__ManageOperationLease_Response *)allocator.allocate(sizeof(xczs_inspection_robot_control__srv__ManageOperationLease_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(xczs_inspection_robot_control__srv__ManageOperationLease_Response));
  bool success = xczs_inspection_robot_control__srv__ManageOperationLease_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
xczs_inspection_robot_control__srv__ManageOperationLease_Response__destroy(xczs_inspection_robot_control__srv__ManageOperationLease_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    xczs_inspection_robot_control__srv__ManageOperationLease_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
xczs_inspection_robot_control__srv__ManageOperationLease_Response__Sequence__init(xczs_inspection_robot_control__srv__ManageOperationLease_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  xczs_inspection_robot_control__srv__ManageOperationLease_Response * data = NULL;

  if (size) {
    data = (xczs_inspection_robot_control__srv__ManageOperationLease_Response *)allocator.zero_allocate(size, sizeof(xczs_inspection_robot_control__srv__ManageOperationLease_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = xczs_inspection_robot_control__srv__ManageOperationLease_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        xczs_inspection_robot_control__srv__ManageOperationLease_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
xczs_inspection_robot_control__srv__ManageOperationLease_Response__Sequence__fini(xczs_inspection_robot_control__srv__ManageOperationLease_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      xczs_inspection_robot_control__srv__ManageOperationLease_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

xczs_inspection_robot_control__srv__ManageOperationLease_Response__Sequence *
xczs_inspection_robot_control__srv__ManageOperationLease_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  xczs_inspection_robot_control__srv__ManageOperationLease_Response__Sequence * array = (xczs_inspection_robot_control__srv__ManageOperationLease_Response__Sequence *)allocator.allocate(sizeof(xczs_inspection_robot_control__srv__ManageOperationLease_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = xczs_inspection_robot_control__srv__ManageOperationLease_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
xczs_inspection_robot_control__srv__ManageOperationLease_Response__Sequence__destroy(xczs_inspection_robot_control__srv__ManageOperationLease_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    xczs_inspection_robot_control__srv__ManageOperationLease_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
xczs_inspection_robot_control__srv__ManageOperationLease_Response__Sequence__are_equal(const xczs_inspection_robot_control__srv__ManageOperationLease_Response__Sequence * lhs, const xczs_inspection_robot_control__srv__ManageOperationLease_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!xczs_inspection_robot_control__srv__ManageOperationLease_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
xczs_inspection_robot_control__srv__ManageOperationLease_Response__Sequence__copy(
  const xczs_inspection_robot_control__srv__ManageOperationLease_Response__Sequence * input,
  xczs_inspection_robot_control__srv__ManageOperationLease_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(xczs_inspection_robot_control__srv__ManageOperationLease_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    xczs_inspection_robot_control__srv__ManageOperationLease_Response * data =
      (xczs_inspection_robot_control__srv__ManageOperationLease_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!xczs_inspection_robot_control__srv__ManageOperationLease_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          xczs_inspection_robot_control__srv__ManageOperationLease_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!xczs_inspection_robot_control__srv__ManageOperationLease_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
