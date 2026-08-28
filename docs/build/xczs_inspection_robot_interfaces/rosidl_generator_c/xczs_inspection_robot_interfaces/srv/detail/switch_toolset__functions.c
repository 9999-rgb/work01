// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from xczs_inspection_robot_interfaces:srv/SwitchToolset.idl
// generated code does not contain a copyright notice
#include "xczs_inspection_robot_interfaces/srv/detail/switch_toolset__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `toolset`
#include "rosidl_runtime_c/string_functions.h"

bool
xczs_inspection_robot_interfaces__srv__SwitchToolset_Request__init(xczs_inspection_robot_interfaces__srv__SwitchToolset_Request * msg)
{
  if (!msg) {
    return false;
  }
  // toolset
  if (!rosidl_runtime_c__String__init(&msg->toolset)) {
    xczs_inspection_robot_interfaces__srv__SwitchToolset_Request__fini(msg);
    return false;
  }
  // expected_generation
  return true;
}

void
xczs_inspection_robot_interfaces__srv__SwitchToolset_Request__fini(xczs_inspection_robot_interfaces__srv__SwitchToolset_Request * msg)
{
  if (!msg) {
    return;
  }
  // toolset
  rosidl_runtime_c__String__fini(&msg->toolset);
  // expected_generation
}

bool
xczs_inspection_robot_interfaces__srv__SwitchToolset_Request__are_equal(const xczs_inspection_robot_interfaces__srv__SwitchToolset_Request * lhs, const xczs_inspection_robot_interfaces__srv__SwitchToolset_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // toolset
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->toolset), &(rhs->toolset)))
  {
    return false;
  }
  // expected_generation
  if (lhs->expected_generation != rhs->expected_generation) {
    return false;
  }
  return true;
}

bool
xczs_inspection_robot_interfaces__srv__SwitchToolset_Request__copy(
  const xczs_inspection_robot_interfaces__srv__SwitchToolset_Request * input,
  xczs_inspection_robot_interfaces__srv__SwitchToolset_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // toolset
  if (!rosidl_runtime_c__String__copy(
      &(input->toolset), &(output->toolset)))
  {
    return false;
  }
  // expected_generation
  output->expected_generation = input->expected_generation;
  return true;
}

xczs_inspection_robot_interfaces__srv__SwitchToolset_Request *
xczs_inspection_robot_interfaces__srv__SwitchToolset_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  xczs_inspection_robot_interfaces__srv__SwitchToolset_Request * msg = (xczs_inspection_robot_interfaces__srv__SwitchToolset_Request *)allocator.allocate(sizeof(xczs_inspection_robot_interfaces__srv__SwitchToolset_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(xczs_inspection_robot_interfaces__srv__SwitchToolset_Request));
  bool success = xczs_inspection_robot_interfaces__srv__SwitchToolset_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
xczs_inspection_robot_interfaces__srv__SwitchToolset_Request__destroy(xczs_inspection_robot_interfaces__srv__SwitchToolset_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    xczs_inspection_robot_interfaces__srv__SwitchToolset_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
xczs_inspection_robot_interfaces__srv__SwitchToolset_Request__Sequence__init(xczs_inspection_robot_interfaces__srv__SwitchToolset_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  xczs_inspection_robot_interfaces__srv__SwitchToolset_Request * data = NULL;

  if (size) {
    data = (xczs_inspection_robot_interfaces__srv__SwitchToolset_Request *)allocator.zero_allocate(size, sizeof(xczs_inspection_robot_interfaces__srv__SwitchToolset_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = xczs_inspection_robot_interfaces__srv__SwitchToolset_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        xczs_inspection_robot_interfaces__srv__SwitchToolset_Request__fini(&data[i - 1]);
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
xczs_inspection_robot_interfaces__srv__SwitchToolset_Request__Sequence__fini(xczs_inspection_robot_interfaces__srv__SwitchToolset_Request__Sequence * array)
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
      xczs_inspection_robot_interfaces__srv__SwitchToolset_Request__fini(&array->data[i]);
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

xczs_inspection_robot_interfaces__srv__SwitchToolset_Request__Sequence *
xczs_inspection_robot_interfaces__srv__SwitchToolset_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  xczs_inspection_robot_interfaces__srv__SwitchToolset_Request__Sequence * array = (xczs_inspection_robot_interfaces__srv__SwitchToolset_Request__Sequence *)allocator.allocate(sizeof(xczs_inspection_robot_interfaces__srv__SwitchToolset_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = xczs_inspection_robot_interfaces__srv__SwitchToolset_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
xczs_inspection_robot_interfaces__srv__SwitchToolset_Request__Sequence__destroy(xczs_inspection_robot_interfaces__srv__SwitchToolset_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    xczs_inspection_robot_interfaces__srv__SwitchToolset_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
xczs_inspection_robot_interfaces__srv__SwitchToolset_Request__Sequence__are_equal(const xczs_inspection_robot_interfaces__srv__SwitchToolset_Request__Sequence * lhs, const xczs_inspection_robot_interfaces__srv__SwitchToolset_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!xczs_inspection_robot_interfaces__srv__SwitchToolset_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
xczs_inspection_robot_interfaces__srv__SwitchToolset_Request__Sequence__copy(
  const xczs_inspection_robot_interfaces__srv__SwitchToolset_Request__Sequence * input,
  xczs_inspection_robot_interfaces__srv__SwitchToolset_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(xczs_inspection_robot_interfaces__srv__SwitchToolset_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    xczs_inspection_robot_interfaces__srv__SwitchToolset_Request * data =
      (xczs_inspection_robot_interfaces__srv__SwitchToolset_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!xczs_inspection_robot_interfaces__srv__SwitchToolset_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          xczs_inspection_robot_interfaces__srv__SwitchToolset_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!xczs_inspection_robot_interfaces__srv__SwitchToolset_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `state`
// Member `active_toolset`
// Member `target_toolset`
// Member `message`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

bool
xczs_inspection_robot_interfaces__srv__SwitchToolset_Response__init(xczs_inspection_robot_interfaces__srv__SwitchToolset_Response * msg)
{
  if (!msg) {
    return false;
  }
  // accepted
  // state
  if (!rosidl_runtime_c__String__init(&msg->state)) {
    xczs_inspection_robot_interfaces__srv__SwitchToolset_Response__fini(msg);
    return false;
  }
  // active_toolset
  if (!rosidl_runtime_c__String__init(&msg->active_toolset)) {
    xczs_inspection_robot_interfaces__srv__SwitchToolset_Response__fini(msg);
    return false;
  }
  // target_toolset
  if (!rosidl_runtime_c__String__init(&msg->target_toolset)) {
    xczs_inspection_robot_interfaces__srv__SwitchToolset_Response__fini(msg);
    return false;
  }
  // generation
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    xczs_inspection_robot_interfaces__srv__SwitchToolset_Response__fini(msg);
    return false;
  }
  return true;
}

void
xczs_inspection_robot_interfaces__srv__SwitchToolset_Response__fini(xczs_inspection_robot_interfaces__srv__SwitchToolset_Response * msg)
{
  if (!msg) {
    return;
  }
  // accepted
  // state
  rosidl_runtime_c__String__fini(&msg->state);
  // active_toolset
  rosidl_runtime_c__String__fini(&msg->active_toolset);
  // target_toolset
  rosidl_runtime_c__String__fini(&msg->target_toolset);
  // generation
  // message
  rosidl_runtime_c__String__fini(&msg->message);
}

bool
xczs_inspection_robot_interfaces__srv__SwitchToolset_Response__are_equal(const xczs_inspection_robot_interfaces__srv__SwitchToolset_Response * lhs, const xczs_inspection_robot_interfaces__srv__SwitchToolset_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // accepted
  if (lhs->accepted != rhs->accepted) {
    return false;
  }
  // state
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->state), &(rhs->state)))
  {
    return false;
  }
  // active_toolset
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->active_toolset), &(rhs->active_toolset)))
  {
    return false;
  }
  // target_toolset
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->target_toolset), &(rhs->target_toolset)))
  {
    return false;
  }
  // generation
  if (lhs->generation != rhs->generation) {
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->message), &(rhs->message)))
  {
    return false;
  }
  return true;
}

bool
xczs_inspection_robot_interfaces__srv__SwitchToolset_Response__copy(
  const xczs_inspection_robot_interfaces__srv__SwitchToolset_Response * input,
  xczs_inspection_robot_interfaces__srv__SwitchToolset_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // accepted
  output->accepted = input->accepted;
  // state
  if (!rosidl_runtime_c__String__copy(
      &(input->state), &(output->state)))
  {
    return false;
  }
  // active_toolset
  if (!rosidl_runtime_c__String__copy(
      &(input->active_toolset), &(output->active_toolset)))
  {
    return false;
  }
  // target_toolset
  if (!rosidl_runtime_c__String__copy(
      &(input->target_toolset), &(output->target_toolset)))
  {
    return false;
  }
  // generation
  output->generation = input->generation;
  // message
  if (!rosidl_runtime_c__String__copy(
      &(input->message), &(output->message)))
  {
    return false;
  }
  return true;
}

xczs_inspection_robot_interfaces__srv__SwitchToolset_Response *
xczs_inspection_robot_interfaces__srv__SwitchToolset_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  xczs_inspection_robot_interfaces__srv__SwitchToolset_Response * msg = (xczs_inspection_robot_interfaces__srv__SwitchToolset_Response *)allocator.allocate(sizeof(xczs_inspection_robot_interfaces__srv__SwitchToolset_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(xczs_inspection_robot_interfaces__srv__SwitchToolset_Response));
  bool success = xczs_inspection_robot_interfaces__srv__SwitchToolset_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
xczs_inspection_robot_interfaces__srv__SwitchToolset_Response__destroy(xczs_inspection_robot_interfaces__srv__SwitchToolset_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    xczs_inspection_robot_interfaces__srv__SwitchToolset_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
xczs_inspection_robot_interfaces__srv__SwitchToolset_Response__Sequence__init(xczs_inspection_robot_interfaces__srv__SwitchToolset_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  xczs_inspection_robot_interfaces__srv__SwitchToolset_Response * data = NULL;

  if (size) {
    data = (xczs_inspection_robot_interfaces__srv__SwitchToolset_Response *)allocator.zero_allocate(size, sizeof(xczs_inspection_robot_interfaces__srv__SwitchToolset_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = xczs_inspection_robot_interfaces__srv__SwitchToolset_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        xczs_inspection_robot_interfaces__srv__SwitchToolset_Response__fini(&data[i - 1]);
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
xczs_inspection_robot_interfaces__srv__SwitchToolset_Response__Sequence__fini(xczs_inspection_robot_interfaces__srv__SwitchToolset_Response__Sequence * array)
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
      xczs_inspection_robot_interfaces__srv__SwitchToolset_Response__fini(&array->data[i]);
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

xczs_inspection_robot_interfaces__srv__SwitchToolset_Response__Sequence *
xczs_inspection_robot_interfaces__srv__SwitchToolset_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  xczs_inspection_robot_interfaces__srv__SwitchToolset_Response__Sequence * array = (xczs_inspection_robot_interfaces__srv__SwitchToolset_Response__Sequence *)allocator.allocate(sizeof(xczs_inspection_robot_interfaces__srv__SwitchToolset_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = xczs_inspection_robot_interfaces__srv__SwitchToolset_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
xczs_inspection_robot_interfaces__srv__SwitchToolset_Response__Sequence__destroy(xczs_inspection_robot_interfaces__srv__SwitchToolset_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    xczs_inspection_robot_interfaces__srv__SwitchToolset_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
xczs_inspection_robot_interfaces__srv__SwitchToolset_Response__Sequence__are_equal(const xczs_inspection_robot_interfaces__srv__SwitchToolset_Response__Sequence * lhs, const xczs_inspection_robot_interfaces__srv__SwitchToolset_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!xczs_inspection_robot_interfaces__srv__SwitchToolset_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
xczs_inspection_robot_interfaces__srv__SwitchToolset_Response__Sequence__copy(
  const xczs_inspection_robot_interfaces__srv__SwitchToolset_Response__Sequence * input,
  xczs_inspection_robot_interfaces__srv__SwitchToolset_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(xczs_inspection_robot_interfaces__srv__SwitchToolset_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    xczs_inspection_robot_interfaces__srv__SwitchToolset_Response * data =
      (xczs_inspection_robot_interfaces__srv__SwitchToolset_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!xczs_inspection_robot_interfaces__srv__SwitchToolset_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          xczs_inspection_robot_interfaces__srv__SwitchToolset_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!xczs_inspection_robot_interfaces__srv__SwitchToolset_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
