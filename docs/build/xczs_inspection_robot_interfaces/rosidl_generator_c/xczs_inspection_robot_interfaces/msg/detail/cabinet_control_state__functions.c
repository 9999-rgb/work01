// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from xczs_inspection_robot_interfaces:msg/CabinetControlState.idl
// generated code does not contain a copyright notice
#include "xczs_inspection_robot_interfaces/msg/detail/cabinet_control_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `control_id`
// Member `state_id`
#include "rosidl_runtime_c/string_functions.h"

bool
xczs_inspection_robot_interfaces__msg__CabinetControlState__init(xczs_inspection_robot_interfaces__msg__CabinetControlState * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    xczs_inspection_robot_interfaces__msg__CabinetControlState__fini(msg);
    return false;
  }
  // control_id
  if (!rosidl_runtime_c__String__init(&msg->control_id)) {
    xczs_inspection_robot_interfaces__msg__CabinetControlState__fini(msg);
    return false;
  }
  // control_type
  // valid
  // position
  // velocity
  // effort
  // normalized_position
  // state_id
  if (!rosidl_runtime_c__String__init(&msg->state_id)) {
    xczs_inspection_robot_interfaces__msg__CabinetControlState__fini(msg);
    return false;
  }
  // activated
  // in_motion
  // transition_sequence
  return true;
}

void
xczs_inspection_robot_interfaces__msg__CabinetControlState__fini(xczs_inspection_robot_interfaces__msg__CabinetControlState * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // control_id
  rosidl_runtime_c__String__fini(&msg->control_id);
  // control_type
  // valid
  // position
  // velocity
  // effort
  // normalized_position
  // state_id
  rosidl_runtime_c__String__fini(&msg->state_id);
  // activated
  // in_motion
  // transition_sequence
}

bool
xczs_inspection_robot_interfaces__msg__CabinetControlState__are_equal(const xczs_inspection_robot_interfaces__msg__CabinetControlState * lhs, const xczs_inspection_robot_interfaces__msg__CabinetControlState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // control_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->control_id), &(rhs->control_id)))
  {
    return false;
  }
  // control_type
  if (lhs->control_type != rhs->control_type) {
    return false;
  }
  // valid
  if (lhs->valid != rhs->valid) {
    return false;
  }
  // position
  if (lhs->position != rhs->position) {
    return false;
  }
  // velocity
  if (lhs->velocity != rhs->velocity) {
    return false;
  }
  // effort
  if (lhs->effort != rhs->effort) {
    return false;
  }
  // normalized_position
  if (lhs->normalized_position != rhs->normalized_position) {
    return false;
  }
  // state_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->state_id), &(rhs->state_id)))
  {
    return false;
  }
  // activated
  if (lhs->activated != rhs->activated) {
    return false;
  }
  // in_motion
  if (lhs->in_motion != rhs->in_motion) {
    return false;
  }
  // transition_sequence
  if (lhs->transition_sequence != rhs->transition_sequence) {
    return false;
  }
  return true;
}

bool
xczs_inspection_robot_interfaces__msg__CabinetControlState__copy(
  const xczs_inspection_robot_interfaces__msg__CabinetControlState * input,
  xczs_inspection_robot_interfaces__msg__CabinetControlState * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // control_id
  if (!rosidl_runtime_c__String__copy(
      &(input->control_id), &(output->control_id)))
  {
    return false;
  }
  // control_type
  output->control_type = input->control_type;
  // valid
  output->valid = input->valid;
  // position
  output->position = input->position;
  // velocity
  output->velocity = input->velocity;
  // effort
  output->effort = input->effort;
  // normalized_position
  output->normalized_position = input->normalized_position;
  // state_id
  if (!rosidl_runtime_c__String__copy(
      &(input->state_id), &(output->state_id)))
  {
    return false;
  }
  // activated
  output->activated = input->activated;
  // in_motion
  output->in_motion = input->in_motion;
  // transition_sequence
  output->transition_sequence = input->transition_sequence;
  return true;
}

xczs_inspection_robot_interfaces__msg__CabinetControlState *
xczs_inspection_robot_interfaces__msg__CabinetControlState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  xczs_inspection_robot_interfaces__msg__CabinetControlState * msg = (xczs_inspection_robot_interfaces__msg__CabinetControlState *)allocator.allocate(sizeof(xczs_inspection_robot_interfaces__msg__CabinetControlState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(xczs_inspection_robot_interfaces__msg__CabinetControlState));
  bool success = xczs_inspection_robot_interfaces__msg__CabinetControlState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
xczs_inspection_robot_interfaces__msg__CabinetControlState__destroy(xczs_inspection_robot_interfaces__msg__CabinetControlState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    xczs_inspection_robot_interfaces__msg__CabinetControlState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
xczs_inspection_robot_interfaces__msg__CabinetControlState__Sequence__init(xczs_inspection_robot_interfaces__msg__CabinetControlState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  xczs_inspection_robot_interfaces__msg__CabinetControlState * data = NULL;

  if (size) {
    data = (xczs_inspection_robot_interfaces__msg__CabinetControlState *)allocator.zero_allocate(size, sizeof(xczs_inspection_robot_interfaces__msg__CabinetControlState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = xczs_inspection_robot_interfaces__msg__CabinetControlState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        xczs_inspection_robot_interfaces__msg__CabinetControlState__fini(&data[i - 1]);
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
xczs_inspection_robot_interfaces__msg__CabinetControlState__Sequence__fini(xczs_inspection_robot_interfaces__msg__CabinetControlState__Sequence * array)
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
      xczs_inspection_robot_interfaces__msg__CabinetControlState__fini(&array->data[i]);
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

xczs_inspection_robot_interfaces__msg__CabinetControlState__Sequence *
xczs_inspection_robot_interfaces__msg__CabinetControlState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  xczs_inspection_robot_interfaces__msg__CabinetControlState__Sequence * array = (xczs_inspection_robot_interfaces__msg__CabinetControlState__Sequence *)allocator.allocate(sizeof(xczs_inspection_robot_interfaces__msg__CabinetControlState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = xczs_inspection_robot_interfaces__msg__CabinetControlState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
xczs_inspection_robot_interfaces__msg__CabinetControlState__Sequence__destroy(xczs_inspection_robot_interfaces__msg__CabinetControlState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    xczs_inspection_robot_interfaces__msg__CabinetControlState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
xczs_inspection_robot_interfaces__msg__CabinetControlState__Sequence__are_equal(const xczs_inspection_robot_interfaces__msg__CabinetControlState__Sequence * lhs, const xczs_inspection_robot_interfaces__msg__CabinetControlState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!xczs_inspection_robot_interfaces__msg__CabinetControlState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
xczs_inspection_robot_interfaces__msg__CabinetControlState__Sequence__copy(
  const xczs_inspection_robot_interfaces__msg__CabinetControlState__Sequence * input,
  xczs_inspection_robot_interfaces__msg__CabinetControlState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(xczs_inspection_robot_interfaces__msg__CabinetControlState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    xczs_inspection_robot_interfaces__msg__CabinetControlState * data =
      (xczs_inspection_robot_interfaces__msg__CabinetControlState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!xczs_inspection_robot_interfaces__msg__CabinetControlState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          xczs_inspection_robot_interfaces__msg__CabinetControlState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!xczs_inspection_robot_interfaces__msg__CabinetControlState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
