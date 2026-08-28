// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from xczs_inspection_robot_control:msg/CabinetControl.idl
// generated code does not contain a copyright notice
#include "xczs_inspection_robot_control/msg/detail/cabinet_control__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `control_id`
// Member `display_name`
// Member `joint_name`
// Member `joint_state_topic`
// Member `pressed_topic`
// Member `state_topic`
// Member `unit`
// Member `state_ids`
// Member `state_labels`
// Member `unavailable_reason`
// Member `required_toolset`
#include "rosidl_runtime_c/string_functions.h"
// Member `state_positions`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
xczs_inspection_robot_control__msg__CabinetControl__init(xczs_inspection_robot_control__msg__CabinetControl * msg)
{
  if (!msg) {
    return false;
  }
  // control_id
  if (!rosidl_runtime_c__String__init(&msg->control_id)) {
    xczs_inspection_robot_control__msg__CabinetControl__fini(msg);
    return false;
  }
  // display_name
  if (!rosidl_runtime_c__String__init(&msg->display_name)) {
    xczs_inspection_robot_control__msg__CabinetControl__fini(msg);
    return false;
  }
  // control_type
  // joint_name
  if (!rosidl_runtime_c__String__init(&msg->joint_name)) {
    xczs_inspection_robot_control__msg__CabinetControl__fini(msg);
    return false;
  }
  // joint_state_topic
  if (!rosidl_runtime_c__String__init(&msg->joint_state_topic)) {
    xczs_inspection_robot_control__msg__CabinetControl__fini(msg);
    return false;
  }
  // pressed_topic
  if (!rosidl_runtime_c__String__init(&msg->pressed_topic)) {
    xczs_inspection_robot_control__msg__CabinetControl__fini(msg);
    return false;
  }
  // state_topic
  if (!rosidl_runtime_c__String__init(&msg->state_topic)) {
    xczs_inspection_robot_control__msg__CabinetControl__fini(msg);
    return false;
  }
  // supported_commands
  // unit
  if (!rosidl_runtime_c__String__init(&msg->unit)) {
    xczs_inspection_robot_control__msg__CabinetControl__fini(msg);
    return false;
  }
  // min_position
  // max_position
  // state_ids
  if (!rosidl_runtime_c__String__Sequence__init(&msg->state_ids, 0)) {
    xczs_inspection_robot_control__msg__CabinetControl__fini(msg);
    return false;
  }
  // state_labels
  if (!rosidl_runtime_c__String__Sequence__init(&msg->state_labels, 0)) {
    xczs_inspection_robot_control__msg__CabinetControl__fini(msg);
    return false;
  }
  // state_positions
  if (!rosidl_runtime_c__double__Sequence__init(&msg->state_positions, 0)) {
    xczs_inspection_robot_control__msg__CabinetControl__fini(msg);
    return false;
  }
  // requires_grasp
  // operable
  // unavailable_reason
  if (!rosidl_runtime_c__String__init(&msg->unavailable_reason)) {
    xczs_inspection_robot_control__msg__CabinetControl__fini(msg);
    return false;
  }
  // required_toolset
  if (!rosidl_runtime_c__String__init(&msg->required_toolset)) {
    xczs_inspection_robot_control__msg__CabinetControl__fini(msg);
    return false;
  }
  // toolset_compatible
  // adapter_validated
  // default_force
  // min_trigger_force
  // max_force
  return true;
}

void
xczs_inspection_robot_control__msg__CabinetControl__fini(xczs_inspection_robot_control__msg__CabinetControl * msg)
{
  if (!msg) {
    return;
  }
  // control_id
  rosidl_runtime_c__String__fini(&msg->control_id);
  // display_name
  rosidl_runtime_c__String__fini(&msg->display_name);
  // control_type
  // joint_name
  rosidl_runtime_c__String__fini(&msg->joint_name);
  // joint_state_topic
  rosidl_runtime_c__String__fini(&msg->joint_state_topic);
  // pressed_topic
  rosidl_runtime_c__String__fini(&msg->pressed_topic);
  // state_topic
  rosidl_runtime_c__String__fini(&msg->state_topic);
  // supported_commands
  // unit
  rosidl_runtime_c__String__fini(&msg->unit);
  // min_position
  // max_position
  // state_ids
  rosidl_runtime_c__String__Sequence__fini(&msg->state_ids);
  // state_labels
  rosidl_runtime_c__String__Sequence__fini(&msg->state_labels);
  // state_positions
  rosidl_runtime_c__double__Sequence__fini(&msg->state_positions);
  // requires_grasp
  // operable
  // unavailable_reason
  rosidl_runtime_c__String__fini(&msg->unavailable_reason);
  // required_toolset
  rosidl_runtime_c__String__fini(&msg->required_toolset);
  // toolset_compatible
  // adapter_validated
  // default_force
  // min_trigger_force
  // max_force
}

bool
xczs_inspection_robot_control__msg__CabinetControl__are_equal(const xczs_inspection_robot_control__msg__CabinetControl * lhs, const xczs_inspection_robot_control__msg__CabinetControl * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // control_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->control_id), &(rhs->control_id)))
  {
    return false;
  }
  // display_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->display_name), &(rhs->display_name)))
  {
    return false;
  }
  // control_type
  if (lhs->control_type != rhs->control_type) {
    return false;
  }
  // joint_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->joint_name), &(rhs->joint_name)))
  {
    return false;
  }
  // joint_state_topic
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->joint_state_topic), &(rhs->joint_state_topic)))
  {
    return false;
  }
  // pressed_topic
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->pressed_topic), &(rhs->pressed_topic)))
  {
    return false;
  }
  // state_topic
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->state_topic), &(rhs->state_topic)))
  {
    return false;
  }
  // supported_commands
  if (lhs->supported_commands != rhs->supported_commands) {
    return false;
  }
  // unit
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->unit), &(rhs->unit)))
  {
    return false;
  }
  // min_position
  if (lhs->min_position != rhs->min_position) {
    return false;
  }
  // max_position
  if (lhs->max_position != rhs->max_position) {
    return false;
  }
  // state_ids
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->state_ids), &(rhs->state_ids)))
  {
    return false;
  }
  // state_labels
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->state_labels), &(rhs->state_labels)))
  {
    return false;
  }
  // state_positions
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->state_positions), &(rhs->state_positions)))
  {
    return false;
  }
  // requires_grasp
  if (lhs->requires_grasp != rhs->requires_grasp) {
    return false;
  }
  // operable
  if (lhs->operable != rhs->operable) {
    return false;
  }
  // unavailable_reason
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->unavailable_reason), &(rhs->unavailable_reason)))
  {
    return false;
  }
  // required_toolset
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->required_toolset), &(rhs->required_toolset)))
  {
    return false;
  }
  // toolset_compatible
  if (lhs->toolset_compatible != rhs->toolset_compatible) {
    return false;
  }
  // adapter_validated
  if (lhs->adapter_validated != rhs->adapter_validated) {
    return false;
  }
  // default_force
  if (lhs->default_force != rhs->default_force) {
    return false;
  }
  // min_trigger_force
  if (lhs->min_trigger_force != rhs->min_trigger_force) {
    return false;
  }
  // max_force
  if (lhs->max_force != rhs->max_force) {
    return false;
  }
  return true;
}

bool
xczs_inspection_robot_control__msg__CabinetControl__copy(
  const xczs_inspection_robot_control__msg__CabinetControl * input,
  xczs_inspection_robot_control__msg__CabinetControl * output)
{
  if (!input || !output) {
    return false;
  }
  // control_id
  if (!rosidl_runtime_c__String__copy(
      &(input->control_id), &(output->control_id)))
  {
    return false;
  }
  // display_name
  if (!rosidl_runtime_c__String__copy(
      &(input->display_name), &(output->display_name)))
  {
    return false;
  }
  // control_type
  output->control_type = input->control_type;
  // joint_name
  if (!rosidl_runtime_c__String__copy(
      &(input->joint_name), &(output->joint_name)))
  {
    return false;
  }
  // joint_state_topic
  if (!rosidl_runtime_c__String__copy(
      &(input->joint_state_topic), &(output->joint_state_topic)))
  {
    return false;
  }
  // pressed_topic
  if (!rosidl_runtime_c__String__copy(
      &(input->pressed_topic), &(output->pressed_topic)))
  {
    return false;
  }
  // state_topic
  if (!rosidl_runtime_c__String__copy(
      &(input->state_topic), &(output->state_topic)))
  {
    return false;
  }
  // supported_commands
  output->supported_commands = input->supported_commands;
  // unit
  if (!rosidl_runtime_c__String__copy(
      &(input->unit), &(output->unit)))
  {
    return false;
  }
  // min_position
  output->min_position = input->min_position;
  // max_position
  output->max_position = input->max_position;
  // state_ids
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->state_ids), &(output->state_ids)))
  {
    return false;
  }
  // state_labels
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->state_labels), &(output->state_labels)))
  {
    return false;
  }
  // state_positions
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->state_positions), &(output->state_positions)))
  {
    return false;
  }
  // requires_grasp
  output->requires_grasp = input->requires_grasp;
  // operable
  output->operable = input->operable;
  // unavailable_reason
  if (!rosidl_runtime_c__String__copy(
      &(input->unavailable_reason), &(output->unavailable_reason)))
  {
    return false;
  }
  // required_toolset
  if (!rosidl_runtime_c__String__copy(
      &(input->required_toolset), &(output->required_toolset)))
  {
    return false;
  }
  // toolset_compatible
  output->toolset_compatible = input->toolset_compatible;
  // adapter_validated
  output->adapter_validated = input->adapter_validated;
  // default_force
  output->default_force = input->default_force;
  // min_trigger_force
  output->min_trigger_force = input->min_trigger_force;
  // max_force
  output->max_force = input->max_force;
  return true;
}

xczs_inspection_robot_control__msg__CabinetControl *
xczs_inspection_robot_control__msg__CabinetControl__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  xczs_inspection_robot_control__msg__CabinetControl * msg = (xczs_inspection_robot_control__msg__CabinetControl *)allocator.allocate(sizeof(xczs_inspection_robot_control__msg__CabinetControl), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(xczs_inspection_robot_control__msg__CabinetControl));
  bool success = xczs_inspection_robot_control__msg__CabinetControl__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
xczs_inspection_robot_control__msg__CabinetControl__destroy(xczs_inspection_robot_control__msg__CabinetControl * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    xczs_inspection_robot_control__msg__CabinetControl__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
xczs_inspection_robot_control__msg__CabinetControl__Sequence__init(xczs_inspection_robot_control__msg__CabinetControl__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  xczs_inspection_robot_control__msg__CabinetControl * data = NULL;

  if (size) {
    data = (xczs_inspection_robot_control__msg__CabinetControl *)allocator.zero_allocate(size, sizeof(xczs_inspection_robot_control__msg__CabinetControl), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = xczs_inspection_robot_control__msg__CabinetControl__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        xczs_inspection_robot_control__msg__CabinetControl__fini(&data[i - 1]);
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
xczs_inspection_robot_control__msg__CabinetControl__Sequence__fini(xczs_inspection_robot_control__msg__CabinetControl__Sequence * array)
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
      xczs_inspection_robot_control__msg__CabinetControl__fini(&array->data[i]);
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

xczs_inspection_robot_control__msg__CabinetControl__Sequence *
xczs_inspection_robot_control__msg__CabinetControl__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  xczs_inspection_robot_control__msg__CabinetControl__Sequence * array = (xczs_inspection_robot_control__msg__CabinetControl__Sequence *)allocator.allocate(sizeof(xczs_inspection_robot_control__msg__CabinetControl__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = xczs_inspection_robot_control__msg__CabinetControl__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
xczs_inspection_robot_control__msg__CabinetControl__Sequence__destroy(xczs_inspection_robot_control__msg__CabinetControl__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    xczs_inspection_robot_control__msg__CabinetControl__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
xczs_inspection_robot_control__msg__CabinetControl__Sequence__are_equal(const xczs_inspection_robot_control__msg__CabinetControl__Sequence * lhs, const xczs_inspection_robot_control__msg__CabinetControl__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!xczs_inspection_robot_control__msg__CabinetControl__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
xczs_inspection_robot_control__msg__CabinetControl__Sequence__copy(
  const xczs_inspection_robot_control__msg__CabinetControl__Sequence * input,
  xczs_inspection_robot_control__msg__CabinetControl__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(xczs_inspection_robot_control__msg__CabinetControl);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    xczs_inspection_robot_control__msg__CabinetControl * data =
      (xczs_inspection_robot_control__msg__CabinetControl *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!xczs_inspection_robot_control__msg__CabinetControl__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          xczs_inspection_robot_control__msg__CabinetControl__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!xczs_inspection_robot_control__msg__CabinetControl__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
