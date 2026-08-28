// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from xczs_inspection_robot_control:action/OperateCabinetControl.idl
// generated code does not contain a copyright notice
#include "xczs_inspection_robot_control/action/detail/operate_cabinet_control__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `control_id`
// Member `target_state`
#include "rosidl_runtime_c/string_functions.h"

bool
xczs_inspection_robot_control__action__OperateCabinetControl_Goal__init(xczs_inspection_robot_control__action__OperateCabinetControl_Goal * msg)
{
  if (!msg) {
    return false;
  }
  // control_id
  if (!rosidl_runtime_c__String__init(&msg->control_id)) {
    xczs_inspection_robot_control__action__OperateCabinetControl_Goal__fini(msg);
    return false;
  }
  // command
  // target_state
  if (!rosidl_runtime_c__String__init(&msg->target_state)) {
    xczs_inspection_robot_control__action__OperateCabinetControl_Goal__fini(msg);
    return false;
  }
  // target_position
  // use_target_position
  // force
  // navigate_to_staging_pose
  return true;
}

void
xczs_inspection_robot_control__action__OperateCabinetControl_Goal__fini(xczs_inspection_robot_control__action__OperateCabinetControl_Goal * msg)
{
  if (!msg) {
    return;
  }
  // control_id
  rosidl_runtime_c__String__fini(&msg->control_id);
  // command
  // target_state
  rosidl_runtime_c__String__fini(&msg->target_state);
  // target_position
  // use_target_position
  // force
  // navigate_to_staging_pose
}

bool
xczs_inspection_robot_control__action__OperateCabinetControl_Goal__are_equal(const xczs_inspection_robot_control__action__OperateCabinetControl_Goal * lhs, const xczs_inspection_robot_control__action__OperateCabinetControl_Goal * rhs)
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
  // command
  if (lhs->command != rhs->command) {
    return false;
  }
  // target_state
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->target_state), &(rhs->target_state)))
  {
    return false;
  }
  // target_position
  if (lhs->target_position != rhs->target_position) {
    return false;
  }
  // use_target_position
  if (lhs->use_target_position != rhs->use_target_position) {
    return false;
  }
  // force
  if (lhs->force != rhs->force) {
    return false;
  }
  // navigate_to_staging_pose
  if (lhs->navigate_to_staging_pose != rhs->navigate_to_staging_pose) {
    return false;
  }
  return true;
}

bool
xczs_inspection_robot_control__action__OperateCabinetControl_Goal__copy(
  const xczs_inspection_robot_control__action__OperateCabinetControl_Goal * input,
  xczs_inspection_robot_control__action__OperateCabinetControl_Goal * output)
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
  // command
  output->command = input->command;
  // target_state
  if (!rosidl_runtime_c__String__copy(
      &(input->target_state), &(output->target_state)))
  {
    return false;
  }
  // target_position
  output->target_position = input->target_position;
  // use_target_position
  output->use_target_position = input->use_target_position;
  // force
  output->force = input->force;
  // navigate_to_staging_pose
  output->navigate_to_staging_pose = input->navigate_to_staging_pose;
  return true;
}

xczs_inspection_robot_control__action__OperateCabinetControl_Goal *
xczs_inspection_robot_control__action__OperateCabinetControl_Goal__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  xczs_inspection_robot_control__action__OperateCabinetControl_Goal * msg = (xczs_inspection_robot_control__action__OperateCabinetControl_Goal *)allocator.allocate(sizeof(xczs_inspection_robot_control__action__OperateCabinetControl_Goal), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(xczs_inspection_robot_control__action__OperateCabinetControl_Goal));
  bool success = xczs_inspection_robot_control__action__OperateCabinetControl_Goal__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
xczs_inspection_robot_control__action__OperateCabinetControl_Goal__destroy(xczs_inspection_robot_control__action__OperateCabinetControl_Goal * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    xczs_inspection_robot_control__action__OperateCabinetControl_Goal__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
xczs_inspection_robot_control__action__OperateCabinetControl_Goal__Sequence__init(xczs_inspection_robot_control__action__OperateCabinetControl_Goal__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  xczs_inspection_robot_control__action__OperateCabinetControl_Goal * data = NULL;

  if (size) {
    data = (xczs_inspection_robot_control__action__OperateCabinetControl_Goal *)allocator.zero_allocate(size, sizeof(xczs_inspection_robot_control__action__OperateCabinetControl_Goal), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = xczs_inspection_robot_control__action__OperateCabinetControl_Goal__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        xczs_inspection_robot_control__action__OperateCabinetControl_Goal__fini(&data[i - 1]);
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
xczs_inspection_robot_control__action__OperateCabinetControl_Goal__Sequence__fini(xczs_inspection_robot_control__action__OperateCabinetControl_Goal__Sequence * array)
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
      xczs_inspection_robot_control__action__OperateCabinetControl_Goal__fini(&array->data[i]);
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

xczs_inspection_robot_control__action__OperateCabinetControl_Goal__Sequence *
xczs_inspection_robot_control__action__OperateCabinetControl_Goal__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  xczs_inspection_robot_control__action__OperateCabinetControl_Goal__Sequence * array = (xczs_inspection_robot_control__action__OperateCabinetControl_Goal__Sequence *)allocator.allocate(sizeof(xczs_inspection_robot_control__action__OperateCabinetControl_Goal__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = xczs_inspection_robot_control__action__OperateCabinetControl_Goal__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
xczs_inspection_robot_control__action__OperateCabinetControl_Goal__Sequence__destroy(xczs_inspection_robot_control__action__OperateCabinetControl_Goal__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    xczs_inspection_robot_control__action__OperateCabinetControl_Goal__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
xczs_inspection_robot_control__action__OperateCabinetControl_Goal__Sequence__are_equal(const xczs_inspection_robot_control__action__OperateCabinetControl_Goal__Sequence * lhs, const xczs_inspection_robot_control__action__OperateCabinetControl_Goal__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!xczs_inspection_robot_control__action__OperateCabinetControl_Goal__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
xczs_inspection_robot_control__action__OperateCabinetControl_Goal__Sequence__copy(
  const xczs_inspection_robot_control__action__OperateCabinetControl_Goal__Sequence * input,
  xczs_inspection_robot_control__action__OperateCabinetControl_Goal__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(xczs_inspection_robot_control__action__OperateCabinetControl_Goal);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    xczs_inspection_robot_control__action__OperateCabinetControl_Goal * data =
      (xczs_inspection_robot_control__action__OperateCabinetControl_Goal *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!xczs_inspection_robot_control__action__OperateCabinetControl_Goal__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          xczs_inspection_robot_control__action__OperateCabinetControl_Goal__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!xczs_inspection_robot_control__action__OperateCabinetControl_Goal__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `message`
// Member `final_state`
// Member `diagnostic_stage`
// Member `policy_reason`
// Member `failure_reason`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

bool
xczs_inspection_robot_control__action__OperateCabinetControl_Result__init(xczs_inspection_robot_control__action__OperateCabinetControl_Result * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // error_code
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    xczs_inspection_robot_control__action__OperateCabinetControl_Result__fini(msg);
    return false;
  }
  // initial_position
  // final_position
  // peak_position
  // final_state
  if (!rosidl_runtime_c__String__init(&msg->final_state)) {
    xczs_inspection_robot_control__action__OperateCabinetControl_Result__fini(msg);
    return false;
  }
  // requested_force
  // estimated_force
  // button_triggered
  // validation_performed
  // operation_executed
  // diagnostic_stage
  if (!rosidl_runtime_c__String__init(&msg->diagnostic_stage)) {
    xczs_inspection_robot_control__action__OperateCabinetControl_Result__fini(msg);
    return false;
  }
  // path_fraction
  // required_fraction
  // moveit_error_code
  // policy_reason
  if (!rosidl_runtime_c__String__init(&msg->policy_reason)) {
    xczs_inspection_robot_control__action__OperateCabinetControl_Result__fini(msg);
    return false;
  }
  // failure_reason
  if (!rosidl_runtime_c__String__init(&msg->failure_reason)) {
    xczs_inspection_robot_control__action__OperateCabinetControl_Result__fini(msg);
    return false;
  }
  // physical_outcome_confirmed
  // final_state_verified
  // transport_succeeded
  // recovery_succeeded
  // grasp_released
  return true;
}

void
xczs_inspection_robot_control__action__OperateCabinetControl_Result__fini(xczs_inspection_robot_control__action__OperateCabinetControl_Result * msg)
{
  if (!msg) {
    return;
  }
  // success
  // error_code
  // message
  rosidl_runtime_c__String__fini(&msg->message);
  // initial_position
  // final_position
  // peak_position
  // final_state
  rosidl_runtime_c__String__fini(&msg->final_state);
  // requested_force
  // estimated_force
  // button_triggered
  // validation_performed
  // operation_executed
  // diagnostic_stage
  rosidl_runtime_c__String__fini(&msg->diagnostic_stage);
  // path_fraction
  // required_fraction
  // moveit_error_code
  // policy_reason
  rosidl_runtime_c__String__fini(&msg->policy_reason);
  // failure_reason
  rosidl_runtime_c__String__fini(&msg->failure_reason);
  // physical_outcome_confirmed
  // final_state_verified
  // transport_succeeded
  // recovery_succeeded
  // grasp_released
}

bool
xczs_inspection_robot_control__action__OperateCabinetControl_Result__are_equal(const xczs_inspection_robot_control__action__OperateCabinetControl_Result * lhs, const xczs_inspection_robot_control__action__OperateCabinetControl_Result * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // error_code
  if (lhs->error_code != rhs->error_code) {
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->message), &(rhs->message)))
  {
    return false;
  }
  // initial_position
  if (lhs->initial_position != rhs->initial_position) {
    return false;
  }
  // final_position
  if (lhs->final_position != rhs->final_position) {
    return false;
  }
  // peak_position
  if (lhs->peak_position != rhs->peak_position) {
    return false;
  }
  // final_state
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->final_state), &(rhs->final_state)))
  {
    return false;
  }
  // requested_force
  if (lhs->requested_force != rhs->requested_force) {
    return false;
  }
  // estimated_force
  if (lhs->estimated_force != rhs->estimated_force) {
    return false;
  }
  // button_triggered
  if (lhs->button_triggered != rhs->button_triggered) {
    return false;
  }
  // validation_performed
  if (lhs->validation_performed != rhs->validation_performed) {
    return false;
  }
  // operation_executed
  if (lhs->operation_executed != rhs->operation_executed) {
    return false;
  }
  // diagnostic_stage
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->diagnostic_stage), &(rhs->diagnostic_stage)))
  {
    return false;
  }
  // path_fraction
  if (lhs->path_fraction != rhs->path_fraction) {
    return false;
  }
  // required_fraction
  if (lhs->required_fraction != rhs->required_fraction) {
    return false;
  }
  // moveit_error_code
  if (lhs->moveit_error_code != rhs->moveit_error_code) {
    return false;
  }
  // policy_reason
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->policy_reason), &(rhs->policy_reason)))
  {
    return false;
  }
  // failure_reason
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->failure_reason), &(rhs->failure_reason)))
  {
    return false;
  }
  // physical_outcome_confirmed
  if (lhs->physical_outcome_confirmed != rhs->physical_outcome_confirmed) {
    return false;
  }
  // final_state_verified
  if (lhs->final_state_verified != rhs->final_state_verified) {
    return false;
  }
  // transport_succeeded
  if (lhs->transport_succeeded != rhs->transport_succeeded) {
    return false;
  }
  // recovery_succeeded
  if (lhs->recovery_succeeded != rhs->recovery_succeeded) {
    return false;
  }
  // grasp_released
  if (lhs->grasp_released != rhs->grasp_released) {
    return false;
  }
  return true;
}

bool
xczs_inspection_robot_control__action__OperateCabinetControl_Result__copy(
  const xczs_inspection_robot_control__action__OperateCabinetControl_Result * input,
  xczs_inspection_robot_control__action__OperateCabinetControl_Result * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // error_code
  output->error_code = input->error_code;
  // message
  if (!rosidl_runtime_c__String__copy(
      &(input->message), &(output->message)))
  {
    return false;
  }
  // initial_position
  output->initial_position = input->initial_position;
  // final_position
  output->final_position = input->final_position;
  // peak_position
  output->peak_position = input->peak_position;
  // final_state
  if (!rosidl_runtime_c__String__copy(
      &(input->final_state), &(output->final_state)))
  {
    return false;
  }
  // requested_force
  output->requested_force = input->requested_force;
  // estimated_force
  output->estimated_force = input->estimated_force;
  // button_triggered
  output->button_triggered = input->button_triggered;
  // validation_performed
  output->validation_performed = input->validation_performed;
  // operation_executed
  output->operation_executed = input->operation_executed;
  // diagnostic_stage
  if (!rosidl_runtime_c__String__copy(
      &(input->diagnostic_stage), &(output->diagnostic_stage)))
  {
    return false;
  }
  // path_fraction
  output->path_fraction = input->path_fraction;
  // required_fraction
  output->required_fraction = input->required_fraction;
  // moveit_error_code
  output->moveit_error_code = input->moveit_error_code;
  // policy_reason
  if (!rosidl_runtime_c__String__copy(
      &(input->policy_reason), &(output->policy_reason)))
  {
    return false;
  }
  // failure_reason
  if (!rosidl_runtime_c__String__copy(
      &(input->failure_reason), &(output->failure_reason)))
  {
    return false;
  }
  // physical_outcome_confirmed
  output->physical_outcome_confirmed = input->physical_outcome_confirmed;
  // final_state_verified
  output->final_state_verified = input->final_state_verified;
  // transport_succeeded
  output->transport_succeeded = input->transport_succeeded;
  // recovery_succeeded
  output->recovery_succeeded = input->recovery_succeeded;
  // grasp_released
  output->grasp_released = input->grasp_released;
  return true;
}

xczs_inspection_robot_control__action__OperateCabinetControl_Result *
xczs_inspection_robot_control__action__OperateCabinetControl_Result__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  xczs_inspection_robot_control__action__OperateCabinetControl_Result * msg = (xczs_inspection_robot_control__action__OperateCabinetControl_Result *)allocator.allocate(sizeof(xczs_inspection_robot_control__action__OperateCabinetControl_Result), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(xczs_inspection_robot_control__action__OperateCabinetControl_Result));
  bool success = xczs_inspection_robot_control__action__OperateCabinetControl_Result__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
xczs_inspection_robot_control__action__OperateCabinetControl_Result__destroy(xczs_inspection_robot_control__action__OperateCabinetControl_Result * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    xczs_inspection_robot_control__action__OperateCabinetControl_Result__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
xczs_inspection_robot_control__action__OperateCabinetControl_Result__Sequence__init(xczs_inspection_robot_control__action__OperateCabinetControl_Result__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  xczs_inspection_robot_control__action__OperateCabinetControl_Result * data = NULL;

  if (size) {
    data = (xczs_inspection_robot_control__action__OperateCabinetControl_Result *)allocator.zero_allocate(size, sizeof(xczs_inspection_robot_control__action__OperateCabinetControl_Result), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = xczs_inspection_robot_control__action__OperateCabinetControl_Result__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        xczs_inspection_robot_control__action__OperateCabinetControl_Result__fini(&data[i - 1]);
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
xczs_inspection_robot_control__action__OperateCabinetControl_Result__Sequence__fini(xczs_inspection_robot_control__action__OperateCabinetControl_Result__Sequence * array)
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
      xczs_inspection_robot_control__action__OperateCabinetControl_Result__fini(&array->data[i]);
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

xczs_inspection_robot_control__action__OperateCabinetControl_Result__Sequence *
xczs_inspection_robot_control__action__OperateCabinetControl_Result__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  xczs_inspection_robot_control__action__OperateCabinetControl_Result__Sequence * array = (xczs_inspection_robot_control__action__OperateCabinetControl_Result__Sequence *)allocator.allocate(sizeof(xczs_inspection_robot_control__action__OperateCabinetControl_Result__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = xczs_inspection_robot_control__action__OperateCabinetControl_Result__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
xczs_inspection_robot_control__action__OperateCabinetControl_Result__Sequence__destroy(xczs_inspection_robot_control__action__OperateCabinetControl_Result__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    xczs_inspection_robot_control__action__OperateCabinetControl_Result__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
xczs_inspection_robot_control__action__OperateCabinetControl_Result__Sequence__are_equal(const xczs_inspection_robot_control__action__OperateCabinetControl_Result__Sequence * lhs, const xczs_inspection_robot_control__action__OperateCabinetControl_Result__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!xczs_inspection_robot_control__action__OperateCabinetControl_Result__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
xczs_inspection_robot_control__action__OperateCabinetControl_Result__Sequence__copy(
  const xczs_inspection_robot_control__action__OperateCabinetControl_Result__Sequence * input,
  xczs_inspection_robot_control__action__OperateCabinetControl_Result__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(xczs_inspection_robot_control__action__OperateCabinetControl_Result);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    xczs_inspection_robot_control__action__OperateCabinetControl_Result * data =
      (xczs_inspection_robot_control__action__OperateCabinetControl_Result *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!xczs_inspection_robot_control__action__OperateCabinetControl_Result__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          xczs_inspection_robot_control__action__OperateCabinetControl_Result__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!xczs_inspection_robot_control__action__OperateCabinetControl_Result__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `current_state`
// Member `message`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

bool
xczs_inspection_robot_control__action__OperateCabinetControl_Feedback__init(xczs_inspection_robot_control__action__OperateCabinetControl_Feedback * msg)
{
  if (!msg) {
    return false;
  }
  // phase
  // progress
  // current_position
  // target_position
  // current_state
  if (!rosidl_runtime_c__String__init(&msg->current_state)) {
    xczs_inspection_robot_control__action__OperateCabinetControl_Feedback__fini(msg);
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    xczs_inspection_robot_control__action__OperateCabinetControl_Feedback__fini(msg);
    return false;
  }
  return true;
}

void
xczs_inspection_robot_control__action__OperateCabinetControl_Feedback__fini(xczs_inspection_robot_control__action__OperateCabinetControl_Feedback * msg)
{
  if (!msg) {
    return;
  }
  // phase
  // progress
  // current_position
  // target_position
  // current_state
  rosidl_runtime_c__String__fini(&msg->current_state);
  // message
  rosidl_runtime_c__String__fini(&msg->message);
}

bool
xczs_inspection_robot_control__action__OperateCabinetControl_Feedback__are_equal(const xczs_inspection_robot_control__action__OperateCabinetControl_Feedback * lhs, const xczs_inspection_robot_control__action__OperateCabinetControl_Feedback * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // phase
  if (lhs->phase != rhs->phase) {
    return false;
  }
  // progress
  if (lhs->progress != rhs->progress) {
    return false;
  }
  // current_position
  if (lhs->current_position != rhs->current_position) {
    return false;
  }
  // target_position
  if (lhs->target_position != rhs->target_position) {
    return false;
  }
  // current_state
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->current_state), &(rhs->current_state)))
  {
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
xczs_inspection_robot_control__action__OperateCabinetControl_Feedback__copy(
  const xczs_inspection_robot_control__action__OperateCabinetControl_Feedback * input,
  xczs_inspection_robot_control__action__OperateCabinetControl_Feedback * output)
{
  if (!input || !output) {
    return false;
  }
  // phase
  output->phase = input->phase;
  // progress
  output->progress = input->progress;
  // current_position
  output->current_position = input->current_position;
  // target_position
  output->target_position = input->target_position;
  // current_state
  if (!rosidl_runtime_c__String__copy(
      &(input->current_state), &(output->current_state)))
  {
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__copy(
      &(input->message), &(output->message)))
  {
    return false;
  }
  return true;
}

xczs_inspection_robot_control__action__OperateCabinetControl_Feedback *
xczs_inspection_robot_control__action__OperateCabinetControl_Feedback__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  xczs_inspection_robot_control__action__OperateCabinetControl_Feedback * msg = (xczs_inspection_robot_control__action__OperateCabinetControl_Feedback *)allocator.allocate(sizeof(xczs_inspection_robot_control__action__OperateCabinetControl_Feedback), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(xczs_inspection_robot_control__action__OperateCabinetControl_Feedback));
  bool success = xczs_inspection_robot_control__action__OperateCabinetControl_Feedback__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
xczs_inspection_robot_control__action__OperateCabinetControl_Feedback__destroy(xczs_inspection_robot_control__action__OperateCabinetControl_Feedback * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    xczs_inspection_robot_control__action__OperateCabinetControl_Feedback__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
xczs_inspection_robot_control__action__OperateCabinetControl_Feedback__Sequence__init(xczs_inspection_robot_control__action__OperateCabinetControl_Feedback__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  xczs_inspection_robot_control__action__OperateCabinetControl_Feedback * data = NULL;

  if (size) {
    data = (xczs_inspection_robot_control__action__OperateCabinetControl_Feedback *)allocator.zero_allocate(size, sizeof(xczs_inspection_robot_control__action__OperateCabinetControl_Feedback), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = xczs_inspection_robot_control__action__OperateCabinetControl_Feedback__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        xczs_inspection_robot_control__action__OperateCabinetControl_Feedback__fini(&data[i - 1]);
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
xczs_inspection_robot_control__action__OperateCabinetControl_Feedback__Sequence__fini(xczs_inspection_robot_control__action__OperateCabinetControl_Feedback__Sequence * array)
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
      xczs_inspection_robot_control__action__OperateCabinetControl_Feedback__fini(&array->data[i]);
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

xczs_inspection_robot_control__action__OperateCabinetControl_Feedback__Sequence *
xczs_inspection_robot_control__action__OperateCabinetControl_Feedback__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  xczs_inspection_robot_control__action__OperateCabinetControl_Feedback__Sequence * array = (xczs_inspection_robot_control__action__OperateCabinetControl_Feedback__Sequence *)allocator.allocate(sizeof(xczs_inspection_robot_control__action__OperateCabinetControl_Feedback__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = xczs_inspection_robot_control__action__OperateCabinetControl_Feedback__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
xczs_inspection_robot_control__action__OperateCabinetControl_Feedback__Sequence__destroy(xczs_inspection_robot_control__action__OperateCabinetControl_Feedback__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    xczs_inspection_robot_control__action__OperateCabinetControl_Feedback__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
xczs_inspection_robot_control__action__OperateCabinetControl_Feedback__Sequence__are_equal(const xczs_inspection_robot_control__action__OperateCabinetControl_Feedback__Sequence * lhs, const xczs_inspection_robot_control__action__OperateCabinetControl_Feedback__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!xczs_inspection_robot_control__action__OperateCabinetControl_Feedback__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
xczs_inspection_robot_control__action__OperateCabinetControl_Feedback__Sequence__copy(
  const xczs_inspection_robot_control__action__OperateCabinetControl_Feedback__Sequence * input,
  xczs_inspection_robot_control__action__OperateCabinetControl_Feedback__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(xczs_inspection_robot_control__action__OperateCabinetControl_Feedback);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    xczs_inspection_robot_control__action__OperateCabinetControl_Feedback * data =
      (xczs_inspection_robot_control__action__OperateCabinetControl_Feedback *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!xczs_inspection_robot_control__action__OperateCabinetControl_Feedback__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          xczs_inspection_robot_control__action__OperateCabinetControl_Feedback__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!xczs_inspection_robot_control__action__OperateCabinetControl_Feedback__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `goal`
// already included above
// #include "xczs_inspection_robot_control/action/detail/operate_cabinet_control__functions.h"

bool
xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Request__init(xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Request__fini(msg);
    return false;
  }
  // goal
  if (!xczs_inspection_robot_control__action__OperateCabinetControl_Goal__init(&msg->goal)) {
    xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Request__fini(msg);
    return false;
  }
  return true;
}

void
xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Request__fini(xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // goal
  xczs_inspection_robot_control__action__OperateCabinetControl_Goal__fini(&msg->goal);
}

bool
xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Request__are_equal(const xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Request * lhs, const xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // goal
  if (!xczs_inspection_robot_control__action__OperateCabinetControl_Goal__are_equal(
      &(lhs->goal), &(rhs->goal)))
  {
    return false;
  }
  return true;
}

bool
xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Request__copy(
  const xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Request * input,
  xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // goal
  if (!xczs_inspection_robot_control__action__OperateCabinetControl_Goal__copy(
      &(input->goal), &(output->goal)))
  {
    return false;
  }
  return true;
}

xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Request *
xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Request * msg = (xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Request *)allocator.allocate(sizeof(xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Request));
  bool success = xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Request__destroy(xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Request__Sequence__init(xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Request * data = NULL;

  if (size) {
    data = (xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Request *)allocator.zero_allocate(size, sizeof(xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Request__fini(&data[i - 1]);
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
xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Request__Sequence__fini(xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Request__Sequence * array)
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
      xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Request__fini(&array->data[i]);
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

xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Request__Sequence *
xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Request__Sequence * array = (xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Request__Sequence *)allocator.allocate(sizeof(xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Request__Sequence__destroy(xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Request__Sequence__are_equal(const xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Request__Sequence * lhs, const xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Request__Sequence__copy(
  const xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Request__Sequence * input,
  xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Request * data =
      (xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Response__init(xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Response * msg)
{
  if (!msg) {
    return false;
  }
  // accepted
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Response__fini(msg);
    return false;
  }
  return true;
}

void
xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Response__fini(xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Response * msg)
{
  if (!msg) {
    return;
  }
  // accepted
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
}

bool
xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Response__are_equal(const xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Response * lhs, const xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // accepted
  if (lhs->accepted != rhs->accepted) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->stamp), &(rhs->stamp)))
  {
    return false;
  }
  return true;
}

bool
xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Response__copy(
  const xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Response * input,
  xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // accepted
  output->accepted = input->accepted;
  // stamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->stamp), &(output->stamp)))
  {
    return false;
  }
  return true;
}

xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Response *
xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Response * msg = (xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Response *)allocator.allocate(sizeof(xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Response));
  bool success = xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Response__destroy(xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Response__Sequence__init(xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Response * data = NULL;

  if (size) {
    data = (xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Response *)allocator.zero_allocate(size, sizeof(xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Response__fini(&data[i - 1]);
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
xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Response__Sequence__fini(xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Response__Sequence * array)
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
      xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Response__fini(&array->data[i]);
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

xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Response__Sequence *
xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Response__Sequence * array = (xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Response__Sequence *)allocator.allocate(sizeof(xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Response__Sequence__destroy(xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Response__Sequence__are_equal(const xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Response__Sequence * lhs, const xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Response__Sequence__copy(
  const xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Response__Sequence * input,
  xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Response * data =
      (xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!xczs_inspection_robot_control__action__OperateCabinetControl_SendGoal_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"

bool
xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Request__init(xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Request__fini(msg);
    return false;
  }
  return true;
}

void
xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Request__fini(xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
}

bool
xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Request__are_equal(const xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Request * lhs, const xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  return true;
}

bool
xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Request__copy(
  const xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Request * input,
  xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  return true;
}

xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Request *
xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Request * msg = (xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Request *)allocator.allocate(sizeof(xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Request));
  bool success = xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Request__destroy(xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Request__Sequence__init(xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Request * data = NULL;

  if (size) {
    data = (xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Request *)allocator.zero_allocate(size, sizeof(xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Request__fini(&data[i - 1]);
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
xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Request__Sequence__fini(xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Request__Sequence * array)
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
      xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Request__fini(&array->data[i]);
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

xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Request__Sequence *
xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Request__Sequence * array = (xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Request__Sequence *)allocator.allocate(sizeof(xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Request__Sequence__destroy(xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Request__Sequence__are_equal(const xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Request__Sequence * lhs, const xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Request__Sequence__copy(
  const xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Request__Sequence * input,
  xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Request * data =
      (xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `result`
// already included above
// #include "xczs_inspection_robot_control/action/detail/operate_cabinet_control__functions.h"

bool
xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Response__init(xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Response * msg)
{
  if (!msg) {
    return false;
  }
  // status
  // result
  if (!xczs_inspection_robot_control__action__OperateCabinetControl_Result__init(&msg->result)) {
    xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Response__fini(msg);
    return false;
  }
  return true;
}

void
xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Response__fini(xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Response * msg)
{
  if (!msg) {
    return;
  }
  // status
  // result
  xczs_inspection_robot_control__action__OperateCabinetControl_Result__fini(&msg->result);
}

bool
xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Response__are_equal(const xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Response * lhs, const xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // status
  if (lhs->status != rhs->status) {
    return false;
  }
  // result
  if (!xczs_inspection_robot_control__action__OperateCabinetControl_Result__are_equal(
      &(lhs->result), &(rhs->result)))
  {
    return false;
  }
  return true;
}

bool
xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Response__copy(
  const xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Response * input,
  xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // status
  output->status = input->status;
  // result
  if (!xczs_inspection_robot_control__action__OperateCabinetControl_Result__copy(
      &(input->result), &(output->result)))
  {
    return false;
  }
  return true;
}

xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Response *
xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Response * msg = (xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Response *)allocator.allocate(sizeof(xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Response));
  bool success = xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Response__destroy(xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Response__Sequence__init(xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Response * data = NULL;

  if (size) {
    data = (xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Response *)allocator.zero_allocate(size, sizeof(xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Response__fini(&data[i - 1]);
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
xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Response__Sequence__fini(xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Response__Sequence * array)
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
      xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Response__fini(&array->data[i]);
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

xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Response__Sequence *
xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Response__Sequence * array = (xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Response__Sequence *)allocator.allocate(sizeof(xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Response__Sequence__destroy(xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Response__Sequence__are_equal(const xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Response__Sequence * lhs, const xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Response__Sequence__copy(
  const xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Response__Sequence * input,
  xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Response * data =
      (xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!xczs_inspection_robot_control__action__OperateCabinetControl_GetResult_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `feedback`
// already included above
// #include "xczs_inspection_robot_control/action/detail/operate_cabinet_control__functions.h"

bool
xczs_inspection_robot_control__action__OperateCabinetControl_FeedbackMessage__init(xczs_inspection_robot_control__action__OperateCabinetControl_FeedbackMessage * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    xczs_inspection_robot_control__action__OperateCabinetControl_FeedbackMessage__fini(msg);
    return false;
  }
  // feedback
  if (!xczs_inspection_robot_control__action__OperateCabinetControl_Feedback__init(&msg->feedback)) {
    xczs_inspection_robot_control__action__OperateCabinetControl_FeedbackMessage__fini(msg);
    return false;
  }
  return true;
}

void
xczs_inspection_robot_control__action__OperateCabinetControl_FeedbackMessage__fini(xczs_inspection_robot_control__action__OperateCabinetControl_FeedbackMessage * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // feedback
  xczs_inspection_robot_control__action__OperateCabinetControl_Feedback__fini(&msg->feedback);
}

bool
xczs_inspection_robot_control__action__OperateCabinetControl_FeedbackMessage__are_equal(const xczs_inspection_robot_control__action__OperateCabinetControl_FeedbackMessage * lhs, const xczs_inspection_robot_control__action__OperateCabinetControl_FeedbackMessage * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // feedback
  if (!xczs_inspection_robot_control__action__OperateCabinetControl_Feedback__are_equal(
      &(lhs->feedback), &(rhs->feedback)))
  {
    return false;
  }
  return true;
}

bool
xczs_inspection_robot_control__action__OperateCabinetControl_FeedbackMessage__copy(
  const xczs_inspection_robot_control__action__OperateCabinetControl_FeedbackMessage * input,
  xczs_inspection_robot_control__action__OperateCabinetControl_FeedbackMessage * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // feedback
  if (!xczs_inspection_robot_control__action__OperateCabinetControl_Feedback__copy(
      &(input->feedback), &(output->feedback)))
  {
    return false;
  }
  return true;
}

xczs_inspection_robot_control__action__OperateCabinetControl_FeedbackMessage *
xczs_inspection_robot_control__action__OperateCabinetControl_FeedbackMessage__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  xczs_inspection_robot_control__action__OperateCabinetControl_FeedbackMessage * msg = (xczs_inspection_robot_control__action__OperateCabinetControl_FeedbackMessage *)allocator.allocate(sizeof(xczs_inspection_robot_control__action__OperateCabinetControl_FeedbackMessage), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(xczs_inspection_robot_control__action__OperateCabinetControl_FeedbackMessage));
  bool success = xczs_inspection_robot_control__action__OperateCabinetControl_FeedbackMessage__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
xczs_inspection_robot_control__action__OperateCabinetControl_FeedbackMessage__destroy(xczs_inspection_robot_control__action__OperateCabinetControl_FeedbackMessage * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    xczs_inspection_robot_control__action__OperateCabinetControl_FeedbackMessage__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
xczs_inspection_robot_control__action__OperateCabinetControl_FeedbackMessage__Sequence__init(xczs_inspection_robot_control__action__OperateCabinetControl_FeedbackMessage__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  xczs_inspection_robot_control__action__OperateCabinetControl_FeedbackMessage * data = NULL;

  if (size) {
    data = (xczs_inspection_robot_control__action__OperateCabinetControl_FeedbackMessage *)allocator.zero_allocate(size, sizeof(xczs_inspection_robot_control__action__OperateCabinetControl_FeedbackMessage), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = xczs_inspection_robot_control__action__OperateCabinetControl_FeedbackMessage__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        xczs_inspection_robot_control__action__OperateCabinetControl_FeedbackMessage__fini(&data[i - 1]);
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
xczs_inspection_robot_control__action__OperateCabinetControl_FeedbackMessage__Sequence__fini(xczs_inspection_robot_control__action__OperateCabinetControl_FeedbackMessage__Sequence * array)
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
      xczs_inspection_robot_control__action__OperateCabinetControl_FeedbackMessage__fini(&array->data[i]);
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

xczs_inspection_robot_control__action__OperateCabinetControl_FeedbackMessage__Sequence *
xczs_inspection_robot_control__action__OperateCabinetControl_FeedbackMessage__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  xczs_inspection_robot_control__action__OperateCabinetControl_FeedbackMessage__Sequence * array = (xczs_inspection_robot_control__action__OperateCabinetControl_FeedbackMessage__Sequence *)allocator.allocate(sizeof(xczs_inspection_robot_control__action__OperateCabinetControl_FeedbackMessage__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = xczs_inspection_robot_control__action__OperateCabinetControl_FeedbackMessage__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
xczs_inspection_robot_control__action__OperateCabinetControl_FeedbackMessage__Sequence__destroy(xczs_inspection_robot_control__action__OperateCabinetControl_FeedbackMessage__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    xczs_inspection_robot_control__action__OperateCabinetControl_FeedbackMessage__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
xczs_inspection_robot_control__action__OperateCabinetControl_FeedbackMessage__Sequence__are_equal(const xczs_inspection_robot_control__action__OperateCabinetControl_FeedbackMessage__Sequence * lhs, const xczs_inspection_robot_control__action__OperateCabinetControl_FeedbackMessage__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!xczs_inspection_robot_control__action__OperateCabinetControl_FeedbackMessage__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
xczs_inspection_robot_control__action__OperateCabinetControl_FeedbackMessage__Sequence__copy(
  const xczs_inspection_robot_control__action__OperateCabinetControl_FeedbackMessage__Sequence * input,
  xczs_inspection_robot_control__action__OperateCabinetControl_FeedbackMessage__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(xczs_inspection_robot_control__action__OperateCabinetControl_FeedbackMessage);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    xczs_inspection_robot_control__action__OperateCabinetControl_FeedbackMessage * data =
      (xczs_inspection_robot_control__action__OperateCabinetControl_FeedbackMessage *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!xczs_inspection_robot_control__action__OperateCabinetControl_FeedbackMessage__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          xczs_inspection_robot_control__action__OperateCabinetControl_FeedbackMessage__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!xczs_inspection_robot_control__action__OperateCabinetControl_FeedbackMessage__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
