// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from xczs_inspection_robot_control:msg/CabinetControlCatalog.idl
// generated code does not contain a copyright notice
#include "xczs_inspection_robot_control/msg/detail/cabinet_control_catalog__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `controls`
#include "xczs_inspection_robot_control/msg/detail/cabinet_control__functions.h"

bool
xczs_inspection_robot_control__msg__CabinetControlCatalog__init(xczs_inspection_robot_control__msg__CabinetControlCatalog * msg)
{
  if (!msg) {
    return false;
  }
  // controls
  if (!xczs_inspection_robot_control__msg__CabinetControl__Sequence__init(&msg->controls, 0)) {
    xczs_inspection_robot_control__msg__CabinetControlCatalog__fini(msg);
    return false;
  }
  return true;
}

void
xczs_inspection_robot_control__msg__CabinetControlCatalog__fini(xczs_inspection_robot_control__msg__CabinetControlCatalog * msg)
{
  if (!msg) {
    return;
  }
  // controls
  xczs_inspection_robot_control__msg__CabinetControl__Sequence__fini(&msg->controls);
}

bool
xczs_inspection_robot_control__msg__CabinetControlCatalog__are_equal(const xczs_inspection_robot_control__msg__CabinetControlCatalog * lhs, const xczs_inspection_robot_control__msg__CabinetControlCatalog * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // controls
  if (!xczs_inspection_robot_control__msg__CabinetControl__Sequence__are_equal(
      &(lhs->controls), &(rhs->controls)))
  {
    return false;
  }
  return true;
}

bool
xczs_inspection_robot_control__msg__CabinetControlCatalog__copy(
  const xczs_inspection_robot_control__msg__CabinetControlCatalog * input,
  xczs_inspection_robot_control__msg__CabinetControlCatalog * output)
{
  if (!input || !output) {
    return false;
  }
  // controls
  if (!xczs_inspection_robot_control__msg__CabinetControl__Sequence__copy(
      &(input->controls), &(output->controls)))
  {
    return false;
  }
  return true;
}

xczs_inspection_robot_control__msg__CabinetControlCatalog *
xczs_inspection_robot_control__msg__CabinetControlCatalog__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  xczs_inspection_robot_control__msg__CabinetControlCatalog * msg = (xczs_inspection_robot_control__msg__CabinetControlCatalog *)allocator.allocate(sizeof(xczs_inspection_robot_control__msg__CabinetControlCatalog), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(xczs_inspection_robot_control__msg__CabinetControlCatalog));
  bool success = xczs_inspection_robot_control__msg__CabinetControlCatalog__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
xczs_inspection_robot_control__msg__CabinetControlCatalog__destroy(xczs_inspection_robot_control__msg__CabinetControlCatalog * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    xczs_inspection_robot_control__msg__CabinetControlCatalog__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
xczs_inspection_robot_control__msg__CabinetControlCatalog__Sequence__init(xczs_inspection_robot_control__msg__CabinetControlCatalog__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  xczs_inspection_robot_control__msg__CabinetControlCatalog * data = NULL;

  if (size) {
    data = (xczs_inspection_robot_control__msg__CabinetControlCatalog *)allocator.zero_allocate(size, sizeof(xczs_inspection_robot_control__msg__CabinetControlCatalog), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = xczs_inspection_robot_control__msg__CabinetControlCatalog__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        xczs_inspection_robot_control__msg__CabinetControlCatalog__fini(&data[i - 1]);
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
xczs_inspection_robot_control__msg__CabinetControlCatalog__Sequence__fini(xczs_inspection_robot_control__msg__CabinetControlCatalog__Sequence * array)
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
      xczs_inspection_robot_control__msg__CabinetControlCatalog__fini(&array->data[i]);
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

xczs_inspection_robot_control__msg__CabinetControlCatalog__Sequence *
xczs_inspection_robot_control__msg__CabinetControlCatalog__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  xczs_inspection_robot_control__msg__CabinetControlCatalog__Sequence * array = (xczs_inspection_robot_control__msg__CabinetControlCatalog__Sequence *)allocator.allocate(sizeof(xczs_inspection_robot_control__msg__CabinetControlCatalog__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = xczs_inspection_robot_control__msg__CabinetControlCatalog__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
xczs_inspection_robot_control__msg__CabinetControlCatalog__Sequence__destroy(xczs_inspection_robot_control__msg__CabinetControlCatalog__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    xczs_inspection_robot_control__msg__CabinetControlCatalog__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
xczs_inspection_robot_control__msg__CabinetControlCatalog__Sequence__are_equal(const xczs_inspection_robot_control__msg__CabinetControlCatalog__Sequence * lhs, const xczs_inspection_robot_control__msg__CabinetControlCatalog__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!xczs_inspection_robot_control__msg__CabinetControlCatalog__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
xczs_inspection_robot_control__msg__CabinetControlCatalog__Sequence__copy(
  const xczs_inspection_robot_control__msg__CabinetControlCatalog__Sequence * input,
  xczs_inspection_robot_control__msg__CabinetControlCatalog__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(xczs_inspection_robot_control__msg__CabinetControlCatalog);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    xczs_inspection_robot_control__msg__CabinetControlCatalog * data =
      (xczs_inspection_robot_control__msg__CabinetControlCatalog *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!xczs_inspection_robot_control__msg__CabinetControlCatalog__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          xczs_inspection_robot_control__msg__CabinetControlCatalog__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!xczs_inspection_robot_control__msg__CabinetControlCatalog__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
