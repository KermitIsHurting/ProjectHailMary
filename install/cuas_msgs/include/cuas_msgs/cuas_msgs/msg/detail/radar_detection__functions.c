// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from cuas_msgs:msg/RadarDetection.idl
// generated code does not contain a copyright notice
#include "cuas_msgs/msg/detail/radar_detection__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
cuas_msgs__msg__RadarDetection__init(cuas_msgs__msg__RadarDetection * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
cuas_msgs__msg__RadarDetection__fini(cuas_msgs__msg__RadarDetection * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
cuas_msgs__msg__RadarDetection__are_equal(const cuas_msgs__msg__RadarDetection * lhs, const cuas_msgs__msg__RadarDetection * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // structure_needs_at_least_one_member
  if (lhs->structure_needs_at_least_one_member != rhs->structure_needs_at_least_one_member) {
    return false;
  }
  return true;
}

bool
cuas_msgs__msg__RadarDetection__copy(
  const cuas_msgs__msg__RadarDetection * input,
  cuas_msgs__msg__RadarDetection * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

cuas_msgs__msg__RadarDetection *
cuas_msgs__msg__RadarDetection__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cuas_msgs__msg__RadarDetection * msg = (cuas_msgs__msg__RadarDetection *)allocator.allocate(sizeof(cuas_msgs__msg__RadarDetection), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(cuas_msgs__msg__RadarDetection));
  bool success = cuas_msgs__msg__RadarDetection__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
cuas_msgs__msg__RadarDetection__destroy(cuas_msgs__msg__RadarDetection * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    cuas_msgs__msg__RadarDetection__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
cuas_msgs__msg__RadarDetection__Sequence__init(cuas_msgs__msg__RadarDetection__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cuas_msgs__msg__RadarDetection * data = NULL;

  if (size) {
    data = (cuas_msgs__msg__RadarDetection *)allocator.zero_allocate(size, sizeof(cuas_msgs__msg__RadarDetection), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = cuas_msgs__msg__RadarDetection__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        cuas_msgs__msg__RadarDetection__fini(&data[i - 1]);
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
cuas_msgs__msg__RadarDetection__Sequence__fini(cuas_msgs__msg__RadarDetection__Sequence * array)
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
      cuas_msgs__msg__RadarDetection__fini(&array->data[i]);
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

cuas_msgs__msg__RadarDetection__Sequence *
cuas_msgs__msg__RadarDetection__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cuas_msgs__msg__RadarDetection__Sequence * array = (cuas_msgs__msg__RadarDetection__Sequence *)allocator.allocate(sizeof(cuas_msgs__msg__RadarDetection__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = cuas_msgs__msg__RadarDetection__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
cuas_msgs__msg__RadarDetection__Sequence__destroy(cuas_msgs__msg__RadarDetection__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    cuas_msgs__msg__RadarDetection__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
cuas_msgs__msg__RadarDetection__Sequence__are_equal(const cuas_msgs__msg__RadarDetection__Sequence * lhs, const cuas_msgs__msg__RadarDetection__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!cuas_msgs__msg__RadarDetection__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
cuas_msgs__msg__RadarDetection__Sequence__copy(
  const cuas_msgs__msg__RadarDetection__Sequence * input,
  cuas_msgs__msg__RadarDetection__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(cuas_msgs__msg__RadarDetection);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    cuas_msgs__msg__RadarDetection * data =
      (cuas_msgs__msg__RadarDetection *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!cuas_msgs__msg__RadarDetection__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          cuas_msgs__msg__RadarDetection__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!cuas_msgs__msg__RadarDetection__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
