// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from cuas_msgs:msg/TrackArray.idl
// generated code does not contain a copyright notice

#ifndef CUAS_MSGS__MSG__DETAIL__TRACK_ARRAY__STRUCT_H_
#define CUAS_MSGS__MSG__DETAIL__TRACK_ARRAY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/TrackArray in the package cuas_msgs.
typedef struct cuas_msgs__msg__TrackArray
{
  uint8_t structure_needs_at_least_one_member;
} cuas_msgs__msg__TrackArray;

// Struct for a sequence of cuas_msgs__msg__TrackArray.
typedef struct cuas_msgs__msg__TrackArray__Sequence
{
  cuas_msgs__msg__TrackArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} cuas_msgs__msg__TrackArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUAS_MSGS__MSG__DETAIL__TRACK_ARRAY__STRUCT_H_
