// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from cuas_msgs:msg/ThreatReport.idl
// generated code does not contain a copyright notice

#ifndef CUAS_MSGS__MSG__DETAIL__THREAT_REPORT__STRUCT_H_
#define CUAS_MSGS__MSG__DETAIL__THREAT_REPORT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/ThreatReport in the package cuas_msgs.
typedef struct cuas_msgs__msg__ThreatReport
{
  uint8_t structure_needs_at_least_one_member;
} cuas_msgs__msg__ThreatReport;

// Struct for a sequence of cuas_msgs__msg__ThreatReport.
typedef struct cuas_msgs__msg__ThreatReport__Sequence
{
  cuas_msgs__msg__ThreatReport * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} cuas_msgs__msg__ThreatReport__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUAS_MSGS__MSG__DETAIL__THREAT_REPORT__STRUCT_H_
