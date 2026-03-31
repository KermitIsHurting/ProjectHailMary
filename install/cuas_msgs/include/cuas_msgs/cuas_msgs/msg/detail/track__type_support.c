// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from cuas_msgs:msg/Track.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "cuas_msgs/msg/detail/track__rosidl_typesupport_introspection_c.h"
#include "cuas_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "cuas_msgs/msg/detail/track__functions.h"
#include "cuas_msgs/msg/detail/track__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void cuas_msgs__msg__Track__rosidl_typesupport_introspection_c__Track_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  cuas_msgs__msg__Track__init(message_memory);
}

void cuas_msgs__msg__Track__rosidl_typesupport_introspection_c__Track_fini_function(void * message_memory)
{
  cuas_msgs__msg__Track__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember cuas_msgs__msg__Track__rosidl_typesupport_introspection_c__Track_message_member_array[1] = {
  {
    "structure_needs_at_least_one_member",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cuas_msgs__msg__Track, structure_needs_at_least_one_member),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers cuas_msgs__msg__Track__rosidl_typesupport_introspection_c__Track_message_members = {
  "cuas_msgs__msg",  // message namespace
  "Track",  // message name
  1,  // number of fields
  sizeof(cuas_msgs__msg__Track),
  cuas_msgs__msg__Track__rosidl_typesupport_introspection_c__Track_message_member_array,  // message members
  cuas_msgs__msg__Track__rosidl_typesupport_introspection_c__Track_init_function,  // function to initialize message memory (memory has to be allocated)
  cuas_msgs__msg__Track__rosidl_typesupport_introspection_c__Track_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t cuas_msgs__msg__Track__rosidl_typesupport_introspection_c__Track_message_type_support_handle = {
  0,
  &cuas_msgs__msg__Track__rosidl_typesupport_introspection_c__Track_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_cuas_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, cuas_msgs, msg, Track)() {
  if (!cuas_msgs__msg__Track__rosidl_typesupport_introspection_c__Track_message_type_support_handle.typesupport_identifier) {
    cuas_msgs__msg__Track__rosidl_typesupport_introspection_c__Track_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &cuas_msgs__msg__Track__rosidl_typesupport_introspection_c__Track_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
