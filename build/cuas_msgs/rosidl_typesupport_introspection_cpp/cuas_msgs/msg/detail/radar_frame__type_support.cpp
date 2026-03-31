// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from cuas_msgs:msg/RadarFrame.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "cuas_msgs/msg/detail/radar_frame__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace cuas_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void RadarFrame_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) cuas_msgs::msg::RadarFrame(_init);
}

void RadarFrame_fini_function(void * message_memory)
{
  auto typed_message = static_cast<cuas_msgs::msg::RadarFrame *>(message_memory);
  typed_message->~RadarFrame();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember RadarFrame_message_member_array[1] = {
  {
    "structure_needs_at_least_one_member",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cuas_msgs::msg::RadarFrame, structure_needs_at_least_one_member),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers RadarFrame_message_members = {
  "cuas_msgs::msg",  // message namespace
  "RadarFrame",  // message name
  1,  // number of fields
  sizeof(cuas_msgs::msg::RadarFrame),
  RadarFrame_message_member_array,  // message members
  RadarFrame_init_function,  // function to initialize message memory (memory has to be allocated)
  RadarFrame_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t RadarFrame_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &RadarFrame_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace cuas_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<cuas_msgs::msg::RadarFrame>()
{
  return &::cuas_msgs::msg::rosidl_typesupport_introspection_cpp::RadarFrame_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, cuas_msgs, msg, RadarFrame)() {
  return &::cuas_msgs::msg::rosidl_typesupport_introspection_cpp::RadarFrame_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
