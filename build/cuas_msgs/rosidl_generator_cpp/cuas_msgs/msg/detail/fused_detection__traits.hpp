// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from cuas_msgs:msg/FusedDetection.idl
// generated code does not contain a copyright notice

#ifndef CUAS_MSGS__MSG__DETAIL__FUSED_DETECTION__TRAITS_HPP_
#define CUAS_MSGS__MSG__DETAIL__FUSED_DETECTION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "cuas_msgs/msg/detail/fused_detection__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace cuas_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const FusedDetection & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const FusedDetection & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const FusedDetection & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace cuas_msgs

namespace rosidl_generator_traits
{

[[deprecated("use cuas_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const cuas_msgs::msg::FusedDetection & msg,
  std::ostream & out, size_t indentation = 0)
{
  cuas_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use cuas_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const cuas_msgs::msg::FusedDetection & msg)
{
  return cuas_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<cuas_msgs::msg::FusedDetection>()
{
  return "cuas_msgs::msg::FusedDetection";
}

template<>
inline const char * name<cuas_msgs::msg::FusedDetection>()
{
  return "cuas_msgs/msg/FusedDetection";
}

template<>
struct has_fixed_size<cuas_msgs::msg::FusedDetection>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<cuas_msgs::msg::FusedDetection>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<cuas_msgs::msg::FusedDetection>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CUAS_MSGS__MSG__DETAIL__FUSED_DETECTION__TRAITS_HPP_
