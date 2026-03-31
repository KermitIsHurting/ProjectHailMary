// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from cuas_msgs:msg/TrackArray.idl
// generated code does not contain a copyright notice

#ifndef CUAS_MSGS__MSG__DETAIL__TRACK_ARRAY__STRUCT_HPP_
#define CUAS_MSGS__MSG__DETAIL__TRACK_ARRAY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__cuas_msgs__msg__TrackArray __attribute__((deprecated))
#else
# define DEPRECATED__cuas_msgs__msg__TrackArray __declspec(deprecated)
#endif

namespace cuas_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TrackArray_
{
  using Type = TrackArray_<ContainerAllocator>;

  explicit TrackArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit TrackArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    cuas_msgs::msg::TrackArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const cuas_msgs::msg::TrackArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<cuas_msgs::msg::TrackArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<cuas_msgs::msg::TrackArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      cuas_msgs::msg::TrackArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<cuas_msgs::msg::TrackArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      cuas_msgs::msg::TrackArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<cuas_msgs::msg::TrackArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<cuas_msgs::msg::TrackArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<cuas_msgs::msg::TrackArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__cuas_msgs__msg__TrackArray
    std::shared_ptr<cuas_msgs::msg::TrackArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__cuas_msgs__msg__TrackArray
    std::shared_ptr<cuas_msgs::msg::TrackArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TrackArray_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const TrackArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TrackArray_

// alias to use template instance with default allocator
using TrackArray =
  cuas_msgs::msg::TrackArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace cuas_msgs

#endif  // CUAS_MSGS__MSG__DETAIL__TRACK_ARRAY__STRUCT_HPP_
