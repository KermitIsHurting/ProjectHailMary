// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from cuas_msgs:msg/TrackArray.idl
// generated code does not contain a copyright notice

#ifndef CUAS_MSGS__MSG__DETAIL__TRACK_ARRAY__FUNCTIONS_H_
#define CUAS_MSGS__MSG__DETAIL__TRACK_ARRAY__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "cuas_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "cuas_msgs/msg/detail/track_array__struct.h"

/// Initialize msg/TrackArray message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * cuas_msgs__msg__TrackArray
 * )) before or use
 * cuas_msgs__msg__TrackArray__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_cuas_msgs
bool
cuas_msgs__msg__TrackArray__init(cuas_msgs__msg__TrackArray * msg);

/// Finalize msg/TrackArray message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_cuas_msgs
void
cuas_msgs__msg__TrackArray__fini(cuas_msgs__msg__TrackArray * msg);

/// Create msg/TrackArray message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * cuas_msgs__msg__TrackArray__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_cuas_msgs
cuas_msgs__msg__TrackArray *
cuas_msgs__msg__TrackArray__create();

/// Destroy msg/TrackArray message.
/**
 * It calls
 * cuas_msgs__msg__TrackArray__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_cuas_msgs
void
cuas_msgs__msg__TrackArray__destroy(cuas_msgs__msg__TrackArray * msg);

/// Check for msg/TrackArray message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_cuas_msgs
bool
cuas_msgs__msg__TrackArray__are_equal(const cuas_msgs__msg__TrackArray * lhs, const cuas_msgs__msg__TrackArray * rhs);

/// Copy a msg/TrackArray message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_cuas_msgs
bool
cuas_msgs__msg__TrackArray__copy(
  const cuas_msgs__msg__TrackArray * input,
  cuas_msgs__msg__TrackArray * output);

/// Initialize array of msg/TrackArray messages.
/**
 * It allocates the memory for the number of elements and calls
 * cuas_msgs__msg__TrackArray__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_cuas_msgs
bool
cuas_msgs__msg__TrackArray__Sequence__init(cuas_msgs__msg__TrackArray__Sequence * array, size_t size);

/// Finalize array of msg/TrackArray messages.
/**
 * It calls
 * cuas_msgs__msg__TrackArray__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_cuas_msgs
void
cuas_msgs__msg__TrackArray__Sequence__fini(cuas_msgs__msg__TrackArray__Sequence * array);

/// Create array of msg/TrackArray messages.
/**
 * It allocates the memory for the array and calls
 * cuas_msgs__msg__TrackArray__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_cuas_msgs
cuas_msgs__msg__TrackArray__Sequence *
cuas_msgs__msg__TrackArray__Sequence__create(size_t size);

/// Destroy array of msg/TrackArray messages.
/**
 * It calls
 * cuas_msgs__msg__TrackArray__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_cuas_msgs
void
cuas_msgs__msg__TrackArray__Sequence__destroy(cuas_msgs__msg__TrackArray__Sequence * array);

/// Check for msg/TrackArray message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_cuas_msgs
bool
cuas_msgs__msg__TrackArray__Sequence__are_equal(const cuas_msgs__msg__TrackArray__Sequence * lhs, const cuas_msgs__msg__TrackArray__Sequence * rhs);

/// Copy an array of msg/TrackArray messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_cuas_msgs
bool
cuas_msgs__msg__TrackArray__Sequence__copy(
  const cuas_msgs__msg__TrackArray__Sequence * input,
  cuas_msgs__msg__TrackArray__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // CUAS_MSGS__MSG__DETAIL__TRACK_ARRAY__FUNCTIONS_H_
