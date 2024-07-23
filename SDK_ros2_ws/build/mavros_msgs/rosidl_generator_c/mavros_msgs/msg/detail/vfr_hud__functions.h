// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from mavros_msgs:msg/VfrHud.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__VFR_HUD__FUNCTIONS_H_
#define MAVROS_MSGS__MSG__DETAIL__VFR_HUD__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "mavros_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "mavros_msgs/msg/detail/vfr_hud__struct.h"

/// Initialize msg/VfrHud message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * mavros_msgs__msg__VfrHud
 * )) before or use
 * mavros_msgs__msg__VfrHud__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_mavros_msgs
bool
mavros_msgs__msg__VfrHud__init(mavros_msgs__msg__VfrHud * msg);

/// Finalize msg/VfrHud message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mavros_msgs
void
mavros_msgs__msg__VfrHud__fini(mavros_msgs__msg__VfrHud * msg);

/// Create msg/VfrHud message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * mavros_msgs__msg__VfrHud__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_mavros_msgs
mavros_msgs__msg__VfrHud *
mavros_msgs__msg__VfrHud__create();

/// Destroy msg/VfrHud message.
/**
 * It calls
 * mavros_msgs__msg__VfrHud__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mavros_msgs
void
mavros_msgs__msg__VfrHud__destroy(mavros_msgs__msg__VfrHud * msg);

/// Check for msg/VfrHud message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_mavros_msgs
bool
mavros_msgs__msg__VfrHud__are_equal(const mavros_msgs__msg__VfrHud * lhs, const mavros_msgs__msg__VfrHud * rhs);

/// Copy a msg/VfrHud message.
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
ROSIDL_GENERATOR_C_PUBLIC_mavros_msgs
bool
mavros_msgs__msg__VfrHud__copy(
  const mavros_msgs__msg__VfrHud * input,
  mavros_msgs__msg__VfrHud * output);

/// Initialize array of msg/VfrHud messages.
/**
 * It allocates the memory for the number of elements and calls
 * mavros_msgs__msg__VfrHud__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_mavros_msgs
bool
mavros_msgs__msg__VfrHud__Sequence__init(mavros_msgs__msg__VfrHud__Sequence * array, size_t size);

/// Finalize array of msg/VfrHud messages.
/**
 * It calls
 * mavros_msgs__msg__VfrHud__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mavros_msgs
void
mavros_msgs__msg__VfrHud__Sequence__fini(mavros_msgs__msg__VfrHud__Sequence * array);

/// Create array of msg/VfrHud messages.
/**
 * It allocates the memory for the array and calls
 * mavros_msgs__msg__VfrHud__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_mavros_msgs
mavros_msgs__msg__VfrHud__Sequence *
mavros_msgs__msg__VfrHud__Sequence__create(size_t size);

/// Destroy array of msg/VfrHud messages.
/**
 * It calls
 * mavros_msgs__msg__VfrHud__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mavros_msgs
void
mavros_msgs__msg__VfrHud__Sequence__destroy(mavros_msgs__msg__VfrHud__Sequence * array);

/// Check for msg/VfrHud message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_mavros_msgs
bool
mavros_msgs__msg__VfrHud__Sequence__are_equal(const mavros_msgs__msg__VfrHud__Sequence * lhs, const mavros_msgs__msg__VfrHud__Sequence * rhs);

/// Copy an array of msg/VfrHud messages.
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
ROSIDL_GENERATOR_C_PUBLIC_mavros_msgs
bool
mavros_msgs__msg__VfrHud__Sequence__copy(
  const mavros_msgs__msg__VfrHud__Sequence * input,
  mavros_msgs__msg__VfrHud__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // MAVROS_MSGS__MSG__DETAIL__VFR_HUD__FUNCTIONS_H_
