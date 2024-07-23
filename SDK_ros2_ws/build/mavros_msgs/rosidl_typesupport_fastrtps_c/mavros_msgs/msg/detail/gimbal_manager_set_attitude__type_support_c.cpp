// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from mavros_msgs:msg/GimbalManagerSetAttitude.idl
// generated code does not contain a copyright notice
#include "mavros_msgs/msg/detail/gimbal_manager_set_attitude__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "mavros_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "mavros_msgs/msg/detail/gimbal_manager_set_attitude__struct.h"
#include "mavros_msgs/msg/detail/gimbal_manager_set_attitude__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "geometry_msgs/msg/detail/quaternion__functions.h"  // q

// forward declare type support functions
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mavros_msgs
size_t get_serialized_size_geometry_msgs__msg__Quaternion(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mavros_msgs
size_t max_serialized_size_geometry_msgs__msg__Quaternion(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mavros_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Quaternion)();


using _GimbalManagerSetAttitude__ros_msg_type = mavros_msgs__msg__GimbalManagerSetAttitude;

static bool _GimbalManagerSetAttitude__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _GimbalManagerSetAttitude__ros_msg_type * ros_message = static_cast<const _GimbalManagerSetAttitude__ros_msg_type *>(untyped_ros_message);
  // Field name: target_system
  {
    cdr << ros_message->target_system;
  }

  // Field name: target_component
  {
    cdr << ros_message->target_component;
  }

  // Field name: flags
  {
    cdr << ros_message->flags;
  }

  // Field name: gimbal_device_id
  {
    cdr << ros_message->gimbal_device_id;
  }

  // Field name: q
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Quaternion
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->q, cdr))
    {
      return false;
    }
  }

  // Field name: angular_velocity_x
  {
    cdr << ros_message->angular_velocity_x;
  }

  // Field name: angular_velocity_y
  {
    cdr << ros_message->angular_velocity_y;
  }

  // Field name: angular_velocity_z
  {
    cdr << ros_message->angular_velocity_z;
  }

  return true;
}

static bool _GimbalManagerSetAttitude__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _GimbalManagerSetAttitude__ros_msg_type * ros_message = static_cast<_GimbalManagerSetAttitude__ros_msg_type *>(untyped_ros_message);
  // Field name: target_system
  {
    cdr >> ros_message->target_system;
  }

  // Field name: target_component
  {
    cdr >> ros_message->target_component;
  }

  // Field name: flags
  {
    cdr >> ros_message->flags;
  }

  // Field name: gimbal_device_id
  {
    cdr >> ros_message->gimbal_device_id;
  }

  // Field name: q
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Quaternion
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->q))
    {
      return false;
    }
  }

  // Field name: angular_velocity_x
  {
    cdr >> ros_message->angular_velocity_x;
  }

  // Field name: angular_velocity_y
  {
    cdr >> ros_message->angular_velocity_y;
  }

  // Field name: angular_velocity_z
  {
    cdr >> ros_message->angular_velocity_z;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mavros_msgs
size_t get_serialized_size_mavros_msgs__msg__GimbalManagerSetAttitude(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _GimbalManagerSetAttitude__ros_msg_type * ros_message = static_cast<const _GimbalManagerSetAttitude__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name target_system
  {
    size_t item_size = sizeof(ros_message->target_system);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name target_component
  {
    size_t item_size = sizeof(ros_message->target_component);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name flags
  {
    size_t item_size = sizeof(ros_message->flags);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name gimbal_device_id
  {
    size_t item_size = sizeof(ros_message->gimbal_device_id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name q

  current_alignment += get_serialized_size_geometry_msgs__msg__Quaternion(
    &(ros_message->q), current_alignment);
  // field.name angular_velocity_x
  {
    size_t item_size = sizeof(ros_message->angular_velocity_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name angular_velocity_y
  {
    size_t item_size = sizeof(ros_message->angular_velocity_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name angular_velocity_z
  {
    size_t item_size = sizeof(ros_message->angular_velocity_z);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _GimbalManagerSetAttitude__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_mavros_msgs__msg__GimbalManagerSetAttitude(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mavros_msgs
size_t max_serialized_size_mavros_msgs__msg__GimbalManagerSetAttitude(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: target_system
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: target_component
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: flags
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: gimbal_device_id
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: q
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_geometry_msgs__msg__Quaternion(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: angular_velocity_x
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: angular_velocity_y
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: angular_velocity_z
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = mavros_msgs__msg__GimbalManagerSetAttitude;
    is_plain =
      (
      offsetof(DataType, angular_velocity_z) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _GimbalManagerSetAttitude__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_mavros_msgs__msg__GimbalManagerSetAttitude(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_GimbalManagerSetAttitude = {
  "mavros_msgs::msg",
  "GimbalManagerSetAttitude",
  _GimbalManagerSetAttitude__cdr_serialize,
  _GimbalManagerSetAttitude__cdr_deserialize,
  _GimbalManagerSetAttitude__get_serialized_size,
  _GimbalManagerSetAttitude__max_serialized_size
};

static rosidl_message_type_support_t _GimbalManagerSetAttitude__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_GimbalManagerSetAttitude,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, mavros_msgs, msg, GimbalManagerSetAttitude)() {
  return &_GimbalManagerSetAttitude__type_support;
}

#if defined(__cplusplus)
}
#endif
