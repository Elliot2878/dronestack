// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from mavros_msgs:msg/EstimatorStatus.idl
// generated code does not contain a copyright notice
#include "mavros_msgs/msg/detail/estimator_status__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "mavros_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "mavros_msgs/msg/detail/estimator_status__struct.h"
#include "mavros_msgs/msg/detail/estimator_status__functions.h"
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

#include "std_msgs/msg/detail/header__functions.h"  // header

// forward declare type support functions
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mavros_msgs
size_t get_serialized_size_std_msgs__msg__Header(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mavros_msgs
size_t max_serialized_size_std_msgs__msg__Header(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mavros_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, std_msgs, msg, Header)();


using _EstimatorStatus__ros_msg_type = mavros_msgs__msg__EstimatorStatus;

static bool _EstimatorStatus__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _EstimatorStatus__ros_msg_type * ros_message = static_cast<const _EstimatorStatus__ros_msg_type *>(untyped_ros_message);
  // Field name: header
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, Header
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->header, cdr))
    {
      return false;
    }
  }

  // Field name: attitude_status_flag
  {
    cdr << (ros_message->attitude_status_flag ? true : false);
  }

  // Field name: velocity_horiz_status_flag
  {
    cdr << (ros_message->velocity_horiz_status_flag ? true : false);
  }

  // Field name: velocity_vert_status_flag
  {
    cdr << (ros_message->velocity_vert_status_flag ? true : false);
  }

  // Field name: pos_horiz_rel_status_flag
  {
    cdr << (ros_message->pos_horiz_rel_status_flag ? true : false);
  }

  // Field name: pos_horiz_abs_status_flag
  {
    cdr << (ros_message->pos_horiz_abs_status_flag ? true : false);
  }

  // Field name: pos_vert_abs_status_flag
  {
    cdr << (ros_message->pos_vert_abs_status_flag ? true : false);
  }

  // Field name: pos_vert_agl_status_flag
  {
    cdr << (ros_message->pos_vert_agl_status_flag ? true : false);
  }

  // Field name: const_pos_mode_status_flag
  {
    cdr << (ros_message->const_pos_mode_status_flag ? true : false);
  }

  // Field name: pred_pos_horiz_rel_status_flag
  {
    cdr << (ros_message->pred_pos_horiz_rel_status_flag ? true : false);
  }

  // Field name: pred_pos_horiz_abs_status_flag
  {
    cdr << (ros_message->pred_pos_horiz_abs_status_flag ? true : false);
  }

  // Field name: gps_glitch_status_flag
  {
    cdr << (ros_message->gps_glitch_status_flag ? true : false);
  }

  // Field name: accel_error_status_flag
  {
    cdr << (ros_message->accel_error_status_flag ? true : false);
  }

  return true;
}

static bool _EstimatorStatus__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _EstimatorStatus__ros_msg_type * ros_message = static_cast<_EstimatorStatus__ros_msg_type *>(untyped_ros_message);
  // Field name: header
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, Header
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->header))
    {
      return false;
    }
  }

  // Field name: attitude_status_flag
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->attitude_status_flag = tmp ? true : false;
  }

  // Field name: velocity_horiz_status_flag
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->velocity_horiz_status_flag = tmp ? true : false;
  }

  // Field name: velocity_vert_status_flag
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->velocity_vert_status_flag = tmp ? true : false;
  }

  // Field name: pos_horiz_rel_status_flag
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->pos_horiz_rel_status_flag = tmp ? true : false;
  }

  // Field name: pos_horiz_abs_status_flag
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->pos_horiz_abs_status_flag = tmp ? true : false;
  }

  // Field name: pos_vert_abs_status_flag
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->pos_vert_abs_status_flag = tmp ? true : false;
  }

  // Field name: pos_vert_agl_status_flag
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->pos_vert_agl_status_flag = tmp ? true : false;
  }

  // Field name: const_pos_mode_status_flag
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->const_pos_mode_status_flag = tmp ? true : false;
  }

  // Field name: pred_pos_horiz_rel_status_flag
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->pred_pos_horiz_rel_status_flag = tmp ? true : false;
  }

  // Field name: pred_pos_horiz_abs_status_flag
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->pred_pos_horiz_abs_status_flag = tmp ? true : false;
  }

  // Field name: gps_glitch_status_flag
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->gps_glitch_status_flag = tmp ? true : false;
  }

  // Field name: accel_error_status_flag
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->accel_error_status_flag = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mavros_msgs
size_t get_serialized_size_mavros_msgs__msg__EstimatorStatus(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _EstimatorStatus__ros_msg_type * ros_message = static_cast<const _EstimatorStatus__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name header

  current_alignment += get_serialized_size_std_msgs__msg__Header(
    &(ros_message->header), current_alignment);
  // field.name attitude_status_flag
  {
    size_t item_size = sizeof(ros_message->attitude_status_flag);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name velocity_horiz_status_flag
  {
    size_t item_size = sizeof(ros_message->velocity_horiz_status_flag);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name velocity_vert_status_flag
  {
    size_t item_size = sizeof(ros_message->velocity_vert_status_flag);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name pos_horiz_rel_status_flag
  {
    size_t item_size = sizeof(ros_message->pos_horiz_rel_status_flag);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name pos_horiz_abs_status_flag
  {
    size_t item_size = sizeof(ros_message->pos_horiz_abs_status_flag);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name pos_vert_abs_status_flag
  {
    size_t item_size = sizeof(ros_message->pos_vert_abs_status_flag);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name pos_vert_agl_status_flag
  {
    size_t item_size = sizeof(ros_message->pos_vert_agl_status_flag);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name const_pos_mode_status_flag
  {
    size_t item_size = sizeof(ros_message->const_pos_mode_status_flag);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name pred_pos_horiz_rel_status_flag
  {
    size_t item_size = sizeof(ros_message->pred_pos_horiz_rel_status_flag);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name pred_pos_horiz_abs_status_flag
  {
    size_t item_size = sizeof(ros_message->pred_pos_horiz_abs_status_flag);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name gps_glitch_status_flag
  {
    size_t item_size = sizeof(ros_message->gps_glitch_status_flag);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name accel_error_status_flag
  {
    size_t item_size = sizeof(ros_message->accel_error_status_flag);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _EstimatorStatus__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_mavros_msgs__msg__EstimatorStatus(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mavros_msgs
size_t max_serialized_size_mavros_msgs__msg__EstimatorStatus(
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

  // member: header
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_std_msgs__msg__Header(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: attitude_status_flag
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: velocity_horiz_status_flag
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: velocity_vert_status_flag
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: pos_horiz_rel_status_flag
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: pos_horiz_abs_status_flag
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: pos_vert_abs_status_flag
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: pos_vert_agl_status_flag
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: const_pos_mode_status_flag
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: pred_pos_horiz_rel_status_flag
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: pred_pos_horiz_abs_status_flag
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: gps_glitch_status_flag
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: accel_error_status_flag
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = mavros_msgs__msg__EstimatorStatus;
    is_plain =
      (
      offsetof(DataType, accel_error_status_flag) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _EstimatorStatus__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_mavros_msgs__msg__EstimatorStatus(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_EstimatorStatus = {
  "mavros_msgs::msg",
  "EstimatorStatus",
  _EstimatorStatus__cdr_serialize,
  _EstimatorStatus__cdr_deserialize,
  _EstimatorStatus__get_serialized_size,
  _EstimatorStatus__max_serialized_size
};

static rosidl_message_type_support_t _EstimatorStatus__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_EstimatorStatus,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, mavros_msgs, msg, EstimatorStatus)() {
  return &_EstimatorStatus__type_support;
}

#if defined(__cplusplus)
}
#endif
