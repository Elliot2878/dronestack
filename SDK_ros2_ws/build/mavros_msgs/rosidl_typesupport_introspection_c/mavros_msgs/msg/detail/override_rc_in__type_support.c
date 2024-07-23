// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from mavros_msgs:msg/OverrideRCIn.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "mavros_msgs/msg/detail/override_rc_in__rosidl_typesupport_introspection_c.h"
#include "mavros_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "mavros_msgs/msg/detail/override_rc_in__functions.h"
#include "mavros_msgs/msg/detail/override_rc_in__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void mavros_msgs__msg__OverrideRCIn__rosidl_typesupport_introspection_c__OverrideRCIn_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  mavros_msgs__msg__OverrideRCIn__init(message_memory);
}

void mavros_msgs__msg__OverrideRCIn__rosidl_typesupport_introspection_c__OverrideRCIn_fini_function(void * message_memory)
{
  mavros_msgs__msg__OverrideRCIn__fini(message_memory);
}

size_t mavros_msgs__msg__OverrideRCIn__rosidl_typesupport_introspection_c__size_function__OverrideRCIn__channels(
  const void * untyped_member)
{
  (void)untyped_member;
  return 18;
}

const void * mavros_msgs__msg__OverrideRCIn__rosidl_typesupport_introspection_c__get_const_function__OverrideRCIn__channels(
  const void * untyped_member, size_t index)
{
  const uint16_t * member =
    (const uint16_t *)(untyped_member);
  return &member[index];
}

void * mavros_msgs__msg__OverrideRCIn__rosidl_typesupport_introspection_c__get_function__OverrideRCIn__channels(
  void * untyped_member, size_t index)
{
  uint16_t * member =
    (uint16_t *)(untyped_member);
  return &member[index];
}

void mavros_msgs__msg__OverrideRCIn__rosidl_typesupport_introspection_c__fetch_function__OverrideRCIn__channels(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint16_t * item =
    ((const uint16_t *)
    mavros_msgs__msg__OverrideRCIn__rosidl_typesupport_introspection_c__get_const_function__OverrideRCIn__channels(untyped_member, index));
  uint16_t * value =
    (uint16_t *)(untyped_value);
  *value = *item;
}

void mavros_msgs__msg__OverrideRCIn__rosidl_typesupport_introspection_c__assign_function__OverrideRCIn__channels(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint16_t * item =
    ((uint16_t *)
    mavros_msgs__msg__OverrideRCIn__rosidl_typesupport_introspection_c__get_function__OverrideRCIn__channels(untyped_member, index));
  const uint16_t * value =
    (const uint16_t *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember mavros_msgs__msg__OverrideRCIn__rosidl_typesupport_introspection_c__OverrideRCIn_message_member_array[1] = {
  {
    "channels",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    18,  // array size
    false,  // is upper bound
    offsetof(mavros_msgs__msg__OverrideRCIn, channels),  // bytes offset in struct
    NULL,  // default value
    mavros_msgs__msg__OverrideRCIn__rosidl_typesupport_introspection_c__size_function__OverrideRCIn__channels,  // size() function pointer
    mavros_msgs__msg__OverrideRCIn__rosidl_typesupport_introspection_c__get_const_function__OverrideRCIn__channels,  // get_const(index) function pointer
    mavros_msgs__msg__OverrideRCIn__rosidl_typesupport_introspection_c__get_function__OverrideRCIn__channels,  // get(index) function pointer
    mavros_msgs__msg__OverrideRCIn__rosidl_typesupport_introspection_c__fetch_function__OverrideRCIn__channels,  // fetch(index, &value) function pointer
    mavros_msgs__msg__OverrideRCIn__rosidl_typesupport_introspection_c__assign_function__OverrideRCIn__channels,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers mavros_msgs__msg__OverrideRCIn__rosidl_typesupport_introspection_c__OverrideRCIn_message_members = {
  "mavros_msgs__msg",  // message namespace
  "OverrideRCIn",  // message name
  1,  // number of fields
  sizeof(mavros_msgs__msg__OverrideRCIn),
  mavros_msgs__msg__OverrideRCIn__rosidl_typesupport_introspection_c__OverrideRCIn_message_member_array,  // message members
  mavros_msgs__msg__OverrideRCIn__rosidl_typesupport_introspection_c__OverrideRCIn_init_function,  // function to initialize message memory (memory has to be allocated)
  mavros_msgs__msg__OverrideRCIn__rosidl_typesupport_introspection_c__OverrideRCIn_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t mavros_msgs__msg__OverrideRCIn__rosidl_typesupport_introspection_c__OverrideRCIn_message_type_support_handle = {
  0,
  &mavros_msgs__msg__OverrideRCIn__rosidl_typesupport_introspection_c__OverrideRCIn_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mavros_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mavros_msgs, msg, OverrideRCIn)() {
  if (!mavros_msgs__msg__OverrideRCIn__rosidl_typesupport_introspection_c__OverrideRCIn_message_type_support_handle.typesupport_identifier) {
    mavros_msgs__msg__OverrideRCIn__rosidl_typesupport_introspection_c__OverrideRCIn_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &mavros_msgs__msg__OverrideRCIn__rosidl_typesupport_introspection_c__OverrideRCIn_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
