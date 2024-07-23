// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from mavros_msgs:msg/RCIn.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "mavros_msgs/msg/detail/rc_in__rosidl_typesupport_introspection_c.h"
#include "mavros_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "mavros_msgs/msg/detail/rc_in__functions.h"
#include "mavros_msgs/msg/detail/rc_in__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `channels`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void mavros_msgs__msg__RCIn__rosidl_typesupport_introspection_c__RCIn_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  mavros_msgs__msg__RCIn__init(message_memory);
}

void mavros_msgs__msg__RCIn__rosidl_typesupport_introspection_c__RCIn_fini_function(void * message_memory)
{
  mavros_msgs__msg__RCIn__fini(message_memory);
}

size_t mavros_msgs__msg__RCIn__rosidl_typesupport_introspection_c__size_function__RCIn__channels(
  const void * untyped_member)
{
  const rosidl_runtime_c__uint16__Sequence * member =
    (const rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  return member->size;
}

const void * mavros_msgs__msg__RCIn__rosidl_typesupport_introspection_c__get_const_function__RCIn__channels(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__uint16__Sequence * member =
    (const rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  return &member->data[index];
}

void * mavros_msgs__msg__RCIn__rosidl_typesupport_introspection_c__get_function__RCIn__channels(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__uint16__Sequence * member =
    (rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  return &member->data[index];
}

void mavros_msgs__msg__RCIn__rosidl_typesupport_introspection_c__fetch_function__RCIn__channels(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint16_t * item =
    ((const uint16_t *)
    mavros_msgs__msg__RCIn__rosidl_typesupport_introspection_c__get_const_function__RCIn__channels(untyped_member, index));
  uint16_t * value =
    (uint16_t *)(untyped_value);
  *value = *item;
}

void mavros_msgs__msg__RCIn__rosidl_typesupport_introspection_c__assign_function__RCIn__channels(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint16_t * item =
    ((uint16_t *)
    mavros_msgs__msg__RCIn__rosidl_typesupport_introspection_c__get_function__RCIn__channels(untyped_member, index));
  const uint16_t * value =
    (const uint16_t *)(untyped_value);
  *item = *value;
}

bool mavros_msgs__msg__RCIn__rosidl_typesupport_introspection_c__resize_function__RCIn__channels(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__uint16__Sequence * member =
    (rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  rosidl_runtime_c__uint16__Sequence__fini(member);
  return rosidl_runtime_c__uint16__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember mavros_msgs__msg__RCIn__rosidl_typesupport_introspection_c__RCIn_message_member_array[3] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mavros_msgs__msg__RCIn, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "rssi",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mavros_msgs__msg__RCIn, rssi),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "channels",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mavros_msgs__msg__RCIn, channels),  // bytes offset in struct
    NULL,  // default value
    mavros_msgs__msg__RCIn__rosidl_typesupport_introspection_c__size_function__RCIn__channels,  // size() function pointer
    mavros_msgs__msg__RCIn__rosidl_typesupport_introspection_c__get_const_function__RCIn__channels,  // get_const(index) function pointer
    mavros_msgs__msg__RCIn__rosidl_typesupport_introspection_c__get_function__RCIn__channels,  // get(index) function pointer
    mavros_msgs__msg__RCIn__rosidl_typesupport_introspection_c__fetch_function__RCIn__channels,  // fetch(index, &value) function pointer
    mavros_msgs__msg__RCIn__rosidl_typesupport_introspection_c__assign_function__RCIn__channels,  // assign(index, value) function pointer
    mavros_msgs__msg__RCIn__rosidl_typesupport_introspection_c__resize_function__RCIn__channels  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers mavros_msgs__msg__RCIn__rosidl_typesupport_introspection_c__RCIn_message_members = {
  "mavros_msgs__msg",  // message namespace
  "RCIn",  // message name
  3,  // number of fields
  sizeof(mavros_msgs__msg__RCIn),
  mavros_msgs__msg__RCIn__rosidl_typesupport_introspection_c__RCIn_message_member_array,  // message members
  mavros_msgs__msg__RCIn__rosidl_typesupport_introspection_c__RCIn_init_function,  // function to initialize message memory (memory has to be allocated)
  mavros_msgs__msg__RCIn__rosidl_typesupport_introspection_c__RCIn_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t mavros_msgs__msg__RCIn__rosidl_typesupport_introspection_c__RCIn_message_type_support_handle = {
  0,
  &mavros_msgs__msg__RCIn__rosidl_typesupport_introspection_c__RCIn_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mavros_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mavros_msgs, msg, RCIn)() {
  mavros_msgs__msg__RCIn__rosidl_typesupport_introspection_c__RCIn_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!mavros_msgs__msg__RCIn__rosidl_typesupport_introspection_c__RCIn_message_type_support_handle.typesupport_identifier) {
    mavros_msgs__msg__RCIn__rosidl_typesupport_introspection_c__RCIn_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &mavros_msgs__msg__RCIn__rosidl_typesupport_introspection_c__RCIn_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
