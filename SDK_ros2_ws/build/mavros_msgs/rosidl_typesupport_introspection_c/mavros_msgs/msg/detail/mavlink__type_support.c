// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from mavros_msgs:msg/Mavlink.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "mavros_msgs/msg/detail/mavlink__rosidl_typesupport_introspection_c.h"
#include "mavros_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "mavros_msgs/msg/detail/mavlink__functions.h"
#include "mavros_msgs/msg/detail/mavlink__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `payload64`
// Member `signature`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void mavros_msgs__msg__Mavlink__rosidl_typesupport_introspection_c__Mavlink_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  mavros_msgs__msg__Mavlink__init(message_memory);
}

void mavros_msgs__msg__Mavlink__rosidl_typesupport_introspection_c__Mavlink_fini_function(void * message_memory)
{
  mavros_msgs__msg__Mavlink__fini(message_memory);
}

size_t mavros_msgs__msg__Mavlink__rosidl_typesupport_introspection_c__size_function__Mavlink__payload64(
  const void * untyped_member)
{
  const rosidl_runtime_c__uint64__Sequence * member =
    (const rosidl_runtime_c__uint64__Sequence *)(untyped_member);
  return member->size;
}

const void * mavros_msgs__msg__Mavlink__rosidl_typesupport_introspection_c__get_const_function__Mavlink__payload64(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__uint64__Sequence * member =
    (const rosidl_runtime_c__uint64__Sequence *)(untyped_member);
  return &member->data[index];
}

void * mavros_msgs__msg__Mavlink__rosidl_typesupport_introspection_c__get_function__Mavlink__payload64(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__uint64__Sequence * member =
    (rosidl_runtime_c__uint64__Sequence *)(untyped_member);
  return &member->data[index];
}

void mavros_msgs__msg__Mavlink__rosidl_typesupport_introspection_c__fetch_function__Mavlink__payload64(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint64_t * item =
    ((const uint64_t *)
    mavros_msgs__msg__Mavlink__rosidl_typesupport_introspection_c__get_const_function__Mavlink__payload64(untyped_member, index));
  uint64_t * value =
    (uint64_t *)(untyped_value);
  *value = *item;
}

void mavros_msgs__msg__Mavlink__rosidl_typesupport_introspection_c__assign_function__Mavlink__payload64(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint64_t * item =
    ((uint64_t *)
    mavros_msgs__msg__Mavlink__rosidl_typesupport_introspection_c__get_function__Mavlink__payload64(untyped_member, index));
  const uint64_t * value =
    (const uint64_t *)(untyped_value);
  *item = *value;
}

bool mavros_msgs__msg__Mavlink__rosidl_typesupport_introspection_c__resize_function__Mavlink__payload64(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__uint64__Sequence * member =
    (rosidl_runtime_c__uint64__Sequence *)(untyped_member);
  rosidl_runtime_c__uint64__Sequence__fini(member);
  return rosidl_runtime_c__uint64__Sequence__init(member, size);
}

size_t mavros_msgs__msg__Mavlink__rosidl_typesupport_introspection_c__size_function__Mavlink__signature(
  const void * untyped_member)
{
  const rosidl_runtime_c__uint8__Sequence * member =
    (const rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return member->size;
}

const void * mavros_msgs__msg__Mavlink__rosidl_typesupport_introspection_c__get_const_function__Mavlink__signature(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__uint8__Sequence * member =
    (const rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return &member->data[index];
}

void * mavros_msgs__msg__Mavlink__rosidl_typesupport_introspection_c__get_function__Mavlink__signature(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__uint8__Sequence * member =
    (rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return &member->data[index];
}

void mavros_msgs__msg__Mavlink__rosidl_typesupport_introspection_c__fetch_function__Mavlink__signature(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint8_t * item =
    ((const uint8_t *)
    mavros_msgs__msg__Mavlink__rosidl_typesupport_introspection_c__get_const_function__Mavlink__signature(untyped_member, index));
  uint8_t * value =
    (uint8_t *)(untyped_value);
  *value = *item;
}

void mavros_msgs__msg__Mavlink__rosidl_typesupport_introspection_c__assign_function__Mavlink__signature(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint8_t * item =
    ((uint8_t *)
    mavros_msgs__msg__Mavlink__rosidl_typesupport_introspection_c__get_function__Mavlink__signature(untyped_member, index));
  const uint8_t * value =
    (const uint8_t *)(untyped_value);
  *item = *value;
}

bool mavros_msgs__msg__Mavlink__rosidl_typesupport_introspection_c__resize_function__Mavlink__signature(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__uint8__Sequence * member =
    (rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  rosidl_runtime_c__uint8__Sequence__fini(member);
  return rosidl_runtime_c__uint8__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember mavros_msgs__msg__Mavlink__rosidl_typesupport_introspection_c__Mavlink_message_member_array[13] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mavros_msgs__msg__Mavlink, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "framing_status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mavros_msgs__msg__Mavlink, framing_status),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "magic",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mavros_msgs__msg__Mavlink, magic),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "len",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mavros_msgs__msg__Mavlink, len),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "incompat_flags",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mavros_msgs__msg__Mavlink, incompat_flags),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "compat_flags",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mavros_msgs__msg__Mavlink, compat_flags),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "seq",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mavros_msgs__msg__Mavlink, seq),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "sysid",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mavros_msgs__msg__Mavlink, sysid),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "compid",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mavros_msgs__msg__Mavlink, compid),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "msgid",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mavros_msgs__msg__Mavlink, msgid),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "checksum",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mavros_msgs__msg__Mavlink, checksum),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "payload64",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    33,  // array size
    true,  // is upper bound
    offsetof(mavros_msgs__msg__Mavlink, payload64),  // bytes offset in struct
    NULL,  // default value
    mavros_msgs__msg__Mavlink__rosidl_typesupport_introspection_c__size_function__Mavlink__payload64,  // size() function pointer
    mavros_msgs__msg__Mavlink__rosidl_typesupport_introspection_c__get_const_function__Mavlink__payload64,  // get_const(index) function pointer
    mavros_msgs__msg__Mavlink__rosidl_typesupport_introspection_c__get_function__Mavlink__payload64,  // get(index) function pointer
    mavros_msgs__msg__Mavlink__rosidl_typesupport_introspection_c__fetch_function__Mavlink__payload64,  // fetch(index, &value) function pointer
    mavros_msgs__msg__Mavlink__rosidl_typesupport_introspection_c__assign_function__Mavlink__payload64,  // assign(index, value) function pointer
    mavros_msgs__msg__Mavlink__rosidl_typesupport_introspection_c__resize_function__Mavlink__payload64  // resize(index) function pointer
  },
  {
    "signature",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    13,  // array size
    true,  // is upper bound
    offsetof(mavros_msgs__msg__Mavlink, signature),  // bytes offset in struct
    NULL,  // default value
    mavros_msgs__msg__Mavlink__rosidl_typesupport_introspection_c__size_function__Mavlink__signature,  // size() function pointer
    mavros_msgs__msg__Mavlink__rosidl_typesupport_introspection_c__get_const_function__Mavlink__signature,  // get_const(index) function pointer
    mavros_msgs__msg__Mavlink__rosidl_typesupport_introspection_c__get_function__Mavlink__signature,  // get(index) function pointer
    mavros_msgs__msg__Mavlink__rosidl_typesupport_introspection_c__fetch_function__Mavlink__signature,  // fetch(index, &value) function pointer
    mavros_msgs__msg__Mavlink__rosidl_typesupport_introspection_c__assign_function__Mavlink__signature,  // assign(index, value) function pointer
    mavros_msgs__msg__Mavlink__rosidl_typesupport_introspection_c__resize_function__Mavlink__signature  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers mavros_msgs__msg__Mavlink__rosidl_typesupport_introspection_c__Mavlink_message_members = {
  "mavros_msgs__msg",  // message namespace
  "Mavlink",  // message name
  13,  // number of fields
  sizeof(mavros_msgs__msg__Mavlink),
  mavros_msgs__msg__Mavlink__rosidl_typesupport_introspection_c__Mavlink_message_member_array,  // message members
  mavros_msgs__msg__Mavlink__rosidl_typesupport_introspection_c__Mavlink_init_function,  // function to initialize message memory (memory has to be allocated)
  mavros_msgs__msg__Mavlink__rosidl_typesupport_introspection_c__Mavlink_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t mavros_msgs__msg__Mavlink__rosidl_typesupport_introspection_c__Mavlink_message_type_support_handle = {
  0,
  &mavros_msgs__msg__Mavlink__rosidl_typesupport_introspection_c__Mavlink_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mavros_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mavros_msgs, msg, Mavlink)() {
  mavros_msgs__msg__Mavlink__rosidl_typesupport_introspection_c__Mavlink_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!mavros_msgs__msg__Mavlink__rosidl_typesupport_introspection_c__Mavlink_message_type_support_handle.typesupport_identifier) {
    mavros_msgs__msg__Mavlink__rosidl_typesupport_introspection_c__Mavlink_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &mavros_msgs__msg__Mavlink__rosidl_typesupport_introspection_c__Mavlink_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
