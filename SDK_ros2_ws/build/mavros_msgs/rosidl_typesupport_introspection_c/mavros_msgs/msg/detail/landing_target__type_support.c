// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from mavros_msgs:msg/LandingTarget.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "mavros_msgs/msg/detail/landing_target__rosidl_typesupport_introspection_c.h"
#include "mavros_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "mavros_msgs/msg/detail/landing_target__functions.h"
#include "mavros_msgs/msg/detail/landing_target__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `pose`
#include "geometry_msgs/msg/pose.h"
// Member `pose`
#include "geometry_msgs/msg/detail/pose__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void mavros_msgs__msg__LandingTarget__rosidl_typesupport_introspection_c__LandingTarget_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  mavros_msgs__msg__LandingTarget__init(message_memory);
}

void mavros_msgs__msg__LandingTarget__rosidl_typesupport_introspection_c__LandingTarget_fini_function(void * message_memory)
{
  mavros_msgs__msg__LandingTarget__fini(message_memory);
}

size_t mavros_msgs__msg__LandingTarget__rosidl_typesupport_introspection_c__size_function__LandingTarget__angle(
  const void * untyped_member)
{
  (void)untyped_member;
  return 2;
}

const void * mavros_msgs__msg__LandingTarget__rosidl_typesupport_introspection_c__get_const_function__LandingTarget__angle(
  const void * untyped_member, size_t index)
{
  const float * member =
    (const float *)(untyped_member);
  return &member[index];
}

void * mavros_msgs__msg__LandingTarget__rosidl_typesupport_introspection_c__get_function__LandingTarget__angle(
  void * untyped_member, size_t index)
{
  float * member =
    (float *)(untyped_member);
  return &member[index];
}

void mavros_msgs__msg__LandingTarget__rosidl_typesupport_introspection_c__fetch_function__LandingTarget__angle(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    mavros_msgs__msg__LandingTarget__rosidl_typesupport_introspection_c__get_const_function__LandingTarget__angle(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void mavros_msgs__msg__LandingTarget__rosidl_typesupport_introspection_c__assign_function__LandingTarget__angle(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    mavros_msgs__msg__LandingTarget__rosidl_typesupport_introspection_c__get_function__LandingTarget__angle(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

size_t mavros_msgs__msg__LandingTarget__rosidl_typesupport_introspection_c__size_function__LandingTarget__size(
  const void * untyped_member)
{
  (void)untyped_member;
  return 2;
}

const void * mavros_msgs__msg__LandingTarget__rosidl_typesupport_introspection_c__get_const_function__LandingTarget__size(
  const void * untyped_member, size_t index)
{
  const float * member =
    (const float *)(untyped_member);
  return &member[index];
}

void * mavros_msgs__msg__LandingTarget__rosidl_typesupport_introspection_c__get_function__LandingTarget__size(
  void * untyped_member, size_t index)
{
  float * member =
    (float *)(untyped_member);
  return &member[index];
}

void mavros_msgs__msg__LandingTarget__rosidl_typesupport_introspection_c__fetch_function__LandingTarget__size(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    mavros_msgs__msg__LandingTarget__rosidl_typesupport_introspection_c__get_const_function__LandingTarget__size(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void mavros_msgs__msg__LandingTarget__rosidl_typesupport_introspection_c__assign_function__LandingTarget__size(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    mavros_msgs__msg__LandingTarget__rosidl_typesupport_introspection_c__get_function__LandingTarget__size(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember mavros_msgs__msg__LandingTarget__rosidl_typesupport_introspection_c__LandingTarget_message_member_array[8] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mavros_msgs__msg__LandingTarget, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "target_num",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mavros_msgs__msg__LandingTarget, target_num),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "frame",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mavros_msgs__msg__LandingTarget, frame),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "angle",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    2,  // array size
    false,  // is upper bound
    offsetof(mavros_msgs__msg__LandingTarget, angle),  // bytes offset in struct
    NULL,  // default value
    mavros_msgs__msg__LandingTarget__rosidl_typesupport_introspection_c__size_function__LandingTarget__angle,  // size() function pointer
    mavros_msgs__msg__LandingTarget__rosidl_typesupport_introspection_c__get_const_function__LandingTarget__angle,  // get_const(index) function pointer
    mavros_msgs__msg__LandingTarget__rosidl_typesupport_introspection_c__get_function__LandingTarget__angle,  // get(index) function pointer
    mavros_msgs__msg__LandingTarget__rosidl_typesupport_introspection_c__fetch_function__LandingTarget__angle,  // fetch(index, &value) function pointer
    mavros_msgs__msg__LandingTarget__rosidl_typesupport_introspection_c__assign_function__LandingTarget__angle,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "distance",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mavros_msgs__msg__LandingTarget, distance),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "size",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    2,  // array size
    false,  // is upper bound
    offsetof(mavros_msgs__msg__LandingTarget, size),  // bytes offset in struct
    NULL,  // default value
    mavros_msgs__msg__LandingTarget__rosidl_typesupport_introspection_c__size_function__LandingTarget__size,  // size() function pointer
    mavros_msgs__msg__LandingTarget__rosidl_typesupport_introspection_c__get_const_function__LandingTarget__size,  // get_const(index) function pointer
    mavros_msgs__msg__LandingTarget__rosidl_typesupport_introspection_c__get_function__LandingTarget__size,  // get(index) function pointer
    mavros_msgs__msg__LandingTarget__rosidl_typesupport_introspection_c__fetch_function__LandingTarget__size,  // fetch(index, &value) function pointer
    mavros_msgs__msg__LandingTarget__rosidl_typesupport_introspection_c__assign_function__LandingTarget__size,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "pose",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mavros_msgs__msg__LandingTarget, pose),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "type",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mavros_msgs__msg__LandingTarget, type),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers mavros_msgs__msg__LandingTarget__rosidl_typesupport_introspection_c__LandingTarget_message_members = {
  "mavros_msgs__msg",  // message namespace
  "LandingTarget",  // message name
  8,  // number of fields
  sizeof(mavros_msgs__msg__LandingTarget),
  mavros_msgs__msg__LandingTarget__rosidl_typesupport_introspection_c__LandingTarget_message_member_array,  // message members
  mavros_msgs__msg__LandingTarget__rosidl_typesupport_introspection_c__LandingTarget_init_function,  // function to initialize message memory (memory has to be allocated)
  mavros_msgs__msg__LandingTarget__rosidl_typesupport_introspection_c__LandingTarget_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t mavros_msgs__msg__LandingTarget__rosidl_typesupport_introspection_c__LandingTarget_message_type_support_handle = {
  0,
  &mavros_msgs__msg__LandingTarget__rosidl_typesupport_introspection_c__LandingTarget_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mavros_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mavros_msgs, msg, LandingTarget)() {
  mavros_msgs__msg__LandingTarget__rosidl_typesupport_introspection_c__LandingTarget_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  mavros_msgs__msg__LandingTarget__rosidl_typesupport_introspection_c__LandingTarget_message_member_array[6].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Pose)();
  if (!mavros_msgs__msg__LandingTarget__rosidl_typesupport_introspection_c__LandingTarget_message_type_support_handle.typesupport_identifier) {
    mavros_msgs__msg__LandingTarget__rosidl_typesupport_introspection_c__LandingTarget_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &mavros_msgs__msg__LandingTarget__rosidl_typesupport_introspection_c__LandingTarget_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
