// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from mavros_msgs:msg/Trajectory.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "mavros_msgs/msg/detail/trajectory__rosidl_typesupport_introspection_c.h"
#include "mavros_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "mavros_msgs/msg/detail/trajectory__functions.h"
#include "mavros_msgs/msg/detail/trajectory__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `point_1`
// Member `point_2`
// Member `point_3`
// Member `point_4`
// Member `point_5`
#include "mavros_msgs/msg/position_target.h"
// Member `point_1`
// Member `point_2`
// Member `point_3`
// Member `point_4`
// Member `point_5`
#include "mavros_msgs/msg/detail/position_target__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void mavros_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__Trajectory_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  mavros_msgs__msg__Trajectory__init(message_memory);
}

void mavros_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__Trajectory_fini_function(void * message_memory)
{
  mavros_msgs__msg__Trajectory__fini(message_memory);
}

size_t mavros_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__size_function__Trajectory__point_valid(
  const void * untyped_member)
{
  (void)untyped_member;
  return 5;
}

const void * mavros_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__get_const_function__Trajectory__point_valid(
  const void * untyped_member, size_t index)
{
  const uint8_t * member =
    (const uint8_t *)(untyped_member);
  return &member[index];
}

void * mavros_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__get_function__Trajectory__point_valid(
  void * untyped_member, size_t index)
{
  uint8_t * member =
    (uint8_t *)(untyped_member);
  return &member[index];
}

void mavros_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__fetch_function__Trajectory__point_valid(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint8_t * item =
    ((const uint8_t *)
    mavros_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__get_const_function__Trajectory__point_valid(untyped_member, index));
  uint8_t * value =
    (uint8_t *)(untyped_value);
  *value = *item;
}

void mavros_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__assign_function__Trajectory__point_valid(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint8_t * item =
    ((uint8_t *)
    mavros_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__get_function__Trajectory__point_valid(untyped_member, index));
  const uint8_t * value =
    (const uint8_t *)(untyped_value);
  *item = *value;
}

size_t mavros_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__size_function__Trajectory__command(
  const void * untyped_member)
{
  (void)untyped_member;
  return 5;
}

const void * mavros_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__get_const_function__Trajectory__command(
  const void * untyped_member, size_t index)
{
  const uint16_t * member =
    (const uint16_t *)(untyped_member);
  return &member[index];
}

void * mavros_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__get_function__Trajectory__command(
  void * untyped_member, size_t index)
{
  uint16_t * member =
    (uint16_t *)(untyped_member);
  return &member[index];
}

void mavros_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__fetch_function__Trajectory__command(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint16_t * item =
    ((const uint16_t *)
    mavros_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__get_const_function__Trajectory__command(untyped_member, index));
  uint16_t * value =
    (uint16_t *)(untyped_value);
  *value = *item;
}

void mavros_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__assign_function__Trajectory__command(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint16_t * item =
    ((uint16_t *)
    mavros_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__get_function__Trajectory__command(untyped_member, index));
  const uint16_t * value =
    (const uint16_t *)(untyped_value);
  *item = *value;
}

size_t mavros_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__size_function__Trajectory__time_horizon(
  const void * untyped_member)
{
  (void)untyped_member;
  return 5;
}

const void * mavros_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__get_const_function__Trajectory__time_horizon(
  const void * untyped_member, size_t index)
{
  const float * member =
    (const float *)(untyped_member);
  return &member[index];
}

void * mavros_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__get_function__Trajectory__time_horizon(
  void * untyped_member, size_t index)
{
  float * member =
    (float *)(untyped_member);
  return &member[index];
}

void mavros_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__fetch_function__Trajectory__time_horizon(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    mavros_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__get_const_function__Trajectory__time_horizon(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void mavros_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__assign_function__Trajectory__time_horizon(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    mavros_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__get_function__Trajectory__time_horizon(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember mavros_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__Trajectory_message_member_array[10] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mavros_msgs__msg__Trajectory, header),  // bytes offset in struct
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
    offsetof(mavros_msgs__msg__Trajectory, type),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "point_1",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mavros_msgs__msg__Trajectory, point_1),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "point_2",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mavros_msgs__msg__Trajectory, point_2),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "point_3",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mavros_msgs__msg__Trajectory, point_3),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "point_4",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mavros_msgs__msg__Trajectory, point_4),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "point_5",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mavros_msgs__msg__Trajectory, point_5),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "point_valid",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    5,  // array size
    false,  // is upper bound
    offsetof(mavros_msgs__msg__Trajectory, point_valid),  // bytes offset in struct
    NULL,  // default value
    mavros_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__size_function__Trajectory__point_valid,  // size() function pointer
    mavros_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__get_const_function__Trajectory__point_valid,  // get_const(index) function pointer
    mavros_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__get_function__Trajectory__point_valid,  // get(index) function pointer
    mavros_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__fetch_function__Trajectory__point_valid,  // fetch(index, &value) function pointer
    mavros_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__assign_function__Trajectory__point_valid,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "command",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    5,  // array size
    false,  // is upper bound
    offsetof(mavros_msgs__msg__Trajectory, command),  // bytes offset in struct
    NULL,  // default value
    mavros_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__size_function__Trajectory__command,  // size() function pointer
    mavros_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__get_const_function__Trajectory__command,  // get_const(index) function pointer
    mavros_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__get_function__Trajectory__command,  // get(index) function pointer
    mavros_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__fetch_function__Trajectory__command,  // fetch(index, &value) function pointer
    mavros_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__assign_function__Trajectory__command,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "time_horizon",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    5,  // array size
    false,  // is upper bound
    offsetof(mavros_msgs__msg__Trajectory, time_horizon),  // bytes offset in struct
    NULL,  // default value
    mavros_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__size_function__Trajectory__time_horizon,  // size() function pointer
    mavros_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__get_const_function__Trajectory__time_horizon,  // get_const(index) function pointer
    mavros_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__get_function__Trajectory__time_horizon,  // get(index) function pointer
    mavros_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__fetch_function__Trajectory__time_horizon,  // fetch(index, &value) function pointer
    mavros_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__assign_function__Trajectory__time_horizon,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers mavros_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__Trajectory_message_members = {
  "mavros_msgs__msg",  // message namespace
  "Trajectory",  // message name
  10,  // number of fields
  sizeof(mavros_msgs__msg__Trajectory),
  mavros_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__Trajectory_message_member_array,  // message members
  mavros_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__Trajectory_init_function,  // function to initialize message memory (memory has to be allocated)
  mavros_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__Trajectory_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t mavros_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__Trajectory_message_type_support_handle = {
  0,
  &mavros_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__Trajectory_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mavros_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mavros_msgs, msg, Trajectory)() {
  mavros_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__Trajectory_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  mavros_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__Trajectory_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mavros_msgs, msg, PositionTarget)();
  mavros_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__Trajectory_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mavros_msgs, msg, PositionTarget)();
  mavros_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__Trajectory_message_member_array[4].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mavros_msgs, msg, PositionTarget)();
  mavros_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__Trajectory_message_member_array[5].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mavros_msgs, msg, PositionTarget)();
  mavros_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__Trajectory_message_member_array[6].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mavros_msgs, msg, PositionTarget)();
  if (!mavros_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__Trajectory_message_type_support_handle.typesupport_identifier) {
    mavros_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__Trajectory_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &mavros_msgs__msg__Trajectory__rosidl_typesupport_introspection_c__Trajectory_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
