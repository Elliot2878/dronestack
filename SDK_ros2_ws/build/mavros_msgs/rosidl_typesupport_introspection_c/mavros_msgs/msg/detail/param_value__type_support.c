// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from mavros_msgs:msg/ParamValue.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "mavros_msgs/msg/detail/param_value__rosidl_typesupport_introspection_c.h"
#include "mavros_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "mavros_msgs/msg/detail/param_value__functions.h"
#include "mavros_msgs/msg/detail/param_value__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void mavros_msgs__msg__ParamValue__rosidl_typesupport_introspection_c__ParamValue_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  mavros_msgs__msg__ParamValue__init(message_memory);
}

void mavros_msgs__msg__ParamValue__rosidl_typesupport_introspection_c__ParamValue_fini_function(void * message_memory)
{
  mavros_msgs__msg__ParamValue__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember mavros_msgs__msg__ParamValue__rosidl_typesupport_introspection_c__ParamValue_message_member_array[2] = {
  {
    "integer",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mavros_msgs__msg__ParamValue, integer),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "real",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mavros_msgs__msg__ParamValue, real),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers mavros_msgs__msg__ParamValue__rosidl_typesupport_introspection_c__ParamValue_message_members = {
  "mavros_msgs__msg",  // message namespace
  "ParamValue",  // message name
  2,  // number of fields
  sizeof(mavros_msgs__msg__ParamValue),
  mavros_msgs__msg__ParamValue__rosidl_typesupport_introspection_c__ParamValue_message_member_array,  // message members
  mavros_msgs__msg__ParamValue__rosidl_typesupport_introspection_c__ParamValue_init_function,  // function to initialize message memory (memory has to be allocated)
  mavros_msgs__msg__ParamValue__rosidl_typesupport_introspection_c__ParamValue_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t mavros_msgs__msg__ParamValue__rosidl_typesupport_introspection_c__ParamValue_message_type_support_handle = {
  0,
  &mavros_msgs__msg__ParamValue__rosidl_typesupport_introspection_c__ParamValue_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mavros_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mavros_msgs, msg, ParamValue)() {
  if (!mavros_msgs__msg__ParamValue__rosidl_typesupport_introspection_c__ParamValue_message_type_support_handle.typesupport_identifier) {
    mavros_msgs__msg__ParamValue__rosidl_typesupport_introspection_c__ParamValue_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &mavros_msgs__msg__ParamValue__rosidl_typesupport_introspection_c__ParamValue_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
