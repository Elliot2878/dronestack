// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from mavros_msgs:srv/FileWrite.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "mavros_msgs/srv/detail/file_write__rosidl_typesupport_introspection_c.h"
#include "mavros_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "mavros_msgs/srv/detail/file_write__functions.h"
#include "mavros_msgs/srv/detail/file_write__struct.h"


// Include directives for member types
// Member `file_path`
#include "rosidl_runtime_c/string_functions.h"
// Member `data`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void mavros_msgs__srv__FileWrite_Request__rosidl_typesupport_introspection_c__FileWrite_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  mavros_msgs__srv__FileWrite_Request__init(message_memory);
}

void mavros_msgs__srv__FileWrite_Request__rosidl_typesupport_introspection_c__FileWrite_Request_fini_function(void * message_memory)
{
  mavros_msgs__srv__FileWrite_Request__fini(message_memory);
}

size_t mavros_msgs__srv__FileWrite_Request__rosidl_typesupport_introspection_c__size_function__FileWrite_Request__data(
  const void * untyped_member)
{
  const rosidl_runtime_c__uint8__Sequence * member =
    (const rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return member->size;
}

const void * mavros_msgs__srv__FileWrite_Request__rosidl_typesupport_introspection_c__get_const_function__FileWrite_Request__data(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__uint8__Sequence * member =
    (const rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return &member->data[index];
}

void * mavros_msgs__srv__FileWrite_Request__rosidl_typesupport_introspection_c__get_function__FileWrite_Request__data(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__uint8__Sequence * member =
    (rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return &member->data[index];
}

void mavros_msgs__srv__FileWrite_Request__rosidl_typesupport_introspection_c__fetch_function__FileWrite_Request__data(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint8_t * item =
    ((const uint8_t *)
    mavros_msgs__srv__FileWrite_Request__rosidl_typesupport_introspection_c__get_const_function__FileWrite_Request__data(untyped_member, index));
  uint8_t * value =
    (uint8_t *)(untyped_value);
  *value = *item;
}

void mavros_msgs__srv__FileWrite_Request__rosidl_typesupport_introspection_c__assign_function__FileWrite_Request__data(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint8_t * item =
    ((uint8_t *)
    mavros_msgs__srv__FileWrite_Request__rosidl_typesupport_introspection_c__get_function__FileWrite_Request__data(untyped_member, index));
  const uint8_t * value =
    (const uint8_t *)(untyped_value);
  *item = *value;
}

bool mavros_msgs__srv__FileWrite_Request__rosidl_typesupport_introspection_c__resize_function__FileWrite_Request__data(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__uint8__Sequence * member =
    (rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  rosidl_runtime_c__uint8__Sequence__fini(member);
  return rosidl_runtime_c__uint8__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember mavros_msgs__srv__FileWrite_Request__rosidl_typesupport_introspection_c__FileWrite_Request_message_member_array[3] = {
  {
    "file_path",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mavros_msgs__srv__FileWrite_Request, file_path),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "offset",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mavros_msgs__srv__FileWrite_Request, offset),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "data",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mavros_msgs__srv__FileWrite_Request, data),  // bytes offset in struct
    NULL,  // default value
    mavros_msgs__srv__FileWrite_Request__rosidl_typesupport_introspection_c__size_function__FileWrite_Request__data,  // size() function pointer
    mavros_msgs__srv__FileWrite_Request__rosidl_typesupport_introspection_c__get_const_function__FileWrite_Request__data,  // get_const(index) function pointer
    mavros_msgs__srv__FileWrite_Request__rosidl_typesupport_introspection_c__get_function__FileWrite_Request__data,  // get(index) function pointer
    mavros_msgs__srv__FileWrite_Request__rosidl_typesupport_introspection_c__fetch_function__FileWrite_Request__data,  // fetch(index, &value) function pointer
    mavros_msgs__srv__FileWrite_Request__rosidl_typesupport_introspection_c__assign_function__FileWrite_Request__data,  // assign(index, value) function pointer
    mavros_msgs__srv__FileWrite_Request__rosidl_typesupport_introspection_c__resize_function__FileWrite_Request__data  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers mavros_msgs__srv__FileWrite_Request__rosidl_typesupport_introspection_c__FileWrite_Request_message_members = {
  "mavros_msgs__srv",  // message namespace
  "FileWrite_Request",  // message name
  3,  // number of fields
  sizeof(mavros_msgs__srv__FileWrite_Request),
  mavros_msgs__srv__FileWrite_Request__rosidl_typesupport_introspection_c__FileWrite_Request_message_member_array,  // message members
  mavros_msgs__srv__FileWrite_Request__rosidl_typesupport_introspection_c__FileWrite_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  mavros_msgs__srv__FileWrite_Request__rosidl_typesupport_introspection_c__FileWrite_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t mavros_msgs__srv__FileWrite_Request__rosidl_typesupport_introspection_c__FileWrite_Request_message_type_support_handle = {
  0,
  &mavros_msgs__srv__FileWrite_Request__rosidl_typesupport_introspection_c__FileWrite_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mavros_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mavros_msgs, srv, FileWrite_Request)() {
  if (!mavros_msgs__srv__FileWrite_Request__rosidl_typesupport_introspection_c__FileWrite_Request_message_type_support_handle.typesupport_identifier) {
    mavros_msgs__srv__FileWrite_Request__rosidl_typesupport_introspection_c__FileWrite_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &mavros_msgs__srv__FileWrite_Request__rosidl_typesupport_introspection_c__FileWrite_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "mavros_msgs/srv/detail/file_write__rosidl_typesupport_introspection_c.h"
// already included above
// #include "mavros_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "mavros_msgs/srv/detail/file_write__functions.h"
// already included above
// #include "mavros_msgs/srv/detail/file_write__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void mavros_msgs__srv__FileWrite_Response__rosidl_typesupport_introspection_c__FileWrite_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  mavros_msgs__srv__FileWrite_Response__init(message_memory);
}

void mavros_msgs__srv__FileWrite_Response__rosidl_typesupport_introspection_c__FileWrite_Response_fini_function(void * message_memory)
{
  mavros_msgs__srv__FileWrite_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember mavros_msgs__srv__FileWrite_Response__rosidl_typesupport_introspection_c__FileWrite_Response_message_member_array[2] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mavros_msgs__srv__FileWrite_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "r_errno",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mavros_msgs__srv__FileWrite_Response, r_errno),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers mavros_msgs__srv__FileWrite_Response__rosidl_typesupport_introspection_c__FileWrite_Response_message_members = {
  "mavros_msgs__srv",  // message namespace
  "FileWrite_Response",  // message name
  2,  // number of fields
  sizeof(mavros_msgs__srv__FileWrite_Response),
  mavros_msgs__srv__FileWrite_Response__rosidl_typesupport_introspection_c__FileWrite_Response_message_member_array,  // message members
  mavros_msgs__srv__FileWrite_Response__rosidl_typesupport_introspection_c__FileWrite_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  mavros_msgs__srv__FileWrite_Response__rosidl_typesupport_introspection_c__FileWrite_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t mavros_msgs__srv__FileWrite_Response__rosidl_typesupport_introspection_c__FileWrite_Response_message_type_support_handle = {
  0,
  &mavros_msgs__srv__FileWrite_Response__rosidl_typesupport_introspection_c__FileWrite_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mavros_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mavros_msgs, srv, FileWrite_Response)() {
  if (!mavros_msgs__srv__FileWrite_Response__rosidl_typesupport_introspection_c__FileWrite_Response_message_type_support_handle.typesupport_identifier) {
    mavros_msgs__srv__FileWrite_Response__rosidl_typesupport_introspection_c__FileWrite_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &mavros_msgs__srv__FileWrite_Response__rosidl_typesupport_introspection_c__FileWrite_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "mavros_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "mavros_msgs/srv/detail/file_write__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers mavros_msgs__srv__detail__file_write__rosidl_typesupport_introspection_c__FileWrite_service_members = {
  "mavros_msgs__srv",  // service namespace
  "FileWrite",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // mavros_msgs__srv__detail__file_write__rosidl_typesupport_introspection_c__FileWrite_Request_message_type_support_handle,
  NULL  // response message
  // mavros_msgs__srv__detail__file_write__rosidl_typesupport_introspection_c__FileWrite_Response_message_type_support_handle
};

static rosidl_service_type_support_t mavros_msgs__srv__detail__file_write__rosidl_typesupport_introspection_c__FileWrite_service_type_support_handle = {
  0,
  &mavros_msgs__srv__detail__file_write__rosidl_typesupport_introspection_c__FileWrite_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mavros_msgs, srv, FileWrite_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mavros_msgs, srv, FileWrite_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mavros_msgs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mavros_msgs, srv, FileWrite)() {
  if (!mavros_msgs__srv__detail__file_write__rosidl_typesupport_introspection_c__FileWrite_service_type_support_handle.typesupport_identifier) {
    mavros_msgs__srv__detail__file_write__rosidl_typesupport_introspection_c__FileWrite_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)mavros_msgs__srv__detail__file_write__rosidl_typesupport_introspection_c__FileWrite_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mavros_msgs, srv, FileWrite_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mavros_msgs, srv, FileWrite_Response)()->data;
  }

  return &mavros_msgs__srv__detail__file_write__rosidl_typesupport_introspection_c__FileWrite_service_type_support_handle;
}
