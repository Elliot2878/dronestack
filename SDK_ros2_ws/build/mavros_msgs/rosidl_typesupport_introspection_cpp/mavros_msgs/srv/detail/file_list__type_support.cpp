// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from mavros_msgs:srv/FileList.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "mavros_msgs/srv/detail/file_list__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace mavros_msgs
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void FileList_Request_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) mavros_msgs::srv::FileList_Request(_init);
}

void FileList_Request_fini_function(void * message_memory)
{
  auto typed_message = static_cast<mavros_msgs::srv::FileList_Request *>(message_memory);
  typed_message->~FileList_Request();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember FileList_Request_message_member_array[1] = {
  {
    "dir_path",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mavros_msgs::srv::FileList_Request, dir_path),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers FileList_Request_message_members = {
  "mavros_msgs::srv",  // message namespace
  "FileList_Request",  // message name
  1,  // number of fields
  sizeof(mavros_msgs::srv::FileList_Request),
  FileList_Request_message_member_array,  // message members
  FileList_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  FileList_Request_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t FileList_Request_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &FileList_Request_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace mavros_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<mavros_msgs::srv::FileList_Request>()
{
  return &::mavros_msgs::srv::rosidl_typesupport_introspection_cpp::FileList_Request_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mavros_msgs, srv, FileList_Request)() {
  return &::mavros_msgs::srv::rosidl_typesupport_introspection_cpp::FileList_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "mavros_msgs/srv/detail/file_list__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace mavros_msgs
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void FileList_Response_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) mavros_msgs::srv::FileList_Response(_init);
}

void FileList_Response_fini_function(void * message_memory)
{
  auto typed_message = static_cast<mavros_msgs::srv::FileList_Response *>(message_memory);
  typed_message->~FileList_Response();
}

size_t size_function__FileList_Response__list(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<mavros_msgs::msg::FileEntry> *>(untyped_member);
  return member->size();
}

const void * get_const_function__FileList_Response__list(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<mavros_msgs::msg::FileEntry> *>(untyped_member);
  return &member[index];
}

void * get_function__FileList_Response__list(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<mavros_msgs::msg::FileEntry> *>(untyped_member);
  return &member[index];
}

void fetch_function__FileList_Response__list(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const mavros_msgs::msg::FileEntry *>(
    get_const_function__FileList_Response__list(untyped_member, index));
  auto & value = *reinterpret_cast<mavros_msgs::msg::FileEntry *>(untyped_value);
  value = item;
}

void assign_function__FileList_Response__list(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<mavros_msgs::msg::FileEntry *>(
    get_function__FileList_Response__list(untyped_member, index));
  const auto & value = *reinterpret_cast<const mavros_msgs::msg::FileEntry *>(untyped_value);
  item = value;
}

void resize_function__FileList_Response__list(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<mavros_msgs::msg::FileEntry> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember FileList_Response_message_member_array[3] = {
  {
    "list",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<mavros_msgs::msg::FileEntry>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mavros_msgs::srv::FileList_Response, list),  // bytes offset in struct
    nullptr,  // default value
    size_function__FileList_Response__list,  // size() function pointer
    get_const_function__FileList_Response__list,  // get_const(index) function pointer
    get_function__FileList_Response__list,  // get(index) function pointer
    fetch_function__FileList_Response__list,  // fetch(index, &value) function pointer
    assign_function__FileList_Response__list,  // assign(index, value) function pointer
    resize_function__FileList_Response__list  // resize(index) function pointer
  },
  {
    "success",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mavros_msgs::srv::FileList_Response, success),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "r_errno",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mavros_msgs::srv::FileList_Response, r_errno),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers FileList_Response_message_members = {
  "mavros_msgs::srv",  // message namespace
  "FileList_Response",  // message name
  3,  // number of fields
  sizeof(mavros_msgs::srv::FileList_Response),
  FileList_Response_message_member_array,  // message members
  FileList_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  FileList_Response_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t FileList_Response_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &FileList_Response_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace mavros_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<mavros_msgs::srv::FileList_Response>()
{
  return &::mavros_msgs::srv::rosidl_typesupport_introspection_cpp::FileList_Response_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mavros_msgs, srv, FileList_Response)() {
  return &::mavros_msgs::srv::rosidl_typesupport_introspection_cpp::FileList_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"
// already included above
// #include "mavros_msgs/srv/detail/file_list__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/service_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/service_type_support_decl.hpp"

namespace mavros_msgs
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

// this is intentionally not const to allow initialization later to prevent an initialization race
static ::rosidl_typesupport_introspection_cpp::ServiceMembers FileList_service_members = {
  "mavros_msgs::srv",  // service namespace
  "FileList",  // service name
  // these two fields are initialized below on the first access
  // see get_service_type_support_handle<mavros_msgs::srv::FileList>()
  nullptr,  // request message
  nullptr  // response message
};

static const rosidl_service_type_support_t FileList_service_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &FileList_service_members,
  get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace mavros_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<mavros_msgs::srv::FileList>()
{
  // get a handle to the value to be returned
  auto service_type_support =
    &::mavros_msgs::srv::rosidl_typesupport_introspection_cpp::FileList_service_type_support_handle;
  // get a non-const and properly typed version of the data void *
  auto service_members = const_cast<::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
    static_cast<const ::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
      service_type_support->data));
  // make sure that both the request_members_ and the response_members_ are initialized
  // if they are not, initialize them
  if (
    service_members->request_members_ == nullptr ||
    service_members->response_members_ == nullptr)
  {
    // initialize the request_members_ with the static function from the external library
    service_members->request_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::mavros_msgs::srv::FileList_Request
      >()->data
      );
    // initialize the response_members_ with the static function from the external library
    service_members->response_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::mavros_msgs::srv::FileList_Response
      >()->data
      );
  }
  // finally return the properly initialized service_type_support handle
  return service_type_support;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mavros_msgs, srv, FileList)() {
  return ::rosidl_typesupport_introspection_cpp::get_service_type_support_handle<mavros_msgs::srv::FileList>();
}

#ifdef __cplusplus
}
#endif
