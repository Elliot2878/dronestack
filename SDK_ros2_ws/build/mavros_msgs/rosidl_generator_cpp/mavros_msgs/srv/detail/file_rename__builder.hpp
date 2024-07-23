// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:srv/FileRename.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__FILE_RENAME__BUILDER_HPP_
#define MAVROS_MSGS__SRV__DETAIL__FILE_RENAME__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/srv/detail/file_rename__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace srv
{

namespace builder
{

class Init_FileRename_Request_new_path
{
public:
  explicit Init_FileRename_Request_new_path(::mavros_msgs::srv::FileRename_Request & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::srv::FileRename_Request new_path(::mavros_msgs::srv::FileRename_Request::_new_path_type arg)
  {
    msg_.new_path = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::srv::FileRename_Request msg_;
};

class Init_FileRename_Request_old_path
{
public:
  Init_FileRename_Request_old_path()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FileRename_Request_new_path old_path(::mavros_msgs::srv::FileRename_Request::_old_path_type arg)
  {
    msg_.old_path = std::move(arg);
    return Init_FileRename_Request_new_path(msg_);
  }

private:
  ::mavros_msgs::srv::FileRename_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::srv::FileRename_Request>()
{
  return mavros_msgs::srv::builder::Init_FileRename_Request_old_path();
}

}  // namespace mavros_msgs


namespace mavros_msgs
{

namespace srv
{

namespace builder
{

class Init_FileRename_Response_r_errno
{
public:
  explicit Init_FileRename_Response_r_errno(::mavros_msgs::srv::FileRename_Response & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::srv::FileRename_Response r_errno(::mavros_msgs::srv::FileRename_Response::_r_errno_type arg)
  {
    msg_.r_errno = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::srv::FileRename_Response msg_;
};

class Init_FileRename_Response_success
{
public:
  Init_FileRename_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FileRename_Response_r_errno success(::mavros_msgs::srv::FileRename_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_FileRename_Response_r_errno(msg_);
  }

private:
  ::mavros_msgs::srv::FileRename_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::srv::FileRename_Response>()
{
  return mavros_msgs::srv::builder::Init_FileRename_Response_success();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__SRV__DETAIL__FILE_RENAME__BUILDER_HPP_
