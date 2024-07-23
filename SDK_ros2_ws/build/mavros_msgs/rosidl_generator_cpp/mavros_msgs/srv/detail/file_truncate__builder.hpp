// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:srv/FileTruncate.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__FILE_TRUNCATE__BUILDER_HPP_
#define MAVROS_MSGS__SRV__DETAIL__FILE_TRUNCATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/srv/detail/file_truncate__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace srv
{

namespace builder
{

class Init_FileTruncate_Request_length
{
public:
  explicit Init_FileTruncate_Request_length(::mavros_msgs::srv::FileTruncate_Request & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::srv::FileTruncate_Request length(::mavros_msgs::srv::FileTruncate_Request::_length_type arg)
  {
    msg_.length = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::srv::FileTruncate_Request msg_;
};

class Init_FileTruncate_Request_file_path
{
public:
  Init_FileTruncate_Request_file_path()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FileTruncate_Request_length file_path(::mavros_msgs::srv::FileTruncate_Request::_file_path_type arg)
  {
    msg_.file_path = std::move(arg);
    return Init_FileTruncate_Request_length(msg_);
  }

private:
  ::mavros_msgs::srv::FileTruncate_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::srv::FileTruncate_Request>()
{
  return mavros_msgs::srv::builder::Init_FileTruncate_Request_file_path();
}

}  // namespace mavros_msgs


namespace mavros_msgs
{

namespace srv
{

namespace builder
{

class Init_FileTruncate_Response_r_errno
{
public:
  explicit Init_FileTruncate_Response_r_errno(::mavros_msgs::srv::FileTruncate_Response & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::srv::FileTruncate_Response r_errno(::mavros_msgs::srv::FileTruncate_Response::_r_errno_type arg)
  {
    msg_.r_errno = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::srv::FileTruncate_Response msg_;
};

class Init_FileTruncate_Response_success
{
public:
  Init_FileTruncate_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FileTruncate_Response_r_errno success(::mavros_msgs::srv::FileTruncate_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_FileTruncate_Response_r_errno(msg_);
  }

private:
  ::mavros_msgs::srv::FileTruncate_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::srv::FileTruncate_Response>()
{
  return mavros_msgs::srv::builder::Init_FileTruncate_Response_success();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__SRV__DETAIL__FILE_TRUNCATE__BUILDER_HPP_
