// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:srv/LogRequestData.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__LOG_REQUEST_DATA__BUILDER_HPP_
#define MAVROS_MSGS__SRV__DETAIL__LOG_REQUEST_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/srv/detail/log_request_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace srv
{

namespace builder
{

class Init_LogRequestData_Request_count
{
public:
  explicit Init_LogRequestData_Request_count(::mavros_msgs::srv::LogRequestData_Request & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::srv::LogRequestData_Request count(::mavros_msgs::srv::LogRequestData_Request::_count_type arg)
  {
    msg_.count = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::srv::LogRequestData_Request msg_;
};

class Init_LogRequestData_Request_offset
{
public:
  explicit Init_LogRequestData_Request_offset(::mavros_msgs::srv::LogRequestData_Request & msg)
  : msg_(msg)
  {}
  Init_LogRequestData_Request_count offset(::mavros_msgs::srv::LogRequestData_Request::_offset_type arg)
  {
    msg_.offset = std::move(arg);
    return Init_LogRequestData_Request_count(msg_);
  }

private:
  ::mavros_msgs::srv::LogRequestData_Request msg_;
};

class Init_LogRequestData_Request_id
{
public:
  Init_LogRequestData_Request_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LogRequestData_Request_offset id(::mavros_msgs::srv::LogRequestData_Request::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_LogRequestData_Request_offset(msg_);
  }

private:
  ::mavros_msgs::srv::LogRequestData_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::srv::LogRequestData_Request>()
{
  return mavros_msgs::srv::builder::Init_LogRequestData_Request_id();
}

}  // namespace mavros_msgs


namespace mavros_msgs
{

namespace srv
{

namespace builder
{

class Init_LogRequestData_Response_success
{
public:
  Init_LogRequestData_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::mavros_msgs::srv::LogRequestData_Response success(::mavros_msgs::srv::LogRequestData_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::srv::LogRequestData_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::srv::LogRequestData_Response>()
{
  return mavros_msgs::srv::builder::Init_LogRequestData_Response_success();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__SRV__DETAIL__LOG_REQUEST_DATA__BUILDER_HPP_
