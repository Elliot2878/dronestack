// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:srv/CommandAck.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__COMMAND_ACK__BUILDER_HPP_
#define MAVROS_MSGS__SRV__DETAIL__COMMAND_ACK__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/srv/detail/command_ack__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace srv
{

namespace builder
{

class Init_CommandAck_Request_result_param2
{
public:
  explicit Init_CommandAck_Request_result_param2(::mavros_msgs::srv::CommandAck_Request & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::srv::CommandAck_Request result_param2(::mavros_msgs::srv::CommandAck_Request::_result_param2_type arg)
  {
    msg_.result_param2 = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::srv::CommandAck_Request msg_;
};

class Init_CommandAck_Request_progress
{
public:
  explicit Init_CommandAck_Request_progress(::mavros_msgs::srv::CommandAck_Request & msg)
  : msg_(msg)
  {}
  Init_CommandAck_Request_result_param2 progress(::mavros_msgs::srv::CommandAck_Request::_progress_type arg)
  {
    msg_.progress = std::move(arg);
    return Init_CommandAck_Request_result_param2(msg_);
  }

private:
  ::mavros_msgs::srv::CommandAck_Request msg_;
};

class Init_CommandAck_Request_result
{
public:
  explicit Init_CommandAck_Request_result(::mavros_msgs::srv::CommandAck_Request & msg)
  : msg_(msg)
  {}
  Init_CommandAck_Request_progress result(::mavros_msgs::srv::CommandAck_Request::_result_type arg)
  {
    msg_.result = std::move(arg);
    return Init_CommandAck_Request_progress(msg_);
  }

private:
  ::mavros_msgs::srv::CommandAck_Request msg_;
};

class Init_CommandAck_Request_command
{
public:
  Init_CommandAck_Request_command()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CommandAck_Request_result command(::mavros_msgs::srv::CommandAck_Request::_command_type arg)
  {
    msg_.command = std::move(arg);
    return Init_CommandAck_Request_result(msg_);
  }

private:
  ::mavros_msgs::srv::CommandAck_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::srv::CommandAck_Request>()
{
  return mavros_msgs::srv::builder::Init_CommandAck_Request_command();
}

}  // namespace mavros_msgs


namespace mavros_msgs
{

namespace srv
{

namespace builder
{

class Init_CommandAck_Response_result
{
public:
  explicit Init_CommandAck_Response_result(::mavros_msgs::srv::CommandAck_Response & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::srv::CommandAck_Response result(::mavros_msgs::srv::CommandAck_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::srv::CommandAck_Response msg_;
};

class Init_CommandAck_Response_success
{
public:
  Init_CommandAck_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CommandAck_Response_result success(::mavros_msgs::srv::CommandAck_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_CommandAck_Response_result(msg_);
  }

private:
  ::mavros_msgs::srv::CommandAck_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::srv::CommandAck_Response>()
{
  return mavros_msgs::srv::builder::Init_CommandAck_Response_success();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__SRV__DETAIL__COMMAND_ACK__BUILDER_HPP_
