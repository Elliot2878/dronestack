// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:srv/CommandLong.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__COMMAND_LONG__BUILDER_HPP_
#define MAVROS_MSGS__SRV__DETAIL__COMMAND_LONG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/srv/detail/command_long__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace srv
{

namespace builder
{

class Init_CommandLong_Request_param7
{
public:
  explicit Init_CommandLong_Request_param7(::mavros_msgs::srv::CommandLong_Request & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::srv::CommandLong_Request param7(::mavros_msgs::srv::CommandLong_Request::_param7_type arg)
  {
    msg_.param7 = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::srv::CommandLong_Request msg_;
};

class Init_CommandLong_Request_param6
{
public:
  explicit Init_CommandLong_Request_param6(::mavros_msgs::srv::CommandLong_Request & msg)
  : msg_(msg)
  {}
  Init_CommandLong_Request_param7 param6(::mavros_msgs::srv::CommandLong_Request::_param6_type arg)
  {
    msg_.param6 = std::move(arg);
    return Init_CommandLong_Request_param7(msg_);
  }

private:
  ::mavros_msgs::srv::CommandLong_Request msg_;
};

class Init_CommandLong_Request_param5
{
public:
  explicit Init_CommandLong_Request_param5(::mavros_msgs::srv::CommandLong_Request & msg)
  : msg_(msg)
  {}
  Init_CommandLong_Request_param6 param5(::mavros_msgs::srv::CommandLong_Request::_param5_type arg)
  {
    msg_.param5 = std::move(arg);
    return Init_CommandLong_Request_param6(msg_);
  }

private:
  ::mavros_msgs::srv::CommandLong_Request msg_;
};

class Init_CommandLong_Request_param4
{
public:
  explicit Init_CommandLong_Request_param4(::mavros_msgs::srv::CommandLong_Request & msg)
  : msg_(msg)
  {}
  Init_CommandLong_Request_param5 param4(::mavros_msgs::srv::CommandLong_Request::_param4_type arg)
  {
    msg_.param4 = std::move(arg);
    return Init_CommandLong_Request_param5(msg_);
  }

private:
  ::mavros_msgs::srv::CommandLong_Request msg_;
};

class Init_CommandLong_Request_param3
{
public:
  explicit Init_CommandLong_Request_param3(::mavros_msgs::srv::CommandLong_Request & msg)
  : msg_(msg)
  {}
  Init_CommandLong_Request_param4 param3(::mavros_msgs::srv::CommandLong_Request::_param3_type arg)
  {
    msg_.param3 = std::move(arg);
    return Init_CommandLong_Request_param4(msg_);
  }

private:
  ::mavros_msgs::srv::CommandLong_Request msg_;
};

class Init_CommandLong_Request_param2
{
public:
  explicit Init_CommandLong_Request_param2(::mavros_msgs::srv::CommandLong_Request & msg)
  : msg_(msg)
  {}
  Init_CommandLong_Request_param3 param2(::mavros_msgs::srv::CommandLong_Request::_param2_type arg)
  {
    msg_.param2 = std::move(arg);
    return Init_CommandLong_Request_param3(msg_);
  }

private:
  ::mavros_msgs::srv::CommandLong_Request msg_;
};

class Init_CommandLong_Request_param1
{
public:
  explicit Init_CommandLong_Request_param1(::mavros_msgs::srv::CommandLong_Request & msg)
  : msg_(msg)
  {}
  Init_CommandLong_Request_param2 param1(::mavros_msgs::srv::CommandLong_Request::_param1_type arg)
  {
    msg_.param1 = std::move(arg);
    return Init_CommandLong_Request_param2(msg_);
  }

private:
  ::mavros_msgs::srv::CommandLong_Request msg_;
};

class Init_CommandLong_Request_confirmation
{
public:
  explicit Init_CommandLong_Request_confirmation(::mavros_msgs::srv::CommandLong_Request & msg)
  : msg_(msg)
  {}
  Init_CommandLong_Request_param1 confirmation(::mavros_msgs::srv::CommandLong_Request::_confirmation_type arg)
  {
    msg_.confirmation = std::move(arg);
    return Init_CommandLong_Request_param1(msg_);
  }

private:
  ::mavros_msgs::srv::CommandLong_Request msg_;
};

class Init_CommandLong_Request_command
{
public:
  explicit Init_CommandLong_Request_command(::mavros_msgs::srv::CommandLong_Request & msg)
  : msg_(msg)
  {}
  Init_CommandLong_Request_confirmation command(::mavros_msgs::srv::CommandLong_Request::_command_type arg)
  {
    msg_.command = std::move(arg);
    return Init_CommandLong_Request_confirmation(msg_);
  }

private:
  ::mavros_msgs::srv::CommandLong_Request msg_;
};

class Init_CommandLong_Request_broadcast
{
public:
  Init_CommandLong_Request_broadcast()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CommandLong_Request_command broadcast(::mavros_msgs::srv::CommandLong_Request::_broadcast_type arg)
  {
    msg_.broadcast = std::move(arg);
    return Init_CommandLong_Request_command(msg_);
  }

private:
  ::mavros_msgs::srv::CommandLong_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::srv::CommandLong_Request>()
{
  return mavros_msgs::srv::builder::Init_CommandLong_Request_broadcast();
}

}  // namespace mavros_msgs


namespace mavros_msgs
{

namespace srv
{

namespace builder
{

class Init_CommandLong_Response_result
{
public:
  explicit Init_CommandLong_Response_result(::mavros_msgs::srv::CommandLong_Response & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::srv::CommandLong_Response result(::mavros_msgs::srv::CommandLong_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::srv::CommandLong_Response msg_;
};

class Init_CommandLong_Response_success
{
public:
  Init_CommandLong_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CommandLong_Response_result success(::mavros_msgs::srv::CommandLong_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_CommandLong_Response_result(msg_);
  }

private:
  ::mavros_msgs::srv::CommandLong_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::srv::CommandLong_Response>()
{
  return mavros_msgs::srv::builder::Init_CommandLong_Response_success();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__SRV__DETAIL__COMMAND_LONG__BUILDER_HPP_
