// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:srv/CommandTriggerControl.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__COMMAND_TRIGGER_CONTROL__BUILDER_HPP_
#define MAVROS_MSGS__SRV__DETAIL__COMMAND_TRIGGER_CONTROL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/srv/detail/command_trigger_control__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace srv
{

namespace builder
{

class Init_CommandTriggerControl_Request_trigger_pause
{
public:
  explicit Init_CommandTriggerControl_Request_trigger_pause(::mavros_msgs::srv::CommandTriggerControl_Request & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::srv::CommandTriggerControl_Request trigger_pause(::mavros_msgs::srv::CommandTriggerControl_Request::_trigger_pause_type arg)
  {
    msg_.trigger_pause = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::srv::CommandTriggerControl_Request msg_;
};

class Init_CommandTriggerControl_Request_sequence_reset
{
public:
  explicit Init_CommandTriggerControl_Request_sequence_reset(::mavros_msgs::srv::CommandTriggerControl_Request & msg)
  : msg_(msg)
  {}
  Init_CommandTriggerControl_Request_trigger_pause sequence_reset(::mavros_msgs::srv::CommandTriggerControl_Request::_sequence_reset_type arg)
  {
    msg_.sequence_reset = std::move(arg);
    return Init_CommandTriggerControl_Request_trigger_pause(msg_);
  }

private:
  ::mavros_msgs::srv::CommandTriggerControl_Request msg_;
};

class Init_CommandTriggerControl_Request_trigger_enable
{
public:
  Init_CommandTriggerControl_Request_trigger_enable()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CommandTriggerControl_Request_sequence_reset trigger_enable(::mavros_msgs::srv::CommandTriggerControl_Request::_trigger_enable_type arg)
  {
    msg_.trigger_enable = std::move(arg);
    return Init_CommandTriggerControl_Request_sequence_reset(msg_);
  }

private:
  ::mavros_msgs::srv::CommandTriggerControl_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::srv::CommandTriggerControl_Request>()
{
  return mavros_msgs::srv::builder::Init_CommandTriggerControl_Request_trigger_enable();
}

}  // namespace mavros_msgs


namespace mavros_msgs
{

namespace srv
{

namespace builder
{

class Init_CommandTriggerControl_Response_result
{
public:
  explicit Init_CommandTriggerControl_Response_result(::mavros_msgs::srv::CommandTriggerControl_Response & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::srv::CommandTriggerControl_Response result(::mavros_msgs::srv::CommandTriggerControl_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::srv::CommandTriggerControl_Response msg_;
};

class Init_CommandTriggerControl_Response_success
{
public:
  Init_CommandTriggerControl_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CommandTriggerControl_Response_result success(::mavros_msgs::srv::CommandTriggerControl_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_CommandTriggerControl_Response_result(msg_);
  }

private:
  ::mavros_msgs::srv::CommandTriggerControl_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::srv::CommandTriggerControl_Response>()
{
  return mavros_msgs::srv::builder::Init_CommandTriggerControl_Response_success();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__SRV__DETAIL__COMMAND_TRIGGER_CONTROL__BUILDER_HPP_
