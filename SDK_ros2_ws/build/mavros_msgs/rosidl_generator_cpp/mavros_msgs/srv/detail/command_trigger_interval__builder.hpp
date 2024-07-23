// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:srv/CommandTriggerInterval.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__COMMAND_TRIGGER_INTERVAL__BUILDER_HPP_
#define MAVROS_MSGS__SRV__DETAIL__COMMAND_TRIGGER_INTERVAL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/srv/detail/command_trigger_interval__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace srv
{

namespace builder
{

class Init_CommandTriggerInterval_Request_integration_time
{
public:
  explicit Init_CommandTriggerInterval_Request_integration_time(::mavros_msgs::srv::CommandTriggerInterval_Request & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::srv::CommandTriggerInterval_Request integration_time(::mavros_msgs::srv::CommandTriggerInterval_Request::_integration_time_type arg)
  {
    msg_.integration_time = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::srv::CommandTriggerInterval_Request msg_;
};

class Init_CommandTriggerInterval_Request_cycle_time
{
public:
  Init_CommandTriggerInterval_Request_cycle_time()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CommandTriggerInterval_Request_integration_time cycle_time(::mavros_msgs::srv::CommandTriggerInterval_Request::_cycle_time_type arg)
  {
    msg_.cycle_time = std::move(arg);
    return Init_CommandTriggerInterval_Request_integration_time(msg_);
  }

private:
  ::mavros_msgs::srv::CommandTriggerInterval_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::srv::CommandTriggerInterval_Request>()
{
  return mavros_msgs::srv::builder::Init_CommandTriggerInterval_Request_cycle_time();
}

}  // namespace mavros_msgs


namespace mavros_msgs
{

namespace srv
{

namespace builder
{

class Init_CommandTriggerInterval_Response_result
{
public:
  explicit Init_CommandTriggerInterval_Response_result(::mavros_msgs::srv::CommandTriggerInterval_Response & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::srv::CommandTriggerInterval_Response result(::mavros_msgs::srv::CommandTriggerInterval_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::srv::CommandTriggerInterval_Response msg_;
};

class Init_CommandTriggerInterval_Response_success
{
public:
  Init_CommandTriggerInterval_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CommandTriggerInterval_Response_result success(::mavros_msgs::srv::CommandTriggerInterval_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_CommandTriggerInterval_Response_result(msg_);
  }

private:
  ::mavros_msgs::srv::CommandTriggerInterval_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::srv::CommandTriggerInterval_Response>()
{
  return mavros_msgs::srv::builder::Init_CommandTriggerInterval_Response_success();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__SRV__DETAIL__COMMAND_TRIGGER_INTERVAL__BUILDER_HPP_
