// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:srv/MessageInterval.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__MESSAGE_INTERVAL__BUILDER_HPP_
#define MAVROS_MSGS__SRV__DETAIL__MESSAGE_INTERVAL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/srv/detail/message_interval__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace srv
{

namespace builder
{

class Init_MessageInterval_Request_message_rate
{
public:
  explicit Init_MessageInterval_Request_message_rate(::mavros_msgs::srv::MessageInterval_Request & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::srv::MessageInterval_Request message_rate(::mavros_msgs::srv::MessageInterval_Request::_message_rate_type arg)
  {
    msg_.message_rate = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::srv::MessageInterval_Request msg_;
};

class Init_MessageInterval_Request_message_id
{
public:
  Init_MessageInterval_Request_message_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MessageInterval_Request_message_rate message_id(::mavros_msgs::srv::MessageInterval_Request::_message_id_type arg)
  {
    msg_.message_id = std::move(arg);
    return Init_MessageInterval_Request_message_rate(msg_);
  }

private:
  ::mavros_msgs::srv::MessageInterval_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::srv::MessageInterval_Request>()
{
  return mavros_msgs::srv::builder::Init_MessageInterval_Request_message_id();
}

}  // namespace mavros_msgs


namespace mavros_msgs
{

namespace srv
{

namespace builder
{

class Init_MessageInterval_Response_success
{
public:
  Init_MessageInterval_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::mavros_msgs::srv::MessageInterval_Response success(::mavros_msgs::srv::MessageInterval_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::srv::MessageInterval_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::srv::MessageInterval_Response>()
{
  return mavros_msgs::srv::builder::Init_MessageInterval_Response_success();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__SRV__DETAIL__MESSAGE_INTERVAL__BUILDER_HPP_
