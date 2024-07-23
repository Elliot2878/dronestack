// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:srv/ParamPull.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__PARAM_PULL__BUILDER_HPP_
#define MAVROS_MSGS__SRV__DETAIL__PARAM_PULL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/srv/detail/param_pull__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace srv
{

namespace builder
{

class Init_ParamPull_Request_force_pull
{
public:
  Init_ParamPull_Request_force_pull()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::mavros_msgs::srv::ParamPull_Request force_pull(::mavros_msgs::srv::ParamPull_Request::_force_pull_type arg)
  {
    msg_.force_pull = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::srv::ParamPull_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::srv::ParamPull_Request>()
{
  return mavros_msgs::srv::builder::Init_ParamPull_Request_force_pull();
}

}  // namespace mavros_msgs


namespace mavros_msgs
{

namespace srv
{

namespace builder
{

class Init_ParamPull_Response_param_received
{
public:
  explicit Init_ParamPull_Response_param_received(::mavros_msgs::srv::ParamPull_Response & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::srv::ParamPull_Response param_received(::mavros_msgs::srv::ParamPull_Response::_param_received_type arg)
  {
    msg_.param_received = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::srv::ParamPull_Response msg_;
};

class Init_ParamPull_Response_success
{
public:
  Init_ParamPull_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ParamPull_Response_param_received success(::mavros_msgs::srv::ParamPull_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_ParamPull_Response_param_received(msg_);
  }

private:
  ::mavros_msgs::srv::ParamPull_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::srv::ParamPull_Response>()
{
  return mavros_msgs::srv::builder::Init_ParamPull_Response_success();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__SRV__DETAIL__PARAM_PULL__BUILDER_HPP_
