// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:srv/ParamPush.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__PARAM_PUSH__BUILDER_HPP_
#define MAVROS_MSGS__SRV__DETAIL__PARAM_PUSH__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/srv/detail/param_push__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::srv::ParamPush_Request>()
{
  return ::mavros_msgs::srv::ParamPush_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace mavros_msgs


namespace mavros_msgs
{

namespace srv
{

namespace builder
{

class Init_ParamPush_Response_param_transfered
{
public:
  explicit Init_ParamPush_Response_param_transfered(::mavros_msgs::srv::ParamPush_Response & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::srv::ParamPush_Response param_transfered(::mavros_msgs::srv::ParamPush_Response::_param_transfered_type arg)
  {
    msg_.param_transfered = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::srv::ParamPush_Response msg_;
};

class Init_ParamPush_Response_success
{
public:
  Init_ParamPush_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ParamPush_Response_param_transfered success(::mavros_msgs::srv::ParamPush_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_ParamPush_Response_param_transfered(msg_);
  }

private:
  ::mavros_msgs::srv::ParamPush_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::srv::ParamPush_Response>()
{
  return mavros_msgs::srv::builder::Init_ParamPush_Response_success();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__SRV__DETAIL__PARAM_PUSH__BUILDER_HPP_
