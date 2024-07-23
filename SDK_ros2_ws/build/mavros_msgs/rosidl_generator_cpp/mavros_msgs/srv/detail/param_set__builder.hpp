// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:srv/ParamSet.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__PARAM_SET__BUILDER_HPP_
#define MAVROS_MSGS__SRV__DETAIL__PARAM_SET__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/srv/detail/param_set__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace srv
{

namespace builder
{

class Init_ParamSet_Request_value
{
public:
  explicit Init_ParamSet_Request_value(::mavros_msgs::srv::ParamSet_Request & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::srv::ParamSet_Request value(::mavros_msgs::srv::ParamSet_Request::_value_type arg)
  {
    msg_.value = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::srv::ParamSet_Request msg_;
};

class Init_ParamSet_Request_param_id
{
public:
  Init_ParamSet_Request_param_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ParamSet_Request_value param_id(::mavros_msgs::srv::ParamSet_Request::_param_id_type arg)
  {
    msg_.param_id = std::move(arg);
    return Init_ParamSet_Request_value(msg_);
  }

private:
  ::mavros_msgs::srv::ParamSet_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::srv::ParamSet_Request>()
{
  return mavros_msgs::srv::builder::Init_ParamSet_Request_param_id();
}

}  // namespace mavros_msgs


namespace mavros_msgs
{

namespace srv
{

namespace builder
{

class Init_ParamSet_Response_value
{
public:
  explicit Init_ParamSet_Response_value(::mavros_msgs::srv::ParamSet_Response & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::srv::ParamSet_Response value(::mavros_msgs::srv::ParamSet_Response::_value_type arg)
  {
    msg_.value = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::srv::ParamSet_Response msg_;
};

class Init_ParamSet_Response_success
{
public:
  Init_ParamSet_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ParamSet_Response_value success(::mavros_msgs::srv::ParamSet_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_ParamSet_Response_value(msg_);
  }

private:
  ::mavros_msgs::srv::ParamSet_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::srv::ParamSet_Response>()
{
  return mavros_msgs::srv::builder::Init_ParamSet_Response_success();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__SRV__DETAIL__PARAM_SET__BUILDER_HPP_
