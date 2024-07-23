// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:msg/ParamEvent.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__PARAM_EVENT__BUILDER_HPP_
#define MAVROS_MSGS__MSG__DETAIL__PARAM_EVENT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/msg/detail/param_event__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace msg
{

namespace builder
{

class Init_ParamEvent_param_count
{
public:
  explicit Init_ParamEvent_param_count(::mavros_msgs::msg::ParamEvent & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::msg::ParamEvent param_count(::mavros_msgs::msg::ParamEvent::_param_count_type arg)
  {
    msg_.param_count = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::msg::ParamEvent msg_;
};

class Init_ParamEvent_param_index
{
public:
  explicit Init_ParamEvent_param_index(::mavros_msgs::msg::ParamEvent & msg)
  : msg_(msg)
  {}
  Init_ParamEvent_param_count param_index(::mavros_msgs::msg::ParamEvent::_param_index_type arg)
  {
    msg_.param_index = std::move(arg);
    return Init_ParamEvent_param_count(msg_);
  }

private:
  ::mavros_msgs::msg::ParamEvent msg_;
};

class Init_ParamEvent_value
{
public:
  explicit Init_ParamEvent_value(::mavros_msgs::msg::ParamEvent & msg)
  : msg_(msg)
  {}
  Init_ParamEvent_param_index value(::mavros_msgs::msg::ParamEvent::_value_type arg)
  {
    msg_.value = std::move(arg);
    return Init_ParamEvent_param_index(msg_);
  }

private:
  ::mavros_msgs::msg::ParamEvent msg_;
};

class Init_ParamEvent_param_id
{
public:
  explicit Init_ParamEvent_param_id(::mavros_msgs::msg::ParamEvent & msg)
  : msg_(msg)
  {}
  Init_ParamEvent_value param_id(::mavros_msgs::msg::ParamEvent::_param_id_type arg)
  {
    msg_.param_id = std::move(arg);
    return Init_ParamEvent_value(msg_);
  }

private:
  ::mavros_msgs::msg::ParamEvent msg_;
};

class Init_ParamEvent_header
{
public:
  Init_ParamEvent_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ParamEvent_param_id header(::mavros_msgs::msg::ParamEvent::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ParamEvent_param_id(msg_);
  }

private:
  ::mavros_msgs::msg::ParamEvent msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::msg::ParamEvent>()
{
  return mavros_msgs::msg::builder::Init_ParamEvent_header();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__PARAM_EVENT__BUILDER_HPP_
