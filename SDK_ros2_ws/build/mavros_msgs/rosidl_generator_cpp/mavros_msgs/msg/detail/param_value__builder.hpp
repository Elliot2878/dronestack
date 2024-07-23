// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:msg/ParamValue.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__PARAM_VALUE__BUILDER_HPP_
#define MAVROS_MSGS__MSG__DETAIL__PARAM_VALUE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/msg/detail/param_value__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace msg
{

namespace builder
{

class Init_ParamValue_real
{
public:
  explicit Init_ParamValue_real(::mavros_msgs::msg::ParamValue & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::msg::ParamValue real(::mavros_msgs::msg::ParamValue::_real_type arg)
  {
    msg_.real = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::msg::ParamValue msg_;
};

class Init_ParamValue_integer
{
public:
  Init_ParamValue_integer()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ParamValue_real integer(::mavros_msgs::msg::ParamValue::_integer_type arg)
  {
    msg_.integer = std::move(arg);
    return Init_ParamValue_real(msg_);
  }

private:
  ::mavros_msgs::msg::ParamValue msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::msg::ParamValue>()
{
  return mavros_msgs::msg::builder::Init_ParamValue_integer();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__PARAM_VALUE__BUILDER_HPP_
