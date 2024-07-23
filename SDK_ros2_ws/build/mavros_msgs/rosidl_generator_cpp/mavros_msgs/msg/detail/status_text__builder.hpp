// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:msg/StatusText.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__STATUS_TEXT__BUILDER_HPP_
#define MAVROS_MSGS__MSG__DETAIL__STATUS_TEXT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/msg/detail/status_text__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace msg
{

namespace builder
{

class Init_StatusText_text
{
public:
  explicit Init_StatusText_text(::mavros_msgs::msg::StatusText & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::msg::StatusText text(::mavros_msgs::msg::StatusText::_text_type arg)
  {
    msg_.text = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::msg::StatusText msg_;
};

class Init_StatusText_severity
{
public:
  explicit Init_StatusText_severity(::mavros_msgs::msg::StatusText & msg)
  : msg_(msg)
  {}
  Init_StatusText_text severity(::mavros_msgs::msg::StatusText::_severity_type arg)
  {
    msg_.severity = std::move(arg);
    return Init_StatusText_text(msg_);
  }

private:
  ::mavros_msgs::msg::StatusText msg_;
};

class Init_StatusText_header
{
public:
  Init_StatusText_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_StatusText_severity header(::mavros_msgs::msg::StatusText::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_StatusText_severity(msg_);
  }

private:
  ::mavros_msgs::msg::StatusText msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::msg::StatusText>()
{
  return mavros_msgs::msg::builder::Init_StatusText_header();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__STATUS_TEXT__BUILDER_HPP_
