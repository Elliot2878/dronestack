// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:msg/ESCStatus.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__ESC_STATUS__BUILDER_HPP_
#define MAVROS_MSGS__MSG__DETAIL__ESC_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/msg/detail/esc_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace msg
{

namespace builder
{

class Init_ESCStatus_esc_status
{
public:
  explicit Init_ESCStatus_esc_status(::mavros_msgs::msg::ESCStatus & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::msg::ESCStatus esc_status(::mavros_msgs::msg::ESCStatus::_esc_status_type arg)
  {
    msg_.esc_status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::msg::ESCStatus msg_;
};

class Init_ESCStatus_header
{
public:
  Init_ESCStatus_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ESCStatus_esc_status header(::mavros_msgs::msg::ESCStatus::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ESCStatus_esc_status(msg_);
  }

private:
  ::mavros_msgs::msg::ESCStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::msg::ESCStatus>()
{
  return mavros_msgs::msg::builder::Init_ESCStatus_header();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__ESC_STATUS__BUILDER_HPP_
