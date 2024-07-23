// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:msg/ESCStatusItem.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__ESC_STATUS_ITEM__BUILDER_HPP_
#define MAVROS_MSGS__MSG__DETAIL__ESC_STATUS_ITEM__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/msg/detail/esc_status_item__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace msg
{

namespace builder
{

class Init_ESCStatusItem_current
{
public:
  explicit Init_ESCStatusItem_current(::mavros_msgs::msg::ESCStatusItem & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::msg::ESCStatusItem current(::mavros_msgs::msg::ESCStatusItem::_current_type arg)
  {
    msg_.current = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::msg::ESCStatusItem msg_;
};

class Init_ESCStatusItem_voltage
{
public:
  explicit Init_ESCStatusItem_voltage(::mavros_msgs::msg::ESCStatusItem & msg)
  : msg_(msg)
  {}
  Init_ESCStatusItem_current voltage(::mavros_msgs::msg::ESCStatusItem::_voltage_type arg)
  {
    msg_.voltage = std::move(arg);
    return Init_ESCStatusItem_current(msg_);
  }

private:
  ::mavros_msgs::msg::ESCStatusItem msg_;
};

class Init_ESCStatusItem_rpm
{
public:
  explicit Init_ESCStatusItem_rpm(::mavros_msgs::msg::ESCStatusItem & msg)
  : msg_(msg)
  {}
  Init_ESCStatusItem_voltage rpm(::mavros_msgs::msg::ESCStatusItem::_rpm_type arg)
  {
    msg_.rpm = std::move(arg);
    return Init_ESCStatusItem_voltage(msg_);
  }

private:
  ::mavros_msgs::msg::ESCStatusItem msg_;
};

class Init_ESCStatusItem_header
{
public:
  Init_ESCStatusItem_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ESCStatusItem_rpm header(::mavros_msgs::msg::ESCStatusItem::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ESCStatusItem_rpm(msg_);
  }

private:
  ::mavros_msgs::msg::ESCStatusItem msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::msg::ESCStatusItem>()
{
  return mavros_msgs::msg::builder::Init_ESCStatusItem_header();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__ESC_STATUS_ITEM__BUILDER_HPP_
