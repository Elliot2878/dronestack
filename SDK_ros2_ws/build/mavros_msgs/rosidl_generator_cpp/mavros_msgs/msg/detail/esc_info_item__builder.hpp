// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:msg/ESCInfoItem.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__ESC_INFO_ITEM__BUILDER_HPP_
#define MAVROS_MSGS__MSG__DETAIL__ESC_INFO_ITEM__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/msg/detail/esc_info_item__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace msg
{

namespace builder
{

class Init_ESCInfoItem_temperature
{
public:
  explicit Init_ESCInfoItem_temperature(::mavros_msgs::msg::ESCInfoItem & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::msg::ESCInfoItem temperature(::mavros_msgs::msg::ESCInfoItem::_temperature_type arg)
  {
    msg_.temperature = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::msg::ESCInfoItem msg_;
};

class Init_ESCInfoItem_error_count
{
public:
  explicit Init_ESCInfoItem_error_count(::mavros_msgs::msg::ESCInfoItem & msg)
  : msg_(msg)
  {}
  Init_ESCInfoItem_temperature error_count(::mavros_msgs::msg::ESCInfoItem::_error_count_type arg)
  {
    msg_.error_count = std::move(arg);
    return Init_ESCInfoItem_temperature(msg_);
  }

private:
  ::mavros_msgs::msg::ESCInfoItem msg_;
};

class Init_ESCInfoItem_failure_flags
{
public:
  explicit Init_ESCInfoItem_failure_flags(::mavros_msgs::msg::ESCInfoItem & msg)
  : msg_(msg)
  {}
  Init_ESCInfoItem_error_count failure_flags(::mavros_msgs::msg::ESCInfoItem::_failure_flags_type arg)
  {
    msg_.failure_flags = std::move(arg);
    return Init_ESCInfoItem_error_count(msg_);
  }

private:
  ::mavros_msgs::msg::ESCInfoItem msg_;
};

class Init_ESCInfoItem_header
{
public:
  Init_ESCInfoItem_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ESCInfoItem_failure_flags header(::mavros_msgs::msg::ESCInfoItem::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ESCInfoItem_failure_flags(msg_);
  }

private:
  ::mavros_msgs::msg::ESCInfoItem msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::msg::ESCInfoItem>()
{
  return mavros_msgs::msg::builder::Init_ESCInfoItem_header();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__ESC_INFO_ITEM__BUILDER_HPP_
