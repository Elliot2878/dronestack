// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:msg/ESCInfo.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__ESC_INFO__BUILDER_HPP_
#define MAVROS_MSGS__MSG__DETAIL__ESC_INFO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/msg/detail/esc_info__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace msg
{

namespace builder
{

class Init_ESCInfo_esc_info
{
public:
  explicit Init_ESCInfo_esc_info(::mavros_msgs::msg::ESCInfo & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::msg::ESCInfo esc_info(::mavros_msgs::msg::ESCInfo::_esc_info_type arg)
  {
    msg_.esc_info = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::msg::ESCInfo msg_;
};

class Init_ESCInfo_info
{
public:
  explicit Init_ESCInfo_info(::mavros_msgs::msg::ESCInfo & msg)
  : msg_(msg)
  {}
  Init_ESCInfo_esc_info info(::mavros_msgs::msg::ESCInfo::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_ESCInfo_esc_info(msg_);
  }

private:
  ::mavros_msgs::msg::ESCInfo msg_;
};

class Init_ESCInfo_connection_type
{
public:
  explicit Init_ESCInfo_connection_type(::mavros_msgs::msg::ESCInfo & msg)
  : msg_(msg)
  {}
  Init_ESCInfo_info connection_type(::mavros_msgs::msg::ESCInfo::_connection_type_type arg)
  {
    msg_.connection_type = std::move(arg);
    return Init_ESCInfo_info(msg_);
  }

private:
  ::mavros_msgs::msg::ESCInfo msg_;
};

class Init_ESCInfo_count
{
public:
  explicit Init_ESCInfo_count(::mavros_msgs::msg::ESCInfo & msg)
  : msg_(msg)
  {}
  Init_ESCInfo_connection_type count(::mavros_msgs::msg::ESCInfo::_count_type arg)
  {
    msg_.count = std::move(arg);
    return Init_ESCInfo_connection_type(msg_);
  }

private:
  ::mavros_msgs::msg::ESCInfo msg_;
};

class Init_ESCInfo_counter
{
public:
  explicit Init_ESCInfo_counter(::mavros_msgs::msg::ESCInfo & msg)
  : msg_(msg)
  {}
  Init_ESCInfo_count counter(::mavros_msgs::msg::ESCInfo::_counter_type arg)
  {
    msg_.counter = std::move(arg);
    return Init_ESCInfo_count(msg_);
  }

private:
  ::mavros_msgs::msg::ESCInfo msg_;
};

class Init_ESCInfo_header
{
public:
  Init_ESCInfo_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ESCInfo_counter header(::mavros_msgs::msg::ESCInfo::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ESCInfo_counter(msg_);
  }

private:
  ::mavros_msgs::msg::ESCInfo msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::msg::ESCInfo>()
{
  return mavros_msgs::msg::builder::Init_ESCInfo_header();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__ESC_INFO__BUILDER_HPP_
