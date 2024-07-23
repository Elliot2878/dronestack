// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:msg/RCIn.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__RC_IN__BUILDER_HPP_
#define MAVROS_MSGS__MSG__DETAIL__RC_IN__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/msg/detail/rc_in__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace msg
{

namespace builder
{

class Init_RCIn_channels
{
public:
  explicit Init_RCIn_channels(::mavros_msgs::msg::RCIn & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::msg::RCIn channels(::mavros_msgs::msg::RCIn::_channels_type arg)
  {
    msg_.channels = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::msg::RCIn msg_;
};

class Init_RCIn_rssi
{
public:
  explicit Init_RCIn_rssi(::mavros_msgs::msg::RCIn & msg)
  : msg_(msg)
  {}
  Init_RCIn_channels rssi(::mavros_msgs::msg::RCIn::_rssi_type arg)
  {
    msg_.rssi = std::move(arg);
    return Init_RCIn_channels(msg_);
  }

private:
  ::mavros_msgs::msg::RCIn msg_;
};

class Init_RCIn_header
{
public:
  Init_RCIn_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RCIn_rssi header(::mavros_msgs::msg::RCIn::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_RCIn_rssi(msg_);
  }

private:
  ::mavros_msgs::msg::RCIn msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::msg::RCIn>()
{
  return mavros_msgs::msg::builder::Init_RCIn_header();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__RC_IN__BUILDER_HPP_
