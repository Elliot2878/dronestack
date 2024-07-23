// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:msg/OverrideRCIn.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__OVERRIDE_RC_IN__BUILDER_HPP_
#define MAVROS_MSGS__MSG__DETAIL__OVERRIDE_RC_IN__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/msg/detail/override_rc_in__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace msg
{

namespace builder
{

class Init_OverrideRCIn_channels
{
public:
  Init_OverrideRCIn_channels()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::mavros_msgs::msg::OverrideRCIn channels(::mavros_msgs::msg::OverrideRCIn::_channels_type arg)
  {
    msg_.channels = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::msg::OverrideRCIn msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::msg::OverrideRCIn>()
{
  return mavros_msgs::msg::builder::Init_OverrideRCIn_channels();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__OVERRIDE_RC_IN__BUILDER_HPP_
