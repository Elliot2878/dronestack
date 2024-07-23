// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:msg/ExtendedState.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__EXTENDED_STATE__BUILDER_HPP_
#define MAVROS_MSGS__MSG__DETAIL__EXTENDED_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/msg/detail/extended_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace msg
{

namespace builder
{

class Init_ExtendedState_landed_state
{
public:
  explicit Init_ExtendedState_landed_state(::mavros_msgs::msg::ExtendedState & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::msg::ExtendedState landed_state(::mavros_msgs::msg::ExtendedState::_landed_state_type arg)
  {
    msg_.landed_state = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::msg::ExtendedState msg_;
};

class Init_ExtendedState_vtol_state
{
public:
  explicit Init_ExtendedState_vtol_state(::mavros_msgs::msg::ExtendedState & msg)
  : msg_(msg)
  {}
  Init_ExtendedState_landed_state vtol_state(::mavros_msgs::msg::ExtendedState::_vtol_state_type arg)
  {
    msg_.vtol_state = std::move(arg);
    return Init_ExtendedState_landed_state(msg_);
  }

private:
  ::mavros_msgs::msg::ExtendedState msg_;
};

class Init_ExtendedState_header
{
public:
  Init_ExtendedState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ExtendedState_vtol_state header(::mavros_msgs::msg::ExtendedState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ExtendedState_vtol_state(msg_);
  }

private:
  ::mavros_msgs::msg::ExtendedState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::msg::ExtendedState>()
{
  return mavros_msgs::msg::builder::Init_ExtendedState_header();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__EXTENDED_STATE__BUILDER_HPP_
