// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:msg/State.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__STATE__BUILDER_HPP_
#define MAVROS_MSGS__MSG__DETAIL__STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/msg/detail/state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace msg
{

namespace builder
{

class Init_State_system_status
{
public:
  explicit Init_State_system_status(::mavros_msgs::msg::State & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::msg::State system_status(::mavros_msgs::msg::State::_system_status_type arg)
  {
    msg_.system_status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::msg::State msg_;
};

class Init_State_mode
{
public:
  explicit Init_State_mode(::mavros_msgs::msg::State & msg)
  : msg_(msg)
  {}
  Init_State_system_status mode(::mavros_msgs::msg::State::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return Init_State_system_status(msg_);
  }

private:
  ::mavros_msgs::msg::State msg_;
};

class Init_State_manual_input
{
public:
  explicit Init_State_manual_input(::mavros_msgs::msg::State & msg)
  : msg_(msg)
  {}
  Init_State_mode manual_input(::mavros_msgs::msg::State::_manual_input_type arg)
  {
    msg_.manual_input = std::move(arg);
    return Init_State_mode(msg_);
  }

private:
  ::mavros_msgs::msg::State msg_;
};

class Init_State_guided
{
public:
  explicit Init_State_guided(::mavros_msgs::msg::State & msg)
  : msg_(msg)
  {}
  Init_State_manual_input guided(::mavros_msgs::msg::State::_guided_type arg)
  {
    msg_.guided = std::move(arg);
    return Init_State_manual_input(msg_);
  }

private:
  ::mavros_msgs::msg::State msg_;
};

class Init_State_armed
{
public:
  explicit Init_State_armed(::mavros_msgs::msg::State & msg)
  : msg_(msg)
  {}
  Init_State_guided armed(::mavros_msgs::msg::State::_armed_type arg)
  {
    msg_.armed = std::move(arg);
    return Init_State_guided(msg_);
  }

private:
  ::mavros_msgs::msg::State msg_;
};

class Init_State_connected
{
public:
  explicit Init_State_connected(::mavros_msgs::msg::State & msg)
  : msg_(msg)
  {}
  Init_State_armed connected(::mavros_msgs::msg::State::_connected_type arg)
  {
    msg_.connected = std::move(arg);
    return Init_State_armed(msg_);
  }

private:
  ::mavros_msgs::msg::State msg_;
};

class Init_State_header
{
public:
  Init_State_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_State_connected header(::mavros_msgs::msg::State::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_State_connected(msg_);
  }

private:
  ::mavros_msgs::msg::State msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::msg::State>()
{
  return mavros_msgs::msg::builder::Init_State_header();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__STATE__BUILDER_HPP_
