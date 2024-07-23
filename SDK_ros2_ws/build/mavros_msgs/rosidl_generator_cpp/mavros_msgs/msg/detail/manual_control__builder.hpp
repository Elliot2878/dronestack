// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:msg/ManualControl.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__MANUAL_CONTROL__BUILDER_HPP_
#define MAVROS_MSGS__MSG__DETAIL__MANUAL_CONTROL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/msg/detail/manual_control__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace msg
{

namespace builder
{

class Init_ManualControl_buttons
{
public:
  explicit Init_ManualControl_buttons(::mavros_msgs::msg::ManualControl & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::msg::ManualControl buttons(::mavros_msgs::msg::ManualControl::_buttons_type arg)
  {
    msg_.buttons = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::msg::ManualControl msg_;
};

class Init_ManualControl_r
{
public:
  explicit Init_ManualControl_r(::mavros_msgs::msg::ManualControl & msg)
  : msg_(msg)
  {}
  Init_ManualControl_buttons r(::mavros_msgs::msg::ManualControl::_r_type arg)
  {
    msg_.r = std::move(arg);
    return Init_ManualControl_buttons(msg_);
  }

private:
  ::mavros_msgs::msg::ManualControl msg_;
};

class Init_ManualControl_z
{
public:
  explicit Init_ManualControl_z(::mavros_msgs::msg::ManualControl & msg)
  : msg_(msg)
  {}
  Init_ManualControl_r z(::mavros_msgs::msg::ManualControl::_z_type arg)
  {
    msg_.z = std::move(arg);
    return Init_ManualControl_r(msg_);
  }

private:
  ::mavros_msgs::msg::ManualControl msg_;
};

class Init_ManualControl_y
{
public:
  explicit Init_ManualControl_y(::mavros_msgs::msg::ManualControl & msg)
  : msg_(msg)
  {}
  Init_ManualControl_z y(::mavros_msgs::msg::ManualControl::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_ManualControl_z(msg_);
  }

private:
  ::mavros_msgs::msg::ManualControl msg_;
};

class Init_ManualControl_x
{
public:
  explicit Init_ManualControl_x(::mavros_msgs::msg::ManualControl & msg)
  : msg_(msg)
  {}
  Init_ManualControl_y x(::mavros_msgs::msg::ManualControl::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_ManualControl_y(msg_);
  }

private:
  ::mavros_msgs::msg::ManualControl msg_;
};

class Init_ManualControl_header
{
public:
  Init_ManualControl_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ManualControl_x header(::mavros_msgs::msg::ManualControl::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ManualControl_x(msg_);
  }

private:
  ::mavros_msgs::msg::ManualControl msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::msg::ManualControl>()
{
  return mavros_msgs::msg::builder::Init_ManualControl_header();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__MANUAL_CONTROL__BUILDER_HPP_
