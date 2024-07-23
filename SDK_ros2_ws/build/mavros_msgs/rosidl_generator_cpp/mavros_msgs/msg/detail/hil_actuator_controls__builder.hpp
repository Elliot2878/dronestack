// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:msg/HilActuatorControls.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__HIL_ACTUATOR_CONTROLS__BUILDER_HPP_
#define MAVROS_MSGS__MSG__DETAIL__HIL_ACTUATOR_CONTROLS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/msg/detail/hil_actuator_controls__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace msg
{

namespace builder
{

class Init_HilActuatorControls_flags
{
public:
  explicit Init_HilActuatorControls_flags(::mavros_msgs::msg::HilActuatorControls & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::msg::HilActuatorControls flags(::mavros_msgs::msg::HilActuatorControls::_flags_type arg)
  {
    msg_.flags = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::msg::HilActuatorControls msg_;
};

class Init_HilActuatorControls_mode
{
public:
  explicit Init_HilActuatorControls_mode(::mavros_msgs::msg::HilActuatorControls & msg)
  : msg_(msg)
  {}
  Init_HilActuatorControls_flags mode(::mavros_msgs::msg::HilActuatorControls::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return Init_HilActuatorControls_flags(msg_);
  }

private:
  ::mavros_msgs::msg::HilActuatorControls msg_;
};

class Init_HilActuatorControls_controls
{
public:
  explicit Init_HilActuatorControls_controls(::mavros_msgs::msg::HilActuatorControls & msg)
  : msg_(msg)
  {}
  Init_HilActuatorControls_mode controls(::mavros_msgs::msg::HilActuatorControls::_controls_type arg)
  {
    msg_.controls = std::move(arg);
    return Init_HilActuatorControls_mode(msg_);
  }

private:
  ::mavros_msgs::msg::HilActuatorControls msg_;
};

class Init_HilActuatorControls_header
{
public:
  Init_HilActuatorControls_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_HilActuatorControls_controls header(::mavros_msgs::msg::HilActuatorControls::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_HilActuatorControls_controls(msg_);
  }

private:
  ::mavros_msgs::msg::HilActuatorControls msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::msg::HilActuatorControls>()
{
  return mavros_msgs::msg::builder::Init_HilActuatorControls_header();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__HIL_ACTUATOR_CONTROLS__BUILDER_HPP_
