// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:msg/HilControls.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__HIL_CONTROLS__BUILDER_HPP_
#define MAVROS_MSGS__MSG__DETAIL__HIL_CONTROLS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/msg/detail/hil_controls__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace msg
{

namespace builder
{

class Init_HilControls_nav_mode
{
public:
  explicit Init_HilControls_nav_mode(::mavros_msgs::msg::HilControls & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::msg::HilControls nav_mode(::mavros_msgs::msg::HilControls::_nav_mode_type arg)
  {
    msg_.nav_mode = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::msg::HilControls msg_;
};

class Init_HilControls_mode
{
public:
  explicit Init_HilControls_mode(::mavros_msgs::msg::HilControls & msg)
  : msg_(msg)
  {}
  Init_HilControls_nav_mode mode(::mavros_msgs::msg::HilControls::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return Init_HilControls_nav_mode(msg_);
  }

private:
  ::mavros_msgs::msg::HilControls msg_;
};

class Init_HilControls_aux4
{
public:
  explicit Init_HilControls_aux4(::mavros_msgs::msg::HilControls & msg)
  : msg_(msg)
  {}
  Init_HilControls_mode aux4(::mavros_msgs::msg::HilControls::_aux4_type arg)
  {
    msg_.aux4 = std::move(arg);
    return Init_HilControls_mode(msg_);
  }

private:
  ::mavros_msgs::msg::HilControls msg_;
};

class Init_HilControls_aux3
{
public:
  explicit Init_HilControls_aux3(::mavros_msgs::msg::HilControls & msg)
  : msg_(msg)
  {}
  Init_HilControls_aux4 aux3(::mavros_msgs::msg::HilControls::_aux3_type arg)
  {
    msg_.aux3 = std::move(arg);
    return Init_HilControls_aux4(msg_);
  }

private:
  ::mavros_msgs::msg::HilControls msg_;
};

class Init_HilControls_aux2
{
public:
  explicit Init_HilControls_aux2(::mavros_msgs::msg::HilControls & msg)
  : msg_(msg)
  {}
  Init_HilControls_aux3 aux2(::mavros_msgs::msg::HilControls::_aux2_type arg)
  {
    msg_.aux2 = std::move(arg);
    return Init_HilControls_aux3(msg_);
  }

private:
  ::mavros_msgs::msg::HilControls msg_;
};

class Init_HilControls_aux1
{
public:
  explicit Init_HilControls_aux1(::mavros_msgs::msg::HilControls & msg)
  : msg_(msg)
  {}
  Init_HilControls_aux2 aux1(::mavros_msgs::msg::HilControls::_aux1_type arg)
  {
    msg_.aux1 = std::move(arg);
    return Init_HilControls_aux2(msg_);
  }

private:
  ::mavros_msgs::msg::HilControls msg_;
};

class Init_HilControls_throttle
{
public:
  explicit Init_HilControls_throttle(::mavros_msgs::msg::HilControls & msg)
  : msg_(msg)
  {}
  Init_HilControls_aux1 throttle(::mavros_msgs::msg::HilControls::_throttle_type arg)
  {
    msg_.throttle = std::move(arg);
    return Init_HilControls_aux1(msg_);
  }

private:
  ::mavros_msgs::msg::HilControls msg_;
};

class Init_HilControls_yaw_rudder
{
public:
  explicit Init_HilControls_yaw_rudder(::mavros_msgs::msg::HilControls & msg)
  : msg_(msg)
  {}
  Init_HilControls_throttle yaw_rudder(::mavros_msgs::msg::HilControls::_yaw_rudder_type arg)
  {
    msg_.yaw_rudder = std::move(arg);
    return Init_HilControls_throttle(msg_);
  }

private:
  ::mavros_msgs::msg::HilControls msg_;
};

class Init_HilControls_pitch_elevator
{
public:
  explicit Init_HilControls_pitch_elevator(::mavros_msgs::msg::HilControls & msg)
  : msg_(msg)
  {}
  Init_HilControls_yaw_rudder pitch_elevator(::mavros_msgs::msg::HilControls::_pitch_elevator_type arg)
  {
    msg_.pitch_elevator = std::move(arg);
    return Init_HilControls_yaw_rudder(msg_);
  }

private:
  ::mavros_msgs::msg::HilControls msg_;
};

class Init_HilControls_roll_ailerons
{
public:
  explicit Init_HilControls_roll_ailerons(::mavros_msgs::msg::HilControls & msg)
  : msg_(msg)
  {}
  Init_HilControls_pitch_elevator roll_ailerons(::mavros_msgs::msg::HilControls::_roll_ailerons_type arg)
  {
    msg_.roll_ailerons = std::move(arg);
    return Init_HilControls_pitch_elevator(msg_);
  }

private:
  ::mavros_msgs::msg::HilControls msg_;
};

class Init_HilControls_header
{
public:
  Init_HilControls_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_HilControls_roll_ailerons header(::mavros_msgs::msg::HilControls::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_HilControls_roll_ailerons(msg_);
  }

private:
  ::mavros_msgs::msg::HilControls msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::msg::HilControls>()
{
  return mavros_msgs::msg::builder::Init_HilControls_header();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__HIL_CONTROLS__BUILDER_HPP_
