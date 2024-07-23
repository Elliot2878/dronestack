// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:msg/VfrHud.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__VFR_HUD__BUILDER_HPP_
#define MAVROS_MSGS__MSG__DETAIL__VFR_HUD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/msg/detail/vfr_hud__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace msg
{

namespace builder
{

class Init_VfrHud_climb
{
public:
  explicit Init_VfrHud_climb(::mavros_msgs::msg::VfrHud & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::msg::VfrHud climb(::mavros_msgs::msg::VfrHud::_climb_type arg)
  {
    msg_.climb = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::msg::VfrHud msg_;
};

class Init_VfrHud_altitude
{
public:
  explicit Init_VfrHud_altitude(::mavros_msgs::msg::VfrHud & msg)
  : msg_(msg)
  {}
  Init_VfrHud_climb altitude(::mavros_msgs::msg::VfrHud::_altitude_type arg)
  {
    msg_.altitude = std::move(arg);
    return Init_VfrHud_climb(msg_);
  }

private:
  ::mavros_msgs::msg::VfrHud msg_;
};

class Init_VfrHud_throttle
{
public:
  explicit Init_VfrHud_throttle(::mavros_msgs::msg::VfrHud & msg)
  : msg_(msg)
  {}
  Init_VfrHud_altitude throttle(::mavros_msgs::msg::VfrHud::_throttle_type arg)
  {
    msg_.throttle = std::move(arg);
    return Init_VfrHud_altitude(msg_);
  }

private:
  ::mavros_msgs::msg::VfrHud msg_;
};

class Init_VfrHud_heading
{
public:
  explicit Init_VfrHud_heading(::mavros_msgs::msg::VfrHud & msg)
  : msg_(msg)
  {}
  Init_VfrHud_throttle heading(::mavros_msgs::msg::VfrHud::_heading_type arg)
  {
    msg_.heading = std::move(arg);
    return Init_VfrHud_throttle(msg_);
  }

private:
  ::mavros_msgs::msg::VfrHud msg_;
};

class Init_VfrHud_groundspeed
{
public:
  explicit Init_VfrHud_groundspeed(::mavros_msgs::msg::VfrHud & msg)
  : msg_(msg)
  {}
  Init_VfrHud_heading groundspeed(::mavros_msgs::msg::VfrHud::_groundspeed_type arg)
  {
    msg_.groundspeed = std::move(arg);
    return Init_VfrHud_heading(msg_);
  }

private:
  ::mavros_msgs::msg::VfrHud msg_;
};

class Init_VfrHud_airspeed
{
public:
  explicit Init_VfrHud_airspeed(::mavros_msgs::msg::VfrHud & msg)
  : msg_(msg)
  {}
  Init_VfrHud_groundspeed airspeed(::mavros_msgs::msg::VfrHud::_airspeed_type arg)
  {
    msg_.airspeed = std::move(arg);
    return Init_VfrHud_groundspeed(msg_);
  }

private:
  ::mavros_msgs::msg::VfrHud msg_;
};

class Init_VfrHud_header
{
public:
  Init_VfrHud_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_VfrHud_airspeed header(::mavros_msgs::msg::VfrHud::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_VfrHud_airspeed(msg_);
  }

private:
  ::mavros_msgs::msg::VfrHud msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::msg::VfrHud>()
{
  return mavros_msgs::msg::builder::Init_VfrHud_header();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__VFR_HUD__BUILDER_HPP_
