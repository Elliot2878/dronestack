// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:msg/HomePosition.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__HOME_POSITION__BUILDER_HPP_
#define MAVROS_MSGS__MSG__DETAIL__HOME_POSITION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/msg/detail/home_position__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace msg
{

namespace builder
{

class Init_HomePosition_approach
{
public:
  explicit Init_HomePosition_approach(::mavros_msgs::msg::HomePosition & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::msg::HomePosition approach(::mavros_msgs::msg::HomePosition::_approach_type arg)
  {
    msg_.approach = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::msg::HomePosition msg_;
};

class Init_HomePosition_orientation
{
public:
  explicit Init_HomePosition_orientation(::mavros_msgs::msg::HomePosition & msg)
  : msg_(msg)
  {}
  Init_HomePosition_approach orientation(::mavros_msgs::msg::HomePosition::_orientation_type arg)
  {
    msg_.orientation = std::move(arg);
    return Init_HomePosition_approach(msg_);
  }

private:
  ::mavros_msgs::msg::HomePosition msg_;
};

class Init_HomePosition_position
{
public:
  explicit Init_HomePosition_position(::mavros_msgs::msg::HomePosition & msg)
  : msg_(msg)
  {}
  Init_HomePosition_orientation position(::mavros_msgs::msg::HomePosition::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_HomePosition_orientation(msg_);
  }

private:
  ::mavros_msgs::msg::HomePosition msg_;
};

class Init_HomePosition_geo
{
public:
  explicit Init_HomePosition_geo(::mavros_msgs::msg::HomePosition & msg)
  : msg_(msg)
  {}
  Init_HomePosition_position geo(::mavros_msgs::msg::HomePosition::_geo_type arg)
  {
    msg_.geo = std::move(arg);
    return Init_HomePosition_position(msg_);
  }

private:
  ::mavros_msgs::msg::HomePosition msg_;
};

class Init_HomePosition_header
{
public:
  Init_HomePosition_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_HomePosition_geo header(::mavros_msgs::msg::HomePosition::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_HomePosition_geo(msg_);
  }

private:
  ::mavros_msgs::msg::HomePosition msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::msg::HomePosition>()
{
  return mavros_msgs::msg::builder::Init_HomePosition_header();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__HOME_POSITION__BUILDER_HPP_
