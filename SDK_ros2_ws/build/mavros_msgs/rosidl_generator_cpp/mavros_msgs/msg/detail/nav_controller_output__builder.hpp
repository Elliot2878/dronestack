// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:msg/NavControllerOutput.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__NAV_CONTROLLER_OUTPUT__BUILDER_HPP_
#define MAVROS_MSGS__MSG__DETAIL__NAV_CONTROLLER_OUTPUT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/msg/detail/nav_controller_output__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace msg
{

namespace builder
{

class Init_NavControllerOutput_xtrack_error
{
public:
  explicit Init_NavControllerOutput_xtrack_error(::mavros_msgs::msg::NavControllerOutput & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::msg::NavControllerOutput xtrack_error(::mavros_msgs::msg::NavControllerOutput::_xtrack_error_type arg)
  {
    msg_.xtrack_error = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::msg::NavControllerOutput msg_;
};

class Init_NavControllerOutput_aspd_error
{
public:
  explicit Init_NavControllerOutput_aspd_error(::mavros_msgs::msg::NavControllerOutput & msg)
  : msg_(msg)
  {}
  Init_NavControllerOutput_xtrack_error aspd_error(::mavros_msgs::msg::NavControllerOutput::_aspd_error_type arg)
  {
    msg_.aspd_error = std::move(arg);
    return Init_NavControllerOutput_xtrack_error(msg_);
  }

private:
  ::mavros_msgs::msg::NavControllerOutput msg_;
};

class Init_NavControllerOutput_alt_error
{
public:
  explicit Init_NavControllerOutput_alt_error(::mavros_msgs::msg::NavControllerOutput & msg)
  : msg_(msg)
  {}
  Init_NavControllerOutput_aspd_error alt_error(::mavros_msgs::msg::NavControllerOutput::_alt_error_type arg)
  {
    msg_.alt_error = std::move(arg);
    return Init_NavControllerOutput_aspd_error(msg_);
  }

private:
  ::mavros_msgs::msg::NavControllerOutput msg_;
};

class Init_NavControllerOutput_wp_dist
{
public:
  explicit Init_NavControllerOutput_wp_dist(::mavros_msgs::msg::NavControllerOutput & msg)
  : msg_(msg)
  {}
  Init_NavControllerOutput_alt_error wp_dist(::mavros_msgs::msg::NavControllerOutput::_wp_dist_type arg)
  {
    msg_.wp_dist = std::move(arg);
    return Init_NavControllerOutput_alt_error(msg_);
  }

private:
  ::mavros_msgs::msg::NavControllerOutput msg_;
};

class Init_NavControllerOutput_target_bearing
{
public:
  explicit Init_NavControllerOutput_target_bearing(::mavros_msgs::msg::NavControllerOutput & msg)
  : msg_(msg)
  {}
  Init_NavControllerOutput_wp_dist target_bearing(::mavros_msgs::msg::NavControllerOutput::_target_bearing_type arg)
  {
    msg_.target_bearing = std::move(arg);
    return Init_NavControllerOutput_wp_dist(msg_);
  }

private:
  ::mavros_msgs::msg::NavControllerOutput msg_;
};

class Init_NavControllerOutput_nav_bearing
{
public:
  explicit Init_NavControllerOutput_nav_bearing(::mavros_msgs::msg::NavControllerOutput & msg)
  : msg_(msg)
  {}
  Init_NavControllerOutput_target_bearing nav_bearing(::mavros_msgs::msg::NavControllerOutput::_nav_bearing_type arg)
  {
    msg_.nav_bearing = std::move(arg);
    return Init_NavControllerOutput_target_bearing(msg_);
  }

private:
  ::mavros_msgs::msg::NavControllerOutput msg_;
};

class Init_NavControllerOutput_nav_pitch
{
public:
  explicit Init_NavControllerOutput_nav_pitch(::mavros_msgs::msg::NavControllerOutput & msg)
  : msg_(msg)
  {}
  Init_NavControllerOutput_nav_bearing nav_pitch(::mavros_msgs::msg::NavControllerOutput::_nav_pitch_type arg)
  {
    msg_.nav_pitch = std::move(arg);
    return Init_NavControllerOutput_nav_bearing(msg_);
  }

private:
  ::mavros_msgs::msg::NavControllerOutput msg_;
};

class Init_NavControllerOutput_nav_roll
{
public:
  explicit Init_NavControllerOutput_nav_roll(::mavros_msgs::msg::NavControllerOutput & msg)
  : msg_(msg)
  {}
  Init_NavControllerOutput_nav_pitch nav_roll(::mavros_msgs::msg::NavControllerOutput::_nav_roll_type arg)
  {
    msg_.nav_roll = std::move(arg);
    return Init_NavControllerOutput_nav_pitch(msg_);
  }

private:
  ::mavros_msgs::msg::NavControllerOutput msg_;
};

class Init_NavControllerOutput_header
{
public:
  Init_NavControllerOutput_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_NavControllerOutput_nav_roll header(::mavros_msgs::msg::NavControllerOutput::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_NavControllerOutput_nav_roll(msg_);
  }

private:
  ::mavros_msgs::msg::NavControllerOutput msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::msg::NavControllerOutput>()
{
  return mavros_msgs::msg::builder::Init_NavControllerOutput_header();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__NAV_CONTROLLER_OUTPUT__BUILDER_HPP_
