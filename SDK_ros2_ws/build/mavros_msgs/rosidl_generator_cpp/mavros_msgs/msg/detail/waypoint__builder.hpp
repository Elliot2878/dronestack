// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:msg/Waypoint.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__WAYPOINT__BUILDER_HPP_
#define MAVROS_MSGS__MSG__DETAIL__WAYPOINT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/msg/detail/waypoint__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace msg
{

namespace builder
{

class Init_Waypoint_z_alt
{
public:
  explicit Init_Waypoint_z_alt(::mavros_msgs::msg::Waypoint & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::msg::Waypoint z_alt(::mavros_msgs::msg::Waypoint::_z_alt_type arg)
  {
    msg_.z_alt = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::msg::Waypoint msg_;
};

class Init_Waypoint_y_long
{
public:
  explicit Init_Waypoint_y_long(::mavros_msgs::msg::Waypoint & msg)
  : msg_(msg)
  {}
  Init_Waypoint_z_alt y_long(::mavros_msgs::msg::Waypoint::_y_long_type arg)
  {
    msg_.y_long = std::move(arg);
    return Init_Waypoint_z_alt(msg_);
  }

private:
  ::mavros_msgs::msg::Waypoint msg_;
};

class Init_Waypoint_x_lat
{
public:
  explicit Init_Waypoint_x_lat(::mavros_msgs::msg::Waypoint & msg)
  : msg_(msg)
  {}
  Init_Waypoint_y_long x_lat(::mavros_msgs::msg::Waypoint::_x_lat_type arg)
  {
    msg_.x_lat = std::move(arg);
    return Init_Waypoint_y_long(msg_);
  }

private:
  ::mavros_msgs::msg::Waypoint msg_;
};

class Init_Waypoint_param4
{
public:
  explicit Init_Waypoint_param4(::mavros_msgs::msg::Waypoint & msg)
  : msg_(msg)
  {}
  Init_Waypoint_x_lat param4(::mavros_msgs::msg::Waypoint::_param4_type arg)
  {
    msg_.param4 = std::move(arg);
    return Init_Waypoint_x_lat(msg_);
  }

private:
  ::mavros_msgs::msg::Waypoint msg_;
};

class Init_Waypoint_param3
{
public:
  explicit Init_Waypoint_param3(::mavros_msgs::msg::Waypoint & msg)
  : msg_(msg)
  {}
  Init_Waypoint_param4 param3(::mavros_msgs::msg::Waypoint::_param3_type arg)
  {
    msg_.param3 = std::move(arg);
    return Init_Waypoint_param4(msg_);
  }

private:
  ::mavros_msgs::msg::Waypoint msg_;
};

class Init_Waypoint_param2
{
public:
  explicit Init_Waypoint_param2(::mavros_msgs::msg::Waypoint & msg)
  : msg_(msg)
  {}
  Init_Waypoint_param3 param2(::mavros_msgs::msg::Waypoint::_param2_type arg)
  {
    msg_.param2 = std::move(arg);
    return Init_Waypoint_param3(msg_);
  }

private:
  ::mavros_msgs::msg::Waypoint msg_;
};

class Init_Waypoint_param1
{
public:
  explicit Init_Waypoint_param1(::mavros_msgs::msg::Waypoint & msg)
  : msg_(msg)
  {}
  Init_Waypoint_param2 param1(::mavros_msgs::msg::Waypoint::_param1_type arg)
  {
    msg_.param1 = std::move(arg);
    return Init_Waypoint_param2(msg_);
  }

private:
  ::mavros_msgs::msg::Waypoint msg_;
};

class Init_Waypoint_autocontinue
{
public:
  explicit Init_Waypoint_autocontinue(::mavros_msgs::msg::Waypoint & msg)
  : msg_(msg)
  {}
  Init_Waypoint_param1 autocontinue(::mavros_msgs::msg::Waypoint::_autocontinue_type arg)
  {
    msg_.autocontinue = std::move(arg);
    return Init_Waypoint_param1(msg_);
  }

private:
  ::mavros_msgs::msg::Waypoint msg_;
};

class Init_Waypoint_is_current
{
public:
  explicit Init_Waypoint_is_current(::mavros_msgs::msg::Waypoint & msg)
  : msg_(msg)
  {}
  Init_Waypoint_autocontinue is_current(::mavros_msgs::msg::Waypoint::_is_current_type arg)
  {
    msg_.is_current = std::move(arg);
    return Init_Waypoint_autocontinue(msg_);
  }

private:
  ::mavros_msgs::msg::Waypoint msg_;
};

class Init_Waypoint_command
{
public:
  explicit Init_Waypoint_command(::mavros_msgs::msg::Waypoint & msg)
  : msg_(msg)
  {}
  Init_Waypoint_is_current command(::mavros_msgs::msg::Waypoint::_command_type arg)
  {
    msg_.command = std::move(arg);
    return Init_Waypoint_is_current(msg_);
  }

private:
  ::mavros_msgs::msg::Waypoint msg_;
};

class Init_Waypoint_frame
{
public:
  Init_Waypoint_frame()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Waypoint_command frame(::mavros_msgs::msg::Waypoint::_frame_type arg)
  {
    msg_.frame = std::move(arg);
    return Init_Waypoint_command(msg_);
  }

private:
  ::mavros_msgs::msg::Waypoint msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::msg::Waypoint>()
{
  return mavros_msgs::msg::builder::Init_Waypoint_frame();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__WAYPOINT__BUILDER_HPP_
