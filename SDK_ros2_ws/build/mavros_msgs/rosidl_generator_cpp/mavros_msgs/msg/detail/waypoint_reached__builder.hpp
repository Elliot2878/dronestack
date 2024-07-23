// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:msg/WaypointReached.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__WAYPOINT_REACHED__BUILDER_HPP_
#define MAVROS_MSGS__MSG__DETAIL__WAYPOINT_REACHED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/msg/detail/waypoint_reached__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace msg
{

namespace builder
{

class Init_WaypointReached_wp_seq
{
public:
  explicit Init_WaypointReached_wp_seq(::mavros_msgs::msg::WaypointReached & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::msg::WaypointReached wp_seq(::mavros_msgs::msg::WaypointReached::_wp_seq_type arg)
  {
    msg_.wp_seq = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::msg::WaypointReached msg_;
};

class Init_WaypointReached_header
{
public:
  Init_WaypointReached_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_WaypointReached_wp_seq header(::mavros_msgs::msg::WaypointReached::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_WaypointReached_wp_seq(msg_);
  }

private:
  ::mavros_msgs::msg::WaypointReached msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::msg::WaypointReached>()
{
  return mavros_msgs::msg::builder::Init_WaypointReached_header();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__WAYPOINT_REACHED__BUILDER_HPP_
