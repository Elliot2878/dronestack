// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:msg/WaypointList.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__WAYPOINT_LIST__BUILDER_HPP_
#define MAVROS_MSGS__MSG__DETAIL__WAYPOINT_LIST__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/msg/detail/waypoint_list__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace msg
{

namespace builder
{

class Init_WaypointList_waypoints
{
public:
  explicit Init_WaypointList_waypoints(::mavros_msgs::msg::WaypointList & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::msg::WaypointList waypoints(::mavros_msgs::msg::WaypointList::_waypoints_type arg)
  {
    msg_.waypoints = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::msg::WaypointList msg_;
};

class Init_WaypointList_current_seq
{
public:
  Init_WaypointList_current_seq()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_WaypointList_waypoints current_seq(::mavros_msgs::msg::WaypointList::_current_seq_type arg)
  {
    msg_.current_seq = std::move(arg);
    return Init_WaypointList_waypoints(msg_);
  }

private:
  ::mavros_msgs::msg::WaypointList msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::msg::WaypointList>()
{
  return mavros_msgs::msg::builder::Init_WaypointList_current_seq();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__WAYPOINT_LIST__BUILDER_HPP_
