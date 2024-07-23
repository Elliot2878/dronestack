// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:srv/WaypointSetCurrent.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__WAYPOINT_SET_CURRENT__BUILDER_HPP_
#define MAVROS_MSGS__SRV__DETAIL__WAYPOINT_SET_CURRENT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/srv/detail/waypoint_set_current__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace srv
{

namespace builder
{

class Init_WaypointSetCurrent_Request_wp_seq
{
public:
  Init_WaypointSetCurrent_Request_wp_seq()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::mavros_msgs::srv::WaypointSetCurrent_Request wp_seq(::mavros_msgs::srv::WaypointSetCurrent_Request::_wp_seq_type arg)
  {
    msg_.wp_seq = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::srv::WaypointSetCurrent_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::srv::WaypointSetCurrent_Request>()
{
  return mavros_msgs::srv::builder::Init_WaypointSetCurrent_Request_wp_seq();
}

}  // namespace mavros_msgs


namespace mavros_msgs
{

namespace srv
{

namespace builder
{

class Init_WaypointSetCurrent_Response_success
{
public:
  Init_WaypointSetCurrent_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::mavros_msgs::srv::WaypointSetCurrent_Response success(::mavros_msgs::srv::WaypointSetCurrent_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::srv::WaypointSetCurrent_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::srv::WaypointSetCurrent_Response>()
{
  return mavros_msgs::srv::builder::Init_WaypointSetCurrent_Response_success();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__SRV__DETAIL__WAYPOINT_SET_CURRENT__BUILDER_HPP_
