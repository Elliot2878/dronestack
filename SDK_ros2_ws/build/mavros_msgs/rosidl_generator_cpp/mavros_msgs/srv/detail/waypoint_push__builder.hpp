// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:srv/WaypointPush.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__WAYPOINT_PUSH__BUILDER_HPP_
#define MAVROS_MSGS__SRV__DETAIL__WAYPOINT_PUSH__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/srv/detail/waypoint_push__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace srv
{

namespace builder
{

class Init_WaypointPush_Request_waypoints
{
public:
  explicit Init_WaypointPush_Request_waypoints(::mavros_msgs::srv::WaypointPush_Request & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::srv::WaypointPush_Request waypoints(::mavros_msgs::srv::WaypointPush_Request::_waypoints_type arg)
  {
    msg_.waypoints = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::srv::WaypointPush_Request msg_;
};

class Init_WaypointPush_Request_start_index
{
public:
  Init_WaypointPush_Request_start_index()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_WaypointPush_Request_waypoints start_index(::mavros_msgs::srv::WaypointPush_Request::_start_index_type arg)
  {
    msg_.start_index = std::move(arg);
    return Init_WaypointPush_Request_waypoints(msg_);
  }

private:
  ::mavros_msgs::srv::WaypointPush_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::srv::WaypointPush_Request>()
{
  return mavros_msgs::srv::builder::Init_WaypointPush_Request_start_index();
}

}  // namespace mavros_msgs


namespace mavros_msgs
{

namespace srv
{

namespace builder
{

class Init_WaypointPush_Response_wp_transfered
{
public:
  explicit Init_WaypointPush_Response_wp_transfered(::mavros_msgs::srv::WaypointPush_Response & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::srv::WaypointPush_Response wp_transfered(::mavros_msgs::srv::WaypointPush_Response::_wp_transfered_type arg)
  {
    msg_.wp_transfered = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::srv::WaypointPush_Response msg_;
};

class Init_WaypointPush_Response_success
{
public:
  Init_WaypointPush_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_WaypointPush_Response_wp_transfered success(::mavros_msgs::srv::WaypointPush_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_WaypointPush_Response_wp_transfered(msg_);
  }

private:
  ::mavros_msgs::srv::WaypointPush_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::srv::WaypointPush_Response>()
{
  return mavros_msgs::srv::builder::Init_WaypointPush_Response_success();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__SRV__DETAIL__WAYPOINT_PUSH__BUILDER_HPP_
