// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:srv/SetMavFrame.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__SET_MAV_FRAME__BUILDER_HPP_
#define MAVROS_MSGS__SRV__DETAIL__SET_MAV_FRAME__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/srv/detail/set_mav_frame__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace srv
{

namespace builder
{

class Init_SetMavFrame_Request_mav_frame
{
public:
  Init_SetMavFrame_Request_mav_frame()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::mavros_msgs::srv::SetMavFrame_Request mav_frame(::mavros_msgs::srv::SetMavFrame_Request::_mav_frame_type arg)
  {
    msg_.mav_frame = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::srv::SetMavFrame_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::srv::SetMavFrame_Request>()
{
  return mavros_msgs::srv::builder::Init_SetMavFrame_Request_mav_frame();
}

}  // namespace mavros_msgs


namespace mavros_msgs
{

namespace srv
{

namespace builder
{

class Init_SetMavFrame_Response_success
{
public:
  Init_SetMavFrame_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::mavros_msgs::srv::SetMavFrame_Response success(::mavros_msgs::srv::SetMavFrame_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::srv::SetMavFrame_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::srv::SetMavFrame_Response>()
{
  return mavros_msgs::srv::builder::Init_SetMavFrame_Response_success();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__SRV__DETAIL__SET_MAV_FRAME__BUILDER_HPP_
