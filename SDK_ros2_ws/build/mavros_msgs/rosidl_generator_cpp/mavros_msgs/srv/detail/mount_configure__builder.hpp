// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:srv/MountConfigure.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__MOUNT_CONFIGURE__BUILDER_HPP_
#define MAVROS_MSGS__SRV__DETAIL__MOUNT_CONFIGURE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/srv/detail/mount_configure__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace srv
{

namespace builder
{

class Init_MountConfigure_Request_yaw_input
{
public:
  explicit Init_MountConfigure_Request_yaw_input(::mavros_msgs::srv::MountConfigure_Request & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::srv::MountConfigure_Request yaw_input(::mavros_msgs::srv::MountConfigure_Request::_yaw_input_type arg)
  {
    msg_.yaw_input = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::srv::MountConfigure_Request msg_;
};

class Init_MountConfigure_Request_pitch_input
{
public:
  explicit Init_MountConfigure_Request_pitch_input(::mavros_msgs::srv::MountConfigure_Request & msg)
  : msg_(msg)
  {}
  Init_MountConfigure_Request_yaw_input pitch_input(::mavros_msgs::srv::MountConfigure_Request::_pitch_input_type arg)
  {
    msg_.pitch_input = std::move(arg);
    return Init_MountConfigure_Request_yaw_input(msg_);
  }

private:
  ::mavros_msgs::srv::MountConfigure_Request msg_;
};

class Init_MountConfigure_Request_roll_input
{
public:
  explicit Init_MountConfigure_Request_roll_input(::mavros_msgs::srv::MountConfigure_Request & msg)
  : msg_(msg)
  {}
  Init_MountConfigure_Request_pitch_input roll_input(::mavros_msgs::srv::MountConfigure_Request::_roll_input_type arg)
  {
    msg_.roll_input = std::move(arg);
    return Init_MountConfigure_Request_pitch_input(msg_);
  }

private:
  ::mavros_msgs::srv::MountConfigure_Request msg_;
};

class Init_MountConfigure_Request_stabilize_yaw
{
public:
  explicit Init_MountConfigure_Request_stabilize_yaw(::mavros_msgs::srv::MountConfigure_Request & msg)
  : msg_(msg)
  {}
  Init_MountConfigure_Request_roll_input stabilize_yaw(::mavros_msgs::srv::MountConfigure_Request::_stabilize_yaw_type arg)
  {
    msg_.stabilize_yaw = std::move(arg);
    return Init_MountConfigure_Request_roll_input(msg_);
  }

private:
  ::mavros_msgs::srv::MountConfigure_Request msg_;
};

class Init_MountConfigure_Request_stabilize_pitch
{
public:
  explicit Init_MountConfigure_Request_stabilize_pitch(::mavros_msgs::srv::MountConfigure_Request & msg)
  : msg_(msg)
  {}
  Init_MountConfigure_Request_stabilize_yaw stabilize_pitch(::mavros_msgs::srv::MountConfigure_Request::_stabilize_pitch_type arg)
  {
    msg_.stabilize_pitch = std::move(arg);
    return Init_MountConfigure_Request_stabilize_yaw(msg_);
  }

private:
  ::mavros_msgs::srv::MountConfigure_Request msg_;
};

class Init_MountConfigure_Request_stabilize_roll
{
public:
  explicit Init_MountConfigure_Request_stabilize_roll(::mavros_msgs::srv::MountConfigure_Request & msg)
  : msg_(msg)
  {}
  Init_MountConfigure_Request_stabilize_pitch stabilize_roll(::mavros_msgs::srv::MountConfigure_Request::_stabilize_roll_type arg)
  {
    msg_.stabilize_roll = std::move(arg);
    return Init_MountConfigure_Request_stabilize_pitch(msg_);
  }

private:
  ::mavros_msgs::srv::MountConfigure_Request msg_;
};

class Init_MountConfigure_Request_mode
{
public:
  explicit Init_MountConfigure_Request_mode(::mavros_msgs::srv::MountConfigure_Request & msg)
  : msg_(msg)
  {}
  Init_MountConfigure_Request_stabilize_roll mode(::mavros_msgs::srv::MountConfigure_Request::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return Init_MountConfigure_Request_stabilize_roll(msg_);
  }

private:
  ::mavros_msgs::srv::MountConfigure_Request msg_;
};

class Init_MountConfigure_Request_header
{
public:
  Init_MountConfigure_Request_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MountConfigure_Request_mode header(::mavros_msgs::srv::MountConfigure_Request::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MountConfigure_Request_mode(msg_);
  }

private:
  ::mavros_msgs::srv::MountConfigure_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::srv::MountConfigure_Request>()
{
  return mavros_msgs::srv::builder::Init_MountConfigure_Request_header();
}

}  // namespace mavros_msgs


namespace mavros_msgs
{

namespace srv
{

namespace builder
{

class Init_MountConfigure_Response_result
{
public:
  explicit Init_MountConfigure_Response_result(::mavros_msgs::srv::MountConfigure_Response & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::srv::MountConfigure_Response result(::mavros_msgs::srv::MountConfigure_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::srv::MountConfigure_Response msg_;
};

class Init_MountConfigure_Response_success
{
public:
  Init_MountConfigure_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MountConfigure_Response_result success(::mavros_msgs::srv::MountConfigure_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_MountConfigure_Response_result(msg_);
  }

private:
  ::mavros_msgs::srv::MountConfigure_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::srv::MountConfigure_Response>()
{
  return mavros_msgs::srv::builder::Init_MountConfigure_Response_success();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__SRV__DETAIL__MOUNT_CONFIGURE__BUILDER_HPP_
