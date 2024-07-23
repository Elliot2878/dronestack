// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:srv/CommandTOLLocal.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__COMMAND_TOL_LOCAL__BUILDER_HPP_
#define MAVROS_MSGS__SRV__DETAIL__COMMAND_TOL_LOCAL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/srv/detail/command_tol_local__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace srv
{

namespace builder
{

class Init_CommandTOLLocal_Request_position
{
public:
  explicit Init_CommandTOLLocal_Request_position(::mavros_msgs::srv::CommandTOLLocal_Request & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::srv::CommandTOLLocal_Request position(::mavros_msgs::srv::CommandTOLLocal_Request::_position_type arg)
  {
    msg_.position = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::srv::CommandTOLLocal_Request msg_;
};

class Init_CommandTOLLocal_Request_yaw
{
public:
  explicit Init_CommandTOLLocal_Request_yaw(::mavros_msgs::srv::CommandTOLLocal_Request & msg)
  : msg_(msg)
  {}
  Init_CommandTOLLocal_Request_position yaw(::mavros_msgs::srv::CommandTOLLocal_Request::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_CommandTOLLocal_Request_position(msg_);
  }

private:
  ::mavros_msgs::srv::CommandTOLLocal_Request msg_;
};

class Init_CommandTOLLocal_Request_rate
{
public:
  explicit Init_CommandTOLLocal_Request_rate(::mavros_msgs::srv::CommandTOLLocal_Request & msg)
  : msg_(msg)
  {}
  Init_CommandTOLLocal_Request_yaw rate(::mavros_msgs::srv::CommandTOLLocal_Request::_rate_type arg)
  {
    msg_.rate = std::move(arg);
    return Init_CommandTOLLocal_Request_yaw(msg_);
  }

private:
  ::mavros_msgs::srv::CommandTOLLocal_Request msg_;
};

class Init_CommandTOLLocal_Request_offset
{
public:
  explicit Init_CommandTOLLocal_Request_offset(::mavros_msgs::srv::CommandTOLLocal_Request & msg)
  : msg_(msg)
  {}
  Init_CommandTOLLocal_Request_rate offset(::mavros_msgs::srv::CommandTOLLocal_Request::_offset_type arg)
  {
    msg_.offset = std::move(arg);
    return Init_CommandTOLLocal_Request_rate(msg_);
  }

private:
  ::mavros_msgs::srv::CommandTOLLocal_Request msg_;
};

class Init_CommandTOLLocal_Request_min_pitch
{
public:
  Init_CommandTOLLocal_Request_min_pitch()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CommandTOLLocal_Request_offset min_pitch(::mavros_msgs::srv::CommandTOLLocal_Request::_min_pitch_type arg)
  {
    msg_.min_pitch = std::move(arg);
    return Init_CommandTOLLocal_Request_offset(msg_);
  }

private:
  ::mavros_msgs::srv::CommandTOLLocal_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::srv::CommandTOLLocal_Request>()
{
  return mavros_msgs::srv::builder::Init_CommandTOLLocal_Request_min_pitch();
}

}  // namespace mavros_msgs


namespace mavros_msgs
{

namespace srv
{

namespace builder
{

class Init_CommandTOLLocal_Response_result
{
public:
  explicit Init_CommandTOLLocal_Response_result(::mavros_msgs::srv::CommandTOLLocal_Response & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::srv::CommandTOLLocal_Response result(::mavros_msgs::srv::CommandTOLLocal_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::srv::CommandTOLLocal_Response msg_;
};

class Init_CommandTOLLocal_Response_success
{
public:
  Init_CommandTOLLocal_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CommandTOLLocal_Response_result success(::mavros_msgs::srv::CommandTOLLocal_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_CommandTOLLocal_Response_result(msg_);
  }

private:
  ::mavros_msgs::srv::CommandTOLLocal_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::srv::CommandTOLLocal_Response>()
{
  return mavros_msgs::srv::builder::Init_CommandTOLLocal_Response_success();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__SRV__DETAIL__COMMAND_TOL_LOCAL__BUILDER_HPP_
