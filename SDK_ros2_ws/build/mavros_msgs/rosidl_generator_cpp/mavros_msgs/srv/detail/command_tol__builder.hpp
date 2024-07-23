// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:srv/CommandTOL.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__COMMAND_TOL__BUILDER_HPP_
#define MAVROS_MSGS__SRV__DETAIL__COMMAND_TOL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/srv/detail/command_tol__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace srv
{

namespace builder
{

class Init_CommandTOL_Request_altitude
{
public:
  explicit Init_CommandTOL_Request_altitude(::mavros_msgs::srv::CommandTOL_Request & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::srv::CommandTOL_Request altitude(::mavros_msgs::srv::CommandTOL_Request::_altitude_type arg)
  {
    msg_.altitude = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::srv::CommandTOL_Request msg_;
};

class Init_CommandTOL_Request_longitude
{
public:
  explicit Init_CommandTOL_Request_longitude(::mavros_msgs::srv::CommandTOL_Request & msg)
  : msg_(msg)
  {}
  Init_CommandTOL_Request_altitude longitude(::mavros_msgs::srv::CommandTOL_Request::_longitude_type arg)
  {
    msg_.longitude = std::move(arg);
    return Init_CommandTOL_Request_altitude(msg_);
  }

private:
  ::mavros_msgs::srv::CommandTOL_Request msg_;
};

class Init_CommandTOL_Request_latitude
{
public:
  explicit Init_CommandTOL_Request_latitude(::mavros_msgs::srv::CommandTOL_Request & msg)
  : msg_(msg)
  {}
  Init_CommandTOL_Request_longitude latitude(::mavros_msgs::srv::CommandTOL_Request::_latitude_type arg)
  {
    msg_.latitude = std::move(arg);
    return Init_CommandTOL_Request_longitude(msg_);
  }

private:
  ::mavros_msgs::srv::CommandTOL_Request msg_;
};

class Init_CommandTOL_Request_yaw
{
public:
  explicit Init_CommandTOL_Request_yaw(::mavros_msgs::srv::CommandTOL_Request & msg)
  : msg_(msg)
  {}
  Init_CommandTOL_Request_latitude yaw(::mavros_msgs::srv::CommandTOL_Request::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_CommandTOL_Request_latitude(msg_);
  }

private:
  ::mavros_msgs::srv::CommandTOL_Request msg_;
};

class Init_CommandTOL_Request_min_pitch
{
public:
  Init_CommandTOL_Request_min_pitch()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CommandTOL_Request_yaw min_pitch(::mavros_msgs::srv::CommandTOL_Request::_min_pitch_type arg)
  {
    msg_.min_pitch = std::move(arg);
    return Init_CommandTOL_Request_yaw(msg_);
  }

private:
  ::mavros_msgs::srv::CommandTOL_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::srv::CommandTOL_Request>()
{
  return mavros_msgs::srv::builder::Init_CommandTOL_Request_min_pitch();
}

}  // namespace mavros_msgs


namespace mavros_msgs
{

namespace srv
{

namespace builder
{

class Init_CommandTOL_Response_result
{
public:
  explicit Init_CommandTOL_Response_result(::mavros_msgs::srv::CommandTOL_Response & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::srv::CommandTOL_Response result(::mavros_msgs::srv::CommandTOL_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::srv::CommandTOL_Response msg_;
};

class Init_CommandTOL_Response_success
{
public:
  Init_CommandTOL_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CommandTOL_Response_result success(::mavros_msgs::srv::CommandTOL_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_CommandTOL_Response_result(msg_);
  }

private:
  ::mavros_msgs::srv::CommandTOL_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::srv::CommandTOL_Response>()
{
  return mavros_msgs::srv::builder::Init_CommandTOL_Response_success();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__SRV__DETAIL__COMMAND_TOL__BUILDER_HPP_
