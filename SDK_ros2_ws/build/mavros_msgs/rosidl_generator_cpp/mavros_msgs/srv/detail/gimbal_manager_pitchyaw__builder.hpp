// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:srv/GimbalManagerPitchyaw.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__GIMBAL_MANAGER_PITCHYAW__BUILDER_HPP_
#define MAVROS_MSGS__SRV__DETAIL__GIMBAL_MANAGER_PITCHYAW__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/srv/detail/gimbal_manager_pitchyaw__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace srv
{

namespace builder
{

class Init_GimbalManagerPitchyaw_Request_gimbal_device_id
{
public:
  explicit Init_GimbalManagerPitchyaw_Request_gimbal_device_id(::mavros_msgs::srv::GimbalManagerPitchyaw_Request & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::srv::GimbalManagerPitchyaw_Request gimbal_device_id(::mavros_msgs::srv::GimbalManagerPitchyaw_Request::_gimbal_device_id_type arg)
  {
    msg_.gimbal_device_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::srv::GimbalManagerPitchyaw_Request msg_;
};

class Init_GimbalManagerPitchyaw_Request_flags
{
public:
  explicit Init_GimbalManagerPitchyaw_Request_flags(::mavros_msgs::srv::GimbalManagerPitchyaw_Request & msg)
  : msg_(msg)
  {}
  Init_GimbalManagerPitchyaw_Request_gimbal_device_id flags(::mavros_msgs::srv::GimbalManagerPitchyaw_Request::_flags_type arg)
  {
    msg_.flags = std::move(arg);
    return Init_GimbalManagerPitchyaw_Request_gimbal_device_id(msg_);
  }

private:
  ::mavros_msgs::srv::GimbalManagerPitchyaw_Request msg_;
};

class Init_GimbalManagerPitchyaw_Request_yaw_rate
{
public:
  explicit Init_GimbalManagerPitchyaw_Request_yaw_rate(::mavros_msgs::srv::GimbalManagerPitchyaw_Request & msg)
  : msg_(msg)
  {}
  Init_GimbalManagerPitchyaw_Request_flags yaw_rate(::mavros_msgs::srv::GimbalManagerPitchyaw_Request::_yaw_rate_type arg)
  {
    msg_.yaw_rate = std::move(arg);
    return Init_GimbalManagerPitchyaw_Request_flags(msg_);
  }

private:
  ::mavros_msgs::srv::GimbalManagerPitchyaw_Request msg_;
};

class Init_GimbalManagerPitchyaw_Request_pitch_rate
{
public:
  explicit Init_GimbalManagerPitchyaw_Request_pitch_rate(::mavros_msgs::srv::GimbalManagerPitchyaw_Request & msg)
  : msg_(msg)
  {}
  Init_GimbalManagerPitchyaw_Request_yaw_rate pitch_rate(::mavros_msgs::srv::GimbalManagerPitchyaw_Request::_pitch_rate_type arg)
  {
    msg_.pitch_rate = std::move(arg);
    return Init_GimbalManagerPitchyaw_Request_yaw_rate(msg_);
  }

private:
  ::mavros_msgs::srv::GimbalManagerPitchyaw_Request msg_;
};

class Init_GimbalManagerPitchyaw_Request_yaw
{
public:
  explicit Init_GimbalManagerPitchyaw_Request_yaw(::mavros_msgs::srv::GimbalManagerPitchyaw_Request & msg)
  : msg_(msg)
  {}
  Init_GimbalManagerPitchyaw_Request_pitch_rate yaw(::mavros_msgs::srv::GimbalManagerPitchyaw_Request::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_GimbalManagerPitchyaw_Request_pitch_rate(msg_);
  }

private:
  ::mavros_msgs::srv::GimbalManagerPitchyaw_Request msg_;
};

class Init_GimbalManagerPitchyaw_Request_pitch
{
public:
  Init_GimbalManagerPitchyaw_Request_pitch()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GimbalManagerPitchyaw_Request_yaw pitch(::mavros_msgs::srv::GimbalManagerPitchyaw_Request::_pitch_type arg)
  {
    msg_.pitch = std::move(arg);
    return Init_GimbalManagerPitchyaw_Request_yaw(msg_);
  }

private:
  ::mavros_msgs::srv::GimbalManagerPitchyaw_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::srv::GimbalManagerPitchyaw_Request>()
{
  return mavros_msgs::srv::builder::Init_GimbalManagerPitchyaw_Request_pitch();
}

}  // namespace mavros_msgs


namespace mavros_msgs
{

namespace srv
{

namespace builder
{

class Init_GimbalManagerPitchyaw_Response_result
{
public:
  explicit Init_GimbalManagerPitchyaw_Response_result(::mavros_msgs::srv::GimbalManagerPitchyaw_Response & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::srv::GimbalManagerPitchyaw_Response result(::mavros_msgs::srv::GimbalManagerPitchyaw_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::srv::GimbalManagerPitchyaw_Response msg_;
};

class Init_GimbalManagerPitchyaw_Response_success
{
public:
  Init_GimbalManagerPitchyaw_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GimbalManagerPitchyaw_Response_result success(::mavros_msgs::srv::GimbalManagerPitchyaw_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_GimbalManagerPitchyaw_Response_result(msg_);
  }

private:
  ::mavros_msgs::srv::GimbalManagerPitchyaw_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::srv::GimbalManagerPitchyaw_Response>()
{
  return mavros_msgs::srv::builder::Init_GimbalManagerPitchyaw_Response_success();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__SRV__DETAIL__GIMBAL_MANAGER_PITCHYAW__BUILDER_HPP_
