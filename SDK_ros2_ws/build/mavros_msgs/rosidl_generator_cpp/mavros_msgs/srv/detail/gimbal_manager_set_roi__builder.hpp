// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:srv/GimbalManagerSetRoi.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__GIMBAL_MANAGER_SET_ROI__BUILDER_HPP_
#define MAVROS_MSGS__SRV__DETAIL__GIMBAL_MANAGER_SET_ROI__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/srv/detail/gimbal_manager_set_roi__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace srv
{

namespace builder
{

class Init_GimbalManagerSetRoi_Request_sysid
{
public:
  explicit Init_GimbalManagerSetRoi_Request_sysid(::mavros_msgs::srv::GimbalManagerSetRoi_Request & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::srv::GimbalManagerSetRoi_Request sysid(::mavros_msgs::srv::GimbalManagerSetRoi_Request::_sysid_type arg)
  {
    msg_.sysid = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::srv::GimbalManagerSetRoi_Request msg_;
};

class Init_GimbalManagerSetRoi_Request_yaw_offset
{
public:
  explicit Init_GimbalManagerSetRoi_Request_yaw_offset(::mavros_msgs::srv::GimbalManagerSetRoi_Request & msg)
  : msg_(msg)
  {}
  Init_GimbalManagerSetRoi_Request_sysid yaw_offset(::mavros_msgs::srv::GimbalManagerSetRoi_Request::_yaw_offset_type arg)
  {
    msg_.yaw_offset = std::move(arg);
    return Init_GimbalManagerSetRoi_Request_sysid(msg_);
  }

private:
  ::mavros_msgs::srv::GimbalManagerSetRoi_Request msg_;
};

class Init_GimbalManagerSetRoi_Request_roll_offset
{
public:
  explicit Init_GimbalManagerSetRoi_Request_roll_offset(::mavros_msgs::srv::GimbalManagerSetRoi_Request & msg)
  : msg_(msg)
  {}
  Init_GimbalManagerSetRoi_Request_yaw_offset roll_offset(::mavros_msgs::srv::GimbalManagerSetRoi_Request::_roll_offset_type arg)
  {
    msg_.roll_offset = std::move(arg);
    return Init_GimbalManagerSetRoi_Request_yaw_offset(msg_);
  }

private:
  ::mavros_msgs::srv::GimbalManagerSetRoi_Request msg_;
};

class Init_GimbalManagerSetRoi_Request_pitch_offset
{
public:
  explicit Init_GimbalManagerSetRoi_Request_pitch_offset(::mavros_msgs::srv::GimbalManagerSetRoi_Request & msg)
  : msg_(msg)
  {}
  Init_GimbalManagerSetRoi_Request_roll_offset pitch_offset(::mavros_msgs::srv::GimbalManagerSetRoi_Request::_pitch_offset_type arg)
  {
    msg_.pitch_offset = std::move(arg);
    return Init_GimbalManagerSetRoi_Request_roll_offset(msg_);
  }

private:
  ::mavros_msgs::srv::GimbalManagerSetRoi_Request msg_;
};

class Init_GimbalManagerSetRoi_Request_altitude
{
public:
  explicit Init_GimbalManagerSetRoi_Request_altitude(::mavros_msgs::srv::GimbalManagerSetRoi_Request & msg)
  : msg_(msg)
  {}
  Init_GimbalManagerSetRoi_Request_pitch_offset altitude(::mavros_msgs::srv::GimbalManagerSetRoi_Request::_altitude_type arg)
  {
    msg_.altitude = std::move(arg);
    return Init_GimbalManagerSetRoi_Request_pitch_offset(msg_);
  }

private:
  ::mavros_msgs::srv::GimbalManagerSetRoi_Request msg_;
};

class Init_GimbalManagerSetRoi_Request_longitude
{
public:
  explicit Init_GimbalManagerSetRoi_Request_longitude(::mavros_msgs::srv::GimbalManagerSetRoi_Request & msg)
  : msg_(msg)
  {}
  Init_GimbalManagerSetRoi_Request_altitude longitude(::mavros_msgs::srv::GimbalManagerSetRoi_Request::_longitude_type arg)
  {
    msg_.longitude = std::move(arg);
    return Init_GimbalManagerSetRoi_Request_altitude(msg_);
  }

private:
  ::mavros_msgs::srv::GimbalManagerSetRoi_Request msg_;
};

class Init_GimbalManagerSetRoi_Request_latitude
{
public:
  explicit Init_GimbalManagerSetRoi_Request_latitude(::mavros_msgs::srv::GimbalManagerSetRoi_Request & msg)
  : msg_(msg)
  {}
  Init_GimbalManagerSetRoi_Request_longitude latitude(::mavros_msgs::srv::GimbalManagerSetRoi_Request::_latitude_type arg)
  {
    msg_.latitude = std::move(arg);
    return Init_GimbalManagerSetRoi_Request_longitude(msg_);
  }

private:
  ::mavros_msgs::srv::GimbalManagerSetRoi_Request msg_;
};

class Init_GimbalManagerSetRoi_Request_gimbal_device_id
{
public:
  explicit Init_GimbalManagerSetRoi_Request_gimbal_device_id(::mavros_msgs::srv::GimbalManagerSetRoi_Request & msg)
  : msg_(msg)
  {}
  Init_GimbalManagerSetRoi_Request_latitude gimbal_device_id(::mavros_msgs::srv::GimbalManagerSetRoi_Request::_gimbal_device_id_type arg)
  {
    msg_.gimbal_device_id = std::move(arg);
    return Init_GimbalManagerSetRoi_Request_latitude(msg_);
  }

private:
  ::mavros_msgs::srv::GimbalManagerSetRoi_Request msg_;
};

class Init_GimbalManagerSetRoi_Request_mode
{
public:
  Init_GimbalManagerSetRoi_Request_mode()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GimbalManagerSetRoi_Request_gimbal_device_id mode(::mavros_msgs::srv::GimbalManagerSetRoi_Request::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return Init_GimbalManagerSetRoi_Request_gimbal_device_id(msg_);
  }

private:
  ::mavros_msgs::srv::GimbalManagerSetRoi_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::srv::GimbalManagerSetRoi_Request>()
{
  return mavros_msgs::srv::builder::Init_GimbalManagerSetRoi_Request_mode();
}

}  // namespace mavros_msgs


namespace mavros_msgs
{

namespace srv
{

namespace builder
{

class Init_GimbalManagerSetRoi_Response_result
{
public:
  explicit Init_GimbalManagerSetRoi_Response_result(::mavros_msgs::srv::GimbalManagerSetRoi_Response & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::srv::GimbalManagerSetRoi_Response result(::mavros_msgs::srv::GimbalManagerSetRoi_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::srv::GimbalManagerSetRoi_Response msg_;
};

class Init_GimbalManagerSetRoi_Response_success
{
public:
  Init_GimbalManagerSetRoi_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GimbalManagerSetRoi_Response_result success(::mavros_msgs::srv::GimbalManagerSetRoi_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_GimbalManagerSetRoi_Response_result(msg_);
  }

private:
  ::mavros_msgs::srv::GimbalManagerSetRoi_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::srv::GimbalManagerSetRoi_Response>()
{
  return mavros_msgs::srv::builder::Init_GimbalManagerSetRoi_Response_success();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__SRV__DETAIL__GIMBAL_MANAGER_SET_ROI__BUILDER_HPP_
