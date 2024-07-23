// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:msg/GimbalDeviceAttitudeStatus.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__GIMBAL_DEVICE_ATTITUDE_STATUS__BUILDER_HPP_
#define MAVROS_MSGS__MSG__DETAIL__GIMBAL_DEVICE_ATTITUDE_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/msg/detail/gimbal_device_attitude_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace msg
{

namespace builder
{

class Init_GimbalDeviceAttitudeStatus_failure_flags
{
public:
  explicit Init_GimbalDeviceAttitudeStatus_failure_flags(::mavros_msgs::msg::GimbalDeviceAttitudeStatus & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::msg::GimbalDeviceAttitudeStatus failure_flags(::mavros_msgs::msg::GimbalDeviceAttitudeStatus::_failure_flags_type arg)
  {
    msg_.failure_flags = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::msg::GimbalDeviceAttitudeStatus msg_;
};

class Init_GimbalDeviceAttitudeStatus_angular_velocity_z
{
public:
  explicit Init_GimbalDeviceAttitudeStatus_angular_velocity_z(::mavros_msgs::msg::GimbalDeviceAttitudeStatus & msg)
  : msg_(msg)
  {}
  Init_GimbalDeviceAttitudeStatus_failure_flags angular_velocity_z(::mavros_msgs::msg::GimbalDeviceAttitudeStatus::_angular_velocity_z_type arg)
  {
    msg_.angular_velocity_z = std::move(arg);
    return Init_GimbalDeviceAttitudeStatus_failure_flags(msg_);
  }

private:
  ::mavros_msgs::msg::GimbalDeviceAttitudeStatus msg_;
};

class Init_GimbalDeviceAttitudeStatus_angular_velocity_y
{
public:
  explicit Init_GimbalDeviceAttitudeStatus_angular_velocity_y(::mavros_msgs::msg::GimbalDeviceAttitudeStatus & msg)
  : msg_(msg)
  {}
  Init_GimbalDeviceAttitudeStatus_angular_velocity_z angular_velocity_y(::mavros_msgs::msg::GimbalDeviceAttitudeStatus::_angular_velocity_y_type arg)
  {
    msg_.angular_velocity_y = std::move(arg);
    return Init_GimbalDeviceAttitudeStatus_angular_velocity_z(msg_);
  }

private:
  ::mavros_msgs::msg::GimbalDeviceAttitudeStatus msg_;
};

class Init_GimbalDeviceAttitudeStatus_angular_velocity_x
{
public:
  explicit Init_GimbalDeviceAttitudeStatus_angular_velocity_x(::mavros_msgs::msg::GimbalDeviceAttitudeStatus & msg)
  : msg_(msg)
  {}
  Init_GimbalDeviceAttitudeStatus_angular_velocity_y angular_velocity_x(::mavros_msgs::msg::GimbalDeviceAttitudeStatus::_angular_velocity_x_type arg)
  {
    msg_.angular_velocity_x = std::move(arg);
    return Init_GimbalDeviceAttitudeStatus_angular_velocity_y(msg_);
  }

private:
  ::mavros_msgs::msg::GimbalDeviceAttitudeStatus msg_;
};

class Init_GimbalDeviceAttitudeStatus_q
{
public:
  explicit Init_GimbalDeviceAttitudeStatus_q(::mavros_msgs::msg::GimbalDeviceAttitudeStatus & msg)
  : msg_(msg)
  {}
  Init_GimbalDeviceAttitudeStatus_angular_velocity_x q(::mavros_msgs::msg::GimbalDeviceAttitudeStatus::_q_type arg)
  {
    msg_.q = std::move(arg);
    return Init_GimbalDeviceAttitudeStatus_angular_velocity_x(msg_);
  }

private:
  ::mavros_msgs::msg::GimbalDeviceAttitudeStatus msg_;
};

class Init_GimbalDeviceAttitudeStatus_flags
{
public:
  explicit Init_GimbalDeviceAttitudeStatus_flags(::mavros_msgs::msg::GimbalDeviceAttitudeStatus & msg)
  : msg_(msg)
  {}
  Init_GimbalDeviceAttitudeStatus_q flags(::mavros_msgs::msg::GimbalDeviceAttitudeStatus::_flags_type arg)
  {
    msg_.flags = std::move(arg);
    return Init_GimbalDeviceAttitudeStatus_q(msg_);
  }

private:
  ::mavros_msgs::msg::GimbalDeviceAttitudeStatus msg_;
};

class Init_GimbalDeviceAttitudeStatus_target_component
{
public:
  explicit Init_GimbalDeviceAttitudeStatus_target_component(::mavros_msgs::msg::GimbalDeviceAttitudeStatus & msg)
  : msg_(msg)
  {}
  Init_GimbalDeviceAttitudeStatus_flags target_component(::mavros_msgs::msg::GimbalDeviceAttitudeStatus::_target_component_type arg)
  {
    msg_.target_component = std::move(arg);
    return Init_GimbalDeviceAttitudeStatus_flags(msg_);
  }

private:
  ::mavros_msgs::msg::GimbalDeviceAttitudeStatus msg_;
};

class Init_GimbalDeviceAttitudeStatus_target_system
{
public:
  explicit Init_GimbalDeviceAttitudeStatus_target_system(::mavros_msgs::msg::GimbalDeviceAttitudeStatus & msg)
  : msg_(msg)
  {}
  Init_GimbalDeviceAttitudeStatus_target_component target_system(::mavros_msgs::msg::GimbalDeviceAttitudeStatus::_target_system_type arg)
  {
    msg_.target_system = std::move(arg);
    return Init_GimbalDeviceAttitudeStatus_target_component(msg_);
  }

private:
  ::mavros_msgs::msg::GimbalDeviceAttitudeStatus msg_;
};

class Init_GimbalDeviceAttitudeStatus_header
{
public:
  Init_GimbalDeviceAttitudeStatus_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GimbalDeviceAttitudeStatus_target_system header(::mavros_msgs::msg::GimbalDeviceAttitudeStatus::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_GimbalDeviceAttitudeStatus_target_system(msg_);
  }

private:
  ::mavros_msgs::msg::GimbalDeviceAttitudeStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::msg::GimbalDeviceAttitudeStatus>()
{
  return mavros_msgs::msg::builder::Init_GimbalDeviceAttitudeStatus_header();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__GIMBAL_DEVICE_ATTITUDE_STATUS__BUILDER_HPP_
