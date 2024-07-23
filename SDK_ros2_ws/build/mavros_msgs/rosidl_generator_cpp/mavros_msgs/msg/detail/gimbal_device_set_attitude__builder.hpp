// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:msg/GimbalDeviceSetAttitude.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__GIMBAL_DEVICE_SET_ATTITUDE__BUILDER_HPP_
#define MAVROS_MSGS__MSG__DETAIL__GIMBAL_DEVICE_SET_ATTITUDE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/msg/detail/gimbal_device_set_attitude__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace msg
{

namespace builder
{

class Init_GimbalDeviceSetAttitude_angular_velocity_z
{
public:
  explicit Init_GimbalDeviceSetAttitude_angular_velocity_z(::mavros_msgs::msg::GimbalDeviceSetAttitude & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::msg::GimbalDeviceSetAttitude angular_velocity_z(::mavros_msgs::msg::GimbalDeviceSetAttitude::_angular_velocity_z_type arg)
  {
    msg_.angular_velocity_z = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::msg::GimbalDeviceSetAttitude msg_;
};

class Init_GimbalDeviceSetAttitude_angular_velocity_y
{
public:
  explicit Init_GimbalDeviceSetAttitude_angular_velocity_y(::mavros_msgs::msg::GimbalDeviceSetAttitude & msg)
  : msg_(msg)
  {}
  Init_GimbalDeviceSetAttitude_angular_velocity_z angular_velocity_y(::mavros_msgs::msg::GimbalDeviceSetAttitude::_angular_velocity_y_type arg)
  {
    msg_.angular_velocity_y = std::move(arg);
    return Init_GimbalDeviceSetAttitude_angular_velocity_z(msg_);
  }

private:
  ::mavros_msgs::msg::GimbalDeviceSetAttitude msg_;
};

class Init_GimbalDeviceSetAttitude_angular_velocity_x
{
public:
  explicit Init_GimbalDeviceSetAttitude_angular_velocity_x(::mavros_msgs::msg::GimbalDeviceSetAttitude & msg)
  : msg_(msg)
  {}
  Init_GimbalDeviceSetAttitude_angular_velocity_y angular_velocity_x(::mavros_msgs::msg::GimbalDeviceSetAttitude::_angular_velocity_x_type arg)
  {
    msg_.angular_velocity_x = std::move(arg);
    return Init_GimbalDeviceSetAttitude_angular_velocity_y(msg_);
  }

private:
  ::mavros_msgs::msg::GimbalDeviceSetAttitude msg_;
};

class Init_GimbalDeviceSetAttitude_q
{
public:
  explicit Init_GimbalDeviceSetAttitude_q(::mavros_msgs::msg::GimbalDeviceSetAttitude & msg)
  : msg_(msg)
  {}
  Init_GimbalDeviceSetAttitude_angular_velocity_x q(::mavros_msgs::msg::GimbalDeviceSetAttitude::_q_type arg)
  {
    msg_.q = std::move(arg);
    return Init_GimbalDeviceSetAttitude_angular_velocity_x(msg_);
  }

private:
  ::mavros_msgs::msg::GimbalDeviceSetAttitude msg_;
};

class Init_GimbalDeviceSetAttitude_flags
{
public:
  explicit Init_GimbalDeviceSetAttitude_flags(::mavros_msgs::msg::GimbalDeviceSetAttitude & msg)
  : msg_(msg)
  {}
  Init_GimbalDeviceSetAttitude_q flags(::mavros_msgs::msg::GimbalDeviceSetAttitude::_flags_type arg)
  {
    msg_.flags = std::move(arg);
    return Init_GimbalDeviceSetAttitude_q(msg_);
  }

private:
  ::mavros_msgs::msg::GimbalDeviceSetAttitude msg_;
};

class Init_GimbalDeviceSetAttitude_target_component
{
public:
  explicit Init_GimbalDeviceSetAttitude_target_component(::mavros_msgs::msg::GimbalDeviceSetAttitude & msg)
  : msg_(msg)
  {}
  Init_GimbalDeviceSetAttitude_flags target_component(::mavros_msgs::msg::GimbalDeviceSetAttitude::_target_component_type arg)
  {
    msg_.target_component = std::move(arg);
    return Init_GimbalDeviceSetAttitude_flags(msg_);
  }

private:
  ::mavros_msgs::msg::GimbalDeviceSetAttitude msg_;
};

class Init_GimbalDeviceSetAttitude_target_system
{
public:
  Init_GimbalDeviceSetAttitude_target_system()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GimbalDeviceSetAttitude_target_component target_system(::mavros_msgs::msg::GimbalDeviceSetAttitude::_target_system_type arg)
  {
    msg_.target_system = std::move(arg);
    return Init_GimbalDeviceSetAttitude_target_component(msg_);
  }

private:
  ::mavros_msgs::msg::GimbalDeviceSetAttitude msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::msg::GimbalDeviceSetAttitude>()
{
  return mavros_msgs::msg::builder::Init_GimbalDeviceSetAttitude_target_system();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__GIMBAL_DEVICE_SET_ATTITUDE__BUILDER_HPP_