// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:msg/MountControl.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__MOUNT_CONTROL__BUILDER_HPP_
#define MAVROS_MSGS__MSG__DETAIL__MOUNT_CONTROL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/msg/detail/mount_control__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace msg
{

namespace builder
{

class Init_MountControl_longitude
{
public:
  explicit Init_MountControl_longitude(::mavros_msgs::msg::MountControl & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::msg::MountControl longitude(::mavros_msgs::msg::MountControl::_longitude_type arg)
  {
    msg_.longitude = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::msg::MountControl msg_;
};

class Init_MountControl_latitude
{
public:
  explicit Init_MountControl_latitude(::mavros_msgs::msg::MountControl & msg)
  : msg_(msg)
  {}
  Init_MountControl_longitude latitude(::mavros_msgs::msg::MountControl::_latitude_type arg)
  {
    msg_.latitude = std::move(arg);
    return Init_MountControl_longitude(msg_);
  }

private:
  ::mavros_msgs::msg::MountControl msg_;
};

class Init_MountControl_altitude
{
public:
  explicit Init_MountControl_altitude(::mavros_msgs::msg::MountControl & msg)
  : msg_(msg)
  {}
  Init_MountControl_latitude altitude(::mavros_msgs::msg::MountControl::_altitude_type arg)
  {
    msg_.altitude = std::move(arg);
    return Init_MountControl_latitude(msg_);
  }

private:
  ::mavros_msgs::msg::MountControl msg_;
};

class Init_MountControl_yaw
{
public:
  explicit Init_MountControl_yaw(::mavros_msgs::msg::MountControl & msg)
  : msg_(msg)
  {}
  Init_MountControl_altitude yaw(::mavros_msgs::msg::MountControl::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_MountControl_altitude(msg_);
  }

private:
  ::mavros_msgs::msg::MountControl msg_;
};

class Init_MountControl_roll
{
public:
  explicit Init_MountControl_roll(::mavros_msgs::msg::MountControl & msg)
  : msg_(msg)
  {}
  Init_MountControl_yaw roll(::mavros_msgs::msg::MountControl::_roll_type arg)
  {
    msg_.roll = std::move(arg);
    return Init_MountControl_yaw(msg_);
  }

private:
  ::mavros_msgs::msg::MountControl msg_;
};

class Init_MountControl_pitch
{
public:
  explicit Init_MountControl_pitch(::mavros_msgs::msg::MountControl & msg)
  : msg_(msg)
  {}
  Init_MountControl_roll pitch(::mavros_msgs::msg::MountControl::_pitch_type arg)
  {
    msg_.pitch = std::move(arg);
    return Init_MountControl_roll(msg_);
  }

private:
  ::mavros_msgs::msg::MountControl msg_;
};

class Init_MountControl_mode
{
public:
  explicit Init_MountControl_mode(::mavros_msgs::msg::MountControl & msg)
  : msg_(msg)
  {}
  Init_MountControl_pitch mode(::mavros_msgs::msg::MountControl::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return Init_MountControl_pitch(msg_);
  }

private:
  ::mavros_msgs::msg::MountControl msg_;
};

class Init_MountControl_header
{
public:
  Init_MountControl_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MountControl_mode header(::mavros_msgs::msg::MountControl::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MountControl_mode(msg_);
  }

private:
  ::mavros_msgs::msg::MountControl msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::msg::MountControl>()
{
  return mavros_msgs::msg::builder::Init_MountControl_header();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__MOUNT_CONTROL__BUILDER_HPP_
