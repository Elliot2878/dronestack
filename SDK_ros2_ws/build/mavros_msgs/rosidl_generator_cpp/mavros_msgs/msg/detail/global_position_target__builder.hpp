// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:msg/GlobalPositionTarget.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__GLOBAL_POSITION_TARGET__BUILDER_HPP_
#define MAVROS_MSGS__MSG__DETAIL__GLOBAL_POSITION_TARGET__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/msg/detail/global_position_target__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace msg
{

namespace builder
{

class Init_GlobalPositionTarget_yaw_rate
{
public:
  explicit Init_GlobalPositionTarget_yaw_rate(::mavros_msgs::msg::GlobalPositionTarget & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::msg::GlobalPositionTarget yaw_rate(::mavros_msgs::msg::GlobalPositionTarget::_yaw_rate_type arg)
  {
    msg_.yaw_rate = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::msg::GlobalPositionTarget msg_;
};

class Init_GlobalPositionTarget_yaw
{
public:
  explicit Init_GlobalPositionTarget_yaw(::mavros_msgs::msg::GlobalPositionTarget & msg)
  : msg_(msg)
  {}
  Init_GlobalPositionTarget_yaw_rate yaw(::mavros_msgs::msg::GlobalPositionTarget::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_GlobalPositionTarget_yaw_rate(msg_);
  }

private:
  ::mavros_msgs::msg::GlobalPositionTarget msg_;
};

class Init_GlobalPositionTarget_acceleration_or_force
{
public:
  explicit Init_GlobalPositionTarget_acceleration_or_force(::mavros_msgs::msg::GlobalPositionTarget & msg)
  : msg_(msg)
  {}
  Init_GlobalPositionTarget_yaw acceleration_or_force(::mavros_msgs::msg::GlobalPositionTarget::_acceleration_or_force_type arg)
  {
    msg_.acceleration_or_force = std::move(arg);
    return Init_GlobalPositionTarget_yaw(msg_);
  }

private:
  ::mavros_msgs::msg::GlobalPositionTarget msg_;
};

class Init_GlobalPositionTarget_velocity
{
public:
  explicit Init_GlobalPositionTarget_velocity(::mavros_msgs::msg::GlobalPositionTarget & msg)
  : msg_(msg)
  {}
  Init_GlobalPositionTarget_acceleration_or_force velocity(::mavros_msgs::msg::GlobalPositionTarget::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_GlobalPositionTarget_acceleration_or_force(msg_);
  }

private:
  ::mavros_msgs::msg::GlobalPositionTarget msg_;
};

class Init_GlobalPositionTarget_altitude
{
public:
  explicit Init_GlobalPositionTarget_altitude(::mavros_msgs::msg::GlobalPositionTarget & msg)
  : msg_(msg)
  {}
  Init_GlobalPositionTarget_velocity altitude(::mavros_msgs::msg::GlobalPositionTarget::_altitude_type arg)
  {
    msg_.altitude = std::move(arg);
    return Init_GlobalPositionTarget_velocity(msg_);
  }

private:
  ::mavros_msgs::msg::GlobalPositionTarget msg_;
};

class Init_GlobalPositionTarget_longitude
{
public:
  explicit Init_GlobalPositionTarget_longitude(::mavros_msgs::msg::GlobalPositionTarget & msg)
  : msg_(msg)
  {}
  Init_GlobalPositionTarget_altitude longitude(::mavros_msgs::msg::GlobalPositionTarget::_longitude_type arg)
  {
    msg_.longitude = std::move(arg);
    return Init_GlobalPositionTarget_altitude(msg_);
  }

private:
  ::mavros_msgs::msg::GlobalPositionTarget msg_;
};

class Init_GlobalPositionTarget_latitude
{
public:
  explicit Init_GlobalPositionTarget_latitude(::mavros_msgs::msg::GlobalPositionTarget & msg)
  : msg_(msg)
  {}
  Init_GlobalPositionTarget_longitude latitude(::mavros_msgs::msg::GlobalPositionTarget::_latitude_type arg)
  {
    msg_.latitude = std::move(arg);
    return Init_GlobalPositionTarget_longitude(msg_);
  }

private:
  ::mavros_msgs::msg::GlobalPositionTarget msg_;
};

class Init_GlobalPositionTarget_type_mask
{
public:
  explicit Init_GlobalPositionTarget_type_mask(::mavros_msgs::msg::GlobalPositionTarget & msg)
  : msg_(msg)
  {}
  Init_GlobalPositionTarget_latitude type_mask(::mavros_msgs::msg::GlobalPositionTarget::_type_mask_type arg)
  {
    msg_.type_mask = std::move(arg);
    return Init_GlobalPositionTarget_latitude(msg_);
  }

private:
  ::mavros_msgs::msg::GlobalPositionTarget msg_;
};

class Init_GlobalPositionTarget_coordinate_frame
{
public:
  explicit Init_GlobalPositionTarget_coordinate_frame(::mavros_msgs::msg::GlobalPositionTarget & msg)
  : msg_(msg)
  {}
  Init_GlobalPositionTarget_type_mask coordinate_frame(::mavros_msgs::msg::GlobalPositionTarget::_coordinate_frame_type arg)
  {
    msg_.coordinate_frame = std::move(arg);
    return Init_GlobalPositionTarget_type_mask(msg_);
  }

private:
  ::mavros_msgs::msg::GlobalPositionTarget msg_;
};

class Init_GlobalPositionTarget_header
{
public:
  Init_GlobalPositionTarget_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GlobalPositionTarget_coordinate_frame header(::mavros_msgs::msg::GlobalPositionTarget::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_GlobalPositionTarget_coordinate_frame(msg_);
  }

private:
  ::mavros_msgs::msg::GlobalPositionTarget msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::msg::GlobalPositionTarget>()
{
  return mavros_msgs::msg::builder::Init_GlobalPositionTarget_header();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__GLOBAL_POSITION_TARGET__BUILDER_HPP_
