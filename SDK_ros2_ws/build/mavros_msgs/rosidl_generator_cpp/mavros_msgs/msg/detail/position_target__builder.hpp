// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:msg/PositionTarget.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__POSITION_TARGET__BUILDER_HPP_
#define MAVROS_MSGS__MSG__DETAIL__POSITION_TARGET__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/msg/detail/position_target__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace msg
{

namespace builder
{

class Init_PositionTarget_yaw_rate
{
public:
  explicit Init_PositionTarget_yaw_rate(::mavros_msgs::msg::PositionTarget & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::msg::PositionTarget yaw_rate(::mavros_msgs::msg::PositionTarget::_yaw_rate_type arg)
  {
    msg_.yaw_rate = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::msg::PositionTarget msg_;
};

class Init_PositionTarget_yaw
{
public:
  explicit Init_PositionTarget_yaw(::mavros_msgs::msg::PositionTarget & msg)
  : msg_(msg)
  {}
  Init_PositionTarget_yaw_rate yaw(::mavros_msgs::msg::PositionTarget::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_PositionTarget_yaw_rate(msg_);
  }

private:
  ::mavros_msgs::msg::PositionTarget msg_;
};

class Init_PositionTarget_acceleration_or_force
{
public:
  explicit Init_PositionTarget_acceleration_or_force(::mavros_msgs::msg::PositionTarget & msg)
  : msg_(msg)
  {}
  Init_PositionTarget_yaw acceleration_or_force(::mavros_msgs::msg::PositionTarget::_acceleration_or_force_type arg)
  {
    msg_.acceleration_or_force = std::move(arg);
    return Init_PositionTarget_yaw(msg_);
  }

private:
  ::mavros_msgs::msg::PositionTarget msg_;
};

class Init_PositionTarget_velocity
{
public:
  explicit Init_PositionTarget_velocity(::mavros_msgs::msg::PositionTarget & msg)
  : msg_(msg)
  {}
  Init_PositionTarget_acceleration_or_force velocity(::mavros_msgs::msg::PositionTarget::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_PositionTarget_acceleration_or_force(msg_);
  }

private:
  ::mavros_msgs::msg::PositionTarget msg_;
};

class Init_PositionTarget_position
{
public:
  explicit Init_PositionTarget_position(::mavros_msgs::msg::PositionTarget & msg)
  : msg_(msg)
  {}
  Init_PositionTarget_velocity position(::mavros_msgs::msg::PositionTarget::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_PositionTarget_velocity(msg_);
  }

private:
  ::mavros_msgs::msg::PositionTarget msg_;
};

class Init_PositionTarget_type_mask
{
public:
  explicit Init_PositionTarget_type_mask(::mavros_msgs::msg::PositionTarget & msg)
  : msg_(msg)
  {}
  Init_PositionTarget_position type_mask(::mavros_msgs::msg::PositionTarget::_type_mask_type arg)
  {
    msg_.type_mask = std::move(arg);
    return Init_PositionTarget_position(msg_);
  }

private:
  ::mavros_msgs::msg::PositionTarget msg_;
};

class Init_PositionTarget_coordinate_frame
{
public:
  explicit Init_PositionTarget_coordinate_frame(::mavros_msgs::msg::PositionTarget & msg)
  : msg_(msg)
  {}
  Init_PositionTarget_type_mask coordinate_frame(::mavros_msgs::msg::PositionTarget::_coordinate_frame_type arg)
  {
    msg_.coordinate_frame = std::move(arg);
    return Init_PositionTarget_type_mask(msg_);
  }

private:
  ::mavros_msgs::msg::PositionTarget msg_;
};

class Init_PositionTarget_header
{
public:
  Init_PositionTarget_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PositionTarget_coordinate_frame header(::mavros_msgs::msg::PositionTarget::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_PositionTarget_coordinate_frame(msg_);
  }

private:
  ::mavros_msgs::msg::PositionTarget msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::msg::PositionTarget>()
{
  return mavros_msgs::msg::builder::Init_PositionTarget_header();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__POSITION_TARGET__BUILDER_HPP_
