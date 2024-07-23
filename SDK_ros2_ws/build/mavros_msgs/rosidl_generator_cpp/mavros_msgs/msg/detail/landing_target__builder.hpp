// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:msg/LandingTarget.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__LANDING_TARGET__BUILDER_HPP_
#define MAVROS_MSGS__MSG__DETAIL__LANDING_TARGET__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/msg/detail/landing_target__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace msg
{

namespace builder
{

class Init_LandingTarget_type
{
public:
  explicit Init_LandingTarget_type(::mavros_msgs::msg::LandingTarget & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::msg::LandingTarget type(::mavros_msgs::msg::LandingTarget::_type_type arg)
  {
    msg_.type = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::msg::LandingTarget msg_;
};

class Init_LandingTarget_pose
{
public:
  explicit Init_LandingTarget_pose(::mavros_msgs::msg::LandingTarget & msg)
  : msg_(msg)
  {}
  Init_LandingTarget_type pose(::mavros_msgs::msg::LandingTarget::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return Init_LandingTarget_type(msg_);
  }

private:
  ::mavros_msgs::msg::LandingTarget msg_;
};

class Init_LandingTarget_size
{
public:
  explicit Init_LandingTarget_size(::mavros_msgs::msg::LandingTarget & msg)
  : msg_(msg)
  {}
  Init_LandingTarget_pose size(::mavros_msgs::msg::LandingTarget::_size_type arg)
  {
    msg_.size = std::move(arg);
    return Init_LandingTarget_pose(msg_);
  }

private:
  ::mavros_msgs::msg::LandingTarget msg_;
};

class Init_LandingTarget_distance
{
public:
  explicit Init_LandingTarget_distance(::mavros_msgs::msg::LandingTarget & msg)
  : msg_(msg)
  {}
  Init_LandingTarget_size distance(::mavros_msgs::msg::LandingTarget::_distance_type arg)
  {
    msg_.distance = std::move(arg);
    return Init_LandingTarget_size(msg_);
  }

private:
  ::mavros_msgs::msg::LandingTarget msg_;
};

class Init_LandingTarget_angle
{
public:
  explicit Init_LandingTarget_angle(::mavros_msgs::msg::LandingTarget & msg)
  : msg_(msg)
  {}
  Init_LandingTarget_distance angle(::mavros_msgs::msg::LandingTarget::_angle_type arg)
  {
    msg_.angle = std::move(arg);
    return Init_LandingTarget_distance(msg_);
  }

private:
  ::mavros_msgs::msg::LandingTarget msg_;
};

class Init_LandingTarget_frame
{
public:
  explicit Init_LandingTarget_frame(::mavros_msgs::msg::LandingTarget & msg)
  : msg_(msg)
  {}
  Init_LandingTarget_angle frame(::mavros_msgs::msg::LandingTarget::_frame_type arg)
  {
    msg_.frame = std::move(arg);
    return Init_LandingTarget_angle(msg_);
  }

private:
  ::mavros_msgs::msg::LandingTarget msg_;
};

class Init_LandingTarget_target_num
{
public:
  explicit Init_LandingTarget_target_num(::mavros_msgs::msg::LandingTarget & msg)
  : msg_(msg)
  {}
  Init_LandingTarget_frame target_num(::mavros_msgs::msg::LandingTarget::_target_num_type arg)
  {
    msg_.target_num = std::move(arg);
    return Init_LandingTarget_frame(msg_);
  }

private:
  ::mavros_msgs::msg::LandingTarget msg_;
};

class Init_LandingTarget_header
{
public:
  Init_LandingTarget_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LandingTarget_target_num header(::mavros_msgs::msg::LandingTarget::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_LandingTarget_target_num(msg_);
  }

private:
  ::mavros_msgs::msg::LandingTarget msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::msg::LandingTarget>()
{
  return mavros_msgs::msg::builder::Init_LandingTarget_header();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__LANDING_TARGET__BUILDER_HPP_
