// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:msg/HilStateQuaternion.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__HIL_STATE_QUATERNION__BUILDER_HPP_
#define MAVROS_MSGS__MSG__DETAIL__HIL_STATE_QUATERNION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/msg/detail/hil_state_quaternion__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace msg
{

namespace builder
{

class Init_HilStateQuaternion_true_airspeed
{
public:
  explicit Init_HilStateQuaternion_true_airspeed(::mavros_msgs::msg::HilStateQuaternion & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::msg::HilStateQuaternion true_airspeed(::mavros_msgs::msg::HilStateQuaternion::_true_airspeed_type arg)
  {
    msg_.true_airspeed = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::msg::HilStateQuaternion msg_;
};

class Init_HilStateQuaternion_ind_airspeed
{
public:
  explicit Init_HilStateQuaternion_ind_airspeed(::mavros_msgs::msg::HilStateQuaternion & msg)
  : msg_(msg)
  {}
  Init_HilStateQuaternion_true_airspeed ind_airspeed(::mavros_msgs::msg::HilStateQuaternion::_ind_airspeed_type arg)
  {
    msg_.ind_airspeed = std::move(arg);
    return Init_HilStateQuaternion_true_airspeed(msg_);
  }

private:
  ::mavros_msgs::msg::HilStateQuaternion msg_;
};

class Init_HilStateQuaternion_geo
{
public:
  explicit Init_HilStateQuaternion_geo(::mavros_msgs::msg::HilStateQuaternion & msg)
  : msg_(msg)
  {}
  Init_HilStateQuaternion_ind_airspeed geo(::mavros_msgs::msg::HilStateQuaternion::_geo_type arg)
  {
    msg_.geo = std::move(arg);
    return Init_HilStateQuaternion_ind_airspeed(msg_);
  }

private:
  ::mavros_msgs::msg::HilStateQuaternion msg_;
};

class Init_HilStateQuaternion_linear_velocity
{
public:
  explicit Init_HilStateQuaternion_linear_velocity(::mavros_msgs::msg::HilStateQuaternion & msg)
  : msg_(msg)
  {}
  Init_HilStateQuaternion_geo linear_velocity(::mavros_msgs::msg::HilStateQuaternion::_linear_velocity_type arg)
  {
    msg_.linear_velocity = std::move(arg);
    return Init_HilStateQuaternion_geo(msg_);
  }

private:
  ::mavros_msgs::msg::HilStateQuaternion msg_;
};

class Init_HilStateQuaternion_linear_acceleration
{
public:
  explicit Init_HilStateQuaternion_linear_acceleration(::mavros_msgs::msg::HilStateQuaternion & msg)
  : msg_(msg)
  {}
  Init_HilStateQuaternion_linear_velocity linear_acceleration(::mavros_msgs::msg::HilStateQuaternion::_linear_acceleration_type arg)
  {
    msg_.linear_acceleration = std::move(arg);
    return Init_HilStateQuaternion_linear_velocity(msg_);
  }

private:
  ::mavros_msgs::msg::HilStateQuaternion msg_;
};

class Init_HilStateQuaternion_angular_velocity
{
public:
  explicit Init_HilStateQuaternion_angular_velocity(::mavros_msgs::msg::HilStateQuaternion & msg)
  : msg_(msg)
  {}
  Init_HilStateQuaternion_linear_acceleration angular_velocity(::mavros_msgs::msg::HilStateQuaternion::_angular_velocity_type arg)
  {
    msg_.angular_velocity = std::move(arg);
    return Init_HilStateQuaternion_linear_acceleration(msg_);
  }

private:
  ::mavros_msgs::msg::HilStateQuaternion msg_;
};

class Init_HilStateQuaternion_orientation
{
public:
  explicit Init_HilStateQuaternion_orientation(::mavros_msgs::msg::HilStateQuaternion & msg)
  : msg_(msg)
  {}
  Init_HilStateQuaternion_angular_velocity orientation(::mavros_msgs::msg::HilStateQuaternion::_orientation_type arg)
  {
    msg_.orientation = std::move(arg);
    return Init_HilStateQuaternion_angular_velocity(msg_);
  }

private:
  ::mavros_msgs::msg::HilStateQuaternion msg_;
};

class Init_HilStateQuaternion_header
{
public:
  Init_HilStateQuaternion_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_HilStateQuaternion_orientation header(::mavros_msgs::msg::HilStateQuaternion::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_HilStateQuaternion_orientation(msg_);
  }

private:
  ::mavros_msgs::msg::HilStateQuaternion msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::msg::HilStateQuaternion>()
{
  return mavros_msgs::msg::builder::Init_HilStateQuaternion_header();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__HIL_STATE_QUATERNION__BUILDER_HPP_
