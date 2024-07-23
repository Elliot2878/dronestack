// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:msg/Thrust.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__THRUST__BUILDER_HPP_
#define MAVROS_MSGS__MSG__DETAIL__THRUST__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/msg/detail/thrust__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace msg
{

namespace builder
{

class Init_Thrust_thrust
{
public:
  explicit Init_Thrust_thrust(::mavros_msgs::msg::Thrust & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::msg::Thrust thrust(::mavros_msgs::msg::Thrust::_thrust_type arg)
  {
    msg_.thrust = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::msg::Thrust msg_;
};

class Init_Thrust_header
{
public:
  Init_Thrust_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Thrust_thrust header(::mavros_msgs::msg::Thrust::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Thrust_thrust(msg_);
  }

private:
  ::mavros_msgs::msg::Thrust msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::msg::Thrust>()
{
  return mavros_msgs::msg::builder::Init_Thrust_header();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__THRUST__BUILDER_HPP_
