// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:msg/Vibration.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__VIBRATION__BUILDER_HPP_
#define MAVROS_MSGS__MSG__DETAIL__VIBRATION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/msg/detail/vibration__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace msg
{

namespace builder
{

class Init_Vibration_clipping
{
public:
  explicit Init_Vibration_clipping(::mavros_msgs::msg::Vibration & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::msg::Vibration clipping(::mavros_msgs::msg::Vibration::_clipping_type arg)
  {
    msg_.clipping = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::msg::Vibration msg_;
};

class Init_Vibration_vibration
{
public:
  explicit Init_Vibration_vibration(::mavros_msgs::msg::Vibration & msg)
  : msg_(msg)
  {}
  Init_Vibration_clipping vibration(::mavros_msgs::msg::Vibration::_vibration_type arg)
  {
    msg_.vibration = std::move(arg);
    return Init_Vibration_clipping(msg_);
  }

private:
  ::mavros_msgs::msg::Vibration msg_;
};

class Init_Vibration_header
{
public:
  Init_Vibration_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Vibration_vibration header(::mavros_msgs::msg::Vibration::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Vibration_vibration(msg_);
  }

private:
  ::mavros_msgs::msg::Vibration msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::msg::Vibration>()
{
  return mavros_msgs::msg::builder::Init_Vibration_header();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__VIBRATION__BUILDER_HPP_
