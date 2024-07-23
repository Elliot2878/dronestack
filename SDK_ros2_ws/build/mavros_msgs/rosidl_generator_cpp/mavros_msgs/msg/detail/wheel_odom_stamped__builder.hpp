// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:msg/WheelOdomStamped.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__WHEEL_ODOM_STAMPED__BUILDER_HPP_
#define MAVROS_MSGS__MSG__DETAIL__WHEEL_ODOM_STAMPED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/msg/detail/wheel_odom_stamped__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace msg
{

namespace builder
{

class Init_WheelOdomStamped_data
{
public:
  explicit Init_WheelOdomStamped_data(::mavros_msgs::msg::WheelOdomStamped & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::msg::WheelOdomStamped data(::mavros_msgs::msg::WheelOdomStamped::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::msg::WheelOdomStamped msg_;
};

class Init_WheelOdomStamped_header
{
public:
  Init_WheelOdomStamped_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_WheelOdomStamped_data header(::mavros_msgs::msg::WheelOdomStamped::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_WheelOdomStamped_data(msg_);
  }

private:
  ::mavros_msgs::msg::WheelOdomStamped msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::msg::WheelOdomStamped>()
{
  return mavros_msgs::msg::builder::Init_WheelOdomStamped_header();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__WHEEL_ODOM_STAMPED__BUILDER_HPP_
