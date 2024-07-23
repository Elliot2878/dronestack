// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:msg/RTCM.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__RTCM__BUILDER_HPP_
#define MAVROS_MSGS__MSG__DETAIL__RTCM__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/msg/detail/rtcm__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace msg
{

namespace builder
{

class Init_RTCM_data
{
public:
  explicit Init_RTCM_data(::mavros_msgs::msg::RTCM & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::msg::RTCM data(::mavros_msgs::msg::RTCM::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::msg::RTCM msg_;
};

class Init_RTCM_header
{
public:
  Init_RTCM_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RTCM_data header(::mavros_msgs::msg::RTCM::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_RTCM_data(msg_);
  }

private:
  ::mavros_msgs::msg::RTCM msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::msg::RTCM>()
{
  return mavros_msgs::msg::builder::Init_RTCM_header();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__RTCM__BUILDER_HPP_
