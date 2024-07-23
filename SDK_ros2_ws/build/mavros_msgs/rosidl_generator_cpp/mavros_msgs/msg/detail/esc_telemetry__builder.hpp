// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:msg/ESCTelemetry.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__ESC_TELEMETRY__BUILDER_HPP_
#define MAVROS_MSGS__MSG__DETAIL__ESC_TELEMETRY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/msg/detail/esc_telemetry__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace msg
{

namespace builder
{

class Init_ESCTelemetry_esc_telemetry
{
public:
  explicit Init_ESCTelemetry_esc_telemetry(::mavros_msgs::msg::ESCTelemetry & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::msg::ESCTelemetry esc_telemetry(::mavros_msgs::msg::ESCTelemetry::_esc_telemetry_type arg)
  {
    msg_.esc_telemetry = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::msg::ESCTelemetry msg_;
};

class Init_ESCTelemetry_header
{
public:
  Init_ESCTelemetry_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ESCTelemetry_esc_telemetry header(::mavros_msgs::msg::ESCTelemetry::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ESCTelemetry_esc_telemetry(msg_);
  }

private:
  ::mavros_msgs::msg::ESCTelemetry msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::msg::ESCTelemetry>()
{
  return mavros_msgs::msg::builder::Init_ESCTelemetry_header();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__ESC_TELEMETRY__BUILDER_HPP_
