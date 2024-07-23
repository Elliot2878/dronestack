// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:msg/ESCTelemetryItem.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__ESC_TELEMETRY_ITEM__BUILDER_HPP_
#define MAVROS_MSGS__MSG__DETAIL__ESC_TELEMETRY_ITEM__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/msg/detail/esc_telemetry_item__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace msg
{

namespace builder
{

class Init_ESCTelemetryItem_count
{
public:
  explicit Init_ESCTelemetryItem_count(::mavros_msgs::msg::ESCTelemetryItem & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::msg::ESCTelemetryItem count(::mavros_msgs::msg::ESCTelemetryItem::_count_type arg)
  {
    msg_.count = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::msg::ESCTelemetryItem msg_;
};

class Init_ESCTelemetryItem_rpm
{
public:
  explicit Init_ESCTelemetryItem_rpm(::mavros_msgs::msg::ESCTelemetryItem & msg)
  : msg_(msg)
  {}
  Init_ESCTelemetryItem_count rpm(::mavros_msgs::msg::ESCTelemetryItem::_rpm_type arg)
  {
    msg_.rpm = std::move(arg);
    return Init_ESCTelemetryItem_count(msg_);
  }

private:
  ::mavros_msgs::msg::ESCTelemetryItem msg_;
};

class Init_ESCTelemetryItem_totalcurrent
{
public:
  explicit Init_ESCTelemetryItem_totalcurrent(::mavros_msgs::msg::ESCTelemetryItem & msg)
  : msg_(msg)
  {}
  Init_ESCTelemetryItem_rpm totalcurrent(::mavros_msgs::msg::ESCTelemetryItem::_totalcurrent_type arg)
  {
    msg_.totalcurrent = std::move(arg);
    return Init_ESCTelemetryItem_rpm(msg_);
  }

private:
  ::mavros_msgs::msg::ESCTelemetryItem msg_;
};

class Init_ESCTelemetryItem_current
{
public:
  explicit Init_ESCTelemetryItem_current(::mavros_msgs::msg::ESCTelemetryItem & msg)
  : msg_(msg)
  {}
  Init_ESCTelemetryItem_totalcurrent current(::mavros_msgs::msg::ESCTelemetryItem::_current_type arg)
  {
    msg_.current = std::move(arg);
    return Init_ESCTelemetryItem_totalcurrent(msg_);
  }

private:
  ::mavros_msgs::msg::ESCTelemetryItem msg_;
};

class Init_ESCTelemetryItem_voltage
{
public:
  explicit Init_ESCTelemetryItem_voltage(::mavros_msgs::msg::ESCTelemetryItem & msg)
  : msg_(msg)
  {}
  Init_ESCTelemetryItem_current voltage(::mavros_msgs::msg::ESCTelemetryItem::_voltage_type arg)
  {
    msg_.voltage = std::move(arg);
    return Init_ESCTelemetryItem_current(msg_);
  }

private:
  ::mavros_msgs::msg::ESCTelemetryItem msg_;
};

class Init_ESCTelemetryItem_temperature
{
public:
  explicit Init_ESCTelemetryItem_temperature(::mavros_msgs::msg::ESCTelemetryItem & msg)
  : msg_(msg)
  {}
  Init_ESCTelemetryItem_voltage temperature(::mavros_msgs::msg::ESCTelemetryItem::_temperature_type arg)
  {
    msg_.temperature = std::move(arg);
    return Init_ESCTelemetryItem_voltage(msg_);
  }

private:
  ::mavros_msgs::msg::ESCTelemetryItem msg_;
};

class Init_ESCTelemetryItem_header
{
public:
  Init_ESCTelemetryItem_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ESCTelemetryItem_temperature header(::mavros_msgs::msg::ESCTelemetryItem::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ESCTelemetryItem_temperature(msg_);
  }

private:
  ::mavros_msgs::msg::ESCTelemetryItem msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::msg::ESCTelemetryItem>()
{
  return mavros_msgs::msg::builder::Init_ESCTelemetryItem_header();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__ESC_TELEMETRY_ITEM__BUILDER_HPP_
