// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:msg/MagnetometerReporter.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__MAGNETOMETER_REPORTER__BUILDER_HPP_
#define MAVROS_MSGS__MSG__DETAIL__MAGNETOMETER_REPORTER__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/msg/detail/magnetometer_reporter__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace msg
{

namespace builder
{

class Init_MagnetometerReporter_confidence
{
public:
  explicit Init_MagnetometerReporter_confidence(::mavros_msgs::msg::MagnetometerReporter & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::msg::MagnetometerReporter confidence(::mavros_msgs::msg::MagnetometerReporter::_confidence_type arg)
  {
    msg_.confidence = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::msg::MagnetometerReporter msg_;
};

class Init_MagnetometerReporter_report
{
public:
  explicit Init_MagnetometerReporter_report(::mavros_msgs::msg::MagnetometerReporter & msg)
  : msg_(msg)
  {}
  Init_MagnetometerReporter_confidence report(::mavros_msgs::msg::MagnetometerReporter::_report_type arg)
  {
    msg_.report = std::move(arg);
    return Init_MagnetometerReporter_confidence(msg_);
  }

private:
  ::mavros_msgs::msg::MagnetometerReporter msg_;
};

class Init_MagnetometerReporter_header
{
public:
  Init_MagnetometerReporter_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MagnetometerReporter_report header(::mavros_msgs::msg::MagnetometerReporter::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MagnetometerReporter_report(msg_);
  }

private:
  ::mavros_msgs::msg::MagnetometerReporter msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::msg::MagnetometerReporter>()
{
  return mavros_msgs::msg::builder::Init_MagnetometerReporter_header();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__MAGNETOMETER_REPORTER__BUILDER_HPP_
