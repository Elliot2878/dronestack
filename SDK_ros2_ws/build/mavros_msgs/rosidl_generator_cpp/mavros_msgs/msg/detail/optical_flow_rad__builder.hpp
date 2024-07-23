// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:msg/OpticalFlowRad.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__OPTICAL_FLOW_RAD__BUILDER_HPP_
#define MAVROS_MSGS__MSG__DETAIL__OPTICAL_FLOW_RAD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/msg/detail/optical_flow_rad__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace msg
{

namespace builder
{

class Init_OpticalFlowRad_distance
{
public:
  explicit Init_OpticalFlowRad_distance(::mavros_msgs::msg::OpticalFlowRad & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::msg::OpticalFlowRad distance(::mavros_msgs::msg::OpticalFlowRad::_distance_type arg)
  {
    msg_.distance = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::msg::OpticalFlowRad msg_;
};

class Init_OpticalFlowRad_time_delta_distance_us
{
public:
  explicit Init_OpticalFlowRad_time_delta_distance_us(::mavros_msgs::msg::OpticalFlowRad & msg)
  : msg_(msg)
  {}
  Init_OpticalFlowRad_distance time_delta_distance_us(::mavros_msgs::msg::OpticalFlowRad::_time_delta_distance_us_type arg)
  {
    msg_.time_delta_distance_us = std::move(arg);
    return Init_OpticalFlowRad_distance(msg_);
  }

private:
  ::mavros_msgs::msg::OpticalFlowRad msg_;
};

class Init_OpticalFlowRad_quality
{
public:
  explicit Init_OpticalFlowRad_quality(::mavros_msgs::msg::OpticalFlowRad & msg)
  : msg_(msg)
  {}
  Init_OpticalFlowRad_time_delta_distance_us quality(::mavros_msgs::msg::OpticalFlowRad::_quality_type arg)
  {
    msg_.quality = std::move(arg);
    return Init_OpticalFlowRad_time_delta_distance_us(msg_);
  }

private:
  ::mavros_msgs::msg::OpticalFlowRad msg_;
};

class Init_OpticalFlowRad_temperature
{
public:
  explicit Init_OpticalFlowRad_temperature(::mavros_msgs::msg::OpticalFlowRad & msg)
  : msg_(msg)
  {}
  Init_OpticalFlowRad_quality temperature(::mavros_msgs::msg::OpticalFlowRad::_temperature_type arg)
  {
    msg_.temperature = std::move(arg);
    return Init_OpticalFlowRad_quality(msg_);
  }

private:
  ::mavros_msgs::msg::OpticalFlowRad msg_;
};

class Init_OpticalFlowRad_integrated_zgyro
{
public:
  explicit Init_OpticalFlowRad_integrated_zgyro(::mavros_msgs::msg::OpticalFlowRad & msg)
  : msg_(msg)
  {}
  Init_OpticalFlowRad_temperature integrated_zgyro(::mavros_msgs::msg::OpticalFlowRad::_integrated_zgyro_type arg)
  {
    msg_.integrated_zgyro = std::move(arg);
    return Init_OpticalFlowRad_temperature(msg_);
  }

private:
  ::mavros_msgs::msg::OpticalFlowRad msg_;
};

class Init_OpticalFlowRad_integrated_ygyro
{
public:
  explicit Init_OpticalFlowRad_integrated_ygyro(::mavros_msgs::msg::OpticalFlowRad & msg)
  : msg_(msg)
  {}
  Init_OpticalFlowRad_integrated_zgyro integrated_ygyro(::mavros_msgs::msg::OpticalFlowRad::_integrated_ygyro_type arg)
  {
    msg_.integrated_ygyro = std::move(arg);
    return Init_OpticalFlowRad_integrated_zgyro(msg_);
  }

private:
  ::mavros_msgs::msg::OpticalFlowRad msg_;
};

class Init_OpticalFlowRad_integrated_xgyro
{
public:
  explicit Init_OpticalFlowRad_integrated_xgyro(::mavros_msgs::msg::OpticalFlowRad & msg)
  : msg_(msg)
  {}
  Init_OpticalFlowRad_integrated_ygyro integrated_xgyro(::mavros_msgs::msg::OpticalFlowRad::_integrated_xgyro_type arg)
  {
    msg_.integrated_xgyro = std::move(arg);
    return Init_OpticalFlowRad_integrated_ygyro(msg_);
  }

private:
  ::mavros_msgs::msg::OpticalFlowRad msg_;
};

class Init_OpticalFlowRad_integrated_y
{
public:
  explicit Init_OpticalFlowRad_integrated_y(::mavros_msgs::msg::OpticalFlowRad & msg)
  : msg_(msg)
  {}
  Init_OpticalFlowRad_integrated_xgyro integrated_y(::mavros_msgs::msg::OpticalFlowRad::_integrated_y_type arg)
  {
    msg_.integrated_y = std::move(arg);
    return Init_OpticalFlowRad_integrated_xgyro(msg_);
  }

private:
  ::mavros_msgs::msg::OpticalFlowRad msg_;
};

class Init_OpticalFlowRad_integrated_x
{
public:
  explicit Init_OpticalFlowRad_integrated_x(::mavros_msgs::msg::OpticalFlowRad & msg)
  : msg_(msg)
  {}
  Init_OpticalFlowRad_integrated_y integrated_x(::mavros_msgs::msg::OpticalFlowRad::_integrated_x_type arg)
  {
    msg_.integrated_x = std::move(arg);
    return Init_OpticalFlowRad_integrated_y(msg_);
  }

private:
  ::mavros_msgs::msg::OpticalFlowRad msg_;
};

class Init_OpticalFlowRad_integration_time_us
{
public:
  explicit Init_OpticalFlowRad_integration_time_us(::mavros_msgs::msg::OpticalFlowRad & msg)
  : msg_(msg)
  {}
  Init_OpticalFlowRad_integrated_x integration_time_us(::mavros_msgs::msg::OpticalFlowRad::_integration_time_us_type arg)
  {
    msg_.integration_time_us = std::move(arg);
    return Init_OpticalFlowRad_integrated_x(msg_);
  }

private:
  ::mavros_msgs::msg::OpticalFlowRad msg_;
};

class Init_OpticalFlowRad_header
{
public:
  Init_OpticalFlowRad_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_OpticalFlowRad_integration_time_us header(::mavros_msgs::msg::OpticalFlowRad::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_OpticalFlowRad_integration_time_us(msg_);
  }

private:
  ::mavros_msgs::msg::OpticalFlowRad msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::msg::OpticalFlowRad>()
{
  return mavros_msgs::msg::builder::Init_OpticalFlowRad_header();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__OPTICAL_FLOW_RAD__BUILDER_HPP_
