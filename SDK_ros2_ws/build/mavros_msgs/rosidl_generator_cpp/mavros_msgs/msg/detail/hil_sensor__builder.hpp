// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:msg/HilSensor.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__HIL_SENSOR__BUILDER_HPP_
#define MAVROS_MSGS__MSG__DETAIL__HIL_SENSOR__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/msg/detail/hil_sensor__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace msg
{

namespace builder
{

class Init_HilSensor_fields_updated
{
public:
  explicit Init_HilSensor_fields_updated(::mavros_msgs::msg::HilSensor & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::msg::HilSensor fields_updated(::mavros_msgs::msg::HilSensor::_fields_updated_type arg)
  {
    msg_.fields_updated = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::msg::HilSensor msg_;
};

class Init_HilSensor_temperature
{
public:
  explicit Init_HilSensor_temperature(::mavros_msgs::msg::HilSensor & msg)
  : msg_(msg)
  {}
  Init_HilSensor_fields_updated temperature(::mavros_msgs::msg::HilSensor::_temperature_type arg)
  {
    msg_.temperature = std::move(arg);
    return Init_HilSensor_fields_updated(msg_);
  }

private:
  ::mavros_msgs::msg::HilSensor msg_;
};

class Init_HilSensor_pressure_alt
{
public:
  explicit Init_HilSensor_pressure_alt(::mavros_msgs::msg::HilSensor & msg)
  : msg_(msg)
  {}
  Init_HilSensor_temperature pressure_alt(::mavros_msgs::msg::HilSensor::_pressure_alt_type arg)
  {
    msg_.pressure_alt = std::move(arg);
    return Init_HilSensor_temperature(msg_);
  }

private:
  ::mavros_msgs::msg::HilSensor msg_;
};

class Init_HilSensor_diff_pressure
{
public:
  explicit Init_HilSensor_diff_pressure(::mavros_msgs::msg::HilSensor & msg)
  : msg_(msg)
  {}
  Init_HilSensor_pressure_alt diff_pressure(::mavros_msgs::msg::HilSensor::_diff_pressure_type arg)
  {
    msg_.diff_pressure = std::move(arg);
    return Init_HilSensor_pressure_alt(msg_);
  }

private:
  ::mavros_msgs::msg::HilSensor msg_;
};

class Init_HilSensor_abs_pressure
{
public:
  explicit Init_HilSensor_abs_pressure(::mavros_msgs::msg::HilSensor & msg)
  : msg_(msg)
  {}
  Init_HilSensor_diff_pressure abs_pressure(::mavros_msgs::msg::HilSensor::_abs_pressure_type arg)
  {
    msg_.abs_pressure = std::move(arg);
    return Init_HilSensor_diff_pressure(msg_);
  }

private:
  ::mavros_msgs::msg::HilSensor msg_;
};

class Init_HilSensor_mag
{
public:
  explicit Init_HilSensor_mag(::mavros_msgs::msg::HilSensor & msg)
  : msg_(msg)
  {}
  Init_HilSensor_abs_pressure mag(::mavros_msgs::msg::HilSensor::_mag_type arg)
  {
    msg_.mag = std::move(arg);
    return Init_HilSensor_abs_pressure(msg_);
  }

private:
  ::mavros_msgs::msg::HilSensor msg_;
};

class Init_HilSensor_gyro
{
public:
  explicit Init_HilSensor_gyro(::mavros_msgs::msg::HilSensor & msg)
  : msg_(msg)
  {}
  Init_HilSensor_mag gyro(::mavros_msgs::msg::HilSensor::_gyro_type arg)
  {
    msg_.gyro = std::move(arg);
    return Init_HilSensor_mag(msg_);
  }

private:
  ::mavros_msgs::msg::HilSensor msg_;
};

class Init_HilSensor_acc
{
public:
  explicit Init_HilSensor_acc(::mavros_msgs::msg::HilSensor & msg)
  : msg_(msg)
  {}
  Init_HilSensor_gyro acc(::mavros_msgs::msg::HilSensor::_acc_type arg)
  {
    msg_.acc = std::move(arg);
    return Init_HilSensor_gyro(msg_);
  }

private:
  ::mavros_msgs::msg::HilSensor msg_;
};

class Init_HilSensor_header
{
public:
  Init_HilSensor_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_HilSensor_acc header(::mavros_msgs::msg::HilSensor::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_HilSensor_acc(msg_);
  }

private:
  ::mavros_msgs::msg::HilSensor msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::msg::HilSensor>()
{
  return mavros_msgs::msg::builder::Init_HilSensor_header();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__HIL_SENSOR__BUILDER_HPP_
