// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mavros_msgs:msg/HilSensor.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__HIL_SENSOR__TRAITS_HPP_
#define MAVROS_MSGS__MSG__DETAIL__HIL_SENSOR__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mavros_msgs/msg/detail/hil_sensor__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'acc'
// Member 'gyro'
// Member 'mag'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"

namespace mavros_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const HilSensor & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: acc
  {
    out << "acc: ";
    to_flow_style_yaml(msg.acc, out);
    out << ", ";
  }

  // member: gyro
  {
    out << "gyro: ";
    to_flow_style_yaml(msg.gyro, out);
    out << ", ";
  }

  // member: mag
  {
    out << "mag: ";
    to_flow_style_yaml(msg.mag, out);
    out << ", ";
  }

  // member: abs_pressure
  {
    out << "abs_pressure: ";
    rosidl_generator_traits::value_to_yaml(msg.abs_pressure, out);
    out << ", ";
  }

  // member: diff_pressure
  {
    out << "diff_pressure: ";
    rosidl_generator_traits::value_to_yaml(msg.diff_pressure, out);
    out << ", ";
  }

  // member: pressure_alt
  {
    out << "pressure_alt: ";
    rosidl_generator_traits::value_to_yaml(msg.pressure_alt, out);
    out << ", ";
  }

  // member: temperature
  {
    out << "temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.temperature, out);
    out << ", ";
  }

  // member: fields_updated
  {
    out << "fields_updated: ";
    rosidl_generator_traits::value_to_yaml(msg.fields_updated, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const HilSensor & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: acc
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "acc:\n";
    to_block_style_yaml(msg.acc, out, indentation + 2);
  }

  // member: gyro
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gyro:\n";
    to_block_style_yaml(msg.gyro, out, indentation + 2);
  }

  // member: mag
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mag:\n";
    to_block_style_yaml(msg.mag, out, indentation + 2);
  }

  // member: abs_pressure
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "abs_pressure: ";
    rosidl_generator_traits::value_to_yaml(msg.abs_pressure, out);
    out << "\n";
  }

  // member: diff_pressure
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "diff_pressure: ";
    rosidl_generator_traits::value_to_yaml(msg.diff_pressure, out);
    out << "\n";
  }

  // member: pressure_alt
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pressure_alt: ";
    rosidl_generator_traits::value_to_yaml(msg.pressure_alt, out);
    out << "\n";
  }

  // member: temperature
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.temperature, out);
    out << "\n";
  }

  // member: fields_updated
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fields_updated: ";
    rosidl_generator_traits::value_to_yaml(msg.fields_updated, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const HilSensor & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace mavros_msgs

namespace rosidl_generator_traits
{

[[deprecated("use mavros_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const mavros_msgs::msg::HilSensor & msg,
  std::ostream & out, size_t indentation = 0)
{
  mavros_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mavros_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const mavros_msgs::msg::HilSensor & msg)
{
  return mavros_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mavros_msgs::msg::HilSensor>()
{
  return "mavros_msgs::msg::HilSensor";
}

template<>
inline const char * name<mavros_msgs::msg::HilSensor>()
{
  return "mavros_msgs/msg/HilSensor";
}

template<>
struct has_fixed_size<mavros_msgs::msg::HilSensor>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Vector3>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<mavros_msgs::msg::HilSensor>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Vector3>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<mavros_msgs::msg::HilSensor>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MAVROS_MSGS__MSG__DETAIL__HIL_SENSOR__TRAITS_HPP_
