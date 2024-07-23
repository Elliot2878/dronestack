// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mavros_msgs:msg/ParamValue.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__PARAM_VALUE__TRAITS_HPP_
#define MAVROS_MSGS__MSG__DETAIL__PARAM_VALUE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mavros_msgs/msg/detail/param_value__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace mavros_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const ParamValue & msg,
  std::ostream & out)
{
  out << "{";
  // member: integer
  {
    out << "integer: ";
    rosidl_generator_traits::value_to_yaml(msg.integer, out);
    out << ", ";
  }

  // member: real
  {
    out << "real: ";
    rosidl_generator_traits::value_to_yaml(msg.real, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ParamValue & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: integer
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "integer: ";
    rosidl_generator_traits::value_to_yaml(msg.integer, out);
    out << "\n";
  }

  // member: real
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "real: ";
    rosidl_generator_traits::value_to_yaml(msg.real, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ParamValue & msg, bool use_flow_style = false)
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
  const mavros_msgs::msg::ParamValue & msg,
  std::ostream & out, size_t indentation = 0)
{
  mavros_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mavros_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const mavros_msgs::msg::ParamValue & msg)
{
  return mavros_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mavros_msgs::msg::ParamValue>()
{
  return "mavros_msgs::msg::ParamValue";
}

template<>
inline const char * name<mavros_msgs::msg::ParamValue>()
{
  return "mavros_msgs/msg/ParamValue";
}

template<>
struct has_fixed_size<mavros_msgs::msg::ParamValue>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<mavros_msgs::msg::ParamValue>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<mavros_msgs::msg::ParamValue>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MAVROS_MSGS__MSG__DETAIL__PARAM_VALUE__TRAITS_HPP_
