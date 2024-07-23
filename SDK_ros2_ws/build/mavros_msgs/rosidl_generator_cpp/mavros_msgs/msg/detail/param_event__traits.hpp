// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mavros_msgs:msg/ParamEvent.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__PARAM_EVENT__TRAITS_HPP_
#define MAVROS_MSGS__MSG__DETAIL__PARAM_EVENT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mavros_msgs/msg/detail/param_event__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'value'
#include "rcl_interfaces/msg/detail/parameter_value__traits.hpp"

namespace mavros_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const ParamEvent & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: param_id
  {
    out << "param_id: ";
    rosidl_generator_traits::value_to_yaml(msg.param_id, out);
    out << ", ";
  }

  // member: value
  {
    out << "value: ";
    to_flow_style_yaml(msg.value, out);
    out << ", ";
  }

  // member: param_index
  {
    out << "param_index: ";
    rosidl_generator_traits::value_to_yaml(msg.param_index, out);
    out << ", ";
  }

  // member: param_count
  {
    out << "param_count: ";
    rosidl_generator_traits::value_to_yaml(msg.param_count, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ParamEvent & msg,
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

  // member: param_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "param_id: ";
    rosidl_generator_traits::value_to_yaml(msg.param_id, out);
    out << "\n";
  }

  // member: value
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "value:\n";
    to_block_style_yaml(msg.value, out, indentation + 2);
  }

  // member: param_index
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "param_index: ";
    rosidl_generator_traits::value_to_yaml(msg.param_index, out);
    out << "\n";
  }

  // member: param_count
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "param_count: ";
    rosidl_generator_traits::value_to_yaml(msg.param_count, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ParamEvent & msg, bool use_flow_style = false)
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
  const mavros_msgs::msg::ParamEvent & msg,
  std::ostream & out, size_t indentation = 0)
{
  mavros_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mavros_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const mavros_msgs::msg::ParamEvent & msg)
{
  return mavros_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mavros_msgs::msg::ParamEvent>()
{
  return "mavros_msgs::msg::ParamEvent";
}

template<>
inline const char * name<mavros_msgs::msg::ParamEvent>()
{
  return "mavros_msgs/msg/ParamEvent";
}

template<>
struct has_fixed_size<mavros_msgs::msg::ParamEvent>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<mavros_msgs::msg::ParamEvent>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<mavros_msgs::msg::ParamEvent>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MAVROS_MSGS__MSG__DETAIL__PARAM_EVENT__TRAITS_HPP_
