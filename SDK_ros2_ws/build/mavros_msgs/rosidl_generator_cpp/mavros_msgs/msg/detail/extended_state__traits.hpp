// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mavros_msgs:msg/ExtendedState.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__EXTENDED_STATE__TRAITS_HPP_
#define MAVROS_MSGS__MSG__DETAIL__EXTENDED_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mavros_msgs/msg/detail/extended_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace mavros_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const ExtendedState & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: vtol_state
  {
    out << "vtol_state: ";
    rosidl_generator_traits::value_to_yaml(msg.vtol_state, out);
    out << ", ";
  }

  // member: landed_state
  {
    out << "landed_state: ";
    rosidl_generator_traits::value_to_yaml(msg.landed_state, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ExtendedState & msg,
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

  // member: vtol_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vtol_state: ";
    rosidl_generator_traits::value_to_yaml(msg.vtol_state, out);
    out << "\n";
  }

  // member: landed_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "landed_state: ";
    rosidl_generator_traits::value_to_yaml(msg.landed_state, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ExtendedState & msg, bool use_flow_style = false)
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
  const mavros_msgs::msg::ExtendedState & msg,
  std::ostream & out, size_t indentation = 0)
{
  mavros_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mavros_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const mavros_msgs::msg::ExtendedState & msg)
{
  return mavros_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mavros_msgs::msg::ExtendedState>()
{
  return "mavros_msgs::msg::ExtendedState";
}

template<>
inline const char * name<mavros_msgs::msg::ExtendedState>()
{
  return "mavros_msgs/msg/ExtendedState";
}

template<>
struct has_fixed_size<mavros_msgs::msg::ExtendedState>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<mavros_msgs::msg::ExtendedState>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<mavros_msgs::msg::ExtendedState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MAVROS_MSGS__MSG__DETAIL__EXTENDED_STATE__TRAITS_HPP_
