// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mavros_msgs:msg/NavControllerOutput.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__NAV_CONTROLLER_OUTPUT__TRAITS_HPP_
#define MAVROS_MSGS__MSG__DETAIL__NAV_CONTROLLER_OUTPUT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mavros_msgs/msg/detail/nav_controller_output__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace mavros_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const NavControllerOutput & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: nav_roll
  {
    out << "nav_roll: ";
    rosidl_generator_traits::value_to_yaml(msg.nav_roll, out);
    out << ", ";
  }

  // member: nav_pitch
  {
    out << "nav_pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.nav_pitch, out);
    out << ", ";
  }

  // member: nav_bearing
  {
    out << "nav_bearing: ";
    rosidl_generator_traits::value_to_yaml(msg.nav_bearing, out);
    out << ", ";
  }

  // member: target_bearing
  {
    out << "target_bearing: ";
    rosidl_generator_traits::value_to_yaml(msg.target_bearing, out);
    out << ", ";
  }

  // member: wp_dist
  {
    out << "wp_dist: ";
    rosidl_generator_traits::value_to_yaml(msg.wp_dist, out);
    out << ", ";
  }

  // member: alt_error
  {
    out << "alt_error: ";
    rosidl_generator_traits::value_to_yaml(msg.alt_error, out);
    out << ", ";
  }

  // member: aspd_error
  {
    out << "aspd_error: ";
    rosidl_generator_traits::value_to_yaml(msg.aspd_error, out);
    out << ", ";
  }

  // member: xtrack_error
  {
    out << "xtrack_error: ";
    rosidl_generator_traits::value_to_yaml(msg.xtrack_error, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const NavControllerOutput & msg,
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

  // member: nav_roll
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "nav_roll: ";
    rosidl_generator_traits::value_to_yaml(msg.nav_roll, out);
    out << "\n";
  }

  // member: nav_pitch
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "nav_pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.nav_pitch, out);
    out << "\n";
  }

  // member: nav_bearing
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "nav_bearing: ";
    rosidl_generator_traits::value_to_yaml(msg.nav_bearing, out);
    out << "\n";
  }

  // member: target_bearing
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target_bearing: ";
    rosidl_generator_traits::value_to_yaml(msg.target_bearing, out);
    out << "\n";
  }

  // member: wp_dist
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "wp_dist: ";
    rosidl_generator_traits::value_to_yaml(msg.wp_dist, out);
    out << "\n";
  }

  // member: alt_error
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "alt_error: ";
    rosidl_generator_traits::value_to_yaml(msg.alt_error, out);
    out << "\n";
  }

  // member: aspd_error
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "aspd_error: ";
    rosidl_generator_traits::value_to_yaml(msg.aspd_error, out);
    out << "\n";
  }

  // member: xtrack_error
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "xtrack_error: ";
    rosidl_generator_traits::value_to_yaml(msg.xtrack_error, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const NavControllerOutput & msg, bool use_flow_style = false)
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
  const mavros_msgs::msg::NavControllerOutput & msg,
  std::ostream & out, size_t indentation = 0)
{
  mavros_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mavros_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const mavros_msgs::msg::NavControllerOutput & msg)
{
  return mavros_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mavros_msgs::msg::NavControllerOutput>()
{
  return "mavros_msgs::msg::NavControllerOutput";
}

template<>
inline const char * name<mavros_msgs::msg::NavControllerOutput>()
{
  return "mavros_msgs/msg/NavControllerOutput";
}

template<>
struct has_fixed_size<mavros_msgs::msg::NavControllerOutput>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<mavros_msgs::msg::NavControllerOutput>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<mavros_msgs::msg::NavControllerOutput>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MAVROS_MSGS__MSG__DETAIL__NAV_CONTROLLER_OUTPUT__TRAITS_HPP_
