// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mavros_msgs:msg/Waypoint.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__WAYPOINT__TRAITS_HPP_
#define MAVROS_MSGS__MSG__DETAIL__WAYPOINT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mavros_msgs/msg/detail/waypoint__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace mavros_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Waypoint & msg,
  std::ostream & out)
{
  out << "{";
  // member: frame
  {
    out << "frame: ";
    rosidl_generator_traits::value_to_yaml(msg.frame, out);
    out << ", ";
  }

  // member: command
  {
    out << "command: ";
    rosidl_generator_traits::value_to_yaml(msg.command, out);
    out << ", ";
  }

  // member: is_current
  {
    out << "is_current: ";
    rosidl_generator_traits::value_to_yaml(msg.is_current, out);
    out << ", ";
  }

  // member: autocontinue
  {
    out << "autocontinue: ";
    rosidl_generator_traits::value_to_yaml(msg.autocontinue, out);
    out << ", ";
  }

  // member: param1
  {
    out << "param1: ";
    rosidl_generator_traits::value_to_yaml(msg.param1, out);
    out << ", ";
  }

  // member: param2
  {
    out << "param2: ";
    rosidl_generator_traits::value_to_yaml(msg.param2, out);
    out << ", ";
  }

  // member: param3
  {
    out << "param3: ";
    rosidl_generator_traits::value_to_yaml(msg.param3, out);
    out << ", ";
  }

  // member: param4
  {
    out << "param4: ";
    rosidl_generator_traits::value_to_yaml(msg.param4, out);
    out << ", ";
  }

  // member: x_lat
  {
    out << "x_lat: ";
    rosidl_generator_traits::value_to_yaml(msg.x_lat, out);
    out << ", ";
  }

  // member: y_long
  {
    out << "y_long: ";
    rosidl_generator_traits::value_to_yaml(msg.y_long, out);
    out << ", ";
  }

  // member: z_alt
  {
    out << "z_alt: ";
    rosidl_generator_traits::value_to_yaml(msg.z_alt, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Waypoint & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: frame
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "frame: ";
    rosidl_generator_traits::value_to_yaml(msg.frame, out);
    out << "\n";
  }

  // member: command
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "command: ";
    rosidl_generator_traits::value_to_yaml(msg.command, out);
    out << "\n";
  }

  // member: is_current
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_current: ";
    rosidl_generator_traits::value_to_yaml(msg.is_current, out);
    out << "\n";
  }

  // member: autocontinue
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "autocontinue: ";
    rosidl_generator_traits::value_to_yaml(msg.autocontinue, out);
    out << "\n";
  }

  // member: param1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "param1: ";
    rosidl_generator_traits::value_to_yaml(msg.param1, out);
    out << "\n";
  }

  // member: param2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "param2: ";
    rosidl_generator_traits::value_to_yaml(msg.param2, out);
    out << "\n";
  }

  // member: param3
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "param3: ";
    rosidl_generator_traits::value_to_yaml(msg.param3, out);
    out << "\n";
  }

  // member: param4
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "param4: ";
    rosidl_generator_traits::value_to_yaml(msg.param4, out);
    out << "\n";
  }

  // member: x_lat
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x_lat: ";
    rosidl_generator_traits::value_to_yaml(msg.x_lat, out);
    out << "\n";
  }

  // member: y_long
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y_long: ";
    rosidl_generator_traits::value_to_yaml(msg.y_long, out);
    out << "\n";
  }

  // member: z_alt
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "z_alt: ";
    rosidl_generator_traits::value_to_yaml(msg.z_alt, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Waypoint & msg, bool use_flow_style = false)
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
  const mavros_msgs::msg::Waypoint & msg,
  std::ostream & out, size_t indentation = 0)
{
  mavros_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mavros_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const mavros_msgs::msg::Waypoint & msg)
{
  return mavros_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mavros_msgs::msg::Waypoint>()
{
  return "mavros_msgs::msg::Waypoint";
}

template<>
inline const char * name<mavros_msgs::msg::Waypoint>()
{
  return "mavros_msgs/msg/Waypoint";
}

template<>
struct has_fixed_size<mavros_msgs::msg::Waypoint>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<mavros_msgs::msg::Waypoint>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<mavros_msgs::msg::Waypoint>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MAVROS_MSGS__MSG__DETAIL__WAYPOINT__TRAITS_HPP_
