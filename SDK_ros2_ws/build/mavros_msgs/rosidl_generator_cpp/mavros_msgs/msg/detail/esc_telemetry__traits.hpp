// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mavros_msgs:msg/ESCTelemetry.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__ESC_TELEMETRY__TRAITS_HPP_
#define MAVROS_MSGS__MSG__DETAIL__ESC_TELEMETRY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mavros_msgs/msg/detail/esc_telemetry__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'esc_telemetry'
#include "mavros_msgs/msg/detail/esc_telemetry_item__traits.hpp"

namespace mavros_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const ESCTelemetry & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: esc_telemetry
  {
    if (msg.esc_telemetry.size() == 0) {
      out << "esc_telemetry: []";
    } else {
      out << "esc_telemetry: [";
      size_t pending_items = msg.esc_telemetry.size();
      for (auto item : msg.esc_telemetry) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ESCTelemetry & msg,
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

  // member: esc_telemetry
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.esc_telemetry.size() == 0) {
      out << "esc_telemetry: []\n";
    } else {
      out << "esc_telemetry:\n";
      for (auto item : msg.esc_telemetry) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ESCTelemetry & msg, bool use_flow_style = false)
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
  const mavros_msgs::msg::ESCTelemetry & msg,
  std::ostream & out, size_t indentation = 0)
{
  mavros_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mavros_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const mavros_msgs::msg::ESCTelemetry & msg)
{
  return mavros_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mavros_msgs::msg::ESCTelemetry>()
{
  return "mavros_msgs::msg::ESCTelemetry";
}

template<>
inline const char * name<mavros_msgs::msg::ESCTelemetry>()
{
  return "mavros_msgs/msg/ESCTelemetry";
}

template<>
struct has_fixed_size<mavros_msgs::msg::ESCTelemetry>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<mavros_msgs::msg::ESCTelemetry>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<mavros_msgs::msg::ESCTelemetry>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MAVROS_MSGS__MSG__DETAIL__ESC_TELEMETRY__TRAITS_HPP_
