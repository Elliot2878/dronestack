// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mavros_msgs:msg/LogEntry.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__LOG_ENTRY__TRAITS_HPP_
#define MAVROS_MSGS__MSG__DETAIL__LOG_ENTRY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mavros_msgs/msg/detail/log_entry__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'time_utc'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace mavros_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const LogEntry & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: id
  {
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << ", ";
  }

  // member: num_logs
  {
    out << "num_logs: ";
    rosidl_generator_traits::value_to_yaml(msg.num_logs, out);
    out << ", ";
  }

  // member: last_log_num
  {
    out << "last_log_num: ";
    rosidl_generator_traits::value_to_yaml(msg.last_log_num, out);
    out << ", ";
  }

  // member: time_utc
  {
    out << "time_utc: ";
    to_flow_style_yaml(msg.time_utc, out);
    out << ", ";
  }

  // member: size
  {
    out << "size: ";
    rosidl_generator_traits::value_to_yaml(msg.size, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const LogEntry & msg,
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

  // member: id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << "\n";
  }

  // member: num_logs
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "num_logs: ";
    rosidl_generator_traits::value_to_yaml(msg.num_logs, out);
    out << "\n";
  }

  // member: last_log_num
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "last_log_num: ";
    rosidl_generator_traits::value_to_yaml(msg.last_log_num, out);
    out << "\n";
  }

  // member: time_utc
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "time_utc:\n";
    to_block_style_yaml(msg.time_utc, out, indentation + 2);
  }

  // member: size
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "size: ";
    rosidl_generator_traits::value_to_yaml(msg.size, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const LogEntry & msg, bool use_flow_style = false)
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
  const mavros_msgs::msg::LogEntry & msg,
  std::ostream & out, size_t indentation = 0)
{
  mavros_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mavros_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const mavros_msgs::msg::LogEntry & msg)
{
  return mavros_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mavros_msgs::msg::LogEntry>()
{
  return "mavros_msgs::msg::LogEntry";
}

template<>
inline const char * name<mavros_msgs::msg::LogEntry>()
{
  return "mavros_msgs/msg/LogEntry";
}

template<>
struct has_fixed_size<mavros_msgs::msg::LogEntry>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<mavros_msgs::msg::LogEntry>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<mavros_msgs::msg::LogEntry>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MAVROS_MSGS__MSG__DETAIL__LOG_ENTRY__TRAITS_HPP_
