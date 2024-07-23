// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mavros_msgs:msg/TimesyncStatus.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__TIMESYNC_STATUS__TRAITS_HPP_
#define MAVROS_MSGS__MSG__DETAIL__TIMESYNC_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mavros_msgs/msg/detail/timesync_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace mavros_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const TimesyncStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: remote_timestamp_ns
  {
    out << "remote_timestamp_ns: ";
    rosidl_generator_traits::value_to_yaml(msg.remote_timestamp_ns, out);
    out << ", ";
  }

  // member: observed_offset_ns
  {
    out << "observed_offset_ns: ";
    rosidl_generator_traits::value_to_yaml(msg.observed_offset_ns, out);
    out << ", ";
  }

  // member: estimated_offset_ns
  {
    out << "estimated_offset_ns: ";
    rosidl_generator_traits::value_to_yaml(msg.estimated_offset_ns, out);
    out << ", ";
  }

  // member: round_trip_time_ms
  {
    out << "round_trip_time_ms: ";
    rosidl_generator_traits::value_to_yaml(msg.round_trip_time_ms, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TimesyncStatus & msg,
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

  // member: remote_timestamp_ns
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "remote_timestamp_ns: ";
    rosidl_generator_traits::value_to_yaml(msg.remote_timestamp_ns, out);
    out << "\n";
  }

  // member: observed_offset_ns
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "observed_offset_ns: ";
    rosidl_generator_traits::value_to_yaml(msg.observed_offset_ns, out);
    out << "\n";
  }

  // member: estimated_offset_ns
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "estimated_offset_ns: ";
    rosidl_generator_traits::value_to_yaml(msg.estimated_offset_ns, out);
    out << "\n";
  }

  // member: round_trip_time_ms
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "round_trip_time_ms: ";
    rosidl_generator_traits::value_to_yaml(msg.round_trip_time_ms, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TimesyncStatus & msg, bool use_flow_style = false)
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
  const mavros_msgs::msg::TimesyncStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  mavros_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mavros_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const mavros_msgs::msg::TimesyncStatus & msg)
{
  return mavros_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mavros_msgs::msg::TimesyncStatus>()
{
  return "mavros_msgs::msg::TimesyncStatus";
}

template<>
inline const char * name<mavros_msgs::msg::TimesyncStatus>()
{
  return "mavros_msgs/msg/TimesyncStatus";
}

template<>
struct has_fixed_size<mavros_msgs::msg::TimesyncStatus>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<mavros_msgs::msg::TimesyncStatus>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<mavros_msgs::msg::TimesyncStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MAVROS_MSGS__MSG__DETAIL__TIMESYNC_STATUS__TRAITS_HPP_
