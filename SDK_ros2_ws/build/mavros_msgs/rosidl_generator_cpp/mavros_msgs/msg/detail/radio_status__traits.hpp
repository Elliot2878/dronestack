// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mavros_msgs:msg/RadioStatus.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__RADIO_STATUS__TRAITS_HPP_
#define MAVROS_MSGS__MSG__DETAIL__RADIO_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mavros_msgs/msg/detail/radio_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace mavros_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const RadioStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: rssi
  {
    out << "rssi: ";
    rosidl_generator_traits::value_to_yaml(msg.rssi, out);
    out << ", ";
  }

  // member: remrssi
  {
    out << "remrssi: ";
    rosidl_generator_traits::value_to_yaml(msg.remrssi, out);
    out << ", ";
  }

  // member: txbuf
  {
    out << "txbuf: ";
    rosidl_generator_traits::value_to_yaml(msg.txbuf, out);
    out << ", ";
  }

  // member: noise
  {
    out << "noise: ";
    rosidl_generator_traits::value_to_yaml(msg.noise, out);
    out << ", ";
  }

  // member: remnoise
  {
    out << "remnoise: ";
    rosidl_generator_traits::value_to_yaml(msg.remnoise, out);
    out << ", ";
  }

  // member: rxerrors
  {
    out << "rxerrors: ";
    rosidl_generator_traits::value_to_yaml(msg.rxerrors, out);
    out << ", ";
  }

  // member: fixed
  {
    out << "fixed: ";
    rosidl_generator_traits::value_to_yaml(msg.fixed, out);
    out << ", ";
  }

  // member: rssi_dbm
  {
    out << "rssi_dbm: ";
    rosidl_generator_traits::value_to_yaml(msg.rssi_dbm, out);
    out << ", ";
  }

  // member: remrssi_dbm
  {
    out << "remrssi_dbm: ";
    rosidl_generator_traits::value_to_yaml(msg.remrssi_dbm, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RadioStatus & msg,
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

  // member: rssi
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rssi: ";
    rosidl_generator_traits::value_to_yaml(msg.rssi, out);
    out << "\n";
  }

  // member: remrssi
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "remrssi: ";
    rosidl_generator_traits::value_to_yaml(msg.remrssi, out);
    out << "\n";
  }

  // member: txbuf
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "txbuf: ";
    rosidl_generator_traits::value_to_yaml(msg.txbuf, out);
    out << "\n";
  }

  // member: noise
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "noise: ";
    rosidl_generator_traits::value_to_yaml(msg.noise, out);
    out << "\n";
  }

  // member: remnoise
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "remnoise: ";
    rosidl_generator_traits::value_to_yaml(msg.remnoise, out);
    out << "\n";
  }

  // member: rxerrors
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rxerrors: ";
    rosidl_generator_traits::value_to_yaml(msg.rxerrors, out);
    out << "\n";
  }

  // member: fixed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fixed: ";
    rosidl_generator_traits::value_to_yaml(msg.fixed, out);
    out << "\n";
  }

  // member: rssi_dbm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rssi_dbm: ";
    rosidl_generator_traits::value_to_yaml(msg.rssi_dbm, out);
    out << "\n";
  }

  // member: remrssi_dbm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "remrssi_dbm: ";
    rosidl_generator_traits::value_to_yaml(msg.remrssi_dbm, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RadioStatus & msg, bool use_flow_style = false)
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
  const mavros_msgs::msg::RadioStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  mavros_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mavros_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const mavros_msgs::msg::RadioStatus & msg)
{
  return mavros_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mavros_msgs::msg::RadioStatus>()
{
  return "mavros_msgs::msg::RadioStatus";
}

template<>
inline const char * name<mavros_msgs::msg::RadioStatus>()
{
  return "mavros_msgs/msg/RadioStatus";
}

template<>
struct has_fixed_size<mavros_msgs::msg::RadioStatus>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<mavros_msgs::msg::RadioStatus>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<mavros_msgs::msg::RadioStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MAVROS_MSGS__MSG__DETAIL__RADIO_STATUS__TRAITS_HPP_
