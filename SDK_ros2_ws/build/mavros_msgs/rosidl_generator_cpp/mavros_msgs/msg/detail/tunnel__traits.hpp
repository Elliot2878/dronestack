// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mavros_msgs:msg/Tunnel.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__TUNNEL__TRAITS_HPP_
#define MAVROS_MSGS__MSG__DETAIL__TUNNEL__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mavros_msgs/msg/detail/tunnel__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace mavros_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Tunnel & msg,
  std::ostream & out)
{
  out << "{";
  // member: target_system
  {
    out << "target_system: ";
    rosidl_generator_traits::value_to_yaml(msg.target_system, out);
    out << ", ";
  }

  // member: target_component
  {
    out << "target_component: ";
    rosidl_generator_traits::value_to_yaml(msg.target_component, out);
    out << ", ";
  }

  // member: payload_type
  {
    out << "payload_type: ";
    rosidl_generator_traits::value_to_yaml(msg.payload_type, out);
    out << ", ";
  }

  // member: payload_length
  {
    out << "payload_length: ";
    rosidl_generator_traits::value_to_yaml(msg.payload_length, out);
    out << ", ";
  }

  // member: payload
  {
    if (msg.payload.size() == 0) {
      out << "payload: []";
    } else {
      out << "payload: [";
      size_t pending_items = msg.payload.size();
      for (auto item : msg.payload) {
        rosidl_generator_traits::value_to_yaml(item, out);
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
  const Tunnel & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: target_system
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target_system: ";
    rosidl_generator_traits::value_to_yaml(msg.target_system, out);
    out << "\n";
  }

  // member: target_component
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target_component: ";
    rosidl_generator_traits::value_to_yaml(msg.target_component, out);
    out << "\n";
  }

  // member: payload_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "payload_type: ";
    rosidl_generator_traits::value_to_yaml(msg.payload_type, out);
    out << "\n";
  }

  // member: payload_length
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "payload_length: ";
    rosidl_generator_traits::value_to_yaml(msg.payload_length, out);
    out << "\n";
  }

  // member: payload
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.payload.size() == 0) {
      out << "payload: []\n";
    } else {
      out << "payload:\n";
      for (auto item : msg.payload) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Tunnel & msg, bool use_flow_style = false)
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
  const mavros_msgs::msg::Tunnel & msg,
  std::ostream & out, size_t indentation = 0)
{
  mavros_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mavros_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const mavros_msgs::msg::Tunnel & msg)
{
  return mavros_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mavros_msgs::msg::Tunnel>()
{
  return "mavros_msgs::msg::Tunnel";
}

template<>
inline const char * name<mavros_msgs::msg::Tunnel>()
{
  return "mavros_msgs/msg/Tunnel";
}

template<>
struct has_fixed_size<mavros_msgs::msg::Tunnel>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<mavros_msgs::msg::Tunnel>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<mavros_msgs::msg::Tunnel>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MAVROS_MSGS__MSG__DETAIL__TUNNEL__TRAITS_HPP_
