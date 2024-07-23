// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mavros_msgs:msg/HilControls.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__HIL_CONTROLS__TRAITS_HPP_
#define MAVROS_MSGS__MSG__DETAIL__HIL_CONTROLS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mavros_msgs/msg/detail/hil_controls__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace mavros_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const HilControls & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: roll_ailerons
  {
    out << "roll_ailerons: ";
    rosidl_generator_traits::value_to_yaml(msg.roll_ailerons, out);
    out << ", ";
  }

  // member: pitch_elevator
  {
    out << "pitch_elevator: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch_elevator, out);
    out << ", ";
  }

  // member: yaw_rudder
  {
    out << "yaw_rudder: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw_rudder, out);
    out << ", ";
  }

  // member: throttle
  {
    out << "throttle: ";
    rosidl_generator_traits::value_to_yaml(msg.throttle, out);
    out << ", ";
  }

  // member: aux1
  {
    out << "aux1: ";
    rosidl_generator_traits::value_to_yaml(msg.aux1, out);
    out << ", ";
  }

  // member: aux2
  {
    out << "aux2: ";
    rosidl_generator_traits::value_to_yaml(msg.aux2, out);
    out << ", ";
  }

  // member: aux3
  {
    out << "aux3: ";
    rosidl_generator_traits::value_to_yaml(msg.aux3, out);
    out << ", ";
  }

  // member: aux4
  {
    out << "aux4: ";
    rosidl_generator_traits::value_to_yaml(msg.aux4, out);
    out << ", ";
  }

  // member: mode
  {
    out << "mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mode, out);
    out << ", ";
  }

  // member: nav_mode
  {
    out << "nav_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.nav_mode, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const HilControls & msg,
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

  // member: roll_ailerons
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "roll_ailerons: ";
    rosidl_generator_traits::value_to_yaml(msg.roll_ailerons, out);
    out << "\n";
  }

  // member: pitch_elevator
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pitch_elevator: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch_elevator, out);
    out << "\n";
  }

  // member: yaw_rudder
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw_rudder: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw_rudder, out);
    out << "\n";
  }

  // member: throttle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "throttle: ";
    rosidl_generator_traits::value_to_yaml(msg.throttle, out);
    out << "\n";
  }

  // member: aux1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "aux1: ";
    rosidl_generator_traits::value_to_yaml(msg.aux1, out);
    out << "\n";
  }

  // member: aux2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "aux2: ";
    rosidl_generator_traits::value_to_yaml(msg.aux2, out);
    out << "\n";
  }

  // member: aux3
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "aux3: ";
    rosidl_generator_traits::value_to_yaml(msg.aux3, out);
    out << "\n";
  }

  // member: aux4
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "aux4: ";
    rosidl_generator_traits::value_to_yaml(msg.aux4, out);
    out << "\n";
  }

  // member: mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mode, out);
    out << "\n";
  }

  // member: nav_mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "nav_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.nav_mode, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const HilControls & msg, bool use_flow_style = false)
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
  const mavros_msgs::msg::HilControls & msg,
  std::ostream & out, size_t indentation = 0)
{
  mavros_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mavros_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const mavros_msgs::msg::HilControls & msg)
{
  return mavros_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mavros_msgs::msg::HilControls>()
{
  return "mavros_msgs::msg::HilControls";
}

template<>
inline const char * name<mavros_msgs::msg::HilControls>()
{
  return "mavros_msgs/msg/HilControls";
}

template<>
struct has_fixed_size<mavros_msgs::msg::HilControls>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<mavros_msgs::msg::HilControls>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<mavros_msgs::msg::HilControls>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MAVROS_MSGS__MSG__DETAIL__HIL_CONTROLS__TRAITS_HPP_
