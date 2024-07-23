// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mavros_msgs:msg/GimbalDeviceInformation.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__GIMBAL_DEVICE_INFORMATION__TRAITS_HPP_
#define MAVROS_MSGS__MSG__DETAIL__GIMBAL_DEVICE_INFORMATION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mavros_msgs/msg/detail/gimbal_device_information__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace mavros_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const GimbalDeviceInformation & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: vendor_name
  {
    out << "vendor_name: ";
    rosidl_generator_traits::value_to_yaml(msg.vendor_name, out);
    out << ", ";
  }

  // member: model_name
  {
    out << "model_name: ";
    rosidl_generator_traits::value_to_yaml(msg.model_name, out);
    out << ", ";
  }

  // member: custom_name
  {
    out << "custom_name: ";
    rosidl_generator_traits::value_to_yaml(msg.custom_name, out);
    out << ", ";
  }

  // member: firmware_version
  {
    out << "firmware_version: ";
    rosidl_generator_traits::value_to_yaml(msg.firmware_version, out);
    out << ", ";
  }

  // member: hardware_version
  {
    out << "hardware_version: ";
    rosidl_generator_traits::value_to_yaml(msg.hardware_version, out);
    out << ", ";
  }

  // member: uid
  {
    out << "uid: ";
    rosidl_generator_traits::value_to_yaml(msg.uid, out);
    out << ", ";
  }

  // member: cap_flags
  {
    out << "cap_flags: ";
    rosidl_generator_traits::value_to_yaml(msg.cap_flags, out);
    out << ", ";
  }

  // member: custom_cap_flags
  {
    out << "custom_cap_flags: ";
    rosidl_generator_traits::value_to_yaml(msg.custom_cap_flags, out);
    out << ", ";
  }

  // member: roll_min
  {
    out << "roll_min: ";
    rosidl_generator_traits::value_to_yaml(msg.roll_min, out);
    out << ", ";
  }

  // member: roll_max
  {
    out << "roll_max: ";
    rosidl_generator_traits::value_to_yaml(msg.roll_max, out);
    out << ", ";
  }

  // member: pitch_min
  {
    out << "pitch_min: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch_min, out);
    out << ", ";
  }

  // member: pitch_max
  {
    out << "pitch_max: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch_max, out);
    out << ", ";
  }

  // member: yaw_min
  {
    out << "yaw_min: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw_min, out);
    out << ", ";
  }

  // member: yaw_max
  {
    out << "yaw_max: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw_max, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GimbalDeviceInformation & msg,
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

  // member: vendor_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vendor_name: ";
    rosidl_generator_traits::value_to_yaml(msg.vendor_name, out);
    out << "\n";
  }

  // member: model_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "model_name: ";
    rosidl_generator_traits::value_to_yaml(msg.model_name, out);
    out << "\n";
  }

  // member: custom_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "custom_name: ";
    rosidl_generator_traits::value_to_yaml(msg.custom_name, out);
    out << "\n";
  }

  // member: firmware_version
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "firmware_version: ";
    rosidl_generator_traits::value_to_yaml(msg.firmware_version, out);
    out << "\n";
  }

  // member: hardware_version
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "hardware_version: ";
    rosidl_generator_traits::value_to_yaml(msg.hardware_version, out);
    out << "\n";
  }

  // member: uid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "uid: ";
    rosidl_generator_traits::value_to_yaml(msg.uid, out);
    out << "\n";
  }

  // member: cap_flags
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cap_flags: ";
    rosidl_generator_traits::value_to_yaml(msg.cap_flags, out);
    out << "\n";
  }

  // member: custom_cap_flags
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "custom_cap_flags: ";
    rosidl_generator_traits::value_to_yaml(msg.custom_cap_flags, out);
    out << "\n";
  }

  // member: roll_min
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "roll_min: ";
    rosidl_generator_traits::value_to_yaml(msg.roll_min, out);
    out << "\n";
  }

  // member: roll_max
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "roll_max: ";
    rosidl_generator_traits::value_to_yaml(msg.roll_max, out);
    out << "\n";
  }

  // member: pitch_min
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pitch_min: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch_min, out);
    out << "\n";
  }

  // member: pitch_max
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pitch_max: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch_max, out);
    out << "\n";
  }

  // member: yaw_min
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw_min: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw_min, out);
    out << "\n";
  }

  // member: yaw_max
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw_max: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw_max, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GimbalDeviceInformation & msg, bool use_flow_style = false)
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
  const mavros_msgs::msg::GimbalDeviceInformation & msg,
  std::ostream & out, size_t indentation = 0)
{
  mavros_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mavros_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const mavros_msgs::msg::GimbalDeviceInformation & msg)
{
  return mavros_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mavros_msgs::msg::GimbalDeviceInformation>()
{
  return "mavros_msgs::msg::GimbalDeviceInformation";
}

template<>
inline const char * name<mavros_msgs::msg::GimbalDeviceInformation>()
{
  return "mavros_msgs/msg/GimbalDeviceInformation";
}

template<>
struct has_fixed_size<mavros_msgs::msg::GimbalDeviceInformation>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<mavros_msgs::msg::GimbalDeviceInformation>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<mavros_msgs::msg::GimbalDeviceInformation>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MAVROS_MSGS__MSG__DETAIL__GIMBAL_DEVICE_INFORMATION__TRAITS_HPP_
