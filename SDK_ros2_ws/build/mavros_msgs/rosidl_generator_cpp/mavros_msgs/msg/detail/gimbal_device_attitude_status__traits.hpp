// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mavros_msgs:msg/GimbalDeviceAttitudeStatus.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__GIMBAL_DEVICE_ATTITUDE_STATUS__TRAITS_HPP_
#define MAVROS_MSGS__MSG__DETAIL__GIMBAL_DEVICE_ATTITUDE_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mavros_msgs/msg/detail/gimbal_device_attitude_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'q'
#include "geometry_msgs/msg/detail/quaternion__traits.hpp"

namespace mavros_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const GimbalDeviceAttitudeStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

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

  // member: flags
  {
    out << "flags: ";
    rosidl_generator_traits::value_to_yaml(msg.flags, out);
    out << ", ";
  }

  // member: q
  {
    out << "q: ";
    to_flow_style_yaml(msg.q, out);
    out << ", ";
  }

  // member: angular_velocity_x
  {
    out << "angular_velocity_x: ";
    rosidl_generator_traits::value_to_yaml(msg.angular_velocity_x, out);
    out << ", ";
  }

  // member: angular_velocity_y
  {
    out << "angular_velocity_y: ";
    rosidl_generator_traits::value_to_yaml(msg.angular_velocity_y, out);
    out << ", ";
  }

  // member: angular_velocity_z
  {
    out << "angular_velocity_z: ";
    rosidl_generator_traits::value_to_yaml(msg.angular_velocity_z, out);
    out << ", ";
  }

  // member: failure_flags
  {
    out << "failure_flags: ";
    rosidl_generator_traits::value_to_yaml(msg.failure_flags, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GimbalDeviceAttitudeStatus & msg,
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

  // member: flags
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "flags: ";
    rosidl_generator_traits::value_to_yaml(msg.flags, out);
    out << "\n";
  }

  // member: q
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "q:\n";
    to_block_style_yaml(msg.q, out, indentation + 2);
  }

  // member: angular_velocity_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "angular_velocity_x: ";
    rosidl_generator_traits::value_to_yaml(msg.angular_velocity_x, out);
    out << "\n";
  }

  // member: angular_velocity_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "angular_velocity_y: ";
    rosidl_generator_traits::value_to_yaml(msg.angular_velocity_y, out);
    out << "\n";
  }

  // member: angular_velocity_z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "angular_velocity_z: ";
    rosidl_generator_traits::value_to_yaml(msg.angular_velocity_z, out);
    out << "\n";
  }

  // member: failure_flags
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "failure_flags: ";
    rosidl_generator_traits::value_to_yaml(msg.failure_flags, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GimbalDeviceAttitudeStatus & msg, bool use_flow_style = false)
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
  const mavros_msgs::msg::GimbalDeviceAttitudeStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  mavros_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mavros_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const mavros_msgs::msg::GimbalDeviceAttitudeStatus & msg)
{
  return mavros_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mavros_msgs::msg::GimbalDeviceAttitudeStatus>()
{
  return "mavros_msgs::msg::GimbalDeviceAttitudeStatus";
}

template<>
inline const char * name<mavros_msgs::msg::GimbalDeviceAttitudeStatus>()
{
  return "mavros_msgs/msg/GimbalDeviceAttitudeStatus";
}

template<>
struct has_fixed_size<mavros_msgs::msg::GimbalDeviceAttitudeStatus>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Quaternion>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<mavros_msgs::msg::GimbalDeviceAttitudeStatus>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Quaternion>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<mavros_msgs::msg::GimbalDeviceAttitudeStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MAVROS_MSGS__MSG__DETAIL__GIMBAL_DEVICE_ATTITUDE_STATUS__TRAITS_HPP_
