// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mavros_msgs:msg/GimbalManagerStatus.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__GIMBAL_MANAGER_STATUS__TRAITS_HPP_
#define MAVROS_MSGS__MSG__DETAIL__GIMBAL_MANAGER_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mavros_msgs/msg/detail/gimbal_manager_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace mavros_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const GimbalManagerStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: flags
  {
    out << "flags: ";
    rosidl_generator_traits::value_to_yaml(msg.flags, out);
    out << ", ";
  }

  // member: gimbal_device_id
  {
    out << "gimbal_device_id: ";
    rosidl_generator_traits::value_to_yaml(msg.gimbal_device_id, out);
    out << ", ";
  }

  // member: sysid_primary
  {
    out << "sysid_primary: ";
    rosidl_generator_traits::value_to_yaml(msg.sysid_primary, out);
    out << ", ";
  }

  // member: compid_primary
  {
    out << "compid_primary: ";
    rosidl_generator_traits::value_to_yaml(msg.compid_primary, out);
    out << ", ";
  }

  // member: sysid_secondary
  {
    out << "sysid_secondary: ";
    rosidl_generator_traits::value_to_yaml(msg.sysid_secondary, out);
    out << ", ";
  }

  // member: compid_secondary
  {
    out << "compid_secondary: ";
    rosidl_generator_traits::value_to_yaml(msg.compid_secondary, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GimbalManagerStatus & msg,
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

  // member: flags
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "flags: ";
    rosidl_generator_traits::value_to_yaml(msg.flags, out);
    out << "\n";
  }

  // member: gimbal_device_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gimbal_device_id: ";
    rosidl_generator_traits::value_to_yaml(msg.gimbal_device_id, out);
    out << "\n";
  }

  // member: sysid_primary
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sysid_primary: ";
    rosidl_generator_traits::value_to_yaml(msg.sysid_primary, out);
    out << "\n";
  }

  // member: compid_primary
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "compid_primary: ";
    rosidl_generator_traits::value_to_yaml(msg.compid_primary, out);
    out << "\n";
  }

  // member: sysid_secondary
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sysid_secondary: ";
    rosidl_generator_traits::value_to_yaml(msg.sysid_secondary, out);
    out << "\n";
  }

  // member: compid_secondary
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "compid_secondary: ";
    rosidl_generator_traits::value_to_yaml(msg.compid_secondary, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GimbalManagerStatus & msg, bool use_flow_style = false)
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
  const mavros_msgs::msg::GimbalManagerStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  mavros_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mavros_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const mavros_msgs::msg::GimbalManagerStatus & msg)
{
  return mavros_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mavros_msgs::msg::GimbalManagerStatus>()
{
  return "mavros_msgs::msg::GimbalManagerStatus";
}

template<>
inline const char * name<mavros_msgs::msg::GimbalManagerStatus>()
{
  return "mavros_msgs/msg/GimbalManagerStatus";
}

template<>
struct has_fixed_size<mavros_msgs::msg::GimbalManagerStatus>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<mavros_msgs::msg::GimbalManagerStatus>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<mavros_msgs::msg::GimbalManagerStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MAVROS_MSGS__MSG__DETAIL__GIMBAL_MANAGER_STATUS__TRAITS_HPP_
