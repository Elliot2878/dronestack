// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mavros_msgs:msg/SysStatus.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__SYS_STATUS__TRAITS_HPP_
#define MAVROS_MSGS__MSG__DETAIL__SYS_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mavros_msgs/msg/detail/sys_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace mavros_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const SysStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: sensors_present
  {
    out << "sensors_present: ";
    rosidl_generator_traits::value_to_yaml(msg.sensors_present, out);
    out << ", ";
  }

  // member: sensors_enabled
  {
    out << "sensors_enabled: ";
    rosidl_generator_traits::value_to_yaml(msg.sensors_enabled, out);
    out << ", ";
  }

  // member: sensors_health
  {
    out << "sensors_health: ";
    rosidl_generator_traits::value_to_yaml(msg.sensors_health, out);
    out << ", ";
  }

  // member: load
  {
    out << "load: ";
    rosidl_generator_traits::value_to_yaml(msg.load, out);
    out << ", ";
  }

  // member: voltage_battery
  {
    out << "voltage_battery: ";
    rosidl_generator_traits::value_to_yaml(msg.voltage_battery, out);
    out << ", ";
  }

  // member: current_battery
  {
    out << "current_battery: ";
    rosidl_generator_traits::value_to_yaml(msg.current_battery, out);
    out << ", ";
  }

  // member: battery_remaining
  {
    out << "battery_remaining: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_remaining, out);
    out << ", ";
  }

  // member: drop_rate_comm
  {
    out << "drop_rate_comm: ";
    rosidl_generator_traits::value_to_yaml(msg.drop_rate_comm, out);
    out << ", ";
  }

  // member: errors_comm
  {
    out << "errors_comm: ";
    rosidl_generator_traits::value_to_yaml(msg.errors_comm, out);
    out << ", ";
  }

  // member: errors_count1
  {
    out << "errors_count1: ";
    rosidl_generator_traits::value_to_yaml(msg.errors_count1, out);
    out << ", ";
  }

  // member: errors_count2
  {
    out << "errors_count2: ";
    rosidl_generator_traits::value_to_yaml(msg.errors_count2, out);
    out << ", ";
  }

  // member: errors_count3
  {
    out << "errors_count3: ";
    rosidl_generator_traits::value_to_yaml(msg.errors_count3, out);
    out << ", ";
  }

  // member: errors_count4
  {
    out << "errors_count4: ";
    rosidl_generator_traits::value_to_yaml(msg.errors_count4, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SysStatus & msg,
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

  // member: sensors_present
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sensors_present: ";
    rosidl_generator_traits::value_to_yaml(msg.sensors_present, out);
    out << "\n";
  }

  // member: sensors_enabled
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sensors_enabled: ";
    rosidl_generator_traits::value_to_yaml(msg.sensors_enabled, out);
    out << "\n";
  }

  // member: sensors_health
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sensors_health: ";
    rosidl_generator_traits::value_to_yaml(msg.sensors_health, out);
    out << "\n";
  }

  // member: load
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "load: ";
    rosidl_generator_traits::value_to_yaml(msg.load, out);
    out << "\n";
  }

  // member: voltage_battery
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "voltage_battery: ";
    rosidl_generator_traits::value_to_yaml(msg.voltage_battery, out);
    out << "\n";
  }

  // member: current_battery
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_battery: ";
    rosidl_generator_traits::value_to_yaml(msg.current_battery, out);
    out << "\n";
  }

  // member: battery_remaining
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "battery_remaining: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_remaining, out);
    out << "\n";
  }

  // member: drop_rate_comm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "drop_rate_comm: ";
    rosidl_generator_traits::value_to_yaml(msg.drop_rate_comm, out);
    out << "\n";
  }

  // member: errors_comm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "errors_comm: ";
    rosidl_generator_traits::value_to_yaml(msg.errors_comm, out);
    out << "\n";
  }

  // member: errors_count1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "errors_count1: ";
    rosidl_generator_traits::value_to_yaml(msg.errors_count1, out);
    out << "\n";
  }

  // member: errors_count2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "errors_count2: ";
    rosidl_generator_traits::value_to_yaml(msg.errors_count2, out);
    out << "\n";
  }

  // member: errors_count3
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "errors_count3: ";
    rosidl_generator_traits::value_to_yaml(msg.errors_count3, out);
    out << "\n";
  }

  // member: errors_count4
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "errors_count4: ";
    rosidl_generator_traits::value_to_yaml(msg.errors_count4, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SysStatus & msg, bool use_flow_style = false)
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
  const mavros_msgs::msg::SysStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  mavros_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mavros_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const mavros_msgs::msg::SysStatus & msg)
{
  return mavros_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mavros_msgs::msg::SysStatus>()
{
  return "mavros_msgs::msg::SysStatus";
}

template<>
inline const char * name<mavros_msgs::msg::SysStatus>()
{
  return "mavros_msgs/msg/SysStatus";
}

template<>
struct has_fixed_size<mavros_msgs::msg::SysStatus>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<mavros_msgs::msg::SysStatus>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<mavros_msgs::msg::SysStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MAVROS_MSGS__MSG__DETAIL__SYS_STATUS__TRAITS_HPP_
