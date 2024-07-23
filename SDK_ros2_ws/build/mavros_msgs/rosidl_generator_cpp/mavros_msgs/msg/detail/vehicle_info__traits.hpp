// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mavros_msgs:msg/VehicleInfo.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__VEHICLE_INFO__TRAITS_HPP_
#define MAVROS_MSGS__MSG__DETAIL__VEHICLE_INFO__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mavros_msgs/msg/detail/vehicle_info__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace mavros_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const VehicleInfo & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: available_info
  {
    out << "available_info: ";
    rosidl_generator_traits::value_to_yaml(msg.available_info, out);
    out << ", ";
  }

  // member: sysid
  {
    out << "sysid: ";
    rosidl_generator_traits::value_to_yaml(msg.sysid, out);
    out << ", ";
  }

  // member: compid
  {
    out << "compid: ";
    rosidl_generator_traits::value_to_yaml(msg.compid, out);
    out << ", ";
  }

  // member: autopilot
  {
    out << "autopilot: ";
    rosidl_generator_traits::value_to_yaml(msg.autopilot, out);
    out << ", ";
  }

  // member: type
  {
    out << "type: ";
    rosidl_generator_traits::value_to_yaml(msg.type, out);
    out << ", ";
  }

  // member: system_status
  {
    out << "system_status: ";
    rosidl_generator_traits::value_to_yaml(msg.system_status, out);
    out << ", ";
  }

  // member: base_mode
  {
    out << "base_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.base_mode, out);
    out << ", ";
  }

  // member: custom_mode
  {
    out << "custom_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.custom_mode, out);
    out << ", ";
  }

  // member: mode
  {
    out << "mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mode, out);
    out << ", ";
  }

  // member: mode_id
  {
    out << "mode_id: ";
    rosidl_generator_traits::value_to_yaml(msg.mode_id, out);
    out << ", ";
  }

  // member: capabilities
  {
    out << "capabilities: ";
    rosidl_generator_traits::value_to_yaml(msg.capabilities, out);
    out << ", ";
  }

  // member: flight_sw_version
  {
    out << "flight_sw_version: ";
    rosidl_generator_traits::value_to_yaml(msg.flight_sw_version, out);
    out << ", ";
  }

  // member: middleware_sw_version
  {
    out << "middleware_sw_version: ";
    rosidl_generator_traits::value_to_yaml(msg.middleware_sw_version, out);
    out << ", ";
  }

  // member: os_sw_version
  {
    out << "os_sw_version: ";
    rosidl_generator_traits::value_to_yaml(msg.os_sw_version, out);
    out << ", ";
  }

  // member: board_version
  {
    out << "board_version: ";
    rosidl_generator_traits::value_to_yaml(msg.board_version, out);
    out << ", ";
  }

  // member: flight_custom_version
  {
    out << "flight_custom_version: ";
    rosidl_generator_traits::value_to_yaml(msg.flight_custom_version, out);
    out << ", ";
  }

  // member: vendor_id
  {
    out << "vendor_id: ";
    rosidl_generator_traits::value_to_yaml(msg.vendor_id, out);
    out << ", ";
  }

  // member: product_id
  {
    out << "product_id: ";
    rosidl_generator_traits::value_to_yaml(msg.product_id, out);
    out << ", ";
  }

  // member: uid
  {
    out << "uid: ";
    rosidl_generator_traits::value_to_yaml(msg.uid, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const VehicleInfo & msg,
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

  // member: available_info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "available_info: ";
    rosidl_generator_traits::value_to_yaml(msg.available_info, out);
    out << "\n";
  }

  // member: sysid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sysid: ";
    rosidl_generator_traits::value_to_yaml(msg.sysid, out);
    out << "\n";
  }

  // member: compid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "compid: ";
    rosidl_generator_traits::value_to_yaml(msg.compid, out);
    out << "\n";
  }

  // member: autopilot
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "autopilot: ";
    rosidl_generator_traits::value_to_yaml(msg.autopilot, out);
    out << "\n";
  }

  // member: type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "type: ";
    rosidl_generator_traits::value_to_yaml(msg.type, out);
    out << "\n";
  }

  // member: system_status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "system_status: ";
    rosidl_generator_traits::value_to_yaml(msg.system_status, out);
    out << "\n";
  }

  // member: base_mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "base_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.base_mode, out);
    out << "\n";
  }

  // member: custom_mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "custom_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.custom_mode, out);
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

  // member: mode_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mode_id: ";
    rosidl_generator_traits::value_to_yaml(msg.mode_id, out);
    out << "\n";
  }

  // member: capabilities
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "capabilities: ";
    rosidl_generator_traits::value_to_yaml(msg.capabilities, out);
    out << "\n";
  }

  // member: flight_sw_version
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "flight_sw_version: ";
    rosidl_generator_traits::value_to_yaml(msg.flight_sw_version, out);
    out << "\n";
  }

  // member: middleware_sw_version
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "middleware_sw_version: ";
    rosidl_generator_traits::value_to_yaml(msg.middleware_sw_version, out);
    out << "\n";
  }

  // member: os_sw_version
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "os_sw_version: ";
    rosidl_generator_traits::value_to_yaml(msg.os_sw_version, out);
    out << "\n";
  }

  // member: board_version
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "board_version: ";
    rosidl_generator_traits::value_to_yaml(msg.board_version, out);
    out << "\n";
  }

  // member: flight_custom_version
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "flight_custom_version: ";
    rosidl_generator_traits::value_to_yaml(msg.flight_custom_version, out);
    out << "\n";
  }

  // member: vendor_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vendor_id: ";
    rosidl_generator_traits::value_to_yaml(msg.vendor_id, out);
    out << "\n";
  }

  // member: product_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "product_id: ";
    rosidl_generator_traits::value_to_yaml(msg.product_id, out);
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
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const VehicleInfo & msg, bool use_flow_style = false)
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
  const mavros_msgs::msg::VehicleInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  mavros_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mavros_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const mavros_msgs::msg::VehicleInfo & msg)
{
  return mavros_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mavros_msgs::msg::VehicleInfo>()
{
  return "mavros_msgs::msg::VehicleInfo";
}

template<>
inline const char * name<mavros_msgs::msg::VehicleInfo>()
{
  return "mavros_msgs/msg/VehicleInfo";
}

template<>
struct has_fixed_size<mavros_msgs::msg::VehicleInfo>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<mavros_msgs::msg::VehicleInfo>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<mavros_msgs::msg::VehicleInfo>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MAVROS_MSGS__MSG__DETAIL__VEHICLE_INFO__TRAITS_HPP_
