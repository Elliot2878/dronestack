// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mavros_msgs:srv/GimbalManagerSetRoi.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__GIMBAL_MANAGER_SET_ROI__TRAITS_HPP_
#define MAVROS_MSGS__SRV__DETAIL__GIMBAL_MANAGER_SET_ROI__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mavros_msgs/srv/detail/gimbal_manager_set_roi__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace mavros_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const GimbalManagerSetRoi_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: mode
  {
    out << "mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mode, out);
    out << ", ";
  }

  // member: gimbal_device_id
  {
    out << "gimbal_device_id: ";
    rosidl_generator_traits::value_to_yaml(msg.gimbal_device_id, out);
    out << ", ";
  }

  // member: latitude
  {
    out << "latitude: ";
    rosidl_generator_traits::value_to_yaml(msg.latitude, out);
    out << ", ";
  }

  // member: longitude
  {
    out << "longitude: ";
    rosidl_generator_traits::value_to_yaml(msg.longitude, out);
    out << ", ";
  }

  // member: altitude
  {
    out << "altitude: ";
    rosidl_generator_traits::value_to_yaml(msg.altitude, out);
    out << ", ";
  }

  // member: pitch_offset
  {
    out << "pitch_offset: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch_offset, out);
    out << ", ";
  }

  // member: roll_offset
  {
    out << "roll_offset: ";
    rosidl_generator_traits::value_to_yaml(msg.roll_offset, out);
    out << ", ";
  }

  // member: yaw_offset
  {
    out << "yaw_offset: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw_offset, out);
    out << ", ";
  }

  // member: sysid
  {
    out << "sysid: ";
    rosidl_generator_traits::value_to_yaml(msg.sysid, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GimbalManagerSetRoi_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mode, out);
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

  // member: latitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "latitude: ";
    rosidl_generator_traits::value_to_yaml(msg.latitude, out);
    out << "\n";
  }

  // member: longitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "longitude: ";
    rosidl_generator_traits::value_to_yaml(msg.longitude, out);
    out << "\n";
  }

  // member: altitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "altitude: ";
    rosidl_generator_traits::value_to_yaml(msg.altitude, out);
    out << "\n";
  }

  // member: pitch_offset
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pitch_offset: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch_offset, out);
    out << "\n";
  }

  // member: roll_offset
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "roll_offset: ";
    rosidl_generator_traits::value_to_yaml(msg.roll_offset, out);
    out << "\n";
  }

  // member: yaw_offset
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw_offset: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw_offset, out);
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
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GimbalManagerSetRoi_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace mavros_msgs

namespace rosidl_generator_traits
{

[[deprecated("use mavros_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const mavros_msgs::srv::GimbalManagerSetRoi_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  mavros_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mavros_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const mavros_msgs::srv::GimbalManagerSetRoi_Request & msg)
{
  return mavros_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<mavros_msgs::srv::GimbalManagerSetRoi_Request>()
{
  return "mavros_msgs::srv::GimbalManagerSetRoi_Request";
}

template<>
inline const char * name<mavros_msgs::srv::GimbalManagerSetRoi_Request>()
{
  return "mavros_msgs/srv/GimbalManagerSetRoi_Request";
}

template<>
struct has_fixed_size<mavros_msgs::srv::GimbalManagerSetRoi_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<mavros_msgs::srv::GimbalManagerSetRoi_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<mavros_msgs::srv::GimbalManagerSetRoi_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace mavros_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const GimbalManagerSetRoi_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: result
  {
    out << "result: ";
    rosidl_generator_traits::value_to_yaml(msg.result, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GimbalManagerSetRoi_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: result
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "result: ";
    rosidl_generator_traits::value_to_yaml(msg.result, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GimbalManagerSetRoi_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace mavros_msgs

namespace rosidl_generator_traits
{

[[deprecated("use mavros_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const mavros_msgs::srv::GimbalManagerSetRoi_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  mavros_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mavros_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const mavros_msgs::srv::GimbalManagerSetRoi_Response & msg)
{
  return mavros_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<mavros_msgs::srv::GimbalManagerSetRoi_Response>()
{
  return "mavros_msgs::srv::GimbalManagerSetRoi_Response";
}

template<>
inline const char * name<mavros_msgs::srv::GimbalManagerSetRoi_Response>()
{
  return "mavros_msgs/srv/GimbalManagerSetRoi_Response";
}

template<>
struct has_fixed_size<mavros_msgs::srv::GimbalManagerSetRoi_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<mavros_msgs::srv::GimbalManagerSetRoi_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<mavros_msgs::srv::GimbalManagerSetRoi_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<mavros_msgs::srv::GimbalManagerSetRoi>()
{
  return "mavros_msgs::srv::GimbalManagerSetRoi";
}

template<>
inline const char * name<mavros_msgs::srv::GimbalManagerSetRoi>()
{
  return "mavros_msgs/srv/GimbalManagerSetRoi";
}

template<>
struct has_fixed_size<mavros_msgs::srv::GimbalManagerSetRoi>
  : std::integral_constant<
    bool,
    has_fixed_size<mavros_msgs::srv::GimbalManagerSetRoi_Request>::value &&
    has_fixed_size<mavros_msgs::srv::GimbalManagerSetRoi_Response>::value
  >
{
};

template<>
struct has_bounded_size<mavros_msgs::srv::GimbalManagerSetRoi>
  : std::integral_constant<
    bool,
    has_bounded_size<mavros_msgs::srv::GimbalManagerSetRoi_Request>::value &&
    has_bounded_size<mavros_msgs::srv::GimbalManagerSetRoi_Response>::value
  >
{
};

template<>
struct is_service<mavros_msgs::srv::GimbalManagerSetRoi>
  : std::true_type
{
};

template<>
struct is_service_request<mavros_msgs::srv::GimbalManagerSetRoi_Request>
  : std::true_type
{
};

template<>
struct is_service_response<mavros_msgs::srv::GimbalManagerSetRoi_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // MAVROS_MSGS__SRV__DETAIL__GIMBAL_MANAGER_SET_ROI__TRAITS_HPP_
