// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mavros_msgs:srv/GimbalManagerPitchyaw.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__GIMBAL_MANAGER_PITCHYAW__TRAITS_HPP_
#define MAVROS_MSGS__SRV__DETAIL__GIMBAL_MANAGER_PITCHYAW__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mavros_msgs/srv/detail/gimbal_manager_pitchyaw__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace mavros_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const GimbalManagerPitchyaw_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: pitch
  {
    out << "pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch, out);
    out << ", ";
  }

  // member: yaw
  {
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
    out << ", ";
  }

  // member: pitch_rate
  {
    out << "pitch_rate: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch_rate, out);
    out << ", ";
  }

  // member: yaw_rate
  {
    out << "yaw_rate: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw_rate, out);
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
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GimbalManagerPitchyaw_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: pitch
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch, out);
    out << "\n";
  }

  // member: yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
    out << "\n";
  }

  // member: pitch_rate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pitch_rate: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch_rate, out);
    out << "\n";
  }

  // member: yaw_rate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw_rate: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw_rate, out);
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

  // member: gimbal_device_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gimbal_device_id: ";
    rosidl_generator_traits::value_to_yaml(msg.gimbal_device_id, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GimbalManagerPitchyaw_Request & msg, bool use_flow_style = false)
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
  const mavros_msgs::srv::GimbalManagerPitchyaw_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  mavros_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mavros_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const mavros_msgs::srv::GimbalManagerPitchyaw_Request & msg)
{
  return mavros_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<mavros_msgs::srv::GimbalManagerPitchyaw_Request>()
{
  return "mavros_msgs::srv::GimbalManagerPitchyaw_Request";
}

template<>
inline const char * name<mavros_msgs::srv::GimbalManagerPitchyaw_Request>()
{
  return "mavros_msgs/srv/GimbalManagerPitchyaw_Request";
}

template<>
struct has_fixed_size<mavros_msgs::srv::GimbalManagerPitchyaw_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<mavros_msgs::srv::GimbalManagerPitchyaw_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<mavros_msgs::srv::GimbalManagerPitchyaw_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace mavros_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const GimbalManagerPitchyaw_Response & msg,
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
  const GimbalManagerPitchyaw_Response & msg,
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

inline std::string to_yaml(const GimbalManagerPitchyaw_Response & msg, bool use_flow_style = false)
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
  const mavros_msgs::srv::GimbalManagerPitchyaw_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  mavros_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mavros_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const mavros_msgs::srv::GimbalManagerPitchyaw_Response & msg)
{
  return mavros_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<mavros_msgs::srv::GimbalManagerPitchyaw_Response>()
{
  return "mavros_msgs::srv::GimbalManagerPitchyaw_Response";
}

template<>
inline const char * name<mavros_msgs::srv::GimbalManagerPitchyaw_Response>()
{
  return "mavros_msgs/srv/GimbalManagerPitchyaw_Response";
}

template<>
struct has_fixed_size<mavros_msgs::srv::GimbalManagerPitchyaw_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<mavros_msgs::srv::GimbalManagerPitchyaw_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<mavros_msgs::srv::GimbalManagerPitchyaw_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<mavros_msgs::srv::GimbalManagerPitchyaw>()
{
  return "mavros_msgs::srv::GimbalManagerPitchyaw";
}

template<>
inline const char * name<mavros_msgs::srv::GimbalManagerPitchyaw>()
{
  return "mavros_msgs/srv/GimbalManagerPitchyaw";
}

template<>
struct has_fixed_size<mavros_msgs::srv::GimbalManagerPitchyaw>
  : std::integral_constant<
    bool,
    has_fixed_size<mavros_msgs::srv::GimbalManagerPitchyaw_Request>::value &&
    has_fixed_size<mavros_msgs::srv::GimbalManagerPitchyaw_Response>::value
  >
{
};

template<>
struct has_bounded_size<mavros_msgs::srv::GimbalManagerPitchyaw>
  : std::integral_constant<
    bool,
    has_bounded_size<mavros_msgs::srv::GimbalManagerPitchyaw_Request>::value &&
    has_bounded_size<mavros_msgs::srv::GimbalManagerPitchyaw_Response>::value
  >
{
};

template<>
struct is_service<mavros_msgs::srv::GimbalManagerPitchyaw>
  : std::true_type
{
};

template<>
struct is_service_request<mavros_msgs::srv::GimbalManagerPitchyaw_Request>
  : std::true_type
{
};

template<>
struct is_service_response<mavros_msgs::srv::GimbalManagerPitchyaw_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // MAVROS_MSGS__SRV__DETAIL__GIMBAL_MANAGER_PITCHYAW__TRAITS_HPP_
