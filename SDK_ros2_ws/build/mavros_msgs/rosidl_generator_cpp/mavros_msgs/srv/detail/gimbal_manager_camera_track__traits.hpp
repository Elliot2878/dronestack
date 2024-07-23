// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mavros_msgs:srv/GimbalManagerCameraTrack.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__GIMBAL_MANAGER_CAMERA_TRACK__TRAITS_HPP_
#define MAVROS_MSGS__SRV__DETAIL__GIMBAL_MANAGER_CAMERA_TRACK__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mavros_msgs/srv/detail/gimbal_manager_camera_track__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace mavros_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const GimbalManagerCameraTrack_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: mode
  {
    out << "mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mode, out);
    out << ", ";
  }

  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << ", ";
  }

  // member: y
  {
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << ", ";
  }

  // member: radius
  {
    out << "radius: ";
    rosidl_generator_traits::value_to_yaml(msg.radius, out);
    out << ", ";
  }

  // member: top_left_x
  {
    out << "top_left_x: ";
    rosidl_generator_traits::value_to_yaml(msg.top_left_x, out);
    out << ", ";
  }

  // member: top_left_y
  {
    out << "top_left_y: ";
    rosidl_generator_traits::value_to_yaml(msg.top_left_y, out);
    out << ", ";
  }

  // member: bottom_right_x
  {
    out << "bottom_right_x: ";
    rosidl_generator_traits::value_to_yaml(msg.bottom_right_x, out);
    out << ", ";
  }

  // member: bottom_right_y
  {
    out << "bottom_right_y: ";
    rosidl_generator_traits::value_to_yaml(msg.bottom_right_y, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GimbalManagerCameraTrack_Request & msg,
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

  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << "\n";
  }

  // member: radius
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "radius: ";
    rosidl_generator_traits::value_to_yaml(msg.radius, out);
    out << "\n";
  }

  // member: top_left_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "top_left_x: ";
    rosidl_generator_traits::value_to_yaml(msg.top_left_x, out);
    out << "\n";
  }

  // member: top_left_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "top_left_y: ";
    rosidl_generator_traits::value_to_yaml(msg.top_left_y, out);
    out << "\n";
  }

  // member: bottom_right_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bottom_right_x: ";
    rosidl_generator_traits::value_to_yaml(msg.bottom_right_x, out);
    out << "\n";
  }

  // member: bottom_right_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bottom_right_y: ";
    rosidl_generator_traits::value_to_yaml(msg.bottom_right_y, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GimbalManagerCameraTrack_Request & msg, bool use_flow_style = false)
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
  const mavros_msgs::srv::GimbalManagerCameraTrack_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  mavros_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mavros_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const mavros_msgs::srv::GimbalManagerCameraTrack_Request & msg)
{
  return mavros_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<mavros_msgs::srv::GimbalManagerCameraTrack_Request>()
{
  return "mavros_msgs::srv::GimbalManagerCameraTrack_Request";
}

template<>
inline const char * name<mavros_msgs::srv::GimbalManagerCameraTrack_Request>()
{
  return "mavros_msgs/srv/GimbalManagerCameraTrack_Request";
}

template<>
struct has_fixed_size<mavros_msgs::srv::GimbalManagerCameraTrack_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<mavros_msgs::srv::GimbalManagerCameraTrack_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<mavros_msgs::srv::GimbalManagerCameraTrack_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace mavros_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const GimbalManagerCameraTrack_Response & msg,
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
  const GimbalManagerCameraTrack_Response & msg,
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

inline std::string to_yaml(const GimbalManagerCameraTrack_Response & msg, bool use_flow_style = false)
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
  const mavros_msgs::srv::GimbalManagerCameraTrack_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  mavros_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mavros_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const mavros_msgs::srv::GimbalManagerCameraTrack_Response & msg)
{
  return mavros_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<mavros_msgs::srv::GimbalManagerCameraTrack_Response>()
{
  return "mavros_msgs::srv::GimbalManagerCameraTrack_Response";
}

template<>
inline const char * name<mavros_msgs::srv::GimbalManagerCameraTrack_Response>()
{
  return "mavros_msgs/srv/GimbalManagerCameraTrack_Response";
}

template<>
struct has_fixed_size<mavros_msgs::srv::GimbalManagerCameraTrack_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<mavros_msgs::srv::GimbalManagerCameraTrack_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<mavros_msgs::srv::GimbalManagerCameraTrack_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<mavros_msgs::srv::GimbalManagerCameraTrack>()
{
  return "mavros_msgs::srv::GimbalManagerCameraTrack";
}

template<>
inline const char * name<mavros_msgs::srv::GimbalManagerCameraTrack>()
{
  return "mavros_msgs/srv/GimbalManagerCameraTrack";
}

template<>
struct has_fixed_size<mavros_msgs::srv::GimbalManagerCameraTrack>
  : std::integral_constant<
    bool,
    has_fixed_size<mavros_msgs::srv::GimbalManagerCameraTrack_Request>::value &&
    has_fixed_size<mavros_msgs::srv::GimbalManagerCameraTrack_Response>::value
  >
{
};

template<>
struct has_bounded_size<mavros_msgs::srv::GimbalManagerCameraTrack>
  : std::integral_constant<
    bool,
    has_bounded_size<mavros_msgs::srv::GimbalManagerCameraTrack_Request>::value &&
    has_bounded_size<mavros_msgs::srv::GimbalManagerCameraTrack_Response>::value
  >
{
};

template<>
struct is_service<mavros_msgs::srv::GimbalManagerCameraTrack>
  : std::true_type
{
};

template<>
struct is_service_request<mavros_msgs::srv::GimbalManagerCameraTrack_Request>
  : std::true_type
{
};

template<>
struct is_service_response<mavros_msgs::srv::GimbalManagerCameraTrack_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // MAVROS_MSGS__SRV__DETAIL__GIMBAL_MANAGER_CAMERA_TRACK__TRAITS_HPP_
