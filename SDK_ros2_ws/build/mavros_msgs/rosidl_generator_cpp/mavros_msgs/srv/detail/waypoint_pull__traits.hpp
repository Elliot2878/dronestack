// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mavros_msgs:srv/WaypointPull.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__WAYPOINT_PULL__TRAITS_HPP_
#define MAVROS_MSGS__SRV__DETAIL__WAYPOINT_PULL__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mavros_msgs/srv/detail/waypoint_pull__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace mavros_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const WaypointPull_Request & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const WaypointPull_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const WaypointPull_Request & msg, bool use_flow_style = false)
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
  const mavros_msgs::srv::WaypointPull_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  mavros_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mavros_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const mavros_msgs::srv::WaypointPull_Request & msg)
{
  return mavros_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<mavros_msgs::srv::WaypointPull_Request>()
{
  return "mavros_msgs::srv::WaypointPull_Request";
}

template<>
inline const char * name<mavros_msgs::srv::WaypointPull_Request>()
{
  return "mavros_msgs/srv/WaypointPull_Request";
}

template<>
struct has_fixed_size<mavros_msgs::srv::WaypointPull_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<mavros_msgs::srv::WaypointPull_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<mavros_msgs::srv::WaypointPull_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace mavros_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const WaypointPull_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: wp_received
  {
    out << "wp_received: ";
    rosidl_generator_traits::value_to_yaml(msg.wp_received, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const WaypointPull_Response & msg,
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

  // member: wp_received
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "wp_received: ";
    rosidl_generator_traits::value_to_yaml(msg.wp_received, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const WaypointPull_Response & msg, bool use_flow_style = false)
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
  const mavros_msgs::srv::WaypointPull_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  mavros_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mavros_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const mavros_msgs::srv::WaypointPull_Response & msg)
{
  return mavros_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<mavros_msgs::srv::WaypointPull_Response>()
{
  return "mavros_msgs::srv::WaypointPull_Response";
}

template<>
inline const char * name<mavros_msgs::srv::WaypointPull_Response>()
{
  return "mavros_msgs/srv/WaypointPull_Response";
}

template<>
struct has_fixed_size<mavros_msgs::srv::WaypointPull_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<mavros_msgs::srv::WaypointPull_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<mavros_msgs::srv::WaypointPull_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<mavros_msgs::srv::WaypointPull>()
{
  return "mavros_msgs::srv::WaypointPull";
}

template<>
inline const char * name<mavros_msgs::srv::WaypointPull>()
{
  return "mavros_msgs/srv/WaypointPull";
}

template<>
struct has_fixed_size<mavros_msgs::srv::WaypointPull>
  : std::integral_constant<
    bool,
    has_fixed_size<mavros_msgs::srv::WaypointPull_Request>::value &&
    has_fixed_size<mavros_msgs::srv::WaypointPull_Response>::value
  >
{
};

template<>
struct has_bounded_size<mavros_msgs::srv::WaypointPull>
  : std::integral_constant<
    bool,
    has_bounded_size<mavros_msgs::srv::WaypointPull_Request>::value &&
    has_bounded_size<mavros_msgs::srv::WaypointPull_Response>::value
  >
{
};

template<>
struct is_service<mavros_msgs::srv::WaypointPull>
  : std::true_type
{
};

template<>
struct is_service_request<mavros_msgs::srv::WaypointPull_Request>
  : std::true_type
{
};

template<>
struct is_service_response<mavros_msgs::srv::WaypointPull_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // MAVROS_MSGS__SRV__DETAIL__WAYPOINT_PULL__TRAITS_HPP_