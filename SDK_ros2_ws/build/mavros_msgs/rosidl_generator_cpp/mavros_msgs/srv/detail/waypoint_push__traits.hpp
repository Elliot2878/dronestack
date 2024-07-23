// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mavros_msgs:srv/WaypointPush.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__WAYPOINT_PUSH__TRAITS_HPP_
#define MAVROS_MSGS__SRV__DETAIL__WAYPOINT_PUSH__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mavros_msgs/srv/detail/waypoint_push__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'waypoints'
#include "mavros_msgs/msg/detail/waypoint__traits.hpp"

namespace mavros_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const WaypointPush_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: start_index
  {
    out << "start_index: ";
    rosidl_generator_traits::value_to_yaml(msg.start_index, out);
    out << ", ";
  }

  // member: waypoints
  {
    if (msg.waypoints.size() == 0) {
      out << "waypoints: []";
    } else {
      out << "waypoints: [";
      size_t pending_items = msg.waypoints.size();
      for (auto item : msg.waypoints) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const WaypointPush_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: start_index
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "start_index: ";
    rosidl_generator_traits::value_to_yaml(msg.start_index, out);
    out << "\n";
  }

  // member: waypoints
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.waypoints.size() == 0) {
      out << "waypoints: []\n";
    } else {
      out << "waypoints:\n";
      for (auto item : msg.waypoints) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const WaypointPush_Request & msg, bool use_flow_style = false)
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
  const mavros_msgs::srv::WaypointPush_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  mavros_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mavros_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const mavros_msgs::srv::WaypointPush_Request & msg)
{
  return mavros_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<mavros_msgs::srv::WaypointPush_Request>()
{
  return "mavros_msgs::srv::WaypointPush_Request";
}

template<>
inline const char * name<mavros_msgs::srv::WaypointPush_Request>()
{
  return "mavros_msgs/srv/WaypointPush_Request";
}

template<>
struct has_fixed_size<mavros_msgs::srv::WaypointPush_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<mavros_msgs::srv::WaypointPush_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<mavros_msgs::srv::WaypointPush_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace mavros_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const WaypointPush_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: wp_transfered
  {
    out << "wp_transfered: ";
    rosidl_generator_traits::value_to_yaml(msg.wp_transfered, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const WaypointPush_Response & msg,
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

  // member: wp_transfered
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "wp_transfered: ";
    rosidl_generator_traits::value_to_yaml(msg.wp_transfered, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const WaypointPush_Response & msg, bool use_flow_style = false)
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
  const mavros_msgs::srv::WaypointPush_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  mavros_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mavros_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const mavros_msgs::srv::WaypointPush_Response & msg)
{
  return mavros_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<mavros_msgs::srv::WaypointPush_Response>()
{
  return "mavros_msgs::srv::WaypointPush_Response";
}

template<>
inline const char * name<mavros_msgs::srv::WaypointPush_Response>()
{
  return "mavros_msgs/srv/WaypointPush_Response";
}

template<>
struct has_fixed_size<mavros_msgs::srv::WaypointPush_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<mavros_msgs::srv::WaypointPush_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<mavros_msgs::srv::WaypointPush_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<mavros_msgs::srv::WaypointPush>()
{
  return "mavros_msgs::srv::WaypointPush";
}

template<>
inline const char * name<mavros_msgs::srv::WaypointPush>()
{
  return "mavros_msgs/srv/WaypointPush";
}

template<>
struct has_fixed_size<mavros_msgs::srv::WaypointPush>
  : std::integral_constant<
    bool,
    has_fixed_size<mavros_msgs::srv::WaypointPush_Request>::value &&
    has_fixed_size<mavros_msgs::srv::WaypointPush_Response>::value
  >
{
};

template<>
struct has_bounded_size<mavros_msgs::srv::WaypointPush>
  : std::integral_constant<
    bool,
    has_bounded_size<mavros_msgs::srv::WaypointPush_Request>::value &&
    has_bounded_size<mavros_msgs::srv::WaypointPush_Response>::value
  >
{
};

template<>
struct is_service<mavros_msgs::srv::WaypointPush>
  : std::true_type
{
};

template<>
struct is_service_request<mavros_msgs::srv::WaypointPush_Request>
  : std::true_type
{
};

template<>
struct is_service_response<mavros_msgs::srv::WaypointPush_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // MAVROS_MSGS__SRV__DETAIL__WAYPOINT_PUSH__TRAITS_HPP_
