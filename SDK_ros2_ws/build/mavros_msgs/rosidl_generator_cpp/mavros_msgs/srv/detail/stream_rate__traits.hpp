// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mavros_msgs:srv/StreamRate.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__STREAM_RATE__TRAITS_HPP_
#define MAVROS_MSGS__SRV__DETAIL__STREAM_RATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mavros_msgs/srv/detail/stream_rate__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace mavros_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const StreamRate_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: stream_id
  {
    out << "stream_id: ";
    rosidl_generator_traits::value_to_yaml(msg.stream_id, out);
    out << ", ";
  }

  // member: message_rate
  {
    out << "message_rate: ";
    rosidl_generator_traits::value_to_yaml(msg.message_rate, out);
    out << ", ";
  }

  // member: on_off
  {
    out << "on_off: ";
    rosidl_generator_traits::value_to_yaml(msg.on_off, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const StreamRate_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: stream_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stream_id: ";
    rosidl_generator_traits::value_to_yaml(msg.stream_id, out);
    out << "\n";
  }

  // member: message_rate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message_rate: ";
    rosidl_generator_traits::value_to_yaml(msg.message_rate, out);
    out << "\n";
  }

  // member: on_off
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "on_off: ";
    rosidl_generator_traits::value_to_yaml(msg.on_off, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const StreamRate_Request & msg, bool use_flow_style = false)
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
  const mavros_msgs::srv::StreamRate_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  mavros_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mavros_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const mavros_msgs::srv::StreamRate_Request & msg)
{
  return mavros_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<mavros_msgs::srv::StreamRate_Request>()
{
  return "mavros_msgs::srv::StreamRate_Request";
}

template<>
inline const char * name<mavros_msgs::srv::StreamRate_Request>()
{
  return "mavros_msgs/srv/StreamRate_Request";
}

template<>
struct has_fixed_size<mavros_msgs::srv::StreamRate_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<mavros_msgs::srv::StreamRate_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<mavros_msgs::srv::StreamRate_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace mavros_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const StreamRate_Response & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const StreamRate_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const StreamRate_Response & msg, bool use_flow_style = false)
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
  const mavros_msgs::srv::StreamRate_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  mavros_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mavros_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const mavros_msgs::srv::StreamRate_Response & msg)
{
  return mavros_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<mavros_msgs::srv::StreamRate_Response>()
{
  return "mavros_msgs::srv::StreamRate_Response";
}

template<>
inline const char * name<mavros_msgs::srv::StreamRate_Response>()
{
  return "mavros_msgs/srv/StreamRate_Response";
}

template<>
struct has_fixed_size<mavros_msgs::srv::StreamRate_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<mavros_msgs::srv::StreamRate_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<mavros_msgs::srv::StreamRate_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<mavros_msgs::srv::StreamRate>()
{
  return "mavros_msgs::srv::StreamRate";
}

template<>
inline const char * name<mavros_msgs::srv::StreamRate>()
{
  return "mavros_msgs/srv/StreamRate";
}

template<>
struct has_fixed_size<mavros_msgs::srv::StreamRate>
  : std::integral_constant<
    bool,
    has_fixed_size<mavros_msgs::srv::StreamRate_Request>::value &&
    has_fixed_size<mavros_msgs::srv::StreamRate_Response>::value
  >
{
};

template<>
struct has_bounded_size<mavros_msgs::srv::StreamRate>
  : std::integral_constant<
    bool,
    has_bounded_size<mavros_msgs::srv::StreamRate_Request>::value &&
    has_bounded_size<mavros_msgs::srv::StreamRate_Response>::value
  >
{
};

template<>
struct is_service<mavros_msgs::srv::StreamRate>
  : std::true_type
{
};

template<>
struct is_service_request<mavros_msgs::srv::StreamRate_Request>
  : std::true_type
{
};

template<>
struct is_service_response<mavros_msgs::srv::StreamRate_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // MAVROS_MSGS__SRV__DETAIL__STREAM_RATE__TRAITS_HPP_
