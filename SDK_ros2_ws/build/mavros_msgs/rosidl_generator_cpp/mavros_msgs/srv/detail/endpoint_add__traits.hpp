// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mavros_msgs:srv/EndpointAdd.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__ENDPOINT_ADD__TRAITS_HPP_
#define MAVROS_MSGS__SRV__DETAIL__ENDPOINT_ADD__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mavros_msgs/srv/detail/endpoint_add__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace mavros_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const EndpointAdd_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: url
  {
    out << "url: ";
    rosidl_generator_traits::value_to_yaml(msg.url, out);
    out << ", ";
  }

  // member: type
  {
    out << "type: ";
    rosidl_generator_traits::value_to_yaml(msg.type, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const EndpointAdd_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: url
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "url: ";
    rosidl_generator_traits::value_to_yaml(msg.url, out);
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
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const EndpointAdd_Request & msg, bool use_flow_style = false)
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
  const mavros_msgs::srv::EndpointAdd_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  mavros_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mavros_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const mavros_msgs::srv::EndpointAdd_Request & msg)
{
  return mavros_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<mavros_msgs::srv::EndpointAdd_Request>()
{
  return "mavros_msgs::srv::EndpointAdd_Request";
}

template<>
inline const char * name<mavros_msgs::srv::EndpointAdd_Request>()
{
  return "mavros_msgs/srv/EndpointAdd_Request";
}

template<>
struct has_fixed_size<mavros_msgs::srv::EndpointAdd_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<mavros_msgs::srv::EndpointAdd_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<mavros_msgs::srv::EndpointAdd_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace mavros_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const EndpointAdd_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: successful
  {
    out << "successful: ";
    rosidl_generator_traits::value_to_yaml(msg.successful, out);
    out << ", ";
  }

  // member: reason
  {
    out << "reason: ";
    rosidl_generator_traits::value_to_yaml(msg.reason, out);
    out << ", ";
  }

  // member: id
  {
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const EndpointAdd_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: successful
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "successful: ";
    rosidl_generator_traits::value_to_yaml(msg.successful, out);
    out << "\n";
  }

  // member: reason
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reason: ";
    rosidl_generator_traits::value_to_yaml(msg.reason, out);
    out << "\n";
  }

  // member: id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const EndpointAdd_Response & msg, bool use_flow_style = false)
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
  const mavros_msgs::srv::EndpointAdd_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  mavros_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mavros_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const mavros_msgs::srv::EndpointAdd_Response & msg)
{
  return mavros_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<mavros_msgs::srv::EndpointAdd_Response>()
{
  return "mavros_msgs::srv::EndpointAdd_Response";
}

template<>
inline const char * name<mavros_msgs::srv::EndpointAdd_Response>()
{
  return "mavros_msgs/srv/EndpointAdd_Response";
}

template<>
struct has_fixed_size<mavros_msgs::srv::EndpointAdd_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<mavros_msgs::srv::EndpointAdd_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<mavros_msgs::srv::EndpointAdd_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<mavros_msgs::srv::EndpointAdd>()
{
  return "mavros_msgs::srv::EndpointAdd";
}

template<>
inline const char * name<mavros_msgs::srv::EndpointAdd>()
{
  return "mavros_msgs/srv/EndpointAdd";
}

template<>
struct has_fixed_size<mavros_msgs::srv::EndpointAdd>
  : std::integral_constant<
    bool,
    has_fixed_size<mavros_msgs::srv::EndpointAdd_Request>::value &&
    has_fixed_size<mavros_msgs::srv::EndpointAdd_Response>::value
  >
{
};

template<>
struct has_bounded_size<mavros_msgs::srv::EndpointAdd>
  : std::integral_constant<
    bool,
    has_bounded_size<mavros_msgs::srv::EndpointAdd_Request>::value &&
    has_bounded_size<mavros_msgs::srv::EndpointAdd_Response>::value
  >
{
};

template<>
struct is_service<mavros_msgs::srv::EndpointAdd>
  : std::true_type
{
};

template<>
struct is_service_request<mavros_msgs::srv::EndpointAdd_Request>
  : std::true_type
{
};

template<>
struct is_service_response<mavros_msgs::srv::EndpointAdd_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // MAVROS_MSGS__SRV__DETAIL__ENDPOINT_ADD__TRAITS_HPP_
