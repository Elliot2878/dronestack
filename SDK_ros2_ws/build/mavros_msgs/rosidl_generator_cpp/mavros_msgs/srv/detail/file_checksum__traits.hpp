// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mavros_msgs:srv/FileChecksum.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__FILE_CHECKSUM__TRAITS_HPP_
#define MAVROS_MSGS__SRV__DETAIL__FILE_CHECKSUM__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mavros_msgs/srv/detail/file_checksum__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace mavros_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const FileChecksum_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: file_path
  {
    out << "file_path: ";
    rosidl_generator_traits::value_to_yaml(msg.file_path, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const FileChecksum_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: file_path
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "file_path: ";
    rosidl_generator_traits::value_to_yaml(msg.file_path, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const FileChecksum_Request & msg, bool use_flow_style = false)
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
  const mavros_msgs::srv::FileChecksum_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  mavros_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mavros_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const mavros_msgs::srv::FileChecksum_Request & msg)
{
  return mavros_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<mavros_msgs::srv::FileChecksum_Request>()
{
  return "mavros_msgs::srv::FileChecksum_Request";
}

template<>
inline const char * name<mavros_msgs::srv::FileChecksum_Request>()
{
  return "mavros_msgs/srv/FileChecksum_Request";
}

template<>
struct has_fixed_size<mavros_msgs::srv::FileChecksum_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<mavros_msgs::srv::FileChecksum_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<mavros_msgs::srv::FileChecksum_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace mavros_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const FileChecksum_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: crc32
  {
    out << "crc32: ";
    rosidl_generator_traits::value_to_yaml(msg.crc32, out);
    out << ", ";
  }

  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: r_errno
  {
    out << "r_errno: ";
    rosidl_generator_traits::value_to_yaml(msg.r_errno, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const FileChecksum_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: crc32
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "crc32: ";
    rosidl_generator_traits::value_to_yaml(msg.crc32, out);
    out << "\n";
  }

  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: r_errno
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "r_errno: ";
    rosidl_generator_traits::value_to_yaml(msg.r_errno, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const FileChecksum_Response & msg, bool use_flow_style = false)
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
  const mavros_msgs::srv::FileChecksum_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  mavros_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mavros_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const mavros_msgs::srv::FileChecksum_Response & msg)
{
  return mavros_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<mavros_msgs::srv::FileChecksum_Response>()
{
  return "mavros_msgs::srv::FileChecksum_Response";
}

template<>
inline const char * name<mavros_msgs::srv::FileChecksum_Response>()
{
  return "mavros_msgs/srv/FileChecksum_Response";
}

template<>
struct has_fixed_size<mavros_msgs::srv::FileChecksum_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<mavros_msgs::srv::FileChecksum_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<mavros_msgs::srv::FileChecksum_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<mavros_msgs::srv::FileChecksum>()
{
  return "mavros_msgs::srv::FileChecksum";
}

template<>
inline const char * name<mavros_msgs::srv::FileChecksum>()
{
  return "mavros_msgs/srv/FileChecksum";
}

template<>
struct has_fixed_size<mavros_msgs::srv::FileChecksum>
  : std::integral_constant<
    bool,
    has_fixed_size<mavros_msgs::srv::FileChecksum_Request>::value &&
    has_fixed_size<mavros_msgs::srv::FileChecksum_Response>::value
  >
{
};

template<>
struct has_bounded_size<mavros_msgs::srv::FileChecksum>
  : std::integral_constant<
    bool,
    has_bounded_size<mavros_msgs::srv::FileChecksum_Request>::value &&
    has_bounded_size<mavros_msgs::srv::FileChecksum_Response>::value
  >
{
};

template<>
struct is_service<mavros_msgs::srv::FileChecksum>
  : std::true_type
{
};

template<>
struct is_service_request<mavros_msgs::srv::FileChecksum_Request>
  : std::true_type
{
};

template<>
struct is_service_response<mavros_msgs::srv::FileChecksum_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // MAVROS_MSGS__SRV__DETAIL__FILE_CHECKSUM__TRAITS_HPP_
