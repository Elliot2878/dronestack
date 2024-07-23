// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mavros_msgs:srv/FileList.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__FILE_LIST__TRAITS_HPP_
#define MAVROS_MSGS__SRV__DETAIL__FILE_LIST__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mavros_msgs/srv/detail/file_list__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace mavros_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const FileList_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: dir_path
  {
    out << "dir_path: ";
    rosidl_generator_traits::value_to_yaml(msg.dir_path, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const FileList_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: dir_path
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "dir_path: ";
    rosidl_generator_traits::value_to_yaml(msg.dir_path, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const FileList_Request & msg, bool use_flow_style = false)
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
  const mavros_msgs::srv::FileList_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  mavros_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mavros_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const mavros_msgs::srv::FileList_Request & msg)
{
  return mavros_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<mavros_msgs::srv::FileList_Request>()
{
  return "mavros_msgs::srv::FileList_Request";
}

template<>
inline const char * name<mavros_msgs::srv::FileList_Request>()
{
  return "mavros_msgs/srv/FileList_Request";
}

template<>
struct has_fixed_size<mavros_msgs::srv::FileList_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<mavros_msgs::srv::FileList_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<mavros_msgs::srv::FileList_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'list'
#include "mavros_msgs/msg/detail/file_entry__traits.hpp"

namespace mavros_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const FileList_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: list
  {
    if (msg.list.size() == 0) {
      out << "list: []";
    } else {
      out << "list: [";
      size_t pending_items = msg.list.size();
      for (auto item : msg.list) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
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
  const FileList_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: list
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.list.size() == 0) {
      out << "list: []\n";
    } else {
      out << "list:\n";
      for (auto item : msg.list) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
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

inline std::string to_yaml(const FileList_Response & msg, bool use_flow_style = false)
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
  const mavros_msgs::srv::FileList_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  mavros_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mavros_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const mavros_msgs::srv::FileList_Response & msg)
{
  return mavros_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<mavros_msgs::srv::FileList_Response>()
{
  return "mavros_msgs::srv::FileList_Response";
}

template<>
inline const char * name<mavros_msgs::srv::FileList_Response>()
{
  return "mavros_msgs/srv/FileList_Response";
}

template<>
struct has_fixed_size<mavros_msgs::srv::FileList_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<mavros_msgs::srv::FileList_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<mavros_msgs::srv::FileList_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<mavros_msgs::srv::FileList>()
{
  return "mavros_msgs::srv::FileList";
}

template<>
inline const char * name<mavros_msgs::srv::FileList>()
{
  return "mavros_msgs/srv/FileList";
}

template<>
struct has_fixed_size<mavros_msgs::srv::FileList>
  : std::integral_constant<
    bool,
    has_fixed_size<mavros_msgs::srv::FileList_Request>::value &&
    has_fixed_size<mavros_msgs::srv::FileList_Response>::value
  >
{
};

template<>
struct has_bounded_size<mavros_msgs::srv::FileList>
  : std::integral_constant<
    bool,
    has_bounded_size<mavros_msgs::srv::FileList_Request>::value &&
    has_bounded_size<mavros_msgs::srv::FileList_Response>::value
  >
{
};

template<>
struct is_service<mavros_msgs::srv::FileList>
  : std::true_type
{
};

template<>
struct is_service_request<mavros_msgs::srv::FileList_Request>
  : std::true_type
{
};

template<>
struct is_service_response<mavros_msgs::srv::FileList_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // MAVROS_MSGS__SRV__DETAIL__FILE_LIST__TRAITS_HPP_
