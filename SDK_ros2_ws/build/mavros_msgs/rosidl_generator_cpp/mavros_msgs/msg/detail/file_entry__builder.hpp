// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:msg/FileEntry.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__FILE_ENTRY__BUILDER_HPP_
#define MAVROS_MSGS__MSG__DETAIL__FILE_ENTRY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/msg/detail/file_entry__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace msg
{

namespace builder
{

class Init_FileEntry_size
{
public:
  explicit Init_FileEntry_size(::mavros_msgs::msg::FileEntry & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::msg::FileEntry size(::mavros_msgs::msg::FileEntry::_size_type arg)
  {
    msg_.size = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::msg::FileEntry msg_;
};

class Init_FileEntry_type
{
public:
  explicit Init_FileEntry_type(::mavros_msgs::msg::FileEntry & msg)
  : msg_(msg)
  {}
  Init_FileEntry_size type(::mavros_msgs::msg::FileEntry::_type_type arg)
  {
    msg_.type = std::move(arg);
    return Init_FileEntry_size(msg_);
  }

private:
  ::mavros_msgs::msg::FileEntry msg_;
};

class Init_FileEntry_name
{
public:
  Init_FileEntry_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FileEntry_type name(::mavros_msgs::msg::FileEntry::_name_type arg)
  {
    msg_.name = std::move(arg);
    return Init_FileEntry_type(msg_);
  }

private:
  ::mavros_msgs::msg::FileEntry msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::msg::FileEntry>()
{
  return mavros_msgs::msg::builder::Init_FileEntry_name();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__FILE_ENTRY__BUILDER_HPP_
