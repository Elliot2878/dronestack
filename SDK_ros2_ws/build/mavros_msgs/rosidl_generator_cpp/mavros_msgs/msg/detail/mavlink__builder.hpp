// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:msg/Mavlink.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__MAVLINK__BUILDER_HPP_
#define MAVROS_MSGS__MSG__DETAIL__MAVLINK__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/msg/detail/mavlink__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace msg
{

namespace builder
{

class Init_Mavlink_signature
{
public:
  explicit Init_Mavlink_signature(::mavros_msgs::msg::Mavlink & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::msg::Mavlink signature(::mavros_msgs::msg::Mavlink::_signature_type arg)
  {
    msg_.signature = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::msg::Mavlink msg_;
};

class Init_Mavlink_payload64
{
public:
  explicit Init_Mavlink_payload64(::mavros_msgs::msg::Mavlink & msg)
  : msg_(msg)
  {}
  Init_Mavlink_signature payload64(::mavros_msgs::msg::Mavlink::_payload64_type arg)
  {
    msg_.payload64 = std::move(arg);
    return Init_Mavlink_signature(msg_);
  }

private:
  ::mavros_msgs::msg::Mavlink msg_;
};

class Init_Mavlink_checksum
{
public:
  explicit Init_Mavlink_checksum(::mavros_msgs::msg::Mavlink & msg)
  : msg_(msg)
  {}
  Init_Mavlink_payload64 checksum(::mavros_msgs::msg::Mavlink::_checksum_type arg)
  {
    msg_.checksum = std::move(arg);
    return Init_Mavlink_payload64(msg_);
  }

private:
  ::mavros_msgs::msg::Mavlink msg_;
};

class Init_Mavlink_msgid
{
public:
  explicit Init_Mavlink_msgid(::mavros_msgs::msg::Mavlink & msg)
  : msg_(msg)
  {}
  Init_Mavlink_checksum msgid(::mavros_msgs::msg::Mavlink::_msgid_type arg)
  {
    msg_.msgid = std::move(arg);
    return Init_Mavlink_checksum(msg_);
  }

private:
  ::mavros_msgs::msg::Mavlink msg_;
};

class Init_Mavlink_compid
{
public:
  explicit Init_Mavlink_compid(::mavros_msgs::msg::Mavlink & msg)
  : msg_(msg)
  {}
  Init_Mavlink_msgid compid(::mavros_msgs::msg::Mavlink::_compid_type arg)
  {
    msg_.compid = std::move(arg);
    return Init_Mavlink_msgid(msg_);
  }

private:
  ::mavros_msgs::msg::Mavlink msg_;
};

class Init_Mavlink_sysid
{
public:
  explicit Init_Mavlink_sysid(::mavros_msgs::msg::Mavlink & msg)
  : msg_(msg)
  {}
  Init_Mavlink_compid sysid(::mavros_msgs::msg::Mavlink::_sysid_type arg)
  {
    msg_.sysid = std::move(arg);
    return Init_Mavlink_compid(msg_);
  }

private:
  ::mavros_msgs::msg::Mavlink msg_;
};

class Init_Mavlink_seq
{
public:
  explicit Init_Mavlink_seq(::mavros_msgs::msg::Mavlink & msg)
  : msg_(msg)
  {}
  Init_Mavlink_sysid seq(::mavros_msgs::msg::Mavlink::_seq_type arg)
  {
    msg_.seq = std::move(arg);
    return Init_Mavlink_sysid(msg_);
  }

private:
  ::mavros_msgs::msg::Mavlink msg_;
};

class Init_Mavlink_compat_flags
{
public:
  explicit Init_Mavlink_compat_flags(::mavros_msgs::msg::Mavlink & msg)
  : msg_(msg)
  {}
  Init_Mavlink_seq compat_flags(::mavros_msgs::msg::Mavlink::_compat_flags_type arg)
  {
    msg_.compat_flags = std::move(arg);
    return Init_Mavlink_seq(msg_);
  }

private:
  ::mavros_msgs::msg::Mavlink msg_;
};

class Init_Mavlink_incompat_flags
{
public:
  explicit Init_Mavlink_incompat_flags(::mavros_msgs::msg::Mavlink & msg)
  : msg_(msg)
  {}
  Init_Mavlink_compat_flags incompat_flags(::mavros_msgs::msg::Mavlink::_incompat_flags_type arg)
  {
    msg_.incompat_flags = std::move(arg);
    return Init_Mavlink_compat_flags(msg_);
  }

private:
  ::mavros_msgs::msg::Mavlink msg_;
};

class Init_Mavlink_len
{
public:
  explicit Init_Mavlink_len(::mavros_msgs::msg::Mavlink & msg)
  : msg_(msg)
  {}
  Init_Mavlink_incompat_flags len(::mavros_msgs::msg::Mavlink::_len_type arg)
  {
    msg_.len = std::move(arg);
    return Init_Mavlink_incompat_flags(msg_);
  }

private:
  ::mavros_msgs::msg::Mavlink msg_;
};

class Init_Mavlink_magic
{
public:
  explicit Init_Mavlink_magic(::mavros_msgs::msg::Mavlink & msg)
  : msg_(msg)
  {}
  Init_Mavlink_len magic(::mavros_msgs::msg::Mavlink::_magic_type arg)
  {
    msg_.magic = std::move(arg);
    return Init_Mavlink_len(msg_);
  }

private:
  ::mavros_msgs::msg::Mavlink msg_;
};

class Init_Mavlink_framing_status
{
public:
  explicit Init_Mavlink_framing_status(::mavros_msgs::msg::Mavlink & msg)
  : msg_(msg)
  {}
  Init_Mavlink_magic framing_status(::mavros_msgs::msg::Mavlink::_framing_status_type arg)
  {
    msg_.framing_status = std::move(arg);
    return Init_Mavlink_magic(msg_);
  }

private:
  ::mavros_msgs::msg::Mavlink msg_;
};

class Init_Mavlink_header
{
public:
  Init_Mavlink_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Mavlink_framing_status header(::mavros_msgs::msg::Mavlink::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Mavlink_framing_status(msg_);
  }

private:
  ::mavros_msgs::msg::Mavlink msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::msg::Mavlink>()
{
  return mavros_msgs::msg::builder::Init_Mavlink_header();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__MAVLINK__BUILDER_HPP_
