// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:msg/DebugValue.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__DEBUG_VALUE__BUILDER_HPP_
#define MAVROS_MSGS__MSG__DETAIL__DEBUG_VALUE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/msg/detail/debug_value__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace msg
{

namespace builder
{

class Init_DebugValue_type
{
public:
  explicit Init_DebugValue_type(::mavros_msgs::msg::DebugValue & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::msg::DebugValue type(::mavros_msgs::msg::DebugValue::_type_type arg)
  {
    msg_.type = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::msg::DebugValue msg_;
};

class Init_DebugValue_data
{
public:
  explicit Init_DebugValue_data(::mavros_msgs::msg::DebugValue & msg)
  : msg_(msg)
  {}
  Init_DebugValue_type data(::mavros_msgs::msg::DebugValue::_data_type arg)
  {
    msg_.data = std::move(arg);
    return Init_DebugValue_type(msg_);
  }

private:
  ::mavros_msgs::msg::DebugValue msg_;
};

class Init_DebugValue_value_int
{
public:
  explicit Init_DebugValue_value_int(::mavros_msgs::msg::DebugValue & msg)
  : msg_(msg)
  {}
  Init_DebugValue_data value_int(::mavros_msgs::msg::DebugValue::_value_int_type arg)
  {
    msg_.value_int = std::move(arg);
    return Init_DebugValue_data(msg_);
  }

private:
  ::mavros_msgs::msg::DebugValue msg_;
};

class Init_DebugValue_value_float
{
public:
  explicit Init_DebugValue_value_float(::mavros_msgs::msg::DebugValue & msg)
  : msg_(msg)
  {}
  Init_DebugValue_value_int value_float(::mavros_msgs::msg::DebugValue::_value_float_type arg)
  {
    msg_.value_float = std::move(arg);
    return Init_DebugValue_value_int(msg_);
  }

private:
  ::mavros_msgs::msg::DebugValue msg_;
};

class Init_DebugValue_name
{
public:
  explicit Init_DebugValue_name(::mavros_msgs::msg::DebugValue & msg)
  : msg_(msg)
  {}
  Init_DebugValue_value_float name(::mavros_msgs::msg::DebugValue::_name_type arg)
  {
    msg_.name = std::move(arg);
    return Init_DebugValue_value_float(msg_);
  }

private:
  ::mavros_msgs::msg::DebugValue msg_;
};

class Init_DebugValue_array_id
{
public:
  explicit Init_DebugValue_array_id(::mavros_msgs::msg::DebugValue & msg)
  : msg_(msg)
  {}
  Init_DebugValue_name array_id(::mavros_msgs::msg::DebugValue::_array_id_type arg)
  {
    msg_.array_id = std::move(arg);
    return Init_DebugValue_name(msg_);
  }

private:
  ::mavros_msgs::msg::DebugValue msg_;
};

class Init_DebugValue_index
{
public:
  explicit Init_DebugValue_index(::mavros_msgs::msg::DebugValue & msg)
  : msg_(msg)
  {}
  Init_DebugValue_array_id index(::mavros_msgs::msg::DebugValue::_index_type arg)
  {
    msg_.index = std::move(arg);
    return Init_DebugValue_array_id(msg_);
  }

private:
  ::mavros_msgs::msg::DebugValue msg_;
};

class Init_DebugValue_header
{
public:
  Init_DebugValue_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DebugValue_index header(::mavros_msgs::msg::DebugValue::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_DebugValue_index(msg_);
  }

private:
  ::mavros_msgs::msg::DebugValue msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::msg::DebugValue>()
{
  return mavros_msgs::msg::builder::Init_DebugValue_header();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__DEBUG_VALUE__BUILDER_HPP_
