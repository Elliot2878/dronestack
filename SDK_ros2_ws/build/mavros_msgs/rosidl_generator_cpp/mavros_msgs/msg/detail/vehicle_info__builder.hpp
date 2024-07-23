// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:msg/VehicleInfo.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__VEHICLE_INFO__BUILDER_HPP_
#define MAVROS_MSGS__MSG__DETAIL__VEHICLE_INFO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/msg/detail/vehicle_info__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace msg
{

namespace builder
{

class Init_VehicleInfo_uid
{
public:
  explicit Init_VehicleInfo_uid(::mavros_msgs::msg::VehicleInfo & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::msg::VehicleInfo uid(::mavros_msgs::msg::VehicleInfo::_uid_type arg)
  {
    msg_.uid = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::msg::VehicleInfo msg_;
};

class Init_VehicleInfo_product_id
{
public:
  explicit Init_VehicleInfo_product_id(::mavros_msgs::msg::VehicleInfo & msg)
  : msg_(msg)
  {}
  Init_VehicleInfo_uid product_id(::mavros_msgs::msg::VehicleInfo::_product_id_type arg)
  {
    msg_.product_id = std::move(arg);
    return Init_VehicleInfo_uid(msg_);
  }

private:
  ::mavros_msgs::msg::VehicleInfo msg_;
};

class Init_VehicleInfo_vendor_id
{
public:
  explicit Init_VehicleInfo_vendor_id(::mavros_msgs::msg::VehicleInfo & msg)
  : msg_(msg)
  {}
  Init_VehicleInfo_product_id vendor_id(::mavros_msgs::msg::VehicleInfo::_vendor_id_type arg)
  {
    msg_.vendor_id = std::move(arg);
    return Init_VehicleInfo_product_id(msg_);
  }

private:
  ::mavros_msgs::msg::VehicleInfo msg_;
};

class Init_VehicleInfo_flight_custom_version
{
public:
  explicit Init_VehicleInfo_flight_custom_version(::mavros_msgs::msg::VehicleInfo & msg)
  : msg_(msg)
  {}
  Init_VehicleInfo_vendor_id flight_custom_version(::mavros_msgs::msg::VehicleInfo::_flight_custom_version_type arg)
  {
    msg_.flight_custom_version = std::move(arg);
    return Init_VehicleInfo_vendor_id(msg_);
  }

private:
  ::mavros_msgs::msg::VehicleInfo msg_;
};

class Init_VehicleInfo_board_version
{
public:
  explicit Init_VehicleInfo_board_version(::mavros_msgs::msg::VehicleInfo & msg)
  : msg_(msg)
  {}
  Init_VehicleInfo_flight_custom_version board_version(::mavros_msgs::msg::VehicleInfo::_board_version_type arg)
  {
    msg_.board_version = std::move(arg);
    return Init_VehicleInfo_flight_custom_version(msg_);
  }

private:
  ::mavros_msgs::msg::VehicleInfo msg_;
};

class Init_VehicleInfo_os_sw_version
{
public:
  explicit Init_VehicleInfo_os_sw_version(::mavros_msgs::msg::VehicleInfo & msg)
  : msg_(msg)
  {}
  Init_VehicleInfo_board_version os_sw_version(::mavros_msgs::msg::VehicleInfo::_os_sw_version_type arg)
  {
    msg_.os_sw_version = std::move(arg);
    return Init_VehicleInfo_board_version(msg_);
  }

private:
  ::mavros_msgs::msg::VehicleInfo msg_;
};

class Init_VehicleInfo_middleware_sw_version
{
public:
  explicit Init_VehicleInfo_middleware_sw_version(::mavros_msgs::msg::VehicleInfo & msg)
  : msg_(msg)
  {}
  Init_VehicleInfo_os_sw_version middleware_sw_version(::mavros_msgs::msg::VehicleInfo::_middleware_sw_version_type arg)
  {
    msg_.middleware_sw_version = std::move(arg);
    return Init_VehicleInfo_os_sw_version(msg_);
  }

private:
  ::mavros_msgs::msg::VehicleInfo msg_;
};

class Init_VehicleInfo_flight_sw_version
{
public:
  explicit Init_VehicleInfo_flight_sw_version(::mavros_msgs::msg::VehicleInfo & msg)
  : msg_(msg)
  {}
  Init_VehicleInfo_middleware_sw_version flight_sw_version(::mavros_msgs::msg::VehicleInfo::_flight_sw_version_type arg)
  {
    msg_.flight_sw_version = std::move(arg);
    return Init_VehicleInfo_middleware_sw_version(msg_);
  }

private:
  ::mavros_msgs::msg::VehicleInfo msg_;
};

class Init_VehicleInfo_capabilities
{
public:
  explicit Init_VehicleInfo_capabilities(::mavros_msgs::msg::VehicleInfo & msg)
  : msg_(msg)
  {}
  Init_VehicleInfo_flight_sw_version capabilities(::mavros_msgs::msg::VehicleInfo::_capabilities_type arg)
  {
    msg_.capabilities = std::move(arg);
    return Init_VehicleInfo_flight_sw_version(msg_);
  }

private:
  ::mavros_msgs::msg::VehicleInfo msg_;
};

class Init_VehicleInfo_mode_id
{
public:
  explicit Init_VehicleInfo_mode_id(::mavros_msgs::msg::VehicleInfo & msg)
  : msg_(msg)
  {}
  Init_VehicleInfo_capabilities mode_id(::mavros_msgs::msg::VehicleInfo::_mode_id_type arg)
  {
    msg_.mode_id = std::move(arg);
    return Init_VehicleInfo_capabilities(msg_);
  }

private:
  ::mavros_msgs::msg::VehicleInfo msg_;
};

class Init_VehicleInfo_mode
{
public:
  explicit Init_VehicleInfo_mode(::mavros_msgs::msg::VehicleInfo & msg)
  : msg_(msg)
  {}
  Init_VehicleInfo_mode_id mode(::mavros_msgs::msg::VehicleInfo::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return Init_VehicleInfo_mode_id(msg_);
  }

private:
  ::mavros_msgs::msg::VehicleInfo msg_;
};

class Init_VehicleInfo_custom_mode
{
public:
  explicit Init_VehicleInfo_custom_mode(::mavros_msgs::msg::VehicleInfo & msg)
  : msg_(msg)
  {}
  Init_VehicleInfo_mode custom_mode(::mavros_msgs::msg::VehicleInfo::_custom_mode_type arg)
  {
    msg_.custom_mode = std::move(arg);
    return Init_VehicleInfo_mode(msg_);
  }

private:
  ::mavros_msgs::msg::VehicleInfo msg_;
};

class Init_VehicleInfo_base_mode
{
public:
  explicit Init_VehicleInfo_base_mode(::mavros_msgs::msg::VehicleInfo & msg)
  : msg_(msg)
  {}
  Init_VehicleInfo_custom_mode base_mode(::mavros_msgs::msg::VehicleInfo::_base_mode_type arg)
  {
    msg_.base_mode = std::move(arg);
    return Init_VehicleInfo_custom_mode(msg_);
  }

private:
  ::mavros_msgs::msg::VehicleInfo msg_;
};

class Init_VehicleInfo_system_status
{
public:
  explicit Init_VehicleInfo_system_status(::mavros_msgs::msg::VehicleInfo & msg)
  : msg_(msg)
  {}
  Init_VehicleInfo_base_mode system_status(::mavros_msgs::msg::VehicleInfo::_system_status_type arg)
  {
    msg_.system_status = std::move(arg);
    return Init_VehicleInfo_base_mode(msg_);
  }

private:
  ::mavros_msgs::msg::VehicleInfo msg_;
};

class Init_VehicleInfo_type
{
public:
  explicit Init_VehicleInfo_type(::mavros_msgs::msg::VehicleInfo & msg)
  : msg_(msg)
  {}
  Init_VehicleInfo_system_status type(::mavros_msgs::msg::VehicleInfo::_type_type arg)
  {
    msg_.type = std::move(arg);
    return Init_VehicleInfo_system_status(msg_);
  }

private:
  ::mavros_msgs::msg::VehicleInfo msg_;
};

class Init_VehicleInfo_autopilot
{
public:
  explicit Init_VehicleInfo_autopilot(::mavros_msgs::msg::VehicleInfo & msg)
  : msg_(msg)
  {}
  Init_VehicleInfo_type autopilot(::mavros_msgs::msg::VehicleInfo::_autopilot_type arg)
  {
    msg_.autopilot = std::move(arg);
    return Init_VehicleInfo_type(msg_);
  }

private:
  ::mavros_msgs::msg::VehicleInfo msg_;
};

class Init_VehicleInfo_compid
{
public:
  explicit Init_VehicleInfo_compid(::mavros_msgs::msg::VehicleInfo & msg)
  : msg_(msg)
  {}
  Init_VehicleInfo_autopilot compid(::mavros_msgs::msg::VehicleInfo::_compid_type arg)
  {
    msg_.compid = std::move(arg);
    return Init_VehicleInfo_autopilot(msg_);
  }

private:
  ::mavros_msgs::msg::VehicleInfo msg_;
};

class Init_VehicleInfo_sysid
{
public:
  explicit Init_VehicleInfo_sysid(::mavros_msgs::msg::VehicleInfo & msg)
  : msg_(msg)
  {}
  Init_VehicleInfo_compid sysid(::mavros_msgs::msg::VehicleInfo::_sysid_type arg)
  {
    msg_.sysid = std::move(arg);
    return Init_VehicleInfo_compid(msg_);
  }

private:
  ::mavros_msgs::msg::VehicleInfo msg_;
};

class Init_VehicleInfo_available_info
{
public:
  explicit Init_VehicleInfo_available_info(::mavros_msgs::msg::VehicleInfo & msg)
  : msg_(msg)
  {}
  Init_VehicleInfo_sysid available_info(::mavros_msgs::msg::VehicleInfo::_available_info_type arg)
  {
    msg_.available_info = std::move(arg);
    return Init_VehicleInfo_sysid(msg_);
  }

private:
  ::mavros_msgs::msg::VehicleInfo msg_;
};

class Init_VehicleInfo_header
{
public:
  Init_VehicleInfo_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_VehicleInfo_available_info header(::mavros_msgs::msg::VehicleInfo::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_VehicleInfo_available_info(msg_);
  }

private:
  ::mavros_msgs::msg::VehicleInfo msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::msg::VehicleInfo>()
{
  return mavros_msgs::msg::builder::Init_VehicleInfo_header();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__VEHICLE_INFO__BUILDER_HPP_
