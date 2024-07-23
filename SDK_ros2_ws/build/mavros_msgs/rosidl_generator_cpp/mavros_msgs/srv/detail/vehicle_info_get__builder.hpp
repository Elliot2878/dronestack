// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mavros_msgs:srv/VehicleInfoGet.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__SRV__DETAIL__VEHICLE_INFO_GET__BUILDER_HPP_
#define MAVROS_MSGS__SRV__DETAIL__VEHICLE_INFO_GET__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mavros_msgs/srv/detail/vehicle_info_get__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mavros_msgs
{

namespace srv
{

namespace builder
{

class Init_VehicleInfoGet_Request_get_all
{
public:
  explicit Init_VehicleInfoGet_Request_get_all(::mavros_msgs::srv::VehicleInfoGet_Request & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::srv::VehicleInfoGet_Request get_all(::mavros_msgs::srv::VehicleInfoGet_Request::_get_all_type arg)
  {
    msg_.get_all = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::srv::VehicleInfoGet_Request msg_;
};

class Init_VehicleInfoGet_Request_compid
{
public:
  explicit Init_VehicleInfoGet_Request_compid(::mavros_msgs::srv::VehicleInfoGet_Request & msg)
  : msg_(msg)
  {}
  Init_VehicleInfoGet_Request_get_all compid(::mavros_msgs::srv::VehicleInfoGet_Request::_compid_type arg)
  {
    msg_.compid = std::move(arg);
    return Init_VehicleInfoGet_Request_get_all(msg_);
  }

private:
  ::mavros_msgs::srv::VehicleInfoGet_Request msg_;
};

class Init_VehicleInfoGet_Request_sysid
{
public:
  Init_VehicleInfoGet_Request_sysid()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_VehicleInfoGet_Request_compid sysid(::mavros_msgs::srv::VehicleInfoGet_Request::_sysid_type arg)
  {
    msg_.sysid = std::move(arg);
    return Init_VehicleInfoGet_Request_compid(msg_);
  }

private:
  ::mavros_msgs::srv::VehicleInfoGet_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::srv::VehicleInfoGet_Request>()
{
  return mavros_msgs::srv::builder::Init_VehicleInfoGet_Request_sysid();
}

}  // namespace mavros_msgs


namespace mavros_msgs
{

namespace srv
{

namespace builder
{

class Init_VehicleInfoGet_Response_vehicles
{
public:
  explicit Init_VehicleInfoGet_Response_vehicles(::mavros_msgs::srv::VehicleInfoGet_Response & msg)
  : msg_(msg)
  {}
  ::mavros_msgs::srv::VehicleInfoGet_Response vehicles(::mavros_msgs::srv::VehicleInfoGet_Response::_vehicles_type arg)
  {
    msg_.vehicles = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mavros_msgs::srv::VehicleInfoGet_Response msg_;
};

class Init_VehicleInfoGet_Response_success
{
public:
  Init_VehicleInfoGet_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_VehicleInfoGet_Response_vehicles success(::mavros_msgs::srv::VehicleInfoGet_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_VehicleInfoGet_Response_vehicles(msg_);
  }

private:
  ::mavros_msgs::srv::VehicleInfoGet_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mavros_msgs::srv::VehicleInfoGet_Response>()
{
  return mavros_msgs::srv::builder::Init_VehicleInfoGet_Response_success();
}

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__SRV__DETAIL__VEHICLE_INFO_GET__BUILDER_HPP_
