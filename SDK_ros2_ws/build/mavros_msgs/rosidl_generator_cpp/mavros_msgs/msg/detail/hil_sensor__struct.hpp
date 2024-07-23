// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mavros_msgs:msg/HilSensor.idl
// generated code does not contain a copyright notice

#ifndef MAVROS_MSGS__MSG__DETAIL__HIL_SENSOR__STRUCT_HPP_
#define MAVROS_MSGS__MSG__DETAIL__HIL_SENSOR__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'acc'
// Member 'gyro'
// Member 'mag'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mavros_msgs__msg__HilSensor __attribute__((deprecated))
#else
# define DEPRECATED__mavros_msgs__msg__HilSensor __declspec(deprecated)
#endif

namespace mavros_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct HilSensor_
{
  using Type = HilSensor_<ContainerAllocator>;

  explicit HilSensor_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    acc(_init),
    gyro(_init),
    mag(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->abs_pressure = 0.0f;
      this->diff_pressure = 0.0f;
      this->pressure_alt = 0.0f;
      this->temperature = 0.0f;
      this->fields_updated = 0ul;
    }
  }

  explicit HilSensor_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    acc(_alloc, _init),
    gyro(_alloc, _init),
    mag(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->abs_pressure = 0.0f;
      this->diff_pressure = 0.0f;
      this->pressure_alt = 0.0f;
      this->temperature = 0.0f;
      this->fields_updated = 0ul;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _acc_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _acc_type acc;
  using _gyro_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _gyro_type gyro;
  using _mag_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _mag_type mag;
  using _abs_pressure_type =
    float;
  _abs_pressure_type abs_pressure;
  using _diff_pressure_type =
    float;
  _diff_pressure_type diff_pressure;
  using _pressure_alt_type =
    float;
  _pressure_alt_type pressure_alt;
  using _temperature_type =
    float;
  _temperature_type temperature;
  using _fields_updated_type =
    uint32_t;
  _fields_updated_type fields_updated;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__acc(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->acc = _arg;
    return *this;
  }
  Type & set__gyro(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->gyro = _arg;
    return *this;
  }
  Type & set__mag(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->mag = _arg;
    return *this;
  }
  Type & set__abs_pressure(
    const float & _arg)
  {
    this->abs_pressure = _arg;
    return *this;
  }
  Type & set__diff_pressure(
    const float & _arg)
  {
    this->diff_pressure = _arg;
    return *this;
  }
  Type & set__pressure_alt(
    const float & _arg)
  {
    this->pressure_alt = _arg;
    return *this;
  }
  Type & set__temperature(
    const float & _arg)
  {
    this->temperature = _arg;
    return *this;
  }
  Type & set__fields_updated(
    const uint32_t & _arg)
  {
    this->fields_updated = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mavros_msgs::msg::HilSensor_<ContainerAllocator> *;
  using ConstRawPtr =
    const mavros_msgs::msg::HilSensor_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mavros_msgs::msg::HilSensor_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mavros_msgs::msg::HilSensor_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::HilSensor_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::HilSensor_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mavros_msgs::msg::HilSensor_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mavros_msgs::msg::HilSensor_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mavros_msgs::msg::HilSensor_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mavros_msgs::msg::HilSensor_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mavros_msgs__msg__HilSensor
    std::shared_ptr<mavros_msgs::msg::HilSensor_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mavros_msgs__msg__HilSensor
    std::shared_ptr<mavros_msgs::msg::HilSensor_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const HilSensor_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->acc != other.acc) {
      return false;
    }
    if (this->gyro != other.gyro) {
      return false;
    }
    if (this->mag != other.mag) {
      return false;
    }
    if (this->abs_pressure != other.abs_pressure) {
      return false;
    }
    if (this->diff_pressure != other.diff_pressure) {
      return false;
    }
    if (this->pressure_alt != other.pressure_alt) {
      return false;
    }
    if (this->temperature != other.temperature) {
      return false;
    }
    if (this->fields_updated != other.fields_updated) {
      return false;
    }
    return true;
  }
  bool operator!=(const HilSensor_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct HilSensor_

// alias to use template instance with default allocator
using HilSensor =
  mavros_msgs::msg::HilSensor_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace mavros_msgs

#endif  // MAVROS_MSGS__MSG__DETAIL__HIL_SENSOR__STRUCT_HPP_
