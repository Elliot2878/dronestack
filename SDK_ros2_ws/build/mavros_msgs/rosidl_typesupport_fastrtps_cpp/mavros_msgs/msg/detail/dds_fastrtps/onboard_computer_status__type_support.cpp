// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from mavros_msgs:msg/OnboardComputerStatus.idl
// generated code does not contain a copyright notice
#include "mavros_msgs/msg/detail/onboard_computer_status__rosidl_typesupport_fastrtps_cpp.hpp"
#include "mavros_msgs/msg/detail/onboard_computer_status__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions
namespace std_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const std_msgs::msg::Header &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  std_msgs::msg::Header &);
size_t get_serialized_size(
  const std_msgs::msg::Header &,
  size_t current_alignment);
size_t
max_serialized_size_Header(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace std_msgs


namespace mavros_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mavros_msgs
cdr_serialize(
  const mavros_msgs::msg::OnboardComputerStatus & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.header,
    cdr);
  // Member: component
  cdr << ros_message.component;
  // Member: uptime
  cdr << ros_message.uptime;
  // Member: type
  cdr << ros_message.type;
  // Member: cpu_cores
  {
    cdr << ros_message.cpu_cores;
  }
  // Member: cpu_combined
  {
    cdr << ros_message.cpu_combined;
  }
  // Member: gpu_cores
  {
    cdr << ros_message.gpu_cores;
  }
  // Member: gpu_combined
  {
    cdr << ros_message.gpu_combined;
  }
  // Member: temperature_board
  cdr << ros_message.temperature_board;
  // Member: temperature_core
  {
    cdr << ros_message.temperature_core;
  }
  // Member: fan_speed
  {
    cdr << ros_message.fan_speed;
  }
  // Member: ram_usage
  cdr << ros_message.ram_usage;
  // Member: ram_total
  cdr << ros_message.ram_total;
  // Member: storage_type
  {
    cdr << ros_message.storage_type;
  }
  // Member: storage_usage
  {
    cdr << ros_message.storage_usage;
  }
  // Member: storage_total
  {
    cdr << ros_message.storage_total;
  }
  // Member: link_type
  {
    cdr << ros_message.link_type;
  }
  // Member: link_tx_rate
  {
    cdr << ros_message.link_tx_rate;
  }
  // Member: link_rx_rate
  {
    cdr << ros_message.link_rx_rate;
  }
  // Member: link_tx_max
  {
    cdr << ros_message.link_tx_max;
  }
  // Member: link_rx_max
  {
    cdr << ros_message.link_rx_max;
  }
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mavros_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  mavros_msgs::msg::OnboardComputerStatus & ros_message)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.header);

  // Member: component
  cdr >> ros_message.component;

  // Member: uptime
  cdr >> ros_message.uptime;

  // Member: type
  cdr >> ros_message.type;

  // Member: cpu_cores
  {
    cdr >> ros_message.cpu_cores;
  }

  // Member: cpu_combined
  {
    cdr >> ros_message.cpu_combined;
  }

  // Member: gpu_cores
  {
    cdr >> ros_message.gpu_cores;
  }

  // Member: gpu_combined
  {
    cdr >> ros_message.gpu_combined;
  }

  // Member: temperature_board
  cdr >> ros_message.temperature_board;

  // Member: temperature_core
  {
    cdr >> ros_message.temperature_core;
  }

  // Member: fan_speed
  {
    cdr >> ros_message.fan_speed;
  }

  // Member: ram_usage
  cdr >> ros_message.ram_usage;

  // Member: ram_total
  cdr >> ros_message.ram_total;

  // Member: storage_type
  {
    cdr >> ros_message.storage_type;
  }

  // Member: storage_usage
  {
    cdr >> ros_message.storage_usage;
  }

  // Member: storage_total
  {
    cdr >> ros_message.storage_total;
  }

  // Member: link_type
  {
    cdr >> ros_message.link_type;
  }

  // Member: link_tx_rate
  {
    cdr >> ros_message.link_tx_rate;
  }

  // Member: link_rx_rate
  {
    cdr >> ros_message.link_rx_rate;
  }

  // Member: link_tx_max
  {
    cdr >> ros_message.link_tx_max;
  }

  // Member: link_rx_max
  {
    cdr >> ros_message.link_rx_max;
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mavros_msgs
get_serialized_size(
  const mavros_msgs::msg::OnboardComputerStatus & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: header

  current_alignment +=
    std_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.header, current_alignment);
  // Member: component
  {
    size_t item_size = sizeof(ros_message.component);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: uptime
  {
    size_t item_size = sizeof(ros_message.uptime);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: type
  {
    size_t item_size = sizeof(ros_message.type);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: cpu_cores
  {
    size_t array_size = 8;
    size_t item_size = sizeof(ros_message.cpu_cores[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: cpu_combined
  {
    size_t array_size = 10;
    size_t item_size = sizeof(ros_message.cpu_combined[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: gpu_cores
  {
    size_t array_size = 4;
    size_t item_size = sizeof(ros_message.gpu_cores[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: gpu_combined
  {
    size_t array_size = 10;
    size_t item_size = sizeof(ros_message.gpu_combined[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: temperature_board
  {
    size_t item_size = sizeof(ros_message.temperature_board);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: temperature_core
  {
    size_t array_size = 8;
    size_t item_size = sizeof(ros_message.temperature_core[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: fan_speed
  {
    size_t array_size = 4;
    size_t item_size = sizeof(ros_message.fan_speed[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: ram_usage
  {
    size_t item_size = sizeof(ros_message.ram_usage);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: ram_total
  {
    size_t item_size = sizeof(ros_message.ram_total);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: storage_type
  {
    size_t array_size = 4;
    size_t item_size = sizeof(ros_message.storage_type[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: storage_usage
  {
    size_t array_size = 4;
    size_t item_size = sizeof(ros_message.storage_usage[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: storage_total
  {
    size_t array_size = 4;
    size_t item_size = sizeof(ros_message.storage_total[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: link_type
  {
    size_t array_size = 6;
    size_t item_size = sizeof(ros_message.link_type[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: link_tx_rate
  {
    size_t array_size = 6;
    size_t item_size = sizeof(ros_message.link_tx_rate[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: link_rx_rate
  {
    size_t array_size = 6;
    size_t item_size = sizeof(ros_message.link_rx_rate[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: link_tx_max
  {
    size_t array_size = 6;
    size_t item_size = sizeof(ros_message.link_tx_max[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: link_rx_max
  {
    size_t array_size = 6;
    size_t item_size = sizeof(ros_message.link_rx_max[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mavros_msgs
max_serialized_size_OnboardComputerStatus(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: header
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        std_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Header(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: component
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: uptime
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: type
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: cpu_cores
  {
    size_t array_size = 8;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: cpu_combined
  {
    size_t array_size = 10;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: gpu_cores
  {
    size_t array_size = 4;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: gpu_combined
  {
    size_t array_size = 10;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: temperature_board
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: temperature_core
  {
    size_t array_size = 8;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: fan_speed
  {
    size_t array_size = 4;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: ram_usage
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: ram_total
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: storage_type
  {
    size_t array_size = 4;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: storage_usage
  {
    size_t array_size = 4;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: storage_total
  {
    size_t array_size = 4;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: link_type
  {
    size_t array_size = 6;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: link_tx_rate
  {
    size_t array_size = 6;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: link_rx_rate
  {
    size_t array_size = 6;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: link_tx_max
  {
    size_t array_size = 6;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: link_rx_max
  {
    size_t array_size = 6;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = mavros_msgs::msg::OnboardComputerStatus;
    is_plain =
      (
      offsetof(DataType, link_rx_max) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _OnboardComputerStatus__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const mavros_msgs::msg::OnboardComputerStatus *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _OnboardComputerStatus__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<mavros_msgs::msg::OnboardComputerStatus *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _OnboardComputerStatus__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const mavros_msgs::msg::OnboardComputerStatus *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _OnboardComputerStatus__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_OnboardComputerStatus(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _OnboardComputerStatus__callbacks = {
  "mavros_msgs::msg",
  "OnboardComputerStatus",
  _OnboardComputerStatus__cdr_serialize,
  _OnboardComputerStatus__cdr_deserialize,
  _OnboardComputerStatus__get_serialized_size,
  _OnboardComputerStatus__max_serialized_size
};

static rosidl_message_type_support_t _OnboardComputerStatus__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_OnboardComputerStatus__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace mavros_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_mavros_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<mavros_msgs::msg::OnboardComputerStatus>()
{
  return &mavros_msgs::msg::typesupport_fastrtps_cpp::_OnboardComputerStatus__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, mavros_msgs, msg, OnboardComputerStatus)() {
  return &mavros_msgs::msg::typesupport_fastrtps_cpp::_OnboardComputerStatus__handle;
}

#ifdef __cplusplus
}
#endif
