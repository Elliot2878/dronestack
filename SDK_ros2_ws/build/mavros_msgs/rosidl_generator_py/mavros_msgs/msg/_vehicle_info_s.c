// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from mavros_msgs:msg/VehicleInfo.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "mavros_msgs/msg/detail/vehicle_info__struct.h"
#include "mavros_msgs/msg/detail/vehicle_info__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool mavros_msgs__msg__vehicle_info__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[42];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("mavros_msgs.msg._vehicle_info.VehicleInfo", full_classname_dest, 41) == 0);
  }
  mavros_msgs__msg__VehicleInfo * ros_message = _ros_message;
  {  // header
    PyObject * field = PyObject_GetAttrString(_pymsg, "header");
    if (!field) {
      return false;
    }
    if (!std_msgs__msg__header__convert_from_py(field, &ros_message->header)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // available_info
    PyObject * field = PyObject_GetAttrString(_pymsg, "available_info");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->available_info = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // sysid
    PyObject * field = PyObject_GetAttrString(_pymsg, "sysid");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->sysid = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // compid
    PyObject * field = PyObject_GetAttrString(_pymsg, "compid");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->compid = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // autopilot
    PyObject * field = PyObject_GetAttrString(_pymsg, "autopilot");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->autopilot = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // type
    PyObject * field = PyObject_GetAttrString(_pymsg, "type");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->type = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // system_status
    PyObject * field = PyObject_GetAttrString(_pymsg, "system_status");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->system_status = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // base_mode
    PyObject * field = PyObject_GetAttrString(_pymsg, "base_mode");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->base_mode = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // custom_mode
    PyObject * field = PyObject_GetAttrString(_pymsg, "custom_mode");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->custom_mode = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // mode
    PyObject * field = PyObject_GetAttrString(_pymsg, "mode");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->mode, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // mode_id
    PyObject * field = PyObject_GetAttrString(_pymsg, "mode_id");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->mode_id = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // capabilities
    PyObject * field = PyObject_GetAttrString(_pymsg, "capabilities");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->capabilities = PyLong_AsUnsignedLongLong(field);
    Py_DECREF(field);
  }
  {  // flight_sw_version
    PyObject * field = PyObject_GetAttrString(_pymsg, "flight_sw_version");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->flight_sw_version = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // middleware_sw_version
    PyObject * field = PyObject_GetAttrString(_pymsg, "middleware_sw_version");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->middleware_sw_version = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // os_sw_version
    PyObject * field = PyObject_GetAttrString(_pymsg, "os_sw_version");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->os_sw_version = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // board_version
    PyObject * field = PyObject_GetAttrString(_pymsg, "board_version");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->board_version = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // flight_custom_version
    PyObject * field = PyObject_GetAttrString(_pymsg, "flight_custom_version");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->flight_custom_version, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // vendor_id
    PyObject * field = PyObject_GetAttrString(_pymsg, "vendor_id");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->vendor_id = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // product_id
    PyObject * field = PyObject_GetAttrString(_pymsg, "product_id");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->product_id = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // uid
    PyObject * field = PyObject_GetAttrString(_pymsg, "uid");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->uid = PyLong_AsUnsignedLongLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * mavros_msgs__msg__vehicle_info__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of VehicleInfo */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("mavros_msgs.msg._vehicle_info");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "VehicleInfo");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  mavros_msgs__msg__VehicleInfo * ros_message = (mavros_msgs__msg__VehicleInfo *)raw_ros_message;
  {  // header
    PyObject * field = NULL;
    field = std_msgs__msg__header__convert_to_py(&ros_message->header);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "header", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // available_info
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->available_info);
    {
      int rc = PyObject_SetAttrString(_pymessage, "available_info", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // sysid
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->sysid);
    {
      int rc = PyObject_SetAttrString(_pymessage, "sysid", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // compid
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->compid);
    {
      int rc = PyObject_SetAttrString(_pymessage, "compid", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // autopilot
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->autopilot);
    {
      int rc = PyObject_SetAttrString(_pymessage, "autopilot", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // type
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->type);
    {
      int rc = PyObject_SetAttrString(_pymessage, "type", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // system_status
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->system_status);
    {
      int rc = PyObject_SetAttrString(_pymessage, "system_status", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // base_mode
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->base_mode);
    {
      int rc = PyObject_SetAttrString(_pymessage, "base_mode", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // custom_mode
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->custom_mode);
    {
      int rc = PyObject_SetAttrString(_pymessage, "custom_mode", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // mode
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->mode.data,
      strlen(ros_message->mode.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "mode", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // mode_id
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->mode_id);
    {
      int rc = PyObject_SetAttrString(_pymessage, "mode_id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // capabilities
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLongLong(ros_message->capabilities);
    {
      int rc = PyObject_SetAttrString(_pymessage, "capabilities", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // flight_sw_version
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->flight_sw_version);
    {
      int rc = PyObject_SetAttrString(_pymessage, "flight_sw_version", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // middleware_sw_version
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->middleware_sw_version);
    {
      int rc = PyObject_SetAttrString(_pymessage, "middleware_sw_version", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // os_sw_version
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->os_sw_version);
    {
      int rc = PyObject_SetAttrString(_pymessage, "os_sw_version", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // board_version
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->board_version);
    {
      int rc = PyObject_SetAttrString(_pymessage, "board_version", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // flight_custom_version
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->flight_custom_version.data,
      strlen(ros_message->flight_custom_version.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "flight_custom_version", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // vendor_id
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->vendor_id);
    {
      int rc = PyObject_SetAttrString(_pymessage, "vendor_id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // product_id
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->product_id);
    {
      int rc = PyObject_SetAttrString(_pymessage, "product_id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // uid
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLongLong(ros_message->uid);
    {
      int rc = PyObject_SetAttrString(_pymessage, "uid", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
