// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from mavros_msgs:msg/GimbalDeviceInformation.idl
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
#include "mavros_msgs/msg/detail/gimbal_device_information__struct.h"
#include "mavros_msgs/msg/detail/gimbal_device_information__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool mavros_msgs__msg__gimbal_device_information__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[67];
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
    assert(strncmp("mavros_msgs.msg._gimbal_device_information.GimbalDeviceInformation", full_classname_dest, 66) == 0);
  }
  mavros_msgs__msg__GimbalDeviceInformation * ros_message = _ros_message;
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
  {  // vendor_name
    PyObject * field = PyObject_GetAttrString(_pymsg, "vendor_name");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->vendor_name, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // model_name
    PyObject * field = PyObject_GetAttrString(_pymsg, "model_name");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->model_name, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // custom_name
    PyObject * field = PyObject_GetAttrString(_pymsg, "custom_name");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->custom_name, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // firmware_version
    PyObject * field = PyObject_GetAttrString(_pymsg, "firmware_version");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->firmware_version = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // hardware_version
    PyObject * field = PyObject_GetAttrString(_pymsg, "hardware_version");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->hardware_version = PyLong_AsUnsignedLong(field);
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
  {  // cap_flags
    PyObject * field = PyObject_GetAttrString(_pymsg, "cap_flags");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->cap_flags = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // custom_cap_flags
    PyObject * field = PyObject_GetAttrString(_pymsg, "custom_cap_flags");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->custom_cap_flags = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // roll_min
    PyObject * field = PyObject_GetAttrString(_pymsg, "roll_min");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->roll_min = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // roll_max
    PyObject * field = PyObject_GetAttrString(_pymsg, "roll_max");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->roll_max = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // pitch_min
    PyObject * field = PyObject_GetAttrString(_pymsg, "pitch_min");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->pitch_min = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // pitch_max
    PyObject * field = PyObject_GetAttrString(_pymsg, "pitch_max");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->pitch_max = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // yaw_min
    PyObject * field = PyObject_GetAttrString(_pymsg, "yaw_min");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->yaw_min = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // yaw_max
    PyObject * field = PyObject_GetAttrString(_pymsg, "yaw_max");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->yaw_max = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * mavros_msgs__msg__gimbal_device_information__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of GimbalDeviceInformation */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("mavros_msgs.msg._gimbal_device_information");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "GimbalDeviceInformation");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  mavros_msgs__msg__GimbalDeviceInformation * ros_message = (mavros_msgs__msg__GimbalDeviceInformation *)raw_ros_message;
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
  {  // vendor_name
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->vendor_name.data,
      strlen(ros_message->vendor_name.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "vendor_name", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // model_name
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->model_name.data,
      strlen(ros_message->model_name.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "model_name", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // custom_name
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->custom_name.data,
      strlen(ros_message->custom_name.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "custom_name", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // firmware_version
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->firmware_version);
    {
      int rc = PyObject_SetAttrString(_pymessage, "firmware_version", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // hardware_version
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->hardware_version);
    {
      int rc = PyObject_SetAttrString(_pymessage, "hardware_version", field);
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
  {  // cap_flags
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->cap_flags);
    {
      int rc = PyObject_SetAttrString(_pymessage, "cap_flags", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // custom_cap_flags
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->custom_cap_flags);
    {
      int rc = PyObject_SetAttrString(_pymessage, "custom_cap_flags", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // roll_min
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->roll_min);
    {
      int rc = PyObject_SetAttrString(_pymessage, "roll_min", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // roll_max
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->roll_max);
    {
      int rc = PyObject_SetAttrString(_pymessage, "roll_max", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pitch_min
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->pitch_min);
    {
      int rc = PyObject_SetAttrString(_pymessage, "pitch_min", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pitch_max
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->pitch_max);
    {
      int rc = PyObject_SetAttrString(_pymessage, "pitch_max", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // yaw_min
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->yaw_min);
    {
      int rc = PyObject_SetAttrString(_pymessage, "yaw_min", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // yaw_max
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->yaw_max);
    {
      int rc = PyObject_SetAttrString(_pymessage, "yaw_max", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
