// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from mavros_msgs:msg/SysStatus.idl
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
#include "mavros_msgs/msg/detail/sys_status__struct.h"
#include "mavros_msgs/msg/detail/sys_status__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool mavros_msgs__msg__sys_status__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[38];
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
    assert(strncmp("mavros_msgs.msg._sys_status.SysStatus", full_classname_dest, 37) == 0);
  }
  mavros_msgs__msg__SysStatus * ros_message = _ros_message;
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
  {  // sensors_present
    PyObject * field = PyObject_GetAttrString(_pymsg, "sensors_present");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->sensors_present = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // sensors_enabled
    PyObject * field = PyObject_GetAttrString(_pymsg, "sensors_enabled");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->sensors_enabled = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // sensors_health
    PyObject * field = PyObject_GetAttrString(_pymsg, "sensors_health");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->sensors_health = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // load
    PyObject * field = PyObject_GetAttrString(_pymsg, "load");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->load = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // voltage_battery
    PyObject * field = PyObject_GetAttrString(_pymsg, "voltage_battery");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->voltage_battery = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // current_battery
    PyObject * field = PyObject_GetAttrString(_pymsg, "current_battery");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->current_battery = (int16_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // battery_remaining
    PyObject * field = PyObject_GetAttrString(_pymsg, "battery_remaining");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->battery_remaining = (int8_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // drop_rate_comm
    PyObject * field = PyObject_GetAttrString(_pymsg, "drop_rate_comm");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->drop_rate_comm = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // errors_comm
    PyObject * field = PyObject_GetAttrString(_pymsg, "errors_comm");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->errors_comm = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // errors_count1
    PyObject * field = PyObject_GetAttrString(_pymsg, "errors_count1");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->errors_count1 = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // errors_count2
    PyObject * field = PyObject_GetAttrString(_pymsg, "errors_count2");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->errors_count2 = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // errors_count3
    PyObject * field = PyObject_GetAttrString(_pymsg, "errors_count3");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->errors_count3 = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // errors_count4
    PyObject * field = PyObject_GetAttrString(_pymsg, "errors_count4");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->errors_count4 = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * mavros_msgs__msg__sys_status__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of SysStatus */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("mavros_msgs.msg._sys_status");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "SysStatus");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  mavros_msgs__msg__SysStatus * ros_message = (mavros_msgs__msg__SysStatus *)raw_ros_message;
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
  {  // sensors_present
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->sensors_present);
    {
      int rc = PyObject_SetAttrString(_pymessage, "sensors_present", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // sensors_enabled
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->sensors_enabled);
    {
      int rc = PyObject_SetAttrString(_pymessage, "sensors_enabled", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // sensors_health
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->sensors_health);
    {
      int rc = PyObject_SetAttrString(_pymessage, "sensors_health", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // load
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->load);
    {
      int rc = PyObject_SetAttrString(_pymessage, "load", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // voltage_battery
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->voltage_battery);
    {
      int rc = PyObject_SetAttrString(_pymessage, "voltage_battery", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // current_battery
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->current_battery);
    {
      int rc = PyObject_SetAttrString(_pymessage, "current_battery", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // battery_remaining
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->battery_remaining);
    {
      int rc = PyObject_SetAttrString(_pymessage, "battery_remaining", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // drop_rate_comm
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->drop_rate_comm);
    {
      int rc = PyObject_SetAttrString(_pymessage, "drop_rate_comm", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // errors_comm
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->errors_comm);
    {
      int rc = PyObject_SetAttrString(_pymessage, "errors_comm", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // errors_count1
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->errors_count1);
    {
      int rc = PyObject_SetAttrString(_pymessage, "errors_count1", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // errors_count2
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->errors_count2);
    {
      int rc = PyObject_SetAttrString(_pymessage, "errors_count2", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // errors_count3
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->errors_count3);
    {
      int rc = PyObject_SetAttrString(_pymessage, "errors_count3", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // errors_count4
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->errors_count4);
    {
      int rc = PyObject_SetAttrString(_pymessage, "errors_count4", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
