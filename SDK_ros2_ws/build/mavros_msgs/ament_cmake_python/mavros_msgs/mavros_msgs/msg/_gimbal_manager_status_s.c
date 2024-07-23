// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from mavros_msgs:msg/GimbalManagerStatus.idl
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
#include "mavros_msgs/msg/detail/gimbal_manager_status__struct.h"
#include "mavros_msgs/msg/detail/gimbal_manager_status__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool mavros_msgs__msg__gimbal_manager_status__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[59];
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
    assert(strncmp("mavros_msgs.msg._gimbal_manager_status.GimbalManagerStatus", full_classname_dest, 58) == 0);
  }
  mavros_msgs__msg__GimbalManagerStatus * ros_message = _ros_message;
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
  {  // flags
    PyObject * field = PyObject_GetAttrString(_pymsg, "flags");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->flags = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // gimbal_device_id
    PyObject * field = PyObject_GetAttrString(_pymsg, "gimbal_device_id");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->gimbal_device_id = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // sysid_primary
    PyObject * field = PyObject_GetAttrString(_pymsg, "sysid_primary");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->sysid_primary = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // compid_primary
    PyObject * field = PyObject_GetAttrString(_pymsg, "compid_primary");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->compid_primary = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // sysid_secondary
    PyObject * field = PyObject_GetAttrString(_pymsg, "sysid_secondary");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->sysid_secondary = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // compid_secondary
    PyObject * field = PyObject_GetAttrString(_pymsg, "compid_secondary");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->compid_secondary = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * mavros_msgs__msg__gimbal_manager_status__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of GimbalManagerStatus */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("mavros_msgs.msg._gimbal_manager_status");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "GimbalManagerStatus");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  mavros_msgs__msg__GimbalManagerStatus * ros_message = (mavros_msgs__msg__GimbalManagerStatus *)raw_ros_message;
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
  {  // flags
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->flags);
    {
      int rc = PyObject_SetAttrString(_pymessage, "flags", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // gimbal_device_id
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->gimbal_device_id);
    {
      int rc = PyObject_SetAttrString(_pymessage, "gimbal_device_id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // sysid_primary
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->sysid_primary);
    {
      int rc = PyObject_SetAttrString(_pymessage, "sysid_primary", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // compid_primary
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->compid_primary);
    {
      int rc = PyObject_SetAttrString(_pymessage, "compid_primary", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // sysid_secondary
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->sysid_secondary);
    {
      int rc = PyObject_SetAttrString(_pymessage, "sysid_secondary", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // compid_secondary
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->compid_secondary);
    {
      int rc = PyObject_SetAttrString(_pymessage, "compid_secondary", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
