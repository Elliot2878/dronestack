// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from mavros_msgs:msg/TerrainReport.idl
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
#include "mavros_msgs/msg/detail/terrain_report__struct.h"
#include "mavros_msgs/msg/detail/terrain_report__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool mavros_msgs__msg__terrain_report__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[46];
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
    assert(strncmp("mavros_msgs.msg._terrain_report.TerrainReport", full_classname_dest, 45) == 0);
  }
  mavros_msgs__msg__TerrainReport * ros_message = _ros_message;
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
  {  // latitude
    PyObject * field = PyObject_GetAttrString(_pymsg, "latitude");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->latitude = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // longitude
    PyObject * field = PyObject_GetAttrString(_pymsg, "longitude");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->longitude = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // spacing
    PyObject * field = PyObject_GetAttrString(_pymsg, "spacing");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->spacing = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // terrain_height
    PyObject * field = PyObject_GetAttrString(_pymsg, "terrain_height");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->terrain_height = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // current_height
    PyObject * field = PyObject_GetAttrString(_pymsg, "current_height");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->current_height = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // pending
    PyObject * field = PyObject_GetAttrString(_pymsg, "pending");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->pending = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // loaded
    PyObject * field = PyObject_GetAttrString(_pymsg, "loaded");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->loaded = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * mavros_msgs__msg__terrain_report__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of TerrainReport */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("mavros_msgs.msg._terrain_report");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "TerrainReport");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  mavros_msgs__msg__TerrainReport * ros_message = (mavros_msgs__msg__TerrainReport *)raw_ros_message;
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
  {  // latitude
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->latitude);
    {
      int rc = PyObject_SetAttrString(_pymessage, "latitude", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // longitude
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->longitude);
    {
      int rc = PyObject_SetAttrString(_pymessage, "longitude", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // spacing
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->spacing);
    {
      int rc = PyObject_SetAttrString(_pymessage, "spacing", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // terrain_height
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->terrain_height);
    {
      int rc = PyObject_SetAttrString(_pymessage, "terrain_height", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // current_height
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->current_height);
    {
      int rc = PyObject_SetAttrString(_pymessage, "current_height", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pending
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->pending);
    {
      int rc = PyObject_SetAttrString(_pymessage, "pending", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // loaded
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->loaded);
    {
      int rc = PyObject_SetAttrString(_pymessage, "loaded", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
