// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from mavros_msgs:msg/OpticalFlowRad.idl
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
#include "mavros_msgs/msg/detail/optical_flow_rad__struct.h"
#include "mavros_msgs/msg/detail/optical_flow_rad__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool mavros_msgs__msg__optical_flow_rad__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[49];
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
    assert(strncmp("mavros_msgs.msg._optical_flow_rad.OpticalFlowRad", full_classname_dest, 48) == 0);
  }
  mavros_msgs__msg__OpticalFlowRad * ros_message = _ros_message;
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
  {  // integration_time_us
    PyObject * field = PyObject_GetAttrString(_pymsg, "integration_time_us");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->integration_time_us = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // integrated_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "integrated_x");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->integrated_x = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // integrated_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "integrated_y");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->integrated_y = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // integrated_xgyro
    PyObject * field = PyObject_GetAttrString(_pymsg, "integrated_xgyro");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->integrated_xgyro = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // integrated_ygyro
    PyObject * field = PyObject_GetAttrString(_pymsg, "integrated_ygyro");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->integrated_ygyro = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // integrated_zgyro
    PyObject * field = PyObject_GetAttrString(_pymsg, "integrated_zgyro");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->integrated_zgyro = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // temperature
    PyObject * field = PyObject_GetAttrString(_pymsg, "temperature");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->temperature = (int16_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // quality
    PyObject * field = PyObject_GetAttrString(_pymsg, "quality");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->quality = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // time_delta_distance_us
    PyObject * field = PyObject_GetAttrString(_pymsg, "time_delta_distance_us");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->time_delta_distance_us = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // distance
    PyObject * field = PyObject_GetAttrString(_pymsg, "distance");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->distance = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * mavros_msgs__msg__optical_flow_rad__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of OpticalFlowRad */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("mavros_msgs.msg._optical_flow_rad");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "OpticalFlowRad");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  mavros_msgs__msg__OpticalFlowRad * ros_message = (mavros_msgs__msg__OpticalFlowRad *)raw_ros_message;
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
  {  // integration_time_us
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->integration_time_us);
    {
      int rc = PyObject_SetAttrString(_pymessage, "integration_time_us", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // integrated_x
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->integrated_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "integrated_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // integrated_y
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->integrated_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "integrated_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // integrated_xgyro
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->integrated_xgyro);
    {
      int rc = PyObject_SetAttrString(_pymessage, "integrated_xgyro", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // integrated_ygyro
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->integrated_ygyro);
    {
      int rc = PyObject_SetAttrString(_pymessage, "integrated_ygyro", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // integrated_zgyro
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->integrated_zgyro);
    {
      int rc = PyObject_SetAttrString(_pymessage, "integrated_zgyro", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // temperature
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->temperature);
    {
      int rc = PyObject_SetAttrString(_pymessage, "temperature", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // quality
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->quality);
    {
      int rc = PyObject_SetAttrString(_pymessage, "quality", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // time_delta_distance_us
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->time_delta_distance_us);
    {
      int rc = PyObject_SetAttrString(_pymessage, "time_delta_distance_us", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // distance
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->distance);
    {
      int rc = PyObject_SetAttrString(_pymessage, "distance", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
