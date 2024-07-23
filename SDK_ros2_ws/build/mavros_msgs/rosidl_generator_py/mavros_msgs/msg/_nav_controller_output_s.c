// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from mavros_msgs:msg/NavControllerOutput.idl
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
#include "mavros_msgs/msg/detail/nav_controller_output__struct.h"
#include "mavros_msgs/msg/detail/nav_controller_output__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool mavros_msgs__msg__nav_controller_output__convert_from_py(PyObject * _pymsg, void * _ros_message)
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
    assert(strncmp("mavros_msgs.msg._nav_controller_output.NavControllerOutput", full_classname_dest, 58) == 0);
  }
  mavros_msgs__msg__NavControllerOutput * ros_message = _ros_message;
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
  {  // nav_roll
    PyObject * field = PyObject_GetAttrString(_pymsg, "nav_roll");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->nav_roll = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // nav_pitch
    PyObject * field = PyObject_GetAttrString(_pymsg, "nav_pitch");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->nav_pitch = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // nav_bearing
    PyObject * field = PyObject_GetAttrString(_pymsg, "nav_bearing");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->nav_bearing = (int16_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // target_bearing
    PyObject * field = PyObject_GetAttrString(_pymsg, "target_bearing");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->target_bearing = (int16_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // wp_dist
    PyObject * field = PyObject_GetAttrString(_pymsg, "wp_dist");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->wp_dist = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // alt_error
    PyObject * field = PyObject_GetAttrString(_pymsg, "alt_error");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->alt_error = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // aspd_error
    PyObject * field = PyObject_GetAttrString(_pymsg, "aspd_error");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->aspd_error = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // xtrack_error
    PyObject * field = PyObject_GetAttrString(_pymsg, "xtrack_error");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->xtrack_error = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * mavros_msgs__msg__nav_controller_output__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of NavControllerOutput */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("mavros_msgs.msg._nav_controller_output");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "NavControllerOutput");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  mavros_msgs__msg__NavControllerOutput * ros_message = (mavros_msgs__msg__NavControllerOutput *)raw_ros_message;
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
  {  // nav_roll
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->nav_roll);
    {
      int rc = PyObject_SetAttrString(_pymessage, "nav_roll", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // nav_pitch
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->nav_pitch);
    {
      int rc = PyObject_SetAttrString(_pymessage, "nav_pitch", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // nav_bearing
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->nav_bearing);
    {
      int rc = PyObject_SetAttrString(_pymessage, "nav_bearing", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // target_bearing
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->target_bearing);
    {
      int rc = PyObject_SetAttrString(_pymessage, "target_bearing", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // wp_dist
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->wp_dist);
    {
      int rc = PyObject_SetAttrString(_pymessage, "wp_dist", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // alt_error
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->alt_error);
    {
      int rc = PyObject_SetAttrString(_pymessage, "alt_error", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // aspd_error
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->aspd_error);
    {
      int rc = PyObject_SetAttrString(_pymessage, "aspd_error", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // xtrack_error
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->xtrack_error);
    {
      int rc = PyObject_SetAttrString(_pymessage, "xtrack_error", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
