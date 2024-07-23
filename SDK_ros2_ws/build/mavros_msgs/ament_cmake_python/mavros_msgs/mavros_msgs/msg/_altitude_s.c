// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from mavros_msgs:msg/Altitude.idl
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
#include "mavros_msgs/msg/detail/altitude__struct.h"
#include "mavros_msgs/msg/detail/altitude__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool mavros_msgs__msg__altitude__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[35];
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
    assert(strncmp("mavros_msgs.msg._altitude.Altitude", full_classname_dest, 34) == 0);
  }
  mavros_msgs__msg__Altitude * ros_message = _ros_message;
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
  {  // monotonic
    PyObject * field = PyObject_GetAttrString(_pymsg, "monotonic");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->monotonic = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // amsl
    PyObject * field = PyObject_GetAttrString(_pymsg, "amsl");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->amsl = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // local
    PyObject * field = PyObject_GetAttrString(_pymsg, "local");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->local = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // relative
    PyObject * field = PyObject_GetAttrString(_pymsg, "relative");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->relative = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // terrain
    PyObject * field = PyObject_GetAttrString(_pymsg, "terrain");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->terrain = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // bottom_clearance
    PyObject * field = PyObject_GetAttrString(_pymsg, "bottom_clearance");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->bottom_clearance = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * mavros_msgs__msg__altitude__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Altitude */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("mavros_msgs.msg._altitude");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Altitude");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  mavros_msgs__msg__Altitude * ros_message = (mavros_msgs__msg__Altitude *)raw_ros_message;
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
  {  // monotonic
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->monotonic);
    {
      int rc = PyObject_SetAttrString(_pymessage, "monotonic", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // amsl
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->amsl);
    {
      int rc = PyObject_SetAttrString(_pymessage, "amsl", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // local
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->local);
    {
      int rc = PyObject_SetAttrString(_pymessage, "local", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // relative
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->relative);
    {
      int rc = PyObject_SetAttrString(_pymessage, "relative", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // terrain
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->terrain);
    {
      int rc = PyObject_SetAttrString(_pymessage, "terrain", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // bottom_clearance
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->bottom_clearance);
    {
      int rc = PyObject_SetAttrString(_pymessage, "bottom_clearance", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
