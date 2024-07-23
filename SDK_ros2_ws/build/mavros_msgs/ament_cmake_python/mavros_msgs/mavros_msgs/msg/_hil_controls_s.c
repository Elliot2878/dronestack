// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from mavros_msgs:msg/HilControls.idl
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
#include "mavros_msgs/msg/detail/hil_controls__struct.h"
#include "mavros_msgs/msg/detail/hil_controls__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool mavros_msgs__msg__hil_controls__convert_from_py(PyObject * _pymsg, void * _ros_message)
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
    assert(strncmp("mavros_msgs.msg._hil_controls.HilControls", full_classname_dest, 41) == 0);
  }
  mavros_msgs__msg__HilControls * ros_message = _ros_message;
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
  {  // roll_ailerons
    PyObject * field = PyObject_GetAttrString(_pymsg, "roll_ailerons");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->roll_ailerons = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // pitch_elevator
    PyObject * field = PyObject_GetAttrString(_pymsg, "pitch_elevator");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->pitch_elevator = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // yaw_rudder
    PyObject * field = PyObject_GetAttrString(_pymsg, "yaw_rudder");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->yaw_rudder = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // throttle
    PyObject * field = PyObject_GetAttrString(_pymsg, "throttle");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->throttle = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // aux1
    PyObject * field = PyObject_GetAttrString(_pymsg, "aux1");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->aux1 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // aux2
    PyObject * field = PyObject_GetAttrString(_pymsg, "aux2");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->aux2 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // aux3
    PyObject * field = PyObject_GetAttrString(_pymsg, "aux3");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->aux3 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // aux4
    PyObject * field = PyObject_GetAttrString(_pymsg, "aux4");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->aux4 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // mode
    PyObject * field = PyObject_GetAttrString(_pymsg, "mode");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->mode = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // nav_mode
    PyObject * field = PyObject_GetAttrString(_pymsg, "nav_mode");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->nav_mode = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * mavros_msgs__msg__hil_controls__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of HilControls */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("mavros_msgs.msg._hil_controls");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "HilControls");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  mavros_msgs__msg__HilControls * ros_message = (mavros_msgs__msg__HilControls *)raw_ros_message;
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
  {  // roll_ailerons
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->roll_ailerons);
    {
      int rc = PyObject_SetAttrString(_pymessage, "roll_ailerons", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pitch_elevator
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->pitch_elevator);
    {
      int rc = PyObject_SetAttrString(_pymessage, "pitch_elevator", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // yaw_rudder
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->yaw_rudder);
    {
      int rc = PyObject_SetAttrString(_pymessage, "yaw_rudder", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // throttle
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->throttle);
    {
      int rc = PyObject_SetAttrString(_pymessage, "throttle", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // aux1
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->aux1);
    {
      int rc = PyObject_SetAttrString(_pymessage, "aux1", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // aux2
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->aux2);
    {
      int rc = PyObject_SetAttrString(_pymessage, "aux2", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // aux3
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->aux3);
    {
      int rc = PyObject_SetAttrString(_pymessage, "aux3", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // aux4
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->aux4);
    {
      int rc = PyObject_SetAttrString(_pymessage, "aux4", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // mode
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->mode);
    {
      int rc = PyObject_SetAttrString(_pymessage, "mode", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // nav_mode
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->nav_mode);
    {
      int rc = PyObject_SetAttrString(_pymessage, "nav_mode", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
