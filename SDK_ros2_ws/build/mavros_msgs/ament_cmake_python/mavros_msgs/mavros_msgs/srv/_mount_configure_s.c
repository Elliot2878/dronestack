// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from mavros_msgs:srv/MountConfigure.idl
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
#include "mavros_msgs/srv/detail/mount_configure__struct.h"
#include "mavros_msgs/srv/detail/mount_configure__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool mavros_msgs__srv__mount_configure__request__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[56];
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
    assert(strncmp("mavros_msgs.srv._mount_configure.MountConfigure_Request", full_classname_dest, 55) == 0);
  }
  mavros_msgs__srv__MountConfigure_Request * ros_message = _ros_message;
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
  {  // mode
    PyObject * field = PyObject_GetAttrString(_pymsg, "mode");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->mode = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // stabilize_roll
    PyObject * field = PyObject_GetAttrString(_pymsg, "stabilize_roll");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->stabilize_roll = (Py_True == field);
    Py_DECREF(field);
  }
  {  // stabilize_pitch
    PyObject * field = PyObject_GetAttrString(_pymsg, "stabilize_pitch");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->stabilize_pitch = (Py_True == field);
    Py_DECREF(field);
  }
  {  // stabilize_yaw
    PyObject * field = PyObject_GetAttrString(_pymsg, "stabilize_yaw");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->stabilize_yaw = (Py_True == field);
    Py_DECREF(field);
  }
  {  // roll_input
    PyObject * field = PyObject_GetAttrString(_pymsg, "roll_input");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->roll_input = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // pitch_input
    PyObject * field = PyObject_GetAttrString(_pymsg, "pitch_input");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->pitch_input = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // yaw_input
    PyObject * field = PyObject_GetAttrString(_pymsg, "yaw_input");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->yaw_input = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * mavros_msgs__srv__mount_configure__request__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of MountConfigure_Request */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("mavros_msgs.srv._mount_configure");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "MountConfigure_Request");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  mavros_msgs__srv__MountConfigure_Request * ros_message = (mavros_msgs__srv__MountConfigure_Request *)raw_ros_message;
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
  {  // stabilize_roll
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->stabilize_roll ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "stabilize_roll", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // stabilize_pitch
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->stabilize_pitch ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "stabilize_pitch", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // stabilize_yaw
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->stabilize_yaw ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "stabilize_yaw", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // roll_input
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->roll_input);
    {
      int rc = PyObject_SetAttrString(_pymessage, "roll_input", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pitch_input
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->pitch_input);
    {
      int rc = PyObject_SetAttrString(_pymessage, "pitch_input", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // yaw_input
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->yaw_input);
    {
      int rc = PyObject_SetAttrString(_pymessage, "yaw_input", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
// already included above
// #include <Python.h>
// already included above
// #include <stdbool.h>
// already included above
// #include "numpy/ndarrayobject.h"
// already included above
// #include "rosidl_runtime_c/visibility_control.h"
// already included above
// #include "mavros_msgs/srv/detail/mount_configure__struct.h"
// already included above
// #include "mavros_msgs/srv/detail/mount_configure__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool mavros_msgs__srv__mount_configure__response__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[57];
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
    assert(strncmp("mavros_msgs.srv._mount_configure.MountConfigure_Response", full_classname_dest, 56) == 0);
  }
  mavros_msgs__srv__MountConfigure_Response * ros_message = _ros_message;
  {  // success
    PyObject * field = PyObject_GetAttrString(_pymsg, "success");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->success = (Py_True == field);
    Py_DECREF(field);
  }
  {  // result
    PyObject * field = PyObject_GetAttrString(_pymsg, "result");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->result = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * mavros_msgs__srv__mount_configure__response__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of MountConfigure_Response */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("mavros_msgs.srv._mount_configure");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "MountConfigure_Response");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  mavros_msgs__srv__MountConfigure_Response * ros_message = (mavros_msgs__srv__MountConfigure_Response *)raw_ros_message;
  {  // success
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->success ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "success", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // result
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->result);
    {
      int rc = PyObject_SetAttrString(_pymessage, "result", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
