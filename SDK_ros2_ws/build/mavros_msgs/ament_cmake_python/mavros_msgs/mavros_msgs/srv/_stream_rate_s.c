// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from mavros_msgs:srv/StreamRate.idl
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
#include "mavros_msgs/srv/detail/stream_rate__struct.h"
#include "mavros_msgs/srv/detail/stream_rate__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool mavros_msgs__srv__stream_rate__request__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[48];
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
    assert(strncmp("mavros_msgs.srv._stream_rate.StreamRate_Request", full_classname_dest, 47) == 0);
  }
  mavros_msgs__srv__StreamRate_Request * ros_message = _ros_message;
  {  // stream_id
    PyObject * field = PyObject_GetAttrString(_pymsg, "stream_id");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->stream_id = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // message_rate
    PyObject * field = PyObject_GetAttrString(_pymsg, "message_rate");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->message_rate = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // on_off
    PyObject * field = PyObject_GetAttrString(_pymsg, "on_off");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->on_off = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * mavros_msgs__srv__stream_rate__request__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of StreamRate_Request */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("mavros_msgs.srv._stream_rate");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "StreamRate_Request");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  mavros_msgs__srv__StreamRate_Request * ros_message = (mavros_msgs__srv__StreamRate_Request *)raw_ros_message;
  {  // stream_id
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->stream_id);
    {
      int rc = PyObject_SetAttrString(_pymessage, "stream_id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // message_rate
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->message_rate);
    {
      int rc = PyObject_SetAttrString(_pymessage, "message_rate", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // on_off
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->on_off ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "on_off", field);
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
// #include "mavros_msgs/srv/detail/stream_rate__struct.h"
// already included above
// #include "mavros_msgs/srv/detail/stream_rate__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool mavros_msgs__srv__stream_rate__response__convert_from_py(PyObject * _pymsg, void * _ros_message)
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
    assert(strncmp("mavros_msgs.srv._stream_rate.StreamRate_Response", full_classname_dest, 48) == 0);
  }
  mavros_msgs__srv__StreamRate_Response * ros_message = _ros_message;
  ros_message->structure_needs_at_least_one_member = 0;

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * mavros_msgs__srv__stream_rate__response__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of StreamRate_Response */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("mavros_msgs.srv._stream_rate");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "StreamRate_Response");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  (void)raw_ros_message;

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
