// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from mavros_msgs:msg/RTKBaseline.idl
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
#include "mavros_msgs/msg/detail/rtk_baseline__struct.h"
#include "mavros_msgs/msg/detail/rtk_baseline__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool mavros_msgs__msg__rtk_baseline__convert_from_py(PyObject * _pymsg, void * _ros_message)
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
    assert(strncmp("mavros_msgs.msg._rtk_baseline.RTKBaseline", full_classname_dest, 41) == 0);
  }
  mavros_msgs__msg__RTKBaseline * ros_message = _ros_message;
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
  {  // time_last_baseline_ms
    PyObject * field = PyObject_GetAttrString(_pymsg, "time_last_baseline_ms");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->time_last_baseline_ms = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // rtk_receiver_id
    PyObject * field = PyObject_GetAttrString(_pymsg, "rtk_receiver_id");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->rtk_receiver_id = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // wn
    PyObject * field = PyObject_GetAttrString(_pymsg, "wn");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->wn = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // tow
    PyObject * field = PyObject_GetAttrString(_pymsg, "tow");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->tow = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // rtk_health
    PyObject * field = PyObject_GetAttrString(_pymsg, "rtk_health");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->rtk_health = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // rtk_rate
    PyObject * field = PyObject_GetAttrString(_pymsg, "rtk_rate");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->rtk_rate = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // nsats
    PyObject * field = PyObject_GetAttrString(_pymsg, "nsats");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->nsats = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // baseline_coords_type
    PyObject * field = PyObject_GetAttrString(_pymsg, "baseline_coords_type");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->baseline_coords_type = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // baseline_a_mm
    PyObject * field = PyObject_GetAttrString(_pymsg, "baseline_a_mm");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->baseline_a_mm = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // baseline_b_mm
    PyObject * field = PyObject_GetAttrString(_pymsg, "baseline_b_mm");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->baseline_b_mm = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // baseline_c_mm
    PyObject * field = PyObject_GetAttrString(_pymsg, "baseline_c_mm");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->baseline_c_mm = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // accuracy
    PyObject * field = PyObject_GetAttrString(_pymsg, "accuracy");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->accuracy = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // iar_num_hypotheses
    PyObject * field = PyObject_GetAttrString(_pymsg, "iar_num_hypotheses");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->iar_num_hypotheses = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * mavros_msgs__msg__rtk_baseline__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of RTKBaseline */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("mavros_msgs.msg._rtk_baseline");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "RTKBaseline");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  mavros_msgs__msg__RTKBaseline * ros_message = (mavros_msgs__msg__RTKBaseline *)raw_ros_message;
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
  {  // time_last_baseline_ms
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->time_last_baseline_ms);
    {
      int rc = PyObject_SetAttrString(_pymessage, "time_last_baseline_ms", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // rtk_receiver_id
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->rtk_receiver_id);
    {
      int rc = PyObject_SetAttrString(_pymessage, "rtk_receiver_id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // wn
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->wn);
    {
      int rc = PyObject_SetAttrString(_pymessage, "wn", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // tow
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->tow);
    {
      int rc = PyObject_SetAttrString(_pymessage, "tow", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // rtk_health
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->rtk_health);
    {
      int rc = PyObject_SetAttrString(_pymessage, "rtk_health", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // rtk_rate
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->rtk_rate);
    {
      int rc = PyObject_SetAttrString(_pymessage, "rtk_rate", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // nsats
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->nsats);
    {
      int rc = PyObject_SetAttrString(_pymessage, "nsats", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // baseline_coords_type
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->baseline_coords_type);
    {
      int rc = PyObject_SetAttrString(_pymessage, "baseline_coords_type", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // baseline_a_mm
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->baseline_a_mm);
    {
      int rc = PyObject_SetAttrString(_pymessage, "baseline_a_mm", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // baseline_b_mm
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->baseline_b_mm);
    {
      int rc = PyObject_SetAttrString(_pymessage, "baseline_b_mm", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // baseline_c_mm
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->baseline_c_mm);
    {
      int rc = PyObject_SetAttrString(_pymessage, "baseline_c_mm", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // accuracy
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->accuracy);
    {
      int rc = PyObject_SetAttrString(_pymessage, "accuracy", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // iar_num_hypotheses
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->iar_num_hypotheses);
    {
      int rc = PyObject_SetAttrString(_pymessage, "iar_num_hypotheses", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
