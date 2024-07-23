// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from mavros_msgs:msg/EstimatorStatus.idl
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
#include "mavros_msgs/msg/detail/estimator_status__struct.h"
#include "mavros_msgs/msg/detail/estimator_status__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool mavros_msgs__msg__estimator_status__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[50];
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
    assert(strncmp("mavros_msgs.msg._estimator_status.EstimatorStatus", full_classname_dest, 49) == 0);
  }
  mavros_msgs__msg__EstimatorStatus * ros_message = _ros_message;
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
  {  // attitude_status_flag
    PyObject * field = PyObject_GetAttrString(_pymsg, "attitude_status_flag");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->attitude_status_flag = (Py_True == field);
    Py_DECREF(field);
  }
  {  // velocity_horiz_status_flag
    PyObject * field = PyObject_GetAttrString(_pymsg, "velocity_horiz_status_flag");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->velocity_horiz_status_flag = (Py_True == field);
    Py_DECREF(field);
  }
  {  // velocity_vert_status_flag
    PyObject * field = PyObject_GetAttrString(_pymsg, "velocity_vert_status_flag");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->velocity_vert_status_flag = (Py_True == field);
    Py_DECREF(field);
  }
  {  // pos_horiz_rel_status_flag
    PyObject * field = PyObject_GetAttrString(_pymsg, "pos_horiz_rel_status_flag");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->pos_horiz_rel_status_flag = (Py_True == field);
    Py_DECREF(field);
  }
  {  // pos_horiz_abs_status_flag
    PyObject * field = PyObject_GetAttrString(_pymsg, "pos_horiz_abs_status_flag");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->pos_horiz_abs_status_flag = (Py_True == field);
    Py_DECREF(field);
  }
  {  // pos_vert_abs_status_flag
    PyObject * field = PyObject_GetAttrString(_pymsg, "pos_vert_abs_status_flag");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->pos_vert_abs_status_flag = (Py_True == field);
    Py_DECREF(field);
  }
  {  // pos_vert_agl_status_flag
    PyObject * field = PyObject_GetAttrString(_pymsg, "pos_vert_agl_status_flag");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->pos_vert_agl_status_flag = (Py_True == field);
    Py_DECREF(field);
  }
  {  // const_pos_mode_status_flag
    PyObject * field = PyObject_GetAttrString(_pymsg, "const_pos_mode_status_flag");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->const_pos_mode_status_flag = (Py_True == field);
    Py_DECREF(field);
  }
  {  // pred_pos_horiz_rel_status_flag
    PyObject * field = PyObject_GetAttrString(_pymsg, "pred_pos_horiz_rel_status_flag");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->pred_pos_horiz_rel_status_flag = (Py_True == field);
    Py_DECREF(field);
  }
  {  // pred_pos_horiz_abs_status_flag
    PyObject * field = PyObject_GetAttrString(_pymsg, "pred_pos_horiz_abs_status_flag");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->pred_pos_horiz_abs_status_flag = (Py_True == field);
    Py_DECREF(field);
  }
  {  // gps_glitch_status_flag
    PyObject * field = PyObject_GetAttrString(_pymsg, "gps_glitch_status_flag");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->gps_glitch_status_flag = (Py_True == field);
    Py_DECREF(field);
  }
  {  // accel_error_status_flag
    PyObject * field = PyObject_GetAttrString(_pymsg, "accel_error_status_flag");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->accel_error_status_flag = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * mavros_msgs__msg__estimator_status__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of EstimatorStatus */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("mavros_msgs.msg._estimator_status");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "EstimatorStatus");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  mavros_msgs__msg__EstimatorStatus * ros_message = (mavros_msgs__msg__EstimatorStatus *)raw_ros_message;
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
  {  // attitude_status_flag
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->attitude_status_flag ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "attitude_status_flag", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // velocity_horiz_status_flag
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->velocity_horiz_status_flag ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "velocity_horiz_status_flag", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // velocity_vert_status_flag
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->velocity_vert_status_flag ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "velocity_vert_status_flag", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pos_horiz_rel_status_flag
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->pos_horiz_rel_status_flag ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "pos_horiz_rel_status_flag", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pos_horiz_abs_status_flag
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->pos_horiz_abs_status_flag ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "pos_horiz_abs_status_flag", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pos_vert_abs_status_flag
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->pos_vert_abs_status_flag ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "pos_vert_abs_status_flag", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pos_vert_agl_status_flag
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->pos_vert_agl_status_flag ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "pos_vert_agl_status_flag", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // const_pos_mode_status_flag
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->const_pos_mode_status_flag ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "const_pos_mode_status_flag", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pred_pos_horiz_rel_status_flag
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->pred_pos_horiz_rel_status_flag ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "pred_pos_horiz_rel_status_flag", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pred_pos_horiz_abs_status_flag
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->pred_pos_horiz_abs_status_flag ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "pred_pos_horiz_abs_status_flag", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // gps_glitch_status_flag
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->gps_glitch_status_flag ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "gps_glitch_status_flag", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // accel_error_status_flag
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->accel_error_status_flag ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "accel_error_status_flag", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
