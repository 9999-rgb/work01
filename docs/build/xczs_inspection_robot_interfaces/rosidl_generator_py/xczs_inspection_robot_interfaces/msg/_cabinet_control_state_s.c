// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from xczs_inspection_robot_interfaces:msg/CabinetControlState.idl
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
#include "xczs_inspection_robot_interfaces/msg/detail/cabinet_control_state__struct.h"
#include "xczs_inspection_robot_interfaces/msg/detail/cabinet_control_state__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool xczs_inspection_robot_interfaces__msg__cabinet_control_state__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[80];
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
    assert(strncmp("xczs_inspection_robot_interfaces.msg._cabinet_control_state.CabinetControlState", full_classname_dest, 79) == 0);
  }
  xczs_inspection_robot_interfaces__msg__CabinetControlState * ros_message = _ros_message;
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
  {  // control_id
    PyObject * field = PyObject_GetAttrString(_pymsg, "control_id");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->control_id, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // control_type
    PyObject * field = PyObject_GetAttrString(_pymsg, "control_type");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->control_type = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // valid
    PyObject * field = PyObject_GetAttrString(_pymsg, "valid");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->valid = (Py_True == field);
    Py_DECREF(field);
  }
  {  // position
    PyObject * field = PyObject_GetAttrString(_pymsg, "position");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->position = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // velocity
    PyObject * field = PyObject_GetAttrString(_pymsg, "velocity");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->velocity = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // effort
    PyObject * field = PyObject_GetAttrString(_pymsg, "effort");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->effort = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // normalized_position
    PyObject * field = PyObject_GetAttrString(_pymsg, "normalized_position");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->normalized_position = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // state_id
    PyObject * field = PyObject_GetAttrString(_pymsg, "state_id");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->state_id, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // activated
    PyObject * field = PyObject_GetAttrString(_pymsg, "activated");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->activated = (Py_True == field);
    Py_DECREF(field);
  }
  {  // in_motion
    PyObject * field = PyObject_GetAttrString(_pymsg, "in_motion");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->in_motion = (Py_True == field);
    Py_DECREF(field);
  }
  {  // transition_sequence
    PyObject * field = PyObject_GetAttrString(_pymsg, "transition_sequence");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->transition_sequence = PyLong_AsUnsignedLongLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * xczs_inspection_robot_interfaces__msg__cabinet_control_state__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of CabinetControlState */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("xczs_inspection_robot_interfaces.msg._cabinet_control_state");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "CabinetControlState");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  xczs_inspection_robot_interfaces__msg__CabinetControlState * ros_message = (xczs_inspection_robot_interfaces__msg__CabinetControlState *)raw_ros_message;
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
  {  // control_id
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->control_id.data,
      strlen(ros_message->control_id.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "control_id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // control_type
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->control_type);
    {
      int rc = PyObject_SetAttrString(_pymessage, "control_type", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // valid
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->valid ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "valid", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // position
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->position);
    {
      int rc = PyObject_SetAttrString(_pymessage, "position", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // velocity
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->velocity);
    {
      int rc = PyObject_SetAttrString(_pymessage, "velocity", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // effort
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->effort);
    {
      int rc = PyObject_SetAttrString(_pymessage, "effort", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // normalized_position
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->normalized_position);
    {
      int rc = PyObject_SetAttrString(_pymessage, "normalized_position", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // state_id
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->state_id.data,
      strlen(ros_message->state_id.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "state_id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // activated
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->activated ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "activated", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // in_motion
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->in_motion ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "in_motion", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // transition_sequence
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLongLong(ros_message->transition_sequence);
    {
      int rc = PyObject_SetAttrString(_pymessage, "transition_sequence", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
