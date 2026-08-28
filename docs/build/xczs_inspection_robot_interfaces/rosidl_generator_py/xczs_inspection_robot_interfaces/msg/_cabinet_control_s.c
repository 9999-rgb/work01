// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from xczs_inspection_robot_interfaces:msg/CabinetControl.idl
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
#include "xczs_inspection_robot_interfaces/msg/detail/cabinet_control__struct.h"
#include "xczs_inspection_robot_interfaces/msg/detail/cabinet_control__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"

#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool xczs_inspection_robot_interfaces__msg__cabinet_control__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[69];
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
    assert(strncmp("xczs_inspection_robot_interfaces.msg._cabinet_control.CabinetControl", full_classname_dest, 68) == 0);
  }
  xczs_inspection_robot_interfaces__msg__CabinetControl * ros_message = _ros_message;
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
  {  // display_name
    PyObject * field = PyObject_GetAttrString(_pymsg, "display_name");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->display_name, PyBytes_AS_STRING(encoded_field));
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
  {  // joint_name
    PyObject * field = PyObject_GetAttrString(_pymsg, "joint_name");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->joint_name, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // joint_state_topic
    PyObject * field = PyObject_GetAttrString(_pymsg, "joint_state_topic");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->joint_state_topic, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // pressed_topic
    PyObject * field = PyObject_GetAttrString(_pymsg, "pressed_topic");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->pressed_topic, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // state_topic
    PyObject * field = PyObject_GetAttrString(_pymsg, "state_topic");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->state_topic, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // supported_commands
    PyObject * field = PyObject_GetAttrString(_pymsg, "supported_commands");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->supported_commands = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // unit
    PyObject * field = PyObject_GetAttrString(_pymsg, "unit");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->unit, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // min_position
    PyObject * field = PyObject_GetAttrString(_pymsg, "min_position");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->min_position = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // max_position
    PyObject * field = PyObject_GetAttrString(_pymsg, "max_position");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->max_position = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // state_ids
    PyObject * field = PyObject_GetAttrString(_pymsg, "state_ids");
    if (!field) {
      return false;
    }
    {
      PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'state_ids'");
      if (!seq_field) {
        Py_DECREF(field);
        return false;
      }
      Py_ssize_t size = PySequence_Size(field);
      if (-1 == size) {
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
      if (!rosidl_runtime_c__String__Sequence__init(&(ros_message->state_ids), size)) {
        PyErr_SetString(PyExc_RuntimeError, "unable to create String__Sequence ros_message");
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
      rosidl_runtime_c__String * dest = ros_message->state_ids.data;
      for (Py_ssize_t i = 0; i < size; ++i) {
        PyObject * item = PySequence_Fast_GET_ITEM(seq_field, i);
        if (!item) {
          Py_DECREF(seq_field);
          Py_DECREF(field);
          return false;
        }
        assert(PyUnicode_Check(item));
        PyObject * encoded_item = PyUnicode_AsUTF8String(item);
        if (!encoded_item) {
          Py_DECREF(seq_field);
          Py_DECREF(field);
          return false;
        }
        rosidl_runtime_c__String__assign(&dest[i], PyBytes_AS_STRING(encoded_item));
        Py_DECREF(encoded_item);
      }
      Py_DECREF(seq_field);
    }
    Py_DECREF(field);
  }
  {  // state_labels
    PyObject * field = PyObject_GetAttrString(_pymsg, "state_labels");
    if (!field) {
      return false;
    }
    {
      PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'state_labels'");
      if (!seq_field) {
        Py_DECREF(field);
        return false;
      }
      Py_ssize_t size = PySequence_Size(field);
      if (-1 == size) {
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
      if (!rosidl_runtime_c__String__Sequence__init(&(ros_message->state_labels), size)) {
        PyErr_SetString(PyExc_RuntimeError, "unable to create String__Sequence ros_message");
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
      rosidl_runtime_c__String * dest = ros_message->state_labels.data;
      for (Py_ssize_t i = 0; i < size; ++i) {
        PyObject * item = PySequence_Fast_GET_ITEM(seq_field, i);
        if (!item) {
          Py_DECREF(seq_field);
          Py_DECREF(field);
          return false;
        }
        assert(PyUnicode_Check(item));
        PyObject * encoded_item = PyUnicode_AsUTF8String(item);
        if (!encoded_item) {
          Py_DECREF(seq_field);
          Py_DECREF(field);
          return false;
        }
        rosidl_runtime_c__String__assign(&dest[i], PyBytes_AS_STRING(encoded_item));
        Py_DECREF(encoded_item);
      }
      Py_DECREF(seq_field);
    }
    Py_DECREF(field);
  }
  {  // state_positions
    PyObject * field = PyObject_GetAttrString(_pymsg, "state_positions");
    if (!field) {
      return false;
    }
    if (PyObject_CheckBuffer(field)) {
      // Optimization for converting arrays of primitives
      Py_buffer view;
      int rc = PyObject_GetBuffer(field, &view, PyBUF_SIMPLE);
      if (rc < 0) {
        Py_DECREF(field);
        return false;
      }
      Py_ssize_t size = view.len / sizeof(double);
      if (!rosidl_runtime_c__double__Sequence__init(&(ros_message->state_positions), size)) {
        PyErr_SetString(PyExc_RuntimeError, "unable to create double__Sequence ros_message");
        PyBuffer_Release(&view);
        Py_DECREF(field);
        return false;
      }
      double * dest = ros_message->state_positions.data;
      rc = PyBuffer_ToContiguous(dest, &view, view.len, 'C');
      if (rc < 0) {
        PyBuffer_Release(&view);
        Py_DECREF(field);
        return false;
      }
      PyBuffer_Release(&view);
    } else {
      PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'state_positions'");
      if (!seq_field) {
        Py_DECREF(field);
        return false;
      }
      Py_ssize_t size = PySequence_Size(field);
      if (-1 == size) {
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
      if (!rosidl_runtime_c__double__Sequence__init(&(ros_message->state_positions), size)) {
        PyErr_SetString(PyExc_RuntimeError, "unable to create double__Sequence ros_message");
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
      double * dest = ros_message->state_positions.data;
      for (Py_ssize_t i = 0; i < size; ++i) {
        PyObject * item = PySequence_Fast_GET_ITEM(seq_field, i);
        if (!item) {
          Py_DECREF(seq_field);
          Py_DECREF(field);
          return false;
        }
        assert(PyFloat_Check(item));
        double tmp = PyFloat_AS_DOUBLE(item);
        memcpy(&dest[i], &tmp, sizeof(double));
      }
      Py_DECREF(seq_field);
    }
    Py_DECREF(field);
  }
  {  // requires_grasp
    PyObject * field = PyObject_GetAttrString(_pymsg, "requires_grasp");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->requires_grasp = (Py_True == field);
    Py_DECREF(field);
  }
  {  // operable
    PyObject * field = PyObject_GetAttrString(_pymsg, "operable");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->operable = (Py_True == field);
    Py_DECREF(field);
  }
  {  // unavailable_reason
    PyObject * field = PyObject_GetAttrString(_pymsg, "unavailable_reason");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->unavailable_reason, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // required_toolset
    PyObject * field = PyObject_GetAttrString(_pymsg, "required_toolset");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->required_toolset, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // toolset_compatible
    PyObject * field = PyObject_GetAttrString(_pymsg, "toolset_compatible");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->toolset_compatible = (Py_True == field);
    Py_DECREF(field);
  }
  {  // adapter_validated
    PyObject * field = PyObject_GetAttrString(_pymsg, "adapter_validated");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->adapter_validated = (Py_True == field);
    Py_DECREF(field);
  }
  {  // default_force
    PyObject * field = PyObject_GetAttrString(_pymsg, "default_force");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->default_force = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // min_trigger_force
    PyObject * field = PyObject_GetAttrString(_pymsg, "min_trigger_force");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->min_trigger_force = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // max_force
    PyObject * field = PyObject_GetAttrString(_pymsg, "max_force");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->max_force = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * xczs_inspection_robot_interfaces__msg__cabinet_control__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of CabinetControl */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("xczs_inspection_robot_interfaces.msg._cabinet_control");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "CabinetControl");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  xczs_inspection_robot_interfaces__msg__CabinetControl * ros_message = (xczs_inspection_robot_interfaces__msg__CabinetControl *)raw_ros_message;
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
  {  // display_name
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->display_name.data,
      strlen(ros_message->display_name.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "display_name", field);
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
  {  // joint_name
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->joint_name.data,
      strlen(ros_message->joint_name.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "joint_name", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // joint_state_topic
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->joint_state_topic.data,
      strlen(ros_message->joint_state_topic.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "joint_state_topic", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pressed_topic
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->pressed_topic.data,
      strlen(ros_message->pressed_topic.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "pressed_topic", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // state_topic
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->state_topic.data,
      strlen(ros_message->state_topic.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "state_topic", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // supported_commands
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->supported_commands);
    {
      int rc = PyObject_SetAttrString(_pymessage, "supported_commands", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // unit
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->unit.data,
      strlen(ros_message->unit.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "unit", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // min_position
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->min_position);
    {
      int rc = PyObject_SetAttrString(_pymessage, "min_position", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // max_position
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->max_position);
    {
      int rc = PyObject_SetAttrString(_pymessage, "max_position", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // state_ids
    PyObject * field = NULL;
    size_t size = ros_message->state_ids.size;
    rosidl_runtime_c__String * src = ros_message->state_ids.data;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    for (size_t i = 0; i < size; ++i) {
      PyObject * decoded_item = PyUnicode_DecodeUTF8(src[i].data, strlen(src[i].data), "replace");
      if (!decoded_item) {
        return NULL;
      }
      int rc = PyList_SetItem(field, i, decoded_item);
      (void)rc;
      assert(rc == 0);
    }
    assert(PySequence_Check(field));
    {
      int rc = PyObject_SetAttrString(_pymessage, "state_ids", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // state_labels
    PyObject * field = NULL;
    size_t size = ros_message->state_labels.size;
    rosidl_runtime_c__String * src = ros_message->state_labels.data;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    for (size_t i = 0; i < size; ++i) {
      PyObject * decoded_item = PyUnicode_DecodeUTF8(src[i].data, strlen(src[i].data), "replace");
      if (!decoded_item) {
        return NULL;
      }
      int rc = PyList_SetItem(field, i, decoded_item);
      (void)rc;
      assert(rc == 0);
    }
    assert(PySequence_Check(field));
    {
      int rc = PyObject_SetAttrString(_pymessage, "state_labels", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // state_positions
    PyObject * field = NULL;
    field = PyObject_GetAttrString(_pymessage, "state_positions");
    if (!field) {
      return NULL;
    }
    assert(field->ob_type != NULL);
    assert(field->ob_type->tp_name != NULL);
    assert(strcmp(field->ob_type->tp_name, "array.array") == 0);
    // ensure that itemsize matches the sizeof of the ROS message field
    PyObject * itemsize_attr = PyObject_GetAttrString(field, "itemsize");
    assert(itemsize_attr != NULL);
    size_t itemsize = PyLong_AsSize_t(itemsize_attr);
    Py_DECREF(itemsize_attr);
    if (itemsize != sizeof(double)) {
      PyErr_SetString(PyExc_RuntimeError, "itemsize doesn't match expectation");
      Py_DECREF(field);
      return NULL;
    }
    // clear the array, poor approach to remove potential default values
    Py_ssize_t length = PyObject_Length(field);
    if (-1 == length) {
      Py_DECREF(field);
      return NULL;
    }
    if (length > 0) {
      PyObject * pop = PyObject_GetAttrString(field, "pop");
      assert(pop != NULL);
      for (Py_ssize_t i = 0; i < length; ++i) {
        PyObject * ret = PyObject_CallFunctionObjArgs(pop, NULL);
        if (!ret) {
          Py_DECREF(pop);
          Py_DECREF(field);
          return NULL;
        }
        Py_DECREF(ret);
      }
      Py_DECREF(pop);
    }
    if (ros_message->state_positions.size > 0) {
      // populating the array.array using the frombytes method
      PyObject * frombytes = PyObject_GetAttrString(field, "frombytes");
      assert(frombytes != NULL);
      double * src = &(ros_message->state_positions.data[0]);
      PyObject * data = PyBytes_FromStringAndSize((const char *)src, ros_message->state_positions.size * sizeof(double));
      assert(data != NULL);
      PyObject * ret = PyObject_CallFunctionObjArgs(frombytes, data, NULL);
      Py_DECREF(data);
      Py_DECREF(frombytes);
      if (!ret) {
        Py_DECREF(field);
        return NULL;
      }
      Py_DECREF(ret);
    }
    Py_DECREF(field);
  }
  {  // requires_grasp
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->requires_grasp ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "requires_grasp", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // operable
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->operable ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "operable", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // unavailable_reason
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->unavailable_reason.data,
      strlen(ros_message->unavailable_reason.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "unavailable_reason", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // required_toolset
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->required_toolset.data,
      strlen(ros_message->required_toolset.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "required_toolset", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // toolset_compatible
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->toolset_compatible ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "toolset_compatible", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // adapter_validated
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->adapter_validated ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "adapter_validated", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // default_force
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->default_force);
    {
      int rc = PyObject_SetAttrString(_pymessage, "default_force", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // min_trigger_force
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->min_trigger_force);
    {
      int rc = PyObject_SetAttrString(_pymessage, "min_trigger_force", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // max_force
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->max_force);
    {
      int rc = PyObject_SetAttrString(_pymessage, "max_force", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
