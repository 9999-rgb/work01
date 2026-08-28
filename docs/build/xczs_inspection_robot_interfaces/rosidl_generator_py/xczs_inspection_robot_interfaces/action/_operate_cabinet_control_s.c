// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from xczs_inspection_robot_interfaces:action/OperateCabinetControl.idl
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
#include "xczs_inspection_robot_interfaces/action/detail/operate_cabinet_control__struct.h"
#include "xczs_inspection_robot_interfaces/action/detail/operate_cabinet_control__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool xczs_inspection_robot_interfaces__action__operate_cabinet_control__goal__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[92];
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
    assert(strncmp("xczs_inspection_robot_interfaces.action._operate_cabinet_control.OperateCabinetControl_Goal", full_classname_dest, 91) == 0);
  }
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_Goal * ros_message = _ros_message;
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
  {  // command
    PyObject * field = PyObject_GetAttrString(_pymsg, "command");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->command = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // target_state
    PyObject * field = PyObject_GetAttrString(_pymsg, "target_state");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->target_state, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // target_position
    PyObject * field = PyObject_GetAttrString(_pymsg, "target_position");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->target_position = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // use_target_position
    PyObject * field = PyObject_GetAttrString(_pymsg, "use_target_position");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->use_target_position = (Py_True == field);
    Py_DECREF(field);
  }
  {  // force
    PyObject * field = PyObject_GetAttrString(_pymsg, "force");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->force = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // navigate_to_staging_pose
    PyObject * field = PyObject_GetAttrString(_pymsg, "navigate_to_staging_pose");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->navigate_to_staging_pose = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * xczs_inspection_robot_interfaces__action__operate_cabinet_control__goal__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of OperateCabinetControl_Goal */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("xczs_inspection_robot_interfaces.action._operate_cabinet_control");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "OperateCabinetControl_Goal");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_Goal * ros_message = (xczs_inspection_robot_interfaces__action__OperateCabinetControl_Goal *)raw_ros_message;
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
  {  // command
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->command);
    {
      int rc = PyObject_SetAttrString(_pymessage, "command", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // target_state
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->target_state.data,
      strlen(ros_message->target_state.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "target_state", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // target_position
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->target_position);
    {
      int rc = PyObject_SetAttrString(_pymessage, "target_position", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // use_target_position
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->use_target_position ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "use_target_position", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // force
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->force);
    {
      int rc = PyObject_SetAttrString(_pymessage, "force", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // navigate_to_staging_pose
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->navigate_to_staging_pose ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "navigate_to_staging_pose", field);
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
// #include "xczs_inspection_robot_interfaces/action/detail/operate_cabinet_control__struct.h"
// already included above
// #include "xczs_inspection_robot_interfaces/action/detail/operate_cabinet_control__functions.h"

// already included above
// #include "rosidl_runtime_c/string.h"
// already included above
// #include "rosidl_runtime_c/string_functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool xczs_inspection_robot_interfaces__action__operate_cabinet_control__result__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[94];
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
    assert(strncmp("xczs_inspection_robot_interfaces.action._operate_cabinet_control.OperateCabinetControl_Result", full_classname_dest, 93) == 0);
  }
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result * ros_message = _ros_message;
  {  // success
    PyObject * field = PyObject_GetAttrString(_pymsg, "success");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->success = (Py_True == field);
    Py_DECREF(field);
  }
  {  // error_code
    PyObject * field = PyObject_GetAttrString(_pymsg, "error_code");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->error_code = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // message
    PyObject * field = PyObject_GetAttrString(_pymsg, "message");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->message, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // initial_position
    PyObject * field = PyObject_GetAttrString(_pymsg, "initial_position");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->initial_position = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // final_position
    PyObject * field = PyObject_GetAttrString(_pymsg, "final_position");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->final_position = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // peak_position
    PyObject * field = PyObject_GetAttrString(_pymsg, "peak_position");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->peak_position = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // final_state
    PyObject * field = PyObject_GetAttrString(_pymsg, "final_state");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->final_state, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // requested_force
    PyObject * field = PyObject_GetAttrString(_pymsg, "requested_force");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->requested_force = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // estimated_force
    PyObject * field = PyObject_GetAttrString(_pymsg, "estimated_force");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->estimated_force = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // button_triggered
    PyObject * field = PyObject_GetAttrString(_pymsg, "button_triggered");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->button_triggered = (Py_True == field);
    Py_DECREF(field);
  }
  {  // validation_performed
    PyObject * field = PyObject_GetAttrString(_pymsg, "validation_performed");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->validation_performed = (Py_True == field);
    Py_DECREF(field);
  }
  {  // operation_executed
    PyObject * field = PyObject_GetAttrString(_pymsg, "operation_executed");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->operation_executed = (Py_True == field);
    Py_DECREF(field);
  }
  {  // diagnostic_stage
    PyObject * field = PyObject_GetAttrString(_pymsg, "diagnostic_stage");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->diagnostic_stage, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // path_fraction
    PyObject * field = PyObject_GetAttrString(_pymsg, "path_fraction");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->path_fraction = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // required_fraction
    PyObject * field = PyObject_GetAttrString(_pymsg, "required_fraction");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->required_fraction = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // moveit_error_code
    PyObject * field = PyObject_GetAttrString(_pymsg, "moveit_error_code");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->moveit_error_code = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // policy_reason
    PyObject * field = PyObject_GetAttrString(_pymsg, "policy_reason");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->policy_reason, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // failure_reason
    PyObject * field = PyObject_GetAttrString(_pymsg, "failure_reason");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->failure_reason, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // physical_outcome_confirmed
    PyObject * field = PyObject_GetAttrString(_pymsg, "physical_outcome_confirmed");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->physical_outcome_confirmed = (Py_True == field);
    Py_DECREF(field);
  }
  {  // final_state_verified
    PyObject * field = PyObject_GetAttrString(_pymsg, "final_state_verified");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->final_state_verified = (Py_True == field);
    Py_DECREF(field);
  }
  {  // transport_succeeded
    PyObject * field = PyObject_GetAttrString(_pymsg, "transport_succeeded");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->transport_succeeded = (Py_True == field);
    Py_DECREF(field);
  }
  {  // recovery_succeeded
    PyObject * field = PyObject_GetAttrString(_pymsg, "recovery_succeeded");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->recovery_succeeded = (Py_True == field);
    Py_DECREF(field);
  }
  {  // grasp_released
    PyObject * field = PyObject_GetAttrString(_pymsg, "grasp_released");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->grasp_released = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * xczs_inspection_robot_interfaces__action__operate_cabinet_control__result__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of OperateCabinetControl_Result */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("xczs_inspection_robot_interfaces.action._operate_cabinet_control");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "OperateCabinetControl_Result");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result * ros_message = (xczs_inspection_robot_interfaces__action__OperateCabinetControl_Result *)raw_ros_message;
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
  {  // error_code
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->error_code);
    {
      int rc = PyObject_SetAttrString(_pymessage, "error_code", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // message
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->message.data,
      strlen(ros_message->message.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "message", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // initial_position
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->initial_position);
    {
      int rc = PyObject_SetAttrString(_pymessage, "initial_position", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // final_position
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->final_position);
    {
      int rc = PyObject_SetAttrString(_pymessage, "final_position", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // peak_position
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->peak_position);
    {
      int rc = PyObject_SetAttrString(_pymessage, "peak_position", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // final_state
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->final_state.data,
      strlen(ros_message->final_state.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "final_state", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // requested_force
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->requested_force);
    {
      int rc = PyObject_SetAttrString(_pymessage, "requested_force", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // estimated_force
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->estimated_force);
    {
      int rc = PyObject_SetAttrString(_pymessage, "estimated_force", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // button_triggered
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->button_triggered ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "button_triggered", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // validation_performed
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->validation_performed ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "validation_performed", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // operation_executed
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->operation_executed ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "operation_executed", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // diagnostic_stage
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->diagnostic_stage.data,
      strlen(ros_message->diagnostic_stage.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "diagnostic_stage", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // path_fraction
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->path_fraction);
    {
      int rc = PyObject_SetAttrString(_pymessage, "path_fraction", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // required_fraction
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->required_fraction);
    {
      int rc = PyObject_SetAttrString(_pymessage, "required_fraction", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // moveit_error_code
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->moveit_error_code);
    {
      int rc = PyObject_SetAttrString(_pymessage, "moveit_error_code", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // policy_reason
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->policy_reason.data,
      strlen(ros_message->policy_reason.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "policy_reason", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // failure_reason
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->failure_reason.data,
      strlen(ros_message->failure_reason.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "failure_reason", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // physical_outcome_confirmed
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->physical_outcome_confirmed ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "physical_outcome_confirmed", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // final_state_verified
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->final_state_verified ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "final_state_verified", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // transport_succeeded
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->transport_succeeded ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "transport_succeeded", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // recovery_succeeded
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->recovery_succeeded ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "recovery_succeeded", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // grasp_released
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->grasp_released ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "grasp_released", field);
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
// #include "xczs_inspection_robot_interfaces/action/detail/operate_cabinet_control__struct.h"
// already included above
// #include "xczs_inspection_robot_interfaces/action/detail/operate_cabinet_control__functions.h"

// already included above
// #include "rosidl_runtime_c/string.h"
// already included above
// #include "rosidl_runtime_c/string_functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool xczs_inspection_robot_interfaces__action__operate_cabinet_control__feedback__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[96];
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
    assert(strncmp("xczs_inspection_robot_interfaces.action._operate_cabinet_control.OperateCabinetControl_Feedback", full_classname_dest, 95) == 0);
  }
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_Feedback * ros_message = _ros_message;
  {  // phase
    PyObject * field = PyObject_GetAttrString(_pymsg, "phase");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->phase = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // progress
    PyObject * field = PyObject_GetAttrString(_pymsg, "progress");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->progress = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // current_position
    PyObject * field = PyObject_GetAttrString(_pymsg, "current_position");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->current_position = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // target_position
    PyObject * field = PyObject_GetAttrString(_pymsg, "target_position");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->target_position = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // current_state
    PyObject * field = PyObject_GetAttrString(_pymsg, "current_state");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->current_state, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // message
    PyObject * field = PyObject_GetAttrString(_pymsg, "message");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->message, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * xczs_inspection_robot_interfaces__action__operate_cabinet_control__feedback__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of OperateCabinetControl_Feedback */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("xczs_inspection_robot_interfaces.action._operate_cabinet_control");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "OperateCabinetControl_Feedback");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_Feedback * ros_message = (xczs_inspection_robot_interfaces__action__OperateCabinetControl_Feedback *)raw_ros_message;
  {  // phase
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->phase);
    {
      int rc = PyObject_SetAttrString(_pymessage, "phase", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // progress
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->progress);
    {
      int rc = PyObject_SetAttrString(_pymessage, "progress", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // current_position
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->current_position);
    {
      int rc = PyObject_SetAttrString(_pymessage, "current_position", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // target_position
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->target_position);
    {
      int rc = PyObject_SetAttrString(_pymessage, "target_position", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // current_state
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->current_state.data,
      strlen(ros_message->current_state.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "current_state", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // message
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->message.data,
      strlen(ros_message->message.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "message", field);
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
// #include "xczs_inspection_robot_interfaces/action/detail/operate_cabinet_control__struct.h"
// already included above
// #include "xczs_inspection_robot_interfaces/action/detail/operate_cabinet_control__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool unique_identifier_msgs__msg__uuid__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * unique_identifier_msgs__msg__uuid__convert_to_py(void * raw_ros_message);
bool xczs_inspection_robot_interfaces__action__operate_cabinet_control__goal__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * xczs_inspection_robot_interfaces__action__operate_cabinet_control__goal__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool xczs_inspection_robot_interfaces__action__operate_cabinet_control__send_goal__request__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[104];
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
    assert(strncmp("xczs_inspection_robot_interfaces.action._operate_cabinet_control.OperateCabinetControl_SendGoal_Request", full_classname_dest, 103) == 0);
  }
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Request * ros_message = _ros_message;
  {  // goal_id
    PyObject * field = PyObject_GetAttrString(_pymsg, "goal_id");
    if (!field) {
      return false;
    }
    if (!unique_identifier_msgs__msg__uuid__convert_from_py(field, &ros_message->goal_id)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // goal
    PyObject * field = PyObject_GetAttrString(_pymsg, "goal");
    if (!field) {
      return false;
    }
    if (!xczs_inspection_robot_interfaces__action__operate_cabinet_control__goal__convert_from_py(field, &ros_message->goal)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * xczs_inspection_robot_interfaces__action__operate_cabinet_control__send_goal__request__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of OperateCabinetControl_SendGoal_Request */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("xczs_inspection_robot_interfaces.action._operate_cabinet_control");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "OperateCabinetControl_SendGoal_Request");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Request * ros_message = (xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Request *)raw_ros_message;
  {  // goal_id
    PyObject * field = NULL;
    field = unique_identifier_msgs__msg__uuid__convert_to_py(&ros_message->goal_id);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "goal_id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // goal
    PyObject * field = NULL;
    field = xczs_inspection_robot_interfaces__action__operate_cabinet_control__goal__convert_to_py(&ros_message->goal);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "goal", field);
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
// #include "xczs_inspection_robot_interfaces/action/detail/operate_cabinet_control__struct.h"
// already included above
// #include "xczs_inspection_robot_interfaces/action/detail/operate_cabinet_control__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool builtin_interfaces__msg__time__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * builtin_interfaces__msg__time__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool xczs_inspection_robot_interfaces__action__operate_cabinet_control__send_goal__response__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[105];
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
    assert(strncmp("xczs_inspection_robot_interfaces.action._operate_cabinet_control.OperateCabinetControl_SendGoal_Response", full_classname_dest, 104) == 0);
  }
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Response * ros_message = _ros_message;
  {  // accepted
    PyObject * field = PyObject_GetAttrString(_pymsg, "accepted");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->accepted = (Py_True == field);
    Py_DECREF(field);
  }
  {  // stamp
    PyObject * field = PyObject_GetAttrString(_pymsg, "stamp");
    if (!field) {
      return false;
    }
    if (!builtin_interfaces__msg__time__convert_from_py(field, &ros_message->stamp)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * xczs_inspection_robot_interfaces__action__operate_cabinet_control__send_goal__response__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of OperateCabinetControl_SendGoal_Response */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("xczs_inspection_robot_interfaces.action._operate_cabinet_control");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "OperateCabinetControl_SendGoal_Response");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Response * ros_message = (xczs_inspection_robot_interfaces__action__OperateCabinetControl_SendGoal_Response *)raw_ros_message;
  {  // accepted
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->accepted ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "accepted", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // stamp
    PyObject * field = NULL;
    field = builtin_interfaces__msg__time__convert_to_py(&ros_message->stamp);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "stamp", field);
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
// #include "xczs_inspection_robot_interfaces/action/detail/operate_cabinet_control__struct.h"
// already included above
// #include "xczs_inspection_robot_interfaces/action/detail/operate_cabinet_control__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool unique_identifier_msgs__msg__uuid__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * unique_identifier_msgs__msg__uuid__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool xczs_inspection_robot_interfaces__action__operate_cabinet_control__get_result__request__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[105];
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
    assert(strncmp("xczs_inspection_robot_interfaces.action._operate_cabinet_control.OperateCabinetControl_GetResult_Request", full_classname_dest, 104) == 0);
  }
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Request * ros_message = _ros_message;
  {  // goal_id
    PyObject * field = PyObject_GetAttrString(_pymsg, "goal_id");
    if (!field) {
      return false;
    }
    if (!unique_identifier_msgs__msg__uuid__convert_from_py(field, &ros_message->goal_id)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * xczs_inspection_robot_interfaces__action__operate_cabinet_control__get_result__request__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of OperateCabinetControl_GetResult_Request */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("xczs_inspection_robot_interfaces.action._operate_cabinet_control");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "OperateCabinetControl_GetResult_Request");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Request * ros_message = (xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Request *)raw_ros_message;
  {  // goal_id
    PyObject * field = NULL;
    field = unique_identifier_msgs__msg__uuid__convert_to_py(&ros_message->goal_id);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "goal_id", field);
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
// #include "xczs_inspection_robot_interfaces/action/detail/operate_cabinet_control__struct.h"
// already included above
// #include "xczs_inspection_robot_interfaces/action/detail/operate_cabinet_control__functions.h"

bool xczs_inspection_robot_interfaces__action__operate_cabinet_control__result__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * xczs_inspection_robot_interfaces__action__operate_cabinet_control__result__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool xczs_inspection_robot_interfaces__action__operate_cabinet_control__get_result__response__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[106];
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
    assert(strncmp("xczs_inspection_robot_interfaces.action._operate_cabinet_control.OperateCabinetControl_GetResult_Response", full_classname_dest, 105) == 0);
  }
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Response * ros_message = _ros_message;
  {  // status
    PyObject * field = PyObject_GetAttrString(_pymsg, "status");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->status = (int8_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // result
    PyObject * field = PyObject_GetAttrString(_pymsg, "result");
    if (!field) {
      return false;
    }
    if (!xczs_inspection_robot_interfaces__action__operate_cabinet_control__result__convert_from_py(field, &ros_message->result)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * xczs_inspection_robot_interfaces__action__operate_cabinet_control__get_result__response__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of OperateCabinetControl_GetResult_Response */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("xczs_inspection_robot_interfaces.action._operate_cabinet_control");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "OperateCabinetControl_GetResult_Response");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Response * ros_message = (xczs_inspection_robot_interfaces__action__OperateCabinetControl_GetResult_Response *)raw_ros_message;
  {  // status
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->status);
    {
      int rc = PyObject_SetAttrString(_pymessage, "status", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // result
    PyObject * field = NULL;
    field = xczs_inspection_robot_interfaces__action__operate_cabinet_control__result__convert_to_py(&ros_message->result);
    if (!field) {
      return NULL;
    }
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
// #include "xczs_inspection_robot_interfaces/action/detail/operate_cabinet_control__struct.h"
// already included above
// #include "xczs_inspection_robot_interfaces/action/detail/operate_cabinet_control__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool unique_identifier_msgs__msg__uuid__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * unique_identifier_msgs__msg__uuid__convert_to_py(void * raw_ros_message);
bool xczs_inspection_robot_interfaces__action__operate_cabinet_control__feedback__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * xczs_inspection_robot_interfaces__action__operate_cabinet_control__feedback__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool xczs_inspection_robot_interfaces__action__operate_cabinet_control__feedback_message__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[103];
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
    assert(strncmp("xczs_inspection_robot_interfaces.action._operate_cabinet_control.OperateCabinetControl_FeedbackMessage", full_classname_dest, 102) == 0);
  }
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_FeedbackMessage * ros_message = _ros_message;
  {  // goal_id
    PyObject * field = PyObject_GetAttrString(_pymsg, "goal_id");
    if (!field) {
      return false;
    }
    if (!unique_identifier_msgs__msg__uuid__convert_from_py(field, &ros_message->goal_id)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // feedback
    PyObject * field = PyObject_GetAttrString(_pymsg, "feedback");
    if (!field) {
      return false;
    }
    if (!xczs_inspection_robot_interfaces__action__operate_cabinet_control__feedback__convert_from_py(field, &ros_message->feedback)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * xczs_inspection_robot_interfaces__action__operate_cabinet_control__feedback_message__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of OperateCabinetControl_FeedbackMessage */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("xczs_inspection_robot_interfaces.action._operate_cabinet_control");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "OperateCabinetControl_FeedbackMessage");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  xczs_inspection_robot_interfaces__action__OperateCabinetControl_FeedbackMessage * ros_message = (xczs_inspection_robot_interfaces__action__OperateCabinetControl_FeedbackMessage *)raw_ros_message;
  {  // goal_id
    PyObject * field = NULL;
    field = unique_identifier_msgs__msg__uuid__convert_to_py(&ros_message->goal_id);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "goal_id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // feedback
    PyObject * field = NULL;
    field = xczs_inspection_robot_interfaces__action__operate_cabinet_control__feedback__convert_to_py(&ros_message->feedback);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "feedback", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
