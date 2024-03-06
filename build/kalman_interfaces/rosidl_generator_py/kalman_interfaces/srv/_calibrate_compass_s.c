// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from kalman_interfaces:srv/CalibrateCompass.idl
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
#include "kalman_interfaces/srv/detail/calibrate_compass__struct.h"
#include "kalman_interfaces/srv/detail/calibrate_compass__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool kalman_interfaces__srv__calibrate_compass__request__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[66];
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
    assert(strncmp("kalman_interfaces.srv._calibrate_compass.CalibrateCompass_Request", full_classname_dest, 65) == 0);
  }
  kalman_interfaces__srv__CalibrateCompass_Request * ros_message = _ros_message;
  {  // duration
    PyObject * field = PyObject_GetAttrString(_pymsg, "duration");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->duration = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // angular_velocity
    PyObject * field = PyObject_GetAttrString(_pymsg, "angular_velocity");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->angular_velocity = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * kalman_interfaces__srv__calibrate_compass__request__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of CalibrateCompass_Request */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("kalman_interfaces.srv._calibrate_compass");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "CalibrateCompass_Request");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  kalman_interfaces__srv__CalibrateCompass_Request * ros_message = (kalman_interfaces__srv__CalibrateCompass_Request *)raw_ros_message;
  {  // duration
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->duration);
    {
      int rc = PyObject_SetAttrString(_pymessage, "duration", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // angular_velocity
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->angular_velocity);
    {
      int rc = PyObject_SetAttrString(_pymessage, "angular_velocity", field);
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
// #include "kalman_interfaces/srv/detail/calibrate_compass__struct.h"
// already included above
// #include "kalman_interfaces/srv/detail/calibrate_compass__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool kalman_interfaces__srv__calibrate_compass__response__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[67];
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
    assert(strncmp("kalman_interfaces.srv._calibrate_compass.CalibrateCompass_Response", full_classname_dest, 66) == 0);
  }
  kalman_interfaces__srv__CalibrateCompass_Response * ros_message = _ros_message;
  {  // success
    PyObject * field = PyObject_GetAttrString(_pymsg, "success");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->success = (Py_True == field);
    Py_DECREF(field);
  }
  {  // cc_mag_field
    PyObject * field = PyObject_GetAttrString(_pymsg, "cc_mag_field");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->cc_mag_field = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // cc_offset0
    PyObject * field = PyObject_GetAttrString(_pymsg, "cc_offset0");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->cc_offset0 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // cc_offset1
    PyObject * field = PyObject_GetAttrString(_pymsg, "cc_offset1");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->cc_offset1 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // cc_offset2
    PyObject * field = PyObject_GetAttrString(_pymsg, "cc_offset2");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->cc_offset2 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // cc_gain0
    PyObject * field = PyObject_GetAttrString(_pymsg, "cc_gain0");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->cc_gain0 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // cc_gain1
    PyObject * field = PyObject_GetAttrString(_pymsg, "cc_gain1");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->cc_gain1 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // cc_gain2
    PyObject * field = PyObject_GetAttrString(_pymsg, "cc_gain2");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->cc_gain2 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // cc_t0
    PyObject * field = PyObject_GetAttrString(_pymsg, "cc_t0");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->cc_t0 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // cc_t1
    PyObject * field = PyObject_GetAttrString(_pymsg, "cc_t1");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->cc_t1 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // cc_t2
    PyObject * field = PyObject_GetAttrString(_pymsg, "cc_t2");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->cc_t2 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // cc_t3
    PyObject * field = PyObject_GetAttrString(_pymsg, "cc_t3");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->cc_t3 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // cc_t4
    PyObject * field = PyObject_GetAttrString(_pymsg, "cc_t4");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->cc_t4 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // cc_t5
    PyObject * field = PyObject_GetAttrString(_pymsg, "cc_t5");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->cc_t5 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * kalman_interfaces__srv__calibrate_compass__response__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of CalibrateCompass_Response */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("kalman_interfaces.srv._calibrate_compass");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "CalibrateCompass_Response");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  kalman_interfaces__srv__CalibrateCompass_Response * ros_message = (kalman_interfaces__srv__CalibrateCompass_Response *)raw_ros_message;
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
  {  // cc_mag_field
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->cc_mag_field);
    {
      int rc = PyObject_SetAttrString(_pymessage, "cc_mag_field", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // cc_offset0
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->cc_offset0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "cc_offset0", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // cc_offset1
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->cc_offset1);
    {
      int rc = PyObject_SetAttrString(_pymessage, "cc_offset1", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // cc_offset2
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->cc_offset2);
    {
      int rc = PyObject_SetAttrString(_pymessage, "cc_offset2", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // cc_gain0
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->cc_gain0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "cc_gain0", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // cc_gain1
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->cc_gain1);
    {
      int rc = PyObject_SetAttrString(_pymessage, "cc_gain1", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // cc_gain2
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->cc_gain2);
    {
      int rc = PyObject_SetAttrString(_pymessage, "cc_gain2", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // cc_t0
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->cc_t0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "cc_t0", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // cc_t1
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->cc_t1);
    {
      int rc = PyObject_SetAttrString(_pymessage, "cc_t1", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // cc_t2
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->cc_t2);
    {
      int rc = PyObject_SetAttrString(_pymessage, "cc_t2", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // cc_t3
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->cc_t3);
    {
      int rc = PyObject_SetAttrString(_pymessage, "cc_t3", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // cc_t4
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->cc_t4);
    {
      int rc = PyObject_SetAttrString(_pymessage, "cc_t4", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // cc_t5
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->cc_t5);
    {
      int rc = PyObject_SetAttrString(_pymessage, "cc_t5", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
