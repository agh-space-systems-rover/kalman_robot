// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from unity_rs_publisher_msgs:msg/CameraMetadata.idl
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
#include "unity_rs_publisher_msgs/msg/detail/camera_metadata__struct.h"
#include "unity_rs_publisher_msgs/msg/detail/camera_metadata__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool unity_rs_publisher_msgs__msg__camera_metadata__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[60];
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
    assert(strncmp("unity_rs_publisher_msgs.msg._camera_metadata.CameraMetadata", full_classname_dest, 59) == 0);
  }
  unity_rs_publisher_msgs__msg__CameraMetadata * ros_message = _ros_message;
  {  // width
    PyObject * field = PyObject_GetAttrString(_pymsg, "width");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->width = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // height
    PyObject * field = PyObject_GetAttrString(_pymsg, "height");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->height = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // depth_min
    PyObject * field = PyObject_GetAttrString(_pymsg, "depth_min");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->depth_min = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // depth_max
    PyObject * field = PyObject_GetAttrString(_pymsg, "depth_max");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->depth_max = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // fx
    PyObject * field = PyObject_GetAttrString(_pymsg, "fx");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->fx = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // fy
    PyObject * field = PyObject_GetAttrString(_pymsg, "fy");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->fy = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // cx
    PyObject * field = PyObject_GetAttrString(_pymsg, "cx");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->cx = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // cy
    PyObject * field = PyObject_GetAttrString(_pymsg, "cy");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->cy = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * unity_rs_publisher_msgs__msg__camera_metadata__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of CameraMetadata */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("unity_rs_publisher_msgs.msg._camera_metadata");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "CameraMetadata");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  unity_rs_publisher_msgs__msg__CameraMetadata * ros_message = (unity_rs_publisher_msgs__msg__CameraMetadata *)raw_ros_message;
  {  // width
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->width);
    {
      int rc = PyObject_SetAttrString(_pymessage, "width", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // height
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->height);
    {
      int rc = PyObject_SetAttrString(_pymessage, "height", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // depth_min
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->depth_min);
    {
      int rc = PyObject_SetAttrString(_pymessage, "depth_min", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // depth_max
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->depth_max);
    {
      int rc = PyObject_SetAttrString(_pymessage, "depth_max", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // fx
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->fx);
    {
      int rc = PyObject_SetAttrString(_pymessage, "fx", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // fy
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->fy);
    {
      int rc = PyObject_SetAttrString(_pymessage, "fy", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // cx
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->cx);
    {
      int rc = PyObject_SetAttrString(_pymessage, "cx", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // cy
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->cy);
    {
      int rc = PyObject_SetAttrString(_pymessage, "cy", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
