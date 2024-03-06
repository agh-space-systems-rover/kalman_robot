// generated from rosidl_generator_py/resource/_idl_pkg_typesupport_entry_point.c.em
// generated code does not contain a copyright notice
#include <Python.h>

static PyMethodDef service_based_nav2_controller_srvs__methods[] = {
  {NULL, NULL, 0, NULL}  /* sentinel */
};

static struct PyModuleDef service_based_nav2_controller_srvs__module = {
  PyModuleDef_HEAD_INIT,
  "_service_based_nav2_controller_srvs_support",
  "_service_based_nav2_controller_srvs_doc",
  -1,  /* -1 means that the module keeps state in global variables */
  service_based_nav2_controller_srvs__methods,
  NULL,
  NULL,
  NULL,
  NULL,
};

#include <stdbool.h>
#include <stdint.h>
#include "rosidl_runtime_c/visibility_control.h"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_runtime_c/action_type_support_struct.h"
#include "service_based_nav2_controller_srvs/srv/detail/compute_velocity_commands__type_support.h"
#include "service_based_nav2_controller_srvs/srv/detail/compute_velocity_commands__struct.h"
#include "service_based_nav2_controller_srvs/srv/detail/compute_velocity_commands__functions.h"

static void * service_based_nav2_controller_srvs__srv__compute_velocity_commands__request__create_ros_message(void)
{
  return service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request__create();
}

static void service_based_nav2_controller_srvs__srv__compute_velocity_commands__request__destroy_ros_message(void * raw_ros_message)
{
  service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request * ros_message = (service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request *)raw_ros_message;
  service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request__destroy(ros_message);
}

ROSIDL_GENERATOR_C_IMPORT
bool service_based_nav2_controller_srvs__srv__compute_velocity_commands__request__convert_from_py(PyObject * _pymsg, void * ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * service_based_nav2_controller_srvs__srv__compute_velocity_commands__request__convert_to_py(void * raw_ros_message);


ROSIDL_GENERATOR_C_IMPORT
const rosidl_message_type_support_t *
ROSIDL_GET_MSG_TYPE_SUPPORT(service_based_nav2_controller_srvs, srv, ComputeVelocityCommands_Request);

int8_t
_register_msg_type__srv__compute_velocity_commands__request(PyObject * pymodule)
{
  int8_t err;

  PyObject * pyobject_create_ros_message = NULL;
  pyobject_create_ros_message = PyCapsule_New(
    (void *)&service_based_nav2_controller_srvs__srv__compute_velocity_commands__request__create_ros_message,
    NULL, NULL);
  if (!pyobject_create_ros_message) {
    // previously added objects will be removed when the module is destroyed
    return -1;
  }
  err = PyModule_AddObject(
    pymodule,
    "create_ros_message_msg__srv__compute_velocity_commands__request",
    pyobject_create_ros_message);
  if (err) {
    // the created capsule needs to be decremented
    Py_XDECREF(pyobject_create_ros_message);
    // previously added objects will be removed when the module is destroyed
    return err;
  }

  PyObject * pyobject_destroy_ros_message = NULL;
  pyobject_destroy_ros_message = PyCapsule_New(
    (void *)&service_based_nav2_controller_srvs__srv__compute_velocity_commands__request__destroy_ros_message,
    NULL, NULL);
  if (!pyobject_destroy_ros_message) {
    // previously added objects will be removed when the module is destroyed
    return -1;
  }
  err = PyModule_AddObject(
    pymodule,
    "destroy_ros_message_msg__srv__compute_velocity_commands__request",
    pyobject_destroy_ros_message);
  if (err) {
    // the created capsule needs to be decremented
    Py_XDECREF(pyobject_destroy_ros_message);
    // previously added objects will be removed when the module is destroyed
    return err;
  }

  PyObject * pyobject_convert_from_py = NULL;
  pyobject_convert_from_py = PyCapsule_New(
    (void *)&service_based_nav2_controller_srvs__srv__compute_velocity_commands__request__convert_from_py,
    NULL, NULL);
  if (!pyobject_convert_from_py) {
    // previously added objects will be removed when the module is destroyed
    return -1;
  }
  err = PyModule_AddObject(
    pymodule,
    "convert_from_py_msg__srv__compute_velocity_commands__request",
    pyobject_convert_from_py);
  if (err) {
    // the created capsule needs to be decremented
    Py_XDECREF(pyobject_convert_from_py);
    // previously added objects will be removed when the module is destroyed
    return err;
  }

  PyObject * pyobject_convert_to_py = NULL;
  pyobject_convert_to_py = PyCapsule_New(
    (void *)&service_based_nav2_controller_srvs__srv__compute_velocity_commands__request__convert_to_py,
    NULL, NULL);
  if (!pyobject_convert_to_py) {
    // previously added objects will be removed when the module is destroyed
    return -1;
  }
  err = PyModule_AddObject(
    pymodule,
    "convert_to_py_msg__srv__compute_velocity_commands__request",
    pyobject_convert_to_py);
  if (err) {
    // the created capsule needs to be decremented
    Py_XDECREF(pyobject_convert_to_py);
    // previously added objects will be removed when the module is destroyed
    return err;
  }

  PyObject * pyobject_type_support = NULL;
  pyobject_type_support = PyCapsule_New(
    (void *)ROSIDL_GET_MSG_TYPE_SUPPORT(service_based_nav2_controller_srvs, srv, ComputeVelocityCommands_Request),
    NULL, NULL);
  if (!pyobject_type_support) {
    // previously added objects will be removed when the module is destroyed
    return -1;
  }
  err = PyModule_AddObject(
    pymodule,
    "type_support_msg__srv__compute_velocity_commands__request",
    pyobject_type_support);
  if (err) {
    // the created capsule needs to be decremented
    Py_XDECREF(pyobject_type_support);
    // previously added objects will be removed when the module is destroyed
    return err;
  }
  return 0;
}

// already included above
// #include <stdbool.h>
// already included above
// #include <stdint.h>
// already included above
// #include "rosidl_runtime_c/visibility_control.h"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "rosidl_runtime_c/action_type_support_struct.h"
// already included above
// #include "service_based_nav2_controller_srvs/srv/detail/compute_velocity_commands__type_support.h"
// already included above
// #include "service_based_nav2_controller_srvs/srv/detail/compute_velocity_commands__struct.h"
// already included above
// #include "service_based_nav2_controller_srvs/srv/detail/compute_velocity_commands__functions.h"

static void * service_based_nav2_controller_srvs__srv__compute_velocity_commands__response__create_ros_message(void)
{
  return service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response__create();
}

static void service_based_nav2_controller_srvs__srv__compute_velocity_commands__response__destroy_ros_message(void * raw_ros_message)
{
  service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response * ros_message = (service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response *)raw_ros_message;
  service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response__destroy(ros_message);
}

ROSIDL_GENERATOR_C_IMPORT
bool service_based_nav2_controller_srvs__srv__compute_velocity_commands__response__convert_from_py(PyObject * _pymsg, void * ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * service_based_nav2_controller_srvs__srv__compute_velocity_commands__response__convert_to_py(void * raw_ros_message);


ROSIDL_GENERATOR_C_IMPORT
const rosidl_message_type_support_t *
ROSIDL_GET_MSG_TYPE_SUPPORT(service_based_nav2_controller_srvs, srv, ComputeVelocityCommands_Response);

int8_t
_register_msg_type__srv__compute_velocity_commands__response(PyObject * pymodule)
{
  int8_t err;

  PyObject * pyobject_create_ros_message = NULL;
  pyobject_create_ros_message = PyCapsule_New(
    (void *)&service_based_nav2_controller_srvs__srv__compute_velocity_commands__response__create_ros_message,
    NULL, NULL);
  if (!pyobject_create_ros_message) {
    // previously added objects will be removed when the module is destroyed
    return -1;
  }
  err = PyModule_AddObject(
    pymodule,
    "create_ros_message_msg__srv__compute_velocity_commands__response",
    pyobject_create_ros_message);
  if (err) {
    // the created capsule needs to be decremented
    Py_XDECREF(pyobject_create_ros_message);
    // previously added objects will be removed when the module is destroyed
    return err;
  }

  PyObject * pyobject_destroy_ros_message = NULL;
  pyobject_destroy_ros_message = PyCapsule_New(
    (void *)&service_based_nav2_controller_srvs__srv__compute_velocity_commands__response__destroy_ros_message,
    NULL, NULL);
  if (!pyobject_destroy_ros_message) {
    // previously added objects will be removed when the module is destroyed
    return -1;
  }
  err = PyModule_AddObject(
    pymodule,
    "destroy_ros_message_msg__srv__compute_velocity_commands__response",
    pyobject_destroy_ros_message);
  if (err) {
    // the created capsule needs to be decremented
    Py_XDECREF(pyobject_destroy_ros_message);
    // previously added objects will be removed when the module is destroyed
    return err;
  }

  PyObject * pyobject_convert_from_py = NULL;
  pyobject_convert_from_py = PyCapsule_New(
    (void *)&service_based_nav2_controller_srvs__srv__compute_velocity_commands__response__convert_from_py,
    NULL, NULL);
  if (!pyobject_convert_from_py) {
    // previously added objects will be removed when the module is destroyed
    return -1;
  }
  err = PyModule_AddObject(
    pymodule,
    "convert_from_py_msg__srv__compute_velocity_commands__response",
    pyobject_convert_from_py);
  if (err) {
    // the created capsule needs to be decremented
    Py_XDECREF(pyobject_convert_from_py);
    // previously added objects will be removed when the module is destroyed
    return err;
  }

  PyObject * pyobject_convert_to_py = NULL;
  pyobject_convert_to_py = PyCapsule_New(
    (void *)&service_based_nav2_controller_srvs__srv__compute_velocity_commands__response__convert_to_py,
    NULL, NULL);
  if (!pyobject_convert_to_py) {
    // previously added objects will be removed when the module is destroyed
    return -1;
  }
  err = PyModule_AddObject(
    pymodule,
    "convert_to_py_msg__srv__compute_velocity_commands__response",
    pyobject_convert_to_py);
  if (err) {
    // the created capsule needs to be decremented
    Py_XDECREF(pyobject_convert_to_py);
    // previously added objects will be removed when the module is destroyed
    return err;
  }

  PyObject * pyobject_type_support = NULL;
  pyobject_type_support = PyCapsule_New(
    (void *)ROSIDL_GET_MSG_TYPE_SUPPORT(service_based_nav2_controller_srvs, srv, ComputeVelocityCommands_Response),
    NULL, NULL);
  if (!pyobject_type_support) {
    // previously added objects will be removed when the module is destroyed
    return -1;
  }
  err = PyModule_AddObject(
    pymodule,
    "type_support_msg__srv__compute_velocity_commands__response",
    pyobject_type_support);
  if (err) {
    // the created capsule needs to be decremented
    Py_XDECREF(pyobject_type_support);
    // previously added objects will be removed when the module is destroyed
    return err;
  }
  return 0;
}

ROSIDL_GENERATOR_C_IMPORT
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, service_based_nav2_controller_srvs, srv, ComputeVelocityCommands)();

int8_t
_register_srv_type__srv__compute_velocity_commands(PyObject * pymodule)
{
  int8_t err;
  PyObject * pyobject_type_support = NULL;
  pyobject_type_support = PyCapsule_New(
    (void *)ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, service_based_nav2_controller_srvs, srv, ComputeVelocityCommands)(),
    NULL, NULL);
  if (!pyobject_type_support) {
    // previously added objects will be removed when the module is destroyed
    return -1;
  }
  err = PyModule_AddObject(
    pymodule,
    "type_support_srv__srv__compute_velocity_commands",
    pyobject_type_support);
  if (err) {
    // the created capsule needs to be decremented
    Py_XDECREF(pyobject_type_support);
    // previously added objects will be removed when the module is destroyed
    return err;
  }
  return 0;
}

PyMODINIT_FUNC
PyInit_service_based_nav2_controller_srvs_s__rosidl_typesupport_introspection_c(void)
{
  PyObject * pymodule = NULL;
  pymodule = PyModule_Create(&service_based_nav2_controller_srvs__module);
  if (!pymodule) {
    return NULL;
  }
  int8_t err;

  err = _register_msg_type__srv__compute_velocity_commands__request(pymodule);
  if (err) {
    Py_XDECREF(pymodule);
    return NULL;
  }

  err = _register_msg_type__srv__compute_velocity_commands__response(pymodule);
  if (err) {
    Py_XDECREF(pymodule);
    return NULL;
  }

  err = _register_srv_type__srv__compute_velocity_commands(pymodule);
  if (err) {
    Py_XDECREF(pymodule);
    return NULL;
  }

  return pymodule;
}
