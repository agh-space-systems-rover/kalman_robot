// Temporary stub for compatibility with Humble builds - not suitable for runtime.

#pragma once

#include <cstdint>
#include <string>
#include <memory>

namespace moveit_msgs {
namespace srv {

struct ServoCommandType_Request
{
  using _command_type_type = int8_t;
  _command_type_type command_type;

  enum
  {
    JOINT_JOG = 0,
    TWIST = 1
  };

  using SharedPtr = std::shared_ptr<ServoCommandType_Request>;
};

struct ServoCommandType_Response
{
  bool success;
  std::string status_message;

  using SharedPtr = std::shared_ptr<ServoCommandType_Response>;
};

struct ServoCommandType
{
  using Request = moveit_msgs::srv::ServoCommandType_Request;
  using Response = moveit_msgs::srv::ServoCommandType_Response;
};

}  // namespace srv
}  // namespace moveit_msgs

// Forward declarations for type support structures
struct rosidl_service_type_support_t;
struct rosidl_message_type_support_t;

// Provide minimal type support function declarations
namespace rosidl_typesupport_cpp {

template<typename ServiceT>
const rosidl_service_type_support_t * get_service_type_support_handle();

template<typename MessageT>
const rosidl_message_type_support_t * get_message_type_support_handle();

// Explicit template instantiation declarations
template<>
const rosidl_service_type_support_t * 
get_service_type_support_handle<moveit_msgs::srv::ServoCommandType>();

template<>
const rosidl_message_type_support_t * 
get_message_type_support_handle<moveit_msgs::srv::ServoCommandType_Request>();

template<>
const rosidl_message_type_support_t * 
get_message_type_support_handle<moveit_msgs::srv::ServoCommandType_Response>();

} // namespace rosidl_typesupport_cpp
