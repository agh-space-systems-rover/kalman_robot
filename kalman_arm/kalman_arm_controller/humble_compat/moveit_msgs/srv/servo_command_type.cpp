// Temporary stub implementation for compatibility with Humble builds

#include "../../moveit_msgs/srv/servo_command_type.hpp"

// Basic type support structure - just enough to satisfy the linker
struct rosidl_service_type_support_t {
  const char * typesupport_identifier;
  const void * data;
};

struct rosidl_message_type_support_t {
  const char * typesupport_identifier;
  const void * data;
};

// Type support structure instances - minimal placeholders
static rosidl_service_type_support_t dummy_service_type_support = { 
  "rosidl_typesupport_cpp", 
  nullptr 
};

static rosidl_message_type_support_t dummy_message_type_support = { 
  "rosidl_typesupport_cpp", 
  nullptr 
};

// Type support function implementations
namespace rosidl_typesupport_cpp {

template<>
const rosidl_service_type_support_t * 
get_service_type_support_handle<moveit_msgs::srv::ServoCommandType>()
{
  return &dummy_service_type_support;
}

template<>
const rosidl_message_type_support_t * 
get_message_type_support_handle<moveit_msgs::srv::ServoCommandType_Request>()
{
  return &dummy_message_type_support;
}

template<>
const rosidl_message_type_support_t * 
get_message_type_support_handle<moveit_msgs::srv::ServoCommandType_Response>()
{
  return &dummy_message_type_support;
}

} // namespace rosidl_typesupport_cpp
