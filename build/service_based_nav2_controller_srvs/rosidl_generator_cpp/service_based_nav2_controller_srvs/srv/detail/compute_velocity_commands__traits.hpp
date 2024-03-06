// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from service_based_nav2_controller_srvs:srv/ComputeVelocityCommands.idl
// generated code does not contain a copyright notice

#ifndef SERVICE_BASED_NAV2_CONTROLLER_SRVS__SRV__DETAIL__COMPUTE_VELOCITY_COMMANDS__TRAITS_HPP_
#define SERVICE_BASED_NAV2_CONTROLLER_SRVS__SRV__DETAIL__COMPUTE_VELOCITY_COMMANDS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "service_based_nav2_controller_srvs/srv/detail/compute_velocity_commands__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose_stamped__traits.hpp"
// Member 'velocity'
#include "geometry_msgs/msg/detail/twist__traits.hpp"
// Member 'path'
#include "nav_msgs/msg/detail/path__traits.hpp"

namespace service_based_nav2_controller_srvs
{

namespace srv
{

inline void to_flow_style_yaml(
  const ComputeVelocityCommands_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: pose
  {
    out << "pose: ";
    to_flow_style_yaml(msg.pose, out);
    out << ", ";
  }

  // member: velocity
  {
    out << "velocity: ";
    to_flow_style_yaml(msg.velocity, out);
    out << ", ";
  }

  // member: path
  {
    out << "path: ";
    to_flow_style_yaml(msg.path, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ComputeVelocityCommands_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pose:\n";
    to_block_style_yaml(msg.pose, out, indentation + 2);
  }

  // member: velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "velocity:\n";
    to_block_style_yaml(msg.velocity, out, indentation + 2);
  }

  // member: path
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "path:\n";
    to_block_style_yaml(msg.path, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ComputeVelocityCommands_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace service_based_nav2_controller_srvs

namespace rosidl_generator_traits
{

[[deprecated("use service_based_nav2_controller_srvs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const service_based_nav2_controller_srvs::srv::ComputeVelocityCommands_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  service_based_nav2_controller_srvs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use service_based_nav2_controller_srvs::srv::to_yaml() instead")]]
inline std::string to_yaml(const service_based_nav2_controller_srvs::srv::ComputeVelocityCommands_Request & msg)
{
  return service_based_nav2_controller_srvs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<service_based_nav2_controller_srvs::srv::ComputeVelocityCommands_Request>()
{
  return "service_based_nav2_controller_srvs::srv::ComputeVelocityCommands_Request";
}

template<>
inline const char * name<service_based_nav2_controller_srvs::srv::ComputeVelocityCommands_Request>()
{
  return "service_based_nav2_controller_srvs/srv/ComputeVelocityCommands_Request";
}

template<>
struct has_fixed_size<service_based_nav2_controller_srvs::srv::ComputeVelocityCommands_Request>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::PoseStamped>::value && has_fixed_size<geometry_msgs::msg::Twist>::value && has_fixed_size<nav_msgs::msg::Path>::value> {};

template<>
struct has_bounded_size<service_based_nav2_controller_srvs::srv::ComputeVelocityCommands_Request>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::PoseStamped>::value && has_bounded_size<geometry_msgs::msg::Twist>::value && has_bounded_size<nav_msgs::msg::Path>::value> {};

template<>
struct is_message<service_based_nav2_controller_srvs::srv::ComputeVelocityCommands_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'cmd_vel'
#include "geometry_msgs/msg/detail/twist_stamped__traits.hpp"

namespace service_based_nav2_controller_srvs
{

namespace srv
{

inline void to_flow_style_yaml(
  const ComputeVelocityCommands_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: cmd_vel
  {
    out << "cmd_vel: ";
    to_flow_style_yaml(msg.cmd_vel, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ComputeVelocityCommands_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: cmd_vel
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cmd_vel:\n";
    to_block_style_yaml(msg.cmd_vel, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ComputeVelocityCommands_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace service_based_nav2_controller_srvs

namespace rosidl_generator_traits
{

[[deprecated("use service_based_nav2_controller_srvs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const service_based_nav2_controller_srvs::srv::ComputeVelocityCommands_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  service_based_nav2_controller_srvs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use service_based_nav2_controller_srvs::srv::to_yaml() instead")]]
inline std::string to_yaml(const service_based_nav2_controller_srvs::srv::ComputeVelocityCommands_Response & msg)
{
  return service_based_nav2_controller_srvs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<service_based_nav2_controller_srvs::srv::ComputeVelocityCommands_Response>()
{
  return "service_based_nav2_controller_srvs::srv::ComputeVelocityCommands_Response";
}

template<>
inline const char * name<service_based_nav2_controller_srvs::srv::ComputeVelocityCommands_Response>()
{
  return "service_based_nav2_controller_srvs/srv/ComputeVelocityCommands_Response";
}

template<>
struct has_fixed_size<service_based_nav2_controller_srvs::srv::ComputeVelocityCommands_Response>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::TwistStamped>::value> {};

template<>
struct has_bounded_size<service_based_nav2_controller_srvs::srv::ComputeVelocityCommands_Response>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::TwistStamped>::value> {};

template<>
struct is_message<service_based_nav2_controller_srvs::srv::ComputeVelocityCommands_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<service_based_nav2_controller_srvs::srv::ComputeVelocityCommands>()
{
  return "service_based_nav2_controller_srvs::srv::ComputeVelocityCommands";
}

template<>
inline const char * name<service_based_nav2_controller_srvs::srv::ComputeVelocityCommands>()
{
  return "service_based_nav2_controller_srvs/srv/ComputeVelocityCommands";
}

template<>
struct has_fixed_size<service_based_nav2_controller_srvs::srv::ComputeVelocityCommands>
  : std::integral_constant<
    bool,
    has_fixed_size<service_based_nav2_controller_srvs::srv::ComputeVelocityCommands_Request>::value &&
    has_fixed_size<service_based_nav2_controller_srvs::srv::ComputeVelocityCommands_Response>::value
  >
{
};

template<>
struct has_bounded_size<service_based_nav2_controller_srvs::srv::ComputeVelocityCommands>
  : std::integral_constant<
    bool,
    has_bounded_size<service_based_nav2_controller_srvs::srv::ComputeVelocityCommands_Request>::value &&
    has_bounded_size<service_based_nav2_controller_srvs::srv::ComputeVelocityCommands_Response>::value
  >
{
};

template<>
struct is_service<service_based_nav2_controller_srvs::srv::ComputeVelocityCommands>
  : std::true_type
{
};

template<>
struct is_service_request<service_based_nav2_controller_srvs::srv::ComputeVelocityCommands_Request>
  : std::true_type
{
};

template<>
struct is_service_response<service_based_nav2_controller_srvs::srv::ComputeVelocityCommands_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // SERVICE_BASED_NAV2_CONTROLLER_SRVS__SRV__DETAIL__COMPUTE_VELOCITY_COMMANDS__TRAITS_HPP_
