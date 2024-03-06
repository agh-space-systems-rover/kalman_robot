// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from kalman_interfaces:srv/SetUeuosColor.idl
// generated code does not contain a copyright notice

#ifndef KALMAN_INTERFACES__SRV__DETAIL__SET_UEUOS_COLOR__TRAITS_HPP_
#define KALMAN_INTERFACES__SRV__DETAIL__SET_UEUOS_COLOR__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "kalman_interfaces/srv/detail/set_ueuos_color__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'color'
#include "std_msgs/msg/detail/color_rgba__traits.hpp"

namespace kalman_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetUeuosColor_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: color
  {
    out << "color: ";
    to_flow_style_yaml(msg.color, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetUeuosColor_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: color
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "color:\n";
    to_block_style_yaml(msg.color, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetUeuosColor_Request & msg, bool use_flow_style = false)
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

}  // namespace kalman_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use kalman_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const kalman_interfaces::srv::SetUeuosColor_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  kalman_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use kalman_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const kalman_interfaces::srv::SetUeuosColor_Request & msg)
{
  return kalman_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<kalman_interfaces::srv::SetUeuosColor_Request>()
{
  return "kalman_interfaces::srv::SetUeuosColor_Request";
}

template<>
inline const char * name<kalman_interfaces::srv::SetUeuosColor_Request>()
{
  return "kalman_interfaces/srv/SetUeuosColor_Request";
}

template<>
struct has_fixed_size<kalman_interfaces::srv::SetUeuosColor_Request>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::ColorRGBA>::value> {};

template<>
struct has_bounded_size<kalman_interfaces::srv::SetUeuosColor_Request>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::ColorRGBA>::value> {};

template<>
struct is_message<kalman_interfaces::srv::SetUeuosColor_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace kalman_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetUeuosColor_Response & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetUeuosColor_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetUeuosColor_Response & msg, bool use_flow_style = false)
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

}  // namespace kalman_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use kalman_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const kalman_interfaces::srv::SetUeuosColor_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  kalman_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use kalman_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const kalman_interfaces::srv::SetUeuosColor_Response & msg)
{
  return kalman_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<kalman_interfaces::srv::SetUeuosColor_Response>()
{
  return "kalman_interfaces::srv::SetUeuosColor_Response";
}

template<>
inline const char * name<kalman_interfaces::srv::SetUeuosColor_Response>()
{
  return "kalman_interfaces/srv/SetUeuosColor_Response";
}

template<>
struct has_fixed_size<kalman_interfaces::srv::SetUeuosColor_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<kalman_interfaces::srv::SetUeuosColor_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<kalman_interfaces::srv::SetUeuosColor_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<kalman_interfaces::srv::SetUeuosColor>()
{
  return "kalman_interfaces::srv::SetUeuosColor";
}

template<>
inline const char * name<kalman_interfaces::srv::SetUeuosColor>()
{
  return "kalman_interfaces/srv/SetUeuosColor";
}

template<>
struct has_fixed_size<kalman_interfaces::srv::SetUeuosColor>
  : std::integral_constant<
    bool,
    has_fixed_size<kalman_interfaces::srv::SetUeuosColor_Request>::value &&
    has_fixed_size<kalman_interfaces::srv::SetUeuosColor_Response>::value
  >
{
};

template<>
struct has_bounded_size<kalman_interfaces::srv::SetUeuosColor>
  : std::integral_constant<
    bool,
    has_bounded_size<kalman_interfaces::srv::SetUeuosColor_Request>::value &&
    has_bounded_size<kalman_interfaces::srv::SetUeuosColor_Response>::value
  >
{
};

template<>
struct is_service<kalman_interfaces::srv::SetUeuosColor>
  : std::true_type
{
};

template<>
struct is_service_request<kalman_interfaces::srv::SetUeuosColor_Request>
  : std::true_type
{
};

template<>
struct is_service_response<kalman_interfaces::srv::SetUeuosColor_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // KALMAN_INTERFACES__SRV__DETAIL__SET_UEUOS_COLOR__TRAITS_HPP_
