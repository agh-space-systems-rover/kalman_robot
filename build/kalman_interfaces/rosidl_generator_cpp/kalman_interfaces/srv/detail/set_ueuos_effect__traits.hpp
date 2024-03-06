// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from kalman_interfaces:srv/SetUeuosEffect.idl
// generated code does not contain a copyright notice

#ifndef KALMAN_INTERFACES__SRV__DETAIL__SET_UEUOS_EFFECT__TRAITS_HPP_
#define KALMAN_INTERFACES__SRV__DETAIL__SET_UEUOS_EFFECT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "kalman_interfaces/srv/detail/set_ueuos_effect__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace kalman_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetUeuosEffect_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: effect
  {
    out << "effect: ";
    rosidl_generator_traits::value_to_yaml(msg.effect, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetUeuosEffect_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: effect
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "effect: ";
    rosidl_generator_traits::value_to_yaml(msg.effect, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetUeuosEffect_Request & msg, bool use_flow_style = false)
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
  const kalman_interfaces::srv::SetUeuosEffect_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  kalman_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use kalman_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const kalman_interfaces::srv::SetUeuosEffect_Request & msg)
{
  return kalman_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<kalman_interfaces::srv::SetUeuosEffect_Request>()
{
  return "kalman_interfaces::srv::SetUeuosEffect_Request";
}

template<>
inline const char * name<kalman_interfaces::srv::SetUeuosEffect_Request>()
{
  return "kalman_interfaces/srv/SetUeuosEffect_Request";
}

template<>
struct has_fixed_size<kalman_interfaces::srv::SetUeuosEffect_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<kalman_interfaces::srv::SetUeuosEffect_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<kalman_interfaces::srv::SetUeuosEffect_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace kalman_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetUeuosEffect_Response & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetUeuosEffect_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetUeuosEffect_Response & msg, bool use_flow_style = false)
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
  const kalman_interfaces::srv::SetUeuosEffect_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  kalman_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use kalman_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const kalman_interfaces::srv::SetUeuosEffect_Response & msg)
{
  return kalman_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<kalman_interfaces::srv::SetUeuosEffect_Response>()
{
  return "kalman_interfaces::srv::SetUeuosEffect_Response";
}

template<>
inline const char * name<kalman_interfaces::srv::SetUeuosEffect_Response>()
{
  return "kalman_interfaces/srv/SetUeuosEffect_Response";
}

template<>
struct has_fixed_size<kalman_interfaces::srv::SetUeuosEffect_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<kalman_interfaces::srv::SetUeuosEffect_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<kalman_interfaces::srv::SetUeuosEffect_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<kalman_interfaces::srv::SetUeuosEffect>()
{
  return "kalman_interfaces::srv::SetUeuosEffect";
}

template<>
inline const char * name<kalman_interfaces::srv::SetUeuosEffect>()
{
  return "kalman_interfaces/srv/SetUeuosEffect";
}

template<>
struct has_fixed_size<kalman_interfaces::srv::SetUeuosEffect>
  : std::integral_constant<
    bool,
    has_fixed_size<kalman_interfaces::srv::SetUeuosEffect_Request>::value &&
    has_fixed_size<kalman_interfaces::srv::SetUeuosEffect_Response>::value
  >
{
};

template<>
struct has_bounded_size<kalman_interfaces::srv::SetUeuosEffect>
  : std::integral_constant<
    bool,
    has_bounded_size<kalman_interfaces::srv::SetUeuosEffect_Request>::value &&
    has_bounded_size<kalman_interfaces::srv::SetUeuosEffect_Response>::value
  >
{
};

template<>
struct is_service<kalman_interfaces::srv::SetUeuosEffect>
  : std::true_type
{
};

template<>
struct is_service_request<kalman_interfaces::srv::SetUeuosEffect_Request>
  : std::true_type
{
};

template<>
struct is_service_response<kalman_interfaces::srv::SetUeuosEffect_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // KALMAN_INTERFACES__SRV__DETAIL__SET_UEUOS_EFFECT__TRAITS_HPP_
