// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from kalman_interfaces:msg/WheelStates.idl
// generated code does not contain a copyright notice

#ifndef KALMAN_INTERFACES__MSG__DETAIL__WHEEL_STATES__TRAITS_HPP_
#define KALMAN_INTERFACES__MSG__DETAIL__WHEEL_STATES__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "kalman_interfaces/msg/detail/wheel_states__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'front_left'
// Member 'front_right'
// Member 'back_left'
// Member 'back_right'
#include "kalman_interfaces/msg/detail/wheel_state__traits.hpp"

namespace kalman_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const WheelStates & msg,
  std::ostream & out)
{
  out << "{";
  // member: front_left
  {
    out << "front_left: ";
    to_flow_style_yaml(msg.front_left, out);
    out << ", ";
  }

  // member: front_right
  {
    out << "front_right: ";
    to_flow_style_yaml(msg.front_right, out);
    out << ", ";
  }

  // member: back_left
  {
    out << "back_left: ";
    to_flow_style_yaml(msg.back_left, out);
    out << ", ";
  }

  // member: back_right
  {
    out << "back_right: ";
    to_flow_style_yaml(msg.back_right, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const WheelStates & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: front_left
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "front_left:\n";
    to_block_style_yaml(msg.front_left, out, indentation + 2);
  }

  // member: front_right
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "front_right:\n";
    to_block_style_yaml(msg.front_right, out, indentation + 2);
  }

  // member: back_left
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "back_left:\n";
    to_block_style_yaml(msg.back_left, out, indentation + 2);
  }

  // member: back_right
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "back_right:\n";
    to_block_style_yaml(msg.back_right, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const WheelStates & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace kalman_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use kalman_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const kalman_interfaces::msg::WheelStates & msg,
  std::ostream & out, size_t indentation = 0)
{
  kalman_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use kalman_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const kalman_interfaces::msg::WheelStates & msg)
{
  return kalman_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<kalman_interfaces::msg::WheelStates>()
{
  return "kalman_interfaces::msg::WheelStates";
}

template<>
inline const char * name<kalman_interfaces::msg::WheelStates>()
{
  return "kalman_interfaces/msg/WheelStates";
}

template<>
struct has_fixed_size<kalman_interfaces::msg::WheelStates>
  : std::integral_constant<bool, has_fixed_size<kalman_interfaces::msg::WheelState>::value> {};

template<>
struct has_bounded_size<kalman_interfaces::msg::WheelStates>
  : std::integral_constant<bool, has_bounded_size<kalman_interfaces::msg::WheelState>::value> {};

template<>
struct is_message<kalman_interfaces::msg::WheelStates>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // KALMAN_INTERFACES__MSG__DETAIL__WHEEL_STATES__TRAITS_HPP_
