// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from kalman_interfaces:msg/WheelState.idl
// generated code does not contain a copyright notice

#ifndef KALMAN_INTERFACES__MSG__DETAIL__WHEEL_STATE__TRAITS_HPP_
#define KALMAN_INTERFACES__MSG__DETAIL__WHEEL_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "kalman_interfaces/msg/detail/wheel_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace kalman_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const WheelState & msg,
  std::ostream & out)
{
  out << "{";
  // member: velocity
  {
    out << "velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity, out);
    out << ", ";
  }

  // member: angle
  {
    out << "angle: ";
    rosidl_generator_traits::value_to_yaml(msg.angle, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const WheelState & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity, out);
    out << "\n";
  }

  // member: angle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "angle: ";
    rosidl_generator_traits::value_to_yaml(msg.angle, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const WheelState & msg, bool use_flow_style = false)
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
  const kalman_interfaces::msg::WheelState & msg,
  std::ostream & out, size_t indentation = 0)
{
  kalman_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use kalman_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const kalman_interfaces::msg::WheelState & msg)
{
  return kalman_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<kalman_interfaces::msg::WheelState>()
{
  return "kalman_interfaces::msg::WheelState";
}

template<>
inline const char * name<kalman_interfaces::msg::WheelState>()
{
  return "kalman_interfaces/msg/WheelState";
}

template<>
struct has_fixed_size<kalman_interfaces::msg::WheelState>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<kalman_interfaces::msg::WheelState>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<kalman_interfaces::msg::WheelState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // KALMAN_INTERFACES__MSG__DETAIL__WHEEL_STATE__TRAITS_HPP_
