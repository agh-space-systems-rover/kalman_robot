// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from kalman_interfaces:msg/MasterMessage.idl
// generated code does not contain a copyright notice

#ifndef KALMAN_INTERFACES__MSG__DETAIL__MASTER_MESSAGE__TRAITS_HPP_
#define KALMAN_INTERFACES__MSG__DETAIL__MASTER_MESSAGE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "kalman_interfaces/msg/detail/master_message__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace kalman_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const MasterMessage & msg,
  std::ostream & out)
{
  out << "{";
  // member: cmd
  {
    out << "cmd: ";
    rosidl_generator_traits::value_to_yaml(msg.cmd, out);
    out << ", ";
  }

  // member: data
  {
    if (msg.data.size() == 0) {
      out << "data: []";
    } else {
      out << "data: [";
      size_t pending_items = msg.data.size();
      for (auto item : msg.data) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MasterMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: cmd
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cmd: ";
    rosidl_generator_traits::value_to_yaml(msg.cmd, out);
    out << "\n";
  }

  // member: data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.data.size() == 0) {
      out << "data: []\n";
    } else {
      out << "data:\n";
      for (auto item : msg.data) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MasterMessage & msg, bool use_flow_style = false)
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
  const kalman_interfaces::msg::MasterMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  kalman_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use kalman_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const kalman_interfaces::msg::MasterMessage & msg)
{
  return kalman_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<kalman_interfaces::msg::MasterMessage>()
{
  return "kalman_interfaces::msg::MasterMessage";
}

template<>
inline const char * name<kalman_interfaces::msg::MasterMessage>()
{
  return "kalman_interfaces/msg/MasterMessage";
}

template<>
struct has_fixed_size<kalman_interfaces::msg::MasterMessage>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<kalman_interfaces::msg::MasterMessage>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<kalman_interfaces::msg::MasterMessage>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // KALMAN_INTERFACES__MSG__DETAIL__MASTER_MESSAGE__TRAITS_HPP_
