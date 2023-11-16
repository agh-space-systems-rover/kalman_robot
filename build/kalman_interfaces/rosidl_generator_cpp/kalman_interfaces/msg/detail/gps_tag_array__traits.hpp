// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from kalman_interfaces:msg/GpsTagArray.idl
// generated code does not contain a copyright notice

#ifndef KALMAN_INTERFACES__MSG__DETAIL__GPS_TAG_ARRAY__TRAITS_HPP_
#define KALMAN_INTERFACES__MSG__DETAIL__GPS_TAG_ARRAY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "kalman_interfaces/msg/detail/gps_tag_array__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'tags'
#include "kalman_interfaces/msg/detail/gps_tag__traits.hpp"

namespace kalman_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const GpsTagArray & msg,
  std::ostream & out)
{
  out << "{";
  // member: tags
  {
    if (msg.tags.size() == 0) {
      out << "tags: []";
    } else {
      out << "tags: [";
      size_t pending_items = msg.tags.size();
      for (auto item : msg.tags) {
        to_flow_style_yaml(item, out);
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
  const GpsTagArray & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: tags
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.tags.size() == 0) {
      out << "tags: []\n";
    } else {
      out << "tags:\n";
      for (auto item : msg.tags) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GpsTagArray & msg, bool use_flow_style = false)
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
  const kalman_interfaces::msg::GpsTagArray & msg,
  std::ostream & out, size_t indentation = 0)
{
  kalman_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use kalman_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const kalman_interfaces::msg::GpsTagArray & msg)
{
  return kalman_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<kalman_interfaces::msg::GpsTagArray>()
{
  return "kalman_interfaces::msg::GpsTagArray";
}

template<>
inline const char * name<kalman_interfaces::msg::GpsTagArray>()
{
  return "kalman_interfaces/msg/GpsTagArray";
}

template<>
struct has_fixed_size<kalman_interfaces::msg::GpsTagArray>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<kalman_interfaces::msg::GpsTagArray>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<kalman_interfaces::msg::GpsTagArray>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // KALMAN_INTERFACES__MSG__DETAIL__GPS_TAG_ARRAY__TRAITS_HPP_
