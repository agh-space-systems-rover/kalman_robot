// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from unity_rs_publisher_msgs:msg/CameraMetadata.idl
// generated code does not contain a copyright notice

#ifndef UNITY_RS_PUBLISHER_MSGS__MSG__DETAIL__CAMERA_METADATA__TRAITS_HPP_
#define UNITY_RS_PUBLISHER_MSGS__MSG__DETAIL__CAMERA_METADATA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "unity_rs_publisher_msgs/msg/detail/camera_metadata__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace unity_rs_publisher_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const CameraMetadata & msg,
  std::ostream & out)
{
  out << "{";
  // member: width
  {
    out << "width: ";
    rosidl_generator_traits::value_to_yaml(msg.width, out);
    out << ", ";
  }

  // member: height
  {
    out << "height: ";
    rosidl_generator_traits::value_to_yaml(msg.height, out);
    out << ", ";
  }

  // member: depth_min
  {
    out << "depth_min: ";
    rosidl_generator_traits::value_to_yaml(msg.depth_min, out);
    out << ", ";
  }

  // member: depth_max
  {
    out << "depth_max: ";
    rosidl_generator_traits::value_to_yaml(msg.depth_max, out);
    out << ", ";
  }

  // member: fx
  {
    out << "fx: ";
    rosidl_generator_traits::value_to_yaml(msg.fx, out);
    out << ", ";
  }

  // member: fy
  {
    out << "fy: ";
    rosidl_generator_traits::value_to_yaml(msg.fy, out);
    out << ", ";
  }

  // member: cx
  {
    out << "cx: ";
    rosidl_generator_traits::value_to_yaml(msg.cx, out);
    out << ", ";
  }

  // member: cy
  {
    out << "cy: ";
    rosidl_generator_traits::value_to_yaml(msg.cy, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CameraMetadata & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: width
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "width: ";
    rosidl_generator_traits::value_to_yaml(msg.width, out);
    out << "\n";
  }

  // member: height
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "height: ";
    rosidl_generator_traits::value_to_yaml(msg.height, out);
    out << "\n";
  }

  // member: depth_min
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "depth_min: ";
    rosidl_generator_traits::value_to_yaml(msg.depth_min, out);
    out << "\n";
  }

  // member: depth_max
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "depth_max: ";
    rosidl_generator_traits::value_to_yaml(msg.depth_max, out);
    out << "\n";
  }

  // member: fx
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fx: ";
    rosidl_generator_traits::value_to_yaml(msg.fx, out);
    out << "\n";
  }

  // member: fy
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fy: ";
    rosidl_generator_traits::value_to_yaml(msg.fy, out);
    out << "\n";
  }

  // member: cx
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cx: ";
    rosidl_generator_traits::value_to_yaml(msg.cx, out);
    out << "\n";
  }

  // member: cy
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cy: ";
    rosidl_generator_traits::value_to_yaml(msg.cy, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CameraMetadata & msg, bool use_flow_style = false)
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

}  // namespace unity_rs_publisher_msgs

namespace rosidl_generator_traits
{

[[deprecated("use unity_rs_publisher_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const unity_rs_publisher_msgs::msg::CameraMetadata & msg,
  std::ostream & out, size_t indentation = 0)
{
  unity_rs_publisher_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use unity_rs_publisher_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const unity_rs_publisher_msgs::msg::CameraMetadata & msg)
{
  return unity_rs_publisher_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<unity_rs_publisher_msgs::msg::CameraMetadata>()
{
  return "unity_rs_publisher_msgs::msg::CameraMetadata";
}

template<>
inline const char * name<unity_rs_publisher_msgs::msg::CameraMetadata>()
{
  return "unity_rs_publisher_msgs/msg/CameraMetadata";
}

template<>
struct has_fixed_size<unity_rs_publisher_msgs::msg::CameraMetadata>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<unity_rs_publisher_msgs::msg::CameraMetadata>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<unity_rs_publisher_msgs::msg::CameraMetadata>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // UNITY_RS_PUBLISHER_MSGS__MSG__DETAIL__CAMERA_METADATA__TRAITS_HPP_
