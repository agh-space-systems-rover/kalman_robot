// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from kalman_interfaces:srv/CalibrateCompass.idl
// generated code does not contain a copyright notice

#ifndef KALMAN_INTERFACES__SRV__DETAIL__CALIBRATE_COMPASS__TRAITS_HPP_
#define KALMAN_INTERFACES__SRV__DETAIL__CALIBRATE_COMPASS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "kalman_interfaces/srv/detail/calibrate_compass__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace kalman_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const CalibrateCompass_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: duration
  {
    out << "duration: ";
    rosidl_generator_traits::value_to_yaml(msg.duration, out);
    out << ", ";
  }

  // member: angular_velocity
  {
    out << "angular_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.angular_velocity, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CalibrateCompass_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: duration
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "duration: ";
    rosidl_generator_traits::value_to_yaml(msg.duration, out);
    out << "\n";
  }

  // member: angular_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "angular_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.angular_velocity, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CalibrateCompass_Request & msg, bool use_flow_style = false)
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
  const kalman_interfaces::srv::CalibrateCompass_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  kalman_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use kalman_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const kalman_interfaces::srv::CalibrateCompass_Request & msg)
{
  return kalman_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<kalman_interfaces::srv::CalibrateCompass_Request>()
{
  return "kalman_interfaces::srv::CalibrateCompass_Request";
}

template<>
inline const char * name<kalman_interfaces::srv::CalibrateCompass_Request>()
{
  return "kalman_interfaces/srv/CalibrateCompass_Request";
}

template<>
struct has_fixed_size<kalman_interfaces::srv::CalibrateCompass_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<kalman_interfaces::srv::CalibrateCompass_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<kalman_interfaces::srv::CalibrateCompass_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace kalman_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const CalibrateCompass_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: cc_mag_field
  {
    out << "cc_mag_field: ";
    rosidl_generator_traits::value_to_yaml(msg.cc_mag_field, out);
    out << ", ";
  }

  // member: cc_offset0
  {
    out << "cc_offset0: ";
    rosidl_generator_traits::value_to_yaml(msg.cc_offset0, out);
    out << ", ";
  }

  // member: cc_offset1
  {
    out << "cc_offset1: ";
    rosidl_generator_traits::value_to_yaml(msg.cc_offset1, out);
    out << ", ";
  }

  // member: cc_offset2
  {
    out << "cc_offset2: ";
    rosidl_generator_traits::value_to_yaml(msg.cc_offset2, out);
    out << ", ";
  }

  // member: cc_gain0
  {
    out << "cc_gain0: ";
    rosidl_generator_traits::value_to_yaml(msg.cc_gain0, out);
    out << ", ";
  }

  // member: cc_gain1
  {
    out << "cc_gain1: ";
    rosidl_generator_traits::value_to_yaml(msg.cc_gain1, out);
    out << ", ";
  }

  // member: cc_gain2
  {
    out << "cc_gain2: ";
    rosidl_generator_traits::value_to_yaml(msg.cc_gain2, out);
    out << ", ";
  }

  // member: cc_t0
  {
    out << "cc_t0: ";
    rosidl_generator_traits::value_to_yaml(msg.cc_t0, out);
    out << ", ";
  }

  // member: cc_t1
  {
    out << "cc_t1: ";
    rosidl_generator_traits::value_to_yaml(msg.cc_t1, out);
    out << ", ";
  }

  // member: cc_t2
  {
    out << "cc_t2: ";
    rosidl_generator_traits::value_to_yaml(msg.cc_t2, out);
    out << ", ";
  }

  // member: cc_t3
  {
    out << "cc_t3: ";
    rosidl_generator_traits::value_to_yaml(msg.cc_t3, out);
    out << ", ";
  }

  // member: cc_t4
  {
    out << "cc_t4: ";
    rosidl_generator_traits::value_to_yaml(msg.cc_t4, out);
    out << ", ";
  }

  // member: cc_t5
  {
    out << "cc_t5: ";
    rosidl_generator_traits::value_to_yaml(msg.cc_t5, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CalibrateCompass_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: cc_mag_field
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cc_mag_field: ";
    rosidl_generator_traits::value_to_yaml(msg.cc_mag_field, out);
    out << "\n";
  }

  // member: cc_offset0
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cc_offset0: ";
    rosidl_generator_traits::value_to_yaml(msg.cc_offset0, out);
    out << "\n";
  }

  // member: cc_offset1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cc_offset1: ";
    rosidl_generator_traits::value_to_yaml(msg.cc_offset1, out);
    out << "\n";
  }

  // member: cc_offset2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cc_offset2: ";
    rosidl_generator_traits::value_to_yaml(msg.cc_offset2, out);
    out << "\n";
  }

  // member: cc_gain0
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cc_gain0: ";
    rosidl_generator_traits::value_to_yaml(msg.cc_gain0, out);
    out << "\n";
  }

  // member: cc_gain1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cc_gain1: ";
    rosidl_generator_traits::value_to_yaml(msg.cc_gain1, out);
    out << "\n";
  }

  // member: cc_gain2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cc_gain2: ";
    rosidl_generator_traits::value_to_yaml(msg.cc_gain2, out);
    out << "\n";
  }

  // member: cc_t0
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cc_t0: ";
    rosidl_generator_traits::value_to_yaml(msg.cc_t0, out);
    out << "\n";
  }

  // member: cc_t1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cc_t1: ";
    rosidl_generator_traits::value_to_yaml(msg.cc_t1, out);
    out << "\n";
  }

  // member: cc_t2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cc_t2: ";
    rosidl_generator_traits::value_to_yaml(msg.cc_t2, out);
    out << "\n";
  }

  // member: cc_t3
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cc_t3: ";
    rosidl_generator_traits::value_to_yaml(msg.cc_t3, out);
    out << "\n";
  }

  // member: cc_t4
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cc_t4: ";
    rosidl_generator_traits::value_to_yaml(msg.cc_t4, out);
    out << "\n";
  }

  // member: cc_t5
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cc_t5: ";
    rosidl_generator_traits::value_to_yaml(msg.cc_t5, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CalibrateCompass_Response & msg, bool use_flow_style = false)
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
  const kalman_interfaces::srv::CalibrateCompass_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  kalman_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use kalman_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const kalman_interfaces::srv::CalibrateCompass_Response & msg)
{
  return kalman_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<kalman_interfaces::srv::CalibrateCompass_Response>()
{
  return "kalman_interfaces::srv::CalibrateCompass_Response";
}

template<>
inline const char * name<kalman_interfaces::srv::CalibrateCompass_Response>()
{
  return "kalman_interfaces/srv/CalibrateCompass_Response";
}

template<>
struct has_fixed_size<kalman_interfaces::srv::CalibrateCompass_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<kalman_interfaces::srv::CalibrateCompass_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<kalman_interfaces::srv::CalibrateCompass_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<kalman_interfaces::srv::CalibrateCompass>()
{
  return "kalman_interfaces::srv::CalibrateCompass";
}

template<>
inline const char * name<kalman_interfaces::srv::CalibrateCompass>()
{
  return "kalman_interfaces/srv/CalibrateCompass";
}

template<>
struct has_fixed_size<kalman_interfaces::srv::CalibrateCompass>
  : std::integral_constant<
    bool,
    has_fixed_size<kalman_interfaces::srv::CalibrateCompass_Request>::value &&
    has_fixed_size<kalman_interfaces::srv::CalibrateCompass_Response>::value
  >
{
};

template<>
struct has_bounded_size<kalman_interfaces::srv::CalibrateCompass>
  : std::integral_constant<
    bool,
    has_bounded_size<kalman_interfaces::srv::CalibrateCompass_Request>::value &&
    has_bounded_size<kalman_interfaces::srv::CalibrateCompass_Response>::value
  >
{
};

template<>
struct is_service<kalman_interfaces::srv::CalibrateCompass>
  : std::true_type
{
};

template<>
struct is_service_request<kalman_interfaces::srv::CalibrateCompass_Request>
  : std::true_type
{
};

template<>
struct is_service_response<kalman_interfaces::srv::CalibrateCompass_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // KALMAN_INTERFACES__SRV__DETAIL__CALIBRATE_COMPASS__TRAITS_HPP_
