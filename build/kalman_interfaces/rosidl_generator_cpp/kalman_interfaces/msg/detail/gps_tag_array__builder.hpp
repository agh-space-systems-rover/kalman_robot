// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from kalman_interfaces:msg/GpsTagArray.idl
// generated code does not contain a copyright notice

#ifndef KALMAN_INTERFACES__MSG__DETAIL__GPS_TAG_ARRAY__BUILDER_HPP_
#define KALMAN_INTERFACES__MSG__DETAIL__GPS_TAG_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "kalman_interfaces/msg/detail/gps_tag_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace kalman_interfaces
{

namespace msg
{

namespace builder
{

class Init_GpsTagArray_tags
{
public:
  Init_GpsTagArray_tags()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::kalman_interfaces::msg::GpsTagArray tags(::kalman_interfaces::msg::GpsTagArray::_tags_type arg)
  {
    msg_.tags = std::move(arg);
    return std::move(msg_);
  }

private:
  ::kalman_interfaces::msg::GpsTagArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::kalman_interfaces::msg::GpsTagArray>()
{
  return kalman_interfaces::msg::builder::Init_GpsTagArray_tags();
}

}  // namespace kalman_interfaces

#endif  // KALMAN_INTERFACES__MSG__DETAIL__GPS_TAG_ARRAY__BUILDER_HPP_
