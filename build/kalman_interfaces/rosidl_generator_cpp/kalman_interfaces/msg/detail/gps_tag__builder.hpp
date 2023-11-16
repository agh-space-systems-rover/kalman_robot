// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from kalman_interfaces:msg/GpsTag.idl
// generated code does not contain a copyright notice

#ifndef KALMAN_INTERFACES__MSG__DETAIL__GPS_TAG__BUILDER_HPP_
#define KALMAN_INTERFACES__MSG__DETAIL__GPS_TAG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "kalman_interfaces/msg/detail/gps_tag__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace kalman_interfaces
{

namespace msg
{

namespace builder
{

class Init_GpsTag_tag_lon
{
public:
  explicit Init_GpsTag_tag_lon(::kalman_interfaces::msg::GpsTag & msg)
  : msg_(msg)
  {}
  ::kalman_interfaces::msg::GpsTag tag_lon(::kalman_interfaces::msg::GpsTag::_tag_lon_type arg)
  {
    msg_.tag_lon = std::move(arg);
    return std::move(msg_);
  }

private:
  ::kalman_interfaces::msg::GpsTag msg_;
};

class Init_GpsTag_tag_lat
{
public:
  explicit Init_GpsTag_tag_lat(::kalman_interfaces::msg::GpsTag & msg)
  : msg_(msg)
  {}
  Init_GpsTag_tag_lon tag_lat(::kalman_interfaces::msg::GpsTag::_tag_lat_type arg)
  {
    msg_.tag_lat = std::move(arg);
    return Init_GpsTag_tag_lon(msg_);
  }

private:
  ::kalman_interfaces::msg::GpsTag msg_;
};

class Init_GpsTag_tag_id
{
public:
  Init_GpsTag_tag_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GpsTag_tag_lat tag_id(::kalman_interfaces::msg::GpsTag::_tag_id_type arg)
  {
    msg_.tag_id = std::move(arg);
    return Init_GpsTag_tag_lat(msg_);
  }

private:
  ::kalman_interfaces::msg::GpsTag msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::kalman_interfaces::msg::GpsTag>()
{
  return kalman_interfaces::msg::builder::Init_GpsTag_tag_id();
}

}  // namespace kalman_interfaces

#endif  // KALMAN_INTERFACES__MSG__DETAIL__GPS_TAG__BUILDER_HPP_
