// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from unity_rs_publisher_msgs:msg/CameraMetadata.idl
// generated code does not contain a copyright notice

#ifndef UNITY_RS_PUBLISHER_MSGS__MSG__DETAIL__CAMERA_METADATA__BUILDER_HPP_
#define UNITY_RS_PUBLISHER_MSGS__MSG__DETAIL__CAMERA_METADATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "unity_rs_publisher_msgs/msg/detail/camera_metadata__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace unity_rs_publisher_msgs
{

namespace msg
{

namespace builder
{

class Init_CameraMetadata_cy
{
public:
  explicit Init_CameraMetadata_cy(::unity_rs_publisher_msgs::msg::CameraMetadata & msg)
  : msg_(msg)
  {}
  ::unity_rs_publisher_msgs::msg::CameraMetadata cy(::unity_rs_publisher_msgs::msg::CameraMetadata::_cy_type arg)
  {
    msg_.cy = std::move(arg);
    return std::move(msg_);
  }

private:
  ::unity_rs_publisher_msgs::msg::CameraMetadata msg_;
};

class Init_CameraMetadata_cx
{
public:
  explicit Init_CameraMetadata_cx(::unity_rs_publisher_msgs::msg::CameraMetadata & msg)
  : msg_(msg)
  {}
  Init_CameraMetadata_cy cx(::unity_rs_publisher_msgs::msg::CameraMetadata::_cx_type arg)
  {
    msg_.cx = std::move(arg);
    return Init_CameraMetadata_cy(msg_);
  }

private:
  ::unity_rs_publisher_msgs::msg::CameraMetadata msg_;
};

class Init_CameraMetadata_fy
{
public:
  explicit Init_CameraMetadata_fy(::unity_rs_publisher_msgs::msg::CameraMetadata & msg)
  : msg_(msg)
  {}
  Init_CameraMetadata_cx fy(::unity_rs_publisher_msgs::msg::CameraMetadata::_fy_type arg)
  {
    msg_.fy = std::move(arg);
    return Init_CameraMetadata_cx(msg_);
  }

private:
  ::unity_rs_publisher_msgs::msg::CameraMetadata msg_;
};

class Init_CameraMetadata_fx
{
public:
  explicit Init_CameraMetadata_fx(::unity_rs_publisher_msgs::msg::CameraMetadata & msg)
  : msg_(msg)
  {}
  Init_CameraMetadata_fy fx(::unity_rs_publisher_msgs::msg::CameraMetadata::_fx_type arg)
  {
    msg_.fx = std::move(arg);
    return Init_CameraMetadata_fy(msg_);
  }

private:
  ::unity_rs_publisher_msgs::msg::CameraMetadata msg_;
};

class Init_CameraMetadata_depth_max
{
public:
  explicit Init_CameraMetadata_depth_max(::unity_rs_publisher_msgs::msg::CameraMetadata & msg)
  : msg_(msg)
  {}
  Init_CameraMetadata_fx depth_max(::unity_rs_publisher_msgs::msg::CameraMetadata::_depth_max_type arg)
  {
    msg_.depth_max = std::move(arg);
    return Init_CameraMetadata_fx(msg_);
  }

private:
  ::unity_rs_publisher_msgs::msg::CameraMetadata msg_;
};

class Init_CameraMetadata_depth_min
{
public:
  explicit Init_CameraMetadata_depth_min(::unity_rs_publisher_msgs::msg::CameraMetadata & msg)
  : msg_(msg)
  {}
  Init_CameraMetadata_depth_max depth_min(::unity_rs_publisher_msgs::msg::CameraMetadata::_depth_min_type arg)
  {
    msg_.depth_min = std::move(arg);
    return Init_CameraMetadata_depth_max(msg_);
  }

private:
  ::unity_rs_publisher_msgs::msg::CameraMetadata msg_;
};

class Init_CameraMetadata_height
{
public:
  explicit Init_CameraMetadata_height(::unity_rs_publisher_msgs::msg::CameraMetadata & msg)
  : msg_(msg)
  {}
  Init_CameraMetadata_depth_min height(::unity_rs_publisher_msgs::msg::CameraMetadata::_height_type arg)
  {
    msg_.height = std::move(arg);
    return Init_CameraMetadata_depth_min(msg_);
  }

private:
  ::unity_rs_publisher_msgs::msg::CameraMetadata msg_;
};

class Init_CameraMetadata_width
{
public:
  Init_CameraMetadata_width()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CameraMetadata_height width(::unity_rs_publisher_msgs::msg::CameraMetadata::_width_type arg)
  {
    msg_.width = std::move(arg);
    return Init_CameraMetadata_height(msg_);
  }

private:
  ::unity_rs_publisher_msgs::msg::CameraMetadata msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::unity_rs_publisher_msgs::msg::CameraMetadata>()
{
  return unity_rs_publisher_msgs::msg::builder::Init_CameraMetadata_width();
}

}  // namespace unity_rs_publisher_msgs

#endif  // UNITY_RS_PUBLISHER_MSGS__MSG__DETAIL__CAMERA_METADATA__BUILDER_HPP_
