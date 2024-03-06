// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from unity_rs_publisher_msgs:msg/CameraMetadata.idl
// generated code does not contain a copyright notice

#ifndef UNITY_RS_PUBLISHER_MSGS__MSG__DETAIL__CAMERA_METADATA__STRUCT_HPP_
#define UNITY_RS_PUBLISHER_MSGS__MSG__DETAIL__CAMERA_METADATA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__unity_rs_publisher_msgs__msg__CameraMetadata __attribute__((deprecated))
#else
# define DEPRECATED__unity_rs_publisher_msgs__msg__CameraMetadata __declspec(deprecated)
#endif

namespace unity_rs_publisher_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct CameraMetadata_
{
  using Type = CameraMetadata_<ContainerAllocator>;

  explicit CameraMetadata_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->width = 0ul;
      this->height = 0ul;
      this->depth_min = 0.0f;
      this->depth_max = 0.0f;
      this->fx = 0.0f;
      this->fy = 0.0f;
      this->cx = 0.0f;
      this->cy = 0.0f;
    }
  }

  explicit CameraMetadata_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->width = 0ul;
      this->height = 0ul;
      this->depth_min = 0.0f;
      this->depth_max = 0.0f;
      this->fx = 0.0f;
      this->fy = 0.0f;
      this->cx = 0.0f;
      this->cy = 0.0f;
    }
  }

  // field types and members
  using _width_type =
    uint32_t;
  _width_type width;
  using _height_type =
    uint32_t;
  _height_type height;
  using _depth_min_type =
    float;
  _depth_min_type depth_min;
  using _depth_max_type =
    float;
  _depth_max_type depth_max;
  using _fx_type =
    float;
  _fx_type fx;
  using _fy_type =
    float;
  _fy_type fy;
  using _cx_type =
    float;
  _cx_type cx;
  using _cy_type =
    float;
  _cy_type cy;

  // setters for named parameter idiom
  Type & set__width(
    const uint32_t & _arg)
  {
    this->width = _arg;
    return *this;
  }
  Type & set__height(
    const uint32_t & _arg)
  {
    this->height = _arg;
    return *this;
  }
  Type & set__depth_min(
    const float & _arg)
  {
    this->depth_min = _arg;
    return *this;
  }
  Type & set__depth_max(
    const float & _arg)
  {
    this->depth_max = _arg;
    return *this;
  }
  Type & set__fx(
    const float & _arg)
  {
    this->fx = _arg;
    return *this;
  }
  Type & set__fy(
    const float & _arg)
  {
    this->fy = _arg;
    return *this;
  }
  Type & set__cx(
    const float & _arg)
  {
    this->cx = _arg;
    return *this;
  }
  Type & set__cy(
    const float & _arg)
  {
    this->cy = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    unity_rs_publisher_msgs::msg::CameraMetadata_<ContainerAllocator> *;
  using ConstRawPtr =
    const unity_rs_publisher_msgs::msg::CameraMetadata_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<unity_rs_publisher_msgs::msg::CameraMetadata_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<unity_rs_publisher_msgs::msg::CameraMetadata_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      unity_rs_publisher_msgs::msg::CameraMetadata_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<unity_rs_publisher_msgs::msg::CameraMetadata_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      unity_rs_publisher_msgs::msg::CameraMetadata_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<unity_rs_publisher_msgs::msg::CameraMetadata_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<unity_rs_publisher_msgs::msg::CameraMetadata_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<unity_rs_publisher_msgs::msg::CameraMetadata_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__unity_rs_publisher_msgs__msg__CameraMetadata
    std::shared_ptr<unity_rs_publisher_msgs::msg::CameraMetadata_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__unity_rs_publisher_msgs__msg__CameraMetadata
    std::shared_ptr<unity_rs_publisher_msgs::msg::CameraMetadata_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CameraMetadata_ & other) const
  {
    if (this->width != other.width) {
      return false;
    }
    if (this->height != other.height) {
      return false;
    }
    if (this->depth_min != other.depth_min) {
      return false;
    }
    if (this->depth_max != other.depth_max) {
      return false;
    }
    if (this->fx != other.fx) {
      return false;
    }
    if (this->fy != other.fy) {
      return false;
    }
    if (this->cx != other.cx) {
      return false;
    }
    if (this->cy != other.cy) {
      return false;
    }
    return true;
  }
  bool operator!=(const CameraMetadata_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CameraMetadata_

// alias to use template instance with default allocator
using CameraMetadata =
  unity_rs_publisher_msgs::msg::CameraMetadata_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace unity_rs_publisher_msgs

#endif  // UNITY_RS_PUBLISHER_MSGS__MSG__DETAIL__CAMERA_METADATA__STRUCT_HPP_
