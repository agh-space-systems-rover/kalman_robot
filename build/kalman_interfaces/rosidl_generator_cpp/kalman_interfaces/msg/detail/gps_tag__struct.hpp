// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from kalman_interfaces:msg/GpsTag.idl
// generated code does not contain a copyright notice

#ifndef KALMAN_INTERFACES__MSG__DETAIL__GPS_TAG__STRUCT_HPP_
#define KALMAN_INTERFACES__MSG__DETAIL__GPS_TAG__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__kalman_interfaces__msg__GpsTag __attribute__((deprecated))
#else
# define DEPRECATED__kalman_interfaces__msg__GpsTag __declspec(deprecated)
#endif

namespace kalman_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GpsTag_
{
  using Type = GpsTag_<ContainerAllocator>;

  explicit GpsTag_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->tag_id = 0;
      this->tag_lat = 0.0f;
      this->tag_lon = 0.0f;
    }
  }

  explicit GpsTag_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->tag_id = 0;
      this->tag_lat = 0.0f;
      this->tag_lon = 0.0f;
    }
  }

  // field types and members
  using _tag_id_type =
    int8_t;
  _tag_id_type tag_id;
  using _tag_lat_type =
    float;
  _tag_lat_type tag_lat;
  using _tag_lon_type =
    float;
  _tag_lon_type tag_lon;

  // setters for named parameter idiom
  Type & set__tag_id(
    const int8_t & _arg)
  {
    this->tag_id = _arg;
    return *this;
  }
  Type & set__tag_lat(
    const float & _arg)
  {
    this->tag_lat = _arg;
    return *this;
  }
  Type & set__tag_lon(
    const float & _arg)
  {
    this->tag_lon = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    kalman_interfaces::msg::GpsTag_<ContainerAllocator> *;
  using ConstRawPtr =
    const kalman_interfaces::msg::GpsTag_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<kalman_interfaces::msg::GpsTag_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<kalman_interfaces::msg::GpsTag_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      kalman_interfaces::msg::GpsTag_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<kalman_interfaces::msg::GpsTag_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      kalman_interfaces::msg::GpsTag_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<kalman_interfaces::msg::GpsTag_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<kalman_interfaces::msg::GpsTag_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<kalman_interfaces::msg::GpsTag_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__kalman_interfaces__msg__GpsTag
    std::shared_ptr<kalman_interfaces::msg::GpsTag_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__kalman_interfaces__msg__GpsTag
    std::shared_ptr<kalman_interfaces::msg::GpsTag_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GpsTag_ & other) const
  {
    if (this->tag_id != other.tag_id) {
      return false;
    }
    if (this->tag_lat != other.tag_lat) {
      return false;
    }
    if (this->tag_lon != other.tag_lon) {
      return false;
    }
    return true;
  }
  bool operator!=(const GpsTag_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GpsTag_

// alias to use template instance with default allocator
using GpsTag =
  kalman_interfaces::msg::GpsTag_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace kalman_interfaces

#endif  // KALMAN_INTERFACES__MSG__DETAIL__GPS_TAG__STRUCT_HPP_
