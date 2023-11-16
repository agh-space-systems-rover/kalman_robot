// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from kalman_interfaces:msg/GpsTagArray.idl
// generated code does not contain a copyright notice

#ifndef KALMAN_INTERFACES__MSG__DETAIL__GPS_TAG_ARRAY__STRUCT_HPP_
#define KALMAN_INTERFACES__MSG__DETAIL__GPS_TAG_ARRAY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'tags'
#include "kalman_interfaces/msg/detail/gps_tag__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__kalman_interfaces__msg__GpsTagArray __attribute__((deprecated))
#else
# define DEPRECATED__kalman_interfaces__msg__GpsTagArray __declspec(deprecated)
#endif

namespace kalman_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GpsTagArray_
{
  using Type = GpsTagArray_<ContainerAllocator>;

  explicit GpsTagArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit GpsTagArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _tags_type =
    std::vector<kalman_interfaces::msg::GpsTag_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<kalman_interfaces::msg::GpsTag_<ContainerAllocator>>>;
  _tags_type tags;

  // setters for named parameter idiom
  Type & set__tags(
    const std::vector<kalman_interfaces::msg::GpsTag_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<kalman_interfaces::msg::GpsTag_<ContainerAllocator>>> & _arg)
  {
    this->tags = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    kalman_interfaces::msg::GpsTagArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const kalman_interfaces::msg::GpsTagArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<kalman_interfaces::msg::GpsTagArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<kalman_interfaces::msg::GpsTagArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      kalman_interfaces::msg::GpsTagArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<kalman_interfaces::msg::GpsTagArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      kalman_interfaces::msg::GpsTagArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<kalman_interfaces::msg::GpsTagArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<kalman_interfaces::msg::GpsTagArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<kalman_interfaces::msg::GpsTagArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__kalman_interfaces__msg__GpsTagArray
    std::shared_ptr<kalman_interfaces::msg::GpsTagArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__kalman_interfaces__msg__GpsTagArray
    std::shared_ptr<kalman_interfaces::msg::GpsTagArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GpsTagArray_ & other) const
  {
    if (this->tags != other.tags) {
      return false;
    }
    return true;
  }
  bool operator!=(const GpsTagArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GpsTagArray_

// alias to use template instance with default allocator
using GpsTagArray =
  kalman_interfaces::msg::GpsTagArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace kalman_interfaces

#endif  // KALMAN_INTERFACES__MSG__DETAIL__GPS_TAG_ARRAY__STRUCT_HPP_
