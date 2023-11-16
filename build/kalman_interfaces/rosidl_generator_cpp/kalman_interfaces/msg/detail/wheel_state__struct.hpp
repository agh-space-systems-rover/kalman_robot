// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from kalman_interfaces:msg/WheelState.idl
// generated code does not contain a copyright notice

#ifndef KALMAN_INTERFACES__MSG__DETAIL__WHEEL_STATE__STRUCT_HPP_
#define KALMAN_INTERFACES__MSG__DETAIL__WHEEL_STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__kalman_interfaces__msg__WheelState __attribute__((deprecated))
#else
# define DEPRECATED__kalman_interfaces__msg__WheelState __declspec(deprecated)
#endif

namespace kalman_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct WheelState_
{
  using Type = WheelState_<ContainerAllocator>;

  explicit WheelState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->velocity = 0.0f;
      this->angle = 0.0f;
    }
  }

  explicit WheelState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->velocity = 0.0f;
      this->angle = 0.0f;
    }
  }

  // field types and members
  using _velocity_type =
    float;
  _velocity_type velocity;
  using _angle_type =
    float;
  _angle_type angle;

  // setters for named parameter idiom
  Type & set__velocity(
    const float & _arg)
  {
    this->velocity = _arg;
    return *this;
  }
  Type & set__angle(
    const float & _arg)
  {
    this->angle = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    kalman_interfaces::msg::WheelState_<ContainerAllocator> *;
  using ConstRawPtr =
    const kalman_interfaces::msg::WheelState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<kalman_interfaces::msg::WheelState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<kalman_interfaces::msg::WheelState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      kalman_interfaces::msg::WheelState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<kalman_interfaces::msg::WheelState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      kalman_interfaces::msg::WheelState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<kalman_interfaces::msg::WheelState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<kalman_interfaces::msg::WheelState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<kalman_interfaces::msg::WheelState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__kalman_interfaces__msg__WheelState
    std::shared_ptr<kalman_interfaces::msg::WheelState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__kalman_interfaces__msg__WheelState
    std::shared_ptr<kalman_interfaces::msg::WheelState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const WheelState_ & other) const
  {
    if (this->velocity != other.velocity) {
      return false;
    }
    if (this->angle != other.angle) {
      return false;
    }
    return true;
  }
  bool operator!=(const WheelState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct WheelState_

// alias to use template instance with default allocator
using WheelState =
  kalman_interfaces::msg::WheelState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace kalman_interfaces

#endif  // KALMAN_INTERFACES__MSG__DETAIL__WHEEL_STATE__STRUCT_HPP_
