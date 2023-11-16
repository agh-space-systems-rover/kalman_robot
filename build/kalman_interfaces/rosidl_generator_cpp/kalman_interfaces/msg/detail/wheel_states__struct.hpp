// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from kalman_interfaces:msg/WheelStates.idl
// generated code does not contain a copyright notice

#ifndef KALMAN_INTERFACES__MSG__DETAIL__WHEEL_STATES__STRUCT_HPP_
#define KALMAN_INTERFACES__MSG__DETAIL__WHEEL_STATES__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'front_left'
// Member 'front_right'
// Member 'back_left'
// Member 'back_right'
#include "kalman_interfaces/msg/detail/wheel_state__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__kalman_interfaces__msg__WheelStates __attribute__((deprecated))
#else
# define DEPRECATED__kalman_interfaces__msg__WheelStates __declspec(deprecated)
#endif

namespace kalman_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct WheelStates_
{
  using Type = WheelStates_<ContainerAllocator>;

  explicit WheelStates_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : front_left(_init),
    front_right(_init),
    back_left(_init),
    back_right(_init)
  {
    (void)_init;
  }

  explicit WheelStates_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : front_left(_alloc, _init),
    front_right(_alloc, _init),
    back_left(_alloc, _init),
    back_right(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _front_left_type =
    kalman_interfaces::msg::WheelState_<ContainerAllocator>;
  _front_left_type front_left;
  using _front_right_type =
    kalman_interfaces::msg::WheelState_<ContainerAllocator>;
  _front_right_type front_right;
  using _back_left_type =
    kalman_interfaces::msg::WheelState_<ContainerAllocator>;
  _back_left_type back_left;
  using _back_right_type =
    kalman_interfaces::msg::WheelState_<ContainerAllocator>;
  _back_right_type back_right;

  // setters for named parameter idiom
  Type & set__front_left(
    const kalman_interfaces::msg::WheelState_<ContainerAllocator> & _arg)
  {
    this->front_left = _arg;
    return *this;
  }
  Type & set__front_right(
    const kalman_interfaces::msg::WheelState_<ContainerAllocator> & _arg)
  {
    this->front_right = _arg;
    return *this;
  }
  Type & set__back_left(
    const kalman_interfaces::msg::WheelState_<ContainerAllocator> & _arg)
  {
    this->back_left = _arg;
    return *this;
  }
  Type & set__back_right(
    const kalman_interfaces::msg::WheelState_<ContainerAllocator> & _arg)
  {
    this->back_right = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    kalman_interfaces::msg::WheelStates_<ContainerAllocator> *;
  using ConstRawPtr =
    const kalman_interfaces::msg::WheelStates_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<kalman_interfaces::msg::WheelStates_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<kalman_interfaces::msg::WheelStates_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      kalman_interfaces::msg::WheelStates_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<kalman_interfaces::msg::WheelStates_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      kalman_interfaces::msg::WheelStates_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<kalman_interfaces::msg::WheelStates_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<kalman_interfaces::msg::WheelStates_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<kalman_interfaces::msg::WheelStates_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__kalman_interfaces__msg__WheelStates
    std::shared_ptr<kalman_interfaces::msg::WheelStates_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__kalman_interfaces__msg__WheelStates
    std::shared_ptr<kalman_interfaces::msg::WheelStates_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const WheelStates_ & other) const
  {
    if (this->front_left != other.front_left) {
      return false;
    }
    if (this->front_right != other.front_right) {
      return false;
    }
    if (this->back_left != other.back_left) {
      return false;
    }
    if (this->back_right != other.back_right) {
      return false;
    }
    return true;
  }
  bool operator!=(const WheelStates_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct WheelStates_

// alias to use template instance with default allocator
using WheelStates =
  kalman_interfaces::msg::WheelStates_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace kalman_interfaces

#endif  // KALMAN_INTERFACES__MSG__DETAIL__WHEEL_STATES__STRUCT_HPP_
