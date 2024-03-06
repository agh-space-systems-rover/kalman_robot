// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from kalman_interfaces:srv/SetUeuosMode.idl
// generated code does not contain a copyright notice

#ifndef KALMAN_INTERFACES__SRV__DETAIL__SET_UEUOS_MODE__STRUCT_HPP_
#define KALMAN_INTERFACES__SRV__DETAIL__SET_UEUOS_MODE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__kalman_interfaces__srv__SetUeuosMode_Request __attribute__((deprecated))
#else
# define DEPRECATED__kalman_interfaces__srv__SetUeuosMode_Request __declspec(deprecated)
#endif

namespace kalman_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetUeuosMode_Request_
{
  using Type = SetUeuosMode_Request_<ContainerAllocator>;

  explicit SetUeuosMode_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mode = 0;
    }
  }

  explicit SetUeuosMode_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mode = 0;
    }
  }

  // field types and members
  using _mode_type =
    uint8_t;
  _mode_type mode;

  // setters for named parameter idiom
  Type & set__mode(
    const uint8_t & _arg)
  {
    this->mode = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t OFF =
    0u;
  static constexpr uint8_t AUTONOMY =
    1u;
  static constexpr uint8_t TELEOP =
    2u;
  static constexpr uint8_t FINISHED =
    3u;

  // pointer types
  using RawPtr =
    kalman_interfaces::srv::SetUeuosMode_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const kalman_interfaces::srv::SetUeuosMode_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<kalman_interfaces::srv::SetUeuosMode_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<kalman_interfaces::srv::SetUeuosMode_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      kalman_interfaces::srv::SetUeuosMode_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<kalman_interfaces::srv::SetUeuosMode_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      kalman_interfaces::srv::SetUeuosMode_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<kalman_interfaces::srv::SetUeuosMode_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<kalman_interfaces::srv::SetUeuosMode_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<kalman_interfaces::srv::SetUeuosMode_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__kalman_interfaces__srv__SetUeuosMode_Request
    std::shared_ptr<kalman_interfaces::srv::SetUeuosMode_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__kalman_interfaces__srv__SetUeuosMode_Request
    std::shared_ptr<kalman_interfaces::srv::SetUeuosMode_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetUeuosMode_Request_ & other) const
  {
    if (this->mode != other.mode) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetUeuosMode_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetUeuosMode_Request_

// alias to use template instance with default allocator
using SetUeuosMode_Request =
  kalman_interfaces::srv::SetUeuosMode_Request_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SetUeuosMode_Request_<ContainerAllocator>::OFF;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SetUeuosMode_Request_<ContainerAllocator>::AUTONOMY;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SetUeuosMode_Request_<ContainerAllocator>::TELEOP;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SetUeuosMode_Request_<ContainerAllocator>::FINISHED;
#endif  // __cplusplus < 201703L

}  // namespace srv

}  // namespace kalman_interfaces


#ifndef _WIN32
# define DEPRECATED__kalman_interfaces__srv__SetUeuosMode_Response __attribute__((deprecated))
#else
# define DEPRECATED__kalman_interfaces__srv__SetUeuosMode_Response __declspec(deprecated)
#endif

namespace kalman_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetUeuosMode_Response_
{
  using Type = SetUeuosMode_Response_<ContainerAllocator>;

  explicit SetUeuosMode_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit SetUeuosMode_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    kalman_interfaces::srv::SetUeuosMode_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const kalman_interfaces::srv::SetUeuosMode_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<kalman_interfaces::srv::SetUeuosMode_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<kalman_interfaces::srv::SetUeuosMode_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      kalman_interfaces::srv::SetUeuosMode_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<kalman_interfaces::srv::SetUeuosMode_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      kalman_interfaces::srv::SetUeuosMode_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<kalman_interfaces::srv::SetUeuosMode_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<kalman_interfaces::srv::SetUeuosMode_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<kalman_interfaces::srv::SetUeuosMode_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__kalman_interfaces__srv__SetUeuosMode_Response
    std::shared_ptr<kalman_interfaces::srv::SetUeuosMode_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__kalman_interfaces__srv__SetUeuosMode_Response
    std::shared_ptr<kalman_interfaces::srv::SetUeuosMode_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetUeuosMode_Response_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetUeuosMode_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetUeuosMode_Response_

// alias to use template instance with default allocator
using SetUeuosMode_Response =
  kalman_interfaces::srv::SetUeuosMode_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace kalman_interfaces

namespace kalman_interfaces
{

namespace srv
{

struct SetUeuosMode
{
  using Request = kalman_interfaces::srv::SetUeuosMode_Request;
  using Response = kalman_interfaces::srv::SetUeuosMode_Response;
};

}  // namespace srv

}  // namespace kalman_interfaces

#endif  // KALMAN_INTERFACES__SRV__DETAIL__SET_UEUOS_MODE__STRUCT_HPP_
