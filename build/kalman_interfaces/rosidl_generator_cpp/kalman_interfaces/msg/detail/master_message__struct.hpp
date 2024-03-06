// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from kalman_interfaces:msg/MasterMessage.idl
// generated code does not contain a copyright notice

#ifndef KALMAN_INTERFACES__MSG__DETAIL__MASTER_MESSAGE__STRUCT_HPP_
#define KALMAN_INTERFACES__MSG__DETAIL__MASTER_MESSAGE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__kalman_interfaces__msg__MasterMessage __attribute__((deprecated))
#else
# define DEPRECATED__kalman_interfaces__msg__MasterMessage __declspec(deprecated)
#endif

namespace kalman_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MasterMessage_
{
  using Type = MasterMessage_<ContainerAllocator>;

  explicit MasterMessage_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->cmd = 0;
    }
  }

  explicit MasterMessage_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->cmd = 0;
    }
  }

  // field types and members
  using _cmd_type =
    uint8_t;
  _cmd_type cmd;
  using _data_type =
    std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>>;
  _data_type data;

  // setters for named parameter idiom
  Type & set__cmd(
    const uint8_t & _arg)
  {
    this->cmd = _arg;
    return *this;
  }
  Type & set__data(
    const std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>> & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t UEUOS_SET_STATE =
    96u;
  static constexpr uint8_t UEUOS_SET_COLOR =
    97u;
  static constexpr uint8_t UEUOS_SET_EFFECT =
    98u;
  static constexpr uint8_t MOTOR_SET_WHEELS =
    64u;
  static constexpr uint8_t AUTONOMY_SWITCH =
    32u;

  // pointer types
  using RawPtr =
    kalman_interfaces::msg::MasterMessage_<ContainerAllocator> *;
  using ConstRawPtr =
    const kalman_interfaces::msg::MasterMessage_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<kalman_interfaces::msg::MasterMessage_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<kalman_interfaces::msg::MasterMessage_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      kalman_interfaces::msg::MasterMessage_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<kalman_interfaces::msg::MasterMessage_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      kalman_interfaces::msg::MasterMessage_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<kalman_interfaces::msg::MasterMessage_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<kalman_interfaces::msg::MasterMessage_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<kalman_interfaces::msg::MasterMessage_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__kalman_interfaces__msg__MasterMessage
    std::shared_ptr<kalman_interfaces::msg::MasterMessage_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__kalman_interfaces__msg__MasterMessage
    std::shared_ptr<kalman_interfaces::msg::MasterMessage_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MasterMessage_ & other) const
  {
    if (this->cmd != other.cmd) {
      return false;
    }
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const MasterMessage_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MasterMessage_

// alias to use template instance with default allocator
using MasterMessage =
  kalman_interfaces::msg::MasterMessage_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t MasterMessage_<ContainerAllocator>::UEUOS_SET_STATE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t MasterMessage_<ContainerAllocator>::UEUOS_SET_COLOR;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t MasterMessage_<ContainerAllocator>::UEUOS_SET_EFFECT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t MasterMessage_<ContainerAllocator>::MOTOR_SET_WHEELS;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t MasterMessage_<ContainerAllocator>::AUTONOMY_SWITCH;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace kalman_interfaces

#endif  // KALMAN_INTERFACES__MSG__DETAIL__MASTER_MESSAGE__STRUCT_HPP_
