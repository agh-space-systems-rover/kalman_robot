// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from kalman_interfaces:srv/SetUeuosColor.idl
// generated code does not contain a copyright notice

#ifndef KALMAN_INTERFACES__SRV__DETAIL__SET_UEUOS_COLOR__STRUCT_HPP_
#define KALMAN_INTERFACES__SRV__DETAIL__SET_UEUOS_COLOR__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'color'
#include "std_msgs/msg/detail/color_rgba__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__kalman_interfaces__srv__SetUeuosColor_Request __attribute__((deprecated))
#else
# define DEPRECATED__kalman_interfaces__srv__SetUeuosColor_Request __declspec(deprecated)
#endif

namespace kalman_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetUeuosColor_Request_
{
  using Type = SetUeuosColor_Request_<ContainerAllocator>;

  explicit SetUeuosColor_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : color(_init)
  {
    (void)_init;
  }

  explicit SetUeuosColor_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : color(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _color_type =
    std_msgs::msg::ColorRGBA_<ContainerAllocator>;
  _color_type color;

  // setters for named parameter idiom
  Type & set__color(
    const std_msgs::msg::ColorRGBA_<ContainerAllocator> & _arg)
  {
    this->color = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    kalman_interfaces::srv::SetUeuosColor_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const kalman_interfaces::srv::SetUeuosColor_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<kalman_interfaces::srv::SetUeuosColor_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<kalman_interfaces::srv::SetUeuosColor_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      kalman_interfaces::srv::SetUeuosColor_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<kalman_interfaces::srv::SetUeuosColor_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      kalman_interfaces::srv::SetUeuosColor_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<kalman_interfaces::srv::SetUeuosColor_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<kalman_interfaces::srv::SetUeuosColor_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<kalman_interfaces::srv::SetUeuosColor_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__kalman_interfaces__srv__SetUeuosColor_Request
    std::shared_ptr<kalman_interfaces::srv::SetUeuosColor_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__kalman_interfaces__srv__SetUeuosColor_Request
    std::shared_ptr<kalman_interfaces::srv::SetUeuosColor_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetUeuosColor_Request_ & other) const
  {
    if (this->color != other.color) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetUeuosColor_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetUeuosColor_Request_

// alias to use template instance with default allocator
using SetUeuosColor_Request =
  kalman_interfaces::srv::SetUeuosColor_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace kalman_interfaces


#ifndef _WIN32
# define DEPRECATED__kalman_interfaces__srv__SetUeuosColor_Response __attribute__((deprecated))
#else
# define DEPRECATED__kalman_interfaces__srv__SetUeuosColor_Response __declspec(deprecated)
#endif

namespace kalman_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetUeuosColor_Response_
{
  using Type = SetUeuosColor_Response_<ContainerAllocator>;

  explicit SetUeuosColor_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit SetUeuosColor_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    kalman_interfaces::srv::SetUeuosColor_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const kalman_interfaces::srv::SetUeuosColor_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<kalman_interfaces::srv::SetUeuosColor_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<kalman_interfaces::srv::SetUeuosColor_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      kalman_interfaces::srv::SetUeuosColor_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<kalman_interfaces::srv::SetUeuosColor_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      kalman_interfaces::srv::SetUeuosColor_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<kalman_interfaces::srv::SetUeuosColor_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<kalman_interfaces::srv::SetUeuosColor_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<kalman_interfaces::srv::SetUeuosColor_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__kalman_interfaces__srv__SetUeuosColor_Response
    std::shared_ptr<kalman_interfaces::srv::SetUeuosColor_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__kalman_interfaces__srv__SetUeuosColor_Response
    std::shared_ptr<kalman_interfaces::srv::SetUeuosColor_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetUeuosColor_Response_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetUeuosColor_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetUeuosColor_Response_

// alias to use template instance with default allocator
using SetUeuosColor_Response =
  kalman_interfaces::srv::SetUeuosColor_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace kalman_interfaces

namespace kalman_interfaces
{

namespace srv
{

struct SetUeuosColor
{
  using Request = kalman_interfaces::srv::SetUeuosColor_Request;
  using Response = kalman_interfaces::srv::SetUeuosColor_Response;
};

}  // namespace srv

}  // namespace kalman_interfaces

#endif  // KALMAN_INTERFACES__SRV__DETAIL__SET_UEUOS_COLOR__STRUCT_HPP_
