// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from service_based_nav2_controller_srvs:srv/ComputeVelocityCommands.idl
// generated code does not contain a copyright notice

#ifndef SERVICE_BASED_NAV2_CONTROLLER_SRVS__SRV__DETAIL__COMPUTE_VELOCITY_COMMANDS__STRUCT_HPP_
#define SERVICE_BASED_NAV2_CONTROLLER_SRVS__SRV__DETAIL__COMPUTE_VELOCITY_COMMANDS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"
// Member 'velocity'
#include "geometry_msgs/msg/detail/twist__struct.hpp"
// Member 'path'
#include "nav_msgs/msg/detail/path__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request __attribute__((deprecated))
#else
# define DEPRECATED__service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request __declspec(deprecated)
#endif

namespace service_based_nav2_controller_srvs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ComputeVelocityCommands_Request_
{
  using Type = ComputeVelocityCommands_Request_<ContainerAllocator>;

  explicit ComputeVelocityCommands_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pose(_init),
    velocity(_init),
    path(_init)
  {
    (void)_init;
  }

  explicit ComputeVelocityCommands_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pose(_alloc, _init),
    velocity(_alloc, _init),
    path(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _pose_type =
    geometry_msgs::msg::PoseStamped_<ContainerAllocator>;
  _pose_type pose;
  using _velocity_type =
    geometry_msgs::msg::Twist_<ContainerAllocator>;
  _velocity_type velocity;
  using _path_type =
    nav_msgs::msg::Path_<ContainerAllocator>;
  _path_type path;

  // setters for named parameter idiom
  Type & set__pose(
    const geometry_msgs::msg::PoseStamped_<ContainerAllocator> & _arg)
  {
    this->pose = _arg;
    return *this;
  }
  Type & set__velocity(
    const geometry_msgs::msg::Twist_<ContainerAllocator> & _arg)
  {
    this->velocity = _arg;
    return *this;
  }
  Type & set__path(
    const nav_msgs::msg::Path_<ContainerAllocator> & _arg)
  {
    this->path = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    service_based_nav2_controller_srvs::srv::ComputeVelocityCommands_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const service_based_nav2_controller_srvs::srv::ComputeVelocityCommands_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<service_based_nav2_controller_srvs::srv::ComputeVelocityCommands_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<service_based_nav2_controller_srvs::srv::ComputeVelocityCommands_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      service_based_nav2_controller_srvs::srv::ComputeVelocityCommands_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<service_based_nav2_controller_srvs::srv::ComputeVelocityCommands_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      service_based_nav2_controller_srvs::srv::ComputeVelocityCommands_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<service_based_nav2_controller_srvs::srv::ComputeVelocityCommands_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<service_based_nav2_controller_srvs::srv::ComputeVelocityCommands_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<service_based_nav2_controller_srvs::srv::ComputeVelocityCommands_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request
    std::shared_ptr<service_based_nav2_controller_srvs::srv::ComputeVelocityCommands_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Request
    std::shared_ptr<service_based_nav2_controller_srvs::srv::ComputeVelocityCommands_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ComputeVelocityCommands_Request_ & other) const
  {
    if (this->pose != other.pose) {
      return false;
    }
    if (this->velocity != other.velocity) {
      return false;
    }
    if (this->path != other.path) {
      return false;
    }
    return true;
  }
  bool operator!=(const ComputeVelocityCommands_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ComputeVelocityCommands_Request_

// alias to use template instance with default allocator
using ComputeVelocityCommands_Request =
  service_based_nav2_controller_srvs::srv::ComputeVelocityCommands_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace service_based_nav2_controller_srvs


// Include directives for member types
// Member 'cmd_vel'
#include "geometry_msgs/msg/detail/twist_stamped__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response __attribute__((deprecated))
#else
# define DEPRECATED__service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response __declspec(deprecated)
#endif

namespace service_based_nav2_controller_srvs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ComputeVelocityCommands_Response_
{
  using Type = ComputeVelocityCommands_Response_<ContainerAllocator>;

  explicit ComputeVelocityCommands_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : cmd_vel(_init)
  {
    (void)_init;
  }

  explicit ComputeVelocityCommands_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : cmd_vel(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _cmd_vel_type =
    geometry_msgs::msg::TwistStamped_<ContainerAllocator>;
  _cmd_vel_type cmd_vel;

  // setters for named parameter idiom
  Type & set__cmd_vel(
    const geometry_msgs::msg::TwistStamped_<ContainerAllocator> & _arg)
  {
    this->cmd_vel = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    service_based_nav2_controller_srvs::srv::ComputeVelocityCommands_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const service_based_nav2_controller_srvs::srv::ComputeVelocityCommands_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<service_based_nav2_controller_srvs::srv::ComputeVelocityCommands_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<service_based_nav2_controller_srvs::srv::ComputeVelocityCommands_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      service_based_nav2_controller_srvs::srv::ComputeVelocityCommands_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<service_based_nav2_controller_srvs::srv::ComputeVelocityCommands_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      service_based_nav2_controller_srvs::srv::ComputeVelocityCommands_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<service_based_nav2_controller_srvs::srv::ComputeVelocityCommands_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<service_based_nav2_controller_srvs::srv::ComputeVelocityCommands_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<service_based_nav2_controller_srvs::srv::ComputeVelocityCommands_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response
    std::shared_ptr<service_based_nav2_controller_srvs::srv::ComputeVelocityCommands_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__service_based_nav2_controller_srvs__srv__ComputeVelocityCommands_Response
    std::shared_ptr<service_based_nav2_controller_srvs::srv::ComputeVelocityCommands_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ComputeVelocityCommands_Response_ & other) const
  {
    if (this->cmd_vel != other.cmd_vel) {
      return false;
    }
    return true;
  }
  bool operator!=(const ComputeVelocityCommands_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ComputeVelocityCommands_Response_

// alias to use template instance with default allocator
using ComputeVelocityCommands_Response =
  service_based_nav2_controller_srvs::srv::ComputeVelocityCommands_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace service_based_nav2_controller_srvs

namespace service_based_nav2_controller_srvs
{

namespace srv
{

struct ComputeVelocityCommands
{
  using Request = service_based_nav2_controller_srvs::srv::ComputeVelocityCommands_Request;
  using Response = service_based_nav2_controller_srvs::srv::ComputeVelocityCommands_Response;
};

}  // namespace srv

}  // namespace service_based_nav2_controller_srvs

#endif  // SERVICE_BASED_NAV2_CONTROLLER_SRVS__SRV__DETAIL__COMPUTE_VELOCITY_COMMANDS__STRUCT_HPP_
