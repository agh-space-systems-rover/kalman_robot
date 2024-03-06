// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from kalman_interfaces:srv/CalibrateCompass.idl
// generated code does not contain a copyright notice

#ifndef KALMAN_INTERFACES__SRV__DETAIL__CALIBRATE_COMPASS__STRUCT_HPP_
#define KALMAN_INTERFACES__SRV__DETAIL__CALIBRATE_COMPASS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__kalman_interfaces__srv__CalibrateCompass_Request __attribute__((deprecated))
#else
# define DEPRECATED__kalman_interfaces__srv__CalibrateCompass_Request __declspec(deprecated)
#endif

namespace kalman_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct CalibrateCompass_Request_
{
  using Type = CalibrateCompass_Request_<ContainerAllocator>;

  explicit CalibrateCompass_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->duration = 30.0f;
      this->angular_velocity = 0.5f;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->duration = 0.0f;
      this->angular_velocity = 0.0f;
    }
  }

  explicit CalibrateCompass_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->duration = 30.0f;
      this->angular_velocity = 0.5f;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->duration = 0.0f;
      this->angular_velocity = 0.0f;
    }
  }

  // field types and members
  using _duration_type =
    float;
  _duration_type duration;
  using _angular_velocity_type =
    float;
  _angular_velocity_type angular_velocity;

  // setters for named parameter idiom
  Type & set__duration(
    const float & _arg)
  {
    this->duration = _arg;
    return *this;
  }
  Type & set__angular_velocity(
    const float & _arg)
  {
    this->angular_velocity = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    kalman_interfaces::srv::CalibrateCompass_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const kalman_interfaces::srv::CalibrateCompass_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<kalman_interfaces::srv::CalibrateCompass_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<kalman_interfaces::srv::CalibrateCompass_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      kalman_interfaces::srv::CalibrateCompass_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<kalman_interfaces::srv::CalibrateCompass_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      kalman_interfaces::srv::CalibrateCompass_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<kalman_interfaces::srv::CalibrateCompass_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<kalman_interfaces::srv::CalibrateCompass_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<kalman_interfaces::srv::CalibrateCompass_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__kalman_interfaces__srv__CalibrateCompass_Request
    std::shared_ptr<kalman_interfaces::srv::CalibrateCompass_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__kalman_interfaces__srv__CalibrateCompass_Request
    std::shared_ptr<kalman_interfaces::srv::CalibrateCompass_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CalibrateCompass_Request_ & other) const
  {
    if (this->duration != other.duration) {
      return false;
    }
    if (this->angular_velocity != other.angular_velocity) {
      return false;
    }
    return true;
  }
  bool operator!=(const CalibrateCompass_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CalibrateCompass_Request_

// alias to use template instance with default allocator
using CalibrateCompass_Request =
  kalman_interfaces::srv::CalibrateCompass_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace kalman_interfaces


#ifndef _WIN32
# define DEPRECATED__kalman_interfaces__srv__CalibrateCompass_Response __attribute__((deprecated))
#else
# define DEPRECATED__kalman_interfaces__srv__CalibrateCompass_Response __declspec(deprecated)
#endif

namespace kalman_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct CalibrateCompass_Response_
{
  using Type = CalibrateCompass_Response_<ContainerAllocator>;

  explicit CalibrateCompass_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->cc_mag_field = 0.0f;
      this->cc_offset0 = 0.0f;
      this->cc_offset1 = 0.0f;
      this->cc_offset2 = 0.0f;
      this->cc_gain0 = 0.0f;
      this->cc_gain1 = 0.0f;
      this->cc_gain2 = 0.0f;
      this->cc_t0 = 0.0f;
      this->cc_t1 = 0.0f;
      this->cc_t2 = 0.0f;
      this->cc_t3 = 0.0f;
      this->cc_t4 = 0.0f;
      this->cc_t5 = 0.0f;
    }
  }

  explicit CalibrateCompass_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->cc_mag_field = 0.0f;
      this->cc_offset0 = 0.0f;
      this->cc_offset1 = 0.0f;
      this->cc_offset2 = 0.0f;
      this->cc_gain0 = 0.0f;
      this->cc_gain1 = 0.0f;
      this->cc_gain2 = 0.0f;
      this->cc_t0 = 0.0f;
      this->cc_t1 = 0.0f;
      this->cc_t2 = 0.0f;
      this->cc_t3 = 0.0f;
      this->cc_t4 = 0.0f;
      this->cc_t5 = 0.0f;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _cc_mag_field_type =
    float;
  _cc_mag_field_type cc_mag_field;
  using _cc_offset0_type =
    float;
  _cc_offset0_type cc_offset0;
  using _cc_offset1_type =
    float;
  _cc_offset1_type cc_offset1;
  using _cc_offset2_type =
    float;
  _cc_offset2_type cc_offset2;
  using _cc_gain0_type =
    float;
  _cc_gain0_type cc_gain0;
  using _cc_gain1_type =
    float;
  _cc_gain1_type cc_gain1;
  using _cc_gain2_type =
    float;
  _cc_gain2_type cc_gain2;
  using _cc_t0_type =
    float;
  _cc_t0_type cc_t0;
  using _cc_t1_type =
    float;
  _cc_t1_type cc_t1;
  using _cc_t2_type =
    float;
  _cc_t2_type cc_t2;
  using _cc_t3_type =
    float;
  _cc_t3_type cc_t3;
  using _cc_t4_type =
    float;
  _cc_t4_type cc_t4;
  using _cc_t5_type =
    float;
  _cc_t5_type cc_t5;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__cc_mag_field(
    const float & _arg)
  {
    this->cc_mag_field = _arg;
    return *this;
  }
  Type & set__cc_offset0(
    const float & _arg)
  {
    this->cc_offset0 = _arg;
    return *this;
  }
  Type & set__cc_offset1(
    const float & _arg)
  {
    this->cc_offset1 = _arg;
    return *this;
  }
  Type & set__cc_offset2(
    const float & _arg)
  {
    this->cc_offset2 = _arg;
    return *this;
  }
  Type & set__cc_gain0(
    const float & _arg)
  {
    this->cc_gain0 = _arg;
    return *this;
  }
  Type & set__cc_gain1(
    const float & _arg)
  {
    this->cc_gain1 = _arg;
    return *this;
  }
  Type & set__cc_gain2(
    const float & _arg)
  {
    this->cc_gain2 = _arg;
    return *this;
  }
  Type & set__cc_t0(
    const float & _arg)
  {
    this->cc_t0 = _arg;
    return *this;
  }
  Type & set__cc_t1(
    const float & _arg)
  {
    this->cc_t1 = _arg;
    return *this;
  }
  Type & set__cc_t2(
    const float & _arg)
  {
    this->cc_t2 = _arg;
    return *this;
  }
  Type & set__cc_t3(
    const float & _arg)
  {
    this->cc_t3 = _arg;
    return *this;
  }
  Type & set__cc_t4(
    const float & _arg)
  {
    this->cc_t4 = _arg;
    return *this;
  }
  Type & set__cc_t5(
    const float & _arg)
  {
    this->cc_t5 = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    kalman_interfaces::srv::CalibrateCompass_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const kalman_interfaces::srv::CalibrateCompass_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<kalman_interfaces::srv::CalibrateCompass_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<kalman_interfaces::srv::CalibrateCompass_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      kalman_interfaces::srv::CalibrateCompass_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<kalman_interfaces::srv::CalibrateCompass_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      kalman_interfaces::srv::CalibrateCompass_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<kalman_interfaces::srv::CalibrateCompass_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<kalman_interfaces::srv::CalibrateCompass_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<kalman_interfaces::srv::CalibrateCompass_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__kalman_interfaces__srv__CalibrateCompass_Response
    std::shared_ptr<kalman_interfaces::srv::CalibrateCompass_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__kalman_interfaces__srv__CalibrateCompass_Response
    std::shared_ptr<kalman_interfaces::srv::CalibrateCompass_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CalibrateCompass_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->cc_mag_field != other.cc_mag_field) {
      return false;
    }
    if (this->cc_offset0 != other.cc_offset0) {
      return false;
    }
    if (this->cc_offset1 != other.cc_offset1) {
      return false;
    }
    if (this->cc_offset2 != other.cc_offset2) {
      return false;
    }
    if (this->cc_gain0 != other.cc_gain0) {
      return false;
    }
    if (this->cc_gain1 != other.cc_gain1) {
      return false;
    }
    if (this->cc_gain2 != other.cc_gain2) {
      return false;
    }
    if (this->cc_t0 != other.cc_t0) {
      return false;
    }
    if (this->cc_t1 != other.cc_t1) {
      return false;
    }
    if (this->cc_t2 != other.cc_t2) {
      return false;
    }
    if (this->cc_t3 != other.cc_t3) {
      return false;
    }
    if (this->cc_t4 != other.cc_t4) {
      return false;
    }
    if (this->cc_t5 != other.cc_t5) {
      return false;
    }
    return true;
  }
  bool operator!=(const CalibrateCompass_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CalibrateCompass_Response_

// alias to use template instance with default allocator
using CalibrateCompass_Response =
  kalman_interfaces::srv::CalibrateCompass_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace kalman_interfaces

namespace kalman_interfaces
{

namespace srv
{

struct CalibrateCompass
{
  using Request = kalman_interfaces::srv::CalibrateCompass_Request;
  using Response = kalman_interfaces::srv::CalibrateCompass_Response;
};

}  // namespace srv

}  // namespace kalman_interfaces

#endif  // KALMAN_INTERFACES__SRV__DETAIL__CALIBRATE_COMPASS__STRUCT_HPP_
