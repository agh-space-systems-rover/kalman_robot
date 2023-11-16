// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from kalman_interfaces:msg/WheelState.idl
// generated code does not contain a copyright notice

#ifndef KALMAN_INTERFACES__MSG__DETAIL__WHEEL_STATE__BUILDER_HPP_
#define KALMAN_INTERFACES__MSG__DETAIL__WHEEL_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "kalman_interfaces/msg/detail/wheel_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace kalman_interfaces
{

namespace msg
{

namespace builder
{

class Init_WheelState_angle
{
public:
  explicit Init_WheelState_angle(::kalman_interfaces::msg::WheelState & msg)
  : msg_(msg)
  {}
  ::kalman_interfaces::msg::WheelState angle(::kalman_interfaces::msg::WheelState::_angle_type arg)
  {
    msg_.angle = std::move(arg);
    return std::move(msg_);
  }

private:
  ::kalman_interfaces::msg::WheelState msg_;
};

class Init_WheelState_velocity
{
public:
  Init_WheelState_velocity()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_WheelState_angle velocity(::kalman_interfaces::msg::WheelState::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_WheelState_angle(msg_);
  }

private:
  ::kalman_interfaces::msg::WheelState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::kalman_interfaces::msg::WheelState>()
{
  return kalman_interfaces::msg::builder::Init_WheelState_velocity();
}

}  // namespace kalman_interfaces

#endif  // KALMAN_INTERFACES__MSG__DETAIL__WHEEL_STATE__BUILDER_HPP_
