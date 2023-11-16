// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from kalman_interfaces:msg/WheelStates.idl
// generated code does not contain a copyright notice

#ifndef KALMAN_INTERFACES__MSG__DETAIL__WHEEL_STATES__BUILDER_HPP_
#define KALMAN_INTERFACES__MSG__DETAIL__WHEEL_STATES__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "kalman_interfaces/msg/detail/wheel_states__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace kalman_interfaces
{

namespace msg
{

namespace builder
{

class Init_WheelStates_back_right
{
public:
  explicit Init_WheelStates_back_right(::kalman_interfaces::msg::WheelStates & msg)
  : msg_(msg)
  {}
  ::kalman_interfaces::msg::WheelStates back_right(::kalman_interfaces::msg::WheelStates::_back_right_type arg)
  {
    msg_.back_right = std::move(arg);
    return std::move(msg_);
  }

private:
  ::kalman_interfaces::msg::WheelStates msg_;
};

class Init_WheelStates_back_left
{
public:
  explicit Init_WheelStates_back_left(::kalman_interfaces::msg::WheelStates & msg)
  : msg_(msg)
  {}
  Init_WheelStates_back_right back_left(::kalman_interfaces::msg::WheelStates::_back_left_type arg)
  {
    msg_.back_left = std::move(arg);
    return Init_WheelStates_back_right(msg_);
  }

private:
  ::kalman_interfaces::msg::WheelStates msg_;
};

class Init_WheelStates_front_right
{
public:
  explicit Init_WheelStates_front_right(::kalman_interfaces::msg::WheelStates & msg)
  : msg_(msg)
  {}
  Init_WheelStates_back_left front_right(::kalman_interfaces::msg::WheelStates::_front_right_type arg)
  {
    msg_.front_right = std::move(arg);
    return Init_WheelStates_back_left(msg_);
  }

private:
  ::kalman_interfaces::msg::WheelStates msg_;
};

class Init_WheelStates_front_left
{
public:
  Init_WheelStates_front_left()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_WheelStates_front_right front_left(::kalman_interfaces::msg::WheelStates::_front_left_type arg)
  {
    msg_.front_left = std::move(arg);
    return Init_WheelStates_front_right(msg_);
  }

private:
  ::kalman_interfaces::msg::WheelStates msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::kalman_interfaces::msg::WheelStates>()
{
  return kalman_interfaces::msg::builder::Init_WheelStates_front_left();
}

}  // namespace kalman_interfaces

#endif  // KALMAN_INTERFACES__MSG__DETAIL__WHEEL_STATES__BUILDER_HPP_
