// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from kalman_interfaces:srv/SetUeuosMode.idl
// generated code does not contain a copyright notice

#ifndef KALMAN_INTERFACES__SRV__DETAIL__SET_UEUOS_MODE__BUILDER_HPP_
#define KALMAN_INTERFACES__SRV__DETAIL__SET_UEUOS_MODE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "kalman_interfaces/srv/detail/set_ueuos_mode__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace kalman_interfaces
{

namespace srv
{

namespace builder
{

class Init_SetUeuosMode_Request_mode
{
public:
  Init_SetUeuosMode_Request_mode()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::kalman_interfaces::srv::SetUeuosMode_Request mode(::kalman_interfaces::srv::SetUeuosMode_Request::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return std::move(msg_);
  }

private:
  ::kalman_interfaces::srv::SetUeuosMode_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::kalman_interfaces::srv::SetUeuosMode_Request>()
{
  return kalman_interfaces::srv::builder::Init_SetUeuosMode_Request_mode();
}

}  // namespace kalman_interfaces


namespace kalman_interfaces
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::kalman_interfaces::srv::SetUeuosMode_Response>()
{
  return ::kalman_interfaces::srv::SetUeuosMode_Response(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace kalman_interfaces

#endif  // KALMAN_INTERFACES__SRV__DETAIL__SET_UEUOS_MODE__BUILDER_HPP_
