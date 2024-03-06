// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from kalman_interfaces:srv/SetUeuosColor.idl
// generated code does not contain a copyright notice

#ifndef KALMAN_INTERFACES__SRV__DETAIL__SET_UEUOS_COLOR__BUILDER_HPP_
#define KALMAN_INTERFACES__SRV__DETAIL__SET_UEUOS_COLOR__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "kalman_interfaces/srv/detail/set_ueuos_color__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace kalman_interfaces
{

namespace srv
{

namespace builder
{

class Init_SetUeuosColor_Request_color
{
public:
  Init_SetUeuosColor_Request_color()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::kalman_interfaces::srv::SetUeuosColor_Request color(::kalman_interfaces::srv::SetUeuosColor_Request::_color_type arg)
  {
    msg_.color = std::move(arg);
    return std::move(msg_);
  }

private:
  ::kalman_interfaces::srv::SetUeuosColor_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::kalman_interfaces::srv::SetUeuosColor_Request>()
{
  return kalman_interfaces::srv::builder::Init_SetUeuosColor_Request_color();
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
auto build<::kalman_interfaces::srv::SetUeuosColor_Response>()
{
  return ::kalman_interfaces::srv::SetUeuosColor_Response(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace kalman_interfaces

#endif  // KALMAN_INTERFACES__SRV__DETAIL__SET_UEUOS_COLOR__BUILDER_HPP_
