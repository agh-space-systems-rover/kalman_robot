// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from kalman_interfaces:msg/MasterMessage.idl
// generated code does not contain a copyright notice

#ifndef KALMAN_INTERFACES__MSG__DETAIL__MASTER_MESSAGE__BUILDER_HPP_
#define KALMAN_INTERFACES__MSG__DETAIL__MASTER_MESSAGE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "kalman_interfaces/msg/detail/master_message__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace kalman_interfaces
{

namespace msg
{

namespace builder
{

class Init_MasterMessage_data
{
public:
  explicit Init_MasterMessage_data(::kalman_interfaces::msg::MasterMessage & msg)
  : msg_(msg)
  {}
  ::kalman_interfaces::msg::MasterMessage data(::kalman_interfaces::msg::MasterMessage::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::kalman_interfaces::msg::MasterMessage msg_;
};

class Init_MasterMessage_cmd
{
public:
  Init_MasterMessage_cmd()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MasterMessage_data cmd(::kalman_interfaces::msg::MasterMessage::_cmd_type arg)
  {
    msg_.cmd = std::move(arg);
    return Init_MasterMessage_data(msg_);
  }

private:
  ::kalman_interfaces::msg::MasterMessage msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::kalman_interfaces::msg::MasterMessage>()
{
  return kalman_interfaces::msg::builder::Init_MasterMessage_cmd();
}

}  // namespace kalman_interfaces

#endif  // KALMAN_INTERFACES__MSG__DETAIL__MASTER_MESSAGE__BUILDER_HPP_
