// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from service_based_nav2_controller_srvs:srv/ComputeVelocityCommands.idl
// generated code does not contain a copyright notice

#ifndef SERVICE_BASED_NAV2_CONTROLLER_SRVS__SRV__DETAIL__COMPUTE_VELOCITY_COMMANDS__BUILDER_HPP_
#define SERVICE_BASED_NAV2_CONTROLLER_SRVS__SRV__DETAIL__COMPUTE_VELOCITY_COMMANDS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "service_based_nav2_controller_srvs/srv/detail/compute_velocity_commands__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace service_based_nav2_controller_srvs
{

namespace srv
{

namespace builder
{

class Init_ComputeVelocityCommands_Request_path
{
public:
  explicit Init_ComputeVelocityCommands_Request_path(::service_based_nav2_controller_srvs::srv::ComputeVelocityCommands_Request & msg)
  : msg_(msg)
  {}
  ::service_based_nav2_controller_srvs::srv::ComputeVelocityCommands_Request path(::service_based_nav2_controller_srvs::srv::ComputeVelocityCommands_Request::_path_type arg)
  {
    msg_.path = std::move(arg);
    return std::move(msg_);
  }

private:
  ::service_based_nav2_controller_srvs::srv::ComputeVelocityCommands_Request msg_;
};

class Init_ComputeVelocityCommands_Request_velocity
{
public:
  explicit Init_ComputeVelocityCommands_Request_velocity(::service_based_nav2_controller_srvs::srv::ComputeVelocityCommands_Request & msg)
  : msg_(msg)
  {}
  Init_ComputeVelocityCommands_Request_path velocity(::service_based_nav2_controller_srvs::srv::ComputeVelocityCommands_Request::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_ComputeVelocityCommands_Request_path(msg_);
  }

private:
  ::service_based_nav2_controller_srvs::srv::ComputeVelocityCommands_Request msg_;
};

class Init_ComputeVelocityCommands_Request_pose
{
public:
  Init_ComputeVelocityCommands_Request_pose()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ComputeVelocityCommands_Request_velocity pose(::service_based_nav2_controller_srvs::srv::ComputeVelocityCommands_Request::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return Init_ComputeVelocityCommands_Request_velocity(msg_);
  }

private:
  ::service_based_nav2_controller_srvs::srv::ComputeVelocityCommands_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::service_based_nav2_controller_srvs::srv::ComputeVelocityCommands_Request>()
{
  return service_based_nav2_controller_srvs::srv::builder::Init_ComputeVelocityCommands_Request_pose();
}

}  // namespace service_based_nav2_controller_srvs


namespace service_based_nav2_controller_srvs
{

namespace srv
{

namespace builder
{

class Init_ComputeVelocityCommands_Response_cmd_vel
{
public:
  Init_ComputeVelocityCommands_Response_cmd_vel()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::service_based_nav2_controller_srvs::srv::ComputeVelocityCommands_Response cmd_vel(::service_based_nav2_controller_srvs::srv::ComputeVelocityCommands_Response::_cmd_vel_type arg)
  {
    msg_.cmd_vel = std::move(arg);
    return std::move(msg_);
  }

private:
  ::service_based_nav2_controller_srvs::srv::ComputeVelocityCommands_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::service_based_nav2_controller_srvs::srv::ComputeVelocityCommands_Response>()
{
  return service_based_nav2_controller_srvs::srv::builder::Init_ComputeVelocityCommands_Response_cmd_vel();
}

}  // namespace service_based_nav2_controller_srvs

#endif  // SERVICE_BASED_NAV2_CONTROLLER_SRVS__SRV__DETAIL__COMPUTE_VELOCITY_COMMANDS__BUILDER_HPP_
