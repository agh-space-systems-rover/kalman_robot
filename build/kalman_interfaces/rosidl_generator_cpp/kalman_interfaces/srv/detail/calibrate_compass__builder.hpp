// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from kalman_interfaces:srv/CalibrateCompass.idl
// generated code does not contain a copyright notice

#ifndef KALMAN_INTERFACES__SRV__DETAIL__CALIBRATE_COMPASS__BUILDER_HPP_
#define KALMAN_INTERFACES__SRV__DETAIL__CALIBRATE_COMPASS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "kalman_interfaces/srv/detail/calibrate_compass__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace kalman_interfaces
{

namespace srv
{

namespace builder
{

class Init_CalibrateCompass_Request_angular_velocity
{
public:
  explicit Init_CalibrateCompass_Request_angular_velocity(::kalman_interfaces::srv::CalibrateCompass_Request & msg)
  : msg_(msg)
  {}
  ::kalman_interfaces::srv::CalibrateCompass_Request angular_velocity(::kalman_interfaces::srv::CalibrateCompass_Request::_angular_velocity_type arg)
  {
    msg_.angular_velocity = std::move(arg);
    return std::move(msg_);
  }

private:
  ::kalman_interfaces::srv::CalibrateCompass_Request msg_;
};

class Init_CalibrateCompass_Request_duration
{
public:
  Init_CalibrateCompass_Request_duration()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CalibrateCompass_Request_angular_velocity duration(::kalman_interfaces::srv::CalibrateCompass_Request::_duration_type arg)
  {
    msg_.duration = std::move(arg);
    return Init_CalibrateCompass_Request_angular_velocity(msg_);
  }

private:
  ::kalman_interfaces::srv::CalibrateCompass_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::kalman_interfaces::srv::CalibrateCompass_Request>()
{
  return kalman_interfaces::srv::builder::Init_CalibrateCompass_Request_duration();
}

}  // namespace kalman_interfaces


namespace kalman_interfaces
{

namespace srv
{

namespace builder
{

class Init_CalibrateCompass_Response_cc_t5
{
public:
  explicit Init_CalibrateCompass_Response_cc_t5(::kalman_interfaces::srv::CalibrateCompass_Response & msg)
  : msg_(msg)
  {}
  ::kalman_interfaces::srv::CalibrateCompass_Response cc_t5(::kalman_interfaces::srv::CalibrateCompass_Response::_cc_t5_type arg)
  {
    msg_.cc_t5 = std::move(arg);
    return std::move(msg_);
  }

private:
  ::kalman_interfaces::srv::CalibrateCompass_Response msg_;
};

class Init_CalibrateCompass_Response_cc_t4
{
public:
  explicit Init_CalibrateCompass_Response_cc_t4(::kalman_interfaces::srv::CalibrateCompass_Response & msg)
  : msg_(msg)
  {}
  Init_CalibrateCompass_Response_cc_t5 cc_t4(::kalman_interfaces::srv::CalibrateCompass_Response::_cc_t4_type arg)
  {
    msg_.cc_t4 = std::move(arg);
    return Init_CalibrateCompass_Response_cc_t5(msg_);
  }

private:
  ::kalman_interfaces::srv::CalibrateCompass_Response msg_;
};

class Init_CalibrateCompass_Response_cc_t3
{
public:
  explicit Init_CalibrateCompass_Response_cc_t3(::kalman_interfaces::srv::CalibrateCompass_Response & msg)
  : msg_(msg)
  {}
  Init_CalibrateCompass_Response_cc_t4 cc_t3(::kalman_interfaces::srv::CalibrateCompass_Response::_cc_t3_type arg)
  {
    msg_.cc_t3 = std::move(arg);
    return Init_CalibrateCompass_Response_cc_t4(msg_);
  }

private:
  ::kalman_interfaces::srv::CalibrateCompass_Response msg_;
};

class Init_CalibrateCompass_Response_cc_t2
{
public:
  explicit Init_CalibrateCompass_Response_cc_t2(::kalman_interfaces::srv::CalibrateCompass_Response & msg)
  : msg_(msg)
  {}
  Init_CalibrateCompass_Response_cc_t3 cc_t2(::kalman_interfaces::srv::CalibrateCompass_Response::_cc_t2_type arg)
  {
    msg_.cc_t2 = std::move(arg);
    return Init_CalibrateCompass_Response_cc_t3(msg_);
  }

private:
  ::kalman_interfaces::srv::CalibrateCompass_Response msg_;
};

class Init_CalibrateCompass_Response_cc_t1
{
public:
  explicit Init_CalibrateCompass_Response_cc_t1(::kalman_interfaces::srv::CalibrateCompass_Response & msg)
  : msg_(msg)
  {}
  Init_CalibrateCompass_Response_cc_t2 cc_t1(::kalman_interfaces::srv::CalibrateCompass_Response::_cc_t1_type arg)
  {
    msg_.cc_t1 = std::move(arg);
    return Init_CalibrateCompass_Response_cc_t2(msg_);
  }

private:
  ::kalman_interfaces::srv::CalibrateCompass_Response msg_;
};

class Init_CalibrateCompass_Response_cc_t0
{
public:
  explicit Init_CalibrateCompass_Response_cc_t0(::kalman_interfaces::srv::CalibrateCompass_Response & msg)
  : msg_(msg)
  {}
  Init_CalibrateCompass_Response_cc_t1 cc_t0(::kalman_interfaces::srv::CalibrateCompass_Response::_cc_t0_type arg)
  {
    msg_.cc_t0 = std::move(arg);
    return Init_CalibrateCompass_Response_cc_t1(msg_);
  }

private:
  ::kalman_interfaces::srv::CalibrateCompass_Response msg_;
};

class Init_CalibrateCompass_Response_cc_gain2
{
public:
  explicit Init_CalibrateCompass_Response_cc_gain2(::kalman_interfaces::srv::CalibrateCompass_Response & msg)
  : msg_(msg)
  {}
  Init_CalibrateCompass_Response_cc_t0 cc_gain2(::kalman_interfaces::srv::CalibrateCompass_Response::_cc_gain2_type arg)
  {
    msg_.cc_gain2 = std::move(arg);
    return Init_CalibrateCompass_Response_cc_t0(msg_);
  }

private:
  ::kalman_interfaces::srv::CalibrateCompass_Response msg_;
};

class Init_CalibrateCompass_Response_cc_gain1
{
public:
  explicit Init_CalibrateCompass_Response_cc_gain1(::kalman_interfaces::srv::CalibrateCompass_Response & msg)
  : msg_(msg)
  {}
  Init_CalibrateCompass_Response_cc_gain2 cc_gain1(::kalman_interfaces::srv::CalibrateCompass_Response::_cc_gain1_type arg)
  {
    msg_.cc_gain1 = std::move(arg);
    return Init_CalibrateCompass_Response_cc_gain2(msg_);
  }

private:
  ::kalman_interfaces::srv::CalibrateCompass_Response msg_;
};

class Init_CalibrateCompass_Response_cc_gain0
{
public:
  explicit Init_CalibrateCompass_Response_cc_gain0(::kalman_interfaces::srv::CalibrateCompass_Response & msg)
  : msg_(msg)
  {}
  Init_CalibrateCompass_Response_cc_gain1 cc_gain0(::kalman_interfaces::srv::CalibrateCompass_Response::_cc_gain0_type arg)
  {
    msg_.cc_gain0 = std::move(arg);
    return Init_CalibrateCompass_Response_cc_gain1(msg_);
  }

private:
  ::kalman_interfaces::srv::CalibrateCompass_Response msg_;
};

class Init_CalibrateCompass_Response_cc_offset2
{
public:
  explicit Init_CalibrateCompass_Response_cc_offset2(::kalman_interfaces::srv::CalibrateCompass_Response & msg)
  : msg_(msg)
  {}
  Init_CalibrateCompass_Response_cc_gain0 cc_offset2(::kalman_interfaces::srv::CalibrateCompass_Response::_cc_offset2_type arg)
  {
    msg_.cc_offset2 = std::move(arg);
    return Init_CalibrateCompass_Response_cc_gain0(msg_);
  }

private:
  ::kalman_interfaces::srv::CalibrateCompass_Response msg_;
};

class Init_CalibrateCompass_Response_cc_offset1
{
public:
  explicit Init_CalibrateCompass_Response_cc_offset1(::kalman_interfaces::srv::CalibrateCompass_Response & msg)
  : msg_(msg)
  {}
  Init_CalibrateCompass_Response_cc_offset2 cc_offset1(::kalman_interfaces::srv::CalibrateCompass_Response::_cc_offset1_type arg)
  {
    msg_.cc_offset1 = std::move(arg);
    return Init_CalibrateCompass_Response_cc_offset2(msg_);
  }

private:
  ::kalman_interfaces::srv::CalibrateCompass_Response msg_;
};

class Init_CalibrateCompass_Response_cc_offset0
{
public:
  explicit Init_CalibrateCompass_Response_cc_offset0(::kalman_interfaces::srv::CalibrateCompass_Response & msg)
  : msg_(msg)
  {}
  Init_CalibrateCompass_Response_cc_offset1 cc_offset0(::kalman_interfaces::srv::CalibrateCompass_Response::_cc_offset0_type arg)
  {
    msg_.cc_offset0 = std::move(arg);
    return Init_CalibrateCompass_Response_cc_offset1(msg_);
  }

private:
  ::kalman_interfaces::srv::CalibrateCompass_Response msg_;
};

class Init_CalibrateCompass_Response_cc_mag_field
{
public:
  explicit Init_CalibrateCompass_Response_cc_mag_field(::kalman_interfaces::srv::CalibrateCompass_Response & msg)
  : msg_(msg)
  {}
  Init_CalibrateCompass_Response_cc_offset0 cc_mag_field(::kalman_interfaces::srv::CalibrateCompass_Response::_cc_mag_field_type arg)
  {
    msg_.cc_mag_field = std::move(arg);
    return Init_CalibrateCompass_Response_cc_offset0(msg_);
  }

private:
  ::kalman_interfaces::srv::CalibrateCompass_Response msg_;
};

class Init_CalibrateCompass_Response_success
{
public:
  Init_CalibrateCompass_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CalibrateCompass_Response_cc_mag_field success(::kalman_interfaces::srv::CalibrateCompass_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_CalibrateCompass_Response_cc_mag_field(msg_);
  }

private:
  ::kalman_interfaces::srv::CalibrateCompass_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::kalman_interfaces::srv::CalibrateCompass_Response>()
{
  return kalman_interfaces::srv::builder::Init_CalibrateCompass_Response_success();
}

}  // namespace kalman_interfaces

#endif  // KALMAN_INTERFACES__SRV__DETAIL__CALIBRATE_COMPASS__BUILDER_HPP_
