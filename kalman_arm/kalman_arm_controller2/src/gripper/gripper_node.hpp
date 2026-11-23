#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "can/i_can_interface.hpp"
#include "gripper/gripper_can_protocol.hpp"

namespace kalman_arm::gripper {

class GripperNode : public rclcpp::Node {
public:
  explicit GripperNode(std::unique_ptr<can::ICanInterface> can_iface);
  ~GripperNode() override;

private:
  void onCanFrame(const can::CanFrame& frame);
  double degToUrdfRad(int16_t upper_deg, int16_t lower_deg) const;
  int16_t clampUpper(int16_t deg) const;
  int16_t clampLower(int16_t deg) const;

  std::unique_ptr<can::ICanInterface> can_;
  std::string can_iface_name_;
  std::string joint_name_;

  mutable std::mutex limits_mutex_;
  int16_t upper_min_deg_{90};
  int16_t upper_max_deg_{180};
  int16_t lower_min_deg_{90};
  int16_t lower_max_deg_{180};

  double min_urdf_rad_{-0.10};
  double max_urdf_rad_{1.57};
  bool block_cmd_during_calib_{true};

  std::atomic<uint8_t> calib_status_{0};

  // Publishery
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pos_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr pos_dual_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr temp_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr torque_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr calib_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;

  // Subskrypcje i serwisy
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr cmd_pos_sub_;
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr cmd_pos_dual_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr torque_limit_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr torque_limit_dual_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr autoclick_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calib_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr soft_reset_srv_;
};

}  // namespace kalman_arm::gripper