#include "gripper/gripper_node.hpp"

#include <algorithm>
#include <stdexcept>

namespace kalman_arm::gripper {

GripperNode::GripperNode(std::unique_ptr<can::ICanInterface> can_iface)
: Node("gripper_node"), can_(std::move(can_iface)) {
  can_iface_name_         = declare_parameter<std::string>("can_interface", "vcan0");
  joint_name_             = declare_parameter<std::string>("joint_name", "arm_joint_jaw");
  upper_min_deg_          = static_cast<int16_t>(declare_parameter<int>("upper_min_deg", 90));
  upper_max_deg_          = static_cast<int16_t>(declare_parameter<int>("upper_max_deg", 180));
  lower_min_deg_          = static_cast<int16_t>(declare_parameter<int>("lower_min_deg", 90));
  lower_max_deg_          = static_cast<int16_t>(declare_parameter<int>("lower_max_deg", 180));
  min_urdf_rad_           = declare_parameter<double>("min_urdf_rad", -0.10);
  max_urdf_rad_           = declare_parameter<double>("max_urdf_rad", 1.57);
  block_cmd_during_calib_ = declare_parameter<bool>("block_cmd_during_calib", true);

  // Publishery telemetrii i RViz
  pos_pub_      = create_publisher<std_msgs::msg::Int16>("/gripper/position", 10);
  pos_dual_pub_ = create_publisher<std_msgs::msg::Int16MultiArray>("/gripper/position_dual", 10);
  temp_pub_     = create_publisher<std_msgs::msg::UInt8MultiArray>("/gripper/temperature", 10);
  torque_pub_   = create_publisher<std_msgs::msg::UInt16MultiArray>("/gripper/torque", 10);
  calib_pub_    = create_publisher<std_msgs::msg::UInt8>("/gripper/calibration_status", 10);
  joint_pub_    = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

  // Komenda wspólna (w stopniach) -> 0xE1
  cmd_pos_sub_ = create_subscription<std_msgs::msg::Int16>(
    "/gripper/command_position", 10,
    [this](const std_msgs::msg::Int16::SharedPtr msg) {
      if (block_cmd_during_calib_ &&
          calib_status_.load() == static_cast<uint8_t>(CalibrationStatus::InProgress)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                             "Ignoring 0xE1: gripper calibration in progress");
        return;
      }
      can_->send(GripperCanProtocol::encodeSetPosition(
        clampUpper(msg->data), clampLower(msg->data)));
    });

  // Komenda osobna [upper_deg, lower_deg] -> 0xE1
  cmd_pos_dual_sub_ = create_subscription<std_msgs::msg::Int16MultiArray>(
    "/gripper/command_position_dual", 10,
    [this](const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
      if (msg->data.size() != 2) {
        RCLCPP_WARN(get_logger(), "command_position_dual requires 2 values [upper_deg, lower_deg]");
        return;
      }
      if (block_cmd_during_calib_ &&
          calib_status_.load() == static_cast<uint8_t>(CalibrationStatus::InProgress)) {
        return;
      }
      can_->send(GripperCanProtocol::encodeSetPosition(
        clampUpper(msg->data[0]), clampLower(msg->data[1])));
    });

  // Limit momentu wspólny (0..1000) -> 0xE7
  torque_limit_sub_ = create_subscription<std_msgs::msg::UInt16>(
    "/gripper/torque_limit", 10,
    [this](const std_msgs::msg::UInt16::SharedPtr msg) {
      can_->send(GripperCanProtocol::encodeSetTorqueLimit(msg->data, msg->data));
    });

  // Limit momentu osobny [upper, lower] -> 0xE7
  torque_limit_dual_sub_ = create_subscription<std_msgs::msg::UInt16MultiArray>(
    "/gripper/torque_limit_dual", 10,
    [this](const std_msgs::msg::UInt16MultiArray::SharedPtr msg) {
      if (msg->data.size() != 2) {
        RCLCPP_WARN(get_logger(), "torque_limit_dual requires 2 values [upper, lower]");
        return;
      }
      can_->send(GripperCanProtocol::encodeSetTorqueLimit(msg->data[0], msg->data[1]));
    });

  // Autoclick PWM (0..180 deg) -> 0xE3
  autoclick_sub_ = create_subscription<std_msgs::msg::UInt8>(
    "/gripper/autoclick_position", 10,
    [this](const std_msgs::msg::UInt8::SharedPtr msg) {
      can_->send(GripperCanProtocol::encodeAutoclickPos(msg->data));
    });

  // Kalibracja momentowa -> 0xEC
  calib_srv_ = create_service<std_srvs::srv::Trigger>(
    "/gripper/calibrate",
    [this](const std_srvs::srv::Trigger::Request::SharedPtr,
           std_srvs::srv::Trigger::Response::SharedPtr res) {
      res->success = can_->send(GripperCanProtocol::encodeCalibrate());
      res->message = res->success ? "Calibration command sent (0xEC)" : "CAN send failed";
    });

  // Soft reset serw Feetech -> 0xE2
  soft_reset_srv_ = create_service<std_srvs::srv::Trigger>(
    "/gripper/soft_reset",
    [this](const std_srvs::srv::Trigger::Request::SharedPtr,
           std_srvs::srv::Trigger::Response::SharedPtr res) {
      res->success = can_->send(GripperCanProtocol::encodeSoftReset());
      res->message = res->success ? "Soft reset command sent (0xE2)" : "CAN send failed";
    });

  // Filtry jądra Linuxa: tylko ramki TX grippera (włącznie z CAN FD 0xED)
  const std::vector<uint32_t> rx_filters = {
    GripperCanProtocol::ID_RETURN_POS,
    GripperCanProtocol::ID_RETURN_TEMP,
    GripperCanProtocol::ID_CALIB_STATUS,
    GripperCanProtocol::ID_RETURN_TORQUE
  };

  if (!can_->open(can_iface_name_, rx_filters, true)) {
    throw std::runtime_error("Failed to open CAN interface: " + can_iface_name_);
  }

  can_->startReceive([this](const can::CanFrame& frame) { onCanFrame(frame); });
  RCLCPP_INFO(get_logger(), "GripperNode started on %s", can_iface_name_.c_str());
}

GripperNode::~GripperNode() {
  if (can_) {
    can_->stopReceive();
    can_->close();
  }
}

int16_t GripperNode::clampUpper(int16_t deg) const {
  std::lock_guard<std::mutex> lock(limits_mutex_);
  return std::clamp(deg, upper_min_deg_, upper_max_deg_);
}

int16_t GripperNode::clampLower(int16_t deg) const {
  std::lock_guard<std::mutex> lock(limits_mutex_);
  return std::clamp(deg, lower_min_deg_, lower_max_deg_);
}

double GripperNode::degToUrdfRad(int16_t upper_deg, int16_t lower_deg) const {
  int16_t u_min, u_max, l_min, l_max;
  {
    std::lock_guard<std::mutex> lock(limits_mutex_);
    u_min = upper_min_deg_;
    u_max = upper_max_deg_;
    l_min = lower_min_deg_;
    l_max = lower_max_deg_;
  }
  double avg_deg = 0.5 * (static_cast<double>(upper_deg) + static_cast<double>(lower_deg));
  double min_deg = 0.5 * (static_cast<double>(u_min) + static_cast<double>(l_min));
  double max_deg = 0.5 * (static_cast<double>(u_max) + static_cast<double>(l_max));

  double clamped = std::clamp(avg_deg, min_deg, max_deg);
  double span    = std::max(1.0, max_deg - min_deg);
  double ratio   = (clamped - min_deg) / span;
  return min_urdf_rad_ + ratio * (max_urdf_rad_ - min_urdf_rad_);
}

void GripperNode::onCanFrame(const can::CanFrame& frame) {
  if (auto pos = GripperCanProtocol::decodePosition(frame)) {
    int16_t avg = static_cast<int16_t>((static_cast<int32_t>(pos->upper) + pos->lower) / 2);

    std_msgs::msg::Int16 pos_msg;
    pos_msg.data = avg;
    pos_pub_->publish(pos_msg);

    std_msgs::msg::Int16MultiArray dual_msg;
    dual_msg.data = {pos->upper, pos->lower};
    pos_dual_pub_->publish(dual_msg);

    sensor_msgs::msg::JointState js;
    js.header.stamp = now();
    js.name         = {joint_name_};
    js.position     = {degToUrdfRad(pos->upper, pos->lower)};
    joint_pub_->publish(js);
    return;
  }

  if (auto temp = GripperCanProtocol::decodeTemperature(frame)) {
    std_msgs::msg::UInt8MultiArray msg;
    msg.data = {temp->upper, temp->lower};
    temp_pub_->publish(msg);
    return;
  }

  if (auto torque = GripperCanProtocol::decodeTorque(frame)) {
    std_msgs::msg::UInt16MultiArray msg;
    msg.data = {torque->upper, torque->lower};
    torque_pub_->publish(msg);
    return;
  }

  if (auto report = GripperCanProtocol::decodeCalibrationStatus(frame)) {
    uint8_t status_val = static_cast<uint8_t>(report->status);
    calib_status_.store(status_val);

    if (report->status == CalibrationStatus::Ok &&
        report->upper_max_deg > report->upper_min_deg &&
        report->lower_max_deg > report->lower_min_deg) {
      std::lock_guard<std::mutex> lock(limits_mutex_);
      upper_min_deg_ = report->upper_min_deg;
      upper_max_deg_ = report->upper_max_deg;
      lower_min_deg_ = report->lower_min_deg;
      lower_max_deg_ = report->lower_max_deg;
      RCLCPP_INFO(get_logger(),
                  "Calibration OK. New limits: upper=[%d, %d] deg, lower=[%d, %d] deg",
                  upper_min_deg_, upper_max_deg_, lower_min_deg_, lower_max_deg_);
    }

    std_msgs::msg::UInt8 st_msg;
    st_msg.data = status_val;
    calib_pub_->publish(st_msg);
  }
}

}  // namespace kalman_arm::gripper