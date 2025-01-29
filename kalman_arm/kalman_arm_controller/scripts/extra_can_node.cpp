#include <cstdint>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/experimental/buffers/intra_process_buffer.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp/timer.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <string>
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/u_int8.hpp"
extern "C" {
#include "kalman_arm_controller/can_libs/can_driver.hpp"
}
#include <stdint.h>

namespace kalman_arm
{
class ExtraCanNode : public rclcpp::Node
{
private:
  CAN_driver::DriverVars_t extra_driver_ = {};

  uint16_t gripper_position_;

  uint16_t max_gripper_;
  uint16_t min_gripper_;
  uint16_t start_pose_;
  std::string can_interface_;

  rclcpp::TimerBase::SharedPtr write_timer_;
  rclcpp::TimerBase::SharedPtr read_timer_;

  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr gripper_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr fastclick_sub_;

  uint16_t calculate_gripper_position(int8_t position)
  {
    gripper_position_ += position;
    if (gripper_position_ > max_gripper_)
    {
      gripper_position_ = max_gripper_;
    }
    else if (gripper_position_ < min_gripper_)
    {
      gripper_position_ = min_gripper_;
    }
    return gripper_position_;
  }

public:
  explicit ExtraCanNode(const rclcpp::NodeOptions& options) : Node("extra_can_node", options)
  {
    this->declare_parameter<uint16_t>("max_gripper", 1250);
    this->get_parameter("max_gripper", max_gripper_);
    this->declare_parameter<uint16_t>("min_gripper", 330);
    this->get_parameter("min_gripper", min_gripper_);
    this->declare_parameter<uint16_t>("start_pose", 330);
    this->get_parameter("start_pose", start_pose_);
    this->declare_parameter<std::string>("can_interface", "can0");
    this->get_parameter("can_interface", can_interface_);

    gripper_position_ = start_pose_;

    CAN_driver::init(&extra_driver_, can_interface_.c_str());

    gripper_sub_ = this->create_subscription<std_msgs::msg::Int8>(
        "gripper/command_incremental", rclcpp::SystemDefaultsQoS(), [this](const std_msgs::msg::Int8::SharedPtr msg) {
          CAN_driver::write_gripper_position(&extra_driver_, calculate_gripper_position(msg->data));
        });

    fastclick_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
        "fastclick", rclcpp::SystemDefaultsQoS(),
        [this](const std_msgs::msg::UInt8::SharedPtr msg) { CAN_driver::write_fastclick(&extra_driver_, msg->data); });
  }

  ~ExtraCanNode() override = default;
};

}  // namespace kalman_arm

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(kalman_arm::ExtraCanNode)
