#include <cstdint>
#include <ratio>
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
#include <thread>
#include "kalman_arm_controller/can_libs/can_messages.hpp"
#include "std_msgs/msg/int8.hpp"
extern "C" {
#include "kalman_arm_controller/can_libs/can_driver.hpp"
#include "kalman_arm_controller/can_libs/can_handlers.hpp"
}
#include "kalman_arm_controller/can_libs/can_types.hpp"
#include <map>
#include <future>
#include <chrono>
#include <stdint.h>

const uint16_t WRITE_CALLBACK_PERIOD_MS = 10;
const uint16_t READ_CALLBACK_PERIOD_MS = 10;
const double JOINT_TIMEOUT = 0.5;

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

  //   std::future<void> writer;
  rclcpp::TimerBase::SharedPtr write_timer_;
  rclcpp::TimerBase::SharedPtr read_timer_;

  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr gripper_sub_;

  //   std::unordered_map<uint8_t, canCmdHandler_t> EXTRA_HANDLES = {
  //     { CMD_GET_GRIPPER, { CMD_GET_GRIPPER, sizeof(cmdGetGripper_t), CAN_handlers::handle_gripper_status } },
  //     //     //   { CMD_JOINT_STATUS, { CMD_JOINT_STATUS, sizeof(jointMotorStatus_t), handle_joint_status } },
  //     //     //   { CMD_JOINT_FAST_STATUS, { CMD_JOINT_FAST_STATUS, sizeof(jointMotorFastStatus_t),
  //     handle_joint_fast_status } }
  //   };

  //   void writeCan()
  //   {
  //     CAN_driver::write_gripper_position(&extra_driver_, gripper_position_);
  //   }

  //   void readCan()
  //   {
  //     std::lock_guard<std::mutex> lock(this->extra_driver_.m_read);
  //     // Do something related to variables that read uses
  //     gripper_position_ = CAN_vars::gripper_position;
  //   }

  uint16_t calculate_gripper_position(int8_t position)
  {
    gripper_position_ += position;
    if (gripper_position_ > max_gripper_)  // FIXME make this dynamic
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
  ExtraCanNode(const rclcpp::NodeOptions& options) : Node("extra_can_node", options)
  {
    this->declare_parameter<uint16_t>("max_gripper", 1250);
    this->get_parameter("max_gripper", max_gripper_);
    this->declare_parameter<uint16_t>("min_gripper", 330);
    this->get_parameter("min_gripper", min_gripper_);
    this->declare_parameter<uint16_t>("start_pose", 330);
    this->get_parameter("start_pose", start_pose_);

    gripper_position_ = start_pose_;

    CAN_driver::init(&extra_driver_, "can0");
    // CAN_driver::startExtraRead(&extra_driver_, &EXTRA_HANDLES);

    // joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>(JOINT_TOPIC, rclcpp::SystemDefaultsQoS());
    gripper_sub_ = this->create_subscription<std_msgs::msg::Int8>(
        "gripper/command_incremental", rclcpp::SystemDefaultsQoS(), [&](const std_msgs::msg::Int8::SharedPtr msg) {
          CAN_driver::write_gripper_position(&extra_driver_, calculate_gripper_position(msg->data));
        });

    // write_timer_ = create_wall_timer(std::chrono::milliseconds(WRITE_CALLBACK_PERIOD_MS),
    //                                  std::bind(&ExtraCanNode::writeCan, this));
    // read_timer_ =
    //     create_wall_timer(std::chrono::milliseconds(READ_CALLBACK_PERIOD_MS), std::bind(&ExtraCanNode::readCan, this));
  }

  ~ExtraCanNode() override
  {
  }
};

}  // namespace kalman_arm

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(kalman_arm::ExtraCanNode)