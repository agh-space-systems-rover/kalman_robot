#include <cstdint>
#include <ratio>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/experimental/buffers/intra_process_buffer.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/event_handler.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp/timer.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <thread>
#include "kalman_arm_controller/can_libs/can_messages.hpp"
#include "std_msgs/msg/u_int16.hpp"
extern "C" {
#include "kalman_arm_controller/can_libs/can_driver.hpp"
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

//   std::future<void> writer;
  rclcpp::TimerBase::SharedPtr write_timer_;
  rclcpp::TimerBase::SharedPtr read_timer_;

  rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr gripper_sub_;

  //   std::unordered_map<uint8_t, canCmdHandler_t> EXTRA_HANDLES = {
  //     // { CMD_SET_GRIPPER, { CMD_SET_GRIPPER, sizeof(cmdSetGripper_t), handle_set_gripper } },
  //     //   { CMD_JOINT_STATUS, { CMD_JOINT_STATUS, sizeof(jointMotorStatus_t), handle_joint_status } },
  //     //   { CMD_JOINT_FAST_STATUS, { CMD_JOINT_FAST_STATUS, sizeof(jointMotorFastStatus_t), handle_joint_fast_status } }
  //   };

  void writeCan()
  {
   RCLCPP_INFO(this->get_logger(), "Writing can"); 
    CAN_driver::write_gripper_position(&extra_driver_, gripper_position_);
   RCLCPP_INFO(this->get_logger(), "Written can"); 
    // if (writer.valid() && writer.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
    // {
    //   RCLCPP_WARN(this->get_logger(), "Previous write still in progress");
    // }
    // else
    // {
    //   {
    //     // std::lock_guard<std::mutex> lock(this->extra_driver_.m_write);
    //     // Do something related to variables that write uses
    //   }
    //   // Run write in a separate thread
    //   printf("!!!!!! Starting wirter");
    //   writer = std::async(std::launch::async, [&] { CAN_driver::write_gripper_position(gripper_position_); });
    // }
  }

  void readCan()
  {
    // std::lock_guard<std::mutex> lock(this->extra_driver_.m_read);
    // Do something related to variables that read uses
    // if ((now() - jointsVelData.lastReceivedTime).seconds() > JOINT_TIMEOUT)
    // {
    //   RCLCPP_WARN(rclcpp::get_logger("master_node_logger"), "Joint data timeout");
    // }
    // else
    // {
    //   auto joint_msg = std::make_unique<control_msgs::msg::JointJog>();
    //   joint_msg->header.stamp = this->now();
    //   joint_msg->header.frame_id = "arm_link";
    //   joint_msg->joint_names.push_back("joint_1");
    //   joint_msg->joint_names.push_back("joint_2");
    //   joint_msg->joint_names.push_back("joint_3");
    //   joint_msg->joint_names.push_back("joint_4");
    //   joint_msg->joint_names.push_back("joint_5");
    //   joint_msg->joint_names.push_back("joint_6");
    //   for (uint8_t i = 0; i < 6; i++)
    //   {
    //     joint_msg->velocities.push_back((double)jointsVelData.velocities[i] / 100.0);
    //   }

    //   joint_pub_->publish(std::move(joint_msg));
    // }
  }

public:
  ExtraCanNode(const rclcpp::NodeOptions& options) : Node("extra_can_node", options)
  {
    CAN_driver::init(&extra_driver_, "can0");
    // CAN_driver::startExtraRead(&extra_driver_, &EXTRA_HANDLES);

    // joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>(JOINT_TOPIC, rclcpp::SystemDefaultsQoS());
    gripper_sub_ = this->create_subscription<std_msgs::msg::UInt16>("gripper_position", rclcpp::SystemDefaultsQoS(),
                                                                    [&](const std_msgs::msg::UInt16::SharedPtr msg) {
                                                                    //   gripper_position_ = msg->data;

   RCLCPP_INFO(this->get_logger(), "Sb called"); 
    CAN_driver::write_gripper_position(&extra_driver_, msg->data);
   RCLCPP_INFO(this->get_logger(), "Sb written");
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