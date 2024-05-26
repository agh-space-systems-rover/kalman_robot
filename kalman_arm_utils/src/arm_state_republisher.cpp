#include "builtin_interfaces/msg/time.hpp"
#include "kalman_interfaces/msg/arm_state.hpp"
#include "kalman_interfaces/msg/master_message.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/int8.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <cstdint>

class ArmStateRepublisher : public rclcpp::Node
{
public:
  ArmStateRepublisher() : Node("arm_state_republisher")
  {
    this->declare_parameter<double>("rate", 10.0);
    this->get_parameter("rate", rate_);

    last_time_ = rclcpp::Clock().now();

    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/arm_controllers/joint_states", 10);

    arm_state_sub_ = this->create_subscription<kalman_interfaces::msg::ArmState>(
        "arm_state", 10, [this](kalman_interfaces::msg::ArmState::SharedPtr msg) { sub_callback(msg); });
  }

private:
  void sub_callback(const kalman_interfaces::msg::ArmState::SharedPtr msg)
  {
    auto now = rclcpp::Clock().now();
    if (now - last_time_ > rclcpp::Duration::from_seconds(1.0 / rate_))
    {
      auto joint_msg = sensor_msgs::msg::JointState();
      joint_msg.name = { "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "gripper_moving_joint" };
      joint_msg.position.resize(7, 0);
      joint_msg.position[0] = msg->joint_1;
      joint_msg.position[1] = msg->joint_2;
      joint_msg.position[2] = msg->joint_3;
      joint_msg.position[3] = msg->joint_4;
      joint_msg.position[4] = msg->joint_5;
      joint_msg.position[5] = msg->joint_6;
      joint_msg.position[6] = 0;

      joint_state_pub_->publish(joint_msg);
    }
  }

  double rate_;
  rclcpp::Time last_time_;
  rclcpp::Subscription<kalman_interfaces::msg::ArmState>::SharedPtr arm_state_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArmStateRepublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
