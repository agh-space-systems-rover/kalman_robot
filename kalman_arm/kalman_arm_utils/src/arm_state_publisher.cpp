#include "builtin_interfaces/msg/time.hpp"
#include "kalman_interfaces/msg/arm_state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <cstdint>
#include <string>

class ArmStatePublisher : public rclcpp::Node
{
public:
  ArmStatePublisher() : Node("arm_state_publisher")
  {
    this->declare_parameter<double>("rate", 10.0);
    this->get_parameter("rate", rate_);

    last_time_ = rclcpp::Clock().now();

    arm_state_pub_ = this->create_publisher<kalman_interfaces::msg::ArmState>("arm_state", 10);

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10, [this](sensor_msgs::msg::JointState::SharedPtr msg) { sub_callback(msg); });
  }

private:
  void sub_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    auto now = rclcpp::Clock().now();
    if (now - last_time_ > rclcpp::Duration::from_seconds(1.0 / rate_))
    {
      auto state_msg = kalman_interfaces::msg::ArmState();

      for (int i = 0; i < 6; i++)
      {
        uint8_t msg_len = msg->name[i].length();
        switch (msg->name[i][msg_len - 1])
        {
          case '1':
            state_msg.joint_1 = msg->position[i];
            break;
          case '2':
            state_msg.joint_2 = msg->position[i];
            break;
          case '3':
            state_msg.joint_3 = msg->position[i];
            break;
          case '4':
            state_msg.joint_4 = msg->position[i];
            break;
          case '5':
            state_msg.joint_5 = msg->position[i];
            break;
          case '6':
            state_msg.joint_6 = msg->position[i];
            break;
        }
      }

      arm_state_pub_->publish(state_msg);
      last_time_ = now;
    }
  }

  double rate_;
  rclcpp::Time last_time_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<kalman_interfaces::msg::ArmState>::SharedPtr arm_state_pub_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArmStatePublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
