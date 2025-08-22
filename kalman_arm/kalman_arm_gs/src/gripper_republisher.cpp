#include "builtin_interfaces/msg/time.hpp"
#include "kalman_interfaces/msg/arm_compressed.hpp"
#include "kalman_interfaces/msg/arm_fk_command.hpp"
#include "kalman_interfaces/msg/master_message.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/int8.hpp"
#include <chrono>
#include <cstdint>

class GripperRepublisher : public rclcpp::Node
{
public:
  GripperRepublisher() : Node("gripper_republisher")
  {
    this->declare_parameter<double>("rate", 10.0);
    this->get_parameter("rate", rate_);

    this->declare_parameter<double>("gripper_scale", 10.0);
    this->get_parameter("gripper_scale", gripper_scale_);

    fk_last_time_ = rclcpp::Clock().now();
    spacenav_last_time_ = rclcpp::Clock().now();

    gripper_pub_ = this->create_publisher<std_msgs::msg::Int8>("/gripper/command_incremental", 10);

    fk_sub_ = this->create_subscription<kalman_interfaces::msg::ArmFkCommand>(
        "/station/arm/fk/command", 10,
        [this](kalman_interfaces::msg::ArmFkCommand::SharedPtr msg) { fk_callback(msg); });

    spacenav_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/spacenav/joy", 10, [this](sensor_msgs::msg::Joy::SharedPtr msg) { spacenav_callback(msg); });
  }

private:
  void fk_callback(const kalman_interfaces::msg::ArmFkCommand::SharedPtr msg)
  {
    auto now = rclcpp::Clock().now();
    if (now - fk_last_time_ > rclcpp::Duration::from_seconds(1.0 / rate_))
    {
      if (msg->gripper == 0.0)
      {
        fk_zeros_counter_++;
      }
      else
      {
        fk_zeros_counter_ = 0;
      }

      if (fk_zeros_counter_ > 5)
      {
        fk_zeros_counter_ = 5;
        fk_send_ = false;
      }
      else
      {
        fk_send_ = true;
      }

      fk_last_time_ = now;
      if (fk_send_)
      {
        send_gripper_msg(static_cast<int8_t>(msg->gripper));
      }
    }
  }

  void spacenav_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    auto now = rclcpp::Clock().now();
    if (now - spacenav_last_time_ > rclcpp::Duration::from_seconds(1.0 / rate_))
    {
      if (msg->buttons[0] == 0 && msg->buttons[1] == 0)
      {
        spacenav_zeros_counter_++;
      }
      else
      {
        spacenav_zeros_counter_ = 0;
      }

      if (spacenav_zeros_counter_ > 5)
      {
        spacenav_zeros_counter_ = 5;
        spacenav_send_ = false;
      }
      else
      {
        spacenav_send_ = true;
      }

      spacenav_last_time_ = now;
      if (spacenav_send_)
      {
        send_gripper_msg(msg->buttons[1] - msg->buttons[0]);
      }
    }
  }

  void send_gripper_msg(const int8_t increment)
  {
    auto msg = std_msgs::msg::Int8();
    msg.data = increment * gripper_scale_;

    gripper_pub_->publish(msg);
  }

  uint8_t convert_twist_data(double twist_data) const
  {
    return uint8_t((twist_data + 1.0) * 100.0 + 0.5);
  }

  bool all_zeros(const kalman_interfaces::msg::ArmFkCommand::SharedPtr msg)
  {
    return (msg->joint_1 == 0.0 && msg->joint_2 == 0.0 && msg->joint_3 == 0.0 && msg->joint_4 == 0.0 &&
            msg->joint_5 == 0.0 && msg->joint_6 == 0.0);
  }

  int fk_zeros_counter_ = 0;
  bool fk_send_ = true;
  int spacenav_zeros_counter_ = 0;
  bool spacenav_send_ = true;
  double rate_;
  double gripper_scale_;
  rclcpp::Time fk_last_time_;
  rclcpp::Time spacenav_last_time_;
  rclcpp::Subscription<kalman_interfaces::msg::ArmFkCommand>::SharedPtr fk_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr spacenav_sub_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr gripper_pub_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GripperRepublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
