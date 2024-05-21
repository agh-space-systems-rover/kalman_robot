#include "builtin_interfaces/msg/time.hpp"
#include "kalman_interfaces/msg/arm_compressed.hpp"
#include "kalman_interfaces/msg/arm_fk_command.hpp"
#include "kalman_interfaces/msg/master_message.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <cstdint>

class JointRepublisher : public rclcpp::Node {
public:
  JointRepublisher() : Node("joint_republisher") {
    this->declare_parameter<double>("rate", 10.0);
    this->get_parameter("rate", rate_);

    last_time_ = rclcpp::Clock().now();

    compressed_pub_ =
        this->create_publisher<kalman_interfaces::msg::ArmCompressed>(
            "/joy_compressed", 10);

    fk_sub_ = this->create_subscription<kalman_interfaces::msg::ArmFkCommand>(
        "/station/arm/fk/command", 10,
        [this](kalman_interfaces::msg::ArmFkCommand::SharedPtr msg) {
          sub_callback(msg);
        });
  }

private:
  void sub_callback(const kalman_interfaces::msg::ArmFkCommand::SharedPtr msg) {
    auto now = rclcpp::Clock().now();
    if (now - last_time_ > rclcpp::Duration::from_seconds(1.0 / rate_)) {
      if (all_zeros(msg)) {
        zeros_counter_++;
      } else {
        zeros_counter_ = 0;
      }

      if (zeros_counter_ > 5){
        zeros_counter_ = 5;
        send_ = false;
      }
      else {
        send_ = true;
      }

      last_time_ = now;
      republish_compressed_msg(msg);
    }
  }

  void republish_compressed_msg(
      const kalman_interfaces::msg::ArmFkCommand::SharedPtr msg) {
    if(send_) {
      auto compressed_msg = kalman_interfaces::msg::ArmCompressed();

      uint8_t mask = 0;

      uint8_t data[6] = {
          convert_twist_data(msg->joint_1), convert_twist_data(msg->joint_2),
          convert_twist_data(msg->joint_3), convert_twist_data(msg->joint_4),
          convert_twist_data(msg->joint_5), convert_twist_data(msg->joint_6)};

      for (int i = 0; i < 6; i++) {
        if (data[i] != 100) {
          mask |= 1 << i;
          compressed_msg.joints_data.push_back(data[i]);
        }
      }

      compressed_msg.joints_mask = mask;

      compressed_pub_->publish(compressed_msg);
    }
  }

  uint8_t convert_twist_data(double twist_data) const {
    return uint8_t((twist_data + 1.0) * 100.0 + 0.5);
  }

  bool all_zeros(const kalman_interfaces::msg::ArmFkCommand::SharedPtr msg) {
    return (
      msg->joint_1 == 0.0 && 
      msg->joint_2 == 0.0 && 
      msg->joint_3 == 0.0 &&
      msg->joint_4 == 0.0 && 
      msg->joint_5 == 0.0 && 
      msg->joint_6 == 0.0
    );
  }

  int zeros_counter_ = 0;
  bool send_ = true;
  double rate_;
  rclcpp::Time last_time_;
  rclcpp::Subscription<kalman_interfaces::msg::ArmFkCommand>::SharedPtr fk_sub_;
  rclcpp::Publisher<kalman_interfaces::msg::ArmCompressed>::SharedPtr
      compressed_pub_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JointRepublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
