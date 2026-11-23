#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "can/socket_can_interface.hpp"
#include "gripper/gripper_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto can_iface = std::make_unique<kalman_arm::can::SocketCanInterface>();
  auto node      = std::make_shared<kalman_arm::gripper::GripperNode>(std::move(can_iface));
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}