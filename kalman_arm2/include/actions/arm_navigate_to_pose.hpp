#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/behavior_tree.h>

#include <future>
#include <kalman_interfaces/action/arm_goto_joint_pose.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <rclcpp_action/client.hpp>
#include <thread>
#include <iostream>

class ArmNavigateToPose : public BT::StatefulActionNode
{
public:
  ArmNavigateToPose(
	  const std::string           &name,
	  const BT::NodeConfiguration &config,
	  rclcpp::Node                *parent
  );

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void           onHalted() override;

private:
  rclcpp::Node::SharedPtr action_node_;
  rclcpp::Node *parent_;
  rclcpp_action::Client<kalman_interfaces::action::ArmGotoJointPose>::SharedPtr client_;
  std::optional<rclcpp_action::ResultCode> last_result_;
  std::mutex m_;
  std::atomic<bool> cancelled_{false};
  std::shared_future<typename rclcpp_action::ClientGoalHandle<kalman_interfaces::action::ArmGotoJointPose>::SharedPtr> goal_handle_future_;
  std::array<float, 6> target_joints_;
};
