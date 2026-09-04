#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>

class SetMissionFeedback : public BT::SyncActionNode {
  public:
	SetMissionFeedback(
	    const std::string &name,
	    const BT::NodeConfiguration &config,
	    rclcpp::Node *parent
	);

	static BT::PortsList providedPorts();

	BT::NodeStatus tick() override;

  private:
	rclcpp::Node *parent_;
};
